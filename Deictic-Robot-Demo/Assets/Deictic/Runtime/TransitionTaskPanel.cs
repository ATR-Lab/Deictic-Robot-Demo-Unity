using System;
using System.Collections.Generic;
using System.Text;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.UI;

namespace Deictic
{
    /// <summary>World-fixed task controls. View selection and task attention are deliberately independent.</summary>
    public sealed class TransitionTaskPanel : MonoBehaviour
    {
        public TransitionOperatorClient Client { get; private set; }
        public RectTransform PanelRect { get; private set; }
        public Text FactsText { get; private set; }
        public IReadOnlyList<Button> Buttons => buttons;
        readonly List<Button> buttons = new List<Button>();
        readonly List<string> keys = new List<string>();
        readonly List<Image> backgrounds = new List<Image>();
        Camera eye;
        Text status, feedback, snapshotTitle;
        Material material;
        bool historyVisible;
        bool focused = true, paused;
        string renderedLease, renderedHash, pendingDisplayLease, pendingDisplayHash;
        DeicticControlMode mode;
        ROSConnection ros;
        JObject hardwareStatus;
        float hardwareStatusAt = float.NegativeInfinity, nextHardwareRender;

        public void Initialize(Transform head, DeicticSettings settings, DeicticBridge bridge = null)
        {
            mode = settings.controlMode;
            eye = head.GetComponent<Camera>();
            Client = gameObject.AddComponent<TransitionOperatorClient>();
            if (mode == DeicticControlMode.HardwareObservation)
            {
                Client.enabled = false;
                ros = bridge ? bridge.Ros : null;
                ros?.Subscribe<StringMsg>("/transition/hardware/status", OnHardwareStatus);
            }
            else Client.Initialize(settings.transitionOperatorUrl);
            material = new Material(Resources.Load<Shader>("Deictic/CameraViewOverlay"));
            material.SetFloat("_StereoSideBySide", 0);
            PanelRect = new GameObject("Transition task controls (world fixed)", typeof(RectTransform), typeof(Canvas)).GetComponent<RectTransform>();
            PanelRect.gameObject.layer = DeicticCameraView.RobotUiLayer;
            PanelRect.SetPositionAndRotation(settings.transitionPanelWorldPosition, Quaternion.identity);
            PanelRect.localScale = Vector3.one * .00085f;
            PanelRect.sizeDelta = new Vector2(1000, 1420);
            var canvas = PanelRect.GetComponent<Canvas>();
            canvas.renderMode = RenderMode.WorldSpace; canvas.worldCamera = eye; canvas.sortingOrder = 21;
            var background = PanelRect.gameObject.AddComponent<Image>();
            background.material = material; background.color = new Color(.025f, .04f, .065f, 1); background.raycastTarget = false;
            Label("Task title", new Vector2(0, 660), new Vector2(940, 60), "TRANSITION TASK · K1", 32);
            status = Label("Task state", new Vector2(0, 565), new Vector2(940, 130), "Connecting", 22);
            snapshotTitle = Label("Facts title", new Vector2(0, 470), new Vector2(940, 44), "Current facts", 25);
            // Keep measured joint and hand facts readable alongside the task facts.
            // Display acknowledgement still requires this entire region to fit and render.
            FactsText = Label("Task facts", new Vector2(0, 245), new Vector2(930, 390), "Waiting for authoritative task state", 20);
            FactsText.alignment = TextAnchor.UpperLeft;
            feedback = Label("Operator feedback", new Vector2(0, -10), new Vector2(930, 90), "No task authority granted by opening this view", 20);
            Add("grant", "Grant task · 10 min", () => Send("grant", new JObject { ["ttl"] = 600 }));
            Add("acknowledge", "Acknowledge facts", () => Send("acknowledge"));
            Add("revoke", "Revoke / request stop", () => Client.Revoke());
            Add("local_attention", "Attend local work", () => Send("attention", new JObject { ["domain"] = "local" }));
            Add("remote_attention", "Attend robot", () => Send("attention", new JObject { ["domain"] = "remote" }));
            Add("start", "Start sequencing", () => Send("run", new JObject { ["enabled"] = true }));
            Add("pause", "Pause new skills", () => Send("run", new JObject { ["enabled"] = false }));
            Add("stable", "Local state stable", () => Send("quiescent", new JObject { ["confirmed"] = true }));
            Add("changing", "Resume local changes", () => Send("quiescent", new JObject { ["confirmed"] = false }));
            Add("ready", "Commit local ready", () => Local("local.ready", true));
            Add("not_ready", "Clear local ready", () => Local("local.ready", false));
            Add("select_a", "Commit selection A", () => Local("local.selected", "A"));
            Add("select_b", "Commit selection B", () => Local("local.selected", "B"));
            Add("cue", "Cue return · 15 sec", () => Send("cue", new JObject { ["window"] = 15 }));
            Add("reconcile", "Inspect reconciliation", () => Send("reconcile"));
            Add("respond_a", "Decision: select A", () => Client.Choose("execute", "select_A"));
            Add("respond_b", "Decision: select B", () => Client.Choose("execute", "select_B"));
            Add("respond_defer", "Defer: sequence done", () => Client.Choose("defer", "sequence_complete"));
            Add("respond_evidence", "Request pointing evidence", () => Client.Choose("request_evidence", "pointing_state"));
            Add("retry", "Retry same decision", () => Client.RetryDecision());
            Add("history", "History / task facts", ToggleHistory);
            Label("Task instructions", new Vector2(0, -645), new Vector2(950, 78),
                "Commit local facts only after doing that work. Decisions record a choice; they do not move the robot.\nView switching does not change attention. Task stop is separate from the robot's emergency stop.", 18);
            Client.Changed += Render;
            RenderPipelineManager.endCameraRendering += AfterCamera;
            Camera.onPostRender += AfterBuiltInCamera;
            Render();
        }

        public Button ButtonFor(string key)
        {
            int index = keys.IndexOf(key); return index < 0 ? null : buttons[index];
        }

        RectTransform Rect(string name, Vector2 position, Vector2 size)
        {
            var rect = new GameObject(name, typeof(RectTransform)).GetComponent<RectTransform>();
            rect.SetParent(PanelRect, false); rect.gameObject.layer = PanelRect.gameObject.layer;
            rect.anchorMin = rect.anchorMax = rect.pivot = new Vector2(.5f, .5f);
            rect.anchoredPosition = position; rect.sizeDelta = size; return rect;
        }

        Text Label(string name, Vector2 position, Vector2 size, string text, int fontSize)
        {
            var label = Rect(name, position, size).gameObject.AddComponent<Text>();
            label.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            label.fontSize = fontSize; label.material = material; label.color = Color.white;
            label.alignment = TextAnchor.MiddleCenter; label.text = text; label.supportRichText = false;
            label.raycastTarget = false; return label;
        }

        void Add(string key, string title, Action action)
        {
            int index = buttons.Count;
            Vector2 position = new Vector2((index % 3 - 1) * 325, -92 - index / 3 * 70);
            var rect = Rect(key, position, new Vector2(306, 58));
            var image = rect.gameObject.AddComponent<Image>(); image.material = material;
            image.color = key == "revoke" ? new Color(.44f, .09f, .12f) : new Color(.055f, .2f, .28f);
            image.raycastTarget = false;
            var button = rect.gameObject.AddComponent<Button>(); button.targetGraphic = image;
            button.navigation = new Navigation { mode = Navigation.Mode.None };
            button.onClick.AddListener(() => { if (button.interactable) action(); });
            var label = Label(key + " label", position, new Vector2(290, 53), title, 19);
            label.rectTransform.SetParent(rect, false);
            label.rectTransform.anchoredPosition = Vector2.zero;
            buttons.Add(button); keys.Add(key); backgrounds.Add(image);
        }

        void Local(string key, JToken value) => Send("local", new JObject { ["key"] = key, ["value"] = value });
        void Send(string operation, JObject data = null) => Client.Send(operation, data);

        void ToggleHistory()
        {
            historyVisible = !historyVisible;
            if (historyVisible) Client.FetchHistory();
            Render();
        }

        public bool PointAtControl(Ray ray, out int index, out Vector3 hit)
        {
            index = -1; hit = default;
            if (!isActiveAndEnabled || !PanelRect || !PanelRect.gameObject.activeInHierarchy) return false;
            // The panel surface consumes hits even on labels/disabled buttons, so it cannot select a target behind it.
            var plane = new Plane(PanelRect.forward, PanelRect.position);
            if (!plane.Raycast(ray, out float distance) || distance < 0 || distance > 4) return false;
            hit = ray.GetPoint(distance);
            Vector3 local = PanelRect.InverseTransformPoint(hit);
            if (!PanelRect.rect.Contains(new Vector2(local.x, local.y))) return false;
            for (int i = 0; i < buttons.Count; i++)
            {
                var rect = (RectTransform)buttons[i].transform;
                Vector3 point = rect.InverseTransformPoint(hit);
                if (buttons[i].interactable && rect.rect.Contains(new Vector2(point.x, point.y))) { index = i; break; }
            }
            return true;
        }

        public void Hover(int index)
        {
            for (int i = 0; i < buttons.Count; i++)
            {
                var color = keys[i] == "revoke" ? new Color(.44f, .09f, .12f) : new Color(.055f, .2f, .28f);
                if (i == index && buttons[i].interactable) color = new Color(.1f, .48f, .58f);
                if (!buttons[i].interactable) color *= .55f;
                color.a = 1; backgrounds[i].color = color;
            }
        }

        public void Click(int index)
        {
            if (index >= 0 && index < buttons.Count && buttons[index].interactable) buttons[index].onClick.Invoke();
        }

        void Update()
        {
            if (mode == DeicticControlMode.HardwareObservation)
            {
                if (Time.realtimeSinceStartup >= nextHardwareRender)
                { RenderHardware(); nextHardwareRender = Time.realtimeSinceStartup + .25f; }
                return;
            }
            RefreshButtons();
            if (pendingDisplayLease != null && Client.Fresh && !Client.Busy && !historyVisible)
            {
                if (Client.Presentation.NeedsDisplayAcknowledgement && pendingDisplayLease == Client.Presentation.LeaseId &&
                    pendingDisplayHash == Client.Presentation.SnapshotHash)
                    Client.Send("displayed", Client.Presentation.DisplayAcknowledgement());
                pendingDisplayLease = pendingDisplayHash = null;
            }
        }

        void Render()
        {
            if (!status) return;
            if (mode == DeicticControlMode.HardwareObservation) { RenderHardware(); return; }
            var p = Client.Presentation; var state = p.State;
            feedback.text = Client.Feedback;
            renderedLease = renderedHash = null;
            if (state == null)
            {
                status.text = ModeLabel() + "\nWaiting for runtime on loopback port 8766\nArm and head teleoperation disabled";
                FactsText.text = "Start the transition runtime and SSH tunnel.\nNo task authority is granted by connecting.";
                RefreshButtons(); return;
            }
            string authority = state["authority"] is JObject grant ?
                ((bool?)grant["revoked"] == true ? "revoked" : "granted; expires in " +
                Math.Max(0, ((double?)grant["expires_at"] ?? 0) - ((double?)state["server_time"] ?? 0)).ToString("F0") + " s") : "not granted";
            status.text = ModeLabel() + " · " + (string)state["backend"] + " · " + (string)state["policy"] +
                "\nAttention: " + (string)state["attention"] + " · Authority: " + authority +
                "\n" + ((bool?)state["auto_dispatch"] == true ? "Sequencing" : "New skills paused") +
                " · Local stable: " + ((bool?)state["local_quiescent"] == true ? "yes" : "no") +
                ((string)state["fault"] is string fault ? "\nFAULT: " + fault : "");
            var lease = state["lease"] as JObject;
            if (historyVisible)
            {
                snapshotTitle.text = "Event history / reconciliation (latest excerpt)";
                string history = Client.History ?? "Loading event history…";
                FactsText.text = history.Length > 750 ? "…" + history.Substring(history.Length - 750) : history;
            }
            else
            {
                var facts = p.HasSnapshot ? p.Snapshot : (JObject)state["facts"];
                snapshotTitle.text = p.HasSnapshot ? "Frozen return snapshot · " + (string)lease?["status"] : "Current task facts";
                var text = new StringBuilder();
                foreach (var fact in facts.Properties())
                    text.Append(fact.Name).Append(": ").Append(fact.Value["value"]?.ToString(Formatting.None) ?? "null")
                        .Append("  [").Append((string)fact.Value["status"]).Append(" · v").Append(fact.Value["version"]).AppendLine("]");
                if (p.HasSnapshot)
                {
                    text.Append("Lease ").Append(p.LeaseId.Substring(0, Math.Min(8, p.LeaseId.Length)))
                        .Append(" · remaining ").Append(Math.Max(0, ((double?)lease?["deadline"] ?? 0) -
                            ((double?)state["server_time"] ?? 0)).ToString("F1")).AppendLine(" s");
                    text.Append(!p.Exposed ? "Return window closed." : p.NeedsDisplayAcknowledgement ?
                        "Look at these facts to acknowledge display." : "Display acknowledged; choose your first decision.");
                    renderedLease = p.LeaseId; renderedHash = p.SnapshotHash;
                    if ((string)state["display"] == "summary" && lease?["decision_summary"] is JArray frozenSummary && frozenSummary.Count > 0)
                        text.Append("\nChanged since acknowledgement: ").Append(string.Join(", ", SummaryKeys(frozenSummary)));
                }
                else if ((string)state["display"] == "summary" && state["decision_summary"] is JArray summary && summary.Count > 0)
                    text.Append("Changed since acknowledgement: ").Append(string.Join(", ", SummaryKeys(summary)));
                FactsText.text = text.ToString();
            }
            RefreshButtons(); Canvas.ForceUpdateCanvases();
        }

        static IEnumerable<string> SummaryKeys(JArray summary)
        {
            foreach (var item in summary) yield return (string)item["fact"];
        }

        string ModeLabel() => mode == DeicticControlMode.HardwareObservation ? "HARDWARE · OBSERVATION ONLY" : "TRANSITION SIMULATION";

        void RefreshButtons()
        {
            if (!Client) return;
            if (mode == DeicticControlMode.HardwareObservation)
            {
                foreach (var button in buttons) { button.interactable = false; button.gameObject.SetActive(false); }
                return;
            }
            bool ready = Client.Fresh && !Client.Busy;
            for (int i = 0; i < buttons.Count; i++)
            {
                bool enabled = ready;
                if (keys[i].StartsWith("respond_", StringComparison.Ordinal)) enabled &= Client.Presentation.CanRespond && !historyVisible;
                if (keys[i] == "retry") enabled &= Client.Presentation.HasPendingDecision;
                if (keys[i] == "revoke") enabled = Client.Fresh; // Client may preempt a poll.
                buttons[i].interactable = enabled;
            }
            if (!Client.Fresh && feedback) feedback.text = Client.Feedback + "\nState unavailable or stale; controls disabled.";
        }

        void AfterCamera(ScriptableRenderContext context, Camera camera) => AfterBuiltInCamera(camera);
        void AfterBuiltInCamera(Camera camera)
        {
            bool operatorFocus = OVRManager.instance != null && OVRManager.isHmdPresent ? OVRManager.hasInputFocus : focused;
            if (camera != eye || !operatorFocus || paused || historyVisible || !Client || !Client.Fresh ||
                !Client.Presentation.NeedsDisplayAcknowledgement || renderedLease != Client.Presentation.LeaseId ||
                renderedHash != Client.Presentation.SnapshotHash || !SnapshotVisible(camera)) return;
            // This callback follows actual rendering by the user's eye camera. A state poll,
            // scene-view repaint, camera toggle, or elapsed timer cannot acknowledge display.
            pendingDisplayLease = renderedLease; pendingDisplayHash = renderedHash;
        }

        public bool SnapshotVisible(Camera camera)
        {
            if (!isActiveAndEnabled || !PanelRect || !PanelRect.gameObject.activeInHierarchy || !FactsText ||
                !camera || !camera.isActiveAndEnabled || (camera.cullingMask & (1 << PanelRect.gameObject.layer)) == 0) return false;
            if (FactsText.preferredHeight > FactsText.rectTransform.rect.height + 1) return false;
            var corners = new Vector3[4]; FactsText.rectTransform.GetWorldCorners(corners);
            foreach (var corner in corners)
            {
                var point = camera.WorldToViewportPoint(corner);
                if (point.z <= camera.nearClipPlane || point.z >= camera.farClipPlane || point.x < 0 || point.x > 1 || point.y < 0 || point.y > 1) return false;
            }
            return true;
        }

        void OnHardwareStatus(StringMsg message)
        {
            try
            {
                if (message.data == null || message.data.Length > 65536) return;
                var received = JObject.Parse(message.data);
                if ((int?)received["schema_version"] != 1 || (string)received["mode"] != "hardware_observation") return;
                hardwareStatus = received; hardwareStatusAt = Time.realtimeSinceStartup;
            }
            catch (Exception) { /* Invalid diagnostic packets cannot grant capability. */ }
        }

        void RenderHardware()
        {
            if (!status) return;
            RefreshButtons();
            bool recentReceipt = Time.realtimeSinceStartup - hardwareStatusAt < 3;
            status.text = "HARDWARE · OBSERVATION ONLY\n" + (recentReceipt ? "Telemetry bridge connected" : "Waiting for telemetry bridge") +
                "\nNo commissioned task authority · all motion controls disabled";
            snapshotTitle.text = "K1 telemetry · receipt timing is not acquisition freshness";
            var text = new StringBuilder();
            if (hardwareStatus != null)
            {
                text.Append("Samples: ").Append(hardwareStatus["sample_counts"]?.ToString(Formatting.None)).AppendLine();
                text.Append("Receipt age (s): ").Append(hardwareStatus["receipt_age_s"]?.ToString(Formatting.None)).AppendLine();
                text.Append("Validation errors: ").Append(hardwareStatus["validation_errors"]?.ToString(Formatting.None)).AppendLine();
                if (hardwareStatus["blockers"] is JArray blockers)
                    foreach (var blocker in blockers) text.Append("• ").Append((string)blocker).AppendLine();
            }
            else text.Append("Start the hardware observation launcher and ROS tunnel.\nCamera and measured joints are receive-only.\nPhysical release remains blocked until commissioning evidence is verified.");
            FactsText.text = text.ToString();
            FactsText.fontSize = 19;
            FactsText.rectTransform.anchoredPosition = new Vector2(0, -45);
            FactsText.rectTransform.sizeDelta = new Vector2(930, 700);
            feedback.rectTransform.anchoredPosition = new Vector2(0, -445);
            feedback.text = "No task mutations, arm clutch, head tracking, or raw commands are available in this mode.\nUse the independent robot stop for an emergency; this observer has no stop capability.";
            renderedLease = renderedHash = pendingDisplayLease = pendingDisplayHash = null;
        }

        public void RequestStop()
        {
            if (mode != DeicticControlMode.HardwareObservation) Client?.Revoke();
        }
        void OnApplicationFocus(bool value) { focused = value; if (!value) pendingDisplayLease = pendingDisplayHash = null; }
        void OnApplicationPause(bool value) { paused = value; if (value) pendingDisplayLease = pendingDisplayHash = null; }
        void OnDisable() { if (PanelRect) PanelRect.gameObject.SetActive(false); pendingDisplayLease = pendingDisplayHash = null; }
        void OnEnable() { if (PanelRect) PanelRect.gameObject.SetActive(true); }
        void OnDestroy()
        {
            RenderPipelineManager.endCameraRendering -= AfterCamera;
            Camera.onPostRender -= AfterBuiltInCamera;
            if (Client) { Client.Changed -= Render; Destroy(Client); }
            if (ros != null) ros.Unsubscribe("/transition/hardware/status");
            if (PanelRect) Destroy(PanelRect.gameObject);
            if (material) Destroy(material);
        }
    }
}
