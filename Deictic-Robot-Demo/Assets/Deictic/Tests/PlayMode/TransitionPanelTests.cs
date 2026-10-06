using System;
using System.Collections;
using System.Collections.Concurrent;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using Deictic;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

public sealed class TransitionPanelTests
{
    GameObject root, head;
    DeicticSettings settings;
    TransitionTaskPanel panel;
    OperatorFixture server;
    Camera camera;
    RenderTexture target;
    NativeSimulatorFixture simulator;
    string workflowRunId, workflowAuthorityId;

    [UnitySetUp]
    public IEnumerator StartSimulator()
    {
        workflowRunId = workflowAuthorityId = null;
        simulator = new NativeSimulatorFixture();
        yield return simulator.Initialize();
    }

    void Create(DeicticControlMode mode, string operatorUrl = null)
    {
        server = new OperatorFixture();
        settings = ScriptableObject.CreateInstance<DeicticSettings>();
        settings.controlMode = mode; settings.connectOnStart = false;
        settings.transitionOperatorUrl = operatorUrl ?? server.Url;
        settings.transitionPanelWorldPosition = new Vector3(0, 1.4f, 2);
        root = new GameObject("Transition operator test");
        head = new GameObject("Operator eye");
        head.transform.position = new Vector3(0, 1.4f, 0);
        camera = head.AddComponent<Camera>(); camera.fieldOfView = 80;
        camera.stereoTargetEye = StereoTargetEyeMask.None;
        target = new RenderTexture(1280, 1200, 24); target.Create(); camera.targetTexture = target;
        camera.aspect = 1280f / 1200;
        var bridge = root.AddComponent<DeicticBridge>(); bridge.Initialize(settings);
        panel = root.AddComponent<TransitionTaskPanel>(); panel.Initialize(head.transform, settings, bridge);
        // The hidden batch editor has no desktop focus. Explicitly simulate operator
        // focus while still requiring this eye camera's actual render callback.
        panel.SendMessage("OnApplicationFocus", true);
    }

    [UnityTest]
    public IEnumerator FrozenSnapshotNeedsVisibleEyeFrameAndAcceptedAckBeforeDecision()
    {
        Create(DeicticControlMode.TransitionSimulation);
        float until = Time.realtimeSinceStartup + 12;
        while (!panel.Client.Fresh && Time.realtimeSinceStartup < until) yield return null;
        Assert.That(panel.Client.Fresh, Is.True, panel.Client.Feedback);
        Assert.That(server.Operations.Count, Is.Zero, "Connecting grants no authority and acknowledges no context");
        Vector3 world = panel.PanelRect.position;
        root.transform.position = new Vector3(3, 2, 1);
        Assert.That(panel.PanelRect.position, Is.EqualTo(world));
        Assert.That(panel.PanelRect.IsChildOf(head.transform), Is.False);
        var grant = panel.ButtonFor("grant");
        while (!grant.interactable && Time.realtimeSinceStartup < until) yield return null;
        Assert.That(panel.PointAtControl(new Ray(head.transform.position, grant.transform.position - head.transform.position), out int index, out _), Is.True);
        Assert.That(panel.Buttons[index], Is.SameAs(grant));
        // A view toggle is independent of task attention, acknowledgement and authority.
        var view = root.AddComponent<DeicticCameraView>(); view.Initialize(head.transform, root.GetComponent<DeicticBridge>());
        view.Toggle(); view.Toggle();
        Assert.That(server.Operations.Count, Is.Zero);

        panel.enabled = false;
        server.Exposed = true;
        until = Time.realtimeSinceStartup + 8;
        while (!panel.Client.Presentation.HasSnapshot && Time.realtimeSinceStartup < until) yield return null;
        Assert.That(panel.Client.Presentation.HasSnapshot, Is.True, panel.Client.Feedback);
        yield return new WaitForSeconds(.5f);
        Assert.That(server.Operations.Count, Is.Zero, "Hidden UI cannot acknowledge display");
        Assert.That(panel.Client.Presentation.CanRespond, Is.False);
        Assert.That(panel.FactsText.text, Does.Contain("\"A\""));
        Assert.That(panel.FactsText.text, Does.Not.Contain("\"B\""), "Live B must not replace captured A");
        panel.enabled = true;
        head.transform.rotation = Quaternion.Euler(0, 180, 0);
        camera.Render();
        yield return new WaitForSeconds(.5f);
        Assert.That(server.Operations.Count, Is.Zero, "Looking away cannot acknowledge display");
        head.transform.rotation = Quaternion.identity;
        Assert.That(panel.Client.Presentation.Snapshot.Count, Is.EqualTo(5), "Exercise the measured K1 fact payload, not only one short fact");
        Assert.That(panel.FactsText.preferredHeight, Is.GreaterThan(205), "The regression payload must exercise the old clipped layout");
        Assert.That(panel.FactsText.text, Does.Contain("robot.joints").And.Contain("robot.hand_reference").And.Contain("3456"));
        Assert.That(panel.FactsText.text, Does.Contain("Changed since acknowledgement:"));
        var fullFactsSize = panel.FactsText.rectTransform.sizeDelta;
        panel.FactsText.rectTransform.sizeDelta = new Vector2(fullFactsSize.x, 10);
        Canvas.ForceUpdateCanvases(); camera.Render();
        yield return null;
        Assert.That(server.Operations.Count, Is.Zero, "Clipped facts cannot acknowledge display");
        Assert.That(panel.SnapshotVisible(camera), Is.False);
        panel.FactsText.rectTransform.sizeDelta = fullFactsSize;
        Canvas.ForceUpdateCanvases();
        Assert.That(panel.SnapshotVisible(camera), Is.True);
        until = Time.realtimeSinceStartup + 8;
        while (!panel.Client.Presentation.CanRespond && Time.realtimeSinceStartup < until) { camera.Render(); yield return null; }
        Assert.That(panel.Client.Presentation.CanRespond, Is.True, panel.Client.Feedback);
        Assert.That(server.Operations.TryPeek(out JObject displayed), Is.True);
        Assert.That((string)displayed["operation"], Is.EqualTo("displayed"));
        Assert.That((string)displayed["data"]["lease_id"], Is.EqualTo("lease-one"));
        Assert.That((string)displayed["data"]["snapshot_hash"], Is.EqualTo(OperatorFixture.Hash));
        CapturePanel(Environment.GetEnvironmentVariable("DEICTIC_PANEL_CAPTURE"));
        until = Time.realtimeSinceStartup + 8;
        while (panel.Client.Busy && Time.realtimeSinceStartup < until) yield return null;
        Assert.That(panel.Client.Choose("execute", "select_A"), Is.True);
        until = Time.realtimeSinceStartup + 8;
        while (panel.Client.Presentation.HasPendingDecision && Time.realtimeSinceStartup < until) yield return null;
        Assert.That(panel.Client.Presentation.HasPendingDecision, Is.False, panel.Client.Feedback);
        Assert.That(server.DecisionCount, Is.EqualTo(1));
    }

    [UnityTest, Category("TransitionRuntimeIntegration")]
    public IEnumerator RealRuntimeAcceptsExplicitTaskWorkflowAndScoresTheRenderedSnapshot()
    {
        string url = Environment.GetEnvironmentVariable("DEICTIC_TRANSITION_TEST_URL");
        if (string.IsNullOrEmpty(url)) Assert.Ignore("Requires a disposable logical or Isaac K1 transition runtime.");
        string expectedBackend = Environment.GetEnvironmentVariable("DEICTIC_TRANSITION_TEST_BACKEND") ?? "logical_simulation";
        Assert.That(expectedBackend, Is.EqualTo("logical_simulation").Or.EqualTo("isaac_sim_k1"),
            "Only the explicitly selected simulation backend can be exercised");
        Create(DeicticControlMode.TransitionSimulation, url);
        yield return WaitReady();
        var initial = panel.Client.Presentation.State;
        Assert.That((string)initial["backend"], Is.EqualTo(expectedBackend), "This test must never target a physical backend");
        Assert.That(initial["authority"]?.Type, Is.EqualTo(JTokenType.Null), "Use a fresh disposable run, not an operator's active session");
        Assert.That(initial["active"]?.Type, Is.EqualTo(JTokenType.Null));
        Assert.That(initial["lease"]?.Type, Is.EqualTo(JTokenType.Null));
        Assert.That(initial["fault"]?.Type, Is.EqualTo(JTokenType.Null));
        Assert.That((bool?)initial["auto_dispatch"], Is.False);
        Assert.That((JArray)initial["completed"], Is.Empty);
        Assert.That((bool?)initial["facts"]?["local.ready"]?["value"], Is.False,
            "The disposable runtime must use examples/k1_pointing.json");
        Assert.That((string)initial["facts"]?["local.selected"]?["value"], Is.EqualTo("none"));
        workflowRunId = (string)initial["run_id"];
        yield return Operate("grant", new JObject { ["ttl"] = 300 });
        float grantUntil = Time.realtimeSinceStartup + 8;
        while (panel.Client.Presentation.State?["authority"]?.Type != JTokenType.Object && Time.realtimeSinceStartup < grantUntil)
        { camera.Render(); yield return null; }
        if (panel.Client.Presentation.RunId == workflowRunId)
            workflowAuthorityId = (string)(panel.Client.Presentation.State?["authority"] as JObject)?["id"];
        Assert.That(workflowAuthorityId, Is.Not.Null.And.Not.Empty, "The test's grant must be observed before sequencing");
        yield return Operate("acknowledge");
        yield return Operate("local", new JObject { ["key"] = "local.ready", ["value"] = true });
        yield return CompleteAndDecide("point_a", "A", "respond_a", "none");
        yield return Operate("quiescent", new JObject { ["confirmed"] = false });
        yield return Operate("local", new JObject { ["key"] = "local.selected", ["value"] = "A" });
        yield return CompleteAndDecide("point_b", "B", "respond_b", "A");
        yield return Operate("quiescent", new JObject { ["confirmed"] = false });
        yield return Operate("local", new JObject { ["key"] = "local.selected", ["value"] = "B" });
        yield return CompleteAndDecide("home", null, "respond_defer", "B");
        Assert.That((JArray)panel.Client.Presentation.State["completed"], Has.Count.EqualTo(3));
        yield return Operate("revoke");
        workflowRunId = workflowAuthorityId = null;
    }

    IEnumerator CompleteAndDecide(string skill, string expectedTarget, string decisionKey, string previousSelection)
    {
        yield return Operate("attention", new JObject { ["domain"] = "local" });
        yield return Operate("run", new JObject { ["enabled"] = true });
        // Each named K1 skill has a 20-second task budget; allow polling and rendering overhead.
        float until = Time.realtimeSinceStartup + 30;
        while (!HasCompleted(skill) && Time.realtimeSinceStartup < until)
        {
            Assert.That(panel.Client.Presentation.State?["fault"]?.Type, Is.EqualTo(JTokenType.Null), panel.Client.Feedback);
            camera.Render(); yield return null;
        }
        Assert.That(HasCompleted(skill), Is.True, "No measured completion for " + skill + ": " + panel.Client.Feedback);
        Assert.That((string)panel.Client.Presentation.State["facts"]["remote.pointed_target"]["value"], Is.EqualTo(expectedTarget));
        yield return Operate("run", new JObject { ["enabled"] = false });
        yield return Operate("quiescent", new JObject { ["confirmed"] = true });
        string previousLease = panel.Client.Presentation.LeaseId;
        yield return Operate("cue", new JObject { ["window"] = 30 });
        until = Time.realtimeSinceStartup + 10;
        while (!panel.Client.Presentation.CanRespond && Time.realtimeSinceStartup < until) { camera.Render(); yield return null; }
        if (!panel.Client.Presentation.CanRespond) CaptureStateEvidence(skill + "-blocked");
        Assert.That(panel.Client.Presentation.CanRespond, Is.True, panel.Client.Feedback +
            "; facts preferred height=" + panel.FactsText.preferredHeight + ", rect height=" + panel.FactsText.rectTransform.rect.height +
            ", visible=" + panel.SnapshotVisible(camera) + ", lease=" + (string)panel.Client.Presentation.State?["lease"]?["status"]);
        Assert.That(panel.Client.Presentation.LeaseId, Is.Not.EqualTo(previousLease), "Each return must bind a new frozen snapshot");
        Assert.That((string)panel.Client.Presentation.Snapshot["remote.pointed_target"]["value"], Is.EqualTo(expectedTarget));
        CaptureStateEvidence(skill + "-snapshot");
        yield return WaitReady();
        var decision = panel.ButtonFor(decisionKey);
        Assert.That(panel.PointAtControl(new Ray(head.transform.position, decision.transform.position - head.transform.position), out int index, out _), Is.True);
        Assert.That(panel.Buttons[index], Is.SameAs(decision));
        panel.Click(index);
        until = Time.realtimeSinceStartup + 10;
        while (panel.Client.Presentation.HasPendingDecision && Time.realtimeSinceStartup < until) { camera.Render(); yield return null; }
        Assert.That(panel.Client.Presentation.HasPendingDecision, Is.False, panel.Client.Feedback);
        Assert.That(panel.Client.Feedback, Is.EqualTo("First decision recorded: correct"));
        until = Time.realtimeSinceStartup + 10;
        while ((string)panel.Client.Presentation.State?["lease"]?["status"] != "closed" && Time.realtimeSinceStartup < until)
        { camera.Render(); yield return null; }
        Assert.That((string)panel.Client.Presentation.State["lease"]["status"], Is.EqualTo("closed"));
        Assert.That((string)panel.Client.Presentation.State["facts"]["local.selected"]["value"], Is.EqualTo(previousSelection),
            "A decision records a choice; the separate local commit changes the selection");
    }

    bool HasCompleted(string skill)
    {
        var completed = panel.Client.Presentation.State?["completed"] as JArray;
        if (completed == null) return false;
        foreach (var value in completed) if ((string)value == skill) return true;
        return false;
    }

    void CapturePanel(string path)
    {
        if (string.IsNullOrEmpty(path)) return;
        camera.Render();
        var previous = RenderTexture.active;
        var pixels = new Texture2D(target.width, target.height, TextureFormat.RGB24, false);
        try
        {
            RenderTexture.active = target;
            pixels.ReadPixels(new Rect(0, 0, target.width, target.height), 0, 0); pixels.Apply();
            File.WriteAllBytes(path, pixels.EncodeToPNG());
        }
        finally { RenderTexture.active = previous; UnityEngine.Object.Destroy(pixels); }
    }

    void CaptureStateEvidence(string name)
    {
        string directory = Environment.GetEnvironmentVariable("DEICTIC_TRANSITION_TEST_EVIDENCE_DIR");
        if (string.IsNullOrEmpty(directory)) return;
        Directory.CreateDirectory(directory);
        CapturePanel(Path.Combine(directory, name + ".png"));
        File.WriteAllText(Path.Combine(directory, name + ".json"), new JObject {
            ["scope"] = "Scripted mono panel render and simulation HTTP integration; not controller or per-eye validation",
            ["run_id"] = panel.Client.Presentation.RunId, ["backend"] = (string)panel.Client.Presentation.State?["backend"],
            ["lease_id"] = panel.Client.Presentation.LeaseId, ["snapshot_hash"] = panel.Client.Presentation.SnapshotHash,
            ["snapshot"] = panel.Client.Presentation.Snapshot, ["state"] = panel.Client.Presentation.State,
            ["preferred_height"] = panel.FactsText.preferredHeight, ["rect_height"] = panel.FactsText.rectTransform.rect.height,
            ["facts_text"] = panel.FactsText.text, ["snapshot_visible"] = panel.SnapshotVisible(camera),
            ["client_fresh"] = panel.Client.Fresh, ["client_feedback"] = panel.Client.Feedback
        }.ToString(Formatting.Indented));
    }

    IEnumerator WaitReady()
    {
        float until = Time.realtimeSinceStartup + 12;
        while ((!panel.Client.Fresh || panel.Client.Busy) && Time.realtimeSinceStartup < until) { camera.Render(); yield return null; }
        Assert.That(panel.Client.Fresh && !panel.Client.Busy, Is.True, panel.Client.Feedback);
    }

    IEnumerator Operate(string operation, JObject data = null)
    {
        yield return WaitReady();
        Assert.That(panel.Client.Send(operation, data), Is.True);
        float until = Time.realtimeSinceStartup + 10;
        while (panel.Client.Busy && Time.realtimeSinceStartup < until) { camera.Render(); yield return null; }
        Assert.That(panel.Client.Feedback, Is.EqualTo("Recorded: " + operation));
    }

    [UnityTest]
    public IEnumerator HardwareObserverHasNoHttpSessionAndNoMutationControls()
    {
        Create(DeicticControlMode.HardwareObservation);
        yield return new WaitForSeconds(.7f);
        Assert.That(panel.Client.enabled, Is.False);
        Assert.That(server.RequestCount, Is.Zero);
        foreach (var button in panel.Buttons)
        {
            Assert.That(button.interactable, Is.False);
            Assert.That(button.gameObject.activeSelf, Is.False);
            button.onClick.Invoke();
        }
        panel.RequestStop();
        yield return null;
        Assert.That(server.RequestCount, Is.Zero);
        Assert.That(root.GetComponent<DeicticBridge>().DirectCommandsAllowed, Is.False);
    }

    [UnityTearDown]
    public IEnumerator Cleanup()
    {
        if (workflowRunId != null && workflowAuthorityId != null && panel && panel.Client)
        {
            // A failed assertion must not leave this disposable run sequencing. Keep
            // cleanup bounded and refuse a restarted session or replacement authority.
            yield return new WaitForSecondsRealtime(.6f);
            float until = Time.realtimeSinceStartup + 8;
            while ((!panel.Client.Fresh || panel.Client.Busy) && Time.realtimeSinceStartup < until) yield return null;
            var state = panel.Client.Presentation.State;
            if (panel.Client.Fresh && !panel.Client.Busy && panel.Client.Presentation.RunId == workflowRunId &&
                (string)(state?["authority"] as JObject)?["id"] == workflowAuthorityId &&
                ((string)state?["backend"] == "logical_simulation" || (string)state?["backend"] == "isaac_sim_k1"))
            {
                panel.Client.Revoke();
                until = Time.realtimeSinceStartup + 6;
                while (panel.Client.Busy && Time.realtimeSinceStartup < until) yield return null;
                Debug.Log("Disposable transition cleanup: " + panel.Client.Feedback + ". This is not measured stillness evidence.");
            }
            else Debug.LogWarning("Disposable transition cleanup could not verify the owned run and authority; stop its owned server process.");
        }
        workflowRunId = workflowAuthorityId = null;
        if (root) UnityEngine.Object.Destroy(root);
        if (head) UnityEngine.Object.Destroy(head);
        if (settings) UnityEngine.Object.Destroy(settings);
        if (target) { target.Release(); UnityEngine.Object.Destroy(target); }
        yield return null;
        server?.Dispose(); server = null;
        if (simulator != null) yield return simulator.Cleanup();
    }

    sealed class OperatorFixture : IDisposable
    {
        public static readonly string Hash = new string('a', 64);
        readonly HttpListener listener = new HttpListener();
        readonly Thread thread;
        public readonly ConcurrentQueue<JObject> Operations = new ConcurrentQueue<JObject>();
        public volatile bool Exposed;
        public int RequestCount, DecisionCount;
        bool closed;
        public string Url { get; }
        public OperatorFixture()
        {
            var reserve = new TcpListener(IPAddress.Loopback, 0); reserve.Start();
            int port = ((IPEndPoint)reserve.LocalEndpoint).Port; reserve.Stop();
            Url = "http://127.0.0.1:" + port;
            listener.Prefixes.Add(Url + "/"); listener.Start();
            thread = new Thread(Serve) { IsBackground = true }; thread.Start();
        }
        void Serve()
        {
            try
            {
                while (listener.IsListening)
                {
                    var context = listener.GetContext(); Interlocked.Increment(ref RequestCount);
                    JObject response;
                    switch (context.Request.Url.AbsolutePath)
                    {
                        case "/api/session": response = JObject.Parse("{'schema_version':1,'session_token':'test-only','run_id':'run-one','backend':'logical_simulation'}"); break;
                        case "/api/time":
                            double now = (DateTime.UtcNow - DateTime.UnixEpoch).TotalSeconds;
                            response = new JObject { ["schema_version"] = 1, ["run_id"] = "run-one", ["scope"] = "display_only",
                                ["clock_domain"] = "server_system_utc", ["server_receive_utc_seconds"] = now,
                                ["server_send_utc_seconds"] = now, ["server_processing_seconds"] = 0 };
                            break;
                        case "/api/state":
                            response = JObject.Parse(@"{'schema_version':1,'run_id':'run-one','backend':'logical_simulation','policy':'A1','display':'summary',
                                'attention':'remote','authority':null,'fault':null,'server_time':10,'auto_dispatch':false,'local_quiescent':true,
                                'facts':{'remote.pointed_target':{'value':'B','status':'known','version':2}},'decision_summary':[],'lease':null}");
                            if (Exposed) response["lease"] = JObject.Parse("{'id':'lease-one','status':'" + (closed ? "closed" : "exposed") +
                                "','deadline':100,'snapshot_hash':'" + Hash + "','snapshot':{'remote.pointed_target':{'value':'A','status':'known','version':1}}}");
                            if (Exposed)
                            {
                                var snapshot = (JObject)response["lease"]["snapshot"];
                                snapshot["local.ready"] = JObject.Parse("{'value':true,'status':'known','version':1}");
                                snapshot["local.selected"] = JObject.Parse("{'value':'none','status':'known','version':0}");
                                snapshot["robot.joints"] = JObject.Parse(@"{'value':{'aaright_shoulder_pitch_joint':-0.7631456260681152,
                                    'right_shoulder_roll_joint':-0.1931027328968048,'right_elbow_pitch_joint':-1.481092095375061,
                                    'right_elbow_yaw_joint':0.5420987010002136},'status':'known','version':3456}");
                                snapshot["robot.hand_reference"] = JObject.Parse(@"{'value':[0.2791843712329864,-0.3264812529087067,0.2384813725948334],
                                    'status':'known','version':3456}");
                                response["lease"]["decision_summary"] = JArray.Parse(@"[{'fact':'local.ready'},{'fact':'remote.pointed_target'},
                                    {'fact':'robot.joints'},{'fact':'robot.hand_reference'}]");
                            }
                            break;
                        case "/api/operator":
                            using (var reader = new StreamReader(context.Request.InputStream, Encoding.UTF8))
                            {
                                var request = JObject.Parse(reader.ReadToEnd());
                                if (context.Request.Headers["X-Session-Token"] != "test-only") throw new Exception("Missing session credential");
                                Operations.Enqueue(request);
                                response = (JObject)request["data"].DeepClone();
                                if ((string)request["operation"] == "respond")
                                { Interlocked.Increment(ref DecisionCount); closed = true; response["label"] = "correct"; }
                            }
                            break;
                        default: response = new JObject(); break;
                    }
                    byte[] bytes = Encoding.UTF8.GetBytes(response.ToString(Formatting.None));
                    context.Response.ContentType = "application/json"; context.Response.ContentLength64 = bytes.Length;
                    context.Response.OutputStream.Write(bytes, 0, bytes.Length); context.Response.Close();
                }
            }
            catch (HttpListenerException) { }
            catch (ObjectDisposedException) { }
        }
        public void Dispose() { listener.Stop(); listener.Close(); thread.Join(1000); }
    }
}
