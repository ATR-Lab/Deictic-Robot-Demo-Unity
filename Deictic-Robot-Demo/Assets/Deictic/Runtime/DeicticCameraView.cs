using UnityEngine;
using UnityEngine.UI;

namespace Deictic
{
    /// <summary>Full-eye robot POV with a separate, world-fixed return control.</summary>
    [DefaultExecutionOrder(60)]
    public sealed class DeicticCameraView : MonoBehaviour
    {
        public bool RobotView { get; private set; }
        public Button ToggleButton { get; private set; }
        public RectTransform ToggleRect { get; private set; }
        public RosCameraStream Stream { get; private set; }
        public Material ImmersiveMaterial { get; private set; }
        public Text StatusText { get; private set; }
        // Reserved in TagManager. Only return controls, pointer and fault/status
        // feedback remain in the headset camera's culling mask during robot POV.
        public const int RobotUiLayer = 30;
        DeicticBridge bridge;
        GameObject canvasRoot, statusPanel;
        Text buttonLabel, instructions;
        Image buttonBackground;
        bool synthetic, stereo;
        Camera headCamera;
        int userCullingMask;
        OVRPassthroughLayer passthrough;
        Material overlayMaterial;

        public void Initialize(Transform head, DeicticBridge connection)
        {
            bridge = connection;
            synthetic = connection.settings.syntheticScene;
            stereo = connection.settings.robotCameraStereo;
            headCamera = head.GetComponent<Camera>();
            // The image's eye selection must never crop text, controls or backgrounds.
            overlayMaterial = new Material(Resources.Load<Shader>("Deictic/CameraViewOverlay"));
            overlayMaterial.SetFloat("_StereoSideBySide", 0);
            ImmersiveMaterial = new Material(Resources.Load<Shader>("Deictic/RobotPovFullscreen"))
            { name = "Live robot POV", hideFlags = HideFlags.DontSave };
            ImmersiveMaterial.SetFloat("_StereoSideBySide", stereo ? 1 : 0);
            bool userPassthrough = !synthetic;
#if UNITY_ANDROID && !UNITY_EDITOR
            // Simulated robot/targets still use the wearer's real surroundings
            // in User view on a Quest. This does not publish camera images.
            userPassthrough = true;
#endif
            if (userPassthrough && OVRManager.instance)
            {
                OVRManager.instance.isInsightPassthroughEnabled = true;
                passthrough = gameObject.AddComponent<OVRPassthroughLayer>();
                passthrough.overlayType = OVROverlay.OverlayType.Underlay;
                if (headCamera) { headCamera.clearFlags = CameraClearFlags.SolidColor; headCamera.backgroundColor = Color.clear; }
            }
            Stream = gameObject.AddComponent<RosCameraStream>();
            Stream.enabled = false;
            Stream.Initialize(connection.Ros, connection.settings.robotCameraTopic, connection.settings.robotCameraTimeout, stereo);
            canvasRoot = new GameObject("Camera view controls", typeof(RectTransform), typeof(Canvas));
            canvasRoot.layer = RobotUiLayer;
            // Like the Interaction SDK's scene canvases, this has a stable world
            // pose. Our existing controller ray owns hover/click dispatch; adding
            // a second PointableCanvas event route would process the same trigger twice.
            canvasRoot.transform.SetPositionAndRotation(
                connection.settings.cameraControlsWorldPosition,
                Quaternion.identity);
            canvasRoot.transform.localScale = Vector3.one * .00085f;
            var canvas = canvasRoot.GetComponent<Canvas>();
            canvas.renderMode = RenderMode.WorldSpace;
            canvas.worldCamera = headCamera;
            canvas.sortingOrder = 20;
            canvasRoot.GetComponent<RectTransform>().sizeDelta = new Vector2(900, 220);

            statusPanel = Rect("Robot POV status", canvasRoot.transform, new Vector2(0, 95), new Vector2(900, 110)).gameObject;
            var background = statusPanel.AddComponent<Image>();
            background.material = overlayMaterial;
            background.color = new Color(.035f, .045f, .065f, 1);
            background.raycastTarget = false;
            StatusText = Label("Camera feed status", statusPanel.transform, new Vector2(0, 22), new Vector2(880, 42), "Waiting for robot camera", 22);
            instructions = Label("Camera view instructions", statusPanel.transform, new Vector2(0, -25), new Vector2(880, 40),
                "User view: target selection · B / Esc: stop", 19);

            ToggleRect = Rect("Camera view toggle", canvasRoot.transform, Vector2.zero, new Vector2(600, 64));
            buttonBackground = ToggleRect.gameObject.AddComponent<Image>();
            buttonBackground.material = overlayMaterial;
            buttonBackground.color = new Color(.055f, .2f, .28f, 1);
            ToggleButton = ToggleRect.gameObject.AddComponent<Button>();
            ToggleButton.targetGraphic = buttonBackground;
            ToggleButton.onClick.AddListener(Toggle);
            buttonLabel = Label("Toggle label", ToggleRect, Vector2.zero, new Vector2(580, 60), "User view  →  Robot POV", 25);
            SetRobotView(false);
        }

        static RectTransform Rect(string name, Transform parent, Vector2 position, Vector2 size)
        {
            var rect = new GameObject(name, typeof(RectTransform)).GetComponent<RectTransform>();
            rect.SetParent(parent, false);
            rect.gameObject.layer = parent.gameObject.layer;
            rect.anchorMin = rect.anchorMax = rect.pivot = new Vector2(.5f, .5f);
            rect.anchoredPosition = position;
            rect.sizeDelta = size;
            return rect;
        }
        Text Label(string name, Transform parent, Vector2 position, Vector2 size, string text, int fontSize)
        {
            var label = Rect(name, parent, position, size).gameObject.AddComponent<Text>();
            label.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            label.material = overlayMaterial;
            label.fontSize = fontSize;
            label.alignment = TextAnchor.MiddleCenter;
            label.color = Color.white;
            label.text = text;
            label.raycastTarget = false;
            return label;
        }
        public void Toggle() { if (bridge == null || !bridge.TeleopControlsBusy) SetRobotView(!RobotView); }
        public void SetRobotView(bool value)
        {
            bool next = value && isActiveAndEnabled;
            if (next && !RobotView && headCamera)
            {
                userCullingMask = headCamera.cullingMask;
                headCamera.cullingMask = 1 << RobotUiLayer;
                RobotPovRendererFeature.Show(headCamera, ImmersiveMaterial);
            }
            else if (!next && RobotView)
            {
                RobotPovRendererFeature.Hide(headCamera);
                if (headCamera) headCamera.cullingMask = userCullingMask;
            }
            RobotView = next;
            if (!canvasRoot) return;
            canvasRoot.SetActive(isActiveAndEnabled);
            statusPanel.SetActive(RobotView);
            ImmersiveMaterial.SetFloat("_HasFrame", 0);
            ImmersiveMaterial.SetTexture("_MainTex", Texture2D.blackTexture);
            Stream.enabled = RobotView;
            if (passthrough) passthrough.hidden = RobotView;
            buttonLabel.text = RobotView ? "Robot POV  →  User view" : "User view  →  Robot POV";
            // A view change only changes presentation; it never emits a goal or execution.
        }
        public bool PointAtToggle(Ray ray, out Vector3 hit)
        {
            hit = default;
            if (!isActiveAndEnabled || !ToggleRect || !ToggleButton.interactable) return false;
            var plane = new Plane(ToggleRect.forward, ToggleRect.position);
            if (!plane.Raycast(ray, out float distance) || distance < 0 || distance > 4) return false;
            Vector3 point = ray.GetPoint(distance);
            Vector3 local = ToggleRect.InverseTransformPoint(point);
            if (!ToggleRect.rect.Contains(new Vector2(local.x, local.y))) return false;
            hit = point;
            return true;
        }
        public void SetHovered(bool hovered)
        {
            if (buttonBackground) buttonBackground.color = hovered ? new Color(.1f, .48f, .58f, 1) : new Color(.055f, .2f, .28f, 1);
        }
        void Update()
        {
            if (ToggleButton) ToggleButton.interactable = bridge == null || !bridge.TeleopControlsBusy;
            if (!RobotView || !Stream) return;
            bool fresh = Stream.HasFreshFrame;
            ImmersiveMaterial.SetFloat("_HasFrame", fresh ? 1 : 0);
            ImmersiveMaterial.SetTexture("_MainTex", fresh ? Stream.Texture : Texture2D.blackTexture);
            ImmersiveMaterial.SetFloat("_SourceAspect", Stream.EyeAspectRatio);
            ImmersiveMaterial.SetFloat("_RosTopLeft", Stream.TopRowAtTextureYZero ? 1 : 0);
            StatusText.color = fresh ? new Color(.5f, 1, .8f) : new Color(1, .72f, .3f);
            StatusText.text = Stream.Status;
            instructions.text = bridge != null && bridge.TeleopControlsBusy ?
                "BIMANUAL CLUTCH · release either trigger to hold both arms · B / Esc stops" :
                "Both triggers: arm control · User view: target selection · B / Esc cancels";
        }
        void OnDestroy()
        {
            SetRobotView(false);
            if (ToggleButton) ToggleButton.onClick.RemoveListener(Toggle);
            if (Stream) { Stream.enabled = false; Destroy(Stream); }
            if (passthrough) Destroy(passthrough);
            if (canvasRoot) Destroy(canvasRoot);
            if (overlayMaterial) Destroy(overlayMaterial);
            if (ImmersiveMaterial) Destroy(ImmersiveMaterial);
        }
        void OnEnable() { if (canvasRoot) SetRobotView(false); }
        void OnDisable() { SetRobotView(false); }
    }
}
