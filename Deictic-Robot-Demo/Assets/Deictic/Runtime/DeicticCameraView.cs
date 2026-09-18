using UnityEngine;
using UnityEngine.UI;

namespace Deictic
{
    /// <summary>World-fixed robot camera monitor. Its pixels are for viewing, not target selection.</summary>
    public sealed class DeicticCameraView : MonoBehaviour
    {
        public bool RobotView { get; private set; }
        public Button ToggleButton { get; private set; }
        public RectTransform ToggleRect { get; private set; }
        public RosCameraStream Stream { get; private set; }
        DeicticBridge bridge;
        GameObject canvasRoot, robotPanel;
        RawImage image;
        Text buttonLabel, frameLabel, instructions;
        Image buttonBackground;
        AspectRatioFitter aspect;
        bool synthetic, stereo;
        OVRPassthroughLayer passthrough;
        Material overlayMaterial, imageMaterial;

        public void Initialize(Transform head, DeicticBridge connection)
        {
            bridge = connection;
            synthetic = connection.settings.syntheticScene;
            stereo = connection.settings.robotCameraStereo;
            // The image's eye selection must never crop text, controls or backgrounds.
            overlayMaterial = new Material(Resources.Load<Shader>("Deictic/CameraViewOverlay"));
            overlayMaterial.SetFloat("_StereoSideBySide", 0);
            imageMaterial = new Material(overlayMaterial);
            imageMaterial.SetFloat("_StereoSideBySide", stereo ? 1 : 0);
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
                var camera = head.GetComponent<Camera>();
                if (camera) { camera.clearFlags = CameraClearFlags.SolidColor; camera.backgroundColor = Color.clear; }
            }
            Stream = gameObject.AddComponent<RosCameraStream>();
            Stream.enabled = false;
            Stream.Initialize(connection.Ros, connection.settings.robotCameraTopic, connection.settings.robotCameraTimeout, stereo);
            canvasRoot = new GameObject("Camera view controls", typeof(RectTransform), typeof(Canvas));
            // Like the Interaction SDK's scene canvases, this has a stable world
            // pose. Our existing controller ray owns hover/click dispatch; adding
            // a second PointableCanvas event route would process the same trigger twice.
            // The configured point is the toggle center; the monitor sits above it.
            canvasRoot.transform.SetPositionAndRotation(
                connection.settings.cameraControlsWorldPosition + Vector3.up * (.00085f * 408),
                Quaternion.identity);
            canvasRoot.transform.localScale = Vector3.one * .00085f;
            var canvas = canvasRoot.GetComponent<Canvas>();
            canvas.renderMode = RenderMode.WorldSpace;
            canvas.worldCamera = head.GetComponent<Camera>();
            canvas.sortingOrder = 20;
            canvasRoot.GetComponent<RectTransform>().sizeDelta = new Vector2(1000, 850);

            robotPanel = Rect("Robot head camera monitor", canvasRoot.transform, Vector2.zero, new Vector2(980, 760)).gameObject;
            var background = robotPanel.AddComponent<Image>();
            background.material = overlayMaterial;
            background.color = new Color(.035f, .045f, .065f, 1);
            background.raycastTarget = false;
            Label("View title", robotPanel.transform, new Vector2(0, 333), new Vector2(920, 42),
                (stereo ? "ROBOT HEAD STEREO" : "ROBOT HEAD CAMERA") + (synthetic ? " / ISAAC SIM" : ""), 26);
            var frame = Rect("Camera image viewport", robotPanel.transform, new Vector2(0, 18), new Vector2(900, 600));
            image = Rect("Camera image", frame, Vector2.zero, frame.sizeDelta).gameObject.AddComponent<RawImage>();
            image.material = imageMaterial;
            image.raycastTarget = false;
            image.uvRect = new Rect(0, 1, 1, -1); // ROS top row first, Unity texture bottom row first.
            aspect = image.gameObject.AddComponent<AspectRatioFitter>();
            aspect.aspectMode = AspectRatioFitter.AspectMode.FitInParent;
            aspect.aspectRatio = 4f / 3f;
            frameLabel = Label("Camera feed status", robotPanel.transform, new Vector2(0, -299), new Vector2(920, 42), "Waiting for robot camera", 22);
            instructions = Label("Camera view instructions", robotPanel.transform, new Vector2(0, -341), new Vector2(920, 40),
                "Return to User view to select targets. B / Esc cancels motion.", 19);

            ToggleRect = Rect("Camera view toggle", canvasRoot.transform, new Vector2(0, -408), new Vector2(600, 64));
            buttonBackground = ToggleRect.gameObject.AddComponent<Image>();
            buttonBackground.material = overlayMaterial;
            buttonBackground.color = new Color(.055f, .2f, .28f, 1);
            ToggleButton = ToggleRect.gameObject.AddComponent<Button>();
            ToggleButton.targetGraphic = buttonBackground;
            ToggleButton.onClick.AddListener(Toggle);
            buttonLabel = Label("Toggle label", ToggleRect, Vector2.zero, new Vector2(580, 60), "User view  →  Robot camera", 25);
            SetRobotView(false);
        }

        static RectTransform Rect(string name, Transform parent, Vector2 position, Vector2 size)
        {
            var rect = new GameObject(name, typeof(RectTransform)).GetComponent<RectTransform>();
            rect.SetParent(parent, false);
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
            RobotView = value && isActiveAndEnabled;
            if (!canvasRoot) return;
            canvasRoot.SetActive(isActiveAndEnabled);
            robotPanel.SetActive(RobotView);
            image.enabled = false;
            Stream.enabled = RobotView;
            if (passthrough) passthrough.hidden = RobotView;
            buttonLabel.text = RobotView ? "Robot camera  →  User view" : "User view  →  Robot camera";
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
            image.enabled = fresh;
            if (fresh)
            {
                image.texture = Stream.Texture;
                aspect.aspectRatio = (float)Stream.Texture.width / ((stereo ? 2 : 1) * Stream.Texture.height);
            }
            frameLabel.color = fresh ? new Color(.5f, 1, .8f) : new Color(1, .72f, .3f);
            frameLabel.text = Stream.Status;
            instructions.text = bridge != null && bridge.TeleopControlsBusy ?
                "BIMANUAL CLUTCH · release either trigger to hold both arms · B / Esc stops" :
                "Both triggers: arm control · User view: target selection · B / Esc cancels";
        }
        void OnDestroy()
        {
            if (ToggleButton) ToggleButton.onClick.RemoveListener(Toggle);
            if (Stream) { Stream.enabled = false; Destroy(Stream); }
            if (passthrough) Destroy(passthrough);
            if (canvasRoot) Destroy(canvasRoot);
            if (overlayMaterial) Destroy(overlayMaterial);
            if (imageMaterial) Destroy(imageMaterial);
        }
        void OnEnable() { if (canvasRoot) SetRobotView(false); }
        void OnDisable() { SetRobotView(false); }
    }
}
