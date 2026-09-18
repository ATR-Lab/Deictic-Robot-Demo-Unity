using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.XR;

namespace Deictic.Editor
{
    /// <summary>One-shot read-only diagnostics; never changes tracking, cameras, or scene objects.</summary>
    public static class XrRuntimeProbe
    {
        [Serializable] sealed class PoseData
        {
            public string path;
            public bool active;
            public Vector3 position, localPosition, lossyScale;
            public Quaternion rotation, localRotation;
            public float[] localToWorld;
        }
        [Serializable] sealed class FocusData
        {
            public bool applicationFocused, editingTextField, keyboardAvailable, keyboardEnabled, mouseAvailable, mouseEnabled;
            public string focusedEditorWindow, mouseOverEditorWindow, keyboardReadError, updateMode, currentUpdateType, backgroundBehavior, editorInputBehavior;
            public int guiKeyboardControl;
            public string[] pressedKeys;
            public ControlData[] controls;
        }
        [Serializable] sealed class ControlData
        {
            public string name, value;
            public bool pressed, pressedThisFrame, releasedThisFrame;
        }
        [Serializable] sealed class ControllerData
        {
            public string controller, error;
            public bool connected, positionTracked, orientationTracked, indexTrigger, indexTriggerDown, buttonOne, buttonOneDown, buttonTwo, buttonTwoDown;
        }
        [Serializable] sealed class InputData
        {
            public string path, inputSource, bridge, viewCamera, cameraView;
            public bool enabled, active, activeAndEnabled, synthetic, hasCandidate, cameraViewEnabled, robotView;
            public Vector3 candidate;
            public PoseData head, pointer;
        }
        [Serializable] sealed class InputSample
        {
            public int frame;
            public double editorTime;
            public FocusData focus;
            public InputData[] inputs;
            public ControllerData[] controllers;
        }
        [Serializable] sealed class InputTrace
        {
            public string startedUtc, endedUtc, phase, note;
            public InputSample[] samples;
        }
        [Serializable] sealed class PointData { public string name; public Vector3 world; }
        [Serializable] sealed class ProjectionData
        {
            public string point, eye, raw, error;
            public bool finite, insideViewport;
            public Vector3 viewport;
        }
        [Serializable] sealed class CameraData
        {
            public string path, tag, targetEye, targetTexture, error;
            public bool enabled, activeInHierarchy, stereoEnabled, orthographic;
            public int cullingMask, targetDisplay;
            public float nearClip, farClip, fieldOfView, aspect, depth;
            public Rect pixelRect;
            public PoseData pose;
            public float[] worldToCamera, projection, monoViewTimesHead, leftViewTimesHead, rightViewTimesHead;
            public string[] nonFiniteMatrixEntries;
            public ProjectionData[] points;
        }
        [Serializable] sealed class Report
        {
            public string utc, phase, unityVersion, activeScene, configuredTrackingOrigin, runtimeTrackingOrigin;
            public bool playing, paused, hmdPresent, vrFocus, inputFocus;
            public int frame, activeCameraCount;
            public double editorTime;
            public FocusData requestedFocus, capturedFocus;
            public PoseData rig, trackingSpace, head;
            public string[] xrInputOrigins, xrDisplays, notes;
            public PointData[] points;
            public CameraData[] cameras;
            public InputData[] deicticInputs;
            public ControllerData[] controllers;
        }

        static Camera requestedCamera;
        static FocusData requestedFocus;
        static double requestTime;
        static bool pending;
        static readonly List<InputSample> inputSamples = new List<InputSample>();
        static EventInfo inputUpdateEvent;
        static Action inputUpdateCallback;
        static double inputTraceStart, lastInputSample;
        static string inputTraceUtc, inputTracePhase;
        static bool tracingInput;

        [MenuItem("Deictic/Diagnostics/Record XR Input for 10 Seconds %&i")]
        public static void RecordInput()
        {
            if (tracingInput) FinishInputTrace();
            inputSamples.Clear(); inputTraceStart = EditorApplication.timeSinceStartup;
            lastInputSample = double.NegativeInfinity; inputTraceUtc = DateTime.UtcNow.ToString("O");
            tracingInput = true; inputTracePhase = "EditorApplication.update fallback";
            try
            {
                var type = Type.GetType("UnityEngine.InputSystem.InputSystem, Unity.InputSystem");
                inputUpdateEvent = type?.GetEvent("onAfterUpdate", BindingFlags.Public | BindingFlags.Static);
                inputUpdateCallback = SampleInput;
                if (inputUpdateEvent != null)
                {
                    inputUpdateEvent.AddEventHandler(null, inputUpdateCallback);
                    inputTracePhase = "InputSystem.onAfterUpdate";
                }
            }
            catch (Exception e) { inputUpdateEvent = null; Debug.LogWarning("XR input trace uses editor fallback: " + e.Message); }
            EditorApplication.update += PollInputTrace;
            Debug.Log("Read-only XR input recording started for 10 seconds. Focus Game view and try V, mouse, and controller actions.");
        }
        static void PollInputTrace()
        {
            if (inputUpdateEvent == null) SampleInput();
            if (EditorApplication.timeSinceStartup - inputTraceStart >= 10 || inputSamples.Count >= 500) FinishInputTrace();
        }
        static void SampleInput()
        {
            if (!tracingInput) return;
            var focus = ReadFocus();
            double now = EditorApplication.timeSinceStartup;
            bool edge = focus.controls != null && focus.controls.Any(c => c.pressedThisFrame || c.releasedThisFrame);
            if (now - lastInputSample < .05 && !edge) return;
            lastInputSample = now;
            inputSamples.Add(new InputSample { frame = Time.frameCount, editorTime = now, focus = focus,
                inputs = ReadInputs(), controllers = ReadControllers() });
        }
        static void FinishInputTrace()
        {
            tracingInput = false; EditorApplication.update -= PollInputTrace;
            try { inputUpdateEvent?.RemoveEventHandler(null, inputUpdateCallback); }
            catch (Exception e) { Debug.LogWarning("XR input trace callback cleanup: " + e.Message); }
            inputUpdateEvent = null; inputUpdateCallback = null;
            var report = new InputTrace { startedUtc = inputTraceUtc, endedUtc = DateTime.UtcNow.ToString("O"),
                phase = inputTracePhase, samples = inputSamples.ToArray(),
                note = "Read-only, 20 Hz plus keyboard/mouse edges, at most 500 samples. After-input state may precede the component Update in that frame. Editor fallback may miss brief edges. No controls or device settings changed." };
            WriteJson("xr-input-trace-", report);
        }

        [MenuItem("Deictic/Diagnostics/Write XR Runtime Probe %&d")]
        public static void WriteProbe()
        {
            ClearPending();
            requestedFocus = ReadFocus();
            requestTime = EditorApplication.timeSinceStartup;
            var rig = UnityEngine.Object.FindFirstObjectByType<OVRCameraRig>();
            requestedCamera = rig && rig.centerEyeAnchor ? rig.centerEyeAnchor.GetComponent<Camera>() : Camera.main;
            if (!EditorApplication.isPlaying || EditorApplication.isPaused || !requestedCamera)
            {
                Capture("immediate (stopped, paused, or no tracked camera)");
                return;
            }
            pending = true;
            RenderPipelineManager.beginCameraRendering += OnCameraRendering;
            EditorApplication.update += CheckTimeout;
            Debug.Log("XR runtime probe requested; waiting up to three seconds for the tracked camera render.");
        }

        static void OnCameraRendering(ScriptableRenderContext context, Camera camera)
        {
            if (pending && camera == requestedCamera) Capture("beginCameraRendering");
        }
        static void CheckTimeout()
        {
            if (pending && EditorApplication.timeSinceStartup - requestTime >= 3)
                Capture("editor timeout fallback (no tracked camera render in three seconds)");
        }
        static void ClearPending()
        {
            pending = false;
            RenderPipelineManager.beginCameraRendering -= OnCameraRendering;
            EditorApplication.update -= CheckTimeout;
        }
        static void Capture(string phase)
        {
            ClearPending();
            try
            {
                var notes = new List<string> { "Read-only snapshot. Opening the menu can change editor/input focus.",
                    "Matrices are row-major. Compare stereo view-times-head against mono view-times-head; camera view includes Unity's Z flip.",
                    "A render-phase callback is preferred; timeout/stopped snapshots may contain last-frame camera matrices." };
                var rig = UnityEngine.Object.FindFirstObjectByType<OVRCameraRig>();
                Transform head = rig ? rig.centerEyeAnchor : null;
                var manager = rig ? rig.GetComponent<OVRManager>() : OVRManager.instance;
                var report = new Report
                {
                    utc = DateTime.UtcNow.ToString("O"), phase = phase, unityVersion = Application.unityVersion,
                    activeScene = UnityEngine.SceneManagement.SceneManager.GetActiveScene().path,
                    playing = EditorApplication.isPlaying, paused = EditorApplication.isPaused,
                    frame = Time.frameCount, editorTime = EditorApplication.timeSinceStartup,
                    requestedFocus = requestedFocus, capturedFocus = ReadFocus(),
                    deicticInputs = ReadInputs(), controllers = ReadControllers(),
                    rig = Pose(rig ? rig.transform : null), trackingSpace = Pose(rig ? rig.trackingSpace : null), head = Pose(head),
                    activeCameraCount = Camera.allCamerasCount
                };
                if (manager)
                {
                    var property = new SerializedObject(manager).FindProperty("_trackingOriginType");
                    report.configuredTrackingOrigin = property == null ? "unavailable" : ((OVRManager.TrackingOrigin)property.intValue).ToString();
                    if (report.playing)
                    {
                        try
                        {
                            report.runtimeTrackingOrigin = manager.trackingOriginType.ToString();
                            report.hmdPresent = OVRManager.isHmdPresent;
                            report.vrFocus = OVRManager.hasVrFocus;
                            report.inputFocus = OVRManager.hasInputFocus;
                        }
                        catch (Exception e) { notes.Add("OVR runtime read: " + e.Message); }
                    }
                }
                var inputs = new List<XRInputSubsystem>(); SubsystemManager.GetSubsystems(inputs);
                report.xrInputOrigins = inputs.Select(s => s.subsystemDescriptor.id + ": running=" + s.running + ", origin=" + s.GetTrackingOriginMode()).ToArray();
                var displays = new List<XRDisplaySubsystem>(); SubsystemManager.GetSubsystems(displays);
                report.xrDisplays = displays.Select(s => s.subsystemDescriptor.id + ": running=" + s.running).ToArray();
                var points = new List<PointData>();
                foreach (Transform transform in UnityEngine.Object.FindObjectsByType<Transform>(FindObjectsInactive.Include, FindObjectsSortMode.None))
                {
                    if (!transform.gameObject.scene.IsValid() || !transform.gameObject.scene.isLoaded) continue;
                    string name = transform.name;
                    if (name != "Table" && name != "Deictic status" && name != "Camera view controls" &&
                        name != "Camera view toggle" && name != "K1 measured state" && !name.StartsWith("Reach target ", StringComparison.Ordinal)) continue;
                    points.Add(new PointData { name = PathOf(transform), world = transform.position });
                    if (name == "Table") points.Add(new PointData { name = PathOf(transform) + "/top", world = transform.TransformPoint(new Vector3(0, .5f, 0)) });
                    if (transform is RectTransform rect)
                    {
                        var corners = new Vector3[4]; rect.GetWorldCorners(corners);
                        for (int i = 0; i < corners.Length; i++) points.Add(new PointData { name = PathOf(transform) + "/corner" + i, world = corners[i] });
                    }
                    if (name == "Deictic status")
                    {
                        var mesh = transform.GetComponent<MeshFilter>();
                        if (mesh && mesh.sharedMesh)
                        {
                            var b = mesh.sharedMesh.bounds;
                            for (int i = 0; i < 4; i++) points.Add(new PointData { name = PathOf(transform) + "/meshCorner" + i,
                                world = transform.TransformPoint(new Vector3(i % 2 == 0 ? b.min.x : b.max.x, i < 2 ? b.min.y : b.max.y, b.center.z)) });
                        }
                    }
                }
                report.points = points.ToArray();
                report.cameras = UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsInactive.Include, FindObjectsSortMode.None)
                    .Where(c => c.gameObject.scene.IsValid() && c.gameObject.scene.isLoaded)
                    .Select(c => ReadCamera(c, head, report.points)).ToArray();
                report.notes = notes.ToArray();
                string directory = System.IO.Path.GetFullPath(System.IO.Path.Combine(Application.dataPath, "../../output"));
                Directory.CreateDirectory(directory);
                string path = System.IO.Path.Combine(directory, "xr-runtime-probe-" + DateTime.UtcNow.ToString("yyyyMMdd-HHmmss-fff") + ".json");
                File.WriteAllText(path, JsonUtility.ToJson(report, true));
                Debug.Log("XR runtime probe saved: " + path);
            }
            catch (Exception e) { Debug.LogError("XR runtime probe could not write a report: " + e); }
        }

        static CameraData ReadCamera(Camera c, Transform head, PointData[] points)
        {
            var result = new CameraData { path = PathOf(c.transform), tag = c.tag, enabled = c.enabled,
                activeInHierarchy = c.gameObject.activeInHierarchy, stereoEnabled = c.stereoEnabled,
                targetEye = c.stereoTargetEye.ToString(), targetTexture = c.targetTexture ? c.targetTexture.name : "none",
                targetDisplay = c.targetDisplay, cullingMask = c.cullingMask, nearClip = c.nearClipPlane,
                farClip = c.farClipPlane, fieldOfView = c.fieldOfView, aspect = c.aspect, depth = c.depth,
                orthographic = c.orthographic, pixelRect = c.pixelRect, pose = Pose(c.transform) };
            var projections = new List<ProjectionData>();
            var invalidMatrices = new List<string>();
            try
            {
                result.worldToCamera = Matrix(c.worldToCameraMatrix, "worldToCamera", invalidMatrices);
                result.projection = Matrix(c.projectionMatrix, "projection", invalidMatrices);
                if (head) result.monoViewTimesHead = Matrix(c.worldToCameraMatrix * head.localToWorldMatrix, "monoViewTimesHead", invalidMatrices);
                if (c.stereoEnabled && head)
                {
                    result.leftViewTimesHead = Matrix(c.GetStereoViewMatrix(Camera.StereoscopicEye.Left) * head.localToWorldMatrix, "leftViewTimesHead", invalidMatrices);
                    result.rightViewTimesHead = Matrix(c.GetStereoViewMatrix(Camera.StereoscopicEye.Right) * head.localToWorldMatrix, "rightViewTimesHead", invalidMatrices);
                }
                foreach (var point in points)
                {
                    var eyes = c.stereoEnabled ? new[] { Camera.MonoOrStereoscopicEye.Mono, Camera.MonoOrStereoscopicEye.Left, Camera.MonoOrStereoscopicEye.Right }
                        : new[] { Camera.MonoOrStereoscopicEye.Mono };
                    foreach (var eye in eyes)
                    {
                        var projection = new ProjectionData { point = point.name, eye = eye.ToString() };
                        try
                        {
                            Vector3 v = c.WorldToViewportPoint(point.world, eye);
                            projection.raw = v.x.ToString("R", CultureInfo.InvariantCulture) + "," + v.y.ToString("R", CultureInfo.InvariantCulture) + "," + v.z.ToString("R", CultureInfo.InvariantCulture);
                            projection.finite = Finite(v.x) && Finite(v.y) && Finite(v.z);
                            projection.viewport = projection.finite ? v : Vector3.zero;
                            projection.insideViewport = projection.finite && v.z > 0 && v.x >= 0 && v.x <= 1 && v.y >= 0 && v.y <= 1;
                        }
                        catch (Exception e) { projection.error = e.Message; }
                        projections.Add(projection);
                    }
                }
            }
            catch (Exception e) { result.error = e.Message; }
            result.points = projections.ToArray(); result.nonFiniteMatrixEntries = invalidMatrices.ToArray(); return result;
        }
        static PoseData Pose(Transform t) => !t ? null : new PoseData { path = PathOf(t), active = t.gameObject.activeInHierarchy,
            position = t.position, rotation = t.rotation, localPosition = t.localPosition, localRotation = t.localRotation,
            lossyScale = t.lossyScale, localToWorld = Matrix(t.localToWorldMatrix) };
        static string PathOf(Transform t) => !t ? "missing" : t.parent ? PathOf(t.parent) + "/" + t.name : t.name;
        static bool Finite(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        static float[] Matrix(Matrix4x4 matrix, string label = "matrix", List<string> invalid = null)
        {
            var values = new float[16];
            for (int row = 0; row < 4; row++) for (int col = 0; col < 4; col++)
            {
                float value = matrix[row, col];
                values[row * 4 + col] = Finite(value) ? value : 0;
                if (!Finite(value)) invalid?.Add(label + "[" + row + "," + col + "]=" + value.ToString("R", CultureInfo.InvariantCulture));
            }
            return values;
        }
        static FocusData ReadFocus()
        {
            var result = new FocusData { applicationFocused = Application.isFocused, editingTextField = EditorGUIUtility.editingTextField,
                guiKeyboardControl = GUIUtility.keyboardControl,
                focusedEditorWindow = EditorWindow.focusedWindow ? EditorWindow.focusedWindow.titleContent.text : "none",
                mouseOverEditorWindow = EditorWindow.mouseOverWindow ? EditorWindow.mouseOverWindow.titleContent.text : "none" };
            var pressed = new List<string>();
            try
            {
                // Reflection avoids changing the editor assembly's package dependencies.
                Type type = Type.GetType("UnityEngine.InputSystem.Keyboard, Unity.InputSystem");
                object keyboard = type?.GetProperty("current", BindingFlags.Public | BindingFlags.Static)?.GetValue(null);
                result.keyboardAvailable = keyboard != null;
                result.keyboardEnabled = ReadBool(keyboard, "enabled");
                var keys = keyboard == null ? null : type.GetProperty("allKeys")?.GetValue(keyboard) as IEnumerable;
                if (keys != null) foreach (object key in keys)
                    if (pressed.Count < 32 && key.GetType().GetProperty("isPressed")?.GetValue(key) is bool down && down)
                        pressed.Add(key.GetType().GetProperty("name")?.GetValue(key)?.ToString() ?? "unknown");
                var controls = new List<ControlData>();
                foreach (string name in new[] { "vKey", "spaceKey", "enterKey", "escapeKey" })
                    controls.Add(ReadControl("keyboard." + name, ReadProperty(keyboard, name)));
                Type mouseType = Type.GetType("UnityEngine.InputSystem.Mouse, Unity.InputSystem");
                object mouse = mouseType?.GetProperty("current", BindingFlags.Public | BindingFlags.Static)?.GetValue(null);
                result.mouseAvailable = mouse != null; result.mouseEnabled = ReadBool(mouse, "enabled");
                foreach (string name in new[] { "leftButton", "rightButton", "position", "delta" })
                    controls.Add(ReadControl("mouse." + name, ReadProperty(mouse, name)));
                result.controls = controls.ToArray();
                Type systemType = Type.GetType("UnityEngine.InputSystem.InputSystem, Unity.InputSystem");
                object settings = systemType?.GetProperty("settings", BindingFlags.Public | BindingFlags.Static)?.GetValue(null);
                result.updateMode = ReadProperty(settings, "updateMode")?.ToString();
                Type stateType = Type.GetType("UnityEngine.InputSystem.LowLevel.InputState, Unity.InputSystem");
                result.currentUpdateType = stateType?.GetProperty("currentUpdateType", BindingFlags.Public | BindingFlags.Static)?.GetValue(null)?.ToString();
                result.backgroundBehavior = ReadProperty(settings, "backgroundBehavior")?.ToString();
                result.editorInputBehavior = ReadProperty(settings, "editorInputBehaviorInPlayMode")?.ToString();
            }
            catch (Exception e) { result.keyboardReadError = e.Message; }
            result.pressedKeys = pressed.ToArray(); return result;
        }
        static object ReadProperty(object value, string name) => value?.GetType().GetProperty(name)?.GetValue(value);
        static bool ReadBool(object value, string name) => ReadProperty(value, name) is bool result && result;
        static ControlData ReadControl(string name, object control)
        {
            return new ControlData { name = name, pressed = ReadBool(control, "isPressed"),
                pressedThisFrame = ReadBool(control, "wasPressedThisFrame"), releasedThisFrame = ReadBool(control, "wasReleasedThisFrame"),
                value = control == null ? "unavailable" : control.GetType().GetMethod("ReadValue", Type.EmptyTypes)?.Invoke(control, null)?.ToString() };
        }
        static InputData[] ReadInputs() => UnityEngine.Object.FindObjectsByType<DeicticInput>(FindObjectsInactive.Include, FindObjectsSortMode.None)
            .Where(i => i.gameObject.scene.IsValid() && i.gameObject.scene.isLoaded).Select(i => new InputData {
                path = PathOf(i.transform), enabled = i.enabled, active = i.gameObject.activeInHierarchy, activeAndEnabled = i.isActiveAndEnabled,
                synthetic = i.synthetic, inputSource = i.InputSource, hasCandidate = i.HasCandidate, candidate = i.Candidate,
                bridge = i.bridge ? PathOf(i.bridge.transform) : "missing", viewCamera = i.viewCamera ? PathOf(i.viewCamera.transform) : "missing",
                head = Pose(i.head), pointer = Pose(i.pointer), cameraView = i.cameraView ? PathOf(i.cameraView.transform) : "missing",
                cameraViewEnabled = i.cameraView && i.cameraView.isActiveAndEnabled, robotView = i.cameraView && i.cameraView.RobotView }).ToArray();
        static ControllerData[] ReadControllers()
        {
            return new[] { OVRInput.Controller.LTouch, OVRInput.Controller.RTouch }.Select(controller => {
                var data = new ControllerData { controller = controller.ToString() };
                if (!EditorApplication.isPlaying) { data.error = "Editor not playing"; return data; }
                try
                {
                    data.connected = OVRInput.IsControllerConnected(controller);
                    data.positionTracked = OVRInput.GetControllerPositionTracked(controller);
                    data.orientationTracked = OVRInput.GetControllerOrientationTracked(controller);
                    data.indexTrigger = OVRInput.Get(OVRInput.Button.PrimaryIndexTrigger, controller);
                    data.indexTriggerDown = OVRInput.GetDown(OVRInput.Button.PrimaryIndexTrigger, controller);
                    data.buttonOne = OVRInput.Get(OVRInput.Button.One, controller);
                    data.buttonOneDown = OVRInput.GetDown(OVRInput.Button.One, controller);
                    data.buttonTwo = OVRInput.Get(OVRInput.Button.Two, controller);
                    data.buttonTwoDown = OVRInput.GetDown(OVRInput.Button.Two, controller);
                }
                catch (Exception e) { data.error = e.Message; }
                return data;
            }).ToArray();
        }
        static void WriteJson(string prefix, object report)
        {
            try
            {
                string directory = System.IO.Path.GetFullPath(System.IO.Path.Combine(Application.dataPath, "../../output"));
                Directory.CreateDirectory(directory);
                string path = System.IO.Path.Combine(directory, prefix + DateTime.UtcNow.ToString("yyyyMMdd-HHmmss-fff") + ".json");
                File.WriteAllText(path, JsonUtility.ToJson(report, true)); Debug.Log("XR input trace saved: " + path);
            }
            catch (Exception e) { Debug.LogError("XR input trace could not write a report: " + e); }
        }
    }
}
