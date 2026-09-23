using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using Deictic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

/// <summary>Opt-in real OVR native pose -> normal DeicticInput.Update -> head Tick -> Isaac test.</summary>
public sealed class NativeHeadInputRosTests
{
    [Serializable] sealed class HeadStatus
    {
        public double stamp;
        public bool active;
        public string session_id, reason;
        public int sequence;
        public double[] measured, targets;
    }
    [Serializable] sealed class NativeSample
    {
        public int sequence;
        public bool hmdPresent, orientationTracked, inputFocus;
        public Quaternion pluginRotation, anchorRotation, sampledRotation;
        public float pluginAnchorErrorDegrees;
        public double[] expectedWireRotation;
    }
    [Serializable] sealed class Evidence
    {
        public string scope = "Actual Meta XR Simulator OVR head tracking through DeicticInput.Update and RobotHeadTracking.Tick; no head transform is assigned by the test";
        public List<NativeSample> sourceSamples = new List<NativeSample>();
        public int matchedCommandCount;
        public HeadStatus activeStatus, releasedStatus;
        public DeicticBridge.RobotHeadCommand releaseCommand;
    }

    GameObject root, rigObject;
    DeicticSettings settings;
    DeicticBridge bridge;
    DeicticCameraView view;
    RobotHeadTracking headTracking;
    OVRCameraRig rig;
    NativeSimulatorFixture native;
    HeadStatus status;
    readonly Dictionary<int, DeicticBridge.RobotHeadCommand> commands = new Dictionary<int, DeicticBridge.RobotHeadCommand>();
    readonly Dictionary<int, NativeSample> samples = new Dictionary<int, NativeSample>();
    readonly HashSet<int> matched = new HashSet<int>();
    readonly Evidence evidence = new Evidence();

    [UnityTest, Category("NativeHeadSimulationIntegration")]
    public IEnumerator NativeTrackedHeadPublishesCurrentPoseAndReleasesInUserView()
    {
        if (Environment.GetEnvironmentVariable("DEICTIC_NATIVE_HEAD_INPUT_TEST") != "1")
            Assert.Ignore("Opt in only with an isolated Isaac endpoint and no competing head publisher.");
        Assert.That(Environment.GetEnvironmentVariable("DEICTIC_NATIVE_SIMULATOR"), Is.EqualTo("1"));
        Assert.That(UnityEngine.Object.FindObjectsByType<OVRCameraRig>(FindObjectsSortMode.None), Is.Empty,
            "Use the isolated test project/scene, not an existing running XR demo.");
        native = new NativeSimulatorFixture();
        yield return native.Initialize();

        // OVRManager can emit optional-extension diagnostics on initialization.
        // Suppression ends before the production input path or ROS assertions run.
        bool previousIgnore = LogAssert.ignoreFailingMessages;
        LogAssert.ignoreFailingMessages = true;
        try
        {
            rigObject = new GameObject("Native head input OVRCameraRig");
            rig = rigObject.AddComponent<OVRCameraRig>();
            rigObject.AddComponent<OVRManager>();
            float startupDeadline = Time.realtimeSinceStartup + 15;
            while ((!OVRManager.OVRManagerinitialized || !OVRManager.isHmdPresent ||
                    !OVRPlugin.GetNodeOrientationTracked(OVRPlugin.Node.EyeCenter) || !OVRManager.hasInputFocus)
                   && Time.realtimeSinceStartup < startupDeadline) yield return null;
            yield return new WaitForSecondsRealtime(.5f);
        }
        finally { LogAssert.ignoreFailingMessages = previousIgnore; }
        Assert.That(OVRManager.OVRManagerinitialized, Is.True, "The real OVRManager must initialize");
        Assert.That(OVRManager.isHmdPresent, Is.True, "The simulator must expose a present native HMD");
        Assert.That(OVRPlugin.GetNodeOrientationTracked(OVRPlugin.Node.EyeCenter), Is.True);
        Assert.That(OVRManager.hasInputFocus, Is.True, "The simulator session must have native XR input focus");

        root = new GameObject("Native head input production wiring");
        settings = UnityEngine.Object.Instantiate(Resources.Load<DeicticSettings>("DeicticSettings"));
        settings.rosHost = "127.0.0.1";
        settings.connectOnStart = true;
        settings.syntheticScene = true;
        bridge = root.AddComponent<DeicticBridge>();
        bridge.Initialize(settings);
        bridge.Ros.Subscribe<RosMessageTypes.Std.StringMsg>("/k1/head/status",
            message => status = JsonUtility.FromJson<HeadStatus>(message.data));
        bridge.Ros.Subscribe<RosMessageTypes.Std.StringMsg>("/k1/head/command", message =>
        {
            var command = JsonUtility.FromJson<DeicticBridge.RobotHeadCommand>(message.data);
            if (SameSession(command.session_id)) commands[command.sequence] = command;
        });
        view = root.AddComponent<DeicticCameraView>();
        view.Initialize(rig.centerEyeAnchor, bridge);
        headTracking = root.AddComponent<RobotHeadTracking>();
        headTracking.bridge = bridge;
        headTracking.head = rig.centerEyeAnchor;
        headTracking.bodyFrame = rig.trackingSpace;
        headTracking.cameraView = view;
        // Use the same Update dispatch as DeicticDemo; never call Tick/Step or
        // assign centerEyeAnchor.rotation from the test.
        var input = root.AddComponent<DeicticInput>();
        input.bridge = bridge;
        input.head = rig.centerEyeAnchor;
        input.pointer = rig.rightControllerAnchor;
        input.viewCamera = rig.centerEyeAnchor.GetComponent<Camera>();
        input.cameraView = view;
        input.robotHead = headTracking;
        input.synthetic = true;

        float deadline = Time.realtimeSinceStartup + 35;
        while ((!bridge.CanTrackRobotHead || !CurrentStatus(false)) && Time.realtimeSinceStartup < deadline)
            yield return null;
        Assert.That(bridge.CanTrackRobotHead && CurrentStatus(false), Is.True,
            "The normal input Update must establish a synchronized inactive head session.");
        view.SetRobotView(true);
        // Optional bounded window for external simulator UI head movement.
        // The test itself never injects or rewrites an XR pose.
        float.TryParse(Environment.GetEnvironmentVariable("DEICTIC_NATIVE_HEAD_OBSERVE_SECONDS"), out float observationSeconds);
        observationSeconds = Mathf.Clamp(observationSeconds, 0, 12);
        float observeUntil = Time.realtimeSinceStartup + observationSeconds;
        deadline = Time.realtimeSinceStartup + 20;
        bool measuredMatches = false;
        while (Time.realtimeSinceStartup < deadline)
        {
            yield return null;
            CaptureNativeSample();
            MatchCommands();
            if (CurrentStatus(true) && commands.TryGetValue(status.sequence, out var accepted) &&
                matched.Contains(status.sequence) && status.targets?.Length == 2 && status.measured?.Length == 2)
            {
                var q = accepted.orientation;
                Vector3 forward = new Quaternion((float)q[0], (float)q[1], (float)q[2], (float)q[3]) * Vector3.right;
                float yaw = Mathf.Clamp(Mathf.Atan2(forward.y, forward.x), -1.012f, 1.012f);
                float pitch = Mathf.Clamp(Mathf.Atan2(-forward.z, new Vector2(forward.x, forward.y).magnitude), -.314f, .794f);
                measuredMatches = Math.Abs(status.targets[0] - yaw) < .002 && Math.Abs(status.targets[1] - pitch) < .002 &&
                    Math.Abs(status.measured[0] - yaw) < .035 && Math.Abs(status.measured[1] - pitch) < .035;
                if (measuredMatches && matched.Count >= 5 && Time.realtimeSinceStartup >= observeUntil) break;
            }
        }
        Assert.That(headTracking.Sample.Tracked && headTracking.Sample.Active, Is.True,
            "The actual Tick must accept OVR EyeCenter tracking and XR focus.");
        Assert.That(matched.Count, Is.GreaterThanOrEqualTo(5), "Five native-source commands must return through ROS unchanged.");
        Assert.That(measuredMatches, Is.True, "Isaac must acknowledge and attain the current native head yaw/pitch target.");
        evidence.activeStatus = status;
        evidence.matchedCommandCount = matched.Count;

        int releaseBoundary = bridge.LastHeadCommandSequence;
        view.SetRobotView(false);
        deadline = Time.realtimeSinceStartup + 3;
        while (!InactiveReleaseAcknowledgedAfter(releaseBoundary) && Time.realtimeSinceStartup < deadline)
            yield return null;
        Assert.That(InactiveReleaseAcknowledgedAfter(releaseBoundary), Is.True,
            "Normal Tick must emit a post-toggle inactive command, echoed by ROS and acknowledged as head_view_inactive; watchdog expiry is insufficient.");
        evidence.releasedStatus = status;
        foreach (var pair in samples) if (matched.Contains(pair.Key)) evidence.sourceSamples.Add(pair.Value);
        string directory = Environment.GetEnvironmentVariable("DEICTIC_HEAD_ARTIFACT_DIR");
        if (!string.IsNullOrWhiteSpace(directory))
        {
            Directory.CreateDirectory(directory);
            File.WriteAllText(Path.Combine(directory, "native-head-input.json"), JsonUtility.ToJson(evidence, true));
        }
        Debug.Log("Native OVR head-input loop passed: " + matched.Count + " commands matched actual tracked poses, measured neck target matched, User-view release acknowledged.");
    }

    void CaptureNativeSample()
    {
        int sequence = bridge.LastHeadCommandSequence;
        if (samples.ContainsKey(sequence) || !headTracking.Sample.Active) return;
        Quaternion plugin = OVRPlugin.GetNodePose(OVRPlugin.Node.EyeCenter, OVRPlugin.Step.Render).ToOVRPose().orientation;
        var sample = new NativeSample
        {
            sequence = sequence, hmdPresent = OVRManager.isHmdPresent,
            orientationTracked = OVRPlugin.GetNodeOrientationTracked(OVRPlugin.Node.EyeCenter), inputFocus = OVRManager.hasInputFocus,
            pluginRotation = plugin, anchorRotation = rig.centerEyeAnchor.localRotation,
            sampledRotation = headTracking.Sample.Rotation,
            pluginAnchorErrorDegrees = Quaternion.Angle(plugin, rig.centerEyeAnchor.localRotation),
            expectedWireRotation = DeicticBridge.TeleopRotation(headTracking.Sample.Rotation)
        };
        Assert.That(sample.hmdPresent && sample.orientationTracked && sample.inputFocus, Is.True);
        Assert.That(sample.pluginAnchorErrorDegrees, Is.LessThan(2), "OVRCameraRig must follow the actual native plugin pose.");
        Assert.That(Quaternion.Angle(sample.sampledRotation, rig.centerEyeAnchor.localRotation), Is.LessThan(2),
            "The production Tick must sample the rig's native orientation in its unchanged tracking frame.");
        samples.Add(sequence, sample);
    }
    void MatchCommands()
    {
        foreach (var pair in commands)
        {
            var command = pair.Value;
            if (!command.active || !command.tracked || !samples.TryGetValue(pair.Key, out var source)) continue;
            Assert.That(command.orientation, Has.Length.EqualTo(4));
            var q = command.orientation;
            var expected = source.expectedWireRotation;
            float angle = Quaternion.Angle(new Quaternion((float)q[0], (float)q[1], (float)q[2], (float)q[3]),
                new Quaternion((float)expected[0], (float)expected[1], (float)expected[2], (float)expected[3]));
            Assert.That(angle, Is.LessThan(2), "ROS command must match the native pose sampled by the production Tick.");
            matched.Add(pair.Key);
        }
    }
    bool SameSession(string value) => bridge != null && Guid.TryParse(value, out var parsed) && parsed == Guid.Parse(bridge.HeadCommandSessionId);
    bool CurrentStatus(bool active) => status != null && SameSession(status.session_id) && status.active == active &&
        status.sequence > 0 && status.sequence <= bridge.LastHeadCommandSequence &&
        RosFrames.Now - status.stamp >= -.05 && RosFrames.Now - status.stamp < .5;
    bool InactiveReleaseAcknowledgedAfter(int sequenceBoundary)
    {
        if (!CurrentStatus(false) || status.reason != "head_view_inactive") return false;
        foreach (var pair in commands)
        {
            var command = pair.Value;
            if (command.sequence > sequenceBoundary && command.sequence <= status.sequence && !command.active &&
                SameSession(command.session_id))
            {
                evidence.releaseCommand = command;
                return true;
            }
        }
        return false;
    }

    [UnityTearDown]
    public IEnumerator Cleanup()
    {
        Debug.Log("Native head teardown: release commands and stop input dispatch");
        if (root)
        {
            var input = root.GetComponent<DeicticInput>();
            if (input) input.enabled = false;
        }
        if (view) view.SetRobotView(false);
        if (headTracking) headTracking.enabled = false;
        if (bridge && bridge.Ros)
        {
            bridge.PublishRobotHead(false, false, Quaternion.identity);
            yield return new WaitForSecondsRealtime(.4f);
            bridge.Ros.Disconnect();
            UnityEngine.Object.Destroy(bridge.Ros.gameObject);
        }

        // Retire the OVR rig's submitted eye frames before unloading its native
        // session. Destroying all three cameras and unloading in the immediately
        // following frame can leave their last render-thread work in flight.
        Debug.Log("Native head teardown: disable rig rendering before frame retirement");
        if (rigObject)
        {
            foreach (var camera in rigObject.GetComponentsInChildren<Camera>(true))
            {
                camera.enabled = false;
                camera.stereoTargetEye = StereoTargetEyeMask.None;
                camera.targetTexture = null;
            }
            if (rig) rig.enabled = false;
            var manager = rigObject.GetComponent<OVRManager>();
            if (manager) manager.enabled = false;
            rigObject.SetActive(false);
        }
        if (root) root.SetActive(false);
        // Yield normal frames, not WaitForEndOfFrame, which can stall when an
        // editor Game view is unfocused. Give the render thread two frames and
        // a bounded 0.2-second drain while no eye camera submits new work.
        yield return null;
        yield return null;
        yield return new WaitForSecondsRealtime(.2f);
        Debug.Log("Native head teardown: destroy owned rig and wait for callbacks to retire");
        UnityEngine.Object.Destroy(root);
        UnityEngine.Object.Destroy(rigObject);
        UnityEngine.Object.Destroy(settings);
        yield return null;
        yield return null;
        yield return new WaitForSecondsRealtime(.2f);
        Debug.Log("Native head teardown: begin owned OpenXR loader cleanup");
        if (native != null) yield return native.Cleanup();
        Debug.Log("Native head teardown: owned OpenXR loader cleanup complete");
    }
}
