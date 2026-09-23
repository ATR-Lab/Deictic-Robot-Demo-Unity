using System;
using System.Collections;
using System.IO;
using Deictic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

/// <summary>Opt-in scripted headset poses through Unity/ROS to actual Isaac neck/camera feedback.</summary>
public sealed class HeadPoseRosLoopTests
{
    [Serializable] sealed class HeadStatus
    {
        public double stamp;
        public bool active;
        public string reason;
        public string session_id;
        public int sequence;
        public double[] measured, targets;
    }
    GameObject root, eye;
    DeicticSettings settings;
    DeicticBridge bridge;
    DeicticCameraView view;
    HeadStatus status;
    readonly RobotHeadSample sample = new RobotHeadSample();
    Quaternion pose = Quaternion.identity;
    NativeSimulatorFixture native;

    [UnityTest, Category("ROSHeadSimulationIntegration")]
    public IEnumerator HeadRotationMovesNeckAndStereoFeedAndUserViewHolds()
    {
        if (Environment.GetEnvironmentVariable("DEICTIC_HEAD_SIM_INTEGRATION") != "1")
            Assert.Ignore("Opt in only with an isolated Isaac endpoint and no other head publisher.");
        native = new NativeSimulatorFixture();
        yield return native.Initialize();
        root = new GameObject("Head tracking bridge validation");
        eye = new GameObject("Head tracking eye camera");
        eye.tag = "MainCamera";
        eye.transform.position = new Vector3(0, 1.7f, 0);
        var camera = eye.AddComponent<Camera>();
        camera.stereoTargetEye = StereoTargetEyeMask.Both;
        settings = UnityEngine.Object.Instantiate(Resources.Load<DeicticSettings>("DeicticSettings"));
        settings.rosHost = "127.0.0.1";
        settings.connectOnStart = true;
        settings.syntheticScene = true;
        bridge = root.AddComponent<DeicticBridge>();
        bridge.Initialize(settings);
        bridge.Ros.Subscribe<RosMessageTypes.Std.StringMsg>("/k1/head/status", message => status = JsonUtility.FromJson<HeadStatus>(message.data));
        view = root.AddComponent<DeicticCameraView>();
        view.Initialize(eye.transform, bridge);
        float deadline = Time.realtimeSinceStartup + 35;
        while ((!bridge.CanTrackRobotHead || status == null) && Time.realtimeSinceStartup < deadline)
        {
            Publish();
            yield return null;
        }
        Assert.That(bridge.CanTrackRobotHead, Is.True, "A synchronized ROS connection is required");
        Assert.That(status, Is.Not.Null, "Updated Isaac head status is required");
        yield return Phase(.4f);
        Assert.That(CurrentAcknowledgement() && !status.active, Is.True, "User view must acknowledge this publisher's inactive head lease");
        view.ToggleButton.onClick.Invoke();
        pose = Quaternion.Euler(8, -14, 0);
        yield return Converge(14 * Mathf.Deg2Rad, 8 * Mathf.Deg2Rad);
        byte[] first = Pixels();
        SaveFrame("head-left-down.png");
        long before = view.Stream.DisplayedFrames;
        pose = Quaternion.Euler(-6, 14, 20);
        yield return Converge(-14 * Mathf.Deg2Rad, -6 * Mathf.Deg2Rad);
        Assert.That(view.Stream.DisplayedFrames, Is.GreaterThan(before));
        byte[] second = Pixels();
        Assert.That(Convert.ToBase64String(second), Is.Not.EqualTo(Convert.ToBase64String(first)),
            "Distinct measured head poses must supply new camera pixels");
        SaveFrame("head-right-up.png");
        Assert.That(RobotPovRendererFeature.ActiveCamera, Is.SameAs(camera));
        if (Environment.GetEnvironmentVariable("DEICTIC_NATIVE_SIMULATOR") == "1")
            Assert.That(camera.stereoEnabled, Is.True, "The Meta XR Simulator must render the live robot feed in stereo");
        int releaseBoundary = bridge.LastHeadCommandSequence;
        view.ToggleButton.onClick.Invoke();
        yield return Phase(.6f);
        Assert.That(CurrentInactiveAcknowledgementAfter(releaseBoundary), Is.True,
            "Returning to User view must acknowledge this publisher's post-toggle inactive command, not watchdog expiry");
        double[] held = (double[])status.measured.Clone();
        pose = Quaternion.Euler(35, 45, 0);
        yield return Phase(.6f);
        Assert.That(CurrentInactiveAcknowledgementAfter(releaseBoundary), Is.True);
        Assert.That(Math.Abs(status.measured[0] - held[0]), Is.LessThan(.025));
        Assert.That(Math.Abs(status.measured[1] - held[1]), Is.LessThan(.025));
        Debug.Log("Unity head-pose bridge: matched both yaw/pitch targets, fresh stereo changed, User-view hold passed.");
    }
    void Publish()
    {
        if (sample.Step(Time.realtimeSinceStartupAsDouble, view && view.RobotView, true, true,
            bridge != null && bridge.CanTrackRobotHead, pose, Quaternion.identity))
            bridge?.PublishRobotHead(sample.Active, sample.Tracked, sample.Rotation);
    }
    IEnumerator Phase(float seconds)
    {
        float end = Time.realtimeSinceStartup + seconds;
        while (Time.realtimeSinceStartup < end) { Publish(); yield return null; }
    }
    IEnumerator Converge(double yaw, double pitch)
    {
        float end = Time.realtimeSinceStartup + 18;
        bool matches = false;
        while (Time.realtimeSinceStartup < end)
        {
            Publish();
            if (CurrentAcknowledgement() && status.active && status.measured?.Length == 2 && status.targets?.Length == 2 &&
                Math.Abs(status.targets[0] - yaw) < .002 && Math.Abs(status.targets[1] - pitch) < .002 &&
                Math.Abs(status.measured[0] - yaw) < .025 && Math.Abs(status.measured[1] - pitch) < .025)
            { matches = true; break; }
            yield return null;
        }
        Assert.That(matches, Is.True, "Measured Isaac neck did not match the Unity orientation: " + JsonUtility.ToJson(status));
        long frames = view.Stream.DisplayedFrames;
        end = Time.realtimeSinceStartup + 10;
        while ((!view.Stream.HasFreshFrame || view.Stream.DisplayedFrames < frames + 2) && Time.realtimeSinceStartup < end)
        { Publish(); yield return null; }
        Assert.That(view.Stream.HasFreshFrame, Is.True, view.Stream.LastError);
        Assert.That(view.Stream.DisplayedFrames, Is.GreaterThanOrEqualTo(frames + 2));
        Assert.That(view.Stream.IsStereo, Is.True);
    }
    byte[] Pixels() => view.Stream.Texture.GetRawTextureData();
    bool CurrentAcknowledgement() => status != null &&
        Guid.TryParse(status.session_id, out var session) && session == Guid.Parse(bridge.HeadCommandSessionId) &&
        status.sequence > 0 && status.sequence <= bridge.LastHeadCommandSequence &&
        RosFrames.Now - status.stamp >= -.05 && RosFrames.Now - status.stamp < .5;
    bool CurrentInactiveAcknowledgementAfter(int sequenceBoundary) => CurrentAcknowledgement() &&
        !status.active && status.reason == "head_view_inactive" && status.sequence > sequenceBoundary;
    void SaveFrame(string name)
    {
        string directory = Environment.GetEnvironmentVariable("DEICTIC_HEAD_ARTIFACT_DIR");
        if (string.IsNullOrWhiteSpace(directory)) return;
        Directory.CreateDirectory(directory);
        File.WriteAllBytes(Path.Combine(directory, name), view.Stream.Texture.EncodeToPNG());
    }
    [UnityTearDown]
    public IEnumerator Cleanup()
    {
        if (view) view.SetRobotView(false);
        if (bridge && bridge.Ros)
        {
            bridge.PublishRobotHead(false, false, Quaternion.identity);
            yield return new WaitForSecondsRealtime(.4f);
            bridge.Ros.Disconnect();
            UnityEngine.Object.Destroy(bridge.Ros.gameObject);
        }
        UnityEngine.Object.Destroy(root);
        UnityEngine.Object.Destroy(eye);
        UnityEngine.Object.Destroy(settings);
        yield return null;
        if (native != null) yield return native.Cleanup();
    }
}
