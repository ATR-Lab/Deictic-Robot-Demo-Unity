using System;
using System.Collections;
using System.Collections.Generic;
using Deictic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

/// <summary>Scripted controller samples through the real Unity bridge to Isaac.
/// This is deliberately not evidence of native OVR trigger delivery.</summary>
public sealed class BimanualRosLoopTests
{
    static readonly string[][] Names = {
        new[] { "aaleft_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_elbow_pitch_joint", "left_elbow_yaw_joint" },
        new[] { "aaright_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_elbow_pitch_joint", "right_elbow_yaw_joint" }
    };
    GameObject root, eye;
    DeicticSettings settings;
    DeicticBridge bridge;
    DeicticCameraView view;
    Camera camera;
    readonly BimanualClutch clutch = new BimanualClutch();
    double[][] joints;
    double jointStamp;
    [Serializable] sealed class BackendCommand { public bool active; public double[] positions; public string session_id; public int sequence; }
    [Serializable] sealed class RelayStatus
    {
        public double stamp;
        public bool active, ready;
        public string owner_session_id, reason;
        public int last_sequence;
    }
    RelayStatus relayStatus;
    string backendSession;
    int lastActiveSequence;
    double maximumCameraAge;
    readonly List<double[]> activeCommands = new List<double[]>();
    NativeSimulatorFixture native;
    Vector3 left, right;
    Vector3 head = new Vector3(0, 1.7f, 0);
    Quaternion headRotation = Quaternion.identity, bodyRotation = Quaternion.identity;
    Quaternion leftRotation = Quaternion.identity, rightRotation = Quaternion.identity;
    const float TranslationScale = .558833641f;

    [UnityTest, Category("ROSBimanualSimulationIntegration")]
    public IEnumerator InitialAbsoluteAcquisitionIndependentArmsAndReclutchReachCurrentPose()
    {
        if (Environment.GetEnvironmentVariable("DEICTIC_BIMANUAL_SIM_INTEGRATION") != "1")
            Assert.Ignore("Opt in only with an isolated Isaac-only endpoint at localhost:10000; stop other Unity publishers.");
        native = new NativeSimulatorFixture();
        yield return native.Initialize();
        // Inverse anatomical mapping of the documented initial Isaac arm poses:
        // L [0,0,0,0], R [0,.5,0,0]. These body-relative ROS poses are derived
        // from K1 FK and the fixed shoulder/tool offsets, not clutch anchors.
        SetFixture(true, new[] { -.0455264, .77754456, -.22415746, 0.0, 0.0, .70710678, .70710678 });
        SetFixture(false, new[] { -.0455264, -.71929068, -.45229812, .17494102, .17494102, -.68512454, .68512454 });
        settings = UnityEngine.Object.Instantiate(Resources.Load<DeicticSettings>("DeicticSettings"));
        settings.rosHost = "127.0.0.1";
        settings.connectOnStart = true;
        settings.syntheticScene = true;
        settings.robotCameraTopic = RosCameraStream.DefaultTopic;
        settings.robotCameraStereo = true;
        root = new GameObject("Scripted bimanual simulation wire test");
        eye = new GameObject("Bimanual live robot POV eye");
        eye.tag = "MainCamera";
        eye.transform.position = head;
        camera = eye.AddComponent<Camera>();
        camera.stereoTargetEye = StereoTargetEyeMask.Both;
        bridge = root.AddComponent<DeicticBridge>();
        bridge.JointStateReceived += message => {
            var sample = new Dictionary<string, double>();
            for (int i = 0; i < Math.Min(message.name.Length, message.position.Length); i++)
                sample[message.name[i]] = message.position[i];
            var next = new[] { new double[4], new double[4] };
            for (int side = 0; side < 2; side++)
                for (int i = 0; i < 4; i++)
                    if (!sample.TryGetValue(Names[side][i], out next[side][i])) return;
            joints = next;
            jointStamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9;
        };
        bridge.Initialize(settings);
        view = root.AddComponent<DeicticCameraView>();
        view.Initialize(eye.transform, bridge);
        view.SetRobotView(true);
        bridge.Ros.Subscribe<RosMessageTypes.Std.StringMsg>("/k1/teleop/command", message => {
            var value = JsonUtility.FromJson<BackendCommand>(message.data);
            if (value.active && value.positions != null && value.positions.Length == 8)
            {
                activeCommands.Add(value.positions);
                backendSession = value.session_id;
                lastActiveSequence = value.sequence;
            }
        });
        bridge.Ros.Subscribe<RosMessageTypes.Std.StringMsg>("/k1/teleop/relay_status",
            message => relayStatus = JsonUtility.FromJson<RelayStatus>(message.data));
        float deadline = Time.realtimeSinceStartup + 30;
        while ((!bridge.CanTeleoperate || joints == null || !view.Stream.HasFreshFrame) && Time.realtimeSinceStartup < deadline)
        {
            Sample(false, false);
            yield return null;
        }
        Assert.That(bridge.CanTeleoperate, Is.True, bridge.Status?.teleop_reason ?? bridge.Feedback);
        Assert.That(joints, Is.Not.Null, "Both four-joint arms must have actual feedback");
        Assert.That(bridge.Status.mode, Is.EqualTo("markerless"), "Use the declared Isaac markerless fixture");
        Assert.That(bridge.Status.teleop_protocol_version, Is.EqualTo(3));
        Assert.That(view.Stream.IsCompressed, Is.True, "Exercise the actual JPEG display transport while controlling the arms");
        Assert.That(view.Stream.HasFreshFrame, Is.True, view.Stream.Status);
        Assert.That(RobotPovRendererFeature.ActiveCamera, Is.SameAs(camera));
        if (Environment.GetEnvironmentVariable("DEICTIC_NATIVE_SIMULATOR") == "1")
            Assert.That(camera.stereoEnabled, Is.True, "Native simulator must render the live robot POV in both eyes");
        long initialCameraFrames = view.Stream.DisplayedFrames;
        yield return Phase(.4f, false, false);
        var initial = Snapshot();
        var fixtureRest = new[] { new[] { 0.0, 0.0, 0.0, 0.0 }, new[] { 0.0, .5, 0.0, 0.0 } };
        Assert.That(MaxDelta(fixtureRest, initial, 0), Is.LessThan(.12), "Restart Isaac in its initial fixture before this test");
        Assert.That(MaxDelta(fixtureRest, initial, 1), Is.LessThan(.12), "Restart Isaac in its initial fixture before this test");
        // Both stationary hands already specify a new absolute target when
        // the triggers first engage. No controller delta is needed to move.
        left += new Vector3(.005f, -.01f, .025f) / TranslationScale;
        right += new Vector3(-.005f, .01f, .025f) / TranslationScale;
        activeCommands.Clear();
        yield return Phase(8f, true, true);
        Assert.That(clutch.Active, Is.True, clutch.Reason);
        Assert.That(bridge.Status.teleop_active, Is.True, bridge.Status.teleop_reason);
        Assert.That(activeCommands.Count, Is.GreaterThan(3), "Observe actual backend commands during acquisition");
        Assert.That(MaxDelta(initial, joints, 0), Is.GreaterThan(.015), "Initial absolute left pose was treated as a zero-motion clutch anchor");
        Assert.That(MaxDelta(initial, joints, 1), Is.GreaterThan(.015), "Initial absolute right pose was treated as a zero-motion clutch anchor");
        var acquired = Snapshot();
        var acquiredLeft = clutch.LeftPosition;
        var acquiredRight = clutch.RightPosition;
        var acquiredLeftRotation = clutch.LeftRotation;
        var acquiredRightRotation = clutch.RightRotation;
        activeCommands.Clear();
        yield return Phase(.4f, true, true);
        AssertSettledCommands();

        // Looking around must not change either body-relative arm target.
        headRotation = Quaternion.Euler(85, 60, 25);
        yield return Phase(.4f, true, true);
        AssertSameTargets(acquiredLeft, acquiredRight, acquiredLeftRotation, acquiredRightRotation);
        AssertSettledCommands();

        // Moving the tracked body as a whole must leave body-relative wrist targets unchanged.
        var bodyTranslation = new Vector3(.7f, .1f, -.5f);
        int commandsBeforeTranslation = activeCommands.Count;
        head += bodyTranslation; left += bodyTranslation; right += bodyTranslation;
        yield return Phase(.3f, true, true);
        Assert.That(activeCommands.Count - commandsBeforeTranslation, Is.GreaterThan(2), "Translation must retain live command publication");
        AssertSameTargets(acquiredLeft, acquiredRight, acquiredLeftRotation, acquiredRightRotation);
        AssertSettledCommands();
        var bodyYaw = Quaternion.Euler(0, 60, 0);
        int commandsBeforeYaw = activeCommands.Count;
        left = head + bodyYaw * (left - head);
        right = head + bodyYaw * (right - head);
        headRotation = bodyYaw * headRotation;
        bodyRotation = bodyYaw * bodyRotation;
        leftRotation = bodyYaw * leftRotation; rightRotation = bodyYaw * rightRotation;
        yield return Phase(.3f, true, true);
        Assert.That(activeCommands.Count - commandsBeforeYaw, Is.GreaterThan(2), "Yaw must retain live command publication");
        AssertSameTargets(acquiredLeft, acquiredRight, acquiredLeftRotation, acquiredRightRotation);
        AssertSettledCommands();
        Assert.That(MaxDelta(acquired, joints, 0), Is.LessThan(.03), "Looking and moving the rig changed the acquired left target");
        Assert.That(MaxDelta(acquired, joints, 1), Is.LessThan(.03), "Looking and moving the rig changed the acquired right target");

        // Inward motion exposes the straight-arm singularity missed by forward-only tests.
        left += bodyRotation * Vector3.right * (.015f / TranslationScale);
        yield return Phase(8f, true, true);
        Assert.That(MaxDelta(acquired, joints, 0), Is.GreaterThan(.015), "Left hand did not move the measured left arm");
        Assert.That(MaxDelta(acquired, joints, 1), Is.LessThan(.03), "Left-only input moved the other arm");
        var afterLeft = Snapshot();
        right += bodyRotation * Vector3.left * (.015f / TranslationScale);
        yield return Phase(8f, true, true);
        Assert.That(MaxDelta(afterLeft, joints, 1), Is.GreaterThan(.015), "Right hand did not move the measured right arm");
        Assert.That(MaxDelta(afterLeft, joints, 0), Is.LessThan(.03), "Right-only input moved the other arm");
        Assert.That(bridge.Status.teleop_active, Is.True, bridge.Status.teleop_reason);
        Assert.That(RosFrames.Now - jointStamp, Is.InRange(-.05, .5), "Final feedback must be fresh");
        Debug.Log($"Scripted Unity→ROS→Isaac joint motion: left={MaxDelta(initial, joints, 0):F5}, right={MaxDelta(initial, joints, 1):F5} rad");

        yield return Phase(.4f, false, true);
        Assert.That(clutch.Active, Is.False);
        Assert.That(bridge.Status.teleop_active, Is.False, "Release either trigger must hold both arms");
        var held = Snapshot();
        left -= bodyRotation * Vector3.forward * (.015f / TranslationScale);
        right -= bodyRotation * Vector3.forward * (.015f / TranslationScale);
        yield return Phase(.7f, true, true);
        Assert.That(clutch.Active, Is.False, "Repressing one trigger without full release must not rearm");
        Assert.That(MaxDelta(held, joints, 0), Is.LessThan(.03));
        Assert.That(MaxDelta(held, joints, 1), Is.LessThan(.03));
        yield return Phase(.4f, false, false);
        activeCommands.Clear();
        yield return Phase(8f, true, true);
        Assert.That(clutch.Active, Is.True, clutch.Reason);
        Assert.That(bridge.Status.teleop_active, Is.True, bridge.Status.teleop_reason);
        Assert.That(activeCommands.Count, Is.GreaterThan(3));
        Assert.That(MaxDelta(held, joints, 0), Is.GreaterThan(.015), "Reclutch must acquire the current absolute left pose");
        Assert.That(MaxDelta(held, joints, 1), Is.GreaterThan(.015), "Reclutch must acquire the current absolute right pose");
        yield return Phase(.3f, false, false);

        // Reacquire once more with stationary current targets while the camera
        // continues sharing the ROS connection. This catches a second-use
        // readiness/input latch without relying on another controller movement.
        var reacquired = Snapshot();
        int commandBoundary = lastActiveSequence;
        activeCommands.Clear();
        yield return Phase(8f, true, true);
        Assert.That(clutch.Active, Is.True, clutch.Reason);
        Assert.That(bridge.Status.teleop_active, Is.True, bridge.Status.teleop_reason);
        Assert.That(activeCommands.Count, Is.GreaterThan(3));
        Assert.That(relayStatus, Is.Not.Null, "The real arm relay must acknowledge reacquisition");
        Assert.That(relayStatus.active && relayStatus.ready, Is.True, relayStatus.reason);
        Assert.That(relayStatus.owner_session_id, Is.EqualTo(backendSession));
        Assert.That(relayStatus.last_sequence, Is.GreaterThan(commandBoundary));
        Assert.That(RosFrames.Now - relayStatus.stamp, Is.InRange(-.05, .30), "Reclutch acknowledgement must be fresh");
        Assert.That(MaxDelta(reacquired, joints, 0), Is.LessThan(.03), "Stationary reacquisition changed the settled left goal");
        Assert.That(MaxDelta(reacquired, joints, 1), Is.LessThan(.03), "Stationary reacquisition changed the settled right goal");
        yield return Phase(.3f, false, false);
        Debug.Log($"Bimanual control with compressed Robot POV: {view.Stream.DisplayedFrames - initialCameraFrames} new frames, maximum age {maximumCameraAge:F3}s, three acquisitions acknowledged.");
    }

    void Sample(bool leftHeld, bool rightHeld)
    {
        if (clutch.Step(Time.realtimeSinceStartupAsDouble, leftHeld, rightHeld, true, true,
            true, true, bridge.CanTeleoperate, head, headRotation, bodyRotation,
            left, leftRotation, right, rightRotation))
            bridge.PublishTeleop(clutch.Active, true, true, clutch.LeftPosition, clutch.LeftRotation,
                clutch.RightPosition, clutch.RightRotation);
    }
    IEnumerator Phase(float seconds, bool leftHeld, bool rightHeld)
    {
        long firstFrame = view.Stream.DisplayedFrames;
        float until = Time.realtimeSinceStartup + seconds;
        while (Time.realtimeSinceStartup < until)
        {
            Sample(leftHeld, rightHeld);
            if (!view.Stream.HasFreshFrame) Assert.Fail("Robot POV became stale during arm control: " + view.Stream.Status);
            maximumCameraAge = Math.Max(maximumCameraAge, view.Stream.FrameAge);
            yield return null;
        }
        if (seconds >= 1)
            Assert.That(view.Stream.DisplayedFrames, Is.GreaterThan(firstFrame), "Live compressed video must advance during each arm phase");
    }
    double[][] Snapshot() => new[] { (double[])joints[0].Clone(), (double[])joints[1].Clone() };
    void SetFixture(bool isLeft, double[] rosBodyPose)
    {
        var pose = RosFrames.Matrix(rosBodyPose);
        Vector3 position = head + bodyRotation * (Vector3)pose.GetColumn(3);
        Quaternion orientation = bodyRotation * pose.rotation;
        if (isLeft) { left = position; leftRotation = orientation; }
        else { right = position; rightRotation = orientation; }
    }
    void AssertSameTargets(Vector3 leftPosition, Vector3 rightPosition, Quaternion leftOrientation, Quaternion rightOrientation)
    {
        Assert.That(Vector3.Distance(clutch.LeftPosition, leftPosition), Is.LessThan(1e-5));
        Assert.That(Vector3.Distance(clutch.RightPosition, rightPosition), Is.LessThan(1e-5));
        Assert.That(Quaternion.Angle(clutch.LeftRotation, leftOrientation), Is.LessThan(.01f));
        Assert.That(Quaternion.Angle(clutch.RightRotation, rightOrientation), Is.LessThan(.01f));
    }
    void AssertSettledCommands()
    {
        Assert.That(activeCommands.Count, Is.GreaterThan(3), "Observe actual backend targets after acquisition settles");
        foreach (var command in activeCommands)
            for (int i = 0; i < 8; i++)
                Assert.That(Math.Abs(command[i] - activeCommands[0][i]), Is.LessThan(.02),
                    "The bounded servo must stay near the settled target while controllers remain stationary");
    }
    static double MaxDelta(double[][] a, double[][] b, int side)
    {
        double result = 0;
        for (int i = 0; i < 4; i++) result = Math.Max(result, Math.Abs(a[side][i] - b[side][i]));
        return result;
    }
    [UnityTearDown]
    public IEnumerator Cleanup()
    {
        if (bridge != null && bridge.Ros != null)
        {
            bridge.PublishTeleop(false, true, true, Vector3.zero, Quaternion.identity, Vector3.zero, Quaternion.identity);
            bridge.Cancel();
            yield return new WaitForSecondsRealtime(.3f);
            if (view && view.Stream) view.Stream.enabled = false;
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
