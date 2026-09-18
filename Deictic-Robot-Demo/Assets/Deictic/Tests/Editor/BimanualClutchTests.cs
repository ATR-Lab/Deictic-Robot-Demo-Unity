using Deictic;
using NUnit.Framework;
using UnityEngine;

public sealed class BimanualClutchTests
{
    static readonly Vector3 Left = new Vector3(-.2f, 1.2f, .4f);
    static readonly Vector3 Right = new Vector3(.2f, 1.2f, .4f);
    static readonly Vector3 Head = new Vector3(0, 1.7f, 0);
    static bool Step(BimanualClutch c, double time, bool l, bool r, bool ready = true,
        bool tracked = true, bool focus = true, Vector3? left = null, Vector3? right = null, Quaternion? yaw = null,
        Vector3? head = null, Quaternion? leftRotation = null, Quaternion? rightRotation = null)
        => c.Step(time, l, r, tracked, tracked, tracked, focus, ready,
            head ?? Head, yaw ?? Quaternion.identity,
            left ?? Left, leftRotation ?? Quaternion.identity, right ?? Right, rightRotation ?? Quaternion.identity);
    static BimanualClutch Armed()
    {
        var c = new BimanualClutch();
        Step(c, 0, false, false);
        return c;
    }
    [Test]
    public void StartupRequiresFullReleaseAndEitherReleaseStopsBoth()
    {
        var c = new BimanualClutch();
        Step(c, 0, true, true);
        Assert.That(c.Active, Is.False);
        Step(c, .1, false, true);
        Assert.That(c.AwaitingRelease, Is.True);
        Step(c, .2, false, false);
        Step(c, .3, true, true);
        Assert.That(c.Active, Is.True);
        Assert.That(Step(c, .301, false, true), Is.True, "Release sends an immediate false packet, outside cadence");
        Assert.That(c.Active, Is.False);
        Step(c, .4, true, true);
        Assert.That(c.Active, Is.False, "Regripping only one trigger must not resume");
        Step(c, .5, false, false);
        Step(c, .6, true, true);
        Assert.That(c.Active, Is.True);
    }
    [TestCase("tracking")]
    [TestCase("focus")]
    [TestCase("backend")]
    [TestCase("recenter")]
    public void FaultsLatchUntilTrackedFullRelease(string fault)
    {
        var c = Armed();
        Step(c, .1, true, true);
        if (fault == "recenter") c.Interrupt("Recenter");
        else Step(c, .2, true, true, fault != "backend", fault != "tracking", fault != "focus");
        Assert.That(c.Active, Is.False);
        Assert.That(c.AwaitingRelease, Is.True);
        Step(c, .3, true, true);
        Assert.That(c.Active, Is.False, "Recovery with held triggers is never an implicit start");
        Step(c, .4, false, false, tracked: false);
        Assert.That(c.AwaitingRelease, Is.True, "Untracked false trigger readings are not a physical release");
        Step(c, .5, false, false);
        Step(c, .6, true, true);
        Assert.That(c.Active, Is.True);
    }
    [Test]
    public void FalseHeartbeatArmsWithoutBackendReadinessAndUpdatesAreLatestOnly20Hz()
    {
        var c = new BimanualClutch();
        Assert.That(Step(c, 0, false, false, ready: false), Is.True);
        Assert.That(c.Active, Is.False);
        Assert.That(c.AwaitingRelease, Is.False);
        Assert.That(Step(c, .02, false, false, ready: false), Is.False);
        Assert.That(Step(c, .05, false, false, ready: false), Is.True);
        Step(c, .1, true, true);
        Vector3 latest = Left + new Vector3(.01f, .02f, .03f);
        Assert.That(Step(c, .11, true, true, left: latest), Is.False);
        Assert.That(Step(c, .15 + 1e-8, true, true, left: latest * 1.01f), Is.True);
        Assert.That(Step(c, 4, true, true, left: latest), Is.True, "One latest packet after a stall, never a catch-up loop");
        Assert.That(Step(c, 4.001, true, true), Is.False);
    }
    [Test]
    public void HeadAndControllersTranslatingTogetherDoNotMoveRelativeTargets()
    {
        var c = Armed();
        Step(c, .1, true, true, yaw: Quaternion.Euler(0, 40, 0));
        Vector3 startLeft = c.LeftPosition, startRight = c.RightPosition;
        Vector3 translation = new Vector3(3, -.2f, -4);
        Step(c, .2, true, true, yaw: Quaternion.Euler(0, 40, 0), head: Head + translation,
            left: Left + translation, right: Right + translation);
        Assert.That(Vector3.Distance(c.LeftPosition, startLeft), Is.LessThan(1e-6));
        Assert.That(Vector3.Distance(c.RightPosition, startRight), Is.LessThan(1e-6));
        Assert.That(c.Active, Is.True);
    }
    [Test]
    public void CurrentHeadYawChangesFrameWhilePitchAndRollAreIgnored()
    {
        var c = Armed();
        Step(c, .1, true, true);
        Step(c, .2, true, true, yaw: Quaternion.Euler(25, 90, 35));
        // Inverse current +90-degree heading maps world forward to local left.
        Assert.That(Vector3.Distance(c.LeftPosition, new Vector3(-.4f, -.5f, -.2f)), Is.LessThan(1e-5));
        Assert.That(Vector3.Distance(c.RightPosition, new Vector3(-.4f, -.5f, .2f)), Is.LessThan(1e-5));
        Assert.That(Quaternion.Angle(c.LeftRotation, Quaternion.Euler(0, -90, 0)), Is.LessThan(.01f));
        Vector3 expected = c.LeftPosition;
        Step(c, .3, true, true, yaw: Quaternion.Euler(-30, 90, -20));
        Assert.That(Vector3.Distance(c.LeftPosition, expected), Is.LessThan(1e-5));
    }
    [Test]
    public void IndependentControllerRotationsAndPositionsHaveConsistentFluSigns()
    {
        var c = Armed();
        Step(c, .1, true, true, leftRotation: Quaternion.Euler(0, 90, 0), rightRotation: Quaternion.Euler(90, 0, 0));
        double[] left = DeicticBridge.TeleopPosition(c.LeftPosition);
        Assert.That(left, Is.EqualTo(new[] { .4, .2, -.5 }).Within(1e-6));
        double[] lq = DeicticBridge.TeleopRotation(c.LeftRotation);
        double[] rq = DeicticBridge.TeleopRotation(c.RightRotation);
        double s = System.Math.Sqrt(.5);
        // q and -q represent the same rotation. ROSGeometry deliberately returns
        // negative w, so verify the rotation rather than one quaternion spelling.
        var leftRos = new Quaternion((float)lq[0], (float)lq[1], (float)lq[2], (float)lq[3]);
        var rightRos = new Quaternion((float)rq[0], (float)rq[1], (float)rq[2], (float)rq[3]);
        Assert.That(Mathf.Abs(Quaternion.Dot(leftRos, new Quaternion(0, 0, (float)-s, (float)s))), Is.EqualTo(1).Within(1e-6));
        Assert.That(Mathf.Abs(Quaternion.Dot(rightRos, new Quaternion(0, (float)s, 0, (float)s))), Is.EqualTo(1).Within(1e-6));
        // Here Vector3 coordinates denote numeric ROS x/y/z: forward (+x)
        // turns right (-y) under Unity yaw+90, and down (-z) under pitch+90.
        Assert.That(Vector3.Distance(leftRos * Vector3.right, Vector3.down), Is.LessThan(1e-6));
        Assert.That(Vector3.Distance(rightRos * Vector3.right, Vector3.back), Is.LessThan(1e-6));
        // The wire DTO must preserve orientations and identify the new convention.
        var packet = new DeicticBridge.TeleopInput { left_position = left, right_position = DeicticBridge.TeleopPosition(c.RightPosition),
            left_rotation = lq, right_rotation = rq };
        var roundTrip = JsonUtility.FromJson<DeicticBridge.TeleopInput>(JsonUtility.ToJson(packet));
        Assert.That(roundTrip.schema_version, Is.EqualTo(2));
        Assert.That(roundTrip.frame_id, Is.EqualTo("teleop_head"));
        Assert.That(roundTrip.left_rotation, Is.EqualTo(lq));
        Assert.That(roundTrip.right_rotation, Is.EqualTo(rq));
    }
    [Test]
    public void NearVerticalHeadHeadingStopsActiveClutchUntilTrackedRelease()
    {
        var c = Armed();
        Step(c, .1, true, true);
        Step(c, .2, true, true, yaw: Quaternion.Euler(89, 40, 0));
        Assert.That(c.Active, Is.False);
        Assert.That(c.PoseValid, Is.False);
        Step(c, .3, true, true);
        Assert.That(c.Active, Is.False);
        Step(c, .4, false, false);
        Step(c, .5, true, true);
        Assert.That(c.Active, Is.True);
    }
    [TestCase("head")]
    [TestCase("left")]
    [TestCase("right")]
    public void InvalidQuaternionStopsBothArms(string source)
    {
        var c = Armed();
        Step(c, .1, true, true);
        Quaternion bad = new Quaternion(0, float.NaN, 0, 0);
        Step(c, .2, true, true, yaw: source == "head" ? bad : Quaternion.identity,
            leftRotation: source == "left" ? bad : Quaternion.identity,
            rightRotation: source == "right" ? bad : Quaternion.identity);
        Assert.That(c.Active, Is.False);
        Assert.That(c.PoseValid, Is.False);
        Assert.That(c.LeftRotation, Is.EqualTo(Quaternion.identity));
        Assert.That(c.RightRotation, Is.EqualTo(Quaternion.identity));
        Assert.That(BimanualClutch.TryNormalize(new Quaternion(0, 0, 0, 0), out _), Is.False);
        Assert.That(BimanualClutch.TryNormalize(new Quaternion(0, 0, 0, 2), out _), Is.False);
        Assert.Throws<System.ArgumentException>(() => DeicticBridge.TeleopRotation(bad));
    }
    [Test]
    public void InvalidPositionsCannotProduceActiveCommand()
    {
        var c = Armed();
        Step(c, .1, true, true, left: new Vector3(float.NaN, 0, 0));
        Assert.That(c.Active, Is.False);
        Assert.That(c.AwaitingRelease, Is.True);
        Assert.That(c.LeftPosition, Is.EqualTo(Vector3.zero));
    }
    [Test]
    public void SyntheticSceneRecenterAndTrackingOriginMotionInvalidateClutch()
    {
        var host = new GameObject("Synthetic origin regression");
        var origin = new GameObject("Tracking space");
        try
        {
            var input = host.AddComponent<DeicticInput>();
            var teleop = host.AddComponent<BimanualTeleop>();
            input.synthetic = true; input.teleop = teleop;
            var flags = System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic;
            typeof(DeicticInput).GetField("trackingSpace", flags).SetValue(input, origin.transform);
            typeof(DeicticInput).GetMethod("RememberTrackingSpace", flags).Invoke(input, null);
            Step(teleop.Clutch, 0, false, false);
            Step(teleop.Clutch, .1, true, true);
            typeof(DeicticInput).GetMethod("OnRecentered", flags).Invoke(input, null);
            Assert.That(teleop.Clutch.Active, Is.False, "Synthetic scene must still respond to native recenter");
            Step(teleop.Clutch, .2, false, false);
            Step(teleop.Clutch, .3, true, true);
            origin.transform.position = Vector3.forward;
            typeof(DeicticInput).GetMethod("CheckTrackingOrigin", flags).Invoke(input, null);
            Assert.That(teleop.Clutch.Active, Is.False);
            Assert.That(teleop.Clutch.AwaitingRelease, Is.True);
        }
        finally { Object.DestroyImmediate(host); Object.DestroyImmediate(origin); }
    }
    [Test]
    public void SingleTriggerIntentCannotMigrateBetweenUiAndTargetsOrSurviveChord()
    {
        var intent = new TriggerReleaseIntent();
        intent.Begin(true, false, Vector3.zero, Vector3.up);
        Assert.That(intent.Release(false, true, Vector3.zero, .04f, out _, out _), Is.EqualTo(TriggerReleaseIntent.Kind.None));
        intent.Begin(false, true, Vector3.one, Vector3.up);
        Assert.That(intent.Release(true, false, Vector3.one, .04f, out _, out _), Is.EqualTo(TriggerReleaseIntent.Kind.None));
        intent.Begin(false, true, Vector3.one, Vector3.up);
        intent.Cancel(); // Left trigger joined, tracking failed, cancel, or view changed.
        Assert.That(intent.Release(false, true, Vector3.one, .04f, out _, out _), Is.EqualTo(TriggerReleaseIntent.Kind.None));
        intent.Begin(false, true, Vector3.one, Vector3.up);
        Assert.That(intent.Release(false, true, Vector3.one + Vector3.right * .01f, .04f, out var point, out _), Is.EqualTo(TriggerReleaseIntent.Kind.Target));
        Assert.That(point, Is.EqualTo(Vector3.one), "Send the reviewed press-time target, not another surface under release");
    }
    [Test]
    public void ReadyHeartbeatCannotHideLastBackendFaultAndProtocolMustMatch()
    {
        var host = new GameObject("Teleop backend status");
        var settings = ScriptableObject.CreateInstance<DeicticSettings>();
        try
        {
            var bridge = host.AddComponent<DeicticBridge>();
            bridge.settings = settings;
            var teleop = host.AddComponent<BimanualTeleop>(); teleop.bridge = bridge;
            var flags = System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic;
            var onStatus = typeof(DeicticBridge).GetMethod("OnStatus", flags);
            double stamp = RosFrames.Now;
            var status = new AlignmentStatus { stamp = stamp, teleop_ready = true, teleop_protocol_version = 1,
                teleop_last_fault = "unreachable_target", teleop_reason = "clutch_released" };
            onStatus.Invoke(bridge, new object[] { new RosMessageTypes.Std.StringMsg(JsonUtility.ToJson(status)) });
            Assert.That(bridge.TeleopProtocolCompatible, Is.False);
            Assert.That(bridge.CanTeleoperate, Is.False);
            Assert.That(bridge.LastTeleopFault, Is.EqualTo("unreachable_target"));
            status.stamp += .001; status.teleop_protocol_version = 2; status.teleop_last_fault = "";
            onStatus.Invoke(bridge, new object[] { new RosMessageTypes.Std.StringMsg(JsonUtility.ToJson(status)) });
            Assert.That(bridge.TeleopProtocolCompatible, Is.True);
            Assert.That(teleop.Feedback, Does.Contain("Last arm stop: unreachable_target"));
            status.stamp = stamp; status.teleop_last_fault = "outdated_fault";
            onStatus.Invoke(bridge, new object[] { new RosMessageTypes.Std.StringMsg(JsonUtility.ToJson(status)) });
            Assert.That(bridge.LastTeleopFault, Is.EqualTo("unreachable_target"), "Replay status cannot overwrite the displayed fault");
        }
        finally { Object.DestroyImmediate(host); Object.DestroyImmediate(settings); }
    }
    [Test]
    public void ApplicationFocusLossStopsClutchAndCancelsPendingAction()
    {
        var host = new GameObject("Teleop focus lifecycle");
        try
        {
            var teleop = host.AddComponent<BimanualTeleop>();
            int interrupts = 0; teleop.Interrupted += () => interrupts++;
            Step(teleop.Clutch, 0, false, false);
            Step(teleop.Clutch, .1, true, true);
            var flags = System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic;
            typeof(BimanualTeleop).GetMethod("OnApplicationFocus", flags).Invoke(teleop, new object[] { false });
            Assert.That(teleop.Clutch.Active, Is.False);
            Assert.That(teleop.Clutch.AwaitingRelease, Is.True);
            Assert.That(interrupts, Is.EqualTo(1));
            Step(teleop.Clutch, .2, true, true);
            Assert.That(teleop.Clutch.Active, Is.False, "Focus recovery with held triggers cannot restart");
        }
        finally { Object.DestroyImmediate(host); }
    }
    [Test]
    public void DisabledTeleopDoesNotReclaimInputOnBridgeCancel()
    {
        var host = new GameObject("Disabled teleop cancellation");
        try
        {
            var bridge = host.AddComponent<DeicticBridge>();
            var teleop = host.AddComponent<BimanualTeleop>();
            teleop.bridge = bridge;
            var flags = System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic;
            typeof(BimanualTeleop).GetMethod("Start", flags).Invoke(teleop, null);
            bridge.SetTeleopIntent(true);
            teleop.enabled = false;
            // EditMode manually enters Start above; match that lifecycle here,
            // since ordinary MonoBehaviours do not receive Play callbacks.
            typeof(BimanualTeleop).GetMethod("OnDisable", flags).Invoke(teleop, null);
            Assert.That(bridge.TeleopControlsBusy, Is.False);
            bridge.Cancel();
            Assert.That(bridge.TeleopControlsBusy, Is.False,
                "A disabled subscriber cannot lock deictic input after its disable cleanup");
        }
        finally { Object.DestroyImmediate(host); }
    }
    [Test]
    public void BridgeBlocksExplicitAndVoiceEntryPointsWhileClutchIntentIsPresent()
    {
        var host = new GameObject("Teleop exclusion");
        var settings = ScriptableObject.CreateInstance<DeicticSettings>();
        try
        {
            var bridge = host.AddComponent<DeicticBridge>();
            bridge.settings = settings;
            bridge.SetTeleopIntent(true);
            Assert.That(bridge.Commit(Vector3.one, Vector3.up), Is.False);
            Assert.That(bridge.Feedback, Does.Contain("clutch"));
            Assert.That(bridge.Execute(), Is.False);
            Assert.That(bridge.Feedback, Does.Contain("clutch"));
        }
        finally { Object.DestroyImmediate(host); Object.DestroyImmediate(settings); }
    }
}
