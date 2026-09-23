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
        Vector3? head = null, Quaternion? leftRotation = null, Quaternion? rightRotation = null,
        Quaternion? bodyRotation = null)
        => c.Step(time, l, r, tracked, tracked, tracked, focus, ready,
            head ?? Head, yaw ?? Quaternion.identity, bodyRotation ?? Quaternion.identity,
            left ?? Left, leftRotation ?? Quaternion.identity, right ?? Right, rightRotation ?? Quaternion.identity);
    static BimanualClutch Armed()
    {
        var c = new BimanualClutch();
        Step(c, 0, false, false);
        return c;
    }

    sealed class AnalogGesture
    {
        public readonly BimanualClutch clutch = new BimanualClutch();
        public readonly IndexTriggerState left = new IndexTriggerState(), right = new IndexTriggerState();
        readonly TriggerReleaseIntent intent = new TriggerReleaseIntent();
        bool rightWasHeld;
        public int clicks;
        public bool hovered;

        public bool Sample(double time, float l, float r, bool ready = true, bool tracked = true,
            bool focus = true, bool overToggle = true)
        {
            left.Sample(l, true); right.Sample(r, true);
            bool publish = clutch.Step(time, left.Held, right.Held, tracked && left.Valid,
                tracked && right.Valid, tracked, focus, ready, Head, Quaternion.identity, Quaternion.identity,
                Left, Quaternion.identity, Right, Quaternion.identity, left.FullyReleased, right.FullyReleased);
            bool pressed = right.Held && !rightWasHeld, released = !right.Held && rightWasHeld;
            rightWasHeld = right.Held;
            // Same action ownership as DeicticInput: the clutch runs before
            // pointer hover and the deferred right-trigger click is cancelled.
            if (clutch.SuppressActions || clutch.AwaitingRelease) intent.Cancel();
            hovered = !clutch.SuppressActions && overToggle;
            if (!clutch.SuppressActions)
            {
                if (pressed) intent.Begin(overToggle, false, Vector3.zero, Vector3.up);
                if (released && intent.Release(overToggle, false, Vector3.zero, .04f, out _, out _) ==
                    TriggerReleaseIntent.Kind.CameraToggle) clicks++;
            }
            return publish;
        }
    }

    [Test]
    public void RepeatedChordsRearmOnTheSameFullReleaseFrameWithoutLeakingUi()
    {
        var input = new AnalogGesture();
        input.Sample(0, 0, 0);
        for (int cycle = 0; cycle < 20; cycle++)
        {
            double time = .001 + cycle * .02;
            Assert.That(input.Sample(time, 1, 1), Is.True);
            Assert.That(input.clutch.Active, Is.True, $"Clutch {cycle + 1} must activate");
            Assert.That(input.hovered, Is.False);
            Assert.That(input.Sample(time + .01, 0, 0), Is.True, "A quick full release must publish outside the 20Hz cadence");
            Assert.That(input.clutch.Active, Is.False);
            Assert.That(input.clutch.AwaitingRelease, Is.False, "The observed release must not be discarded by Interrupt");
            Assert.That(input.hovered, Is.False, "The release frame still belongs to the arm gesture");
        }
        Assert.That(input.clicks, Is.Zero);
    }

    [TestCase(true)]
    [TestCase(false)]
    public void StaggeredReleaseRequiresBothTriggersToReleaseBeforeNextChord(bool leftFirst)
    {
        var input = new AnalogGesture();
        input.Sample(0, 0, 0); input.Sample(.1, 1, 1);
        input.Sample(.101, leftFirst ? 0 : 1, leftFirst ? 1 : 0);
        Assert.That(input.clutch.Active, Is.False);
        Assert.That(input.clutch.AwaitingRelease, Is.True);
        Assert.That(input.hovered, Is.False);
        input.Sample(.102, 1, 1);
        Assert.That(input.clutch.Active, Is.False, "Regripping only one released trigger must not resume");
        Assert.That(input.Sample(.103, 0, 0), Is.True, "The rearm release is published even inside the heartbeat period");
        input.Sample(.104, 1, 1);
        Assert.That(input.clutch.Active, Is.True);
        Assert.That(input.clicks, Is.Zero);
    }

    [Test]
    public void AnalogThresholdNoiseDoesNotStopClutchAndPartialReleaseCannotRearm()
    {
        var input = new AnalogGesture();
        input.Sample(0, 0, 0); input.Sample(.1, .7f, .7f);
        input.Sample(.11, .49f, .51f);
        input.Sample(.12, .51f, .49f);
        input.Sample(.13, .2f, .2f);
        Assert.That(input.clutch.Active, Is.True, "A half-pressure threshold crossing is not physical release");
        input.Sample(.14, .05f, .2f);
        Assert.That(input.clutch.Active, Is.False);
        Assert.That(input.clutch.AwaitingRelease, Is.True);
        input.Sample(.15, .2f, .05f);
        Assert.That(input.clutch.AwaitingRelease, Is.True, "Both below the SDK button threshold does not mean fully released");
        input.Sample(.16, .8f, .8f);
        Assert.That(input.clutch.Active, Is.False);
        input.Sample(.17, .05f, .05f);
        input.Sample(.18, .8f, .8f);
        Assert.That(input.clutch.Active, Is.True);
        Assert.That(input.clicks, Is.Zero);
    }

    [Test]
    public void ReadinessDropStillRequiresFreshReleaseAndCannotTurnHeldChordIntoUi()
    {
        var input = new AnalogGesture();
        input.Sample(0, 0, 0); input.Sample(.1, 1, 1);
        input.Sample(.11, 1, 1, ready: false);
        Assert.That(input.clutch.Active, Is.False);
        input.Sample(.12, 1, 1, ready: true);
        Assert.That(input.clutch.Active, Is.False, "A readiness recovery cannot restart held input");
        Assert.That(input.hovered, Is.False);
        input.Sample(.13, 0, 0, ready: false);
        Assert.That(input.clutch.AwaitingRelease, Is.False, "Physical release can be observed while readiness catches up");
        input.Sample(.14, 1, 1, ready: false);
        input.Sample(.15, 1, 1, ready: true);
        Assert.That(input.clutch.Active, Is.False);
        input.Sample(.16, 0, 0); input.Sample(.17, 1, 1);
        Assert.That(input.clutch.Active, Is.True);
        Assert.That(input.clicks, Is.Zero);
    }

    [Test]
    public void RightFirstPressOverUiIsCancelledAsSoonAsLeftSqueezeBegins()
    {
        var input = new AnalogGesture();
        input.Sample(0, 0, 0);
        input.Sample(.1, 0, .8f); // A legitimate UI intent is pending until release.
        Assert.That(input.hovered, Is.True);
        input.Sample(.11, .2f, .8f); // Left squeeze is not yet a button press.
        Assert.That(input.clutch.Active, Is.False);
        Assert.That(input.hovered, Is.False);
        input.Sample(.12, .8f, .8f);
        Assert.That(input.clutch.Active, Is.True);
        input.Sample(.13, 0, 0);
        Assert.That(input.clicks, Is.Zero);
        input.Sample(.14, 0, .8f); input.Sample(.15, 0, 0);
        Assert.That(input.clicks, Is.EqualTo(1), "A fresh right-only UI press remains usable after the arm gesture");
    }

    [Test]
    public void StartupPartialSqueezeAndInvalidAxisCannotCountAsFullRelease()
    {
        var input = new AnalogGesture();
        input.Sample(0, .2f, .2f);
        input.Sample(.1, .8f, .8f);
        Assert.That(input.clutch.Active, Is.False);
        input.Sample(.2, 0, 0, tracked: false);
        Assert.That(input.clutch.AwaitingRelease, Is.True);
        input.Sample(.3, float.NaN, 0);
        Assert.That(input.left.FullyReleased, Is.False);
        Assert.That(input.clutch.AwaitingRelease, Is.True);
        input.Sample(.4, 0, 0); input.Sample(.5, 1, 1);
        Assert.That(input.clutch.Active, Is.True);
        input.Sample(.6, float.PositiveInfinity, 0);
        Assert.That(input.clutch.Active, Is.False);
    }

    [Test]
    public void DesktopWithoutXrDoesNotClaimUiWithMissingAnalogInputs()
    {
        var host = new GameObject("Desktop trigger ownership");
        try
        {
            var teleop = host.AddComponent<BimanualTeleop>();
            teleop.Tick(false);
            Assert.That(teleop.SuppressActions, Is.False);
            Assert.That(teleop.Clutch.Active, Is.False);
            Assert.That(teleop.Clutch.AwaitingRelease, Is.True);
        }
        finally { Object.DestroyImmediate(host); }
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
        var body = Quaternion.Euler(0, 40, 0);
        Step(c, .1, true, true, bodyRotation: body);
        Vector3 startLeft = c.LeftPosition, startRight = c.RightPosition;
        Vector3 translation = new Vector3(3, -.2f, -4);
        Step(c, .2, true, true, bodyRotation: body, head: Head + translation,
            left: Left + translation, right: Right + translation);
        Assert.That(Vector3.Distance(c.LeftPosition, startLeft), Is.LessThan(1e-6));
        Assert.That(Vector3.Distance(c.RightPosition, startRight), Is.LessThan(1e-6));
        Assert.That(c.Active, Is.True);
    }
    [Test]
    public void HeadYawPitchAndRollDoNotMoveArmTargets()
    {
        var c = Armed();
        var leftOrientation = Quaternion.Euler(10, 20, 30);
        var rightOrientation = Quaternion.Euler(-20, 40, -10);
        Step(c, .1, true, true, leftRotation: leftOrientation, rightRotation: rightOrientation);
        var expectedLeft = c.LeftPosition;
        var expectedRight = c.RightPosition;
        foreach (var gaze in new[] { Quaternion.Euler(25, 90, 35), Quaternion.Euler(-30, -90, -20),
                                    Quaternion.Euler(90, 40, 0) })
        {
            Step(c, .2, true, true, yaw: gaze, leftRotation: leftOrientation, rightRotation: rightOrientation);
            Assert.That(c.Active, Is.True);
            Assert.That(c.PoseValid, Is.True);
            Assert.That(Vector3.Distance(c.LeftPosition, expectedLeft), Is.LessThan(1e-6));
            Assert.That(Vector3.Distance(c.RightPosition, expectedRight), Is.LessThan(1e-6));
            Assert.That(Quaternion.Angle(c.LeftRotation, leftOrientation), Is.LessThan(.01f));
            Assert.That(Quaternion.Angle(c.RightRotation, rightOrientation), Is.LessThan(.01f));
        }
    }
    [Test]
    public void BodyHeadingDefinesArmFrameAndIgnoresBodyPitchAndRoll()
    {
        var c = Armed();
        Step(c, .1, true, true);
        Step(c, .2, true, true, bodyRotation: Quaternion.Euler(25, 90, 35));
        // Inverse current +90-degree heading maps world forward to local left.
        Assert.That(Vector3.Distance(c.LeftPosition, new Vector3(-.4f, -.5f, -.2f)), Is.LessThan(1e-5));
        Assert.That(Vector3.Distance(c.RightPosition, new Vector3(-.4f, -.5f, .2f)), Is.LessThan(1e-5));
        Assert.That(Quaternion.Angle(c.LeftRotation, Quaternion.Euler(0, -90, 0)), Is.LessThan(.01f));
        Vector3 expected = c.LeftPosition;
        Step(c, .3, true, true, bodyRotation: Quaternion.Euler(-30, 90, -20));
        Assert.That(Vector3.Distance(c.LeftPosition, expected), Is.LessThan(1e-5));
    }
    [Test]
    public void CommonRigYawPreservesTargetsOnlyWhenBodyHeadingMovesWithRig()
    {
        var c = Armed();
        Step(c, .1, true, true);
        var expectedLeft = c.LeftPosition;
        var expectedRight = c.RightPosition;
        var yaw = Quaternion.Euler(0, 65, 0);
        var movedLeft = Head + yaw * (Left - Head);
        var movedRight = Head + yaw * (Right - Head);
        Step(c, .2, true, true, yaw: yaw, bodyRotation: yaw, left: movedLeft, right: movedRight,
            leftRotation: yaw, rightRotation: yaw);
        Assert.That(Vector3.Distance(c.LeftPosition, expectedLeft), Is.LessThan(1e-6));
        Assert.That(Vector3.Distance(c.RightPosition, expectedRight), Is.LessThan(1e-6));
        Assert.That(Quaternion.Angle(c.LeftRotation, Quaternion.identity), Is.LessThan(.01f));
        Assert.That(Quaternion.Angle(c.RightRotation, Quaternion.identity), Is.LessThan(.01f));
        Step(c, .3, true, true, yaw: yaw, left: movedLeft, right: movedRight,
            leftRotation: yaw, rightRotation: yaw);
        Assert.That(Vector3.Distance(c.LeftPosition, expectedLeft), Is.GreaterThan(.1f),
            "Head yaw alone must not recenter the arm frame");
    }
    [Test]
    public void InitialActivePacketAndReclutchContainCurrentAbsoluteHandPoses()
    {
        var c = Armed();
        var firstLeft = Left + new Vector3(-.03f, .06f, .02f);
        var firstRight = Right + new Vector3(.02f, -.04f, .03f);
        var rotation = Quaternion.Euler(15, 25, -10);
        Assert.That(Step(c, .001, true, true, left: firstLeft, right: firstRight, leftRotation: rotation), Is.True,
            "Activation must publish immediately, even before the idle heartbeat cadence");
        Assert.That(c.Active, Is.True);
        Assert.That(Vector3.Distance(c.LeftPosition, firstLeft - Head), Is.LessThan(1e-6));
        Assert.That(Vector3.Distance(c.RightPosition, firstRight - Head), Is.LessThan(1e-6));
        Assert.That(Quaternion.Angle(c.LeftRotation, rotation), Is.LessThan(.01f));
        Step(c, .1, false, false);
        var nextLeft = Left + new Vector3(.05f, -.02f, .08f);
        Assert.That(Step(c, .102, true, true, left: nextLeft, right: firstRight), Is.True);
        Assert.That(c.Active, Is.True);
        Assert.That(Vector3.Distance(c.LeftPosition, nextLeft - Head), Is.LessThan(1e-6),
            "Reclutch uses the current pose; neither the first pose nor a zero delta is sent");
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
        Assert.That(roundTrip.schema_version, Is.EqualTo(3));
        Assert.That(roundTrip.frame_id, Is.EqualTo("teleop_body"));
        Assert.That(roundTrip.left_rotation, Is.EqualTo(lq));
        Assert.That(roundTrip.right_rotation, Is.EqualTo(rq));
    }
    [Test]
    public void NearVerticalBodyHeadingStopsActiveClutchUntilTrackedRelease()
    {
        var c = Armed();
        Step(c, .1, true, true);
        Step(c, .2, true, true, bodyRotation: Quaternion.Euler(89, 40, 0));
        Assert.That(c.Active, Is.False);
        Assert.That(c.PoseValid, Is.False);
        Step(c, .3, true, true);
        Assert.That(c.Active, Is.False);
        Step(c, .4, false, false);
        Step(c, .5, true, true);
        Assert.That(c.Active, Is.True);
    }
    [TestCase("head")]
    [TestCase("body")]
    [TestCase("left")]
    [TestCase("right")]
    public void InvalidQuaternionStopsBothArms(string source)
    {
        var c = Armed();
        Step(c, .1, true, true);
        Quaternion bad = new Quaternion(0, float.NaN, 0, 0);
        Step(c, .2, true, true, yaw: source == "head" ? bad : Quaternion.identity,
            bodyRotation: source == "body" ? bad : Quaternion.identity,
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
            var status = new AlignmentStatus { stamp = stamp, teleop_ready = true, teleop_protocol_version = 2,
                teleop_last_fault = "unreachable_target", teleop_reason = "clutch_released" };
            onStatus.Invoke(bridge, new object[] { new RosMessageTypes.Std.StringMsg(JsonUtility.ToJson(status)) });
            Assert.That(bridge.TeleopProtocolCompatible, Is.False);
            Assert.That(bridge.CanTeleoperate, Is.False);
            Assert.That(bridge.LastTeleopFault, Is.EqualTo("unreachable_target"));
            status.stamp += .001; status.teleop_protocol_version = 3; status.teleop_last_fault = "";
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
