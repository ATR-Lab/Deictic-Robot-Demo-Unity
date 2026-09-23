using Deictic;
using NUnit.Framework;
using UnityEngine;

public sealed class RobotHeadTrackingTests
{
    [Test]
    public void LatestHeadPosePublishesAt50HzWithoutChangingArmCadence()
    {
        var state = new RobotHeadSample();
        Assert.That(RobotHeadSample.PublishPeriod, Is.EqualTo(.02).Within(1e-12));
        Assert.That(BimanualClutch.PublishPeriod, Is.EqualTo(.05).Within(1e-12));
        Assert.That(state.Step(0, true, true, true, true, Quaternion.identity, Quaternion.identity), Is.True);
        var pose = Quaternion.Euler(12, 25, 0);
        Assert.That(state.Step(.019, true, true, true, true, pose, Quaternion.identity), Is.False);
        Assert.That(state.Step(.02, true, true, true, true, pose, Quaternion.identity), Is.True);
        Assert.That(Quaternion.Angle(state.Rotation, pose), Is.LessThan(.01));
        Assert.That(state.Step(.039, true, true, true, true, pose, Quaternion.identity), Is.False);
        Assert.That(state.Step(.04, true, true, true, true, pose, Quaternion.identity), Is.True);
        Assert.That(state.Step(5, true, true, true, true, pose, Quaternion.identity), Is.True);
        Assert.That(state.Step(5.001, true, true, true, true, pose, Quaternion.identity), Is.False,
            "A stalled render loop sends one latest pose, never a burst of old head poses");
    }

    [Test]
    public void AbsoluteHeadRotationIsPreservedOnFirstFrameAndEveryViewEntry()
    {
        var state = new RobotHeadSample();
        var pose = Quaternion.Euler(22, 35, 14);
        Assert.That(state.Step(0, false, true, true, true, pose, Quaternion.identity), Is.True);
        Assert.That(state.Active, Is.False);
        Assert.That(state.Step(.001, true, true, true, true, pose, Quaternion.identity), Is.True);
        Assert.That(state.Active, Is.True);
        Assert.That(Quaternion.Angle(state.Rotation, pose), Is.LessThan(.01));
        state.Step(.002, false, true, true, true, pose, Quaternion.identity);
        pose = Quaternion.Euler(-15, -20, 0);
        state.Step(.003, true, true, true, true, pose, Quaternion.identity);
        Assert.That(Quaternion.Angle(state.Rotation, pose), Is.LessThan(.01), "No neutral capture at POV entry");
    }

    [Test]
    public void CommonTrackingSpaceYawDoesNotChangeRobotOrientation()
    {
        var state = new RobotHeadSample();
        var pose = Quaternion.Euler(20, -30, 12);
        var originYaw = Quaternion.Euler(0, 65, 0);
        state.Step(0, true, true, true, true, pose, Quaternion.identity);
        var expected = state.Rotation;
        state.Step(.1, true, true, true, true, originYaw * pose, originYaw);
        Assert.That(Quaternion.Angle(state.Rotation, expected), Is.LessThan(.01));
    }

    [TestCase("tracking")]
    [TestCase("focus")]
    [TestCase("clock")]
    [TestCase("view")]
    public void LossSendsAnImmediateHoldOutsideCadence(string cause)
    {
        var state = new RobotHeadSample();
        state.Step(0, true, true, true, true, Quaternion.identity, Quaternion.identity);
        Assert.That(state.Step(.001, cause != "view", cause != "tracking", cause != "focus", cause != "clock",
            Quaternion.identity, Quaternion.identity), Is.True);
        Assert.That(state.Active, Is.False);
        state.Reset();
        Assert.That(state.Tracked, Is.False);
        Assert.That(state.Rotation, Is.EqualTo(Quaternion.identity));
    }

    [Test]
    public void VerticalGazeIsValidButMalformedPoseOrBodyFrameCannotCommandMotion()
    {
        var state = new RobotHeadSample();
        state.Step(0, true, true, true, true, Quaternion.Euler(90, 0, 0), Quaternion.identity);
        Assert.That(state.Active, Is.True, "The neck controller clamps physical pitch independently");
        state.Step(.1, true, true, true, true, new Quaternion(0, float.NaN, 0, 1), Quaternion.identity);
        Assert.That(state.Active, Is.False);
        state.Step(.2, true, true, true, true, Quaternion.identity, Quaternion.Euler(90, 0, 0));
        Assert.That(state.Active, Is.False);
    }

    [Test]
    public void WireAxesMatchK1YawAndPitchAndUseAtomicQuaternion()
    {
        var state = new RobotHeadSample();
        state.Step(0, true, true, true, true, Quaternion.Euler(20, 30, 0), Quaternion.identity);
        double[] q = DeicticBridge.TeleopRotation(state.Rotation);
        var ros = new Quaternion((float)q[0], (float)q[1], (float)q[2], (float)q[3]);
        // Numeric ROS forward +X turns toward -Y for Unity right yaw and -Z for looking down.
        Vector3 forward = ros * Vector3.right;
        Assert.That(Mathf.Atan2(forward.y, forward.x) * Mathf.Rad2Deg, Is.EqualTo(-30).Within(.001));
        Assert.That(Mathf.Atan2(-forward.z, new Vector2(forward.x, forward.y).magnitude) * Mathf.Rad2Deg,
            Is.EqualTo(20).Within(.001));
        var message = new DeicticBridge.RobotHeadCommand { active = true, tracked = true, orientation = q };
        var packet = JsonUtility.FromJson<DeicticBridge.RobotHeadCommand>(JsonUtility.ToJson(message));
        Assert.That(packet.schema_version, Is.EqualTo(1));
        Assert.That(packet.frame_id, Is.EqualTo("base_link"));
        Assert.That(packet.orientation, Is.EqualTo(q));
    }
}
