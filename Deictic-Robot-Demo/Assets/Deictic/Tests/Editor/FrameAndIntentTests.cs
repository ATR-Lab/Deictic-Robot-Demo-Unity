using System;
using System.Reflection;
using Deictic;
using NUnit.Framework;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public sealed class FrameAndIntentTests
{
    [Test]
    public void OptimizerUpdatesPreservePreviewButOriginResetAndBackendInvalidationClearIt()
    {
        var host = new GameObject("Preview epoch test");
        var settings = ScriptableObject.CreateInstance<DeicticSettings>();
        try
        {
            var bridge = host.AddComponent<DeicticBridge>();
            bridge.settings = settings;
            var status = new AlignmentStatus
            {
                stamp = RosFrames.Now, confidence = 1, can_commit = true,
                alignment_version = 1, alignment_epoch = 3,
                base_from_headset_world = new double[] { 0, 0, 0, 0, 0, 0, 1 }
            };
            var flags = BindingFlags.Instance | BindingFlags.NonPublic;
            var onStatus = typeof(DeicticBridge).GetMethod("OnStatus", flags);
            var hasPreview = typeof(DeicticBridge).GetProperty(nameof(DeicticBridge.HasPreview));
            onStatus.Invoke(bridge, new object[] { new StringMsg(JsonUtility.ToJson(status)) });
            hasPreview.SetValue(bridge, true); // A previously received, correlated preview.
            status.stamp += .01;
            status.alignment_version++;
            onStatus.Invoke(bridge, new object[] { new StringMsg(JsonUtility.ToJson(status)) });
            Assert.That(bridge.HasPreview, Is.True, "An optimizer update alone does not revoke the server's preview");
            status.stamp += .01;
            status.alignment_epoch++;
            onStatus.Invoke(bridge, new object[] { new StringMsg(JsonUtility.ToJson(status)) });
            Assert.That(bridge.HasPreview, Is.False, "Recentered coordinates cannot reuse an old preview");
            hasPreview.SetValue(bridge, true);
            typeof(DeicticBridge).GetMethod("OnPreview", flags).Invoke(bridge,
                new object[] { new RosMessageTypes.Trajectory.JointTrajectoryMsg() });
            Assert.That(bridge.HasPreview, Is.False, "The controller can invalidate a preview within the same epoch");
            Assert.That(bridge.Execute(), Is.False, "Clock, connection and preview gates remain required");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(host);
            UnityEngine.Object.DestroyImmediate(settings);
        }
    }

    [Test]
    public void TrackingInvalidationRequiresObservationCapturedAfterEvent()
    {
        var host = new GameObject("Tracking invalidation test");
        var settings = ScriptableObject.CreateInstance<DeicticSettings>();
        double oldOffset = RosFrames.ClockOffsetSeconds;
        try
        {
            RosFrames.ClockOffsetSeconds = 0;
            var bridge = host.AddComponent<DeicticBridge>();
            settings.statusTimeout = 5;
            bridge.settings = settings;
            // Exercise status validation without initializing ROS or native OVR.
            MethodInfo onStatus = typeof(DeicticBridge).GetMethod("OnStatus", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.That(onStatus, Is.Not.Null);
            bridge.InvalidateTracking("Test recenter");
            Assert.That(bridge.HasAlignment, Is.False);
            Assert.That(bridge.HasPreview, Is.False);

            var status = new AlignmentStatus
            {
                stamp = RosFrames.Now,
                confidence = 1,
                can_commit = true,
                alignment_version = 1,
                base_from_headset_world = new double[] { 0, 0, 0, 0, 0, 0, 1 },
                registration_age = 1
            };
            onStatus.Invoke(bridge, new object[] { new StringMsg(JsonUtility.ToJson(status)) });
            Assert.That(bridge.HasAlignment, Is.False, "New status must not revive a pre-event registration");

            status.stamp += .01;
            status.registration_age = 0;
            onStatus.Invoke(bridge, new object[] { new StringMsg(JsonUtility.ToJson(status)) });
            Assert.That(bridge.HasAlignment, Is.True, "A post-event registration can restore alignment");
            Assert.That(bridge.CanCommit, Is.False, "Clock and connection gates still apply");
            Assert.That(bridge.Execute(), Is.False);
        }
        finally
        {
            RosFrames.ClockOffsetSeconds = oldOffset;
            UnityEngine.Object.DestroyImmediate(host);
            UnityEngine.Object.DestroyImmediate(settings);
        }
    }

    [Test]
    public void EmptyRos2MessageIncludesRequiredCdrDummyByte()
    {
#if ROS2
        var serializer = new MessageSerializer();
        serializer.SerializeMessageWithLength(new EmptyMsg());
        // Fast CDR on ROS Jazzy rejects a header-only Empty. The five-byte
        // message below was independently accepted by rclpy.deserialize_message.
        Assert.That(serializer.GetBytes(), Is.EqualTo(new byte[] { 5, 0, 0, 0, 0, 1, 0, 0, 0 }));

        serializer.Clear();
        serializer.SerializeMessage(new EmptyMsg());
        serializer.Write((byte)0x42);
        var deserializer = new MessageDeserializer();
        deserializer.InitWithBuffer(serializer.GetBytes());
        Assert.That(EmptyMsg.Deserialize(deserializer), Is.Not.Null);
        deserializer.Read(out byte following);
        Assert.That(following, Is.EqualTo(0x42), "Empty must consume its dummy byte before the next field");

        // Native Fast DDS pads this same Empty message to eight bytes.
        deserializer.InitWithBuffer(new byte[] { 0, 1, 0, 0, 0, 0, 0, 0 });
        Assert.That(EmptyMsg.Deserialize(deserializer), Is.Not.Null);
#else
        Assert.Ignore("ROS 2 serialization regression; this build is configured for ROS 1");
#endif
    }

    [Test]
    public void AxesMapToRosForwardLeftUp()
    {
        var p = RosFrames.Pose(new Vector3(2, 3, 4), Quaternion.identity);
        Assert.That(p.position.x, Is.EqualTo(4));
        Assert.That(p.position.y, Is.EqualTo(-2));
        Assert.That(p.position.z, Is.EqualTo(3));
    }
    [Test]
    public void PoseConversionPreservesCompositionAcrossHandedness()
    {
        var pose = Matrix4x4.TRS(new Vector3(.2f, -.3f, .8f), Quaternion.Euler(12, 53, -19), Vector3.one);
        var p = RosFrames.Pose(pose.GetColumn(3), pose.rotation);
        var actual = RosFrames.Matrix(new[] { p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w });
        Vector3 point = new Vector3(.4f, .2f, -.8f);
        Assert.That(Vector3.Distance(actual.MultiplyPoint3x4(point), pose.MultiplyPoint3x4(point)), Is.LessThan(1e-6));
        Assert.That(Vector3.Distance(actual.inverse.MultiplyPoint3x4(actual.MultiplyPoint3x4(point)), point), Is.LessThan(1e-6));
    }
    [Test]
    public void OpticalAxesAreRightDownForwardInRosWorld()
    {
        var p = RosFrames.OpticalPose(Vector3.zero, Quaternion.identity);
        var q = new Quaternion((float)p.orientation.x, (float)p.orientation.y, (float)p.orientation.z, (float)p.orientation.w);
        Assert.That(Vector3.Distance(q * Vector3.right, new Vector3(0, -1, 0)), Is.LessThan(1e-6));
        Assert.That(Vector3.Distance(q * Vector3.up, new Vector3(0, 0, -1)), Is.LessThan(1e-6));
        Assert.That(Vector3.Distance(q * Vector3.forward, new Vector3(1, 0, 0)), Is.LessThan(1e-6));
    }
    [Test]
    public void ConflictingHeadRayDefersToExplicitPointer()
    {
        var head = new RaycastHit { point = Vector3.zero, normal = Vector3.up };
        var pointer = new RaycastHit { point = Vector3.right, normal = Vector3.forward };
        Assert.That(TargetResolver.Resolve(true, head, true, pointer, .04f, out Vector3 point, out Vector3 normal), Is.True);
        Assert.That(point, Is.EqualTo(Vector3.right)); Assert.That(normal, Is.EqualTo(Vector3.forward));
        pointer.point = new Vector3(.02f, 0, 0);
        TargetResolver.Resolve(true, head, true, pointer, .04f, out point, out normal);
        Assert.That(point.x, Is.EqualTo(.01f).Within(1e-6));
    }
    [Test]
    public void NoRayHitCannotProduceAnOriginGoal()
    {
        Assert.That(TargetResolver.Resolve(false, default, false, default, .04f, out _, out _), Is.False);
        Assert.Throws<ArgumentException>(() => RosFrames.Matrix(new[] { double.NaN, 0, 0, 0, 0, 0, 1 }));
        Assert.Throws<ArgumentException>(() => RosFrames.Matrix(new double[7]));
    }
}
