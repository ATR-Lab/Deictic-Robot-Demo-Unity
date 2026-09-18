using System;
using System.Reflection;
using Deictic;
using NUnit.Framework;
using RosMessageTypes.Trajectory;
using UnityEngine;

public sealed class PreviewFeedbackTests
{
    const BindingFlags PrivateInstance = BindingFlags.Instance | BindingFlags.NonPublic;

    static void Invoke(object target, string method, params object[] arguments) =>
        target.GetType().GetMethod(method, PrivateInstance).Invoke(target, arguments);

    static T Field<T>(object target, string name) =>
        (T)target.GetType().GetField(name, PrivateInstance).GetValue(target);

    static void SetProperty(object target, string name, object value) =>
        target.GetType().GetProperty(name).SetValue(target, value);

    [TestCase(true)]
    [TestCase(false)]
    public void AlignmentUpdatesReprojectTheSamePlanAndAccuratelyLabelItsTarget(bool hasCommittedTarget)
    {
        var robot = new GameObject("Preview feedback test");
        var bridgeHost = new GameObject("Preview bridge test");
        K1RobotView view = null;
        try
        {
            // Start away from the world origin to expose accidental world-space FK storage.
            robot.transform.SetPositionAndRotation(new Vector3(1, 2, 3), Quaternion.Euler(12, 27, -8));
            var jointHost = new GameObject("Test elbow");
            jointHost.transform.SetParent(robot.transform, false);
            var joint = jointHost.AddComponent<K1Joint>();
            joint.jointName = "right_elbow_yaw_joint"; joint.axis = Vector3.up;
            var tool = new GameObject("right_tool").transform;
            tool.SetParent(jointHost.transform, false);
            Vector3 initialPoint = new Vector3(.2f, .1f, .3f);
            tool.localPosition = initialPoint;
            Vector3 plannedPoint = Quaternion.AngleAxis(-90, Vector3.up) * initialPoint;
            Vector3 requestedPoint = plannedPoint + new Vector3(.0015f, .0005f, -.001f);
            var bridge = bridgeHost.AddComponent<DeicticBridge>();
            if (hasCommittedTarget) SetProperty(bridge, nameof(DeicticBridge.CommittedBasePoint), (Vector3?)requestedPoint);
            view = robot.AddComponent<K1RobotView>();
            view.preview = true; view.bridge = bridge;
            Invoke(view, "Start");
            var first = new JointTrajectoryPointMsg { positions = new[] { 0.0 } };
            var last = new JointTrajectoryPointMsg { positions = new[] { Math.PI / 2 } };
            last.time_from_start.sec = 1;
            var message = new JointTrajectoryMsg { joint_names = new[] { joint.jointName }, points = new[] { first, last } };
            Invoke(view, "SetPreview", message);
            var path = Field<LineRenderer>(view, "path");
            var marker = Field<GameObject>(view, "plannedEndpoint");
            var ownedMaterial = path.sharedMaterial;
            var ghostMaterial = Field<Material>(view, "previewMaterial");
            Assert.That(path.useWorldSpace, Is.False);
            Assert.That(Vector3.Distance(path.GetPosition(0), initialPoint), Is.LessThan(1e-6));
            Assert.That(Vector3.Distance(path.GetPosition(1), plannedPoint), Is.LessThan(1e-6));
            Assert.That(marker.GetComponentInChildren<TextMesh>().text,
                Is.EqualTo(hasCommittedTarget ? "Committed target" : "Planned endpoint"));
            Assert.That(robot.GetComponentsInChildren<Collider>(true), Is.Empty, "Feedback must never intercept target rays");

            var worldFromBase = Matrix4x4.TRS(new Vector3(-.4f, .8f, 1.1f), Quaternion.Euler(-17, 43, 9), Vector3.one);
            SetProperty(bridge, nameof(DeicticBridge.BaseFromWorld), worldFromBase.inverse);
            SetProperty(bridge, nameof(DeicticBridge.HasAlignment), true);
            Invoke(view, "Update");
            Assert.That(Vector3.Distance(path.transform.TransformPoint(path.GetPosition(0)),
                worldFromBase.MultiplyPoint3x4(initialPoint)), Is.LessThan(1e-6));
            Assert.That(Vector3.Distance(path.transform.TransformPoint(path.GetPosition(1)),
                worldFromBase.MultiplyPoint3x4(plannedPoint)), Is.LessThan(1e-6));
            Assert.That(Vector3.Distance(marker.transform.position,
                worldFromBase.MultiplyPoint3x4(hasCommittedTarget ? requestedPoint : plannedPoint)), Is.LessThan(1e-6));
            Assert.That(Field<JointTrajectoryMsg>(view, "trajectory"), Is.SameAs(message), "Alignment updates must preserve the approved plan");
            Assert.That(last.positions[0], Is.EqualTo(Math.PI / 2));

            bridge.Cancel(); // Exercise the subscribed PreviewCleared path without ROS/native XR.
            Assert.That(path.positionCount, Is.Zero);
            Assert.That(marker.activeSelf, Is.False);
            Assert.That(bridge.CommittedBasePoint.HasValue, Is.False);
            // Start was invoked manually in EditMode, outside Unity's Play lifecycle;
            // exercise its paired cleanup explicitly before removing the component.
            Invoke(view, "OnDestroy");
            UnityEngine.Object.DestroyImmediate(view);
            Assert.That(path == null, Is.True);
            Assert.That(marker == null, Is.True);
            Assert.That(ownedMaterial == null, Is.True);
            Assert.That(ghostMaterial == null, Is.True);
        }
        finally
        {
            if (view) UnityEngine.Object.DestroyImmediate(view);
            UnityEngine.Object.DestroyImmediate(robot);
            UnityEngine.Object.DestroyImmediate(bridgeHost);
        }
    }

    [Test]
    public void ClearingAnOldWirePreviewPreservesThePendingGoalButCancelClearsIt()
    {
        var host = new GameObject("Pending goal feedback test");
        try
        {
            var bridge = host.AddComponent<DeicticBridge>();
            Vector3 point = new Vector3(.2f, .3f, -.1f);
            SetProperty(bridge, nameof(DeicticBridge.CommittedBasePoint), (Vector3?)point);
            typeof(DeicticBridge).GetField("pendingGoalNanoseconds", PrivateInstance).SetValue(bridge, 123456L);
            Invoke(bridge, "OnPreview", new JointTrajectoryMsg());
            Assert.That(bridge.CommittedBasePoint, Is.EqualTo((Vector3?)point));
            Assert.That(Field<long>(bridge, "pendingGoalNanoseconds"), Is.EqualTo(123456L));
            bridge.Cancel();
            Assert.That(bridge.CommittedBasePoint.HasValue, Is.False);
            Assert.That(Field<long>(bridge, "pendingGoalNanoseconds"), Is.EqualTo(-1));
        }
        finally { UnityEngine.Object.DestroyImmediate(host); }
    }
}
