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
    GameObject root;
    DeicticSettings settings;
    DeicticBridge bridge;
    readonly BimanualClutch clutch = new BimanualClutch();
    double[][] joints;
    double jointStamp;
    [Serializable] sealed class BackendCommand { public bool active; public double[] positions; }
    readonly List<double[]> activeCommands = new List<double[]>();
    Vector3 left = new Vector3(-.2f, 1.2f, .3f), right = new Vector3(.2f, 1.2f, .3f);
    Vector3 head = new Vector3(0, 1.7f, 0);
    Quaternion headRotation = Quaternion.identity, leftRotation = Quaternion.identity, rightRotation = Quaternion.identity;

    [UnityTest, Category("ROSBimanualSimulationIntegration")]
    public IEnumerator ScriptedClutchMovesEachIsaacArmAndReleaseHolds()
    {
        if (Environment.GetEnvironmentVariable("DEICTIC_BIMANUAL_SIM_INTEGRATION") != "1")
            Assert.Ignore("Opt in only with an isolated Isaac-only endpoint at localhost:10000; stop other Unity publishers.");
        settings = UnityEngine.Object.Instantiate(Resources.Load<DeicticSettings>("DeicticSettings"));
        settings.rosHost = "127.0.0.1";
        settings.connectOnStart = true;
        settings.syntheticScene = true;
        root = new GameObject("Scripted bimanual simulation wire test");
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
        bridge.Ros.Subscribe<RosMessageTypes.Std.StringMsg>("/k1/teleop/command", message => {
            var value = JsonUtility.FromJson<BackendCommand>(message.data);
            if (value.active && value.positions != null && value.positions.Length == 8)
                activeCommands.Add(value.positions);
        });
        float deadline = Time.realtimeSinceStartup + 30;
        while ((!bridge.CanTeleoperate || joints == null) && Time.realtimeSinceStartup < deadline)
        {
            Sample(false, false);
            yield return null;
        }
        Assert.That(bridge.CanTeleoperate, Is.True, bridge.Status?.teleop_reason ?? bridge.Feedback);
        Assert.That(joints, Is.Not.Null, "Both four-joint arms must have actual feedback");
        Assert.That(bridge.Status.mode, Is.EqualTo("markerless"), "Use the declared Isaac markerless fixture");
        yield return Phase(.4f, false, false);
        var initial = Snapshot();
        activeCommands.Clear();
        yield return Phase(.5f, true, true);
        Assert.That(clutch.Active, Is.True, clutch.Reason);
        Assert.That(bridge.Status.teleop_active, Is.True, bridge.Status.teleop_reason);
        AssertConstantCommands();
        // Isaac drives settle against gravity after capturing a measured hold.
        // Check exact command constancy separately from physical tracking error.
        Assert.That(MaxDelta(initial, joints, 0), Is.LessThan(.03), "Left arm exceeded hold tracking allowance");
        Assert.That(MaxDelta(initial, joints, 1), Is.LessThan(.03), "Right arm exceeded hold tracking allowance");

        // Moving the tracked body as a whole must leave body-relative wrist targets unchanged.
        var bodyTranslation = new Vector3(.7f, .1f, -.5f);
        int commandsBeforeTranslation = activeCommands.Count;
        head += bodyTranslation; left += bodyTranslation; right += bodyTranslation;
        yield return Phase(.3f, true, true);
        Assert.That(activeCommands.Count - commandsBeforeTranslation, Is.GreaterThan(2), "Translation must retain live command publication");
        AssertConstantCommands();
        var bodyYaw = Quaternion.Euler(0, 60, 0);
        int commandsBeforeYaw = activeCommands.Count;
        left = head + bodyYaw * (left - head);
        right = head + bodyYaw * (right - head);
        headRotation = bodyYaw * headRotation;
        leftRotation = bodyYaw * leftRotation; rightRotation = bodyYaw * rightRotation;
        yield return Phase(.3f, true, true);
        Assert.That(activeCommands.Count - commandsBeforeYaw, Is.GreaterThan(2), "Yaw must retain live command publication");
        AssertConstantCommands();

        // Inward motion exposes the straight-arm singularity missed by forward-only tests.
        left += headRotation * Vector3.right * .10f;
        yield return Phase(8f, true, true);
        Assert.That(MaxDelta(initial, joints, 0), Is.GreaterThan(.015), "Left hand did not move the measured left arm");
        Assert.That(MaxDelta(initial, joints, 1), Is.LessThan(.015), "Left-only input moved the other arm");
        var afterLeft = Snapshot();
        right += headRotation * Vector3.left * .10f;
        yield return Phase(8f, true, true);
        Assert.That(MaxDelta(afterLeft, joints, 1), Is.GreaterThan(.015), "Right hand did not move the measured right arm");
        Assert.That(bridge.Status.teleop_active, Is.True, bridge.Status.teleop_reason);
        Assert.That(RosFrames.Now - jointStamp, Is.InRange(-.05, .5), "Final feedback must be fresh");
        Debug.Log($"Scripted Unity→ROS→Isaac joint motion: left={MaxDelta(initial, joints, 0):F5}, right={MaxDelta(initial, joints, 1):F5} rad");

        yield return Phase(.4f, false, true);
        Assert.That(clutch.Active, Is.False);
        Assert.That(bridge.Status.teleop_active, Is.False, "Release either trigger must hold both arms");
        var held = Snapshot();
        left += Vector3.forward * .02f;
        right += Vector3.forward * .02f;
        yield return Phase(.7f, true, true);
        Assert.That(clutch.Active, Is.False, "Repressing one trigger without full release must not rearm");
        Assert.That(MaxDelta(held, joints, 0), Is.LessThan(.01));
        Assert.That(MaxDelta(held, joints, 1), Is.LessThan(.01));
        yield return Phase(.4f, false, false);
        activeCommands.Clear();
        yield return Phase(.5f, true, true);
        Assert.That(clutch.Active, Is.True, clutch.Reason);
        Assert.That(bridge.Status.teleop_active, Is.True, bridge.Status.teleop_reason);
        AssertConstantCommands();
        Assert.That(MaxDelta(held, joints, 0), Is.LessThan(.03), "Reclutch must capture new references");
        Assert.That(MaxDelta(held, joints, 1), Is.LessThan(.03));
        yield return Phase(.3f, false, false);
    }

    void Sample(bool leftHeld, bool rightHeld)
    {
        if (clutch.Step(Time.realtimeSinceStartupAsDouble, leftHeld, rightHeld, true, true,
            true, true, bridge.CanTeleoperate, head, headRotation,
            left, leftRotation, right, rightRotation))
            bridge.PublishTeleop(clutch.Active, true, true, clutch.LeftPosition, clutch.LeftRotation,
                clutch.RightPosition, clutch.RightRotation);
    }
    IEnumerator Phase(float seconds, bool leftHeld, bool rightHeld)
    {
        float until = Time.realtimeSinceStartup + seconds;
        while (Time.realtimeSinceStartup < until) { Sample(leftHeld, rightHeld); yield return null; }
    }
    double[][] Snapshot() => new[] { (double[])joints[0].Clone(), (double[])joints[1].Clone() };
    void AssertConstantCommands()
    {
        Assert.That(activeCommands.Count, Is.GreaterThan(3), "Observe actual backend targets during zero-input clutch");
        foreach (var command in activeCommands)
            for (int i = 0; i < 8; i++)
                Assert.That(Math.Abs(command[i] - activeCommands[0][i]), Is.LessThan(1e-6),
                    "A stationary controller must not change the captured joint targets");
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
            bridge.Ros.Disconnect();
            UnityEngine.Object.Destroy(bridge.Ros.gameObject);
        }
        UnityEngine.Object.Destroy(root);
        UnityEngine.Object.Destroy(settings);
        yield return null;
    }
}
