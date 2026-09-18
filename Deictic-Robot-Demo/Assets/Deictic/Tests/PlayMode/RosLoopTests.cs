using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using Deictic;
using NUnit.Framework;
using RosMessageTypes.Geometry;
using UnityEngine;
using UnityEngine.TestTools;

public sealed class RosLoopTests
{
    DeicticBridge activeBridge;
    GameObject activeRoot, activeCamera;
    DeicticSettings activeSettings;

    [UnityTest, Category("ROSIntegration")]
    public IEnumerator SelectPreviewExecuteAndCancelThroughRos()
        => RunReach(false);

    [UnityTest, Category("ROSMarkerlessSimulationIntegration")]
    public IEnumerator SelectFixedWorldTargetThroughMarkerlessRos()
        => RunReach(true);

    [UnityTest, Category("ROSMarkerlessSimulationWorkspaceIntegration")]
    public IEnumerator SelectFiveFixedWorldTargetsThroughMarkerlessRos()
    {
        // A bounded workspace check in the same static fixture, not the paper's
        // mobile-user protocol: subsequent reaches start at the measured prior pose.
        for (int targetIndex = 1; targetIndex <= 5; targetIndex++)
        {
            yield return RunReach(true, targetIndex);
            yield return ReleaseSimulationConnection();
        }
    }

    IEnumerator RunReach(bool markerless, int targetIndex = 3)
    {
        string optIn = markerless ? "DEICTIC_MARKERLESS_SIM_INTEGRATION" : "DEICTIC_ROS_INTEGRATION";
        if (Environment.GetEnvironmentVariable(optIn) != "1")
            Assert.Ignore("Opt in with " + optIn + "=1 and an isolated Isaac-only ROS endpoint on localhost:10000.");
        var cameraObject = new GameObject("Integration camera");
        activeCamera = cameraObject;
        cameraObject.tag = "MainCamera";
        var camera = cameraObject.AddComponent<Camera>();
        camera.nearClipPlane = .02f;
        camera.backgroundColor = new Color(.045f, .065f, .1f);
        camera.clearFlags = CameraClearFlags.SolidColor;
        camera.transform.position = new Vector3(.8f, 1.35f, 2.15f);
        camera.transform.LookAt(new Vector3(.13f, .94f, 1.28f));
        var settings = UnityEngine.Object.Instantiate(Resources.Load<DeicticSettings>("DeicticSettings"));
        activeSettings = settings;
        Assert.That(settings.robotVisual, Is.Not.Null, "Build K1 assets before integration tests");
        settings.rosHost = "127.0.0.1"; settings.connectOnStart = true; settings.syntheticScene = true;
        var root = new GameObject("Integration demo");
        activeRoot = root;
        root.AddComponent<DeicticDemo>().settings = settings;
        yield return null;
        var bridge = root.GetComponent<DeicticBridge>();
        activeBridge = bridge;
        Assert.That(bridge, Is.Not.Null);
        Assert.That(targetIndex, Is.InRange(1, 5));
        double targetBaseX = .06 + .02 * targetIndex;
        double toolError = double.PositiveInfinity;
        double toolStamp = double.NegativeInfinity;
        bridge.Ros.Subscribe<PoseStampedMsg>("/k1/end_effector_pose", msg =>
        {
            if (msg.header.frame_id != "base_link") return;
            double stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
            if (stamp <= toolStamp) return;
            toolStamp = stamp;
            var p = msg.pose.position;
            toolError = Math.Sqrt(Math.Pow(p.x - targetBaseX, 2) + Math.Pow(p.y + .25, 2) + Math.Pow(p.z - .02, 2));
        });
        float until = Time.realtimeSinceStartup + 30;
        while (!bridge.CanCommit && Time.realtimeSinceStartup < until) yield return null;
        Assert.That(bridge.CanCommit, Is.True, "No fresh alignment: " + bridge.Feedback);
        Assert.That(bridge.Status.mode, Is.EqualTo(markerless ? "markerless" : "synthetic_test"));
        if (markerless) Assert.That(bridge.Status.backend, Is.EqualTo("gtsam_isam2"));
        Vector3 point;
        if (markerless)
        {
            // Independent fixture geometry: the world target never follows the
            // estimated alignment or the measured robot visual's moving parent.
            var target = GameObject.Find("Reach target " + targetIndex);
            Assert.That(target, Is.Not.Null);
            point = target.transform.position;
            Assert.That(Vector3.Distance(point, new Vector3(.25f, .92f, 1.2f + (float)targetBaseX)), Is.LessThan(1e-5f));
            float groundingError = Vector3.Distance(bridge.BaseFromWorld.MultiplyPoint3x4(point), new Vector3(.25f, .02f, (float)targetBaseX));
            Assert.That(groundingError, Is.LessThan(.03f), "Estimated alignment misgrounds the fixed world target");
            Debug.Log($"Independent fixed-world grounding error: {groundingError * 1000:F2} mm");
            Debug.Log($"Static workspace target {targetIndex}: fixed base goal ({targetBaseX:F2}, -0.25, 0.02) m");
        }
        else
        {
            // Known-alignment transport/control check. Back-solving through X
            // intentionally removes alignment error; it is not a grounding test.
            point = bridge.BaseFromWorld.inverse.MultiplyPoint3x4(new Vector3(.25f, .02f, .12f));
        }
        Assert.That(bridge.Commit(point, Vector3.up), Is.True);
        until = Time.realtimeSinceStartup + 12;
        while (!bridge.HasPreview && Time.realtimeSinceStartup < until) yield return null;
        Assert.That(bridge.HasPreview, Is.True, "Planner rejected goal: " + bridge.Feedback);
        // Let at least two CPU registration results arrive while the user
        // reviews a markerless preview. Small accepted alignment updates must
        // not force execution to race the registration publisher.
        yield return new WaitForSeconds(markerless ? 4.5f : .3f);
        if (SystemInfo.graphicsDeviceType != UnityEngine.Rendering.GraphicsDeviceType.Null)
        {
            var texture = new RenderTexture(1440, 1000, 24);
            camera.targetTexture = texture; camera.Render();
            var previous = RenderTexture.active; RenderTexture.active = texture;
            var png = new Texture2D(1440, 1000, TextureFormat.RGB24, false);
            png.ReadPixels(new Rect(0, 0, 1440, 1000), 0, 0); png.Apply();
            string output = Environment.GetEnvironmentVariable("DEICTIC_SCREENSHOT");
            if (!string.IsNullOrEmpty(output)) { Directory.CreateDirectory(Path.GetDirectoryName(output)); File.WriteAllBytes(output, png.EncodeToPNG()); }
            RenderTexture.active = previous; camera.targetTexture = null;
            UnityEngine.Object.Destroy(texture); UnityEngine.Object.Destroy(png);
        }
        string[] armNames = { "aaright_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_elbow_pitch_joint", "right_elbow_yaw_joint" };
        Dictionary<string, double> start = null, current = null;
        bridge.JointStateReceived += msg =>
        {
            if (msg.name == null || msg.position == null || msg.name.Length != msg.position.Length) return;
            var sample = new Dictionary<string, double>();
            for (int i = 0; i < msg.name.Length; i++) sample[msg.name[i]] = msg.position[i];
            foreach (string name in armNames) if (!sample.ContainsKey(name)) return;
            if (start == null) start = sample;
            current = sample;
        };
        yield return new WaitForSeconds(.2f);
        double executeStamp = RosFrames.Now;
        Assert.That(bridge.Execute(), Is.True, bridge.Feedback);
        until = Time.realtimeSinceStartup + 10;
        bool moved = false;
        while (!moved && Time.realtimeSinceStartup < until)
        {
            yield return null;
            if (start != null && current != null)
                foreach (string name in armNames) moved |= Math.Abs(start[name] - current[name]) > .005;
        }
        Assert.That(moved, Is.True, "No measured joint motion returned after execution");
        until = Time.realtimeSinceStartup + 20;
        while ((bridge.Status.operation != "execution_succeeded" || toolError > .03) && Time.realtimeSinceStartup < until) yield return null;
        Assert.That(bridge.Status.operation, Is.EqualTo("execution_succeeded"), bridge.Feedback);
        Assert.That(toolStamp, Is.GreaterThan(executeStamp), "Tool feedback must follow this execution request");
        Assert.That(RosFrames.Now - toolStamp, Is.InRange(-.05, .5), "Measured tool feedback is stale");
        Assert.That(toolError, Is.LessThan(.03), "Measured Isaac tool missed the paper's 3 cm reaching tolerance");
        Debug.Log($"Measured Isaac reach error: {toolError * 1000:F2} mm");
        bridge.Cancel();
        Assert.That(bridge.HasPreview, Is.False);
    }

    [UnityTearDown]
    public IEnumerator ReleaseSimulationConnection()
    {
        // Teardown runs after a failed assertion too. Let the cancel leave the
        // connector's asynchronous queue before disconnecting its transport.
        if (activeBridge != null && activeBridge.Ros != null)
        {
            activeBridge.Cancel();
            yield return new WaitForSecondsRealtime(.2f);
            activeBridge.Ros.Disconnect();
            // This fixture owns its isolated connection. Do not retain its
            // subscriber callbacks when constructing the next demo instance.
            UnityEngine.Object.Destroy(activeBridge.Ros.gameObject);
        }
        UnityEngine.Object.Destroy(activeRoot);
        UnityEngine.Object.Destroy(activeCamera);
        UnityEngine.Object.Destroy(activeSettings);
        activeBridge = null; activeRoot = activeCamera = null; activeSettings = null;
        // Destroy is deferred until frame end. A following workspace case must
        // not reuse the connection that has just been scheduled for destruction.
        yield return null;
    }
}
