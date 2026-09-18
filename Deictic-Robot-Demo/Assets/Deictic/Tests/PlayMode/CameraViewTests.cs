using System;
using System.Collections;
using System.IO;
using Deictic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using UnityEngine.UI;

public sealed class CameraViewTests
{
    GameObject root, head;
    DeicticBridge bridge;
    DeicticSettings settings;
    DeicticCameraView view;

    void Create(bool connect)
    {
        root = new GameObject("Camera toggle test");
        head = new GameObject("Headset viewpoint");
        head.transform.position = new Vector3(0, 1.7f, 0);
        head.AddComponent<Camera>().clearFlags = CameraClearFlags.SolidColor;
        settings = ScriptableObject.CreateInstance<DeicticSettings>();
        settings.connectOnStart = connect;
        settings.syntheticScene = true;
        bridge = root.AddComponent<DeicticBridge>();
        bridge.Initialize(settings);
        view = root.AddComponent<DeicticCameraView>();
        view.Initialize(head.transform, bridge);
    }

    [UnityTest]
    public IEnumerator ToggleButtonIsReachableAndReturnsToUserViewWithoutCamera()
    {
        Create(false);
        yield return null;
        Assert.That(view.RobotView, Is.False);
        Assert.That(view.Stream.enabled, Is.False, "Hidden camera must not consume network bandwidth");
        Ray buttonRay = new Ray(head.transform.position, view.ToggleRect.position - head.transform.position);
        Assert.That(view.PointAtToggle(buttonRay, out Vector3 hit), Is.True);
        Assert.That(Vector3.Distance(hit, view.ToggleRect.position), Is.LessThan(.001f));
        Assert.That(view.PointAtToggle(new Ray(head.transform.position, Vector3.forward), out _), Is.False);
        view.ToggleButton.onClick.Invoke();
        yield return null;
        Assert.That(view.RobotView, Is.True);
        Assert.That(view.Stream.enabled, Is.True);
        Assert.That(view.Stream.HasFreshFrame, Is.False);
        Assert.That(view.ToggleRect.parent.GetComponentInChildren<RawImage>().enabled, Is.False, "Missing feed cannot look live");
        Assert.That(bridge.HasPreview, Is.False, "Changing view must not create a motion request");
        view.ToggleButton.onClick.Invoke();
        yield return null;
        Assert.That(view.RobotView, Is.False);
        Assert.That(view.Stream.enabled, Is.False);
        Assert.That(view.ToggleButton.gameObject.activeInHierarchy, Is.True, "Return button stays available without a camera");
        view.SetRobotView(true);
        view.enabled = false;
        Assert.That(view.Stream.enabled, Is.False, "Disabling the owner must release its sibling image subscription");
        Assert.That(view.ToggleButton.gameObject.activeInHierarchy, Is.False);
        Assert.That(view.PointAtToggle(buttonRay, out _), Is.False, "An invisible control must not consume target input");
        view.SetRobotView(true);
        Assert.That(view.Stream.enabled, Is.False, "Calling a disabled view must not reactivate network traffic");
        view.enabled = true;
        Assert.That(view.RobotView, Is.False);
        Assert.That(view.ToggleButton.gameObject.activeInHierarchy, Is.True);
    }

    [UnityTest]
    public IEnumerator MonitorAndReturnButtonStayInWorldWhenHeadAndOwnerMove()
    {
        Create(false);
        view.SetRobotView(true);
        yield return null;
        Transform canvas = view.ToggleRect.parent;
        var raw = canvas.GetComponentInChildren<RawImage>();
        Material controlsMaterial = view.ToggleButton.targetGraphic.material;
        Material cameraMaterial = raw.material;
        Vector3 canvasPosition = canvas.position;
        Quaternion canvasRotation = canvas.rotation;
        Vector3 buttonPosition = view.ToggleRect.position;
        Assert.That(Vector3.Distance(buttonPosition, settings.cameraControlsWorldPosition), Is.LessThan(.0001f));
        Assert.That(canvas.IsChildOf(head.transform), Is.False);
        Assert.That(canvas.IsChildOf(root.transform), Is.False);
        Assert.That(cameraMaterial, Is.Not.SameAs(controlsMaterial), "Stereo selection is image-only");
        Assert.That(controlsMaterial.GetFloat("_StereoSideBySide"), Is.Zero);
        Assert.That(cameraMaterial.GetFloat("_StereoSideBySide"), Is.EqualTo(1));
        Assert.That(canvas.GetComponentsInChildren<Collider>(true), Is.Empty, "Monitor pixels are never targeting surfaces");

        head.transform.SetPositionAndRotation(new Vector3(.25f, 1.8f, -.1f), Quaternion.Euler(14, 35, 0));
        root.transform.SetPositionAndRotation(new Vector3(2, 0, 1), Quaternion.Euler(0, 60, 0));
        yield return null;
        yield return null;
        Assert.That(Vector3.Distance(canvas.position, canvasPosition), Is.LessThan(.0001f));
        Assert.That(Quaternion.Angle(canvas.rotation, canvasRotation), Is.LessThan(.001f));
        Assert.That(Vector3.Distance(view.ToggleRect.position, buttonPosition), Is.LessThan(.0001f));
        Assert.That(view.PointAtToggle(new Ray(head.transform.position, buttonPosition - head.transform.position), out _), Is.True);
        view.ToggleButton.onClick.Invoke();
        Assert.That(view.RobotView, Is.False);
        Assert.That(Vector3.Distance(view.ToggleRect.position, buttonPosition), Is.LessThan(.0001f));
        view.ToggleButton.onClick.Invoke();
        Assert.That(view.RobotView, Is.True);
        Assert.That(raw.enabled, Is.False, "A missing stereo pair must not look live");
        Assert.That(bridge.HasPreview, Is.False);

        UnityEngine.Object.Destroy(view);
        yield return null;
        yield return null;
        Assert.That(canvas == null, Is.True, "Destroying the owner cleans up its scene-root canvas");
        Assert.That(controlsMaterial == null, Is.True);
        Assert.That(cameraMaterial == null, Is.True);
    }

    [UnityTest, Category("ROSCameraIntegration")]
    public IEnumerator RobotCameraFeedAppearsThroughRosAndToggleReleasesIt()
    {
        if (Environment.GetEnvironmentVariable("DEICTIC_CAMERA_INTEGRATION") != "1")
            Assert.Ignore("Requires isolated ROS endpoint, atomic SBS camera relay, and actual Isaac head stereo cameras.");
        Create(true);
        view.ToggleButton.onClick.Invoke();
        float until = Time.realtimeSinceStartup + 30;
        while (!view.Stream.HasFreshFrame && Time.realtimeSinceStartup < until) yield return null;
        Assert.That(view.Stream.HasFreshFrame, Is.True, view.Stream.Status);
        Assert.That(view.Stream.IsStereo, Is.True);
        Assert.That(view.Stream.Texture.width, Is.InRange(2, 960));
        Assert.That(view.Stream.Texture.width % 2, Is.Zero, "An SBS pair must contain two equal-width eyes");
        Assert.That(view.Stream.Texture.height, Is.InRange(1, 360));
        Assert.That(view.Stream.DisplayedFrames, Is.GreaterThan(0));
        yield return null;
        var raw = view.ToggleRect.parent.GetComponentInChildren<RawImage>();
        Assert.That(raw.texture, Is.EqualTo(view.Stream.Texture));
        Assert.That(raw.uvRect.height, Is.EqualTo(-1), "ROS image rows must be displayed upright");
        Assert.That(raw.GetComponent<AspectRatioFitter>().aspectRatio,
            Is.EqualTo((float)view.Stream.Texture.width / (2 * view.Stream.Texture.height)).Within(.0001f));
        Debug.Log($"Actual ROS head stereo view: {view.Stream.Texture.width}x{view.Stream.Texture.height} SBS, frame age {view.Stream.FrameAge:F3}s");
        long firstFrameCount = view.Stream.DisplayedFrames;
        int staleSamples = 0;
        double worstAge = 0;
        until = Time.realtimeSinceStartup + 10;
        while (Time.realtimeSinceStartup < until)
        {
            yield return null;
            if (!view.Stream.HasFreshFrame) staleSamples++;
            worstAge = Math.Max(worstAge, view.Stream.FrameAge);
        }
        long newFrames = view.Stream.DisplayedFrames - firstFrameCount;
        Debug.Log($"Sustained ROS head stereo: {newFrames} new frames in 10s, maximum age {worstAge:F3}s, stale samples {staleSamples}");
        Assert.That(newFrames, Is.GreaterThanOrEqualTo(3), "Live stereo must keep advancing after the first frame");
        Assert.That(staleSamples, Is.Zero, "The live monitor must stay fresh throughout the observation");
        string output = Environment.GetEnvironmentVariable("DEICTIC_CAMERA_SCREENSHOT");
        if (!string.IsNullOrEmpty(output) && SystemInfo.graphicsDeviceType != UnityEngine.Rendering.GraphicsDeviceType.Null)
        {
            var camera = head.GetComponent<Camera>();
            camera.fieldOfView = 70;
            var target = new RenderTexture(1200, 1000, 24);
            camera.targetTexture = target;
            camera.Render();
            var previous = RenderTexture.active;
            RenderTexture.active = target;
            var png = new Texture2D(1200, 1000, TextureFormat.RGB24, false);
            png.ReadPixels(new Rect(0, 0, 1200, 1000), 0, 0);
            png.Apply();
            Directory.CreateDirectory(Path.GetDirectoryName(output));
            File.WriteAllBytes(output, png.EncodeToPNG());
            camera.targetTexture = null;
            RenderTexture.active = previous;
            UnityEngine.Object.Destroy(target);
            UnityEngine.Object.Destroy(png);
        }
        view.Stream.enabled = false;
        yield return null;
        Assert.That(raw.enabled, Is.False, "Disconnected/stale feeds must not remain presented as live");
        view.ToggleButton.onClick.Invoke();
        Assert.That(view.RobotView, Is.False);
        Assert.That(view.ToggleButton.gameObject.activeInHierarchy, Is.True);
    }

    [UnityTearDown]
    public IEnumerator Cleanup()
    {
        if (view && view.Stream) view.Stream.enabled = false;
        if (bridge && bridge.Ros) bridge.Ros.Disconnect();
        UnityEngine.Object.Destroy(root);
        UnityEngine.Object.Destroy(head);
        UnityEngine.Object.Destroy(settings);
        yield return null;
    }
}
