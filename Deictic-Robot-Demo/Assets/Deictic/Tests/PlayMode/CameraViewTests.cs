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
    const int RobotUiLayer = 30;
    GameObject root, head;
    Camera eyeCamera;
    int userCullingMask;
    DeicticBridge bridge;
    DeicticSettings settings;
    DeicticCameraView view;

    void Create(bool connect)
    {
        root = new GameObject("Camera toggle test");
        head = new GameObject("Headset viewpoint");
        head.transform.position = new Vector3(0, 1.7f, 0);
        eyeCamera = head.AddComponent<Camera>();
        eyeCamera.clearFlags = CameraClearFlags.SolidColor;
        // Distinguish restoration of the original mask from Unity's default.
        eyeCamera.cullingMask = ~(1 << 29);
        userCullingMask = eyeCamera.cullingMask;
        settings = ScriptableObject.CreateInstance<DeicticSettings>();
        settings.connectOnStart = connect;
        settings.syntheticScene = true;
        bridge = root.AddComponent<DeicticBridge>();
        bridge.Initialize(settings);
        view = root.AddComponent<DeicticCameraView>();
        view.Initialize(head.transform, bridge);
    }

    [UnityTest]
    public IEnumerator ToggleButtonEntersImmersiveViewAndReturnsToUserWithoutCamera()
    {
        Create(false);
        yield return null;
        Assert.That(view.RobotView, Is.False);
        Assert.That(view.Stream.enabled, Is.False, "Hidden camera must not consume bandwidth");
        Assert.That(eyeCamera.cullingMask, Is.EqualTo(userCullingMask));
        Assert.That(RobotPovRendererFeature.ActiveCamera, Is.Null);
        Ray buttonRay = new Ray(head.transform.position, view.ToggleRect.position - head.transform.position);
        Assert.That(view.PointAtToggle(buttonRay, out Vector3 hit), Is.True);
        Assert.That(Vector3.Distance(hit, view.ToggleRect.position), Is.LessThan(.001f));
        Assert.That(view.PointAtToggle(new Ray(head.transform.position, Vector3.forward), out _), Is.False);

        view.ToggleButton.onClick.Invoke();
        yield return null;
        Assert.That(view.RobotView, Is.True);
        Assert.That(view.Stream.enabled, Is.True);
        Assert.That(eyeCamera.cullingMask, Is.EqualTo(1 << RobotUiLayer),
            "Robot POV should hide the User scene and keep return controls renderable");
        Assert.That(RobotPovRendererFeature.ActiveCamera, Is.SameAs(eyeCamera));
        Assert.That(view.ToggleButton.gameObject.activeInHierarchy, Is.True);
        Assert.That(view.ToggleButton.gameObject.layer, Is.EqualTo(RobotUiLayer));
        Assert.That(view.Stream.HasFreshFrame, Is.False);
        Assert.That(view.ImmersiveMaterial.GetFloat("_HasFrame"), Is.LessThan(.5f),
            "A missing feed must not show a previous robot frame");
        Assert.That(view.StatusText, Is.Not.Null);
        Assert.That(view.StatusText.gameObject.activeInHierarchy, Is.True);
        Assert.That(bridge.HasPreview, Is.False, "Changing view must not request motion");

        view.ToggleButton.onClick.Invoke();
        yield return null;
        Assert.That(view.RobotView, Is.False);
        Assert.That(view.Stream.enabled, Is.False);
        Assert.That(eyeCamera.cullingMask, Is.EqualTo(userCullingMask));
        Assert.That(RobotPovRendererFeature.ActiveCamera, Is.Null);
        Assert.That(view.ToggleButton.gameObject.activeInHierarchy, Is.True);

        view.SetRobotView(true);
        view.enabled = false;
        Assert.That(view.Stream.enabled, Is.False, "Disabling the owner releases its image subscription");
        Assert.That(eyeCamera.cullingMask, Is.EqualTo(userCullingMask));
        Assert.That(RobotPovRendererFeature.ActiveCamera, Is.Null);
        Assert.That(view.ToggleButton.gameObject.activeInHierarchy, Is.False);
        Assert.That(view.PointAtToggle(buttonRay, out _), Is.False);
        view.SetRobotView(true);
        Assert.That(view.Stream.enabled, Is.False, "A disabled view must not reactivate the stream");
        view.enabled = true;
        Assert.That(view.RobotView, Is.False);
        Assert.That(eyeCamera.cullingMask, Is.EqualTo(userCullingMask));
        Assert.That(view.ToggleButton.gameObject.activeInHierarchy, Is.True);
    }

    [UnityTest]
    public IEnumerator ImmersiveFeedHasNoWorldMonitorAndReturnButtonStaysFixed()
    {
        Create(false);
        view.SetRobotView(true);
        yield return null;
        Transform canvas = view.ToggleRect.parent;
        Material controlsMaterial = view.ToggleButton.targetGraphic.material;
        Material immersiveMaterial = view.ImmersiveMaterial;
        Vector3 canvasPosition = canvas.position;
        Quaternion canvasRotation = canvas.rotation;
        Vector3 buttonPosition = view.ToggleRect.position;
        Assert.That(Vector3.Distance(buttonPosition, settings.cameraControlsWorldPosition), Is.LessThan(.0001f));
        Assert.That(canvas.IsChildOf(head.transform), Is.False);
        Assert.That(canvas.IsChildOf(root.transform), Is.False);
        Assert.That(canvas.GetComponentsInChildren<RawImage>(true), Is.Empty,
            "The robot feed should take over the eye view, not occupy a world monitor");
        Assert.That(canvas.GetComponentsInChildren<Collider>(true), Is.Empty);
        Assert.That(immersiveMaterial, Is.Not.Null);
        Assert.That(immersiveMaterial, Is.Not.SameAs(controlsMaterial));
        Assert.That(eyeCamera.cullingMask, Is.EqualTo(1 << RobotUiLayer));
        Assert.That(RobotPovRendererFeature.ActiveCamera, Is.SameAs(eyeCamera));

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
        Assert.That(eyeCamera.cullingMask, Is.EqualTo(userCullingMask));
        view.ToggleButton.onClick.Invoke();
        Assert.That(view.RobotView, Is.True);
        Assert.That(view.ImmersiveMaterial.GetFloat("_HasFrame"), Is.LessThan(.5f));
        Assert.That(bridge.HasPreview, Is.False);

        UnityEngine.Object.Destroy(view);
        yield return null;
        yield return null;
        Assert.That(canvas == null, Is.True, "Destroying the view removes its scene-root controls");
        Assert.That(controlsMaterial == null, Is.True);
        Assert.That(immersiveMaterial == null, Is.True);
        Assert.That(eyeCamera.cullingMask, Is.EqualTo(userCullingMask));
        Assert.That(RobotPovRendererFeature.ActiveCamera, Is.Null);
    }

    [UnityTest, Category("ROSCameraIntegration")]
    public IEnumerator RobotCameraFeedFillsEyeViewThroughRosAndToggleReleasesIt()
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
        Assert.That(view.Stream.Texture.width % 2, Is.Zero, "SBS must contain equal-width eyes");
        Assert.That(view.Stream.Texture.height, Is.InRange(1, 360));
        Assert.That(view.Stream.DisplayedFrames, Is.GreaterThan(0));
        yield return null;
        yield return null;
        Assert.That(view.ImmersiveMaterial.mainTexture, Is.SameAs(view.Stream.Texture));
        Assert.That(view.ImmersiveMaterial.GetFloat("_HasFrame"), Is.GreaterThan(.5f));
        Assert.That(eyeCamera.cullingMask, Is.EqualTo(1 << RobotUiLayer));
        Assert.That(RobotPovRendererFeature.ActiveCamera, Is.SameAs(eyeCamera));
        Debug.Log($"Actual immersive ROS head stereo view: {view.Stream.Texture.width}x{view.Stream.Texture.height} SBS, frame age {view.Stream.FrameAge:F3}s");

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
        Debug.Log($"Sustained immersive ROS head stereo: {newFrames} frames in 10s, maximum age {worstAge:F3}s, stale samples {staleSamples}");
        Assert.That(newFrames, Is.GreaterThanOrEqualTo(3), "Live stereo must keep advancing");
        Assert.That(staleSamples, Is.Zero, "The robot POV must stay fresh");

        string output = Environment.GetEnvironmentVariable("DEICTIC_CAMERA_SCREENSHOT");
        if (!string.IsNullOrEmpty(output) && SystemInfo.graphicsDeviceType != UnityEngine.Rendering.GraphicsDeviceType.Null)
        {
            var target = new RenderTexture(1200, 1000, 24);
            eyeCamera.targetTexture = target;
            eyeCamera.Render();
            var previous = RenderTexture.active;
            RenderTexture.active = target;
            var png = new Texture2D(1200, 1000, TextureFormat.RGB24, false);
            png.ReadPixels(new Rect(0, 0, 1200, 1000), 0, 0);
            png.Apply();
            Directory.CreateDirectory(Path.GetDirectoryName(output));
            File.WriteAllBytes(output, png.EncodeToPNG());
            eyeCamera.targetTexture = null;
            RenderTexture.active = previous;
            UnityEngine.Object.Destroy(target);
            UnityEngine.Object.Destroy(png);
        }

        view.Stream.enabled = false;
        yield return null;
        yield return null;
        Assert.That(view.ImmersiveMaterial.GetFloat("_HasFrame"), Is.LessThan(.5f),
            "A disconnected or stale feed must not remain live");
        Assert.That(view.StatusText.gameObject.activeInHierarchy, Is.True);
        view.ToggleButton.onClick.Invoke();
        Assert.That(view.RobotView, Is.False);
        Assert.That(eyeCamera.cullingMask, Is.EqualTo(userCullingMask));
        Assert.That(RobotPovRendererFeature.ActiveCamera, Is.Null);
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
