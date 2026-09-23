#if UNITY_EDITOR
using System;
using System.Collections;
using System.IO;
using Deictic;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.TestTools;

/// <summary>Exercises the installed URP feature, UI ordering and actual camera output.</summary>
public sealed class RobotPovRenderTests
{
    const int Width = 320, Height = 240;
    GameObject root, head, otherHead, occluder;
    DeicticSettings settings;
    DeicticBridge bridge;
    DeicticCameraView view;
    Camera camera;
    RenderTexture target, previousTarget;
    Texture2D source, pixels;
    RenderPipelineAsset previousPipeline;
    bool pipelineChanged;

    [UnityTest, Category("GPU")]
    public IEnumerator DesktopRendererTakesOverViewAndKeepsReturnControl() => CheckRenderer("PC");

    [UnityTest, Category("GPU")]
    public IEnumerator QuestRendererTakesOverViewAndKeepsReturnControl() => CheckRenderer("Mobile");

    IEnumerator CheckRenderer(string profile)
    {
        if (SystemInfo.graphicsDeviceType == GraphicsDeviceType.Null)
            Assert.Ignore("Requires a graphics device, rerun without -nographics.");
        previousPipeline = QualitySettings.renderPipeline;
        previousTarget = RenderTexture.active;
        var pipeline = AssetDatabase.LoadAssetAtPath<RenderPipelineAsset>($"Assets/Settings/{profile}_RPAsset.asset");
        Assert.That(pipeline, Is.Not.Null);
        QualitySettings.renderPipeline = pipeline;
        pipelineChanged = true;
        root = new GameObject("Fullscreen rendering test");
        head = new GameObject("Test eye");
        head.transform.position = new Vector3(0, 1.45f, 0);
        camera = head.AddComponent<Camera>();
        camera.enabled = false;
        camera.clearFlags = CameraClearFlags.SolidColor;
        camera.backgroundColor = Color.magenta;
        camera.fieldOfView = 60;
        camera.nearClipPlane = .05f;
        camera.farClipPlane = 5;
        camera.stereoTargetEye = StereoTargetEyeMask.None;
        target = new RenderTexture(Width, Height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Linear);
        target.Create();
        camera.targetTexture = target;
        camera.aspect = (float)Width / Height;
        pixels = new Texture2D(Width, Height, TextureFormat.RGBA32, false, true);
        source = new Texture2D(16, 6, TextureFormat.RGBA32, false, true) { filterMode = FilterMode.Point };
        var colors = new Color[96];
        for (int y = 0; y < 6; y++)
            for (int x = 0; x < 16; x++)
                colors[y * 16 + x] = x < 8 ? (y < 3 ? Color.red : Color.blue) : Color.green;
        source.SetPixels(colors);
        source.Apply();
        settings = ScriptableObject.CreateInstance<DeicticSettings>();
        settings.syntheticScene = true;
        settings.connectOnStart = false;
        bridge = root.AddComponent<DeicticBridge>();
        bridge.Initialize(settings);
        view = root.AddComponent<DeicticCameraView>();
        view.Initialize(head.transform, bridge);

        // This transparent scene geometry would leak over the feed if only an
        // opaque-background pass changed and the normal scene still rendered.
        occluder = GameObject.CreatePrimitive(PrimitiveType.Quad);
        occluder.transform.SetPositionAndRotation(new Vector3(0, 1.45f, .5f), Quaternion.identity);
        occluder.transform.localScale = Vector3.one * 4;
        occluder.GetComponent<Renderer>().sharedMaterial = DemoVisuals.Material(new Color(1, 0, 1, .8f));
        yield return null;
        Render();
        AssertColor(pixels.GetPixel(3, Height - 4), Color.magenta, "User view before takeover");

        view.SetRobotView(true);
        yield return null;
        SetFixtureFrame();
        Render();
        CheckCorners();
        Vector3 buttonSample = camera.WorldToViewportPoint(view.ToggleRect.TransformPoint(new Vector3(-250, 0, 0)));
        var buttonPixel = pixels.GetPixel((int)(buttonSample.x * Width), (int)(buttonSample.y * Height));
        Color buttonColor = view.ToggleButton.targetGraphic.color;
        if (QualitySettings.activeColorSpace == ColorSpace.Linear) buttonColor = buttonColor.linear;
        AssertColor(buttonPixel, buttonColor, "Return button must render above the feed");
        Assert.That(view.PointAtToggle(new Ray(head.transform.position, view.ToggleRect.position - head.transform.position), out _), Is.True);
        SaveArtifact(profile + "-robot-pov.png");

        // No screen edges or parallax appear when the head moves away from UI.
        head.transform.SetPositionAndRotation(new Vector3(.3f, 1.8f, -.2f), Quaternion.Euler(25, 110, 12));
        Render();
        CheckCorners();

        // Other cameras never inherit this view, including the editor Scene view.
        otherHead = new GameObject("Unrelated camera");
        var other = otherHead.AddComponent<Camera>();
        other.enabled = false;
        other.clearFlags = CameraClearFlags.SolidColor;
        other.backgroundColor = Color.green;
        other.cullingMask = 0;
        other.targetTexture = target;
        other.Render();
        Read();
        AssertColor(pixels.GetPixel(Width / 2, Height / 2), Color.green, "Unrelated camera");

        view.ImmersiveMaterial.SetFloat("_HasFrame", 0);
        Render();
        var stale = pixels.GetPixel(Width / 2, Height / 2);
        Assert.That(stale.maxColorComponent, Is.LessThan(.08f), "Stale source must become an opaque waiting field");
        Assert.That(stale.a, Is.GreaterThan(.98f));
        view.SetRobotView(false);
        Render();
        AssertColor(pixels.GetPixel(Width / 2, Height / 2), Color.magenta, "User view restored");
    }

    void SetFixtureFrame()
    {
        view.ImmersiveMaterial.mainTexture = source;
        view.ImmersiveMaterial.SetFloat("_HasFrame", 1);
        view.ImmersiveMaterial.SetFloat("_SourceAspect", 4f / 3f);
    }
    void Render() { Canvas.ForceUpdateCanvases(); camera.Render(); Read(); }
    void Read()
    {
        RenderTexture.active = target;
        pixels.ReadPixels(new Rect(0, 0, Width, Height), 0, 0);
        pixels.Apply();
        RenderTexture.active = previousTarget;
    }
    void CheckCorners()
    {
        AssertColor(pixels.GetPixel(3, Height - 4), Color.red, "top-left / first ROS rows");
        AssertColor(pixels.GetPixel(Width - 4, Height - 4), Color.red, "top-right / first ROS rows");
        AssertColor(pixels.GetPixel(3, 3), Color.blue, "bottom-left / last ROS rows");
        AssertColor(pixels.GetPixel(Width - 4, 3), Color.blue, "bottom-right / last ROS rows");
    }
    static void AssertColor(Color actual, Color expected, string location)
    {
        Assert.That(actual.r, Is.EqualTo(expected.r).Within(.03f), location + " red");
        Assert.That(actual.g, Is.EqualTo(expected.g).Within(.03f), location + " green");
        Assert.That(actual.b, Is.EqualTo(expected.b).Within(.03f), location + " blue");
        Assert.That(actual.a, Is.GreaterThan(.98f), location + " alpha");
    }
    void SaveArtifact(string filename)
    {
        string directory = Environment.GetEnvironmentVariable("DEICTIC_POV_ARTIFACT_DIR");
        if (string.IsNullOrWhiteSpace(directory)) return;
        Directory.CreateDirectory(directory);
        File.WriteAllBytes(Path.Combine(directory, filename), pixels.EncodeToPNG());
    }
    [UnityTearDown]
    public IEnumerator Cleanup()
    {
        if (view) view.SetRobotView(false);
        if (bridge && bridge.Ros) bridge.Ros.Disconnect();
        if (camera) camera.targetTexture = null;
        if (otherHead) otherHead.GetComponent<Camera>().targetTexture = null;
        if (occluder) UnityEngine.Object.Destroy(occluder.GetComponent<Renderer>().sharedMaterial);
        UnityEngine.Object.Destroy(root);
        UnityEngine.Object.Destroy(head);
        UnityEngine.Object.Destroy(otherHead);
        UnityEngine.Object.Destroy(occluder);
        UnityEngine.Object.Destroy(settings);
        UnityEngine.Object.Destroy(source);
        UnityEngine.Object.Destroy(pixels);
        if (target) { target.Release(); UnityEngine.Object.Destroy(target); }
        RenderTexture.active = previousTarget;
        if (pipelineChanged) QualitySettings.renderPipeline = previousPipeline;
        yield return null;
    }
}
#endif
