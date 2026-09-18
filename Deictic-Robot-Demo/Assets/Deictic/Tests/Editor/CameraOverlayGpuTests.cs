using System;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Actual GPU readback of the shipped material, without an XR device or ROS.
/// Run with a graphics device (not -nographics). This exercises desktop single-
/// pass instancing; it does not claim Android multiview/device verification.
/// </summary>
public sealed class CameraOverlayGpuTests
{
    const int Size = 64;

    [Test, Category("GPU")]
    public void SinglePassImageRoutesLeftAndRightSourcesToDifferentArrayLayers()
    {
        using (var draw = new GpuDraw())
        {
            draw.Render(singlePass: true, sideBySide: true, rosRows: false);
            AssertColor(draw.Read(0, 16, 32), Color.red, "left eye, left side");
            AssertColor(draw.Read(0, 48, 32), Color.red, "left eye, right side");
            AssertColor(draw.Read(1, 16, 32), Color.green, "right eye, left side");
            AssertColor(draw.Read(1, 48, 32), Color.green, "right eye, right side");
        }
    }

    [Test, Category("GPU")]
    public void SinglePassControlsKeepTheirWholeTextureVisibleInBothEyes()
    {
        using (var draw = new GpuDraw())
        {
            draw.Render(singlePass: true, sideBySide: false, rosRows: false);
            for (int eye = 0; eye < 2; eye++)
            {
                AssertColor(draw.Read(eye, 16, 32), Color.red, $"UI left half, eye {eye}");
                AssertColor(draw.Read(eye, 48, 32), Color.green, $"UI right half, eye {eye}");
            }
        }
    }

    [Test, Category("GPU")]
    public void MonoPreviewShowsLeftEyeWithFirstRosRowAtTop()
    {
        using (var draw = new GpuDraw())
        {
            // Uploaded row zero represents the source's TOP row. Mesh UVs use
            // the same vertical reversal as RawImage. Right-eye colors differ,
            // so a whole-SBS or wrong-eye mono preview cannot satisfy this test.
            draw.Render(singlePass: false, sideBySide: true, rosRows: true);
            AssertColor(draw.Read(0, 16, 48), Color.red, "mono top-left / first ROS row");
            AssertColor(draw.Read(0, 48, 48), Color.red, "mono top-right / first ROS row");
            AssertColor(draw.Read(0, 16, 16), Color.blue, "mono bottom-left / last ROS row");
            AssertColor(draw.Read(0, 48, 16), Color.blue, "mono bottom-right / last ROS row");
        }
    }

    static void AssertColor(Color actual, Color expected, string location)
    {
        Assert.That(actual.r, Is.EqualTo(expected.r).Within(.025f), location + " red");
        Assert.That(actual.g, Is.EqualTo(expected.g).Within(.025f), location + " green");
        Assert.That(actual.b, Is.EqualTo(expected.b).Within(.025f), location + " blue");
        Assert.That(actual.a, Is.GreaterThan(.98f), location + " alpha");
    }

    sealed class GpuDraw : IDisposable
    {
        readonly RenderTexture previousTarget;
        readonly bool previousStereoInstancing, previousMultiview;
        readonly Matrix4x4[] previousStereoVP;
        readonly Matrix4x4 previousVP;
        readonly int previousEye;
        RenderTexture target, readableTarget;
        Texture2D source, readback;
        Material material;
        Mesh quad;
        GameObject cameraObject;

        public GpuDraw()
        {
            if (SystemInfo.graphicsDeviceType == GraphicsDeviceType.Null ||
                !SystemInfo.supports2DArrayTextures || !SystemInfo.supportsInstancing ||
                (SystemInfo.copyTextureSupport & CopyTextureSupport.Basic) == 0)
                Assert.Ignore("GPU test requires array textures, instancing and texture copy; rerun without -nographics.");
            previousTarget = RenderTexture.active;
            previousStereoInstancing = Shader.IsKeywordEnabled("STEREO_INSTANCING_ON");
            previousMultiview = Shader.IsKeywordEnabled("STEREO_MULTIVIEW_ON");
            previousStereoVP = Shader.GetGlobalMatrixArray("unity_StereoMatrixVP");
            previousVP = Shader.GetGlobalMatrix("unity_MatrixVP");
            previousEye = Shader.GetGlobalInt("unity_StereoEyeIndex");
        }

        public void Render(bool singlePass, bool sideBySide, bool rosRows)
        {
            Shader shader = Resources.Load<Shader>("Deictic/CameraViewOverlay");
            Assert.That(shader, Is.Not.Null, "Test the shipped Resources shader");
            Assert.That(shader.isSupported, Is.True, "The active GPU must support the shipped shader");
            material = new Material(shader) { enableInstancing = true };
            material.SetFloat("_StereoSideBySide", sideBySide ? 1 : 0);
            material.SetFloat("_ZTest", (float)CompareFunction.Always);
            material.SetColor("_Color", Color.white);
            material.SetVector("_TextureSampleAdd", Vector4.zero);

            source = new Texture2D(4, 4, TextureFormat.RGBA32, false, true)
            { filterMode = FilterMode.Point, wrapMode = TextureWrapMode.Clamp };
            var pixels = new Color[16];
            for (int y = 0; y < 4; y++)
                for (int x = 0; x < 4; x++)
                    pixels[y * 4 + x] = x < 2
                        ? (rosRows && y >= 2 ? Color.blue : Color.red)
                        : (rosRows && y >= 2 ? Color.yellow : Color.green);
            source.SetPixels(pixels);
            source.Apply(false, false);
            material.SetTexture("_MainTex", source);

            quad = new Mesh { name = "Camera overlay GPU test quad" };
            quad.vertices = new[] { new Vector3(-1, -1, .5f), new Vector3(1, -1, .5f),
                new Vector3(-1, 1, .5f), new Vector3(1, 1, .5f) };
            // RawImage's uvRect=(0,1,1,-1): source row zero is at the top.
            quad.uv = new[] { new Vector2(0, 1), new Vector2(1, 1), new Vector2(0, 0), new Vector2(1, 0) };
            quad.colors = new[] { Color.white, Color.white, Color.white, Color.white };
            quad.triangles = new[] { 0, 2, 1, 2, 3, 1 };
            quad.RecalculateBounds();

            target = new RenderTexture(Size, Size, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Linear)
            {
                dimension = singlePass ? TextureDimension.Tex2DArray : TextureDimension.Tex2D,
                volumeDepth = singlePass ? 2 : 1,
                vrUsage = singlePass ? VRTextureUsage.TwoEyes : VRTextureUsage.None,
                antiAliasing = 1,
                filterMode = FilterMode.Point
            };
            Assert.That(target.Create(), Is.True);
            readableTarget = new RenderTexture(Size, Size, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Linear);
            Assert.That(readableTarget.Create(), Is.True);
            readback = new Texture2D(Size, Size, TextureFormat.RGBA32, false, true);

            // Use a real Unity camera convention: +Y is the top of its viewport,
            // it looks along world +Z, and the quad is half a meter in front.
            // Identity clip matrices bypass Unity's render-texture Y conversion
            // on D3D11 and therefore cannot test RawImage's row orientation.
            cameraObject = new GameObject("Camera overlay projection reference")
            { hideFlags = HideFlags.HideAndDontSave };
            var camera = cameraObject.AddComponent<Camera>();
            camera.enabled = false;
            camera.orthographic = true;
            camera.orthographicSize = 1;
            camera.aspect = 1;
            camera.nearClipPlane = .1f;
            camera.farClipPlane = 2;
            camera.targetTexture = target;
            camera.stereoTargetEye = StereoTargetEyeMask.None;
            Matrix4x4 view = camera.worldToCameraMatrix;
            Matrix4x4 projection = camera.projectionMatrix;
            Matrix4x4 gpuVP = GL.GetGPUProjectionMatrix(projection, true) * view;
            Assert.That(camera.WorldToViewportPoint(new Vector3(0, .5f, .5f)).y,
                Is.GreaterThan(.5f), "World +Y must be the camera's upper viewport");

            using (var commands = new CommandBuffer { name = "Camera overlay actual stereo GPU regression" })
            {
                for (int eye = 0; eye < (singlePass ? 2 : 1); eye++)
                {
                    commands.SetRenderTarget(target, 0, CubemapFace.Unknown, eye);
                    commands.ClearRenderTarget(false, true, Color.magenta);
                }
                commands.SetRenderTarget(target, 0, CubemapFace.Unknown, singlePass ? -1 : 0);
                commands.SetViewport(new Rect(0, 0, Size, Size));
                // Match URP's logical camera matrices plus GPU-adjusted VP
                // constants, including its renderIntoTexture=true orientation.
                commands.SetViewProjectionMatrices(view, projection);
                commands.SetGlobalMatrix("unity_MatrixVP", gpuVP);
                commands.SetGlobalMatrixArray("unity_StereoMatrixVP", new[] { gpuVP, gpuVP });
                commands.SetGlobalInt("unity_StereoEyeIndex", 0);
                commands.DisableShaderKeyword("STEREO_MULTIVIEW_ON");
                if (singlePass)
                {
                    commands.SetSinglePassStereo(SinglePassStereoMode.Instancing);
                    commands.EnableShaderKeyword("STEREO_INSTANCING_ON");
                    // Same one-object/two-view draw expansion as URP XRPass.
                    commands.SetInstanceMultiplier(2);
                    commands.DrawMeshInstanced(quad, 0, material, 0, new[] { Matrix4x4.identity }, 1);
                }
                else
                {
                    commands.SetSinglePassStereo(SinglePassStereoMode.None);
                    commands.DisableShaderKeyword("STEREO_INSTANCING_ON");
                    commands.SetInstanceMultiplier(1);
                    commands.DrawMesh(quad, Matrix4x4.identity, material, 0, 0);
                }
                commands.SetSinglePassStereo(SinglePassStereoMode.None);
                commands.SetInstanceMultiplier(1);
                commands.DisableShaderKeyword("STEREO_INSTANCING_ON");
                Graphics.ExecuteCommandBuffer(commands);
            }
        }

        public Color Read(int eye, int x, int y)
        {
            // Copy one actual array layer, never choose its source-eye texture in
            // CPU code. ReadPixels synchronizes the rendered GPU result.
            Graphics.CopyTexture(target, eye, 0, readableTarget, 0, 0);
            RenderTexture.active = readableTarget;
            readback.ReadPixels(new Rect(0, 0, Size, Size), 0, 0, false);
            readback.Apply(false, false);
            return readback.GetPixel(x, y);
        }

        public void Dispose()
        {
            RenderTexture.active = previousTarget;
            using (var reset = new CommandBuffer())
            {
                reset.SetSinglePassStereo(SinglePassStereoMode.None);
                reset.SetInstanceMultiplier(1);
                if (previousStereoInstancing) reset.EnableShaderKeyword("STEREO_INSTANCING_ON");
                else reset.DisableShaderKeyword("STEREO_INSTANCING_ON");
                if (previousMultiview) reset.EnableShaderKeyword("STEREO_MULTIVIEW_ON");
                else reset.DisableShaderKeyword("STEREO_MULTIVIEW_ON");
                if (previousStereoVP != null && previousStereoVP.Length > 0)
                    reset.SetGlobalMatrixArray("unity_StereoMatrixVP", previousStereoVP);
                reset.SetGlobalMatrix("unity_MatrixVP", previousVP);
                reset.SetGlobalInt("unity_StereoEyeIndex", previousEye);
                Graphics.ExecuteCommandBuffer(reset);
            }
            if (target) { target.Release(); UnityEngine.Object.DestroyImmediate(target); }
            if (readableTarget) { readableTarget.Release(); UnityEngine.Object.DestroyImmediate(readableTarget); }
            if (source) UnityEngine.Object.DestroyImmediate(source);
            if (readback) UnityEngine.Object.DestroyImmediate(readback);
            if (material) UnityEngine.Object.DestroyImmediate(material);
            if (quad) UnityEngine.Object.DestroyImmediate(quad);
            if (cameraObject) UnityEngine.Object.DestroyImmediate(cameraObject);
        }
    }
}
