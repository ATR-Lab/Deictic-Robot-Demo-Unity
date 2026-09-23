using System;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Reads actual eye render-target layers from the shipped full-screen shader.
/// Run with a graphics device. Desktop instancing is not Android multiview.
/// </summary>
public sealed class RobotPovGpuTests
{
    const int Size = 64;

    [Test, Category("GPU")]
    public void FullscreenTriangleFillsBothEyesWithTheirOwnImages()
    {
        using (var draw = new GpuDraw())
        {
            draw.Render(SourceKind.SolidEyes, hasFrame: true, sourceAspect: 1);
            foreach (var point in new[] {
                new Vector2Int(1, 1), new Vector2Int(62, 1),
                new Vector2Int(1, 62), new Vector2Int(62, 62),
                new Vector2Int(32, 32)
            })
            {
                AssertColor(draw.Read(0, point.x, point.y), Color.red, "left eye " + point);
                AssertColor(draw.Read(1, point.x, point.y), Color.green, "right eye " + point);
            }
        }
    }

    [Test, Category("GPU")]
    public void FirstRosRowAppearsAtTopOfBothEyes()
    {
        using (var draw = new GpuDraw())
        {
            draw.Render(SourceKind.RosRows, hasFrame: true, sourceAspect: 1);
            AssertColor(draw.Read(0, 32, 56), Color.red, "left eye, first ROS row");
            AssertColor(draw.Read(0, 32, 8), Color.blue, "left eye, last ROS row");
            AssertColor(draw.Read(1, 32, 56), Color.green, "right eye, first ROS row");
            AssertColor(draw.Read(1, 32, 8), Color.yellow, "right eye, last ROS row");
        }
    }

    [Test, Category("GPU")]
    public void DecodedJpegRowsRemainUprightAndSeparateInBothEyes()
    {
        using (var draw = new GpuDraw())
        {
            draw.Render(SourceKind.JpegRows, hasFrame: true, sourceAspect: 1);
            AssertColor(draw.Read(0, 32, 56), Color.red, "decoded left top");
            AssertColor(draw.Read(0, 32, 8), Color.blue, "decoded left bottom");
            AssertColor(draw.Read(1, 32, 56), Color.green, "decoded right top");
            AssertColor(draw.Read(1, 32, 8), Color.yellow, "decoded right bottom");
        }
    }

    [Test, Category("GPU")]
    public void MissingFrameDrawsOpaqueFallbackOverEveryEyePixel()
    {
        using (var draw = new GpuDraw())
        {
            draw.Render(SourceKind.SolidEyes, hasFrame: false, sourceAspect: 1);
            foreach (int eye in new[] { 0, 1 })
                foreach (var point in new[] {
                    new Vector2Int(1, 1), new Vector2Int(62, 1),
                    new Vector2Int(1, 62), new Vector2Int(62, 62),
                    new Vector2Int(32, 32)
                })
                {
                    Color pixel = draw.Read(eye, point.x, point.y);
                    Assert.That(pixel.r, Is.InRange(0, .04f), "fallback red at " + point);
                    Assert.That(pixel.g, Is.InRange(0, .04f), "fallback green at " + point);
                    Assert.That(pixel.b, Is.InRange(0, .04f), "fallback blue at " + point);
                    Assert.That(pixel.a, Is.GreaterThan(.98f), "fallback must occlude User view");
                }
        }
    }

    [Test, Category("GPU")]
    public void NarrowEyeCenterCropsWideCameraWithoutStretching()
    {
        // Each eye is 8x4 (aspect 2); the target eye is square. Correct fill
        // crops columns 0,1,6,7 and maps a central 2x2 marker to a square
        // spanning approximately x/y 16..48. Fitting or stretching would
        // expose edge markers or shrink the marker horizontally.
        using (var draw = new GpuDraw())
        {
            draw.Render(SourceKind.CropMarkers, hasFrame: true, sourceAspect: 2);
            foreach (int eye in new[] { 0, 1 })
            {
                AssertColor(draw.Read(eye, 4, 32), Color.blue, "cropped left edge");
                AssertColor(draw.Read(eye, 60, 32), Color.blue, "cropped right edge");
                AssertColor(draw.Read(eye, 20, 32), Color.yellow, "square marker left");
                AssertColor(draw.Read(eye, 44, 32), Color.yellow, "square marker right");
                AssertColor(draw.Read(eye, 32, 20), Color.yellow, "square marker bottom");
                AssertColor(draw.Read(eye, 32, 44), Color.yellow, "square marker top");
                AssertColor(draw.Read(eye, 32, 8), Color.blue, "uncropped vertical edge");
                AssertColor(draw.Read(eye, 32, 56), Color.blue, "uncropped vertical edge");
            }
        }
    }

    static void AssertColor(Color actual, Color expected, string location)
    {
        Assert.That(actual.r, Is.EqualTo(expected.r).Within(.025f), location + " red");
        Assert.That(actual.g, Is.EqualTo(expected.g).Within(.025f), location + " green");
        Assert.That(actual.b, Is.EqualTo(expected.b).Within(.025f), location + " blue");
        Assert.That(actual.a, Is.GreaterThan(.98f), location + " alpha");
    }

    enum SourceKind { SolidEyes, RosRows, JpegRows, CropMarkers }

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

        public void Render(SourceKind kind, bool hasFrame, float sourceAspect)
        {
            Shader shader = Resources.Load<Shader>("Deictic/RobotPovFullscreen");
            Assert.That(shader, Is.Not.Null, "Test the shipped Resources shader");
            Assert.That(shader.isSupported, Is.True, "The active GPU must support the shipped shader");
            material = new Material(shader) { enableInstancing = true };
            material.SetFloat("_StereoSideBySide", 1);
            material.SetFloat("_HasFrame", hasFrame ? 1 : 0);
            material.SetFloat("_SourceAspect", sourceAspect);
            material.SetFloat("_ViewportAspect", 1);
            // The URP full-screen vertex uses this dynamic scaling input.
            // D3D render textures use bottom-left destination coordinates.
            material.SetVector("_BlitScaleBias", new Vector4(1, 1, 0, 0));
            int height = kind == SourceKind.CropMarkers ? 4 : 8;
            source = new Texture2D(16, height, TextureFormat.RGBA32, false, true)
            { filterMode = FilterMode.Point, wrapMode = TextureWrapMode.Clamp };
            var pixels = new Color[16 * height];
            for (int y = 0; y < height; y++)
                for (int x = 0; x < 16; x++)
                {
                    int withinEye = x % 8;
                    if (kind == SourceKind.SolidEyes)
                        pixels[y * 16 + x] = x < 8 ? Color.red : Color.green;
                    else if (kind == SourceKind.RosRows || kind == SourceKind.JpegRows)
                        // Pixel row zero represents the first ROS (top) row.
                        // A JPEG source is authored in Unity texture coordinates
                        // before encoding, so its upper half is at high y.
                        pixels[y * 16 + x] = x < 8
                            ? ((kind == SourceKind.JpegRows ? y >= 4 : y < 4) ? Color.red : Color.blue)
                            : ((kind == SourceKind.JpegRows ? y >= 4 : y < 4) ? Color.green : Color.yellow);
                    else
                        pixels[y * 16 + x] = withinEye == 0 || withinEye == 7
                            ? Color.magenta
                            : (withinEye == 3 || withinEye == 4) && (y == 1 || y == 2)
                                ? Color.yellow : Color.blue;
                }
            source.SetPixels(pixels);
            source.Apply(false, false);
            if (kind == SourceKind.JpegRows)
            {
                byte[] jpeg = source.EncodeToJPG(95);
                var message = new RosMessageTypes.Sensor.CompressedImageMsg
                    { format = Deictic.RosJpegPacking.Format, data = jpeg };
                message.header.frame_id = "k1_head_stereo_optical";
                Assert.That(Deictic.RosJpegPacking.TryValidate(message, true, out _, out _, out string error), Is.True, error);
                Assert.That(ImageConversion.LoadImage(source, jpeg, false), Is.True);
                material.SetFloat("_RosTopLeft", 0);
            }
            material.SetTexture("_MainTex", source);

            target = new RenderTexture(Size, Size, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Linear)
            {
                dimension = TextureDimension.Tex2DArray,
                volumeDepth = 2,
                vrUsage = VRTextureUsage.TwoEyes,
                antiAliasing = 1,
                filterMode = FilterMode.Point
            };
            Assert.That(target.Create(), Is.True);
            readableTarget = new RenderTexture(Size, Size, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Linear);
            Assert.That(readableTarget.Create(), Is.True);
            readback = new Texture2D(Size, Size, TextureFormat.RGBA32, false, true);

            using (var commands = new CommandBuffer { name = "Robot POV procedural stereo GPU regression" })
            {
                for (int eye = 0; eye < 2; eye++)
                {
                    commands.SetRenderTarget(target, 0, CubemapFace.Unknown, eye);
                    commands.ClearRenderTarget(false, true, Color.magenta);
                }
                commands.SetRenderTarget(target, 0, CubemapFace.Unknown, -1);
                commands.SetViewport(new Rect(0, 0, Size, Size));
                commands.SetGlobalMatrix("unity_MatrixVP", Matrix4x4.identity);
                commands.SetGlobalMatrixArray("unity_StereoMatrixVP",
                    new[] { Matrix4x4.identity, Matrix4x4.identity });
                commands.SetGlobalInt("unity_StereoEyeIndex", 0);
                commands.DisableShaderKeyword("STEREO_MULTIVIEW_ON");
                commands.SetSinglePassStereo(SinglePassStereoMode.Instancing);
                commands.EnableShaderKeyword("STEREO_INSTANCING_ON");
                commands.SetInstanceMultiplier(2);
                commands.DrawProcedural(Matrix4x4.identity, material, 0, MeshTopology.Triangles, 3, 1);
                commands.SetSinglePassStereo(SinglePassStereoMode.None);
                commands.SetInstanceMultiplier(1);
                commands.DisableShaderKeyword("STEREO_INSTANCING_ON");
                Graphics.ExecuteCommandBuffer(commands);
            }
        }

        public Color Read(int eye, int x, int y)
        {
            // Copy the actual XR target layer. No CPU-side eye selection.
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
        }
    }
}
