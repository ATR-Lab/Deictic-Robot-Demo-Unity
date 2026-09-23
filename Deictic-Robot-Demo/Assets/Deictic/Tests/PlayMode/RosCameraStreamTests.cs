using System;
using System.Collections;
using System.Reflection;
using Deictic;
using NUnit.Framework;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.TestTools;

/// <summary>Actual main-thread JPEG decode and subscription lifecycle, without ROS services.</summary>
public sealed class RosCameraStreamTests
{
    GameObject root;
    ROSConnection ros;
    RosCameraStream stream;
    double previousOffset;
    const BindingFlags Private = BindingFlags.Instance | BindingFlags.NonPublic;

    void Create(string topic = RosCameraStream.DefaultTopic)
    {
        previousOffset = RosFrames.ClockOffsetSeconds;
        root = new GameObject("Offline camera stream test");
        ros = root.AddComponent<ROSConnection>();
        ros.ConnectOnStart = false;
        stream = root.AddComponent<RosCameraStream>();
        stream.Initialize(ros, topic, .5f, true);
    }

    static CompressedImageMsg Jpeg(Color leftTop, Color leftBottom, Color rightTop, Color rightBottom)
    {
        var image = new Texture2D(64, 32, TextureFormat.RGB24, false);
        var pixels = new Color[64 * 32];
        for (int y = 0; y < 32; y++)
            for (int x = 0; x < 64; x++)
                pixels[y * 64 + x] = x < 32 ? (y >= 16 ? leftTop : leftBottom) : (y >= 16 ? rightTop : rightBottom);
        image.SetPixels(pixels);
        image.Apply();
        byte[] bytes = image.EncodeToJPG(95);
        UnityEngine.Object.Destroy(image);
        return new CompressedImageMsg { header = RosFrames.Header("k1_head_stereo_optical"),
            format = RosJpegPacking.Format, data = bytes };
    }

    static void Stamp(CompressedImageMsg message, double time) => message.header = RosFrames.Header("k1_head_stereo_optical", time);
    void Receive(CompressedImageMsg message) => typeof(RosCameraStream).GetMethod("ReceiveCompressed", Private).Invoke(stream, new object[] { message });
    void Apply() => typeof(RosCameraStream).GetMethod("Update", Private).Invoke(stream, null);

    [UnityTest]
    public IEnumerator LatestJpegDecodesImmediatelyAndKeepsBothEyeRowsUpright()
    {
        Create();
        Assert.That(stream.IsCompressed, Is.True);
        var first = Jpeg(Color.white, Color.black, Color.white, Color.black);
        var latest = Jpeg(Color.red, Color.blue, Color.green, Color.yellow);
        double start = RosFrames.Now - .02;
        Stamp(first, start); Stamp(latest, start + .001);
        Receive(first); Receive(latest);
        Assert.That(stream.Texture, Is.Null, "Callbacks must not invoke native image decoding");
        Apply();
        Assert.That(stream.DroppedFrames, Is.EqualTo(1));
        Assert.That(stream.DisplayedFrames, Is.EqualTo(1));
        Assert.That(stream.TopRowAtTextureYZero, Is.False);
        Assert.That(stream.Texture.width, Is.EqualTo(64));
        Assert.That(stream.Texture.height, Is.EqualTo(32));
        AssertPixel(stream.Texture.GetPixel(16, 24), Color.red);
        AssertPixel(stream.Texture.GetPixel(16, 8), Color.blue);
        AssertPixel(stream.Texture.GetPixel(48, 24), Color.green);
        AssertPixel(stream.Texture.GetPixel(48, 8), Color.yellow);
        Assert.That(stream.EyeAspectRatio, Is.EqualTo(1));

        // A second fresh frame in the same Unity time slice must not wait for
        // the old 5 Hz display poll. Only the current latest frame is applied.
        Stamp(first, start + .002);
        Receive(first); Apply();
        Assert.That(stream.DisplayedFrames, Is.EqualTo(2));
        Stamp(latest, start + .001);
        Receive(latest); Apply();
        Assert.That(stream.DisplayedFrames, Is.EqualTo(2), "Replayed/older captures must not replace the latest frame");
        Assert.That(stream.RejectedFrames, Is.EqualTo(1));
        yield return null;
    }

    [UnityTest]
    public IEnumerator InvalidStaleAndHiddenJpegsCannotReplaceValidTexture()
    {
        Create();
        var image = Jpeg(Color.red, Color.blue, Color.green, Color.yellow);
        Stamp(image, RosFrames.Now - .02);
        Receive(image); Apply();
        var validTexture = stream.Texture;
        long displayed = stream.DisplayedFrames;
        var bad = new CompressedImageMsg { header = RosFrames.Header("k1_head_left_camera_optical"),
            format = image.format, data = image.data };
        Receive(bad); Apply();
        bad.header = RosFrames.Header("k1_head_stereo_optical");
        bad.header.stamp.nanosec = 1000000000;
        Receive(bad); Apply();
        bad.header = RosFrames.Header("k1_head_stereo_optical", RosFrames.Now - 2);
        Receive(bad); Apply();
        Assert.That(stream.RejectedFrames, Is.EqualTo(3));
        Assert.That(stream.Texture, Is.SameAs(validTexture));

        Stamp(image, RosFrames.Now);
        Receive(image);
        RosFrames.ClockOffsetSeconds += 1;
        try { Apply(); }
        finally { RosFrames.ClockOffsetSeconds = previousOffset; }
        Assert.That(stream.RejectedFrames, Is.EqualTo(4), "Recheck age immediately before decoding");
        Assert.That(stream.Texture, Is.SameAs(validTexture));
        Assert.That(stream.DisplayedFrames, Is.EqualTo(displayed));

        Stamp(image, RosFrames.Now + .001);
        Receive(image);
        stream.enabled = false;
        Apply();
        Assert.That(stream.FrameAge, Is.EqualTo(double.PositiveInfinity));
        stream.enabled = true;
        Apply();
        Assert.That(stream.DisplayedFrames, Is.EqualTo(displayed), "Disable/re-enable discards queued frames");
        Stamp(image, RosFrames.Now + .002);
        Receive(image); Apply();
        Assert.That(stream.DisplayedFrames, Is.EqualTo(displayed + 1));
        yield return null;
    }

    [UnityTest]
    public IEnumerator ExplicitRawTopicRetainsLegacyImageAndShaderOrigin()
    {
        Create(RosCameraStream.RawTopic);
        Assert.That(stream.IsCompressed, Is.False);
        var raw = new ImageMsg { header = RosFrames.Header("k1_head_stereo_optical"),
            width = 4, height = 2, encoding = "rgb8", step = 12,
            data = new byte[] { 255,0,0, 255,0,0, 0,255,0, 0,255,0,
                                0,0,255, 0,0,255, 255,255,0, 255,255,0 } };
        typeof(RosCameraStream).GetMethod("Receive", Private).Invoke(stream, new object[] { raw });
        Apply();
        Assert.That(stream.DisplayedFrames, Is.EqualTo(1));
        Assert.That(stream.TopRowAtTextureYZero, Is.True);
        AssertPixel(stream.Texture.GetPixel(0, 0), Color.red);
        AssertPixel(stream.Texture.GetPixel(2, 0), Color.green);
        yield return null;
    }

    static void AssertPixel(Color actual, Color expected)
    {
        Assert.That(actual.r, Is.EqualTo(expected.r).Within(.04));
        Assert.That(actual.g, Is.EqualTo(expected.g).Within(.04));
        Assert.That(actual.b, Is.EqualTo(expected.b).Within(.04));
    }

    [UnityTearDown]
    public IEnumerator Cleanup()
    {
        RosFrames.ClockOffsetSeconds = previousOffset;
        if (stream) stream.enabled = false;
        if (ros) ros.Disconnect();
        UnityEngine.Object.Destroy(root);
        yield return null;
    }
}
