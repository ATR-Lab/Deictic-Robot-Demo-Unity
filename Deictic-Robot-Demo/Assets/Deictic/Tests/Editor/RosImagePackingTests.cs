using Deictic;
using NUnit.Framework;
using RosMessageTypes.Sensor;

public sealed class RosImagePackingTests
{
    [Test]
    public void AtomicStereoPreservesEyeAndRowOrderAndRejectsMonoSources()
    {
        var image = new ImageMsg { width = 4, height = 2, step = 12, encoding = "rgb8",
            data = new byte[] { 255,0,0, 255,0,0, 0,255,0, 0,255,0,
                              0,0,255, 0,0,255, 255,255,0, 255,255,0 } };
        image.header.frame_id = "k1_head_stereo_optical";
        Assert.That(RosImagePacking.IsHeadStereoFrame(image), Is.True);
        byte[] output = null;
        Assert.That(RosImagePacking.TryPackRgba(image, ref output, out _, out _, out string error), Is.True, error);
        Assert.That(output, Is.EqualTo(new byte[] { 255,0,0,255, 255,0,0,255, 0,255,0,255, 0,255,0,255,
                                                  0,0,255,255, 0,0,255,255, 255,255,0,255, 255,255,0,255 }));
        image.header.frame_id = "k1_wrist_camera_optical";
        Assert.That(RosImagePacking.IsHeadStereoFrame(image), Is.False);
        image.header.frame_id = "k1_head_stereo_optical";
        image.width = 3;
        Assert.That(RosImagePacking.IsHeadStereoFrame(image), Is.False, "Unequal eye widths must never display");
        image.width = 962;
        Assert.That(RosImagePacking.IsHeadStereoFrame(image), Is.False, "Bound viewer bandwidth/allocation");
        image.width = 4; image.height = 361;
        Assert.That(RosImagePacking.IsHeadStereoFrame(image), Is.False);
    }

    [TestCase("rgb8")]
    [TestCase("bgr8")]
    [TestCase("rgba8")]
    [TestCase("bgra8")]
    public void ConvertsChannelsAndRowPaddingWithoutFlippingOrReallocating(string encoding)
    {
        int channels = encoding.Contains("a") ? 4 : 3;
        bool bgr = encoding.StartsWith("bgr");
        int stride = 2 * channels + 3;
        var image = new ImageMsg { width = 2, height = 2, step = (uint)stride,
            encoding = encoding, is_bigendian = 1, data = new byte[stride * 2] };
        var expected = new byte[16];
        for (int i = 0; i < image.data.Length; i++) image.data[i] = 222;
        for (int pixel = 0; pixel < 4; pixel++)
        {
            int source = pixel / 2 * stride + pixel % 2 * channels;
            byte r = (byte)(pixel * 4 + 1), g = (byte)(pixel * 4 + 2), b = (byte)(pixel * 4 + 3);
            byte a = channels == 4 ? (byte)(pixel * 4 + 4) : (byte)255;
            image.data[source] = bgr ? b : r;
            image.data[source + 1] = g;
            image.data[source + 2] = bgr ? r : b;
            if (channels == 4) image.data[source + 3] = a;
            expected[pixel * 4] = r; expected[pixel * 4 + 1] = g;
            expected[pixel * 4 + 2] = b; expected[pixel * 4 + 3] = a;
        }
        byte[] output = null;
        Assert.That(RosImagePacking.TryPackRgba(image, ref output, out int width, out int height, out string error), Is.True, error);
        Assert.That(width, Is.EqualTo(2)); Assert.That(height, Is.EqualTo(2));
        Assert.That(output, Is.EqualTo(expected));
        byte[] reusable = output;
        Assert.That(RosImagePacking.TryPackRgba(image, ref output, out _, out _, out error), Is.True, error);
        Assert.That(output, Is.SameAs(reusable));
    }

    [TestCase("zero")]
    [TestCase("axis")]
    [TestCase("pixels")]
    [TestCase("short_stride")]
    [TestCase("short_payload")]
    [TestCase("extra_payload")]
    [TestCase("payload_cap")]
    [TestCase("overflow")]
    [TestCase("encoding")]
    [TestCase("null_data")]
    [TestCase("endianness")]
    public void RejectsInvalidMetadataBeforeAllocatingOutput(string fault)
    {
        var image = new ImageMsg { width = 1, height = 1, step = 3, encoding = "rgb8", data = new byte[3] };
        switch (fault)
        {
            case "zero": image.width = 0; break;
            case "axis": image.width = RosImagePacking.MaxDimension + 1; break;
            case "pixels": image.width = image.height = 2048; image.step = 6144; break;
            case "short_stride": image.step = 2; image.data = new byte[2]; break;
            case "short_payload": image.data = new byte[2]; break;
            case "extra_payload": image.data = new byte[4]; break;
            case "payload_cap": image.step = RosImagePacking.MaxPayloadBytes + 1; break;
            case "overflow": image.width = image.height = image.step = uint.MaxValue; break;
            case "encoding": image.encoding = "32FC1"; break;
            case "null_data": image.data = null; break;
            case "endianness": image.is_bigendian = 2; break;
        }
        byte[] output = { 42 };
        byte[] existing = output;
        Assert.That(RosImagePacking.TryPackRgba(image, ref output, out int width, out int height, out string error), Is.False);
        Assert.That(output, Is.SameAs(existing));
        Assert.That(width, Is.Zero); Assert.That(height, Is.Zero);
        Assert.That(error, Is.Not.Empty);
    }
}
