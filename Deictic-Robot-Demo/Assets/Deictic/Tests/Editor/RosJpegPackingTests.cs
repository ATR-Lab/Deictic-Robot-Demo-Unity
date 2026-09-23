using System;
using Deictic;
using NUnit.Framework;
using RosMessageTypes.Sensor;

public sealed class RosJpegPackingTests
{
    // Minimal marker stream sufficient for metadata inspection. Pixel decoding
    // is exercised with actual encoder output by GPU and PlayMode tests.
    static CompressedImageMsg Frame(int width = 960, int height = 360)
    {
        var message = new CompressedImageMsg { format = RosJpegPacking.Format, data = new byte[] {
            0xff, 0xd8,
            0xff, 0xc0, 0, 17, 8, (byte)(height >> 8), (byte)height, (byte)(width >> 8), (byte)width, 3,
            1, 0x11, 0, 2, 0x11, 0, 3, 0x11, 0,
            0xff, 0xda, 0, 12, 3, 1, 0, 2, 0, 3, 0, 0, 63, 0,
            1, 2, 0xff, 0, 3, 0xff, 0xd0, 4, 0xff, 0xd9 } };
        message.header.frame_id = "k1_head_stereo_optical";
        return message;
    }

    [Test]
    public void BoundedColourStereoHeaderAcceptsStuffedEntropyAndRestartMarkers()
    {
        Assert.That(RosJpegPacking.TryValidate(Frame(), true, out int width, out int height, out string error), Is.True, error);
        Assert.That(width, Is.EqualTo(960));
        Assert.That(height, Is.EqualTo(360));
    }

    [TestCase("oversize_width")]
    [TestCase("oversize_height")]
    [TestCase("odd_width")]
    [TestCase("zero_height")]
    [TestCase("mono")]
    [TestCase("mono_source")]
    [TestCase("format")]
    [TestCase("payload")]
    [TestCase("truncated")]
    [TestCase("segment")]
    [TestCase("nested_frame")]
    [TestCase("height_change")]
    [TestCase("trailing_image")]
    public void RejectsMalformedOrUnboundedJpegBeforeNativeDecode(string fault)
    {
        var message = Frame();
        switch (fault)
        {
            case "oversize_width": message = Frame(962); break;
            case "oversize_height": message = Frame(960, 361); break;
            case "odd_width": message = Frame(959); break;
            case "zero_height": message = Frame(960, 0); break;
            case "mono": message.data[11] = 1; break;
            case "mono_source": message.header.frame_id = "k1_head_left_camera_optical"; break;
            case "format": message.format = "png"; break;
            case "payload": message.data = new byte[RosJpegPacking.MaxPayloadBytes + 1]; break;
            case "truncated":
                byte[] truncated = message.data;
                Array.Resize(ref truncated, truncated.Length - 1);
                message.data = truncated;
                break;
            case "segment": message.data[4] = 255; message.data[5] = 255; break;
            case "nested_frame":
                byte[] secondFrame = new byte[19];
                Buffer.BlockCopy(message.data, 2, secondFrame, 0, secondFrame.Length);
                InsertBeforeEnd(message, secondFrame);
                break;
            case "height_change": InsertBeforeEnd(message, new byte[] { 0xff, 0xdc, 0, 4, 0xff, 0xff }); break;
            case "trailing_image":
                byte[] extra = new byte[message.data.Length * 2];
                Buffer.BlockCopy(message.data, 0, extra, 0, message.data.Length);
                Buffer.BlockCopy(message.data, 0, extra, message.data.Length, message.data.Length);
                message.data = extra;
                break;
        }
        Assert.That(RosJpegPacking.TryValidate(message, true, out int width, out int height, out string error), Is.False);
        Assert.That(width, Is.Zero);
        Assert.That(height, Is.Zero);
        Assert.That(error, Is.Not.Empty);
    }

    static void InsertBeforeEnd(CompressedImageMsg message, byte[] segment)
    {
        byte[] original = message.data;
        var expanded = new byte[original.Length + segment.Length];
        Buffer.BlockCopy(original, 0, expanded, 0, original.Length - 2);
        Buffer.BlockCopy(segment, 0, expanded, original.Length - 2, segment.Length);
        expanded[expanded.Length - 2] = 0xff;
        expanded[expanded.Length - 1] = 0xd9;
        message.data = expanded;
    }
}
