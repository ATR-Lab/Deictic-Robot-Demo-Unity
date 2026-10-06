using System;
using System.Threading;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Deictic
{
    /// <summary>Visible-only, latest-frame robot camera on the shared ROS connection.</summary>
    [DisallowMultipleComponent]
    [DefaultExecutionOrder(50)]
    public sealed class RosCameraStream : MonoBehaviour
    {
        public const string RawTopic = "/deictic/camera_view/stereo/image_raw";
        public const string DefaultTopic = RawTopic + "/compressed";
        public const string HardwareMonoTopic = "/transition/hardware/head/image_raw/compressed";
        readonly object pendingLock = new object();
        ROSConnection ros;
        string topic = DefaultTopic;
        float timeout = 1;
        bool subscribed, accepting;
        sealed class Frame
        {
            public ImageMsg raw;
            public CompressedImageMsg jpeg;
            public double stamp;
            public int width, height;
        }
        Frame pending;
        byte[] pixels;
        Texture2D texture, decodeTexture;
        double newestStamp = double.NegativeInfinity;
        double displayedStamp = double.NegativeInfinity;
        string lastError = "";
        long received, displayed, dropped, rejected;
        int displayClockEpoch;

        public Texture2D Texture => texture;
        public bool IsStereo { get; private set; }
        public bool IsCompressed { get; private set; }
        public bool DiagnosticReceiptOnly { get; private set; }
        public TransitionDisplayClock DisplayClock { get; private set; }
        // Raw ROS pixels have row zero at texture y=0. Unity's JPEG decoder
        // produces its normal bottom-left texture layout, so it needs no flip.
        public bool TopRowAtTextureYZero { get; private set; } = true;
        public float EyeAspectRatio => texture == null ? 4f / 3f : (float)texture.width / ((IsStereo ? 2 : 1) * texture.height);
        public double FrameAge => double.IsNegativeInfinity(displayedStamp)
            ? double.PositiveInfinity : Math.Max(0, CameraNow - displayedStamp);
        double CameraNow => DiagnosticReceiptOnly ? Time.realtimeSinceStartupAsDouble : DisplayClock != null ?
            RosFrames.LocalNow + DisplayClock.OffsetSeconds : RosFrames.Now;
        bool DisplayClockReady => DisplayClock == null || DisplayClock.IsReady(RosFrames.LocalNow, Time.realtimeSinceStartupAsDouble);
        public bool HasFreshFrame => isActiveAndEnabled && subscribed && texture != null &&
            ros != null && ros.HasConnectionThread && !ros.HasConnectionError &&
            DisplayClockReady && (DisplayClock == null || displayClockEpoch == DisplayClock.Epoch) &&
            CameraNow - displayedStamp >= -.25 && FrameAge <= timeout;
        public string LastError => lastError;
        public long ReceivedFrames => Interlocked.Read(ref received);
        public long DisplayedFrames => Interlocked.Read(ref displayed);
        public long DroppedFrames => Interlocked.Read(ref dropped);
        public long RejectedFrames => Interlocked.Read(ref rejected);
        public string Status
        {
            get
            {
                if (!isActiveAndEnabled) return "Robot camera paused";
                if (ros == null) return "Camera stream not initialized";
                if (!ros.HasConnectionThread || ros.HasConnectionError) return "Waiting for ROS camera connection";
                if (!DisplayClockReady) return "Waiting for bounded simulator display clock · no task authority implied";
                if (!string.IsNullOrEmpty(lastError)) return lastError;
                if (texture == null || double.IsPositiveInfinity(FrameAge)) return "Waiting for robot camera frame";
                if (DiagnosticReceiptOnly) return HasFreshFrame ?
                    $"K1 mono diagnostic · received {FrameAge:F2}s ago · capture freshness unverified" : "K1 diagnostic camera receipt timeout";
                if (DisplayClock != null && HasFreshFrame) return
                    $"Simulator camera · {FrameAge:F2}s old · display clock uncertainty ≤{DisplayClock.UncertaintySeconds * 1000:F0}ms";
                return HasFreshFrame ? $"Robot camera {texture.width}×{texture.height} · {FrameAge:F2}s old"
                    : $"Robot camera frame stale · {FrameAge:F2}s old";
            }
        }

        public void Initialize(ROSConnection connection, string imageTopic = DefaultTopic, float timeout = 1f, bool stereoSideBySide = false,
            bool diagnosticReceiptOnly = false)
        {
            if (connection == null) throw new ArgumentNullException(nameof(connection));
            if (string.IsNullOrWhiteSpace(imageTopic)) throw new ArgumentException("A camera topic is required", nameof(imageTopic));
            if (!float.IsFinite(timeout) || timeout <= 0) throw new ArgumentOutOfRangeException(nameof(timeout));
            if (diagnosticReceiptOnly && (imageTopic != HardwareMonoTopic || stereoSideBySide))
                throw new ArgumentException("Diagnostic receipt timing requires the dedicated hardware mono topic");
            Unsubscribe();
            ros = connection; topic = imageTopic; this.timeout = timeout;
            IsStereo = stereoSideBySide;
            DiagnosticReceiptOnly = diagnosticReceiptOnly;
            IsCompressed = topic.EndsWith("/compressed", StringComparison.Ordinal);
            if (isActiveAndEnabled) Subscribe();
        }

        void OnEnable() { if (ros != null) Subscribe(); }
        void OnDisable() => Unsubscribe();

        public void UseDisplayClock(TransitionDisplayClock clock)
        {
            if (DiagnosticReceiptOnly) throw new InvalidOperationException("Hardware diagnostics use receipt timing, never a simulation clock");
            DisplayClock = clock ?? throw new ArgumentNullException(nameof(clock));
            displayClockEpoch = DisplayClock.Epoch;
            lock (pendingLock) { pending = null; newestStamp = double.NegativeInfinity; }
            displayedStamp = double.NegativeInfinity;
        }

        void Subscribe()
        {
            if (subscribed) return;
            lock (pendingLock)
            {
                pending = null; accepting = true; newestStamp = double.NegativeInfinity;
            }
            displayedStamp = double.NegativeInfinity;
            lastError = "";
            if (IsCompressed) ros.Subscribe<CompressedImageMsg>(topic, ReceiveCompressed);
            else ros.Subscribe<ImageMsg>(topic, Receive);
            subscribed = true;
        }

        void Unsubscribe()
        {
            lock (pendingLock) { accepting = false; pending = null; }
            // Connector 0.7 owns subscribers per topic; this component owns this
            // image-topic subscription only. Never disconnect the shared socket.
            if (subscribed && ros != null) ros.Unsubscribe(topic);
            subscribed = false;
            displayedStamp = double.NegativeInfinity;
        }

        void Receive(ImageMsg message)
        {
            CheckDisplayClockEpoch();
            // Connector 0.7 dispatches from Update. A lock also makes a future
            // threaded dispatcher safe; texture APIs are used only in Update.
            Interlocked.Increment(ref received);
            if (!TryStamp(message?.header, out double stamp)) return;
            if (message.width == 0 || message.height == 0 ||
                message.width > RosImagePacking.MaxDimension || message.height > RosImagePacking.MaxDimension ||
                message.data == null || message.data.Length > RosImagePacking.MaxPayloadBytes)
            {
                Interlocked.Increment(ref rejected);
                lastError = "Invalid robot camera image dimensions or payload";
                return;
            }
            if (IsStereo && !RosImagePacking.IsHeadStereoFrame(message))
            {
                Interlocked.Increment(ref rejected);
                lastError = "Expected an atomic left/right head-camera pair";
                return;
            }
            Queue(new Frame { raw = message, stamp = stamp });
        }

        void ReceiveCompressed(CompressedImageMsg message)
        {
            CheckDisplayClockEpoch();
            Interlocked.Increment(ref received);
            if (!TryStamp(message?.header, out double stamp)) return;
            int width, height; string error;
            bool valid = DiagnosticReceiptOnly ? RosJpegPacking.TryValidateHardwareDiagnostic(message, out width, out height, out error) :
                RosJpegPacking.TryValidate(message, IsStereo, out width, out height, out error);
            if (!valid)
            {
                Interlocked.Increment(ref rejected);
                lastError = error;
                return;
            }
            Queue(new Frame { jpeg = message, stamp = stamp, width = width, height = height });
        }

        bool TryStamp(RosMessageTypes.Std.HeaderMsg header, out double stamp)
        {
            stamp = 0;
            if (header?.stamp == null || header.stamp.sec < 0 || header.stamp.nanosec >= 1000000000)
            {
                Interlocked.Increment(ref rejected);
                lastError = "Invalid robot camera acquisition timestamp";
                return false;
            }
            stamp = header.stamp.sec + header.stamp.nanosec * 1e-9;
            // Hardware vendor stamps have no certified acquisition/clock mapping. This
            // observer reports only receipt age and never feeds task evidence or control.
            if (DiagnosticReceiptOnly) { stamp = Time.realtimeSinceStartupAsDouble; return true; }
            if (stamp > 0 && Fresh(stamp)) return true;
            Interlocked.Increment(ref rejected);
            lastError = "Robot camera timestamp stale or clocks unsynchronized";
            return false;
        }

        bool Fresh(double stamp)
        {
            double age = CameraNow - stamp;
            return DisplayClockReady && double.IsFinite(age) && age >= -.25 && age <= timeout;
        }

        void CheckDisplayClockEpoch()
        {
            if (DisplayClock == null || displayClockEpoch == DisplayClock.Epoch) return;
            lock (pendingLock) { pending = null; newestStamp = double.NegativeInfinity; }
            displayedStamp = double.NegativeInfinity; displayClockEpoch = DisplayClock.Epoch;
        }

        void Queue(Frame frame)
        {
            lock (pendingLock)
            {
                if (!accepting) return;
                if (frame.stamp <= newestStamp)
                {
                    Interlocked.Increment(ref rejected);
                    return;
                }
                if (pending != null) Interlocked.Increment(ref dropped);
                pending = frame; newestStamp = frame.stamp;
            }
        }

        void Update()
        {
            CheckDisplayClockEpoch();
            // Run after the Connector Update and before the POV material update.
            // No display poll delay: decode only the newest queued frame once.
            if (!subscribed) return;
            Frame frame;
            lock (pendingLock) { frame = pending; pending = null; }
            if (frame == null) return;
            if (!Fresh(frame.stamp))
            {
                Interlocked.Increment(ref rejected);
                lastError = "Robot camera timestamp stale or clocks unsynchronized";
                return;
            }
            bool topRowAtZero;
            if (frame.jpeg != null)
            {
                if (decodeTexture == null) decodeTexture = NewTexture(2, 2);
                // Headers, encoded byte count and allocation dimensions were
                // validated before entering Unity's native JPEG decoder.
                bool decoded;
                try { decoded = ImageConversion.LoadImage(decodeTexture, frame.jpeg.data, false); }
                catch (ArgumentException) { decoded = false; }
                catch (UnityException) { decoded = false; }
                if (!decoded ||
                    decodeTexture.width != frame.width || decodeTexture.height != frame.height)
                {
                    Interlocked.Increment(ref rejected);
                    lastError = "Invalid robot camera JPEG";
                    return;
                }
                topRowAtZero = false;
            }
            else
            {
                if (!RosImagePacking.TryPackRgba(frame.raw, ref pixels, out int width, out int height, out string error))
                {
                    Interlocked.Increment(ref rejected); lastError = error; return;
                }
                if (decodeTexture == null || decodeTexture.width != width || decodeTexture.height != height ||
                    decodeTexture.format != TextureFormat.RGBA32)
                {
                    if (decodeTexture != null) Destroy(decodeTexture);
                    decodeTexture = NewTexture(width, height);
                }
                decodeTexture.LoadRawTextureData(pixels);
                decodeTexture.Apply(false, false);
                topRowAtZero = true;
            }
            // Decoding may itself outlive the frame lease. Keep the previously
            // displayed texture intact until a complete fresh decode succeeds.
            if (!Fresh(frame.stamp))
            {
                Interlocked.Increment(ref rejected);
                lastError = "Robot camera frame expired during decoding";
                return;
            }
            var previous = texture; texture = decodeTexture; decodeTexture = previous;
            TopRowAtTextureYZero = topRowAtZero;
            displayedStamp = frame.stamp; lastError = "";
            Interlocked.Increment(ref displayed);
        }

        Texture2D NewTexture(int width, int height) => new Texture2D(width, height, TextureFormat.RGBA32, false, false)
        {
            name = IsStereo ? "Live K1 head stereo" : "Live K1 head camera", wrapMode = TextureWrapMode.Clamp,
            filterMode = FilterMode.Bilinear, hideFlags = HideFlags.DontSave
        };

        void OnDestroy()
        {
            Unsubscribe();
            if (texture != null) Destroy(texture);
            if (decodeTexture != null) Destroy(decodeTexture);
            pixels = null;
        }
    }

    /// <summary>Validate bounded, atomic colour JPEGs before native decoding.</summary>
    public static class RosJpegPacking
    {
        public const int MaxPayloadBytes = 512 * 1024;
        public const string Format = "rgb8; jpeg compressed bgr8";
        public static bool TryValidateHardwareDiagnostic(CompressedImageMsg image, out int width, out int height, out string error)
        {
            width = height = 0;
            error = "Expected the K1 diagnostic head colour optical frame";
            if (image?.header?.frame_id != "head_color_optical_frame") return false;
            return TryValidate(image, false, out width, out height, out error);
        }

        public static bool TryValidate(CompressedImageMsg image, bool stereo,
            out int width, out int height, out string error)
        {
            width = height = 0;
            error = "Invalid robot camera JPEG header or payload";
            byte[] data = image?.data;
            if (image?.format != Format || data == null || data.Length < 4 || data.Length > MaxPayloadBytes ||
                data[0] != 0xff || data[1] != 0xd8) return false;
            if (stereo && image.header?.frame_id != "k1_head_stereo_optical")
            { error = "Expected an atomic left/right head-camera pair"; return false; }
            int cursor = 2, frameWidth = 0, frameHeight = 0;
            bool scan = false, sawScan = false, sawFrame = false;
            while (cursor < data.Length)
            {
                if (scan)
                {
                    while (cursor < data.Length && data[cursor] != 0xff) cursor++;
                    if (cursor == data.Length) return false;
                }
                if (data[cursor++] != 0xff) return false;
                while (cursor < data.Length && data[cursor] == 0xff) cursor++;
                if (cursor == data.Length) return false;
                int marker = data[cursor++];
                if (scan && (marker == 0 || (marker >= 0xd0 && marker <= 0xd7))) continue;
                if (marker == 0xd9)
                {
                    if (!sawFrame || !sawScan || cursor != data.Length) return false;
                    width = frameWidth; height = frameHeight; error = ""; return true;
                }
                // Reject nested images, changing dimensions (DNL), non-colour
                // arithmetic/lossless JPEGs and malformed standalone markers.
                if (marker == 0 || marker == 1 || marker == 0xd8 || marker == 0xdc ||
                    (marker >= 0xd0 && marker <= 0xd7) || cursor + 2 > data.Length) return false;
                int length = (data[cursor] << 8) | data[cursor + 1];
                if (length < 2 || length > data.Length - cursor) return false;
                bool sof = marker >= 0xc0 && marker <= 0xcf && marker != 0xc4 && marker != 0xc8 && marker != 0xcc;
                if (sof)
                {
                    if (sawFrame || (marker != 0xc0 && marker != 0xc2) || length != 17 ||
                        data[cursor + 2] != 8 || data[cursor + 7] != 3) return false;
                    frameHeight = (data[cursor + 3] << 8) | data[cursor + 4];
                    frameWidth = (data[cursor + 5] << 8) | data[cursor + 6];
                    if (frameWidth < 1 || frameWidth > 960 || frameHeight < 1 || frameHeight > 360 ||
                        (stereo && (frameWidth < 2 || frameWidth % 2 != 0))) return false;
                    sawFrame = true;
                }
                if (marker == 0xda)
                {
                    if (!sawFrame || length < 6) return false;
                    sawScan = true;
                }
                scan = marker == 0xda;
                cursor += length;
            }
            return false;
        }
    }

    /// <summary>Bounded, allocation-reusing packing independent of Unity textures.</summary>
    public static class RosImagePacking
    {
        public const int MaxDimension = 2048;
        public const int MaxPixels = 2097152;
        public const int MaxPayloadBytes = 16777216;

        // The relay owns synchronization. A dedicated frame id and even width
        // prevent accidentally displaying a mono/wrist feed as two eye images.
        public static bool IsHeadStereoFrame(ImageMsg image) => image != null &&
            image.header?.frame_id == "k1_head_stereo_optical" &&
            image.width >= 2 && image.width <= 960 && image.width % 2 == 0 &&
            image.height > 0 && image.height <= 360 && image.encoding == "rgb8";

        public static bool TryPackRgba(ImageMsg image, ref byte[] output, out int width, out int height, out string error)
        {
            width = height = 0;
            error = "Invalid robot camera image dimensions or payload";
            if (image == null || image.width == 0 || image.height == 0 ||
                image.width > MaxDimension || image.height > MaxDimension || image.is_bigendian > 1) return false;
            int channels;
            bool bgr;
            switch (image.encoding)
            {
                case "rgb8": channels = 3; bgr = false; break;
                case "bgr8": channels = 3; bgr = true; break;
                case "rgba8": channels = 4; bgr = false; break;
                case "bgra8": channels = 4; bgr = true; break;
                default: error = "Unsupported robot camera encoding: " + image.encoding; return false;
            }
            long count = (long)image.width * image.height;
            long payload = (long)image.step * image.height;
            if (count > MaxPixels || image.step < (long)image.width * channels ||
                payload > MaxPayloadBytes || image.data == null || image.data.LongLength != payload) return false;
            width = (int)image.width; height = (int)image.height;
            int size = (int)count * 4;
            if (output == null || output.Length != size) output = new byte[size];
            for (int row = 0; row < height; row++)
            {
                int source = row * (int)image.step, destination = row * width * 4;
                if (channels == 4 && !bgr)
                {
                    Buffer.BlockCopy(image.data, source, output, destination, width * 4);
                    continue;
                }
                for (int column = 0; column < width; column++, source += channels, destination += 4)
                {
                    output[destination] = image.data[source + (bgr ? 2 : 0)];
                    output[destination + 1] = image.data[source + 1];
                    output[destination + 2] = image.data[source + (bgr ? 0 : 2)];
                    output[destination + 3] = channels == 4 ? image.data[source + 3] : (byte)255;
                }
            }
            error = "";
            return true;
        }
    }
}
