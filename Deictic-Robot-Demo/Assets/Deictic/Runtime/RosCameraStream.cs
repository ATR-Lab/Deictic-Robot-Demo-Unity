using System;
using System.Threading;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Deictic
{
    /// <summary>Visible-only, latest-frame robot camera on the shared ROS connection.</summary>
    [DisallowMultipleComponent]
    public sealed class RosCameraStream : MonoBehaviour
    {
        public const string DefaultTopic = "/deictic/camera_view/stereo/image_raw";
        [Range(1, 30)] public float displayHz = 5;
        readonly object pendingLock = new object();
        ROSConnection ros;
        string topic = DefaultTopic;
        float timeout = 1;
        bool subscribed, accepting;
        ImageMsg pending;
        byte[] pixels;
        Texture2D texture;
        double newestStamp = double.NegativeInfinity;
        double displayedStamp = double.NegativeInfinity;
        float nextDisplayTime;
        string lastError = "";
        long received, displayed, dropped, rejected;

        public Texture2D Texture => texture;
        public bool IsStereo { get; private set; }
        public float EyeAspectRatio => texture == null ? 4f / 3f : (float)texture.width / ((IsStereo ? 2 : 1) * texture.height);
        public double FrameAge => double.IsNegativeInfinity(displayedStamp)
            ? double.PositiveInfinity : Math.Max(0, RosFrames.Now - displayedStamp);
        public bool HasFreshFrame => isActiveAndEnabled && subscribed && texture != null &&
            ros != null && ros.HasConnectionThread && !ros.HasConnectionError &&
            RosFrames.Now - displayedStamp >= -.25 && FrameAge <= timeout;
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
                if (!string.IsNullOrEmpty(lastError)) return lastError;
                if (texture == null || double.IsPositiveInfinity(FrameAge)) return "Waiting for robot camera frame";
                return HasFreshFrame ? $"Robot camera {texture.width}×{texture.height} · {FrameAge:F2}s old"
                    : $"Robot camera frame stale · {FrameAge:F2}s old";
            }
        }

        public void Initialize(ROSConnection connection, string imageTopic = DefaultTopic, float timeout = 1f, bool stereoSideBySide = false)
        {
            if (connection == null) throw new ArgumentNullException(nameof(connection));
            if (string.IsNullOrWhiteSpace(imageTopic)) throw new ArgumentException("A camera topic is required", nameof(imageTopic));
            if (!float.IsFinite(timeout) || timeout <= 0) throw new ArgumentOutOfRangeException(nameof(timeout));
            Unsubscribe();
            ros = connection; topic = imageTopic; this.timeout = timeout;
            IsStereo = stereoSideBySide;
            if (isActiveAndEnabled) Subscribe();
        }

        void OnEnable() { if (ros != null) Subscribe(); }
        void OnDisable() => Unsubscribe();

        void Subscribe()
        {
            if (subscribed) return;
            lock (pendingLock)
            {
                pending = null; accepting = true; newestStamp = double.NegativeInfinity;
            }
            displayedStamp = double.NegativeInfinity;
            lastError = ""; nextDisplayTime = 0;
            ros.Subscribe<ImageMsg>(topic, Receive);
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
            // Connector 0.7 dispatches from Update. A lock also makes a future
            // threaded dispatcher safe; texture APIs are used only in Update.
            Interlocked.Increment(ref received);
            if (message?.header?.stamp == null || message.header.stamp.sec < 0 ||
                message.header.stamp.nanosec >= 1000000000)
            {
                Interlocked.Increment(ref rejected);
                lastError = "Invalid robot camera acquisition timestamp";
                return;
            }
            double stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9;
            double age = RosFrames.Now - stamp;
            if (stamp <= 0 || age < -.25 || age > timeout || !double.IsFinite(age))
            {
                Interlocked.Increment(ref rejected);
                lastError = "Robot camera timestamp stale or clocks unsynchronized";
                return;
            }
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
            lock (pendingLock)
            {
                if (!accepting) return;
                if (stamp <= newestStamp)
                {
                    Interlocked.Increment(ref rejected);
                    return;
                }
                if (pending != null) Interlocked.Increment(ref dropped);
                pending = message; newestStamp = stamp;
            }
        }

        void Update()
        {
            if (!subscribed || Time.unscaledTime < nextDisplayTime) return;
            nextDisplayTime = Time.unscaledTime + 1f / Mathf.Clamp(displayHz, 1, 30);
            ImageMsg frame;
            lock (pendingLock) { frame = pending; pending = null; }
            if (frame == null) return;
            double stamp = frame.header.stamp.sec + frame.header.stamp.nanosec * 1e-9;
            double age = RosFrames.Now - stamp;
            if (!double.IsFinite(age) || age < -.25 || age > timeout)
            {
                Interlocked.Increment(ref rejected);
                lastError = "Robot camera timestamp stale or clocks unsynchronized";
                return;
            }
            if (!RosImagePacking.TryPackRgba(frame, ref pixels, out int width, out int height, out string error))
            {
                Interlocked.Increment(ref rejected); lastError = error; return;
            }
            if (texture == null || texture.width != width || texture.height != height)
            {
                if (texture != null) Destroy(texture);
                texture = new Texture2D(width, height, TextureFormat.RGBA32, false, false)
                {
                    name = IsStereo ? "Live K1 head stereo" : "Live K1 head camera", wrapMode = TextureWrapMode.Clamp,
                    filterMode = FilterMode.Bilinear, hideFlags = HideFlags.DontSave
                };
            }
            // Preserve ROS top-left row order. The UI flips its RawImage uvRect.
            texture.LoadRawTextureData(pixels);
            texture.Apply(false, false);
            displayedStamp = stamp; lastError = "";
            Interlocked.Increment(ref displayed);
        }

        void OnDestroy()
        {
            Unsubscribe();
            if (texture != null) Destroy(texture);
            pixels = null;
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
