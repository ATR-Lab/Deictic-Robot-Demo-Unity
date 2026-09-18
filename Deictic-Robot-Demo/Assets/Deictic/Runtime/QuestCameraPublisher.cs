using System;
using Meta.XR;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using UnityEngine;

namespace Deictic
{
    /// <summary>Optional physical Quest RGB + room-mesh depth adapter. Never substitutes synthetic data for camera observations.</summary>
    public sealed class QuestCameraPublisher : MonoBehaviour
    {
        public DeicticBridge bridge;
        public DeicticInput surface;
        public int width = 160;
        public int height = 120;
        public float interval = 1;
        PassthroughCameraAccess cameraAccess;
        Texture2D reduced;
        float nextFrame;
        const string Prefix = "/deictic/headset/";
        void Start()
        {
#if UNITY_ANDROID && !UNITY_EDITOR
            UnityEngine.Android.Permission.RequestUserPermission(OVRPermissionsRequester.PassthroughCameraAccessPermission);
            cameraAccess = gameObject.AddComponent<PassthroughCameraAccess>();
            cameraAccess.CameraPosition = PassthroughCameraAccess.CameraPositionType.Left;
            cameraAccess.RequestedResolution = new Vector2Int(1280, 960);
            reduced = new Texture2D(width, height, TextureFormat.RGB24, false);
            bridge.Ros.RegisterPublisher<CompressedImageMsg>(Prefix + "image/compressed");
            bridge.Ros.RegisterPublisher<CameraInfoMsg>(Prefix + "camera_info");
            bridge.Ros.RegisterPublisher<ImageMsg>(Prefix + "depth");
            bridge.Ros.RegisterPublisher<PoseStampedMsg>(Prefix + "camera_pose");
#else
            Debug.LogWarning("Quest camera publisher requires a physical Quest Android build; use Isaac camera topics in simulation.");
            enabled = false;
#endif
        }
        void LateUpdate()
        {
            if (!cameraAccess || !cameraAccess.IsPlaying || !cameraAccess.IsUpdatedThisFrame || Time.unscaledTime < nextFrame) return;
            nextFrame = Time.unscaledTime + Mathf.Max(.5f, interval);
            Pose pose = cameraAccess.GetCameraPose();
            double stamp = (cameraAccess.Timestamp - DateTime.UnixEpoch).TotalSeconds + RosFrames.ClockOffsetSeconds;
            if (Math.Abs(stamp - RosFrames.Now) > .5) return;
            var pixels = cameraAccess.GetColors();
            Vector2Int resolution = cameraAccess.CurrentResolution;
            if (!pixels.IsCreated || pixels.Length != resolution.x * resolution.y) return;
            var colors = new Color32[width * height];
            var depths = new float[width * height];
            Quaternion inverse = Quaternion.Inverse(pose.rotation);
            // Pixel/depth share the same sample center; ROS rows start at the image top.
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                {
                    float u = (x + .5f) / width, v = (y + .5f) / height;
                    colors[y * width + x] = pixels[Mathf.Min(resolution.y - 1, (int)(v * resolution.y)) * resolution.x + Mathf.Min(resolution.x - 1, (int)(u * resolution.x))];
                    Ray ray = cameraAccess.ViewportPointToRay(new Vector2(u, v), pose);
                    depths[(height - 1 - y) * width + x] = surface.Raycast(ray, out RaycastHit hit)
                        ? (inverse * (hit.point - pose.position)).z : float.NaN;
                }
            reduced.SetPixels32(colors); reduced.Apply(false);
            Vector3 lower = inverse * cameraAccess.ViewportPointToRay(Vector2.zero, pose).direction;
            Vector3 upper = inverse * cameraAccess.ViewportPointToRay(Vector2.one, pose).direction;
            lower /= lower.z; upper /= upper.z;
            double fx = width / (upper.x - lower.x), fy = height / (upper.y - lower.y);
            double cx = -lower.x * fx - .5, cy = height + lower.y * fy - .5;
            var header = RosFrames.Header("headset_camera_optical", stamp);
            var info = new CameraInfoMsg
            {
                header = header, width = (uint)width, height = (uint)height,
                distortion_model = "plumb_bob", d = new double[5],
                k = new[] { fx, 0, cx, 0, fy, cy, 0, 0, 1.0 },
                r = new[] { 1.0, 0, 0, 0, 1, 0, 0, 0, 1 },
                p = new[] { fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0 }
            };
            byte[] depthBytes = new byte[depths.Length * sizeof(float)];
            Buffer.BlockCopy(depths, 0, depthBytes, 0, depthBytes.Length);
            bridge.Ros.Publish(Prefix + "camera_info", info);
            bridge.Ros.Publish(Prefix + "camera_pose", new PoseStampedMsg(RosFrames.Header("headset_world", stamp), RosFrames.OpticalPose(pose.position, pose.rotation)));
            bridge.Ros.Publish(Prefix + "depth", new ImageMsg(header, (uint)height, (uint)width, "32FC1", (byte)(BitConverter.IsLittleEndian ? 0 : 1), (uint)width * 4, depthBytes));
            bridge.Ros.Publish(Prefix + "image/compressed", new CompressedImageMsg(header, "jpeg", reduced.EncodeToJPG(85)));
        }
        void OnDestroy() { if (reduced) Destroy(reduced); }
    }
}
