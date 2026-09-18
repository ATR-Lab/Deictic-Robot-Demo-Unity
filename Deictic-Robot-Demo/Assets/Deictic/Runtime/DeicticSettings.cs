using UnityEngine;

namespace Deictic
{
    [CreateAssetMenu(menuName = "Deictic/Demo Settings")]
    public sealed class DeicticSettings : ScriptableObject
    {
        [Tooltip("ROS-TCP-Endpoint address. Use localhost with the documented SSH tunnel on Windows.")]
        public string rosHost = "127.0.0.1";
        public int rosPort = 10000;
        public bool connectOnStart = true;
        public bool syntheticScene = true;
        [Tooltip("Enable only on device with camera permission and captured room geometry.")]
        public bool publishHeadsetCamera = false;
        public float agreementDistance = .04f;
        public float maximumRayDistance = 4f;
        public float statusTimeout = 1f;
        public float minimumConfidence = .65f;
        public float headsetPublishHz = 30f;
        [Tooltip("Hold both index triggers for paired, relative position-only arm control. Release either to hold both arms.")]
        public bool bimanualTeleop = true;
        [Tooltip("Atomic left/right head-camera pair, packed side by side in one sensor_msgs/Image.")]
        public string robotCameraTopic = RosCameraStream.DefaultTopic;
        public bool robotCameraStereo = true;
        public float robotCameraTimeout = 1.5f;
        [Tooltip("World position of the camera toggle; the monitor sits above it and never follows the head.")]
        public Vector3 cameraControlsWorldPosition = new Vector3(0, 1.25f, .8f);
        public GameObject robotVisual;
        public TextAsset robotDescription;
    }
}
