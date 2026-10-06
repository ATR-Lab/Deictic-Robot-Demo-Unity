using System;
using System.IO;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace Deictic
{
    public enum DeicticControlMode { ManualSimulation, TransitionSimulation, HardwareObservation }

    /// <summary>Machine-local startup configuration. Never changes control ownership during a run.</summary>
    public static class TransitionRuntimeConfig
    {
        public const string FileName = "transition-runtime.json";

        public static string Load(DeicticSettings settings)
        {
#if UNITY_ANDROID && !UNITY_EDITOR
            // Quest Link runs the Windows player/editor. Android uses its build settings;
            // StreamingAssets is inside the APK and is not a mutable filesystem path.
            return null;
#else
            string path = Path.Combine(Application.streamingAssetsPath, FileName);
            if (!File.Exists(path)) return null;
            try { Apply(settings, File.ReadAllText(path)); return null; }
            catch (Exception e)
            {
                settings.controlMode = DeicticControlMode.HardwareObservation;
                settings.connectOnStart = false;
                return "Startup configuration rejected; commands disabled: " + e.Message;
            }
#endif
        }

        public static void Apply(DeicticSettings settings, string json)
        {
            var data = JObject.Parse(json, new JsonLoadSettings { DuplicatePropertyNameHandling = DuplicatePropertyNameHandling.Error });
            foreach (var property in data.Properties())
                if (property.Name != "schema_version" && property.Name != "control_mode" && property.Name != "ros_host" &&
                    property.Name != "ros_port" && property.Name != "operator_url" && property.Name != "robot_camera_topic" &&
                    property.Name != "robot_camera_stereo") throw new ArgumentException("Unknown field: " + property.Name);
            if (data["schema_version"]?.Type != JTokenType.Integer || (int)data["schema_version"] != 1)
                throw new ArgumentException("schema_version must be 1");
            DeicticControlMode mode;
            switch (Text(data, "control_mode"))
            {
                case "manual_simulation": mode = DeicticControlMode.ManualSimulation; break;
                case "transition_simulation": mode = DeicticControlMode.TransitionSimulation; break;
                case "hardware_observation": mode = DeicticControlMode.HardwareObservation; break;
                default: throw new ArgumentException("Unknown control_mode");
            }
            string address = data["operator_url"] == null ? settings.transitionOperatorUrl : Text(data, "operator_url");
            ValidateOperatorUrl(address);
            int port = settings.rosPort;
            if (data["ros_port"] != null)
            {
                if (data["ros_port"].Type != JTokenType.Integer) throw new ArgumentException("ros_port must be an integer");
                port = (int)data["ros_port"];
                if (port < 1 || port > 65535) throw new ArgumentException("Invalid ROS port");
            }
            string host = data["ros_host"] == null ? settings.rosHost : Text(data, "ros_host");
            string topic = data["robot_camera_topic"] == null ? settings.robotCameraTopic : Text(data, "robot_camera_topic");
            if (data["robot_camera_stereo"] != null && data["robot_camera_stereo"].Type != JTokenType.Boolean)
                throw new ArgumentException("robot_camera_stereo must be boolean");
            // Apply only after every field has passed validation.
            settings.controlMode = mode;
            settings.transitionOperatorUrl = address.TrimEnd('/');
            settings.rosHost = host; settings.rosPort = port; settings.robotCameraTopic = topic;
            if (data["robot_camera_stereo"] != null) settings.robotCameraStereo = (bool)data["robot_camera_stereo"];
        }

        static string Text(JObject data, string key)
        {
            if (data[key]?.Type != JTokenType.String || string.IsNullOrWhiteSpace((string)data[key]))
                throw new ArgumentException(key + " must be a nonempty string");
            return (string)data[key];
        }

        public static void ValidateOperatorUrl(string address)
        {
            if (!Uri.TryCreate(address, UriKind.Absolute, out var uri) || uri.Scheme != "http" ||
                (uri.Host != "127.0.0.1" && uri.Host != "localhost") || uri.Port < 1 ||
                uri.AbsolutePath != "/" || uri.Query.Length != 0 || uri.Fragment.Length != 0 || uri.UserInfo.Length != 0)
                throw new ArgumentException("Operator URL must be http://127.0.0.1:PORT or http://localhost:PORT through SSH");
        }
    }
}
