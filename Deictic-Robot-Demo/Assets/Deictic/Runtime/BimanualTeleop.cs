using UnityEngine;

namespace Deictic
{
    /// <summary>Samples both XR controllers together; the ROS controller owns IK and motion limits.</summary>
    public sealed class BimanualTeleop : MonoBehaviour
    {
        public DeicticBridge bridge;
        public Transform head, bodyFrame, leftController, rightController;
        public BimanualClutch Clutch { get; } = new BimanualClutch();
        public bool LeftHeld { get; private set; }
        public bool RightHeld { get; private set; }
        public float LeftTriggerValue => leftTrigger.Value;
        public float RightTriggerValue => rightTrigger.Value;
        public bool SuppressActions => isActiveAndEnabled && Clutch.SuppressActions;
        public event System.Action Interrupted;
        public string Feedback
        {
            get
            {
                string state = Clutch.Active || Clutch.AwaitingRelease ? Clutch.Reason :
                    bridge != null && !bridge.CanTeleoperate ? "Arm control waiting: " +
                    (!bridge.TeleopProtocolCompatible ? "backend protocol v3 required" :
                        bridge.Status?.teleop_reason ?? "fresh ROS status and clock") : Clutch.Reason;
                if (bridge != null && !string.IsNullOrEmpty(bridge.LastTeleopFault))
                    state += "\nLast arm stop: " + bridge.LastTeleopFault;
                if (bridge?.Status != null && bridge.Status.teleop_limited)
                    state += "\nArm target limited — " + bridge.Status.teleop_reason;
                var status = bridge?.Status;
                if (status != null && status.teleop_active &&
                    ValidPair(status.teleop_measured_position_errors) && ValidPair(status.teleop_measured_orientation_errors))
                    state += $"\nArm error: {status.teleop_position_error * 1000:F0} mm / {status.teleop_orientation_error * Mathf.Rad2Deg:F1} deg";
                return state;
            }
        }
        static bool ValidPair(float[] values) => values != null && values.Length == 2 &&
            float.IsFinite(values[0]) && float.IsFinite(values[1]);
        bool focused = true, paused;
        readonly IndexTriggerState leftTrigger = new IndexTriggerState();
        readonly IndexTriggerState rightTrigger = new IndexTriggerState();

        void OnEnable() { OVRManager.InputFocusLost += OnInputFocusLost; }
        void Start() { if (bridge != null) bridge.TeleopStopRequested += OnStopRequested; }

        // Called before deictic input dispatch, so a chord never leaks a click.
        public void Tick(bool xrActive)
        {
            leftTrigger.Sample(xrActive ? OVRInput.Get(OVRInput.RawAxis1D.LIndexTrigger, OVRInput.Controller.LTouch) : 0, xrActive);
            rightTrigger.Sample(xrActive ? OVRInput.Get(OVRInput.RawAxis1D.RIndexTrigger, OVRInput.Controller.RTouch) : 0, xrActive);
            LeftHeld = leftTrigger.Held;
            RightHeld = rightTrigger.Held;
            bool leftTracked = xrActive && leftController && OVRInput.IsControllerConnected(OVRInput.Controller.LTouch) &&
                OVRInput.GetControllerPositionTracked(OVRInput.Controller.LTouch) && OVRInput.GetControllerOrientationTracked(OVRInput.Controller.LTouch);
            bool rightTracked = xrActive && rightController && OVRInput.IsControllerConnected(OVRInput.Controller.RTouch) &&
                OVRInput.GetControllerPositionTracked(OVRInput.Controller.RTouch) && OVRInput.GetControllerOrientationTracked(OVRInput.Controller.RTouch);
            bool headTracked = xrActive && head && OVRPlugin.GetNodePositionTracked(OVRPlugin.Node.EyeCenter) &&
                OVRPlugin.GetNodeOrientationTracked(OVRPlugin.Node.EyeCenter);
            bool hasFocus = focused && !paused && (!xrActive || OVRManager.hasInputFocus);
            bool publish = Clutch.Step(Time.realtimeSinceStartupAsDouble, LeftHeld, RightHeld,
                leftTracked && leftTrigger.Valid, rightTracked && rightTrigger.Valid,
                headTracked, hasFocus, bridge != null && bridge.CanTeleoperate,
                head ? head.position : Vector3.zero, head ? head.rotation : Quaternion.identity,
                bodyFrame ? bodyFrame.rotation : Quaternion.identity,
                leftController ? leftController.position : Vector3.zero, leftController ? leftController.rotation : Quaternion.identity,
                rightController ? rightController.position : Vector3.zero, rightController ? rightController.rotation : Quaternion.identity,
                !xrActive || leftTrigger.FullyReleased, !xrActive || rightTrigger.FullyReleased);
            bridge?.SetTeleopIntent(SuppressActions);
            if (publish && bridge != null)
                bridge.PublishTeleop(Clutch.Active, leftTracked && Clutch.PoseValid, rightTracked && Clutch.PoseValid,
                    Clutch.LeftPosition, Clutch.LeftRotation, Clutch.RightPosition, Clutch.RightRotation);
        }
        public void Interrupt(string reason)
        {
            Clutch.Interrupt(reason);
            Interrupted?.Invoke();
            bridge?.SetTeleopIntent(true);
            bridge?.PublishTeleop(false, false, false, Vector3.zero, Quaternion.identity, Vector3.zero, Quaternion.identity);
        }
        void OnStopRequested()
        {
            // Start subscribes after runtime wiring. A disabled component must not
            // reclaim deictic input when the bridge later broadcasts Cancel.
            if (isActiveAndEnabled) Interrupt("Arm control stopped; release both triggers");
        }
        void OnInputFocusLost() => Interrupt("XR focus lost; release both triggers");
        void OnApplicationFocus(bool value)
        {
            focused = value;
            if (!value) Interrupt("Application focus lost; release both triggers");
        }
        void OnApplicationPause(bool value)
        {
            paused = value;
            if (value) Interrupt("Application paused; release both triggers");
        }
        void OnDisable()
        {
            OVRManager.InputFocusLost -= OnInputFocusLost;
            Interrupt("Arm control disabled");
            bridge?.SetTeleopIntent(false);
        }
        void OnDestroy() { if (bridge != null) bridge.TeleopStopRequested -= OnStopRequested; }
    }
}
