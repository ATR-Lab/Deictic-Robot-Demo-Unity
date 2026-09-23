using UnityEngine;

namespace Deictic
{
    /// <summary>Head orientation in the calibrated user-forward frame, independent of arm clutch.</summary>
    public sealed class RobotHeadSample
    {
        public const double PublishPeriod = 1.0 / 50.0;
        public bool Active { get; private set; }
        public bool Tracked { get; private set; }
        public Quaternion Rotation { get; private set; } = Quaternion.identity;
        double nextPublish = double.NegativeInfinity;

        public bool Step(double now, bool robotView, bool tracked, bool focused, bool ready,
            Quaternion headRotation, Quaternion bodyRotation)
        {
            bool previous = Active;
            bool valid = tracked && focused &&
                BimanualClutch.TryNormalize(headRotation, out headRotation) &&
                BimanualClutch.TryNormalize(bodyRotation, out bodyRotation);
            Vector3 forward = bodyRotation * Vector3.forward;
            forward.y = 0;
            valid &= float.IsFinite(forward.sqrMagnitude) && forward.sqrMagnitude > .01f;
            Tracked = valid;
            Rotation = valid ? (Quaternion.Inverse(Quaternion.LookRotation(forward.normalized, Vector3.up)) * headRotation).normalized : Quaternion.identity;
            Active = robotView && valid && ready;
            if (previous != Active || now >= nextPublish)
            {
                nextPublish = now + PublishPeriod;
                return true;
            }
            return false;
        }

        public void Reset()
        {
            Active = Tracked = false;
            Rotation = Quaternion.identity;
            nextPublish = double.NegativeInfinity;
        }
    }

    public sealed class RobotHeadTracking : MonoBehaviour
    {
        public DeicticBridge bridge;
        public DeicticCameraView cameraView;
        public Transform head, bodyFrame;
        public RobotHeadSample Sample { get; } = new RobotHeadSample();
        bool focused = true, paused;

        void OnEnable() => OVRManager.InputFocusLost += Interrupt;

        // Called by input dispatch even while arm control suppresses UI actions.
        public void Tick(bool xrActive)
        {
            bool tracked = xrActive && head && OVRPlugin.GetNodeOrientationTracked(OVRPlugin.Node.EyeCenter);
            bool focus = focused && !paused && (!xrActive || OVRManager.hasInputFocus);
            if (Sample.Step(Time.realtimeSinceStartupAsDouble, cameraView && cameraView.RobotView,
                tracked, focus, bridge != null && bridge.CanTrackRobotHead,
                head ? head.rotation : Quaternion.identity, bodyFrame ? bodyFrame.rotation : Quaternion.identity))
                bridge?.PublishRobotHead(Sample.Active, Sample.Tracked, Sample.Rotation);
        }

        public void Interrupt()
        {
            Sample.Reset();
            bridge?.PublishRobotHead(false, false, Quaternion.identity);
        }
        void OnApplicationFocus(bool value) { focused = value; if (!value) Interrupt(); }
        void OnApplicationPause(bool value) { paused = value; if (value) Interrupt(); }
        void OnDisable()
        {
            OVRManager.InputFocusLost -= Interrupt;
            Interrupt();
        }
    }
}
