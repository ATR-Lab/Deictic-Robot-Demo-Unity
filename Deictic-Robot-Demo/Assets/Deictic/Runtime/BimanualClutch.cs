using System;
using UnityEngine;

namespace Deictic
{
    /// <summary>Pure input state: no robot commands, XR APIs, or clocks are read here.</summary>
    public sealed class BimanualClutch
    {
        public const double PublishPeriod = 1.0 / 20.0;
        public bool Active { get; private set; }
        public bool AwaitingRelease { get; private set; } = true;
        public bool SuppressActions { get; private set; }
        public Vector3 LeftPosition { get; private set; }
        public Vector3 RightPosition { get; private set; }
        public Quaternion LeftRotation { get; private set; } = Quaternion.identity;
        public Quaternion RightRotation { get; private set; } = Quaternion.identity;
        public bool PoseValid { get; private set; }
        public string Reason { get; private set; } = "Release both triggers to arm";
        double nextPublish;

        public bool Step(double now, bool leftHeld, bool rightHeld, bool leftTracked, bool rightTracked,
            bool headTracked, bool focused, bool ready, Vector3 headPosition, Quaternion headRotation,
            Vector3 left, Quaternion leftRotation, Vector3 right, Quaternion rightRotation)
        {
            bool wasActive = Active;
            bool valid = focused && headTracked && leftTracked && rightTracked &&
                Finite(headPosition) && Finite(left) && Finite(right) &&
                TryNormalize(headRotation, out headRotation) &&
                TryNormalize(leftRotation, out leftRotation) && TryNormalize(rightRotation, out rightRotation);
            Vector3 forward = headRotation * Vector3.forward;
            forward.y = 0;
            // Yaw becomes ill-conditioned near vertical gaze. Hold and require
            // release rather than inventing an orientation or flipping the frame.
            valid &= Finite(forward) && forward.sqrMagnitude > .01f;
            Quaternion inverseYaw = valid ? Quaternion.Inverse(Quaternion.LookRotation(forward.normalized, Vector3.up)) : Quaternion.identity;
            Vector3 relativeLeft = inverseYaw * (left - headPosition);
            Vector3 relativeRight = inverseYaw * (right - headPosition);
            valid &= Finite(relativeLeft) && Finite(relativeRight);
            if (!valid)
                Interrupt("Tracking, focus or head heading invalid; release both triggers");
            else
            {
                if (AwaitingRelease && !leftHeld && !rightHeld)
                {
                    AwaitingRelease = false;
                    Reason = "Hold both index triggers for arm control";
                }
                if (Active && (!leftHeld || !rightHeld || !ready))
                    Interrupt(ready ? "Arms held; release both triggers to rearm" : "Teleoperation unavailable; release both triggers");
                else if (!Active && !AwaitingRelease && leftHeld && rightHeld)
                {
                    if (ready)
                    {
                        Active = true;
                        Reason = "BIMANUAL ACTIVE — release either trigger to hold both arms";
                    }
                    else Interrupt("Teleoperation unavailable; release both triggers");
                }
            }
            SuppressActions = Active || leftHeld || (AwaitingRelease && (leftHeld || rightHeld));
            PoseValid = valid;
            if (valid)
            {
                // Recomputed from the CURRENT head pose, including while idle.
                // The backend owns clutch anchors, arm-length scaling and IK.
                LeftPosition = relativeLeft;
                RightPosition = relativeRight;
                LeftRotation = (inverseYaw * leftRotation).normalized;
                RightRotation = (inverseYaw * rightRotation).normalized;
            }
            else ClearPoses();
            bool transition = Active != wasActive;
            if (transition || now >= nextPublish)
            {
                nextPublish = now + PublishPeriod;
                return true;
            }
            return false;
        }

        public void Interrupt(string reason)
        {
            Active = false;
            AwaitingRelease = true;
            SuppressActions = true;
            PoseValid = false;
            ClearPoses();
            Reason = reason;
        }
        void ClearPoses()
        {
            LeftPosition = RightPosition = Vector3.zero;
            LeftRotation = RightRotation = Quaternion.identity;
        }
        public static bool TryNormalize(Quaternion rotation, out Quaternion normalized)
        {
            float normSquared = Quaternion.Dot(rotation, rotation);
            if (!float.IsFinite(normSquared) || Mathf.Abs(normSquared - 1f) > .02f)
            { normalized = Quaternion.identity; return false; }
            normalized = rotation.normalized;
            return true;
        }
        static bool Finite(Vector3 p) => float.IsFinite(p.x) && float.IsFinite(p.y) && float.IsFinite(p.z);
    }

    /// <summary>A right trigger cannot select until released without ever joining a clutch.</summary>
    public sealed class TriggerReleaseIntent
    {
        public enum Kind { None, Target, CameraToggle }
        Kind pending;
        Vector3 point, normal;
        public void Begin(bool overToggle, bool hasTarget, Vector3 target, Vector3 surfaceNormal)
        {
            pending = overToggle ? Kind.CameraToggle : hasTarget ? Kind.Target : Kind.None;
            point = target; normal = surfaceNormal;
        }
        public void Cancel() => pending = Kind.None;
        public Kind Release(bool overToggle, bool hasTarget, Vector3 currentTarget, float maximumDrift,
            out Vector3 target, out Vector3 surfaceNormal)
        {
            Kind result = pending;
            pending = Kind.None;
            target = point; surfaceNormal = normal;
            if (result == Kind.CameraToggle && !overToggle) return Kind.None;
            if (result == Kind.Target && (overToggle || !hasTarget || Vector3.Distance(point, currentTarget) > maximumDrift)) return Kind.None;
            return result;
        }
    }
}
