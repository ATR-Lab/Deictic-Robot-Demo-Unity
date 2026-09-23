using Meta.XR.MRUtilityKit;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Deictic
{
    public sealed class DeicticInput : MonoBehaviour
    {
        public DeicticBridge bridge;
        public Transform head;
        public Transform pointer;
        public Camera viewCamera;
        public DeicticCameraView cameraView;
        public BimanualTeleop teleop;
        public RobotHeadTracking robotHead;
        public bool synthetic;
        public Vector3 Candidate { get; private set; }
        public bool HasCandidate { get; private set; }
        public string InputSource { get; private set; } = "Head direction";
        Vector3 candidateNormal;
        LineRenderer pointerLine;
        GameObject cursor;
        float nextPose;
        OVRDisplay subscribedDisplay;
        Transform trackingSpace;
        Vector3 trackingSpacePosition;
        Quaternion trackingSpaceRotation;
        Vector3 trackingSpaceScale;
        readonly TriggerReleaseIntent triggerIntent = new TriggerReleaseIntent();
        readonly IndexTriggerState standaloneRightTrigger = new IndexTriggerState();
        bool rightWasHeld;

        void Start()
        {
            cursor = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            cursor.name = "Deictic target candidate";
            Destroy(cursor.GetComponent<Collider>());
            cursor.transform.localScale = Vector3.one * .02f;
            cursor.GetComponent<Renderer>().material = DemoVisuals.Material(new Color(.1f, .95f, .65f));
            // Transparent queue draws the ray after the fullscreen robot feed.
            pointerLine = DemoVisuals.Line("Controller pointer", new Color(0, 1, 1, .95f), .002f);
            pointerLine.gameObject.layer = DeicticCameraView.RobotUiLayer;
            OVRCameraRig rig = head ? head.GetComponentInParent<OVRCameraRig>() : null;
            trackingSpace = rig ? rig.trackingSpace : null;
            RememberTrackingSpace();
            SubscribeRecenter();
            if (teleop) teleop.Interrupted += OnTeleopInterrupted;
        }
        void Update()
        {
            if (bridge == null) return;
            // Cancel and the keyboard view shortcut remain usable when pose tracking is lost.
            bool xrActive = OVRManager.instance != null && OVRManager.isHmdPresent;
            SubscribeRecenter();
            CheckTrackingOrigin();
            if (robotHead && robotHead.isActiveAndEnabled) robotHead.Tick(xrActive);
            if (teleop && teleop.isActiveAndEnabled) teleop.Tick(xrActive);
            bool rightHeld;
            if (teleop && teleop.isActiveAndEnabled) rightHeld = teleop.RightHeld;
            else
            {
                standaloneRightTrigger.Sample(xrActive ? OVRInput.Get(OVRInput.RawAxis1D.RIndexTrigger,
                    OVRInput.Controller.RTouch) : 0, xrActive);
                rightHeld = standaloneRightTrigger.Held;
            }
            bool triggerPressed = rightHeld && !rightWasHeld;
            bool triggerReleased = !rightHeld && rightWasHeld;
            rightWasHeld = rightHeld;
            bool suppress = teleop && teleop.SuppressActions;
            if (!xrActive || suppress || (teleop && teleop.Clutch.AwaitingRelease)) triggerIntent.Cancel();
            Keyboard k = Keyboard.current;
            bool cancel = (xrActive && OVRInput.GetDown(OVRInput.Button.Two, OVRInput.Controller.RTouch)) ||
                (k != null && k.escapeKey.wasPressedThisFrame);
            bool viewShortcut = !suppress && cameraView && cameraView.isActiveAndEnabled && k != null && k.vKey.wasPressedThisFrame;
            if (cancel) bridge.Cancel();
            else if (viewShortcut) cameraView.ToggleButton.onClick.Invoke();
            if (!head) return;
            if (!synthetic)
            {
                bool valid = OVRManager.isHmdPresent &&
                    OVRPlugin.GetNodePositionTracked(OVRPlugin.Node.EyeCenter) &&
                    OVRPlugin.GetNodeOrientationTracked(OVRPlugin.Node.EyeCenter);
                bridge.SetHeadTrackingValid(valid);
                if (!valid)
                {
                    HasCandidate = false;
                    cursor.SetActive(false);
                    pointerLine.positionCount = 0;
                    return;
                }
            }
            if (Time.unscaledTime >= nextPose)
            {
                bridge.PublishHeadset(head);
                nextPose = Time.unscaledTime + 1f / Mathf.Max(1, bridge.settings.headsetPublishHz);
            }
            if (suppress)
            {
                HasCandidate = false;
                cursor.SetActive(false);
                pointerLine.positionCount = 0;
                if (cameraView) cameraView.SetHovered(false);
                InputSource = teleop.Feedback;
                return;
            }
            Ray headRay = new Ray(head.position, head.forward);
            bool headHit = Raycast(headRay, out RaycastHit h);
            bool tracked = xrActive && pointer && OVRInput.IsControllerConnected(OVRInput.Controller.RTouch) &&
                OVRInput.GetControllerPositionTracked(OVRInput.Controller.RTouch);
            Ray ray = tracked ? new Ray(pointer.position, pointer.forward) : headRay;
            InputSource = tracked ? "Controller pointer + head direction" : "Head direction";
#if UNITY_EDITOR || UNITY_STANDALONE
            if (!tracked && viewCamera && Mouse.current != null && synthetic)
            {
                ray = viewCamera.ScreenPointToRay(Mouse.current.position.ReadValue());
                InputSource = "Desktop mouse + head direction";
                tracked = true;
            }
#endif
            RaycastHit p = default;
            bool pointerHit = tracked && Raycast(ray, out p);
            Vector3 uiPoint = default;
            bool overToggle = cameraView && cameraView.PointAtToggle(ray, out uiPoint);
            if (cameraView) cameraView.SetHovered(overToggle);
            Vector3 point = default;
            HasCandidate = !overToggle && (!cameraView || !cameraView.RobotView) &&
                TargetResolver.Resolve(headHit, h, pointerHit, p, bridge.settings.agreementDistance, out point, out candidateNormal);
            Candidate = point;
            cursor.SetActive(HasCandidate);
            if (HasCandidate) cursor.transform.position = point;
            bool showPointer = (tracked || overToggle) && (!cameraView || !cameraView.RobotView || overToggle);
            pointerLine.positionCount = showPointer ? 2 : 0;
            if (showPointer) { pointerLine.SetPosition(0, ray.origin); pointerLine.SetPosition(1, overToggle ? uiPoint : pointerHit ? p.point : ray.GetPoint(2)); }
            if (cancel || viewShortcut)
            {
                triggerIntent.Cancel();
                HasCandidate = false;
                cursor.SetActive(false);
                return;
            }
            if (triggerPressed) triggerIntent.Begin(overToggle, HasCandidate, Candidate, candidateNormal);
            if (triggerReleased)
            {
                var intent = triggerIntent.Release(overToggle, HasCandidate, Candidate, bridge.settings.agreementDistance,
                    out Vector3 selectedPoint, out Vector3 selectedNormal);
                if (intent == TriggerReleaseIntent.Kind.CameraToggle) cameraView.ToggleButton.onClick.Invoke();
                else if (intent == TriggerReleaseIntent.Kind.Target) bridge.Commit(selectedPoint, selectedNormal);
            }
            bool commit = false;
            bool execute = xrActive && OVRInput.GetDown(OVRInput.Button.One, OVRInput.Controller.RTouch);
            if (k != null) { commit |= k.spaceKey.wasPressedThisFrame; execute |= k.enterKey.wasPressedThisFrame; }
            if (cameraView && overToggle && (commit || (Mouse.current != null && Mouse.current.leftButton.wasPressedThisFrame)))
            {
                cameraView.ToggleButton.onClick.Invoke();
                HasCandidate = false;
                cursor.SetActive(false);
            }
            else if (commit) Commit();
            else if (execute) bridge.Execute();
        }
        public bool Raycast(Ray ray, out RaycastHit hit)
        {
            hit = default;
            if (!synthetic)
            {
                MRUKRoom room = MRUK.Instance ? MRUK.Instance.GetCurrentRoom() : null;
                return room && room.Raycast(ray, bridge.settings.maximumRayDistance, out hit);
            }
            // Explicit target surfaces only: robot, overlays and UI cannot become goals.
            bool found = false;
            foreach (RaycastHit candidate in Physics.RaycastAll(ray, bridge.settings.maximumRayDistance, ~0, QueryTriggerInteraction.Ignore))
                if (candidate.collider.GetComponentInParent<DeicticSurface>() && (!found || candidate.distance < hit.distance))
                { hit = candidate; found = true; }
            return found;
        }
        public void Commit() { if ((!teleop || !teleop.SuppressActions) && HasCandidate && (!cameraView || !cameraView.RobotView)) bridge.Commit(Candidate, candidateNormal); }
        void SubscribeRecenter()
        {
            OVRDisplay display = OVRManager.display;
            if (object.ReferenceEquals(display, subscribedDisplay)) return;
            if (subscribedDisplay != null) subscribedDisplay.RecenteredPose -= OnRecentered;
            subscribedDisplay = display;
            if (subscribedDisplay != null) subscribedDisplay.RecenteredPose += OnRecentered;
        }
        void RememberTrackingSpace()
        {
            if (!trackingSpace) return;
            trackingSpacePosition = trackingSpace.position;
            trackingSpaceRotation = trackingSpace.rotation;
            trackingSpaceScale = trackingSpace.lossyScale;
        }
        void CheckTrackingOrigin()
        {
            if (!trackingSpace || (Vector3.Distance(trackingSpace.position, trackingSpacePosition) <= .0001f &&
                Quaternion.Angle(trackingSpace.rotation, trackingSpaceRotation) <= .01f &&
                Vector3.Distance(trackingSpace.lossyScale, trackingSpaceScale) <= .0001f)) return;
            teleop?.Interrupt("Tracking origin changed; release both triggers");
            robotHead?.Interrupt();
            triggerIntent.Cancel();
            if (!synthetic) bridge.InvalidateTracking("Headset tracking origin changed", "origin_changed");
            RememberTrackingSpace();
        }
        void OnRecentered()
        {
            teleop?.Interrupt("Headset recentered; release both triggers");
            robotHead?.Interrupt();
            triggerIntent.Cancel();
            RememberTrackingSpace();
            if (synthetic) return;
            bridge.InvalidateTracking("Headset recentered", "recentered");
        }
        /// <summary>Connect a speech recognizer's final transcript to this method. No cloud credentials are embedded.</summary>
        public void OnVoiceTranscript(string transcript)
        {
            switch ((transcript ?? "").Trim().ToLowerInvariant())
            {
                case "reach there": case "select target": Commit(); break;
                case "execute": bridge.Execute(); break;
                case "cancel": case "stop": bridge.Cancel(); break;
            }
        }
        void OnTeleopInterrupted() { triggerIntent.Cancel(); rightWasHeld = false; }
        void OnApplicationFocus(bool focused) { if (!focused) OnTeleopInterrupted(); }
        void OnApplicationPause(bool paused) { if (paused) OnTeleopInterrupted(); }
        void OnDestroy()
        {
            if (teleop) teleop.Interrupted -= OnTeleopInterrupted;
            if (subscribedDisplay != null) subscribedDisplay.RecenteredPose -= OnRecentered;
            if (cursor) Destroy(cursor);
            if (pointerLine) Destroy(pointerLine.gameObject);
        }
    }
}
