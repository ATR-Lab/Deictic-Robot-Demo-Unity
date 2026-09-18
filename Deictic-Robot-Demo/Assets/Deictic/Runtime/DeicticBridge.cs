using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Trajectory;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Deictic
{
    [Serializable]
    public sealed class AlignmentStatus
    {
        public double stamp;
        public float confidence;
        public bool can_commit;
        public string reason;
        public string backend;
        public string mode;
        public string operation;
        public int alignment_version;
        public int alignment_epoch;
        public double[] base_from_headset_world;
        public float registration_age;
        public bool teleop_active;
        public bool teleop_ready;
        public string teleop_state;
        public string teleop_reason;
        public int teleop_protocol_version;
        public string teleop_last_fault;
        public bool teleop_limited;
        public float teleop_position_error;
        public float teleop_orientation_error;
        public float[] teleop_measured_position_errors;
        public float[] teleop_measured_orientation_errors;
        public float[] teleop_commanded_position_errors;
        public float[] teleop_commanded_orientation_errors;
        public float teleop_translation_scale;
        public bool teleop_relay_ready;
    }

    public sealed class DeicticBridge : MonoBehaviour
    {
        public DeicticSettings settings;
        public ROSConnection Ros { get; private set; }
        public AlignmentStatus Status { get; private set; }
        public Matrix4x4 BaseFromWorld { get; private set; } = Matrix4x4.identity;
        public bool HasAlignment { get; private set; }
        public bool HasPreview { get; private set; }
        // Unity/RUF coordinates in the robot base frame, retained while awaiting
        // the correlated preview. The planned FK endpoint may differ by IK tolerance.
        public Vector3? CommittedBasePoint { get; private set; }
        public string Feedback { get; private set; } = "Waiting for ROS alignment";
        public event Action<JointStateMsg> JointStateReceived;
        public event Action<JointTrajectoryMsg> PreviewReceived;
        public event Action PreviewCleared;
        public event Action TeleopStopRequested;
        public bool TeleopControlsBusy { get; private set; }
        public const int TeleopProtocolVersion = 2;
        public bool TeleopProtocolCompatible => Status != null && Status.teleop_protocol_version == TeleopProtocolVersion;
        public string LastTeleopFault { get; private set; }
        readonly string teleopSession = Guid.NewGuid().ToString("N");
        int teleopSequence;
        float statusTime = float.NegativeInfinity;
        double sourceStamp = double.NegativeInfinity;
        int previewEpoch = -1;
        long pendingGoalNanoseconds = -1;
        double clockRequest = -1;
        float nextClockRequest;
        float lastClockReply = float.NegativeInfinity;
        bool clockSynchronized;
        readonly RosClockSync clockSync = new RosClockSync();
        bool headTrackingValid = true;
        bool applicationWasPaused;
        double trackingInvalidatedLocalTime = double.NegativeInfinity;
        public bool CanCommit => clockSynchronized && Time.realtimeSinceStartup - lastClockReply < 15 &&
            headTrackingValid && HasAlignment && Status != null && Status.can_commit &&
            Status.confidence >= settings.minimumConfidence && Time.realtimeSinceStartup - statusTime < settings.statusTimeout &&
            Ros != null && Ros.HasConnectionThread && !Ros.HasConnectionError;
        public bool CanTeleoperate => clockSynchronized && Time.realtimeSinceStartup - lastClockReply < 15 &&
            TeleopProtocolCompatible && Status.teleop_ready && Status.teleop_relay_ready &&
            Time.realtimeSinceStartup - statusTime < settings.statusTimeout &&
            Ros != null && Ros.HasConnectionThread && !Ros.HasConnectionError;

        public void Initialize(DeicticSettings config)
        {
            settings = config;
            headTrackingValid = config.syntheticScene;
            Ros = ROSConnection.GetOrCreateInstance();
            Ros.ConnectOnStart = false;
            Ros.RosIPAddress = config.rosHost;
            Ros.RosPort = config.rosPort;
            Ros.RegisterPublisher<PoseStampedMsg>("/deictic/headset_pose");
            Ros.RegisterPublisher<PoseStampedMsg>("/deictic/goal");
            Ros.RegisterPublisher<EmptyMsg>("/deictic/cancel");
            Ros.RegisterPublisher<StringMsg>("/deictic/execute_request");
            Ros.RegisterPublisher<StringMsg>("/deictic/registration_failure");
            Ros.RegisterPublisher<StringMsg>("/deictic/teleop/input", queue_size: 1, latch: false);
            Ros.RegisterPublisher<Float64Msg>("/deictic/time_sync/request");
            Ros.Subscribe<Float64MultiArrayMsg>("/deictic/time_sync/reply", OnClockReply);
            Ros.Subscribe<StringMsg>("/deictic/status", OnStatus);
            Ros.Subscribe<JointStateMsg>("/joint_states", msg => JointStateReceived?.Invoke(msg));
            Ros.Subscribe<JointTrajectoryMsg>("/deictic/preview", OnPreview);
            if (config.connectOnStart) Ros.Connect();
        }
        void OnStatus(StringMsg message)
        {
            try
            {
                AlignmentStatus candidate = JsonUtility.FromJson<AlignmentStatus>(message.data);
                // Reject replayed/delayed status; receive time alone cannot establish freshness.
                double age = RosFrames.Now - candidate.stamp;
                if (!double.IsFinite(candidate.stamp) || candidate.stamp <= sourceStamp || age > settings.statusTimeout || age < -.25)
                { Feedback = "Stale status or clocks unsynchronized"; return; }
                bool validTransform = candidate.base_from_headset_world != null && candidate.base_from_headset_world.Length == 7;
                if (validTransform)
                {
                    Matrix4x4 next = RosFrames.Matrix(candidate.base_from_headset_world);
                    BaseFromWorld = next;
                    // A freshly published status may still describe an observation
                    // captured before tracking was lost or its world origin changed.
                    double registrationStamp = candidate.stamp - candidate.registration_age;
                    HasAlignment = double.IsFinite(registrationStamp) &&
                        registrationStamp > trackingInvalidatedLocalTime + RosFrames.ClockOffsetSeconds;
                }
                else HasAlignment = false;
                // The controller checks displacement of the committed target
                // against the original preview transform. Small continuous
                // optimizer updates need not invalidate a reviewed trajectory.
                // Origin resets always do; backend invalidations also publish
                // an empty preview, and execution retains exact goal correlation.
                if (Status != null && candidate.alignment_epoch != Status.alignment_epoch) ClearPreview();
                Status = candidate;
                // Keep the last backend fault visible even after neutral
                // heartbeat status returns to ready/clutch_released.
                if (!string.IsNullOrEmpty(candidate.teleop_last_fault)) LastTeleopFault = candidate.teleop_last_fault;
                sourceStamp = candidate.stamp;
                statusTime = Time.realtimeSinceStartup;
                Feedback = candidate.reason + (string.IsNullOrEmpty(candidate.operation) ? "" : " / " + candidate.operation);
                if (!candidate.can_commit) ClearPreview();
            }
            catch (Exception e) { HasAlignment = false; Feedback = "Invalid alignment: " + e.Message; ClearPreview(); }
        }
        void OnPreview(JointTrajectoryMsg msg)
        {
            // The backend clears the preceding plan before answering a new request.
            // Preserve that new request's stamp; cancel/status invalidation clear it locally.
            if (msg.points == null || msg.points.Length == 0)
            { HasPreview = false; previewEpoch = -1; PreviewCleared?.Invoke(); return; }
            long stamp = (long)msg.header.stamp.sec * 1000000000L + msg.header.stamp.nanosec;
            if (!CanCommit || pendingGoalNanoseconds < 0 || stamp != pendingGoalNanoseconds) return;
            if (msg.joint_names == null || msg.joint_names.Length == 0) return;
            double lastTime = -1;
            foreach (JointTrajectoryPointMsg point in msg.points)
            {
                double time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
                if (point.positions == null || point.positions.Length != msg.joint_names.Length || time < 0 || time <= lastTime) return;
                foreach (double q in point.positions) if (double.IsNaN(q) || double.IsInfinity(q)) return;
                lastTime = time;
            }
            previewEpoch = Status.alignment_epoch;
            HasPreview = true;
            PreviewReceived?.Invoke(msg);
            Feedback = "Preview ready - press A / Enter to execute in simulation";
        }
        void Update()
        {
            if (Ros != null && Ros.HasConnectionThread && !Ros.HasConnectionError && Time.realtimeSinceStartup >= nextClockRequest)
            {
                clockRequest = RosFrames.LocalNow;
                Ros.Publish("/deictic/time_sync/request", new Float64Msg(clockRequest));
                bool recentClock = clockSynchronized && Time.realtimeSinceStartup - lastClockReply < 10;
                nextClockRequest = Time.realtimeSinceStartup + (recentClock ? 5 : .5f);
            }
            if (Status != null && !CanCommit && HasPreview) ClearPreview();
        }
        void OnClockReply(Float64MultiArrayMsg message)
        {
            double received = RosFrames.LocalNow;
            if (!clockSync.TryAccept(message.data, received, clockRequest, Time.realtimeSinceStartup))
            {
                // Retry a delayed/asymmetric exchange promptly instead of
                // treating its uncertain offset as a valid command clock.
                nextClockRequest = Mathf.Min(nextClockRequest, Time.realtimeSinceStartup + .5f);
                return;
            }
            double offset = clockSync.OffsetSeconds;
            if (!clockSynchronized) Debug.Log($"ROS clock synchronized: offset={offset:F4}s, round-trip={clockSync.RoundTripSeconds:F4}s");
            if (clockSynchronized && Math.Abs(offset - RosFrames.ClockOffsetSeconds) > .1)
            { ClearPreview(); HasAlignment = false; TeleopStopRequested?.Invoke(); }
            RosFrames.ClockOffsetSeconds = offset;
            clockSynchronized = true;
            // Higher-latency replies cannot extend the life of an older best sample.
            lastClockReply = (float)clockSync.SelectedAt;
        }
        public void PublishHeadset(Transform head)
        {
            if (Ros != null && head != null) Ros.Publish("/deictic/headset_pose", RosFrames.StampedPose(head.position, head.rotation, "headset_world"));
        }
        public bool Commit(Vector3 worldPoint, Vector3 worldNormal)
        {
            if (TeleopControlsBusy) { Feedback = "Selection blocked during arm clutch"; return false; }
            ClearPreview();
            if (!CanCommit) { Feedback = "Commit blocked: fresh confident alignment required"; return false; }
            Vector3 p = BaseFromWorld.MultiplyPoint3x4(worldPoint);
            Vector3 approach = -BaseFromWorld.MultiplyVector(worldNormal).normalized;
            if (approach.sqrMagnitude < .9f) approach = Vector3.down;
            Vector3 up = Mathf.Abs(Vector3.Dot(approach, Vector3.up)) > .95f ? Vector3.forward : Vector3.up;
            PoseStampedMsg goal = RosFrames.StampedPose(p, Quaternion.LookRotation(approach, up), "base_link");
            pendingGoalNanoseconds = (long)goal.header.stamp.sec * 1000000000L + goal.header.stamp.nanosec;
            CommittedBasePoint = p;
            Ros.Publish("/deictic/goal", goal);
            Feedback = "Goal sent for workspace, collision and IK checks";
            return true;
        }
        public bool Execute()
        {
            if (TeleopControlsBusy) { Feedback = "Execution blocked during arm clutch"; return false; }
            if (!CanCommit || !HasPreview || previewEpoch != Status.alignment_epoch)
            { Feedback = "Execution blocked: select and preview a valid target first"; return false; }
            Ros.Publish("/deictic/execute_request", new StringMsg(JsonUtility.ToJson(new ExecuteRequest
            {
                goal_stamp_sec = (int)(pendingGoalNanoseconds / 1000000000L),
                goal_stamp_nanosec = (uint)(pendingGoalNanoseconds % 1000000000L)
            })));
            ClearPreview();
            Feedback = "Execution requested";
            return true;
        }
        [Serializable]
        sealed class ExecuteRequest
        {
            public int schema_version = 1;
            public int goal_stamp_sec;
            public uint goal_stamp_nanosec;
        }
        public void Cancel()
        {
            TeleopStopRequested?.Invoke();
            Ros?.Publish("/deictic/cancel", new EmptyMsg());
            ClearPreview();
            Feedback = "Cancel requested";
        }
        [Serializable]
        sealed class TrackingFailure
        {
            public int schema_version = 1;
            public double stamp;
            public string reason;
            public string source = "quest_tracking";
            public string event_type;
        }
        public void SetHeadTrackingValid(bool valid)
        {
            if (headTrackingValid == valid) return;
            headTrackingValid = valid;
            InvalidateTracking(valid ? "Head tracking restored" : "Head tracking lost", valid ? "restored" : "lost");
        }
        public void InvalidateTracking(string reason, string eventType = "tracking_invalidated")
        {
            TeleopStopRequested?.Invoke();
            trackingInvalidatedLocalTime = RosFrames.LocalNow;
            HasAlignment = false;
            ClearPreview();
            Ros?.Publish("/deictic/cancel", new EmptyMsg());
            Ros?.Publish("/deictic/registration_failure", new StringMsg(JsonUtility.ToJson(
                new TrackingFailure { stamp = RosFrames.Now, reason = reason, event_type = eventType })));
            Feedback = reason + "; waiting for post-event registration";
        }
        public void SetTeleopIntent(bool busy)
        {
            if (busy && !TeleopControlsBusy) ClearPreview();
            TeleopControlsBusy = busy;
        }
        [Serializable]
        public sealed class TeleopInput
        {
            public int schema_version = TeleopProtocolVersion;
            public string session_id;
            public int sequence;
            public double stamp;
            public string frame_id = "teleop_head";
            public bool clutch, left_tracked, right_tracked;
            public double[] left_position, right_position;
            public double[] left_rotation, right_rotation;
        }
        public static double[] TeleopPosition(Vector3 unityHeadRelativePosition)
        {
            var pose = RosFrames.Pose(unityHeadRelativePosition, Quaternion.identity);
            return new[] { pose.position.x, pose.position.y, pose.position.z };
        }
        public static double[] TeleopRotation(Quaternion unityHeadRelativeRotation)
        {
            if (!BimanualClutch.TryNormalize(unityHeadRelativeRotation, out Quaternion rotation))
                throw new ArgumentException("Teleop orientation must be finite and normalized");
            var q = RosFrames.Pose(Vector3.zero, rotation).orientation;
            return new[] { q.x, q.y, q.z, q.w };
        }
        public bool PublishTeleop(bool clutch, bool leftTracked, bool rightTracked,
            Vector3 left, Quaternion leftRotation, Vector3 right, Quaternion rightRotation)
        {
            if (Ros == null || !Ros.HasConnectionThread || Ros.HasConnectionError || (clutch && !CanTeleoperate)) return false;
            // A false packet is also sent when clock quality has been lost: it
            // can only request a hold, and the server watchdog remains decisive.
            Ros.Publish("/deictic/teleop/input", new StringMsg(JsonUtility.ToJson(new TeleopInput
            {
                session_id = teleopSession, sequence = ++teleopSequence, stamp = RosFrames.Now,
                clutch = clutch, left_tracked = leftTracked, right_tracked = rightTracked,
                left_position = TeleopPosition(left), right_position = TeleopPosition(right),
                left_rotation = TeleopRotation(leftRotation), right_rotation = TeleopRotation(rightRotation)
            })));
            return true;
        }
        void ClearPreview() { HasPreview = false; previewEpoch = -1; pendingGoalNanoseconds = -1; CommittedBasePoint = null; PreviewCleared?.Invoke(); }
        void OnApplicationPause(bool paused)
        {
            if (paused)
            {
                applicationWasPaused = true;
                InvalidateTracking("Application paused", "paused");
                clockSynchronized = false;
                clockSync.Reset();
            }
            else if (applicationWasPaused)
            {
                applicationWasPaused = false;
                InvalidateTracking("Application resumed", "resumed");
                clockSynchronized = false;
                clockSync.Reset();
            }
        }
    }
}
