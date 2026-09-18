using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Sensor;
using RosMessageTypes.Trajectory;
using UnityEngine;

namespace Deictic
{
    public sealed class K1RobotView : MonoBehaviour
    {
        public DeicticBridge bridge;
        public bool preview;
        readonly Dictionary<string, K1Joint> joints = new Dictionary<string, K1Joint>();
        JointTrajectoryMsg trajectory;
        float startTime;
        LineRenderer path;
        GameObject plannedEndpoint;
        Transform endpointLabel;
        Material previewMaterial;
        Material feedbackMaterial;
        Transform tool;
        Renderer[] renderers;
        void Start()
        {
            foreach (K1Joint joint in GetComponentsInChildren<K1Joint>()) joints[joint.jointName] = joint;
            foreach (Transform t in GetComponentsInChildren<Transform>()) if (t.name == "right_tool") tool = t;
            renderers = GetComponentsInChildren<Renderer>();
            if (preview)
            {
                foreach (Renderer r in renderers) r.enabled = false;
                renderers = renderers.Where(r => r.GetComponentsInParent<K1Joint>().Any(j =>
                    j.jointName == "aaright_shoulder_pitch_joint" || j.jointName == "right_shoulder_roll_joint" ||
                    j.jointName == "right_elbow_pitch_joint" || j.jointName == "right_elbow_yaw_joint")).ToArray();
                previewMaterial = DemoVisuals.Material(new Color(.1f, .8f, .85f, .3f));
                foreach (Renderer r in renderers) { r.sharedMaterial = previewMaterial; r.transform.localScale *= 1.004f; }
                bridge.PreviewReceived += SetPreview;
                bridge.PreviewCleared += Clear;
                path = DemoVisuals.Line("Planned tool path", new Color(.1f, 1f, .8f), .006f);
                feedbackMaterial = path.sharedMaterial;
                path.transform.SetParent(transform, false);
                path.useWorldSpace = false;
                // These graphics belong to the base frame, not the moving ghost arm.
                // Neither creates a collider that could intercept a target ray.
                plannedEndpoint = new GameObject("Preview target marker");
                plannedEndpoint.transform.SetParent(transform, false);
                var ring = plannedEndpoint.AddComponent<LineRenderer>();
                ring.useWorldSpace = false; ring.loop = true;
                ring.sharedMaterial = feedbackMaterial;
                ring.startWidth = ring.endWidth = .003f;
                ring.positionCount = 32;
                for (int i = 0; i < ring.positionCount; i++)
                {
                    float angle = 2 * Mathf.PI * i / ring.positionCount;
                    ring.SetPosition(i, new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle)) * .015f);
                }
                endpointLabel = new GameObject("Planned endpoint label").transform;
                endpointLabel.SetParent(plannedEndpoint.transform, false);
                endpointLabel.localPosition = new Vector3(.02f, .025f, 0);
                var label = endpointLabel.gameObject.AddComponent<TextMesh>();
                label.text = "Planned endpoint";
                label.characterSize = .0025f; label.fontSize = 32;
                label.color = new Color(.1f, 1f, .8f);
                Clear();
            }
            else bridge.JointStateReceived += SetState;
        }
        void SetState(JointStateMsg msg)
        {
            if (msg.name == null || msg.position == null || msg.name.Length != msg.position.Length) return;
            Apply(msg.name, msg.position);
        }
        void Apply(string[] names, double[] positions)
        {
            if (names == null || positions == null || names.Length != positions.Length) return;
            for (int i = 0; i < names.Length; i++)
                if (!double.IsNaN(positions[i]) && !double.IsInfinity(positions[i]) && joints.TryGetValue(names[i], out K1Joint joint)) joint.SetRadians(positions[i]);
        }
        void SetPreview(JointTrajectoryMsg message)
        {
            trajectory = message; startTime = Time.unscaledTime;
            foreach (Renderer r in renderers) r.enabled = true;
            if (tool)
            {
                path.positionCount = message.points.Length;
                for (int i = 0; i < message.points.Length; i++)
                {
                    Apply(message.joint_names, message.points[i].positions);
                    path.SetPosition(i, transform.InverseTransformPoint(tool.position));
                }
                // Prefer the exact correlated request over the IK endpoint. A view
                // with no local request can still label the preview endpoint honestly.
                bool hasCommittedTarget = bridge.CommittedBasePoint.HasValue;
                plannedEndpoint.transform.localPosition = hasCommittedTarget
                    ? bridge.CommittedBasePoint.Value : path.GetPosition(path.positionCount - 1);
                endpointLabel.GetComponent<TextMesh>().text = hasCommittedTarget ? "Committed target" : "Planned endpoint";
                plannedEndpoint.SetActive(true);
            }
        }
        static double Seconds(JointTrajectoryPointMsg p) => p.time_from_start.sec + p.time_from_start.nanosec * 1e-9;
        void Update()
        {
            if (bridge.HasAlignment)
            {
                Matrix4x4 worldFromBase = bridge.BaseFromWorld.inverse;
                transform.SetPositionAndRotation(worldFromBase.GetColumn(3), worldFromBase.rotation);
            }
            if (!preview || trajectory == null) return;
            if (endpointLabel && Camera.main) endpointLabel.rotation = Camera.main.transform.rotation;
            var points = trajectory.points;
            double duration = Seconds(points[points.Length - 1]);
            double t = duration > 0 ? (Time.unscaledTime - startTime) % (duration + 1) : 0;
            int hi = 0;
            while (hi < points.Length - 1 && Seconds(points[hi]) < t) hi++;
            if (hi == 0) { Apply(trajectory.joint_names, points[0].positions); return; }
            double a = Seconds(points[hi - 1]), b = Seconds(points[hi]);
            float blend = b > a ? Mathf.Clamp01((float)((t - a) / (b - a))) : 1;
            double[] q = new double[trajectory.joint_names.Length];
            for (int j = 0; j < q.Length; j++) q[j] = points[hi - 1].positions[j] * (1 - blend) + points[hi].positions[j] * blend;
            Apply(trajectory.joint_names, q);
        }
        void Clear()
        {
            trajectory = null;
            if (renderers != null) foreach (Renderer r in renderers) if (r) r.enabled = false;
            if (path) path.positionCount = 0;
            if (plannedEndpoint) plannedEndpoint.SetActive(false);
        }
        static void Release(Object owned)
        {
            if (!owned) return;
            if (Application.isPlaying) Destroy(owned); else DestroyImmediate(owned);
        }
        void OnDestroy()
        {
            if (bridge) { bridge.JointStateReceived -= SetState; bridge.PreviewReceived -= SetPreview; bridge.PreviewCleared -= Clear; }
            if (path) Release(path.gameObject);
            if (plannedEndpoint) Release(plannedEndpoint);
            Release(feedbackMaterial);
            Release(previewMaterial);
        }
    }
}
