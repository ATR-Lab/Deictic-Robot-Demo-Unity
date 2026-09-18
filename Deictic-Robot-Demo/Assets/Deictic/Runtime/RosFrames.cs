using System;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Deictic
{
    /// <summary>Unity right/up/forward to ROS forward/left/up. All lengths are meters.</summary>
    public static class RosFrames
    {
        public static double LocalNow => (DateTime.UtcNow.Ticks - DateTime.UnixEpoch.Ticks) / 1e7;
        public static double ClockOffsetSeconds { get; set; }
        public static double Now => LocalNow + ClockOffsetSeconds;
        public static HeaderMsg Header(string frame, double stamp = -1)
        {
            if (stamp < 0) stamp = Now;
            int seconds = (int)Math.Floor(stamp);
            return new HeaderMsg { frame_id = frame, stamp = new TimeMsg(seconds, (uint)((stamp - seconds) * 1e9)) };
        }
        public static PoseMsg Pose(Vector3 position, Quaternion rotation)
        {
            Vector3 p = FLU.ConvertFromRUF(position);
            Quaternion q = FLU.ConvertFromRUF(rotation);
            return new PoseMsg(new PointMsg(p.x, p.y, p.z), new QuaternionMsg(q.x, q.y, q.z, q.w));
        }
        public static PoseStampedMsg StampedPose(Vector3 p, Quaternion q, string frame, double stamp = -1)
            => new PoseStampedMsg(Header(frame, stamp), Pose(p, q));
        public static Matrix4x4 Matrix(double[] pose)
        {
            if (pose == null || pose.Length != 7) throw new ArgumentException("Expected xyz + xyzw pose");
            foreach (double v in pose) if (double.IsNaN(v) || double.IsInfinity(v)) throw new ArgumentException("Non-finite pose");
            var q = new Quaternion((float)pose[3], (float)pose[4], (float)pose[5], (float)pose[6]);
            if (Math.Abs(Quaternion.Dot(q, q) - 1) > .02) throw new ArgumentException("Quaternion must be normalized");
            return Matrix4x4.TRS(FLU.ConvertToRUF(new Vector3((float)pose[0], (float)pose[1], (float)pose[2])), FLU.ConvertToRUF(q), Vector3.one);
        }
        // Unlike a body frame, an optical frame has right/down/forward axes.
        // This post-rotation converts local optical coordinates into body FLU.
        public static PoseMsg OpticalPose(Vector3 worldPosition, Quaternion worldRotation)
        {
            PoseMsg p = Pose(worldPosition, worldRotation);
            var body = new Quaternion((float)p.orientation.x, (float)p.orientation.y, (float)p.orientation.z, (float)p.orientation.w);
            Quaternion optical = body * new Quaternion(-.5f, .5f, -.5f, .5f);
            p.orientation = new QuaternionMsg(optical.x, optical.y, optical.z, optical.w);
            return p;
        }
    }
}
