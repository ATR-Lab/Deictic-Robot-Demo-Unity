using UnityEngine;
namespace Deictic
{
    public sealed class K1Joint : MonoBehaviour
    {
        public string jointName;
        public Vector3 axis;
        public Quaternion restRotation = Quaternion.identity;
        public void SetRadians(double radians) => transform.localRotation = restRotation * Quaternion.AngleAxis(-(float)radians * Mathf.Rad2Deg, axis);
    }
}
