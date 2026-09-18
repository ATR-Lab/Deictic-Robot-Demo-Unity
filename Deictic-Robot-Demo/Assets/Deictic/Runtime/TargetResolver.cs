using UnityEngine;

namespace Deictic
{
    public static class TargetResolver
    {
        public static bool Resolve(bool hasHead, RaycastHit head, bool hasPointer, RaycastHit pointer,
            float agreement, out Vector3 point, out Vector3 normal)
        {
            point = Vector3.zero; normal = Vector3.up;
            if (!hasHead && !hasPointer) return false;
            RaycastHit chosen = hasPointer ? pointer : head;
            point = chosen.point;
            normal = chosen.normal.sqrMagnitude > .5f ? chosen.normal.normalized : Vector3.up;
            if (hasHead && hasPointer && Vector3.Distance(head.point, pointer.point) <= agreement)
                point = (head.point + pointer.point) * .5f;
            return true;
        }
    }
}
