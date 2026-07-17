using UnityEngine;

namespace Dexter.Spider
{
    [DisallowMultipleComponent]
    public sealed class SpiderEnvironmentForces : MonoBehaviour
    {
        [Header("Feature Toggles")]
        [SerializeField] private bool enableFoliageFriction = true;
        [SerializeField] private bool enableGroundFriction = false;
        [Header("Foliage Friction")]
        [SerializeField, Range(0.1f, 1f)] private float foliageMovementMultiplier = 0.3f;
        [SerializeField, Min(0.05f)] private float foliageCheckRadius = 0.9f;
        [SerializeField] private LayerMask foliageLayers = ~0;
        [Header("Ground Friction")]
        [SerializeField, Range(0.1f, 1f)] private float groundMovementMultiplier = 0.9f;
        private readonly Collider[] overlapBuffer = new Collider[64];

        public float GetMovementMultiplier(Vector3 position)
        {
            float result = enableGroundFriction ? groundMovementMultiplier : 1f;
            if (!enableFoliageFriction) return result;
            int count = Physics.OverlapSphereNonAlloc(position, foliageCheckRadius,
                overlapBuffer, foliageLayers, QueryTriggerInteraction.Ignore);
            for (int i = 0; i < count; i++)
            {
                Collider c = overlapBuffer[i];
                if (c == null || c.transform.IsChildOf(transform)) continue;
                // The base terrain is not foliage, even if its GameObject is
                // named after a grass/meadow environment.
                if (c is TerrainCollider) continue;
                Transform t = c.transform;
                while (t != null && t != transform)
                {
                    string n = t.name.ToLowerInvariant();
                    if (n.Contains("flower") || n.Contains("bush") ||
                        n.Contains("shrub") || n.Contains("plant") ||
                        n.Contains("foliage") || n.Contains("grass"))
                        return Mathf.Min(result, foliageMovementMultiplier);
                    t = t.parent;
                }
            }
            return result;
        }
    }
}
