using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// Applies passive gravity and ground friction without owning gait or input.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class SpiderTerrainForces : MonoBehaviour
    {
        [SerializeField, Min(0f)] private float gravityAcceleration = 9.81f;
        [SerializeField, Min(0f)] private float groundFrictionCoefficient = 0.90f;
        [SerializeField, Min(0f)] private float maximumSlideSpeed = 0.375f;

        private Terrain activeTerrain;
        private Vector3 environmentalVelocity;

        public Vector3 ApplyForces(Vector3 worldPosition)
        {
            EnsureTerrain();
            if (TrySampleTerrainNormal(worldPosition, out Vector3 groundNormal))
            {
                Vector3 slopeAcceleration = Vector3.ProjectOnPlane(
                    Vector3.down * gravityAcceleration, groundNormal);
                environmentalVelocity += slopeAcceleration * Time.deltaTime;

                float frictionDeceleration = groundFrictionCoefficient *
                                             gravityAcceleration *
                                             Mathf.Max(0f, groundNormal.y);
                environmentalVelocity = Vector3.MoveTowards(
                    environmentalVelocity,
                    Vector3.zero,
                    frictionDeceleration * Time.deltaTime);
                environmentalVelocity = Vector3.ProjectOnPlane(
                    environmentalVelocity, groundNormal);
            }
            else
            {
                environmentalVelocity += Vector3.down *
                                         gravityAcceleration * Time.deltaTime;
            }

            environmentalVelocity = Vector3.ClampMagnitude(
                environmentalVelocity, maximumSlideSpeed);
            return worldPosition + environmentalVelocity * Time.deltaTime;
        }

        public void ResetForces()
        {
            environmentalVelocity = Vector3.zero;
        }

        public void StopOnCollision()
        {
            environmentalVelocity = Vector3.zero;
        }

        private void EnsureTerrain()
        {
            if (activeTerrain != null)
                return;
            activeTerrain = Terrain.activeTerrain;
            if (activeTerrain == null)
                activeTerrain = FindFirstObjectByType<Terrain>();
        }

        private bool TrySampleTerrainNormal(Vector3 worldPosition, out Vector3 normal)
        {
            normal = Vector3.up;
            if (activeTerrain == null || activeTerrain.terrainData == null)
                return false;

            Vector3 origin = activeTerrain.transform.position;
            Vector3 size = activeTerrain.terrainData.size;
            if (worldPosition.x < origin.x || worldPosition.x > origin.x + size.x ||
                worldPosition.z < origin.z || worldPosition.z > origin.z + size.z)
                return false;

            float normalizedX = Mathf.InverseLerp(
                origin.x, origin.x + size.x, worldPosition.x);
            float normalizedZ = Mathf.InverseLerp(
                origin.z, origin.z + size.z, worldPosition.z);
            normal = activeTerrain.terrainData.GetInterpolatedNormal(
                normalizedX, normalizedZ).normalized;
            return true;
        }

        private void OnValidate()
        {
            gravityAcceleration = Mathf.Max(0f, gravityAcceleration);
            groundFrictionCoefficient = Mathf.Max(0f, groundFrictionCoefficient);
            maximumSlideSpeed = Mathf.Max(0f, maximumSlideSpeed);
        }
    }
}
