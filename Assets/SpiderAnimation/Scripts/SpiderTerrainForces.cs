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

        [Header("Body Weight")]
        [Tooltip("Small world-space distance the body settles while the spider is resting.")]
        [SerializeField, Min(0f)] private float restingBodyDrop = 0.09f;
        [Tooltip("How quickly the body settles and rises without affecting locomotion.")]
        [SerializeField, Min(0.01f)] private float bodyHeightResponse = 3f;
        [Tooltip("Maximum body tip when one finger is moving faster than the other.")]
        [SerializeField, Range(0f, 8f)] private float maximumBalanceTipDegrees = 4.5f;
        [Tooltip("Amplifies small differences in index and middle-finger rhythm.")]
        [SerializeField, Range(0.5f, 3f)] private float cadenceBalanceSensitivity = 1.75f;
        [Tooltip("Maximum world-space body shift toward the faster-working side.")]
        [SerializeField, Min(0f)] private float maximumBalanceShift = 0.10f;
        [Tooltip("How quickly the body responds to a left/right cadence imbalance.")]
        [SerializeField, Min(0.01f)] private float balanceTipResponse = 3f;

        private Terrain activeTerrain;
        private Vector3 environmentalVelocity;
        private float currentBodyDrop;
        private float currentBalanceTip;

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

        /// <summary>
        /// Applies a bounded visual weight offset after IK has finished. Because
        /// the controller restores the authored pose before the next solve, this
        /// cannot accumulate or feed back into locomotion.
        /// </summary>
        public void ApplyBodyWeightAfterIk(
            Transform body,
            Vector3 restLocalPosition,
            Quaternion restLocalRotation,
            bool isMoving,
            float normalizedCadenceImbalance,
            Vector3 rigRightWorld,
            Vector3 rigForwardWorld)
        {
            if (body == null)
                return;

            float targetDrop = isMoving ? 0f : restingBodyDrop;
            currentBodyDrop = Mathf.MoveTowards(
                currentBodyDrop,
                targetDrop,
                bodyHeightResponse * restingBodyDrop * Time.deltaTime);

            float weightedImbalance = Mathf.Clamp(
                normalizedCadenceImbalance * cadenceBalanceSensitivity,
                -1f,
                1f);
            Vector3 safeRight = Vector3.ProjectOnPlane(
                rigRightWorld, Vector3.up).normalized;
            Vector3 worldOffset = -Vector3.up * currentBodyDrop -
                                  safeRight * weightedImbalance * maximumBalanceShift;
            Vector3 localOffset = body.parent != null
                ? body.parent.InverseTransformVector(worldOffset)
                : worldOffset;
            body.localPosition = restLocalPosition + localOffset;

            float targetTip = weightedImbalance *
                              maximumBalanceTipDegrees;
            currentBalanceTip = Mathf.MoveTowards(
                currentBalanceTip,
                targetTip,
                balanceTipResponse * maximumBalanceTipDegrees * Time.deltaTime);
            body.localRotation = restLocalRotation;
            Vector3 safeForward = Vector3.ProjectOnPlane(
                rigForwardWorld, Vector3.up).normalized;
            if (safeForward.sqrMagnitude > 0.0001f)
                body.rotation = Quaternion.AngleAxis(
                    currentBalanceTip, safeForward) * body.rotation;
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
            restingBodyDrop = Mathf.Max(0f, restingBodyDrop);
            bodyHeightResponse = Mathf.Max(0.01f, bodyHeightResponse);
            maximumBalanceShift = Mathf.Max(0f, maximumBalanceShift);
            balanceTipResponse = Mathf.Max(0.01f, balanceTipResponse);
        }
    }
}
