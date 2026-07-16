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
        [SerializeField, Min(0f)] private float maximumSlideSpeed = 0.4f;
        [Tooltip("Additional spider-like adhesion to the surface. Full effect requires a supported, balanced stance.")]
        [SerializeField, Range(0f, 1f)] private float stickySurfaceGrip = 0.2f;

        [Header("Terrain Movement")]
        [Tooltip("Walking speed retained on the configured steep slope angle.")]
        [SerializeField, Range(0.1f, 1f)] private float steepSlopeSpeedMultiplier = 0.45f;
        [SerializeField, Range(1f, 70f)] private float steepSlopeAngle = 38f;

        [Header("Terrain Body Clearance")]
        [Tooltip("Distance from the body center used to sample front, back, left, and right terrain heights.")]
        [SerializeField, Min(0.05f)] private float bodyClearanceSampleRadius = 0.40f;
        [Tooltip("Extra vertical clearance above the highest terrain sample.")]
        [SerializeField, Min(0f)] private float bodyClearanceSafetyMargin = 0.04f;
        [Tooltip("Maximum visual body alignment to the terrain slope.")]
        [SerializeField, Range(0f, 25f)] private float maximumSlopeAlignmentDegrees = 12f;
        [SerializeField, Min(0.01f)] private float slopeAlignmentResponse = 3f;

        [Header("Body Weight")]
        [Tooltip("Mass of the spider body in kilograms. Higher mass requires more finger support to stand fully upright.")]
        [SerializeField, Min(0.01f)] private float bodyMassKg = 2.5f;
        [Tooltip("Body mass that full, balanced finger activity can support at the authored standing height.")]
        [SerializeField, Min(0.01f)] private float fullSupportCapacityKg = 2.5f;
        [Tooltip("Center of mass relative to the authored body center: X is lateral, Y is vertical, Z is forward.")]
        [SerializeField] private Vector3 centerOfMassOffset = Vector3.zero;
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
        private float currentCenterPitch;
        private Quaternion currentTerrainTilt = Quaternion.identity;
        private float currentGripStability;

        public Vector3 ApplyForces(Vector3 worldPosition)
        {
            EnsureTerrain();
            if (TrySampleTerrainNormal(worldPosition, out Vector3 groundNormal))
            {
                Vector3 slopeAcceleration = Vector3.ProjectOnPlane(
                    Vector3.down * gravityAcceleration, groundNormal);
                environmentalVelocity += slopeAcceleration * Time.deltaTime;

                float normalStrength = gravityAcceleration *
                                       Mathf.Max(0f, groundNormal.y);
                float frictionDeceleration = groundFrictionCoefficient *
                                             normalStrength;
                float adhesionDeceleration = stickySurfaceGrip *
                                             currentGripStability *
                                             normalStrength;
                environmentalVelocity = Vector3.MoveTowards(
                    environmentalVelocity,
                    Vector3.zero,
                    (frictionDeceleration + adhesionDeceleration) *
                    Time.deltaTime);
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

        public float GetMovementMultiplier(Vector3 worldPosition)
        {
            EnsureTerrain();
            if (!TrySampleTerrainNormal(worldPosition, out Vector3 normal))
                return 1f;

            float slope = Vector3.Angle(normal, Vector3.up);
            return Mathf.Lerp(
                1f,
                steepSlopeSpeedMultiplier,
                Mathf.InverseLerp(0f, steepSlopeAngle, slope));
        }

        public Quaternion GetSlopeAlignedRootRotation(
            Vector3 worldPosition,
            Quaternion currentRotation)
        {
            EnsureTerrain();
            if (!TrySampleTerrainNormal(worldPosition, out Vector3 normal))
            {
                currentTerrainTilt = Quaternion.Slerp(
                    currentTerrainTilt,
                    Quaternion.identity,
                    1f - Mathf.Exp(-slopeAlignmentResponse * Time.deltaTime));
                return currentRotation;
            }

            Vector3 limitedUp = Vector3.RotateTowards(
                Vector3.up,
                normal,
                maximumSlopeAlignmentDegrees * Mathf.Deg2Rad,
                0f);
            Quaternion targetTilt = Quaternion.FromToRotation(
                Vector3.up, limitedUp);
            float blend = 1f - Mathf.Exp(-slopeAlignmentResponse * Time.deltaTime);
            currentTerrainTilt = Quaternion.Slerp(
                currentTerrainTilt, targetTilt, blend);
            return currentTerrainTilt * currentRotation;
        }

        public bool TryGetRequiredBodyHeight(
            Vector3 worldPosition,
            Vector3 rigForward,
            Vector3 rigRight,
            float authoredGroundClearance,
            out float requiredHeight)
        {
            EnsureTerrain();
            requiredHeight = worldPosition.y;
            Vector3 forward = Vector3.ProjectOnPlane(rigForward, Vector3.up).normalized;
            Vector3 right = Vector3.ProjectOnPlane(rigRight, Vector3.up).normalized;
            Vector3[] offsets =
            {
                Vector3.zero,
                forward * bodyClearanceSampleRadius,
                -forward * bodyClearanceSampleRadius,
                right * bodyClearanceSampleRadius,
                -right * bodyClearanceSampleRadius
            };

            bool foundTerrain = false;
            float highestTerrain = float.NegativeInfinity;
            for (int i = 0; i < offsets.Length; i++)
            {
                if (!TrySampleTerrainHeight(worldPosition + offsets[i], out float height))
                    continue;
                highestTerrain = Mathf.Max(highestTerrain, height);
                foundTerrain = true;
            }

            if (foundTerrain)
                requiredHeight = highestTerrain + authoredGroundClearance +
                                 bodyClearanceSafetyMargin;
            return foundTerrain;
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
            float totalVirtualSupport,
            float normalizedCadenceImbalance,
            Vector3 rigRightWorld,
            Vector3 rigForwardWorld)
        {
            if (body == null)
                return;

            // Gravity wins at zero support. Matching, sustained finger motion
            // supplies the virtual string tension needed to reach full height.
            float supportedFraction = Mathf.Clamp01(
                totalVirtualSupport * fullSupportCapacityKg / bodyMassKg);
            currentGripStability = supportedFraction *
                (1f - 0.5f * Mathf.Clamp01(
                    Mathf.Abs(normalizedCadenceImbalance)));
            float targetDrop = restingBodyDrop *
                               (1f - supportedFraction);
            float massResponse = Mathf.Clamp(
                fullSupportCapacityKg / bodyMassKg, 0.25f, 4f);
            currentBodyDrop = Mathf.MoveTowards(
                currentBodyDrop,
                targetDrop,
                bodyHeightResponse * restingBodyDrop * massResponse * Time.deltaTime);

            Vector3 safeRight = Vector3.ProjectOnPlane(
                rigRightWorld, Vector3.up).normalized;
            float lateralCenterBias = maximumBalanceShift > 0.0001f
                ? centerOfMassOffset.x / maximumBalanceShift
                : 0f;
            // A higher center of mass has more tipping leverage; lowering it
            // makes the spider naturally more stable.
            float centerHeightLeverage = Mathf.Clamp(
                1f + centerOfMassOffset.y /
                Mathf.Max(0.01f, restingBodyDrop), 0.5f, 2f);
            float weightedImbalance = Mathf.Clamp(
                (normalizedCadenceImbalance * cadenceBalanceSensitivity +
                 lateralCenterBias) * centerHeightLeverage,
                -1f,
                1f);
            Vector3 worldOffset = -Vector3.up * currentBodyDrop -
                                  safeRight * weightedImbalance * maximumBalanceShift;
            Vector3 localOffset = body.parent != null
                ? body.parent.InverseTransformVector(worldOffset)
                : worldOffset;
            body.localPosition = restLocalPosition + localOffset;

            float targetTip = weightedImbalance *
                              maximumBalanceTipDegrees;
            float forwardCenterBias = maximumBalanceShift > 0.0001f
                ? centerOfMassOffset.z / maximumBalanceShift
                : 0f;
            float targetPitch = Mathf.Clamp(
                forwardCenterBias * centerHeightLeverage, -1f, 1f) *
                maximumBalanceTipDegrees;
            currentBalanceTip = Mathf.MoveTowards(
                currentBalanceTip,
                targetTip,
                balanceTipResponse * maximumBalanceTipDegrees * Time.deltaTime);
            currentCenterPitch = Mathf.MoveTowards(
                currentCenterPitch,
                targetPitch,
                balanceTipResponse * maximumBalanceTipDegrees * Time.deltaTime);
            body.localRotation = restLocalRotation;
            Vector3 safeForward = Vector3.ProjectOnPlane(
                rigForwardWorld, Vector3.up).normalized;
            Vector3 safeBodyRight = Vector3.ProjectOnPlane(
                rigRightWorld, Vector3.up).normalized;
            if (safeForward.sqrMagnitude > 0.0001f)
                body.rotation = Quaternion.AngleAxis(
                    currentBalanceTip, safeForward) * body.rotation;
            if (safeBodyRight.sqrMagnitude > 0.0001f)
                body.rotation = Quaternion.AngleAxis(
                    currentCenterPitch, safeBodyRight) * body.rotation;
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

        private bool TrySampleTerrainHeight(Vector3 worldPosition, out float height)
        {
            height = 0f;
            if (activeTerrain == null || activeTerrain.terrainData == null)
                return false;

            Vector3 origin = activeTerrain.transform.position;
            Vector3 size = activeTerrain.terrainData.size;
            if (worldPosition.x < origin.x || worldPosition.x > origin.x + size.x ||
                worldPosition.z < origin.z || worldPosition.z > origin.z + size.z)
                return false;

            height = activeTerrain.SampleHeight(worldPosition) + origin.y;
            return true;
        }

        private void OnValidate()
        {
            gravityAcceleration = Mathf.Max(0f, gravityAcceleration);
            groundFrictionCoefficient = Mathf.Max(0f, groundFrictionCoefficient);
            maximumSlideSpeed = Mathf.Max(0f, maximumSlideSpeed);
            stickySurfaceGrip = Mathf.Clamp01(stickySurfaceGrip);
            bodyMassKg = Mathf.Max(0.01f, bodyMassKg);
            fullSupportCapacityKg = Mathf.Max(0.01f, fullSupportCapacityKg);
            steepSlopeSpeedMultiplier = Mathf.Clamp(
                steepSlopeSpeedMultiplier, 0.1f, 1f);
            steepSlopeAngle = Mathf.Clamp(steepSlopeAngle, 1f, 70f);
            bodyClearanceSampleRadius = Mathf.Max(0.05f, bodyClearanceSampleRadius);
            bodyClearanceSafetyMargin = Mathf.Max(0f, bodyClearanceSafetyMargin);
            slopeAlignmentResponse = Mathf.Max(0.01f, slopeAlignmentResponse);
            restingBodyDrop = Mathf.Max(0f, restingBodyDrop);
            bodyHeightResponse = Mathf.Max(0.01f, bodyHeightResponse);
            maximumBalanceShift = Mathf.Max(0f, maximumBalanceShift);
            balanceTipResponse = Mathf.Max(0.01f, balanceTipResponse);
        }
    }
}
