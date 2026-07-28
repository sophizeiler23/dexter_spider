using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// Applies passive gravity and ground friction without owning gait or input.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class SpiderTerrainForces : MonoBehaviour
    {
        [Header("Gravity")]
        [SerializeField, Min(0f)] private float gravityAcceleration = 9.81f;

        [Header("Friction and Grip")]
        [SerializeField, Min(0f)] private float groundFrictionCoefficient = 0.90f;
        [SerializeField, Min(0f)] private float maximumSlideSpeed = 0.4f;
        [Tooltip("Additional spider-like adhesion to the surface. Full effect requires a supported, balanced stance.")]
        [SerializeField, Range(0f, 1f)] private float stickySurfaceGrip = 0.2f;
        [Tooltip("Small amount of drive retained while the next tetrapod group is planting. Lower values make the push-pull rhythm more pronounced.")]
        [SerializeField, Range(0f, 1f)] private float minimumPlantedTraction = 0.35f;
        [Tooltip("Shapes how quickly traction builds after the feet plant. Values above one emphasize the loaded push phase.")]
        [SerializeField, Range(0.5f, 3f)] private float plantedTractionExponent = 1.35f;
        [Tooltip("How quickly commanded walking speed builds while planted feet push.")]
        [SerializeField, Min(0.01f)] private float walkingAcceleration = 5f;
        [Tooltip("How quickly surface friction removes walking speed when the push ends or input is released.")]
        [SerializeField, Min(0.01f)] private float walkingBrakingDeceleration = 12f;

        [Header("Experimental Surface Forces (PhilS94-inspired)")]
        [Tooltip("Uses explicit mass, gravity, adhesion, friction, and drag forces for passive terrain motion.")]
        [SerializeField] private bool enableExplicitSurfaceForces = true;
        [Tooltip("Maximum active adhesion force holding the spider to a surface, in newtons.")]
        [SerializeField, Min(0f)] private float surfaceAdhesionForceNewtons = 18f;
        [Tooltip("Velocity-proportional damping along the contacted surface.")]
        [SerializeField, Min(0f)] private float surfaceLinearDrag = 1.5f;
        [Tooltip("Below this tangential speed, sufficient grip settles the spider completely.")]
        [SerializeField, Min(0f)] private float staticGripVelocity = 0.025f;

        [Header("Terrain Movement")]
        [Tooltip("Walking speed retained on the configured steep slope angle.")]
        [SerializeField, Range(0.1f, 1f)] private float steepSlopeSpeedMultiplier = 0.45f;
        [SerializeField, Range(1f, 70f)] private float steepSlopeAngle = 38f;

        [Header("Steep Surface Climbing")]
        [Tooltip("Allows terrain heightmap walls to be treated as walkable surfaces instead of obstacles.")]
        [SerializeField] private bool enableSteepSurfaceClimbing = true;
        [Tooltip("Surface angle where the spider switches from normal slope handling to wall climbing.")]
        [SerializeField, Range(20f, 85f)] private float climbStartAngle = 45f;
        [Tooltip("Steepest terrain angle the spider can align to and climb.")]
        [SerializeField, Range(45f, 89.9f)] private float maximumClimbAngle = 89.5f;
        [Tooltip("Distance ahead used to detect a steep wall before the body reaches it.")]
        [SerializeField, Min(0.05f)] private float climbSurfaceProbeDistance = 0.6f;
        [Tooltip("Required agreement between a collider face and the TerrainData normal. This keeps tree trunks blocking.")]
        [SerializeField, Range(0.25f, 1f)] private float climbNormalMatch = 0.7f;
        [Tooltip("Extra gravity-cancelling adhesion on climbable walls. A value above one holds the spider against a near-vertical surface at rest.")]
        [SerializeField, Range(1f, 2f)] private float steepSurfaceAdhesionMultiplier = 1.2f;
        [Tooltip("Minimum fraction of wall adhesion retained even while the fingers are neutral.")]
        [SerializeField, Range(0f, 1f)] private float minimumClimbGrip = 1f;
        [Tooltip("Distance used to project each foot onto the active steep TerrainCollider face.")]
        [SerializeField, Min(0.5f)] private float steepFootContactProbeDistance = 4f;

        [Header("Terrain Body Clearance")]
        [Tooltip("Distance from the body center used to sample front, back, left, and right terrain heights.")]
        [SerializeField, Min(0.05f)] private float bodyClearanceSampleRadius = 0.40f;
        [Tooltip("Extra vertical clearance above the highest terrain sample.")]
        [SerializeField, Min(0f)] private float bodyClearanceSafetyMargin = 0.25f;
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
        [Tooltip("World-space body drop retained both at rest and while walking.")]
        [SerializeField, Min(0f)] private float restingBodyDrop = 0.76f;
        [Tooltip("Keeps the visible torso above the sampled terrain surface.")]
        [SerializeField, Min(0f)] private float torsoSurfaceClearance = 0.12f;
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
        private TerrainCollider activeTerrainCollider;
        private Vector3 environmentalVelocity;
        private float currentBodyDrop;
        private float currentBalanceTip;
        private float currentCenterPitch;
        private Quaternion currentTerrainTilt = Quaternion.identity;
        private float currentGripStability;
        private Vector3 previousRootSurfaceNormal = Vector3.up;
        private Vector3 cornerFromNormal = Vector3.up;
        private Vector3 cornerToNormal = Vector3.up;
        private bool hasPreviousRootSurfaceNormal;
        private bool rootCornerTransitionActive;

        public Vector3 ApplyForces(Vector3 worldPosition)
        {
            EnsureTerrain();
            if (TrySampleTerrainNormal(worldPosition, out Vector3 groundNormal))
            {
                if (enableExplicitSurfaceForces)
                {
                    IntegrateExplicitSurfaceForces(groundNormal);
                    environmentalVelocity = Vector3.ClampMagnitude(
                        environmentalVelocity, maximumSlideSpeed);
                    return worldPosition + environmentalVelocity * Time.deltaTime;
                }

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
                if (IsClimbableNormal(groundNormal))
                {
                    // On a vertical wall groundNormal.y approaches zero, so
                    // ordinary normal-force friction also approaches zero.
                    // Spider adhesion is an active surface grip and must not
                    // disappear with the gravitational normal force.
                    float climbAdhesion = stickySurfaceGrip *
                        Mathf.Max(currentGripStability, minimumClimbGrip) *
                        gravityAcceleration * steepSurfaceAdhesionMultiplier;
                    adhesionDeceleration = Mathf.Max(
                        adhesionDeceleration, climbAdhesion);
                }
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

        private void IntegrateExplicitSurfaceForces(Vector3 groundNormal)
        {
            Vector3 normal = groundNormal.normalized;
            float mass = Mathf.Max(0.01f, bodyMassKg);
            Vector3 gravityForce = Vector3.down * mass * gravityAcceleration;
            Vector3 tangentGravityForce = Vector3.ProjectOnPlane(
                gravityForce, normal);
            environmentalVelocity += tangentGravityForce / mass * Time.deltaTime;

            float passiveNormalForce = Mathf.Max(
                0f, -Vector3.Dot(gravityForce, normal));
            float gripStability = IsClimbableNormal(normal)
                ? Mathf.Max(currentGripStability, minimumClimbGrip)
                : currentGripStability;
            float activeAdhesionForce = stickySurfaceGrip *
                                        gripStability *
                                        surfaceAdhesionForceNewtons;
            float maximumFrictionForce =
                groundFrictionCoefficient * passiveNormalForce +
                activeAdhesionForce;
            float frictionAcceleration = maximumFrictionForce / mass;
            environmentalVelocity = Vector3.MoveTowards(
                environmentalVelocity,
                Vector3.zero,
                frictionAcceleration * Time.deltaTime);
            environmentalVelocity *= Mathf.Exp(
                -surfaceLinearDrag * Time.deltaTime);
            environmentalVelocity = Vector3.ProjectOnPlane(
                environmentalVelocity, normal);

            if (environmentalVelocity.magnitude <= staticGripVelocity &&
                tangentGravityForce.magnitude <= maximumFrictionForce)
                environmentalVelocity = Vector3.zero;
        }

        public float GetMovementMultiplier(Vector3 worldPosition)
        {
            EnsureTerrain();
            if (!TryGetTravelSurfaceNormal(
                    worldPosition, Vector3.zero, out Vector3 normal))
                return 1f;

            float slope = Vector3.Angle(normal, Vector3.up);
            return Mathf.Lerp(
                1f,
                steepSlopeSpeedMultiplier,
                Mathf.InverseLerp(0f, steepSlopeAngle, slope));
        }

        public Vector3 GetSurfaceTravelDirection(
            Vector3 worldPosition,
            Vector3 intendedForward)
        {
            EnsureTerrain();
            Vector3 planarForward = Vector3.ProjectOnPlane(
                intendedForward, Vector3.up).normalized;
            if (planarForward.sqrMagnitude < 0.0001f)
                return intendedForward.normalized;
            // Translation must follow the surface currently under the root.
            // Using the uphill look-ahead normal here turns the travel vector
            // almost vertical before the root reaches a sharp wall, removing
            // the horizontal progress needed to enter that wall sample. Pose
            // and foot preparation may still anticipate the upcoming surface.
            if (!TrySampleTerrainNormal(
                    worldPosition, out Vector3 normal))
                return planarForward;

            // Building forward from a stable lateral axis remains well-defined
            // when the surface is almost vertical. A direct projection of
            // planarForward becomes nearly zero at 90 degrees and can suddenly
            // choose a sideways direction at the rim of a trough.
            Vector3 planarRight = Vector3.Cross(
                Vector3.up, planarForward).normalized;
            Vector3 surfaceRight = Vector3.ProjectOnPlane(
                planarRight, normal).normalized;
            if (surfaceRight.sqrMagnitude < 0.0001f)
            {
                Vector3 projectedForward = Vector3.ProjectOnPlane(
                    planarForward, normal).normalized;
                return projectedForward.sqrMagnitude > 0.0001f
                    ? projectedForward
                    : planarForward;
            }

            Vector3 surfaceForward = Vector3.Cross(
                surfaceRight, normal).normalized;
            Vector3 projectedDirection = Vector3.ProjectOnPlane(
                planarForward, normal);
            if (projectedDirection.sqrMagnitude > 0.000001f &&
                Vector3.Dot(surfaceForward, projectedDirection) < 0f)
                surfaceForward = -surfaceForward;
            return surfaceForward;
        }

        public bool IsClimbableTerrainContact(
            Vector3 worldPoint,
            Vector3 colliderNormal,
            Vector3 approachDirection)
        {
            EnsureTerrain();
            if (!enableSteepSurfaceClimbing)
                return false;

            Vector3 normalizedColliderNormal = colliderNormal.normalized;
            if (IsMatchingClimbableTerrainNormal(
                    worldPoint, normalizedColliderNormal))
                return true;

            // At a hard ground/wall corner, the collision face can be reached
            // one frame before TerrainData at the body center reports a steep
            // normal. Probe just beyond the contact in the approach direction
            // so that collision does not prevent the root from ever entering
            // the climbable sample. Ordinary terrain-painted tree contacts
            // remain blocking because nearby TerrainData stays ground-like.
            Vector3 planarApproach = Vector3.ProjectOnPlane(
                approachDirection, Vector3.up).normalized;
            if (planarApproach.sqrMagnitude < 0.0001f)
                return false;

            return IsMatchingClimbableTerrainNormal(
                worldPoint + planarApproach * climbSurfaceProbeDistance,
                normalizedColliderNormal);
        }

        private bool IsMatchingClimbableTerrainNormal(
            Vector3 worldPoint,
            Vector3 colliderNormal)
        {
            return TrySampleTerrainNormal(
                       worldPoint, out Vector3 terrainNormal) &&
                   IsClimbableNormal(terrainNormal) &&
                   Vector3.Dot(
                       terrainNormal, colliderNormal) >= climbNormalMatch;
        }

        /// <summary>
        /// Returns a foot contact offset away from a steep Terrain surface
        /// along its normal. Height-only grounding cannot plant a foot on a
        /// near-vertical wall because world up is no longer the support axis.
        /// </summary>
        public bool TryGetClimbSurfaceContact(
            Vector3 worldPosition,
            float surfaceClearance,
            out Vector3 contact,
            out Vector3 surfaceNormal)
        {
            EnsureTerrain();
            contact = worldPosition;
            surfaceNormal = Vector3.up;
            if (!TrySampleTerrainNormal(worldPosition, out surfaceNormal) ||
                !IsClimbableNormal(surfaceNormal) ||
                !TrySampleTerrainHeight(worldPosition, out float height))
                return false;

            Vector3 surfacePoint = new Vector3(
                worldPosition.x, height, worldPosition.z);
            contact = surfacePoint + surfaceNormal.normalized *
                      Mathf.Max(0f, surfaceClearance);
            return true;
        }

        public bool TryGetTerrainSurfaceFrame(
            Vector3 worldPosition,
            out Vector3 surfacePoint,
            out Vector3 surfaceNormal)
        {
            EnsureTerrain();
            surfacePoint = worldPosition;
            surfaceNormal = Vector3.up;
            if (!TrySampleTerrainNormal(worldPosition, out surfaceNormal) ||
                !TrySampleTerrainHeight(worldPosition, out float height))
                return false;
            surfacePoint = new Vector3(
                worldPosition.x, height, worldPosition.z);
            return true;
        }

        /// <summary>
        /// Builds one continuous support pose for the root across a sharp
        /// ground/wall normal discontinuity. Clearance is enforced against the
        /// current face and both faces of an active corner, so procedural root
        /// motion cannot cut through either surface while its up axis blends.
        /// </summary>
        public bool TryGetContinuousRootSurfacePose(
            Vector3 worldPosition,
            float surfaceClearance,
            out Vector3 surfacePoint,
            out Vector3 rawSurfaceNormal,
            out Vector3 supportNormal,
            out Vector3 rootPosition,
            out bool isCornerTransition)
        {
            rootPosition = worldPosition;
            supportNormal = Vector3.up;
            isCornerTransition = false;
            if (!TryGetTerrainSurfaceFrame(
                    worldPosition,
                    out surfacePoint,
                    out rawSurfaceNormal))
                return false;

            supportNormal = (currentTerrainTilt * Vector3.up).normalized;
            if (supportNormal.sqrMagnitude < 0.0001f)
                supportNormal = rawSurfaceNormal;

            if (!hasPreviousRootSurfaceNormal)
            {
                previousRootSurfaceNormal = rawSurfaceNormal;
                hasPreviousRootSurfaceNormal = true;
            }

            if (!rootCornerTransitionActive && Vector3.Angle(
                    previousRootSurfaceNormal,
                    rawSurfaceNormal) >= 20f)
            {
                rootCornerTransitionActive = true;
                cornerFromNormal = previousRootSurfaceNormal;
                cornerToNormal = rawSurfaceNormal;
            }

            isCornerTransition = rootCornerTransitionActive;

            float clearance = Mathf.Max(0f, surfaceClearance);
            Vector3 clearanceOffset = supportNormal * clearance;
            EnforceNormalClearance(
                ref clearanceOffset, rawSurfaceNormal, clearance);
            if (rootCornerTransitionActive)
            {
                EnforceNormalClearance(
                    ref clearanceOffset, cornerFromNormal, clearance);
                EnforceNormalClearance(
                    ref clearanceOffset, cornerToNormal, clearance);

                bool reachedNewFace = Vector3.Angle(
                    supportNormal, cornerToNormal) <= 8f;
                bool currentFaceIsStable = Vector3.Angle(
                    rawSurfaceNormal, cornerToNormal) <= 12f;
                if (reachedNewFace && currentFaceIsStable)
                    rootCornerTransitionActive = false;
            }

            previousRootSurfaceNormal = rawSurfaceNormal;
            rootPosition = surfacePoint + clearanceOffset;
            return true;
        }

        private static void EnforceNormalClearance(
            ref Vector3 offset,
            Vector3 surfaceNormal,
            float requiredClearance)
        {
            Vector3 normal = surfaceNormal.normalized;
            if (normal.sqrMagnitude < 0.0001f)
                return;

            float missingClearance = requiredClearance -
                                     Vector3.Dot(offset, normal);
            if (missingClearance > 0f)
                offset += normal * missingClearance;
        }

        public bool TryProjectFootToClimbSurface(
            Vector3 desiredFootPosition,
            Vector3 preferredSurfaceNormal,
            float surfaceClearance,
            out Vector3 contact,
            out Vector3 surfaceNormal,
            bool allowTransitionSurface = false)
        {
            EnsureTerrain();
            contact = desiredFootPosition;
            surfaceNormal = preferredSurfaceNormal.normalized;
            if (activeTerrainCollider == null ||
                (!allowTransitionSurface &&
                 !IsClimbableNormal(surfaceNormal)))
                return false;

            float probeDistance = Mathf.Max(
                0.5f, steepFootContactProbeDistance);
            Ray ray = new Ray(
                desiredFootPosition + surfaceNormal * probeDistance,
                -surfaceNormal);
            if (!activeTerrainCollider.Raycast(
                    ray, out RaycastHit hit, probeDistance * 2f))
                return false;
            if (Vector3.Dot(hit.normal, surfaceNormal) < climbNormalMatch)
                return false;

            surfaceNormal = hit.normal.normalized;
            contact = hit.point + surfaceNormal *
                      Mathf.Max(0f, surfaceClearance);
            return true;
        }

        public bool IsClimbableSurfaceNormal(Vector3 surfaceNormal)
        {
            return IsClimbableNormal(surfaceNormal);
        }

        public bool ShouldAnticipateClimbTransition(
            Vector3 worldPosition,
            Vector3 intendedForward,
            Vector3 rawSurfaceNormal,
            Vector3 supportNormal)
        {
            if (!enableSteepSurfaceClimbing)
                return false;

            float rawSlope = Vector3.Angle(
                rawSurfaceNormal, Vector3.up);
            float supportSlope = Vector3.Angle(
                supportNormal, Vector3.up);
            bool supportAlreadyTurning =
                   rawSlope >= Mathf.Max(0f, climbStartAngle - 15f) &&
                   rawSlope < climbStartAngle &&
                   supportSlope >= climbStartAngle &&
                   Vector3.Angle(rawSurfaceNormal, supportNormal) >= 8f;
            if (supportAlreadyTurning)
                return true;

            Vector3 planarForward = Vector3.ProjectOnPlane(
                intendedForward, Vector3.up).normalized;
            if (planarForward.sqrMagnitude < 0.0001f)
                return false;

            Vector3 probePosition = worldPosition + planarForward *
                (climbSurfaceProbeDistance * 3f);
            if (!TrySampleTerrainNormal(
                    probePosition, out Vector3 probeNormal) ||
                !IsClimbableNormal(probeNormal))
                return false;

            bool hasCurrentHeight = TrySampleTerrainHeight(
                worldPosition, out float currentHeight);
            bool hasProbeHeight = TrySampleTerrainHeight(
                probePosition, out float probeHeight);
            return !hasCurrentHeight || !hasProbeHeight ||
                   probeHeight > currentHeight + 0.01f;
        }

        public float GetWalkingTractionMultiplier(float plantedPush)
        {
            float shapedPush = Mathf.Pow(
                Mathf.Clamp01(plantedPush), plantedTractionExponent);
            return Mathf.Lerp(minimumPlantedTraction, 1f, shapedPush);
        }

        public float MoveWalkingSpeed(
            float currentSpeed,
            float requestedSpeed,
            float deltaTime)
        {
            float response = Mathf.Abs(requestedSpeed) < Mathf.Abs(currentSpeed)
                ? walkingBrakingDeceleration
                : walkingAcceleration;
            return Mathf.MoveTowards(
                currentSpeed, requestedSpeed, response * deltaTime);
        }

        public Quaternion GetSlopeAlignedRootRotation(
            Vector3 worldPosition,
            Quaternion currentRotation,
            Vector3 movementDirection)
        {
            EnsureTerrain();
            Vector3 planarForward = Vector3.ProjectOnPlane(
                currentRotation * Vector3.forward, Vector3.up).normalized;
            if (!TryGetTravelSurfaceNormal(
                    worldPosition, planarForward, out Vector3 normal))
            {
                currentTerrainTilt = Quaternion.Slerp(
                    currentTerrainTilt,
                    Quaternion.identity,
                    1f - Mathf.Exp(-slopeAlignmentResponse * Time.deltaTime));
                return currentRotation;
            }

            float slope = Vector3.Angle(normal, Vector3.up);
            float alignmentLimit = enableSteepSurfaceClimbing &&
                                   slope >= climbStartAngle
                ? maximumClimbAngle
                : maximumSlopeAlignmentDegrees;
            Vector3 limitedUp = Vector3.RotateTowards(
                Vector3.up,
                normal,
                alignmentLimit * Mathf.Deg2Rad,
                0f);
            Quaternion targetTilt = Quaternion.FromToRotation(
                Vector3.up, limitedUp);
            float blend = 1f - Mathf.Exp(-slopeAlignmentResponse * Time.deltaTime);
            currentTerrainTilt = Quaternion.Slerp(
                currentTerrainTilt, targetTilt, blend);

            // Applying a generic tilt to the old yaw does not guarantee that
            // transform.forward matches the projected travel vector. The
            // mismatch becomes conspicuous on diagonal and near-vertical
            // surfaces, where the spider can appear to strafe sideways.
            Vector3 alignedUp = currentTerrainTilt * Vector3.up;
            Vector3 alignedForward = Vector3.ProjectOnPlane(
                movementDirection, alignedUp).normalized;
            if (alignedForward.sqrMagnitude < 0.0001f)
            {
                alignedForward = Vector3.ProjectOnPlane(
                    currentRotation * Vector3.forward,
                    alignedUp).normalized;
            }
            if (alignedForward.sqrMagnitude < 0.0001f)
                return currentTerrainTilt * currentRotation;

            return Quaternion.LookRotation(alignedForward, alignedUp);
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
            if (TryGetTravelSurfaceNormal(
                    worldPosition, forward, out Vector3 climbNormal) &&
                IsClimbableNormal(climbNormal))
            {
                // On a wall, sampling ahead finds a much higher point and used
                // to elevator the body upward before the legs could step. The
                // center sample follows the actual surface as forward movement
                // advances into the wall.
                if (!TrySampleTerrainHeight(
                        worldPosition, out float centerSurfaceHeight))
                    return false;
                requiredHeight = centerSurfaceHeight +
                                 authoredGroundClearance +
                                 bodyClearanceSafetyMargin;
                return true;
            }
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

        private bool TryGetTravelSurfaceNormal(
            Vector3 worldPosition,
            Vector3 planarForward,
            out Vector3 normal)
        {
            normal = Vector3.up;
            bool foundCurrent = TrySampleTerrainNormal(
                worldPosition, out Vector3 currentNormal);
            // Once the body is on a steep face, that contact owns orientation.
            // Looking across a narrow trough can otherwise select the normal
            // from the opposite wall.
            if (foundCurrent && IsClimbableNormal(currentNormal))
            {
                normal = currentNormal;
                return true;
            }

            if (enableSteepSurfaceClimbing &&
                planarForward.sqrMagnitude > 0.0001f)
            {
                Vector3 probePosition = worldPosition +
                    planarForward.normalized * climbSurfaceProbeDistance;
                if (TrySampleTerrainNormal(
                        probePosition, out Vector3 probeNormal) &&
                    IsClimbableNormal(probeNormal))
                {
                    bool hasCurrentHeight = TrySampleTerrainHeight(
                        worldPosition, out float currentHeight);
                    bool hasProbeHeight = TrySampleTerrainHeight(
                        probePosition, out float probeHeight);
                    // Anticipate an uphill wall so the legs can reach for it,
                    // but do not rotate down a drop until the body has crossed
                    // the lip. This prevents root-height correction from
                    // cancelling all horizontal progress at a descent.
                    bool probeIsUphill = !hasCurrentHeight || !hasProbeHeight ||
                                         probeHeight > currentHeight + 0.01f;
                    if (probeIsUphill)
                    {
                        normal = probeNormal;
                        return true;
                    }
                }
            }

            if (!foundCurrent)
                return false;
            normal = currentNormal;
            return true;
        }

        private bool IsClimbableNormal(Vector3 normal)
        {
            float angle = Vector3.Angle(normal, Vector3.up);
            return enableSteepSurfaceClimbing &&
                   angle >= climbStartAngle &&
                   angle <= maximumClimbAngle;
        }

        public void ResetForces()
        {
            environmentalVelocity = Vector3.zero;
            currentTerrainTilt = Quaternion.identity;
            previousRootSurfaceNormal = Vector3.up;
            cornerFromNormal = Vector3.up;
            cornerToNormal = Vector3.up;
            hasPreviousRootSurfaceNormal = false;
            rootCornerTransitionActive = false;
        }

        public void StopOnCollision()
        {
            environmentalVelocity = Vector3.zero;
        }

        /// <summary>
        /// Applies a bounded visual weight offset before IK. The body is the
        /// parent of the leg chains, so the legs must solve from this final body
        /// pose or their feet will be lifted away from the ground afterward.
        /// </summary>
        public void ApplyBodyWeightBeforeIk(
            Transform body,
            Vector3 restLocalPosition,
            Quaternion restLocalRotation,
            float totalVirtualSupport,
            float normalizedCadenceImbalance,
            Vector3 rigRightWorld,
            Vector3 rigForwardWorld,
            bool surfaceTransitionActive,
            Vector3 transitionSupportNormal)
        {
            if (body == null)
                return;

            // Finger support still controls surface grip and balance, but it no
            // longer raises the torso. The user-authored low body height is held
            // consistently at rest and throughout the walking cycle.
            float supportedFraction = Mathf.Clamp01(
                totalVirtualSupport * fullSupportCapacityKg / bodyMassKg);
            currentGripStability = supportedFraction *
                (1f - 0.5f * Mathf.Clamp01(
                    Mathf.Abs(normalizedCadenceImbalance)));
            float targetDrop = restingBodyDrop;
            float massResponse = Mathf.Clamp(
                fullSupportCapacityKg / bodyMassKg, 0.25f, 4f);
            currentBodyDrop = Mathf.MoveTowards(
                currentBodyDrop,
                targetDrop,
                bodyHeightResponse * restingBodyDrop * massResponse * Time.deltaTime);

            Vector3 sampledSupportUp = surfaceTransitionActive &&
                                       transitionSupportNormal.sqrMagnitude > 0.0001f
                ? transitionSupportNormal.normalized
                : (currentTerrainTilt * Vector3.up).normalized;
            float supportSlope = Vector3.Angle(
                sampledSupportUp, Vector3.up);
            float climbPostureWeight = Mathf.SmoothStep(
                0f,
                1f,
                Mathf.InverseLerp(
                    climbStartAngle - 15f,
                    climbStartAngle + 15f,
                    supportSlope));
            // Ground posture uses world up while wall posture uses the surface
            // normal. Blending both direction and body drop prevents the torso
            // from switching modes at one arbitrary slope threshold.
            Vector3 supportUp = Vector3.Slerp(
                Vector3.up,
                sampledSupportUp,
                climbPostureWeight).normalized;
            Vector3 safeRight = Vector3.ProjectOnPlane(
                rigRightWorld, supportUp).normalized;
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
            // Fade out the low ground posture as support rotates onto a wall.
            // This keeps the torso outside both surfaces throughout the shared
            // root/leg transition instead of dropping it at exactly 45 degrees.
            float supportOffset = torsoSurfaceClearance -
                                  currentBodyDrop *
                                  (1f - climbPostureWeight);
            Vector3 worldOffset = supportUp * supportOffset -
                                  safeRight * weightedImbalance * maximumBalanceShift;
            float targetTip = weightedImbalance *
                              maximumBalanceTipDegrees;
            Vector3 localOffset = body.parent != null
                ? body.parent.InverseTransformVector(worldOffset)
                : worldOffset;
            body.localPosition = restLocalPosition + localOffset;
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
                rigForwardWorld, supportUp).normalized;
            Vector3 safeBodyRight = Vector3.ProjectOnPlane(
                rigRightWorld, supportUp).normalized;
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
            {
                if (activeTerrainCollider == null)
                    activeTerrainCollider =
                        activeTerrain.GetComponent<TerrainCollider>();
                return;
            }
            activeTerrain = Terrain.activeTerrain;
            if (activeTerrain == null)
                activeTerrain = FindAnyObjectByType<Terrain>();
            if (activeTerrain != null)
                activeTerrainCollider =
                    activeTerrain.GetComponent<TerrainCollider>();
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
            minimumPlantedTraction = Mathf.Clamp01(minimumPlantedTraction);
            plantedTractionExponent = Mathf.Clamp(
                plantedTractionExponent, 0.5f, 3f);
            walkingAcceleration = Mathf.Max(0.01f, walkingAcceleration);
            walkingBrakingDeceleration = Mathf.Max(
                0.01f, walkingBrakingDeceleration);
            surfaceAdhesionForceNewtons = Mathf.Max(
                0f, surfaceAdhesionForceNewtons);
            surfaceLinearDrag = Mathf.Max(0f, surfaceLinearDrag);
            staticGripVelocity = Mathf.Max(0f, staticGripVelocity);
            bodyMassKg = Mathf.Max(0.01f, bodyMassKg);
            fullSupportCapacityKg = Mathf.Max(0.01f, fullSupportCapacityKg);
            steepSlopeSpeedMultiplier = Mathf.Clamp(
                steepSlopeSpeedMultiplier, 0.1f, 1f);
            steepSlopeAngle = Mathf.Clamp(steepSlopeAngle, 1f, 70f);
            climbStartAngle = Mathf.Clamp(climbStartAngle, 20f, 85f);
            maximumClimbAngle = Mathf.Clamp(
                maximumClimbAngle, climbStartAngle, 89.9f);
            climbSurfaceProbeDistance = Mathf.Max(
                0.05f, climbSurfaceProbeDistance);
            climbNormalMatch = Mathf.Clamp(climbNormalMatch, 0.25f, 1f);
            steepSurfaceAdhesionMultiplier = Mathf.Clamp(
                steepSurfaceAdhesionMultiplier, 1f, 2f);
            minimumClimbGrip = Mathf.Clamp01(minimumClimbGrip);
            steepFootContactProbeDistance = Mathf.Max(
                0.5f, steepFootContactProbeDistance);
            bodyClearanceSampleRadius = Mathf.Max(0.05f, bodyClearanceSampleRadius);
            bodyClearanceSafetyMargin = Mathf.Max(0f, bodyClearanceSafetyMargin);
            slopeAlignmentResponse = Mathf.Max(0.01f, slopeAlignmentResponse);
            restingBodyDrop = Mathf.Max(0f, restingBodyDrop);
            torsoSurfaceClearance = Mathf.Max(0f, torsoSurfaceClearance);
            bodyHeightResponse = Mathf.Max(0.01f, bodyHeightResponse);
            maximumBalanceShift = Mathf.Max(0f, maximumBalanceShift);
            balanceTipResponse = Mathf.Max(0.01f, balanceTipResponse);
        }
    }
}
