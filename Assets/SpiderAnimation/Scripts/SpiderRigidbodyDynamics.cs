using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// Surface-relative Rigidbody locomotion. Forward/down contact probes,
    /// artificial surface gravity, adhesion, and torque alignment are based on
    /// the architecture described by PhilS94's wall-walking spider project;
    /// this is an original implementation for the Dexter controller.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Rigidbody), typeof(CapsuleCollider))]
    [DefaultExecutionOrder(-50)]
    public sealed class SpiderRigidbodyDynamics : MonoBehaviour
    {
        [Header("Rigidbody Body")]
        [SerializeField, Min(0.05f)] private float bodyMassKg = 2.5f;
        [SerializeField] private Vector3 centerOfMass = new Vector3(0f, 0.16f, 0f);
        [SerializeField, Min(0f)] private float linearDamping = 0.55f;
        [SerializeField, Min(0f)] private float angularDamping = 2.8f;
        [SerializeField, Min(0.1f)] private float maximumBodySpeed = 18f;
        [SerializeField, Min(0.1f)] private float maximumAngularSpeed = 7.5f;
        [Tooltip("Animated IK colliders cannot be part of a dynamic compound Rigidbody without injecting false impulses. Foot contacts still use casts.")]
        [SerializeField] private bool disableAnimatedLegColliders = true;
        [Tooltip("Keeps the physical capsule centered on the visibly lowered torso instead of above it at the rig root.")]
        [SerializeField] private bool followVisibleBodyWithCollider = true;
        [SerializeField, Min(0.05f)] private float physicalBodyRadius = 0.34f;
        [SerializeField, Min(0.1f)] private float physicalBodyHeight = 0.62f;

        [Header("Surface-Relative Gravity and Adhesion")]
        [SerializeField, Min(0f)] private float worldGravityScale = 1f;
        [SerializeField, Min(0f)] private float surfaceAdhesionAcceleration = 6f;
        [SerializeField, Min(0f)] private float clearanceSpringStiffness = 75f;
        [SerializeField, Min(0f)] private float clearanceSpringDamping = 18f;
        [Tooltip("Cancels tangential gravity while planted so the spider sticks to slopes and walls at neutral input.")]
        [SerializeField, Range(0f, 1.25f)] private float staticSurfaceGrip = 1f;
        [SerializeField, Min(0.01f)] private float surfaceProbeDistance = 1.1f;
        [SerializeField, Min(0.01f)] private float surfaceProbeRadius = 0.18f;
        [SerializeField, Min(0.1f)] private float surfaceNormalResponse = 9f;

        [Header("Dexter Foot Propulsion")]
        [SerializeField, Min(0f)] private float velocityDriveGain = 48f;
        [SerializeField, Min(0f)] private float positionDriveGain = 48f;
        [SerializeField, Min(0f)] private float maximumFootDriveForce = 360f;
        [Tooltip("Fraction of sideways position/velocity correction retained while forward propulsion stays aligned with the spider. Lower values reduce drift-induced spins.")]
        [SerializeField, Range(0f, 0.5f)] private float lateralDriveCorrectionRatio = 0.15f;
        [Tooltip("Cancels yaw torque accidentally created when the same traction force is applied through an uneven set of planted feet. Intentional rotation torque remains unaffected.")]
        [SerializeField, Range(0f, 1f)] private float driveYawTorqueCancellation = 1f;
        [SerializeField, Min(0f)] private float maximumDriveYawCancellationTorque = 30f;
        [SerializeField, Range(0f, 1f)] private float minimumPlantedAuthority = 0.12f;
        [SerializeField, Min(0f)] private float footSurfaceFriction = 7f;
        [SerializeField, Min(0.05f)] private float footProbeDistance = 0.7f;
        [SerializeField, Min(0.05f)] private float maximumTargetLead = 0.4f;

        [Header("Airborne Jump Drive")]
        [Tooltip("Airborne targets may lead farther than walking targets so a strong calibrated jump is not reduced to a short hop.")]
        [SerializeField, Min(0.1f)] private float airborneTargetLead = 3.5f;
        [SerializeField, Min(0.1f)] private float maximumAirborneSpeed = 24f;

        [Header("Surface Alignment Torque")]
        [SerializeField, Min(0f)] private float rotationSpringTorque = 72f;
        [SerializeField, Min(0f)] private float rotationDampingTorque = 8f;
        [SerializeField, Min(0f)] private float maximumRotationTorque = 105f;
        [Tooltip("Allows bounded physical lean and angular give while retaining enough control for procedural feet. One is loosest; zero is rigidly aligned.")]
        [SerializeField, Range(0f, 0.8f)] private float ragdollBodyCompliance = 0.35f;

        private readonly RaycastHit[] hitBuffer = new RaycastHit[24];
        private Rigidbody physicsBody;
        private CapsuleCollider bodyCollider;
        private Transform visibleBody;
        private Transform[] feet = System.Array.Empty<Transform>();
        private Vector3 desiredPosition;
        private Quaternion desiredRotation = Quaternion.identity;
        private Vector3 desiredSupportNormal = Vector3.up;
        private Vector3 smoothedSurfaceNormal = Vector3.up;
        private Vector3 previousFixedTarget;
        private float plantedDrive;
        private bool airborne;
        private bool hasTarget;
        private bool hasPreviousFixedTarget;
        private bool animatedCollidersDisabled;
        private float lastExternalBodyCollisionAt = float.NegativeInfinity;
        private Vector3 debugGravityForce;
        private Vector3 debugDriveForce;
        private Vector3 debugLateralCorrectionForce;
        private Vector3 debugFrictionForce;
        private Vector3 debugAdhesionForce;
        private Vector3 debugClearanceForce;
        private Vector3 debugRotationTorque;

        public Rigidbody PhysicsBody => physicsBody;
        public Vector3 DebugGravityForce => debugGravityForce;
        public Vector3 DebugDriveForce => debugDriveForce;
        public Vector3 DebugLateralCorrectionForce =>
            debugLateralCorrectionForce;
        public Vector3 DebugFrictionForce => debugFrictionForce;
        public Vector3 DebugAdhesionForce => debugAdhesionForce;
        public Vector3 DebugClearanceForce => debugClearanceForce;
        public Vector3 DebugRotationTorque => debugRotationTorque;
        public Vector3 SurfaceNormal => smoothedSurfaceNormal;

        private void Awake()
        {
            EnsureBody();
        }

        public void Initialize(
            Transform[] footEffectors,
            Transform visibleBodyTransform)
        {
            EnsureBody();
            feet = footEffectors ?? System.Array.Empty<Transform>();
            visibleBody = visibleBodyTransform;
            UpdatePhysicalBodyCollider();
            animatedCollidersDisabled = false;
            DisableAnimatedColliderResponse();
            desiredPosition = physicsBody.position;
            desiredRotation = physicsBody.rotation;
            previousFixedTarget = desiredPosition;
            smoothedSurfaceNormal = transform.up;
            hasPreviousFixedTarget = false;
        }

        public float ApplyJumpImpulse(
            Vector3 forward,
            Vector3 surfaceNormal,
            float distance,
            float height)
        {
            EnsureBody();
            Vector3 up = surfaceNormal.sqrMagnitude > 0.0001f
                ? surfaceNormal.normalized
                : Vector3.up;
            Vector3 tangentForward = Vector3.ProjectOnPlane(
                forward, up).normalized;
            if (tangentForward.sqrMagnitude < 0.0001f)
                tangentForward = Vector3.ProjectOnPlane(
                    transform.forward, up).normalized;

            float gravity = Mathf.Max(
                0.1f, Physics.gravity.magnitude * worldGravityScale);
            float verticalSpeed = Mathf.Sqrt(
                2f * gravity * Mathf.Max(0.01f, height));
            float flightDuration = Mathf.Max(
                0.2f, 2f * verticalSpeed / gravity);
            float horizontalSpeed = Mathf.Max(0f, distance) / flightDuration;
            Vector3 desiredVelocity =
                tangentForward * horizontalSpeed + up * verticalSpeed;
            Vector3 retainedTangentVelocity = Vector3.ProjectOnPlane(
                physicsBody.linearVelocity, up) * 0.2f;
            Vector3 velocityChange = desiredVelocity -
                (Vector3.Project(physicsBody.linearVelocity, up) +
                 retainedTangentVelocity);
            physicsBody.AddForce(velocityChange, ForceMode.VelocityChange);

            return flightDuration;
        }

        public bool HasSupportContact(Vector3 supportNormal)
        {
            Vector3 normal = supportNormal.sqrMagnitude > 0.0001f
                ? supportNormal.normalized
                : smoothedSurfaceNormal;
            return CountFootContacts(normal) >= 2 ||
                   TryClosestExternalSphereCast(
                       physicsBody.worldCenterOfMass,
                       -normal,
                       physicalBodyRadius * 0.8f,
                       physicalBodyHeight * 0.75f + 0.2f,
                       out _);
        }

        public bool HasBodySupportContact(Vector3 supportNormal)
        {
            if (Time.time - lastExternalBodyCollisionAt <=
                Mathf.Max(0.08f, Time.fixedDeltaTime * 2.5f))
                return true;
            Vector3 normal = supportNormal.sqrMagnitude > 0.0001f
                ? supportNormal.normalized
                : smoothedSurfaceNormal;
            return TryClosestExternalSphereCast(
                physicsBody.worldCenterOfMass,
                -normal,
                physicalBodyRadius * 0.8f,
                physicalBodyHeight * 0.6f + 0.25f,
                out _);
        }

        private void OnCollisionEnter(Collision collision)
        {
            RecordExternalBodyCollision(collision);
        }

        private void OnCollisionStay(Collision collision)
        {
            RecordExternalBodyCollision(collision);
        }

        private void RecordExternalBodyCollision(Collision collision)
        {
            if (collision == null || collision.collider == null ||
                collision.collider.transform.IsChildOf(transform))
                return;
            lastExternalBodyCollisionAt = Time.time;
        }

        public void CompleteJumpLanding(Vector3 supportNormal)
        {
            if (physicsBody == null)
                return;
            Vector3 normal = supportNormal.sqrMagnitude > 0.0001f
                ? supportNormal.normalized
                : smoothedSurfaceNormal;
            float inwardSpeed = Vector3.Dot(
                physicsBody.linearVelocity, normal);
            if (inwardSpeed < 0f)
                physicsBody.linearVelocity -= normal * inwardSpeed;
            Vector3 normalVelocity = Vector3.Project(
                physicsBody.linearVelocity, normal);
            Vector3 tangentVelocity = Vector3.ProjectOnPlane(
                physicsBody.linearVelocity, normal);
            physicsBody.linearVelocity = normalVelocity +
                tangentVelocity * 0.35f;
        }

        public void SetDesiredPose(
            Vector3 position,
            Quaternion rotation,
            Vector3 supportNormal,
            float plantedPush,
            bool isAirborne)
        {
            desiredPosition = position;
            desiredRotation = rotation;
            desiredSupportNormal = supportNormal.sqrMagnitude > 0.0001f
                ? supportNormal.normalized
                : Vector3.up;
            plantedDrive = Mathf.Clamp01(plantedPush);
            airborne = isAirborne;
            hasTarget = true;
        }

        public Vector3 ConstrainDesiredPosition(
            Vector3 position,
            bool isAirborne = false)
        {
            if (physicsBody == null)
                return position;
            float targetLead = isAirborne
                ? airborneTargetLead
                : maximumTargetLead;
            return physicsBody.position + Vector3.ClampMagnitude(
                position - physicsBody.position,
                targetLead);
        }

        private void FixedUpdate()
        {
            if (!hasTarget || physicsBody == null)
                return;

            debugDriveForce = Vector3.zero;
            debugLateralCorrectionForce = Vector3.zero;
            debugFrictionForce = Vector3.zero;
            debugAdhesionForce = Vector3.zero;
            debugClearanceForce = Vector3.zero;
            debugRotationTorque = Vector3.zero;
            DisableAnimatedColliderResponse();
            UpdatePhysicalBodyCollider();

            float normalBlend = 1f - Mathf.Exp(
                -surfaceNormalResponse * Time.fixedDeltaTime);
            Vector3 probedNormal = ProbeSurfaceNormal(desiredSupportNormal);
            smoothedSurfaceNormal = Vector3.Slerp(
                smoothedSurfaceNormal, probedNormal, normalBlend).normalized;

            float dt = Time.fixedDeltaTime;
            Vector3 targetVelocity = hasPreviousFixedTarget
                ? (desiredPosition - previousFixedTarget) / Mathf.Max(0.001f, dt)
                : Vector3.zero;
            previousFixedTarget = desiredPosition;
            hasPreviousFixedTarget = true;
            float activeMaximumSpeed = airborne
                ? maximumAirborneSpeed
                : maximumBodySpeed;
            targetVelocity = Vector3.ClampMagnitude(
                targetVelocity, activeMaximumSpeed);
            float activeTargetLead = airborne
                ? airborneTargetLead
                : maximumTargetLead;
            Vector3 positionError = Vector3.ClampMagnitude(
                desiredPosition - physicsBody.position,
                activeTargetLead);

            // In the air, gravity pulls back toward the takeoff surface so a
            // sticky wall-walking spider follows a real ballistic arc relative
            // to that surface. Grounded motion retains normal world gravity.
            Vector3 gravityAcceleration = airborne
                ? -desiredSupportNormal *
                  (Physics.gravity.magnitude * worldGravityScale)
                : Physics.gravity * worldGravityScale;
            debugGravityForce = gravityAcceleration * bodyMassKg;
            physicsBody.AddForce(debugGravityForce, ForceMode.Force);

            if (airborne)
            {
                // Translation is intentionally impulse-only while airborne.
                // Position chasing here would turn a physical jump back into
                // a kinematic arc and make distance depend on frame timing.
                ApplyRotationTorque();
                return;
            }

            Vector3 normal = smoothedSurfaceNormal;
            float slopeAdhesion = Mathf.SmoothStep(
                0f,
                1f,
                Mathf.InverseLerp(
                    25f, 70f, Vector3.Angle(normal, Vector3.up)));
            Vector3 adhesionForce = -normal *
                (surfaceAdhesionAcceleration * bodyMassKg * slopeAdhesion);
            debugAdhesionForce = adhesionForce;
            physicsBody.AddForce(adhesionForce, ForceMode.Force);

            float normalError = Vector3.Dot(positionError, normal);
            float normalVelocity = Vector3.Dot(
                physicsBody.linearVelocity, normal);
            Vector3 gravityForce = Physics.gravity *
                                   bodyMassKg * worldGravityScale;
            Vector3 slopeHoldForce = -Vector3.ProjectOnPlane(
                gravityForce, normal) * staticSurfaceGrip;
            debugAdhesionForce += slopeHoldForce;
            physicsBody.AddForce(slopeHoldForce, ForceMode.Force);
            float surfaceSlope = Vector3.Angle(normal, Vector3.up);
            Vector3 clearanceDirection = surfaceSlope < 40f
                ? Vector3.up
                : normal;
            normalError = Vector3.Dot(
                positionError, clearanceDirection);
            normalVelocity = Vector3.Dot(
                physicsBody.linearVelocity, clearanceDirection);
            float normalGravityCompensation = Mathf.Max(
                0f, -Vector3.Dot(gravityForce, clearanceDirection));
            Vector3 clearanceForce = clearanceDirection *
                (normalError * clearanceSpringStiffness -
                 normalVelocity * clearanceSpringDamping +
                 normalGravityCompensation);
            debugClearanceForce = clearanceForce;

            Vector3 tangentPositionError = Vector3.ProjectOnPlane(
                positionError, normal);
            Vector3 tangentVelocityError = Vector3.ProjectOnPlane(
                targetVelocity - physicsBody.linearVelocity, normal);
            float authority = Mathf.Lerp(
                minimumPlantedAuthority, 1f, plantedDrive);
            Vector3 surfaceForward = Vector3.ProjectOnPlane(
                desiredRotation * Vector3.forward, normal).normalized;
            if (surfaceForward.sqrMagnitude < 0.0001f)
                surfaceForward = Vector3.ProjectOnPlane(
                    transform.forward, normal).normalized;
            if (surfaceForward.sqrMagnitude < 0.0001f)
                surfaceForward = Vector3.Cross(
                    normal, transform.right).normalized;

            // Propulsion is allowed only along the intended surface-forward
            // direction. Sideways position correction remains available at a
            // deliberately lower authority so it cannot become a second
            // steering input or dominate the force arrow.
            Vector3 forwardPositionError = surfaceForward *
                Vector3.Dot(tangentPositionError, surfaceForward);
            Vector3 forwardVelocityError = surfaceForward *
                Vector3.Dot(tangentVelocityError, surfaceForward);
            Vector3 lateralPositionError =
                tangentPositionError - forwardPositionError;
            Vector3 lateralVelocityError =
                tangentVelocityError - forwardVelocityError;
            Vector3 propulsionForce =
                forwardPositionError * positionDriveGain +
                forwardVelocityError *
                (velocityDriveGain * authority);
            Vector3 lateralCorrectionForce =
                (lateralPositionError * positionDriveGain +
                 lateralVelocityError *
                 (velocityDriveGain * authority)) *
                lateralDriveCorrectionRatio;
            Vector3 driveForce =
                propulsionForce + lateralCorrectionForce;
            float driveScale = driveForce.magnitude >
                               maximumFootDriveForce
                ? maximumFootDriveForce /
                  Mathf.Max(0.0001f, driveForce.magnitude)
                : 1f;
            propulsionForce *= driveScale;
            lateralCorrectionForce *= driveScale;
            driveForce = propulsionForce + lateralCorrectionForce;
            Vector3 frictionForce = -Vector3.ProjectOnPlane(
                physicsBody.linearVelocity, normal) * footSurfaceFriction;
            debugDriveForce = propulsionForce;
            debugLateralCorrectionForce = lateralCorrectionForce;
            debugFrictionForce = frictionForce;

            int contacts = CountFootContacts(normal);
            if (contacts > 0)
            {
                // Propulsion and traction come from the feet. Normal support
                // is applied at the center of mass so an intermittently
                // detected single foot cannot catapult or flip the torso.
                Vector3 contactTorque = ApplyAtContactingFeet(
                    normal, driveForce + frictionForce, contacts);
                Vector3 unintendedYawTorque = Vector3.Project(
                    contactTorque, normal);
                Vector3 yawCancellationTorque = Vector3.ClampMagnitude(
                    -unintendedYawTorque * driveYawTorqueCancellation,
                    maximumDriveYawCancellationTorque);
                physicsBody.AddTorque(
                    yawCancellationTorque, ForceMode.Force);
                physicsBody.AddForce(clearanceForce, ForceMode.Force);
            }
            else
                physicsBody.AddForce(
                    clearanceForce + driveForce * 0.05f,
                    ForceMode.Force);

            ApplyRotationTorque();
            if (physicsBody.linearVelocity.magnitude > maximumBodySpeed)
                physicsBody.linearVelocity = Vector3.ClampMagnitude(
                    physicsBody.linearVelocity, maximumBodySpeed);
            if (physicsBody.angularVelocity.magnitude > maximumAngularSpeed)
                physicsBody.angularVelocity = Vector3.ClampMagnitude(
                    physicsBody.angularVelocity, maximumAngularSpeed);
        }

        private Vector3 ProbeSurfaceNormal(Vector3 fallback)
        {
            Vector3 origin = physicsBody.worldCenterOfMass + fallback * 0.2f;
            if (TryClosestExternalSphereCast(
                    origin, -fallback, surfaceProbeRadius,
                    surfaceProbeDistance, out RaycastHit downHit))
                return downHit.normal.normalized;

            Vector3 forward = Vector3.ProjectOnPlane(
                desiredRotation * Vector3.forward, fallback).normalized;
            if (forward.sqrMagnitude > 0.0001f &&
                TryClosestExternalSphereCast(
                    origin, forward, surfaceProbeRadius,
                    surfaceProbeDistance, out RaycastHit forwardHit))
                return forwardHit.normal.normalized;
            return fallback;
        }

        private int CountFootContacts(Vector3 normal)
        {
            int count = 0;
            for (int i = 0; i < feet.Length; i++)
            {
                if (TryFootContact(feet[i], normal, out _))
                    count++;
            }
            return count;
        }

        // Returns the torque created by the distributed contact forces so the
        // yaw component can be cancelled without suppressing commanded turns.
        private Vector3 ApplyAtContactingFeet(
            Vector3 normal,
            Vector3 totalForce,
            int contactCount)
        {
            // Never route the entire drive through one transient ray hit.
            // Missing contacts reduce authority instead of multiplying torque.
            Vector3 forcePerFoot = totalForce / Mathf.Max(4, contactCount);
            Vector3 appliedTorque = Vector3.zero;
            for (int i = 0; i < feet.Length; i++)
            {
                if (!TryFootContact(feet[i], normal, out RaycastHit hit))
                    continue;
                physicsBody.AddForceAtPosition(
                    forcePerFoot, hit.point, ForceMode.Force);
                appliedTorque += Vector3.Cross(
                    hit.point - physicsBody.worldCenterOfMass,
                    forcePerFoot);
            }
            return appliedTorque;
        }

        private bool TryFootContact(
            Transform foot,
            Vector3 normal,
            out RaycastHit hit)
        {
            hit = default;
            return foot != null && TryClosestExternalSphereCast(
                foot.position + normal * 0.12f,
                -normal,
                0.045f,
                footProbeDistance,
                out hit);
        }

        private bool TryClosestExternalSphereCast(
            Vector3 origin,
            Vector3 direction,
            float radius,
            float distance,
            out RaycastHit closestHit)
        {
            closestHit = default;
            int hits = Physics.SphereCastNonAlloc(
                origin, radius, direction.normalized, hitBuffer, distance,
                Physics.AllLayers, QueryTriggerInteraction.Ignore);
            float closestDistance = float.PositiveInfinity;
            bool found = false;
            for (int i = 0; i < hits; i++)
            {
                Collider collider = hitBuffer[i].collider;
                if (collider == null || collider.transform.IsChildOf(transform) ||
                    hitBuffer[i].distance >= closestDistance)
                    continue;
                closestDistance = hitBuffer[i].distance;
                closestHit = hitBuffer[i];
                found = true;
            }
            return found;
        }

        private void ApplyRotationTorque()
        {
            Quaternion error = desiredRotation *
                               Quaternion.Inverse(physicsBody.rotation);
            error.ToAngleAxis(out float angleDegrees, out Vector3 axis);
            if (angleDegrees > 180f)
                angleDegrees -= 360f;
            if (axis.sqrMagnitude < 0.0001f || float.IsNaN(axis.x))
                return;
            float compliance = Mathf.Clamp(ragdollBodyCompliance, 0f, 0.8f);
            float springScale = airborne
                ? Mathf.Lerp(1f, 0.35f, compliance)
                : Mathf.Lerp(1f, 0.55f, compliance);
            float dampingScale = Mathf.Lerp(1f, 0.65f, compliance);
            Vector3 torque = axis.normalized *
                             (angleDegrees * Mathf.Deg2Rad *
                              rotationSpringTorque * springScale) -
                             physicsBody.angularVelocity *
                             (rotationDampingTorque * dampingScale);
            debugRotationTorque = Vector3.ClampMagnitude(
                torque, maximumRotationTorque);
            physicsBody.AddTorque(
                debugRotationTorque, ForceMode.Force);
        }

        private void EnsureBody()
        {
            if (physicsBody == null)
                physicsBody = GetComponent<Rigidbody>();
            if (bodyCollider == null)
                bodyCollider = GetComponent<CapsuleCollider>();
            physicsBody.mass = Mathf.Max(0.05f, bodyMassKg);
            physicsBody.useGravity = false;
            physicsBody.isKinematic = false;
            physicsBody.interpolation = RigidbodyInterpolation.Interpolate;
            physicsBody.collisionDetectionMode =
                CollisionDetectionMode.ContinuousDynamic;
            physicsBody.linearDamping = Mathf.Max(0f, linearDamping);
            physicsBody.angularDamping = Mathf.Max(0f, angularDamping);
            physicsBody.automaticCenterOfMass = false;
            physicsBody.centerOfMass = centerOfMass;
            physicsBody.maxAngularVelocity = Mathf.Max(
                0.1f, maximumAngularSpeed);
            physicsBody.solverIterations = 12;
            physicsBody.solverVelocityIterations = 8;
            DisableAnimatedColliderResponse();
        }

        private void UpdatePhysicalBodyCollider()
        {
            if (bodyCollider == null)
                bodyCollider = GetComponent<CapsuleCollider>();
            if (bodyCollider == null)
                return;

            if (followVisibleBodyWithCollider && visibleBody != null)
                bodyCollider.center = transform.InverseTransformPoint(
                    visibleBody.position);
            bodyCollider.direction = 1;
            bodyCollider.radius = Mathf.Max(0.05f, physicalBodyRadius);
            bodyCollider.height = Mathf.Max(
                physicalBodyRadius * 2f,
                physicalBodyHeight);
            bodyCollider.isTrigger = false;
        }

        private void DisableAnimatedColliderResponse()
        {
            if (!disableAnimatedLegColliders || animatedCollidersDisabled)
                return;

            Collider[] colliders = GetComponentsInChildren<Collider>(true);
            int animatedColliderCount = 0;
            for (int i = 0; i < colliders.Length; i++)
            {
                Collider collider = colliders[i];
                if (collider == null || collider.transform == transform)
                    continue;
                animatedColliderCount++;
                collider.enabled = false;
            }
            // If no animated colliders exist yet, another component may still
            // build them later in Awake; retry on the first physics frame.
            animatedCollidersDisabled = animatedColliderCount > 0;
        }

        private void OnValidate()
        {
            bodyMassKg = Mathf.Max(0.05f, bodyMassKg);
            maximumBodySpeed = Mathf.Max(0.1f, maximumBodySpeed);
            maximumAngularSpeed = Mathf.Max(0.1f, maximumAngularSpeed);
            physicalBodyRadius = Mathf.Max(0.05f, physicalBodyRadius);
            physicalBodyHeight = Mathf.Max(
                physicalBodyRadius * 2f, physicalBodyHeight);
            surfaceProbeDistance = Mathf.Max(0.01f, surfaceProbeDistance);
            surfaceProbeRadius = Mathf.Max(0.01f, surfaceProbeRadius);
            footProbeDistance = Mathf.Max(0.05f, footProbeDistance);
            maximumTargetLead = Mathf.Max(0.05f, maximumTargetLead);
            lateralDriveCorrectionRatio = Mathf.Clamp(
                lateralDriveCorrectionRatio, 0f, 0.5f);
            driveYawTorqueCancellation = Mathf.Clamp01(
                driveYawTorqueCancellation);
            maximumDriveYawCancellationTorque = Mathf.Max(
                0f, maximumDriveYawCancellationTorque);
            airborneTargetLead = Mathf.Max(0.1f, airborneTargetLead);
            maximumAirborneSpeed = Mathf.Max(0.1f, maximumAirborneSpeed);
            ragdollBodyCompliance = Mathf.Clamp(
                ragdollBodyCompliance, 0f, 0.8f);
        }
    }
}
