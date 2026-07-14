using System;
using System.Collections.Generic;
using Dexter.Visualize;
using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// Maps two Dexter finger force vectors to the spider's left and right leg sets,
    /// applies an alternating tetrapod gait, then reconstructs every chain with CCD IK.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(DexterRelayUdpReceiver))]
    public sealed class DexterFrontLegIK : MonoBehaviour
    {
        [Header("Relay")]
        [SerializeField] private DexterRelayUdpReceiver receiver;
        [SerializeField] private DexterFinger leftLegFinger = DexterFinger.Index;
        [SerializeField] private DexterFinger rightLegFinger = DexterFinger.Middle;
        [SerializeField] private bool tareOnEnable = true;
        [SerializeField, Min(0.25f)] private float calibrationDurationSeconds = 5f;

        [Header("Rig")]
        [SerializeField] private string bodyBoneName = "body";
        [SerializeField] private string leftFrontRootName = "front1.L";
        [SerializeField] private string rightFrontRootName = "front1.R";
        [SerializeField] private string leftSecondRowRootName = "mid1.L";
        [SerializeField] private string rightSecondRowRootName = "mid1.R";
        [SerializeField] private string leftThirdRowRootName = "mid1.L.001";
        [SerializeField] private string rightThirdRowRootName = "mid1.R.001";
        [SerializeField] private string leftLastRowRootName = "back1.L";
        [SerializeField] private string rightLastRowRootName = "back1.R";

        [Header("Force Mapping")]
        [Tooltip("Body-local displacement in Unity units per Newton: Fx -> X, Fy -> Z.")]
        [SerializeField] private Vector2 displacementPerNewton = new Vector2(0.90f, 0.90f);
        [Tooltip("Residual force below this magnitude is treated as sensor noise. The threshold is subtracted above it.")]
        [SerializeField, Min(0f)] private float forceDeadZone = 0.10f;
        [SerializeField, Min(0.01f)] private float maximumDisplacement = 0.75f;
        [SerializeField, Min(0.01f)] private float responseSpeed = 14f;
        [SerializeField] private bool invertFx;
        [SerializeField] private bool invertFy;

        [Header("Temporal Smoothing")]
        [Tooltip("Time window in seconds used to smooth fast Dexter input changes. Set to zero to disable.")]
        [SerializeField, Min(0f)] private float temporalSmoothingWindow = 0.25f;

        [Header("Walking Gait")]
        [Tooltip("Lifts the foot that is advancing while the retreating foot stays planted.")]
        [SerializeField] private bool liftAdvancingLeg = true;
        [Tooltip("Maximum height at the midpoint of the foot's ground-to-ground step arc.")]
        [SerializeField, Min(0f)] private float stepLiftHeight = 0.62f;
        [Tooltip("Minimum forward change required to begin a new step. Filters sensor jitter.")]
        [SerializeField, Min(0.001f)] private float stepTriggerDistance = 0.08f;

        [Header("Safe Foot Workspace")]
        [Tooltip("Maximum movement farther away from the body.")]
        [SerializeField, Min(0f)] private float maximumOutwardMovement = 0.55f;
        [Tooltip("Maximum movement toward the body. Kept small to prevent crossing the body.")]
        [SerializeField, Min(0f)] private float maximumInwardMovement = 0.10f;
        [Tooltip("Maximum movement parallel to the side of the body.")]
        [SerializeField, Min(0f)] private float maximumForeAftMovement = 0.65f;
        [Tooltip("Fraction of the gap between adjacent rows that each foot may use. Values below 0.5 keep rows from overlapping.")]
        [SerializeField, Range(0.1f, 0.49f)] private float rowLaneFraction = 0.40f;

        [Header("Natural Leg Shape")]
        [Tooltip("Closest a foot may move toward its hip, relative to its authored resting reach.")]
        [SerializeField, Range(0.5f, 1f)] private float minimumRestReachRatio = 0.82f;
        [Tooltip("Farthest a foot may move from its hip, relative to its authored resting reach.")]
        [SerializeField, Range(1f, 1.25f)] private float maximumRestReachRatio = 1.06f;

        [Header("IK")]
        [SerializeField, Range(1, 32)] private int solverIterations = 14;
        [SerializeField, Min(0.00001f)] private float positionTolerance = 0.001f;
        [SerializeField, Range(1f, 180f)] private float maximumJointStepDegrees = 28f;
        [Tooltip("Initial hip rotation toward the target so motion involves the whole leg.")]
        [SerializeField, Range(0f, 30f)] private float baseJointLeadDegrees = 16f;
        [Tooltip("Maximum hip rotation away from the authored rest pose.")]
        [SerializeField, Range(1f, 120f)] private float maximumBaseDeviationDegrees = 38f;
        [Tooltip("Maximum rotation of each remaining joint away from its authored rest pose.")]
        [SerializeField, Range(1f, 120f)] private float maximumJointDeviationDegrees = 42f;
        [SerializeField] private bool drawTargets = true;

        private sealed class LegChain
        {
            public Transform[] Joints;
            public Transform Effector;
            public Quaternion[] RestLocalRotations;
            public Vector3 RestTarget;
            public Vector3 RestWorldTarget;
            public Vector3 CurrentDisplacement;
            public Vector3 TargetDisplacement;
            public float PreviousTargetForward;
            public float SwingStartForward;
            public float SwingEndForward;
            public float StepLift;
            public bool IsSwinging;
            public float MinimumForeAftMovement;
            public float MaximumForeAftMovement;
        }

        private Transform body;
        private Transform rigSpace;
        private LegChain[] leftLegs;
        private LegChain[] rightLegs;
        private Vector3 rootLocalPosition;
        private Quaternion rootLocalRotation;
        private Vector3 rootLocalScale;
        private Vector3 bodyLocalPosition;
        private Quaternion bodyLocalRotation;
        private Vector3 bodyLocalScale;

        private Vector2 leftBaseline;
        private Vector2 rightBaseline;
        private Vector2 leftBaselineSum;
        private Vector2 rightBaselineSum;
        private int leftBaselineSamples;
        private int rightBaselineSamples;
        private bool isTaring;
        private float tareStartRealtime = -1f;
        private long lastTareSequence = long.MinValue;
        private bool hasBaseline;
        private bool initialized;
        private Vector3 smoothedLeftInput;
        private Vector3 smoothedRightInput;

        public bool IsReceiving => receiver != null && receiver.HasRecentFrame;
        public bool IsTaring => isTaring;

        private void Awake()
        {
            InitializeRig();
        }

        private void OnEnable()
        {
            if (!initialized)
                InitializeRig();
            if (tareOnEnable)
                BeginTare();
        }

        private void LateUpdate()
        {
            if (!initialized)
                return;

            RestoreFixedTransforms();
            RestoreLegPoses(leftLegs);
            RestoreLegPoses(rightLegs);

            UpdateTare();
            UpdateDisplacements();

            SolveLegs(leftLegs);
            SolveLegs(rightLegs);

            RestoreFixedTransforms();
        }

        [ContextMenu("Tare Dexter Forces")]
        public void BeginTare()
        {
            leftBaselineSum = Vector2.zero;
            rightBaselineSum = Vector2.zero;
            leftBaselineSamples = 0;
            rightBaselineSamples = 0;
            isTaring = true;
            tareStartRealtime = -1f;
            lastTareSequence = long.MinValue;
            hasBaseline = false;
            smoothedLeftInput = Vector3.zero;
            smoothedRightInput = Vector3.zero;
        }

        [ContextMenu("Rebuild Dexter Leg Chains")]
        public void InitializeRig()
        {
            initialized = false;
            // These controls are intentionally fixed so older serialized components
            // cannot retain the original Thumb-to-left-leg assignment.
            leftLegFinger = DexterFinger.Index;
            rightLegFinger = DexterFinger.Middle;
            ApplyResponsiveMovementDefaults();

            if (receiver == null)
                receiver = GetComponent<DexterRelayUdpReceiver>();

            Transform searchRoot = FindRigSearchRoot();
            rigSpace = searchRoot;
            body = FindDescendant(searchRoot, bodyBoneName);
            if (receiver == null || body == null)
            {
                Debug.LogError(
                    $"{nameof(DexterFrontLegIK)} requires a relay receiver and a '{bodyBoneName}' bone.",
                    this);
                enabled = false;
                return;
            }

            leftLegs = BuildSideChains(searchRoot, true);
            rightLegs = BuildSideChains(searchRoot, false);
            if (leftLegs == null || rightLegs == null)
            {
                Debug.LogError($"{nameof(DexterFrontLegIK)} could not build all eight leg chains.", this);
                enabled = false;
                return;
            }

            ConfigureRowLanes(leftLegs);
            ConfigureRowLanes(rightLegs);

            CaptureFixedTransforms();
            initialized = true;
            Debug.Log(
                $"{nameof(DexterFrontLegIK)} ready: {leftLegFinger} drives the left tetrapod gait, " +
                $"{rightLegFinger} drives the right tetrapod gait.",
                this);
        }

        private LegChain[] BuildSideChains(Transform searchRoot, bool leftSide)
        {
            string[] rootNames = leftSide
                ? new[] { leftFrontRootName, leftSecondRowRootName, leftThirdRowRootName, leftLastRowRootName }
                : new[] { rightFrontRootName, rightSecondRowRootName, rightThirdRowRootName, rightLastRowRootName };
            var side = new LegChain[rootNames.Length];

            for (int i = 0; i < rootNames.Length; i++)
            {
                Transform root = FindDescendant(searchRoot, rootNames[i]);
                if (root == null)
                {
                    Debug.LogError($"Missing spider leg root '{rootNames[i]}'.", this);
                    return null;
                }

                side[i] = BuildChain(root);
                if (side[i] == null)
                    return null;
            }

            return side;
        }

        private void ConfigureRowLanes(LegChain[] side)
        {
            Vector3 rigForward = GetRigForward();
            for (int row = 0; row < side.Length; row++)
            {
                LegChain leg = side[row];
                float restPosition = Vector3.Dot(leg.RestWorldTarget, rigForward);
                float minimum = -maximumForeAftMovement;
                float maximum = maximumForeAftMovement;

                for (int neighborOffset = -1; neighborOffset <= 1; neighborOffset += 2)
                {
                    int neighborRow = row + neighborOffset;
                    if (neighborRow < 0 || neighborRow >= side.Length)
                        continue;

                    float neighborPosition = Vector3.Dot(side[neighborRow].RestWorldTarget, rigForward);
                    float safeOffset = (neighborPosition - restPosition) * rowLaneFraction;
                    if (safeOffset < 0f)
                        minimum = Mathf.Max(minimum, safeOffset);
                    else
                        maximum = Mathf.Min(maximum, safeOffset);
                }

                leg.MinimumForeAftMovement = minimum;
                leg.MaximumForeAftMovement = maximum;
            }
        }

        private LegChain BuildChain(Transform legRoot)
        {
            var chain = new List<Transform>(6) { legRoot };
            Transform current = legRoot;

            while (current.childCount == 1 && chain.Count < 16)
            {
                current = current.GetChild(0);
                chain.Add(current);
            }

            if (chain.Count < 3)
                return null;

            Transform effector = chain[chain.Count - 1];
            chain.RemoveAt(chain.Count - 1);
            var restRotations = new Quaternion[chain.Count];
            for (int i = 0; i < chain.Count; i++)
                restRotations[i] = chain[i].localRotation;

            return new LegChain
            {
                Joints = chain.ToArray(),
                Effector = effector,
                RestLocalRotations = restRotations,
                RestTarget = body.InverseTransformPoint(effector.position),
                RestWorldTarget = effector.position
            };
        }

        private void UpdateTare()
        {
            DexterForceFrame frame = receiver.LatestFrame;
            if (!isTaring || frame == null || frame.sequence == lastTareSequence)
                return;

            lastTareSequence = frame.sequence;
            bool capturedSample = false;
            if (TryReadForce(leftLegFinger, out Vector2 leftForce))
            {
                leftBaselineSum += leftForce;
                leftBaselineSamples++;
                capturedSample = true;
            }
            if (TryReadForce(rightLegFinger, out Vector2 rightForce))
            {
                rightBaselineSum += rightForce;
                rightBaselineSamples++;
                capturedSample = true;
            }

            if (!capturedSample)
                return;

            if (tareStartRealtime < 0f)
                tareStartRealtime = Time.realtimeSinceStartup;
            if (Time.realtimeSinceStartup - tareStartRealtime < calibrationDurationSeconds)
                return;

            leftBaseline = leftBaselineSamples > 0 ? leftBaselineSum / leftBaselineSamples : Vector2.zero;
            rightBaseline = rightBaselineSamples > 0 ? rightBaselineSum / rightBaselineSamples : Vector2.zero;
            hasBaseline = true;
            isTaring = false;
            Debug.Log(
                $"{nameof(DexterFrontLegIK)} calibration complete. " +
                $"Index baseline=({leftBaseline.x:F3}, {leftBaseline.y:F3}) N, " +
                $"Middle baseline=({rightBaseline.x:F3}, {rightBaseline.y:F3}) N.",
                this);
        }

        private void UpdateDisplacements()
        {
            Vector3 leftTarget = Vector3.zero;
            Vector3 rightTarget = Vector3.zero;

            if (receiver.HasRecentFrame && !IsTaring)
            {
                if (TryReadForce(leftLegFinger, out Vector2 leftForce))
                    leftTarget = MapForce(leftForce - (hasBaseline ? leftBaseline : Vector2.zero));
                if (TryReadForce(rightLegFinger, out Vector2 rightForce))
                    rightTarget = MapForce(rightForce - (hasBaseline ? rightBaseline : Vector2.zero));
            }

            if (temporalSmoothingWindow > 0f)
            {
                float smoothingBlend = 1f - Mathf.Exp(-Time.deltaTime / temporalSmoothingWindow);
                smoothedLeftInput = Vector3.Lerp(smoothedLeftInput, leftTarget, smoothingBlend);
                smoothedRightInput = Vector3.Lerp(smoothedRightInput, rightTarget, smoothingBlend);
            }
            else
            {
                smoothedLeftInput = leftTarget;
                smoothedRightInput = rightTarget;
            }

            float responseBlend = 1f - Mathf.Exp(-responseSpeed * Time.deltaTime);
            UpdateSideDisplacements(leftLegs, smoothedLeftInput, responseBlend);
            UpdateSideDisplacements(rightLegs, smoothedRightInput, responseBlend);
        }

        private void UpdateSideDisplacements(LegChain[] side, Vector3 frontTarget, float blend)
        {
            for (int row = 0; row < side.Length; row++)
            {
                // Alternating tetrapod gait: rows 1 and 3 match the front leg;
                // rows 2 and 4 move fore/aft in the opposite phase.
                Vector3 rowTarget = frontTarget;
                if (row == 1 || row == 3)
                    rowTarget.z = -rowTarget.z;

                LegChain leg = side[row];
                leg.TargetDisplacement = rowTarget;
                UpdateWalkingStep(leg, rowTarget.z);
                leg.CurrentDisplacement = Vector3.Lerp(leg.CurrentDisplacement, rowTarget, blend);
                UpdateStepLift(leg);
            }
        }

        private void UpdateWalkingStep(LegChain leg, float targetForward)
        {
            float targetChange = targetForward - leg.PreviousTargetForward;
            float remainingAdvance = targetForward - leg.CurrentDisplacement.z;

            // A forward-moving foot is the swing leg. A backward-moving foot is
            // the planted leg, which makes opposing Index/Middle motion read as a gait.
            if (liftAdvancingLeg && !leg.IsSwinging &&
                targetChange > 0f && remainingAdvance > stepTriggerDistance)
            {
                leg.IsSwinging = true;
                leg.SwingStartForward = leg.CurrentDisplacement.z;
                leg.SwingEndForward = targetForward;
            }
            else if (leg.IsSwinging)
            {
                if (targetForward < leg.CurrentDisplacement.z)
                {
                    leg.IsSwinging = false;
                }
                else
                {
                    leg.SwingEndForward = Mathf.Max(leg.SwingEndForward, targetForward);
                }
            }

            leg.PreviousTargetForward = targetForward;
        }

        private void UpdateStepLift(LegChain leg)
        {
            if (!liftAdvancingLeg || !leg.IsSwinging)
            {
                leg.StepLift = 0f;
                return;
            }

            float swingDistance = leg.SwingEndForward - leg.SwingStartForward;
            if (swingDistance <= stepTriggerDistance)
            {
                leg.IsSwinging = false;
                leg.StepLift = 0f;
                return;
            }

            float progress = Mathf.Clamp01(
                (leg.CurrentDisplacement.z - leg.SwingStartForward) / swingDistance);
            // Ease both ends of the sine arc so the foot leaves and meets the
            // ground gently instead of changing vertical speed abruptly.
            float easedProgress = progress * progress * (3f - 2f * progress);
            leg.StepLift = Mathf.Sin(easedProgress * Mathf.PI) * stepLiftHeight;
            if (progress >= 0.995f)
            {
                leg.IsSwinging = false;
                leg.StepLift = 0f;
            }
        }

        private Vector3 MapForce(Vector2 force)
        {
            float magnitude = force.magnitude;
            if (magnitude <= forceDeadZone)
                return Vector3.zero;

            // Remove the whole noise radius, not just values inside it. This makes
            // the response continuous: output starts at zero at the dead-zone edge.
            force *= (magnitude - forceDeadZone) / magnitude;

            float fx = force.x * (invertFx ? -1f : 1f);
            float fy = force.y * (invertFy ? -1f : 1f);
            var displacement = new Vector3(
                fx * displacementPerNewton.x,
                0f,
                fy * displacementPerNewton.y);
            return Vector3.ClampMagnitude(displacement, maximumDisplacement);
        }

        private bool TryReadForce(DexterFinger finger, out Vector2 force)
        {
            force = Vector2.zero;
            DexterFingerMeasurement measurement = receiver.GetFinger(finger);
            if (measurement == null || !measurement.has_data || measurement.force == null || measurement.force.Length < 2)
                return false;

            force = new Vector2(measurement.force[0], measurement.force[1]);
            return true;
        }

        private Vector3 GetConstrainedWorldTarget(LegChain leg, Vector3 displacement)
        {
            Vector3 rigRight = Vector3.ProjectOnPlane(rigSpace.right, Vector3.up).normalized;
            if (rigRight.sqrMagnitude < 0.5f)
                rigRight = Vector3.right;

            Vector3 rigForward = GetRigForward();

            Vector3 requested = rigRight * displacement.x + rigForward * displacement.z;
            Vector3 outward = Vector3.ProjectOnPlane(leg.RestWorldTarget - body.position, Vector3.up).normalized;
            if (outward.sqrMagnitude < 0.5f)
                outward = Vector3.Dot(leg.RestWorldTarget - body.position, rigRight) >= 0f ? rigRight : -rigRight;
            float outwardAmount = Mathf.Clamp(
                Vector3.Dot(requested, outward),
                -maximumInwardMovement,
                maximumOutwardMovement);
            float foreAftAmount = Mathf.Clamp(
                Vector3.Dot(requested, rigForward),
                leg.MinimumForeAftMovement,
                leg.MaximumForeAftMovement);

            Vector3 groundTarget = leg.RestWorldTarget + outward * outwardAmount + rigForward * foreAftAmount;
            groundTarget.y = leg.RestWorldTarget.y;

            // Establish a safe planted/landing position first. The vertical arc
            // is applied afterward so the reach limiter cannot squash its height.
            Vector3 target = ConstrainTargetReach(leg, groundTarget);
            target.y += leg.StepLift;
            return target;
        }

        private Vector3 ConstrainTargetReach(LegChain leg, Vector3 target)
        {
            Vector3 hip = leg.Joints[0].position;
            float restingReach = Vector3.Distance(hip, leg.RestWorldTarget);
            Vector3 hipToTarget = target - hip;
            if (restingReach < 0.0001f || hipToTarget.sqrMagnitude < 0.0000001f)
                return leg.RestWorldTarget;

            float minimumReach = restingReach * minimumRestReachRatio;
            float maximumReach = restingReach * maximumRestReachRatio;
            float constrainedReach = Mathf.Clamp(hipToTarget.magnitude, minimumReach, maximumReach);
            return hip + hipToTarget.normalized * constrainedReach;
        }

        private Vector3 GetRigForward()
        {
            Vector3 rigRight = Vector3.ProjectOnPlane(rigSpace.right, Vector3.up).normalized;
            if (rigRight.sqrMagnitude < 0.5f)
                rigRight = Vector3.right;

            Vector3 rigForward = Vector3.Cross(Vector3.up, rigRight).normalized;
            if (Vector3.Dot(rigForward, rigSpace.forward) < 0f)
                rigForward = -rigForward;
            return rigForward;
        }

        private void SolveCcd(LegChain leg, Vector3 worldTarget)
        {
            RotateJointTowardTarget(leg.Joints[0], leg.Effector, worldTarget, baseJointLeadDegrees);
            ClampJointToRest(leg, 0);

            float toleranceSquared = positionTolerance * positionTolerance;
            for (int iteration = 0; iteration < solverIterations; iteration++)
            {
                if ((leg.Effector.position - worldTarget).sqrMagnitude <= toleranceSquared)
                    return;

                for (int jointIndex = leg.Joints.Length - 1; jointIndex >= 0; jointIndex--)
                {
                    RotateJointTowardTarget(
                        leg.Joints[jointIndex],
                        leg.Effector,
                        worldTarget,
                        maximumJointStepDegrees);
                    ClampJointToRest(leg, jointIndex);
                }
            }
        }

        private void ClampJointToRest(LegChain leg, int jointIndex)
        {
            float maximumDeviation = jointIndex == 0
                ? maximumBaseDeviationDegrees
                : maximumJointDeviationDegrees;
            Transform joint = leg.Joints[jointIndex];
            joint.localRotation = Quaternion.RotateTowards(
                leg.RestLocalRotations[jointIndex],
                joint.localRotation,
                maximumDeviation);
        }

        private static void RotateJointTowardTarget(
            Transform joint,
            Transform effector,
            Vector3 worldTarget,
            float maximumDegrees)
        {
            if (maximumDegrees <= 0f)
                return;

            Vector3 toEffector = effector.position - joint.position;
            Vector3 toTarget = worldTarget - joint.position;
            if (toEffector.sqrMagnitude < 0.0000001f || toTarget.sqrMagnitude < 0.0000001f)
                return;

            Quaternion fullDelta = Quaternion.FromToRotation(toEffector, toTarget);
            Quaternion limitedDelta = Quaternion.RotateTowards(
                Quaternion.identity,
                fullDelta,
                maximumDegrees);
            joint.rotation = limitedDelta * joint.rotation;
        }

        private void CaptureFixedTransforms()
        {
            rootLocalPosition = transform.localPosition;
            rootLocalRotation = transform.localRotation;
            rootLocalScale = transform.localScale;
            bodyLocalPosition = body.localPosition;
            bodyLocalRotation = body.localRotation;
            bodyLocalScale = body.localScale;
        }

        private void RestoreFixedTransforms()
        {
            transform.localPosition = rootLocalPosition;
            transform.localRotation = rootLocalRotation;
            transform.localScale = rootLocalScale;
            body.localPosition = bodyLocalPosition;
            body.localRotation = bodyLocalRotation;
            body.localScale = bodyLocalScale;
        }

        private static void RestoreLegPose(LegChain leg)
        {
            for (int i = 0; i < leg.Joints.Length; i++)
                leg.Joints[i].localRotation = leg.RestLocalRotations[i];
        }

        private static void RestoreLegPoses(LegChain[] legs)
        {
            for (int i = 0; i < legs.Length; i++)
                RestoreLegPose(legs[i]);
        }

        private void SolveLegs(LegChain[] legs)
        {
            for (int i = 0; i < legs.Length; i++)
            {
                LegChain leg = legs[i];
                SolveCcd(leg, GetConstrainedWorldTarget(leg, leg.CurrentDisplacement));
            }
        }

        private static Transform FindDescendant(Transform root, string targetName)
        {
            Transform[] descendants = root.GetComponentsInChildren<Transform>(true);
            for (int i = 0; i < descendants.Length; i++)
            {
                if (string.Equals(descendants[i].name, targetName, StringComparison.OrdinalIgnoreCase))
                    return descendants[i];
            }
            return null;
        }

        private Transform FindRigSearchRoot()
        {
            Transform candidate = transform;
            while (candidate.parent != null)
            {
                if (FindDescendant(candidate, leftFrontRootName) != null &&
                    FindDescendant(candidate, rightFrontRootName) != null)
                    return candidate;

                candidate = candidate.parent;
            }

            return candidate;
        }

        private void OnDrawGizmosSelected()
        {
            if (!drawTargets || !initialized || body == null)
                return;

            Gizmos.color = new Color(0.2f, 0.9f, 1f, 0.95f);
            DrawSideTargets(leftLegs);
            DrawSideTargets(rightLegs);
        }

        private void DrawSideTargets(LegChain[] legs)
        {
            for (int i = 0; i < legs.Length; i++)
            {
                LegChain leg = legs[i];
                Gizmos.DrawWireSphere(GetConstrainedWorldTarget(leg, leg.CurrentDisplacement), 0.03f);
            }
        }

        private void OnValidate()
        {
            leftLegFinger = DexterFinger.Index;
            rightLegFinger = DexterFinger.Middle;
            ApplyResponsiveMovementDefaults();
            calibrationDurationSeconds = Mathf.Max(0.25f, calibrationDurationSeconds);
            displacementPerNewton.x = Mathf.Max(0f, displacementPerNewton.x);
            displacementPerNewton.y = Mathf.Max(0f, displacementPerNewton.y);
            forceDeadZone = Mathf.Max(0.10f, forceDeadZone);
            maximumDisplacement = Mathf.Max(0.01f, maximumDisplacement);
            responseSpeed = Mathf.Max(0.01f, responseSpeed);
            temporalSmoothingWindow = Mathf.Max(0f, temporalSmoothingWindow);
            maximumOutwardMovement = Mathf.Max(0f, maximumOutwardMovement);
            maximumInwardMovement = Mathf.Max(0f, maximumInwardMovement);
            maximumForeAftMovement = Mathf.Max(0f, maximumForeAftMovement);
            rowLaneFraction = Mathf.Clamp(rowLaneFraction, 0.1f, 0.49f);
            minimumRestReachRatio = Mathf.Clamp(minimumRestReachRatio, 0.5f, 1f);
            maximumRestReachRatio = Mathf.Clamp(maximumRestReachRatio, 1f, 1.25f);
            stepLiftHeight = Mathf.Max(0f, stepLiftHeight);
            stepTriggerDistance = Mathf.Max(0.001f, stepTriggerDistance);
            baseJointLeadDegrees = Mathf.Clamp(baseJointLeadDegrees, 0f, 30f);
            maximumBaseDeviationDegrees = Mathf.Clamp(maximumBaseDeviationDegrees, 1f, 120f);
            maximumJointDeviationDegrees = Mathf.Clamp(maximumJointDeviationDegrees, 1f, 120f);
            solverIterations = Mathf.Clamp(solverIterations, 1, 32);
            positionTolerance = Mathf.Max(0.00001f, positionTolerance);
        }

        private void ApplyResponsiveMovementDefaults()
        {
            displacementPerNewton.x = Mathf.Max(0.90f, displacementPerNewton.x);
            displacementPerNewton.y = Mathf.Max(0.90f, displacementPerNewton.y);
            maximumDisplacement = Mathf.Max(0.75f, maximumDisplacement);
            maximumOutwardMovement = Mathf.Max(0.55f, maximumOutwardMovement);
            maximumInwardMovement = Mathf.Max(0.10f, maximumInwardMovement);
            maximumForeAftMovement = Mathf.Max(0.65f, maximumForeAftMovement);
            baseJointLeadDegrees = Mathf.Max(16f, baseJointLeadDegrees);
        }
    }
}
