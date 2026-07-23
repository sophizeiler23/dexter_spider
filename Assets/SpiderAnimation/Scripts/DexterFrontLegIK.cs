using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using Dexter.Visualize;
using UnityEngine;
using UnityEngine.Serialization;

namespace Dexter.Spider
{
    /// <summary>
    /// Maps two Dexter finger force vectors to the spider's left and right leg sets,
    /// applies an alternating tetrapod gait, then reconstructs every chain with CCD IK.
    /// </summary>
    [DefaultExecutionOrder(-100)]
    [DisallowMultipleComponent]
    [RequireComponent(typeof(DexterRelayUdpReceiver))]
    [RequireComponent(typeof(SpiderTerrainForces))]
    public sealed class DexterFrontLegIK : MonoBehaviour
    {
        [Header("Relay")]
        [SerializeField] private DexterRelayUdpReceiver receiver;
        [SerializeField] private DexterFinger leftLegFinger = DexterFinger.Index;
        [SerializeField] private DexterFinger rightLegFinger = DexterFinger.Middle;
        [SerializeField] private bool tareOnEnable;
        [SerializeField, Min(0.25f)] private float calibrationDurationSeconds = 10f;

        [Header("Calibration")]
        [Tooltip("Master switch for the calibration phase. Turn off to skip the on-screen calibration HUD entirely and use live, uncalibrated input immediately.")]
        [SerializeField] private bool enableCalibrationPhase = true;

        [Header("iPad Active Movement Calibration")]
        [Tooltip("When iPad position frames are detected, pause movement and learn a comfortable walking displacement during the first calibration period. Physical Dexter/editor input is unaffected.")]
        [FormerlySerializedAs("calibrateDexterMovementOnStart")]
        [SerializeField] private bool calibrateIpadMovementOnStart = true;
        [Tooltip("Percentile of the recorded walking displacements used as the normal iPad movement. A middle percentile ignores brief peaks while remaining responsive.")]
        [FormerlySerializedAs("dexterCalibrationReferencePercentile")]
        [SerializeField, Range(0.25f, 0.8f)] private float ipadCalibrationReferencePercentile = 0.45f;
        [Tooltip("Safety floor in post-scaled iPad displacement units. The relay multiplies centimeters by 5, so 5 corresponds to 1 cm.")]
        [FormerlySerializedAs("minimumDexterCalibrationForce")]
        [SerializeField, Min(0.01f)] private float minimumIpadCalibrationDisplacement = 5f;
        [Tooltip("iPad displacement needed for maximum walking and turning speed, as a multiple of the learned normal displacement. Lower values are more sensitive.")]
        [FormerlySerializedAs("dexterMaximumSpeedForceMultiplier")]
        [SerializeField, Range(1.05f, 6f)] private float ipadMaximumSpeedDisplacementMultiplier = 1.2f;

        [Header("Diagnostics")]
        [Tooltip("Writes one CSV row per unique Dexter frame with raw input and solved foot positions.")]
        [SerializeField] private bool recordDiagnosticTrace;
        [SerializeField, Min(1)] private int traceFlushIntervalRows = 30;

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
        [SerializeField] private Vector2 displacementPerNewton = new Vector2(1.25f, 1.25f);
        [Tooltip("Residual force below this magnitude is treated as sensor noise. The threshold is subtracted above it.")]
        [SerializeField, Min(0f)] private float forceDeadZone = 0.03f;
        [SerializeField, Min(0.01f)] private float maximumDisplacement = 0.75f;
        [SerializeField, Min(0.01f)] private float responseSpeed = 14f;
        [SerializeField] private bool invertFx;
        [SerializeField] private bool invertFy;

        [Header("Temporal Smoothing")]
        [Tooltip("Time window in seconds used to smooth fast Dexter input changes. Set to zero to disable.")]
        [SerializeField, Min(0f)] private float temporalSmoothingWindow = 0.06f;

        [Header("Walking Gait")]
        [Tooltip("Maximum height at the midpoint of the foot's ground-to-ground step arc.")]
        [SerializeField, Min(0f)] private float stepLiftHeight = 0.62f;
        [Tooltip("Dexter displacement needed to activate the gait.")]
        [SerializeField, Min(0f)] private float gaitActivationThreshold = 0.01f;
        [SerializeField, Min(0f)] private float minimumStrideLength = 0.08f;
        [SerializeField, Min(0f)] private float maximumStrideLength = 0.34f;
        [SerializeField, Min(0.01f)] private float minimumGaitCyclesPerSecond = 0.25f;
        [SerializeField, Min(0.01f)] private float maximumGaitCyclesPerSecond = 1.50f;
        [Tooltip("Force magnitude in Newtons that produces the maximum gait speed and stride.")]
        [SerializeField, Min(0.01f)] private float forceForMaximumGaitSpeed = 0.60f;
        [Tooltip("Minimum mapped Y displacement required for movement and forward/back direction. X only affects strength when Y is present.")]
        [SerializeField, Min(0f)] private float gaitDirectionDeadZone = 0.005f;
        [Tooltip("Fraction of a cycle spent lifting and placing a foot. The rest is the planted stance phase.")]
        [SerializeField, Range(0.2f, 0.6f)] private float swingPhaseFraction = 0.50f;
        [Tooltip("How quickly walking fades in and out as finger movement starts and stops.")]
        [SerializeField, Min(0.01f)] private float gaitBlendSpeed = 8f;
        [Tooltip("Extra lift emphasis for the swing phase during walking.")]
        [SerializeField, Range(1f, 3f)] private float walkingLiftEmphasis = 1.5f;
        [Tooltip("Extra forward reach emphasis during the swing phase.")]
        [SerializeField, Range(1f, 2f)] private float walkingStrideEmphasis = 1.3f;
        [Tooltip("Emphasizes how strongly a planted foot contributes to body traction without sliding the planted target.")]
        [SerializeField, Range(1f, 3f)] private float walkingPushEmphasis = 1.6f;

        [Header("Alternating Finger Locomotion")]
        [Tooltip("Minimum forward/back force in Newtons for a finger to register. Kept low for small finger forces.")]
        [SerializeField, Min(0f)] private float alternationForceThreshold = 0.005f;
        [Tooltip("Minimum Y-force lead over the other finger before an alternation registers. Prevents noise from rapidly swapping the active finger.")]
        [SerializeField, Min(0f)] private float alternationDominanceMargin = 0.003f;
        [Tooltip("Both fingers must return below this Y force before another press can register.")]
        [SerializeField, Min(0f)] private float alternationReleaseThreshold = 0.003f;
        [Tooltip("Tare noise standard deviations used to derive a separate threshold for each finger.")]
        [SerializeField, Range(2f, 8f)] private float noiseStandardDeviationMultiplier = 4f;
        [Tooltip("Maximum adaptive activation threshold, preserving sensitivity even with a noisy sensor.")]
        [SerializeField, Min(0.01f)] private float maximumAdaptiveNoiseThreshold = 0.03f;
        [Tooltip("Upper forward-speed ceiling. Actual speed is also limited by gait cadence multiplied by stride length so the body cannot outrun planted feet.")]
        [SerializeField, Min(0f)] private float normalForceTravelSpeed = 2.0f;
        [Tooltip("Brief neutral gap allowed between fingers without cancelling an already established alternating walk.")]
        [SerializeField, Min(0.05f)] private float inputDriveReleaseDelay = 0.35f;
        [Tooltip("Shortest accepted time between finger changes. Rejects one-frame dominance noise without making deliberate alternation feel delayed.")]
        [SerializeField, Min(0f)] private float minimumPressInterval = 0.08f;
        [Tooltip("Maximum time between opposite-finger presses for them to count as one walking alternation.")]
        [SerializeField, Min(0.05f)] private float maximumAlternationInterval = 0.6f;
        [Tooltip("How long a confirmed left/right alternation keeps the force-driven gait active. Each new valid alternation refreshes this window.")]
        [SerializeField, Min(0.1f)] private float alternatingDriveSustainTime = 0.9f;
        [Tooltip("Time used to smooth each discrete forward movement.")]
        [SerializeField, Min(0.01f)] private float locomotionSmoothTime = 0.10f;
        [Tooltip("Combined Y force required to request a change of travel direction.")]
        [SerializeField, Min(0f)] private float directionChangeForceThreshold = 0.18f;
        [Tooltip("How long the reverse-direction force must be held before direction changes.")]
        [SerializeField, Min(0f)] private float directionChangeHoldTime = 0.30f;
        [Header("Force-Driven Step Speed")]
        [Tooltip("Global crawl-speed scale applied at every finger displacement while keeping leg cadence matched to actual travel.")]
        [SerializeField, Range(0.5f, 2f)] private float overallMovementSpeedMultiplier = 1.25f;
        [Tooltip("Half-step duration at the normal force point.")]
        [SerializeField, Min(0.05f)] private float referenceHalfStepDuration = 0.50f;
        [Tooltip("Post-scaled iPad Y input at or below minimum speed.")]
        [SerializeField, Min(0.001f)] private float minimumSpeedDisplacement = 5f;
        [Tooltip("Post-scaled iPad Y input that reaches maximum speed.")]
        [SerializeField, Min(0.001f)] private float maximumSpeedDisplacement = 25f;
        [Tooltip("Stride distance at or below an input of 5.")]
        [SerializeField, Min(0f)] private float minimumForceStrideLength = 0.25f;
        [Tooltip("Stride distance at an input of 25. Default is twice the minimum stride.")]
        [SerializeField, Min(0f)] private float maximumForceStrideLength = 0.40f;
        [Tooltip("Walking sine-arc height at or below an input of 5, relative to Step Lift Height.")]
        [SerializeField, Range(0.1f, 1f)] private float minimumForceLiftMultiplier = 0.55f;
        [Tooltip("Walking sine-arc height at an input of 25, relative to Step Lift Height.")]
        [SerializeField, Range(1f, 2f)] private float maximumForceLiftMultiplier = 1.15f;
        [Tooltip("Ground push distance for a valid input at or below 5.")]
        [SerializeField, Min(0f)] private float minimumLegPushDistance = 0.04f;
        [Tooltip("Ground push distance when finger input reaches 25.")]
        [SerializeField, Min(0f)] private float maximumLegPushDistance = 0.18f;
        [Tooltip("How quickly planted legs build and release their push.")]
        [SerializeField, Min(0.01f)] private float legPushResponse = 6f;
        [Tooltip("Slowest multiplier used for a valid, very small finger movement.")]
        [SerializeField, Range(0.1f, 2f)] private float minimumStepSpeedMultiplier = 1.7f;
        [Tooltip("Hard limit on the force-driven leg and body movement multiplier.")]
        [SerializeField, Range(1f, 6f)] private float maximumStepSpeedMultiplier = 4.0f;

        [Header("Two-Finger Turning")]
        [Tooltip("Minimum post-scaled X input on both fingers before an iPad turn registers.")]
        [SerializeField, Min(0f)] private float turnActivationForce = 1f;
        [Tooltip("X must be at least this fraction of Y on both fingers. This prevents a mostly vertical walking gesture from being mistaken for a turn.")]
        [SerializeField, Range(0f, 1f)] private float turnAxisDominanceRatio = 0.65f;
        [Tooltip("Post-scaled iPad X input at or below minimum turn speed.")]
        [SerializeField, Min(0.001f)] private float turnMinimumSpeedDisplacement = 5f;
        [Tooltip("Post-scaled iPad X input that reaches maximum turn speed.")]
        [SerializeField, Min(0.01f)] private float turnForceForMaximumSpeed = 25f;
        [SerializeField, Min(0f)] private float minimumTurnDegreesPerSecond = 10f;
        [SerializeField, Min(0f)] private float maximumTurnDegreesPerSecond = 45f;
        [Tooltip("How quickly rotation and the turning gait blend in and out.")]
        [SerializeField, Min(0.01f)] private float turnResponseSpeed = 4f;
        [Tooltip("Outside-leg stride relative to the normal stride.")]
        [SerializeField, Min(0f)] private float outsideTurnStrideMultiplier = 1.15f;
        [Tooltip("Inside-leg counter-stride relative to the normal stride.")]
        [SerializeField, Min(0f)] private float insideTurnStrideMultiplier = 0.55f;
        [Tooltip("Time for the two mirrored leg groups to settle back into their natural stance after turning.")]
        [SerializeField, Min(0.1f)] private float turnRecoveryDuration = 0.80f;

        [Header("Obstacle Collision")]
        [SerializeField] private LayerMask obstacleLayers = ~0;
        [SerializeField, Min(0.05f)] private float bodyCollisionRadius = 0.32f;
        [SerializeField, Min(0f)] private float bodyCollisionHeight = 0.42f;
        [SerializeField, Min(0f)] private float obstacleSkin = 0.05f;

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
        [Tooltip("Maximum horizontal distance a planted foot may trail its natural stance before it lifts and replants.")]
        [SerializeField, Min(0.05f)] private float maximumPlantedFootLag = 0.58f;
        [Tooltip("Duration of an automatic anti-drag recovery step.")]
        [SerializeField, Min(0.05f)] private float lagRecoveryDuration = 0.24f;

        [Header("Cadence Balance Posture")]
        [Tooltip("Maximum outward foot reach on the slower side during a cadence imbalance.")]
        [SerializeField, Min(0f)] private float balanceLegExtension = 0.12f;
        [Tooltip("Maximum inward foot draw on the faster side during a cadence imbalance.")]
        [SerializeField, Min(0f)] private float balanceLegBend = 0.08f;
        [Tooltip("Maximum world-space lift when one finger directly raises its side.")]
        [SerializeField, Min(0f)] private float fingerMimicLiftHeight = 0.4f;
        [Tooltip("How quickly the side follows its finger up and down.")]
        [SerializeField, Min(0.01f)] private float fingerMimicResponse = 1.5f;
        [Tooltip("Extra direct-finger lift applied to the front leg of each side.")]
        [SerializeField, Range(1f, 3f)] private float frontLegMimicLiftMultiplier = 1.6f;
        [Tooltip("How strongly lifting one side shifts the body's balance toward that side.")]
        [SerializeField, Range(0f, 2f)] private float fingerLiftBalanceInfluence = 0.8f;
        [Tooltip("How much a lifted side loses its effective ground support.")]
        [SerializeField, Range(0f, 1f)] private float fingerLiftSupportLoss = 0.75f;
        [Tooltip("Per-finger press cadence needed to provide full virtual string support.")]
        [SerializeField, Min(0.1f)] private float cadenceForFullSupport = 1.5f;
        [SerializeField, Min(0.01f)] private float balancePostureResponse = 3f;
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
            public Vector3 RestRigLocalTarget;
            public Vector3 WorldTarget;
            public Vector3 SwingStartWorldTarget;
            public Vector3 SwingEndWorldTarget;
            public Vector3 SwingSurfaceNormal;
            public float GroundClearance;
            public bool WasSwinging;
            public bool IsRecoveringLag;
            public float LagRecoveryProgress;
            public Vector3 LagRecoveryStart;
            public Vector3 CurrentDisplacement;
            public Vector3 TargetDisplacement;
            public float StepLift;
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
        private Vector2 leftBaselineSquareSum;
        private Vector2 rightBaselineSquareSum;
        private int leftBaselineSamples;
        private int rightBaselineSamples;
        private bool isTaring;
        private float tareStartRealtime = -1f;
        private long lastTareSequence = long.MinValue;
        private bool hasBaseline;
        private bool initialized;
        private Vector2 smoothedLeftForce;
        private Vector2 smoothedRightForce;
        private float leftGaitPhase;
        private float rightGaitPhase;
        private float leftGaitWeight;
        private float rightGaitWeight;
        private int lastActiveFinger;
        private int pressedFinger;
        private float lastFingerPressTime = -1f;
        private float lastLeftFingerPressTime = -1f;
        private float lastRightFingerPressTime = -1f;
        private float leftFingerCadence;
        private float rightFingerCadence;
        private float currentCadenceImbalance;
        private float targetForwardDistance;
        private float currentForwardDistance;
        private bool alternatingDriveAuthorized;
        private bool gaitDriveActive;
        private float lastCompletedAlternationTime = -1f;
        private float lastLiveFingerInputTime = -1f;
        private float forwardSpeed;
        private int travelDirection = 1;
        private int pendingTravelDirection;
        private float pendingDirectionStartTime = -1f;
        private float targetLeftGaitPhase;
        private float targetRightGaitPhase;
        private float gaitPhaseSpeed = 1.43f;
        private float turnInput;
        private float turnGaitPhase;
        private float turnGaitWeight;
        private float currentTurnSpeed;
        private float currentYawDegrees;
        private float lastAppliedForwardDistance;
        private Vector3 locomotionWorldPosition;
        private bool hasLocomotionWorldPosition;
        private Vector3 climbSurfaceAnchor;
        private bool hasClimbSurfaceAnchor;
        private bool wasTurnGestureActive;
        private bool isRecoveringFromTurn;
        private float turnRecoveryElapsed;
        private SpiderTerrainForces terrainForces;
        private SpiderEnvironmentForces environmentForces;
        private float activeForceSpeedMultiplier = 1f;
        private float activeForceStrideLength = 0.17f;
        private float activeForceLiftMultiplier = 0.55f;
        private bool leftSideWaveInPlace;
        private bool rightSideWaveInPlace;
        private float leftFingerMimicLift;
        private float rightFingerMimicLift;
        private float currentLeftLegPushDistance;
        private float currentRightLegPushDistance;
        private float leftLegPushDistance;
        private float rightLegPushDistance;
        private float leftAdaptiveActivationThreshold;
        private float rightAdaptiveActivationThreshold;
        private float leftAdaptiveReleaseThreshold;
        private float rightAdaptiveReleaseThreshold;
        private Terrain activeTerrain;
        private readonly RaycastHit[] groundHitBuffer = new RaycastHit[32];
        private float terrainRootClearance;
        private bool hasTerrainRootClearance;
        private StreamWriter traceWriter;
        private long lastTracedSequence = long.MinValue;
        private int unflushedTraceRows;
        private string traceFilePath;
        private readonly List<float> ipadMovementCalibrationSamples =
            new List<float>(1024);
        private bool waitingForIpadMovementCalibration;
        private bool isIpadMovementCalibrating;
        private bool ipadMovementCalibrationComplete;
        private float ipadMovementCalibrationStart = -1f;
        private float ipadCalibrationCompleteMessageUntil = -1f;
        private float calibratedIpadReferenceDisplacement = 5f;
        private long lastIpadMovementCalibrationSequence = long.MinValue;

        public bool IsReceiving => receiver != null && receiver.HasRecentFrame;
        public bool IsTaring => isTaring;
        public bool IsIpadMovementCalibrating =>
            isIpadMovementCalibrating;
        /// <summary>Master calibration-phase switch, independent of the iPad-specific movement calibration below.</summary>
        public bool IsCalibrationPhaseEnabled => enableCalibrationPhase;
        private bool IsCalibrationPhaseActive =>
            enableCalibrationPhase && calibrateIpadMovementOnStart;
        public bool ShouldShowCalibrationHud =>
            IsCalibrationPhaseActive && IsCalibrationHudVisible();
        public float ForwardSpeed => forwardSpeed;
        public string TraceFilePath => traceFilePath;

        public DexterFinger LeftLegFinger => leftLegFinger;
        public DexterFinger RightLegFinger => rightLegFinger;
        /// <summary>
        /// The left-side finger's force after baseline subtraction and temporal smoothing —
        /// this is the actual signal <see cref="UpdateTurning"/>/<see cref="UpdateAlternatingLocomotion"/>
        /// act on, as opposed to the raw value coming straight off the relay.
        /// </summary>
        public Vector2 ProcessedLeftForce => smoothedLeftForce;
        /// <summary>Right-side counterpart of <see cref="ProcessedLeftForce"/>.</summary>
        public Vector2 ProcessedRightForce => smoothedRightForce;

        private void Awake()
        {
            InitializeRig();
        }

        private void OnEnable()
        {
            if (!initialized || body == null ||
                leftLegs == null || rightLegs == null)
                InitializeRig();
            PrepareIpadMovementCalibration();
            if (tareOnEnable && !IsCalibrationPhaseActive)
                BeginTare();
            else
                UseUntaredRelayDefaults();
            BeginDiagnosticTrace();
        }

        private void PrepareIpadMovementCalibration()
        {
            waitingForIpadMovementCalibration =
                IsCalibrationPhaseActive;
            isIpadMovementCalibrating = false;
            ipadMovementCalibrationComplete = false;
            ipadMovementCalibrationStart = -1f;
            ipadCalibrationCompleteMessageUntil = -1f;
            lastIpadMovementCalibrationSequence = long.MinValue;
            ipadMovementCalibrationSamples.Clear();
            calibratedIpadReferenceDisplacement = Mathf.Max(
                minimumIpadCalibrationDisplacement,
                minimumSpeedDisplacement);
        }

        private void UseUntaredRelayDefaults()
        {
            isTaring = false;
            hasBaseline = false;
            alternatingDriveAuthorized = false;
            gaitDriveActive = false;
            lastCompletedAlternationTime = -1f;
            lastLiveFingerInputTime = -1f;
            leftBaseline = Vector2.zero;
            rightBaseline = Vector2.zero;
            leftAdaptiveActivationThreshold = alternationForceThreshold;
            rightAdaptiveActivationThreshold = alternationForceThreshold;
            leftAdaptiveReleaseThreshold = alternationReleaseThreshold;
            rightAdaptiveReleaseThreshold = alternationReleaseThreshold;
        }

        private void OnDisable()
        {
            EndDiagnosticTrace();
            RestoreLegPoses(leftLegs);
            RestoreLegPoses(rightLegs);
            if (body != null)
            {
                body.localPosition = bodyLocalPosition;
                body.localRotation = bodyLocalRotation;
            }
        }

        private void OnApplicationQuit()
        {
            // Ensure the final buffered row is written when Unity exits
            // directly instead of disabling the scene object first.
            EndDiagnosticTrace();
        }

        private void LateUpdate()
        {
            if (!initialized || body == null ||
                leftLegs == null || rightLegs == null)
                return;

            RestoreFixedTransforms();
            RestoreLegPoses(leftLegs);
            RestoreLegPoses(rightLegs);

            UpdateTare();
            UpdateIpadMovementCalibration();
            UpdateDisplacements();

            float leftVirtualSupport = Mathf.Clamp01(
                leftFingerCadence / cadenceForFullSupport);
            float rightVirtualSupport = Mathf.Clamp01(
                rightFingerCadence / cadenceForFullSupport);
            float leftLiftStrength = fingerMimicLiftHeight > 0.0001f
                ? Mathf.Clamp01(leftFingerMimicLift / fingerMimicLiftHeight)
                : 0f;
            float rightLiftStrength = fingerMimicLiftHeight > 0.0001f
                ? Mathf.Clamp01(rightFingerMimicLift / fingerMimicLiftHeight)
                : 0f;
            float effectiveLeftSupport = leftVirtualSupport *
                (1f - leftLiftStrength * fingerLiftSupportLoss);
            float effectiveRightSupport = rightVirtualSupport *
                (1f - rightLiftStrength * fingerLiftSupportLoss);
            float totalVirtualSupport =
                (effectiveLeftSupport + effectiveRightSupport) * 0.5f;
            float targetCadenceImbalance =
                effectiveLeftSupport - effectiveRightSupport +
                (rightLiftStrength - leftLiftStrength) *
                fingerLiftBalanceInfluence;
            currentCadenceImbalance = Mathf.MoveTowards(
                currentCadenceImbalance,
                targetCadenceImbalance,
                balancePostureResponse * Time.deltaTime);

            // Set the weighted body pose before solving the legs. The body bone
            // is also the parent of the leg chains, so moving it after IK would
            // lift every already-solved foot away from the ground.
            terrainForces?.ApplyBodyWeightBeforeIk(
                body,
                bodyLocalPosition,
                bodyLocalRotation,
                totalVirtualSupport,
                currentCadenceImbalance,
                rigSpace.right,
                rigSpace.forward);

            SolveLegsWithBalancePosture(
                leftLegs, true, currentCadenceImbalance, leftFingerMimicLift,
                currentLeftLegPushDistance);
            SolveLegsWithBalancePosture(
                rightLegs, false, currentCadenceImbalance, rightFingerMimicLift,
                currentRightLegPushDistance);

            WriteDiagnosticTraceRow();
        }

        private bool IsCalibrationHudVisible()
        {
            bool waitingForDevice = waitingForIpadMovementCalibration &&
                (receiver == null || !receiver.HasRecentFrame ||
                 !IsRelayPositionFrame(receiver.LatestFrame) ||
                 !HasActiveIpadCalibrationFinger());
            bool showComplete = ipadMovementCalibrationComplete &&
                Time.realtimeSinceStartup <=
                ipadCalibrationCompleteMessageUntil;
            return waitingForDevice || isIpadMovementCalibrating ||
                showComplete;
        }

        private void OnGUI()
        {
            if (!ShouldShowCalibrationHud)
                return;

            bool waitingForDevice = waitingForIpadMovementCalibration &&
                (receiver == null || !receiver.HasRecentFrame ||
                 !IsRelayPositionFrame(receiver.LatestFrame) ||
                 !HasActiveIpadCalibrationFinger());
            bool showComplete = ipadMovementCalibrationComplete &&
                Time.realtimeSinceStartup <=
                ipadCalibrationCompleteMessageUntil;

            Rect panel = DexterSpiderHudLayout.CalibrationPanel;
            DrawRect(panel, new Color(0.08f, 0.09f, 0.12f, 0.92f));
            DrawOutline(panel, new Color(0.25f, 0.29f, 0.37f, 1f), 1f);

            var titleStyle = new GUIStyle(GUI.skin.label)
            {
                alignment = TextAnchor.MiddleCenter,
                fontSize = 14,
                fontStyle = FontStyle.Bold,
                normal = { textColor = Color.white }
            };
            var messageStyle = new GUIStyle(GUI.skin.label)
            {
                alignment = TextAnchor.MiddleCenter,
                fontSize = 11,
                wordWrap = true,
                normal = { textColor = new Color(0.88f, 0.9f, 0.94f, 1f) }
            };
            var countdownStyle = new GUIStyle(GUI.skin.label)
            {
                alignment = TextAnchor.MiddleCenter,
                fontSize = 28,
                fontStyle = FontStyle.Bold,
                normal = { textColor = new Color32(255, 214, 102, 255) }
            };

            if (waitingForDevice)
            {
                GUI.Label(
                    new Rect(panel.x + 12f, panel.y + 8f,
                        panel.width - 24f, 20f),
                    "WAITING FOR RELAY INPUT",
                    titleStyle);
                GUI.Label(
                    new Rect(panel.x + 16f, panel.y + 30f,
                        panel.width - 32f, 52f),
                    "Start the relay and place a finger on index or middle. Calibration begins on first active touch and counts down from 10.",
                    messageStyle);
            }
            else if (isIpadMovementCalibrating)
            {
                float elapsed = Mathf.Max(
                    0f,
                    Time.realtimeSinceStartup -
                    ipadMovementCalibrationStart);
                float remaining = Mathf.Max(
                    0f, calibrationDurationSeconds - elapsed);
                int secondsRemaining = Mathf.CeilToInt(remaining);
                GUI.Label(
                    new Rect(panel.x + 12f, panel.y + 6f,
                        panel.width - 24f, 18f),
                    "CALIBRATING",
                    titleStyle);
                GUI.Label(
                    new Rect(panel.x + panel.width * 0.5f - 28f,
                        panel.y + 24f, 56f, 34f),
                    secondsRemaining.ToString(),
                    countdownStyle);
                GUI.Label(
                    new Rect(panel.x + 16f, panel.y + 58f,
                        panel.width - 32f, 28f),
                    "Alternate index and middle through your normal walking displacement range.",
                    messageStyle);
            }
            else
            {
                GUI.Label(
                    new Rect(panel.x + 12f, panel.y + 10f,
                        panel.width - 24f, 20f),
                    "CALIBRATION COMPLETE",
                    titleStyle);
                GUI.Label(
                    new Rect(panel.x + 16f, panel.y + 34f,
                        panel.width - 32f, 44f),
                    $"Normal walking displacement: {calibratedIpadReferenceDisplacement:0.00}",
                    messageStyle);
            }
        }

        private static void DrawRect(Rect rect, Color color)
        {
            Color previous = GUI.color;
            GUI.color = color;
            GUI.DrawTexture(rect, Texture2D.whiteTexture);
            GUI.color = previous;
        }

        private static void DrawOutline(Rect rect, Color color, float width)
        {
            DrawRect(new Rect(rect.x, rect.y, rect.width, width), color);
            DrawRect(new Rect(rect.x, rect.yMax - width, rect.width, width), color);
            DrawRect(new Rect(rect.x, rect.y, width, rect.height), color);
            DrawRect(new Rect(rect.xMax - width, rect.y, width, rect.height), color);
        }

        private void BeginDiagnosticTrace()
        {
            EndDiagnosticTrace();
            if (!recordDiagnosticTrace)
                return;

            try
            {
                string logDirectory = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Logs"));
                Directory.CreateDirectory(logDirectory);
                traceFilePath = Path.Combine(
                    logDirectory,
                    $"DexterSpiderTrace_{DateTime.Now:yyyyMMdd_HHmmss_fff}.csv");
                var stream = new FileStream(
                    traceFilePath,
                    FileMode.Create,
                    FileAccess.Write,
                    FileShare.ReadWrite);
                traceWriter = new StreamWriter(stream, new UTF8Encoding(false));
                WriteDiagnosticTraceHeader();
                traceWriter.Flush();
                lastTracedSequence = long.MinValue;
                unflushedTraceRows = 0;
                Debug.Log($"Dexter spider diagnostic trace: {traceFilePath}", this);
            }
            catch (Exception exception)
            {
                Debug.LogError($"Could not start Dexter diagnostic trace: {exception.Message}", this);
                EndDiagnosticTrace();
            }
        }

        private void EndDiagnosticTrace()
        {
            if (traceWriter == null)
                return;

            traceWriter.Flush();
            traceWriter.Dispose();
            traceWriter = null;
        }

        private void WriteDiagnosticTraceHeader()
        {
            var row = new StringBuilder(2048);
            row.Append("unity_realtime_s,unity_time_s,unity_frame,sequence,frame_age_s,has_recent,is_taring,");
            row.Append("thumb_raw,index_raw,middle_raw,ring_raw,pinky_raw,");
            row.Append("thumb_fx,thumb_fy,index_fx,index_fy,middle_fx,middle_fy,ring_fx,ring_fy,pinky_fx,pinky_fy,");
            row.Append("index_baseline_x,index_baseline_y,middle_baseline_x,middle_baseline_y,");
            row.Append("index_activation_threshold,middle_activation_threshold,index_release_threshold,middle_release_threshold,");
            row.Append("pressed_finger,last_active_finger,travel_direction,forward_speed,target_forward_distance,current_forward_distance,");
            row.Append("turn_input,turn_speed_deg_s,yaw_degrees,");
            row.Append("left_gait_phase,left_target_gait_phase,right_gait_phase,right_target_gait_phase,");
            row.Append("spider_x,spider_y,spider_z,body_x,body_y,body_z,body_pitch,body_roll,body_yaw");
            AppendFootHeader(row, "left", leftLegs != null ? leftLegs.Length : 4);
            AppendFootHeader(row, "right", rightLegs != null ? rightLegs.Length : 4);
            traceWriter.WriteLine(row.ToString());
        }

        private static void AppendFootHeader(StringBuilder row, string side, int count)
        {
            for (int i = 0; i < count; i++)
            {
                int number = i + 1;
                row.Append($",{side}_foot_{number}_x,{side}_foot_{number}_y,{side}_foot_{number}_z");
                row.Append($",{side}_target_{number}_x,{side}_target_{number}_y,{side}_target_{number}_z");
                row.Append($",{side}_ground_{number}_y,{side}_ground_{number}_clearance");
            }
        }

        private void WriteDiagnosticTraceRow()
        {
            DexterForceFrame frame = receiver != null ? receiver.LatestFrame : null;
            if (traceWriter == null)
                return;

            long sequence = frame != null ? frame.sequence : Time.frameCount;
            if (sequence == lastTracedSequence)
                return;
            lastTracedSequence = sequence;
            var row = new StringBuilder(4096);
            row.Append(Time.realtimeSinceStartupAsDouble.ToString("R", CultureInfo.InvariantCulture));
            AppendNumber(row, Time.timeAsDouble);
            AppendInteger(row, Time.frameCount);
            AppendInteger(row, sequence);
            AppendNumber(row, receiver != null ? receiver.LastFrameAge : double.NaN);
            AppendBoolean(row, receiver != null && receiver.HasRecentFrame);
            AppendBoolean(row, isTaring);

            DexterFingerMeasurement thumb = receiver != null ? receiver.GetFinger(DexterFinger.Thumb) : null;
            DexterFingerMeasurement index = receiver != null ? receiver.GetFinger(DexterFinger.Index) : null;
            DexterFingerMeasurement middle = receiver != null ? receiver.GetFinger(DexterFinger.Middle) : null;
            DexterFingerMeasurement ring = receiver != null ? receiver.GetFinger(DexterFinger.Ring) : null;
            DexterFingerMeasurement pinky = receiver != null ? receiver.GetFinger(DexterFinger.Pinky) : null;
            AppendIntegerArray(row, thumb?.raw);
            AppendIntegerArray(row, index?.raw);
            AppendIntegerArray(row, middle?.raw);
            AppendIntegerArray(row, ring?.raw);
            AppendIntegerArray(row, pinky?.raw);
            AppendForce(row, thumb);
            AppendForce(row, index);
            AppendForce(row, middle);
            AppendForce(row, ring);
            AppendForce(row, pinky);

            AppendVector2(row, leftBaseline);
            AppendVector2(row, rightBaseline);
            AppendNumber(row, leftAdaptiveActivationThreshold);
            AppendNumber(row, rightAdaptiveActivationThreshold);
            AppendNumber(row, leftAdaptiveReleaseThreshold);
            AppendNumber(row, rightAdaptiveReleaseThreshold);
            AppendInteger(row, pressedFinger);
            AppendInteger(row, lastActiveFinger);
            AppendInteger(row, travelDirection);
            AppendNumber(row, forwardSpeed);
            AppendNumber(row, targetForwardDistance);
            AppendNumber(row, currentForwardDistance);
            AppendNumber(row, turnInput);
            AppendNumber(row, currentTurnSpeed);
            AppendNumber(row, currentYawDegrees);
            AppendNumber(row, leftGaitPhase);
            AppendNumber(row, targetLeftGaitPhase);
            AppendNumber(row, rightGaitPhase);
            AppendNumber(row, targetRightGaitPhase);
            AppendVector3(row, transform.position);
            AppendVector3(row, body != null ? body.position : Vector3.zero);
            Vector3 bodyAngles = body != null ? body.eulerAngles : Vector3.zero;
            AppendNumber(row, bodyAngles.x);
            AppendNumber(row, bodyAngles.z);
            AppendNumber(row, bodyAngles.y);
            AppendFeet(row, leftLegs);
            AppendFeet(row, rightLegs);

            traceWriter.WriteLine(row.ToString());
            unflushedTraceRows++;
            if (unflushedTraceRows >= Mathf.Max(1, traceFlushIntervalRows))
            {
                traceWriter.Flush();
                unflushedTraceRows = 0;
            }
        }

        private void AppendFeet(StringBuilder row, LegChain[] legs)
        {
            if (legs == null)
                return;
            for (int i = 0; i < legs.Length; i++)
            {
                AppendVector3(row, legs[i].Effector.position);
                AppendVector3(row, legs[i].WorldTarget);
                if (TrySampleTerrainHeight(legs[i].Effector.position, out float groundHeight))
                {
                    AppendNumber(row, groundHeight);
                    AppendNumber(row, legs[i].Effector.position.y - groundHeight);
                }
                else
                {
                    AppendNumber(row, double.NaN);
                    AppendNumber(row, double.NaN);
                }
            }
        }

        private static void AppendForce(StringBuilder row, DexterFingerMeasurement measurement)
        {
            float x = measurement?.force != null && measurement.force.Length > 0 ? measurement.force[0] : float.NaN;
            float y = measurement?.force != null && measurement.force.Length > 1 ? measurement.force[1] : float.NaN;
            AppendNumber(row, x);
            AppendNumber(row, y);
        }

        private static void AppendIntegerArray(StringBuilder row, int[] values)
        {
            row.Append(',').Append('"');
            if (values != null)
            {
                for (int i = 0; i < values.Length; i++)
                {
                    if (i > 0)
                        row.Append(';');
                    row.Append(values[i].ToString(CultureInfo.InvariantCulture));
                }
            }
            row.Append('"');
        }

        private static void AppendVector2(StringBuilder row, Vector2 value)
        {
            AppendNumber(row, value.x);
            AppendNumber(row, value.y);
        }

        private static void AppendVector3(StringBuilder row, Vector3 value)
        {
            AppendNumber(row, value.x);
            AppendNumber(row, value.y);
            AppendNumber(row, value.z);
        }

        private static void AppendNumber(StringBuilder row, double value)
        {
            row.Append(',').Append(value.ToString("R", CultureInfo.InvariantCulture));
        }

        private static void AppendInteger(StringBuilder row, long value)
        {
            row.Append(',').Append(value.ToString(CultureInfo.InvariantCulture));
        }

        private static void AppendBoolean(StringBuilder row, bool value)
        {
            row.Append(',').Append(value ? '1' : '0');
        }

        [ContextMenu("Tare Dexter Forces")]
        public void BeginTare()
        {
            leftBaselineSum = Vector2.zero;
            rightBaselineSum = Vector2.zero;
            leftBaselineSamples = 0;
            rightBaselineSamples = 0;
            leftBaselineSquareSum = Vector2.zero;
            rightBaselineSquareSum = Vector2.zero;
            isTaring = true;
            tareStartRealtime = -1f;
            lastTareSequence = long.MinValue;
            hasBaseline = false;
            smoothedLeftForce = Vector2.zero;
            smoothedRightForce = Vector2.zero;
            leftGaitPhase = 0f;
            rightGaitPhase = 0f;
            leftGaitWeight = 0f;
            rightGaitWeight = 0f;
            lastActiveFinger = 0;
            pressedFinger = 0;
            lastFingerPressTime = -1f;
            lastLeftFingerPressTime = -1f;
            lastRightFingerPressTime = -1f;
            leftFingerCadence = 0f;
            rightFingerCadence = 0f;
            currentCadenceImbalance = 0f;
            targetForwardDistance = 0f;
            currentForwardDistance = 0f;
            alternatingDriveAuthorized = false;
            gaitDriveActive = false;
            lastCompletedAlternationTime = -1f;
            lastLiveFingerInputTime = -1f;
            forwardSpeed = 0f;
            turnInput = 0f;
            turnGaitPhase = 0f;
            turnGaitWeight = 0f;
            currentTurnSpeed = 0f;
            currentYawDegrees = 0f;
            lastAppliedForwardDistance = 0f;
            locomotionWorldPosition = transform.position;
            hasLocomotionWorldPosition = true;
            hasClimbSurfaceAnchor = false;
            wasTurnGestureActive = false;
            isRecoveringFromTurn = false;
            turnRecoveryElapsed = 0f;
            terrainForces?.ResetForces();
            activeForceSpeedMultiplier = 1f;
            activeForceStrideLength = minimumForceStrideLength;
            activeForceLiftMultiplier = minimumForceLiftMultiplier;
            leftSideWaveInPlace = false;
            rightSideWaveInPlace = false;
            leftFingerMimicLift = 0f;
            rightFingerMimicLift = 0f;
            currentLeftLegPushDistance = 0f;
            currentRightLegPushDistance = 0f;
            travelDirection = 1;
            pendingTravelDirection = 0;
            pendingDirectionStartTime = -1f;
            leftGaitPhase = 0f;
            rightGaitPhase = 0f;
            targetLeftGaitPhase = 0f;
            targetRightGaitPhase = 0f;
            gaitPhaseSpeed = 0.5f / referenceHalfStepDuration;
            leftAdaptiveActivationThreshold = alternationForceThreshold;
            rightAdaptiveActivationThreshold = alternationForceThreshold;
            leftAdaptiveReleaseThreshold = alternationReleaseThreshold;
            rightAdaptiveReleaseThreshold = alternationReleaseThreshold;
            ResetWorldFootTargets(leftLegs);
            ResetWorldFootTargets(rightLegs);
        }

        [ContextMenu("Rebuild Dexter Leg Chains")]
        public void InitializeRig()
        {
            initialized = false;
            DisableConflictingProceduralControllers();
            // These controls are intentionally fixed so older serialized components
            // cannot retain the original Thumb-to-left-leg assignment.
            leftLegFinger = DexterFinger.Index;
            rightLegFinger = DexterFinger.Middle;
            ApplyResponsiveMovementDefaults();

            if (receiver == null)
                receiver = GetComponent<DexterRelayUdpReceiver>();
            terrainForces = GetComponent<SpiderTerrainForces>();
            environmentForces = GetComponent<SpiderEnvironmentForces>();
            if (terrainForces == null)
                terrainForces = gameObject.AddComponent<SpiderTerrainForces>();

            activeTerrain = Terrain.activeTerrain;
            if (activeTerrain == null)
                activeTerrain = FindAnyObjectByType<Terrain>();

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

        private void DisableConflictingProceduralControllers()
        {
            ProceduralSpiderLegIK[] proceduralControllers =
                GetComponentsInChildren<ProceduralSpiderLegIK>(true);
            for (int i = 0; i < proceduralControllers.Length; i++)
            {
                ProceduralSpiderLegIK procedural = proceduralControllers[i];
                if (!procedural.enabled)
                    continue;

                procedural.enabled = false;
                Debug.Log(
                    $"{nameof(DexterFrontLegIK)} disabled {nameof(ProceduralSpiderLegIK)} " +
                    "so both controllers do not pose the same legs.",
                    procedural);
            }
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
                RestWorldTarget = effector.position,
                RestRigLocalTarget = rigSpace.InverseTransformPoint(effector.position),
                WorldTarget = effector.position,
                SwingStartWorldTarget = effector.position,
                SwingEndWorldTarget = effector.position,
                SwingSurfaceNormal = Vector3.up,
                GroundClearance = Mathf.Min(
                    0.12f, GetGroundClearance(effector.position))
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
                leftBaselineSquareSum += Vector2.Scale(leftForce, leftForce);
                leftBaselineSamples++;
                capturedSample = true;
            }
            if (TryReadForce(rightLegFinger, out Vector2 rightForce))
            {
                rightBaselineSum += rightForce;
                rightBaselineSquareSum += Vector2.Scale(rightForce, rightForce);
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
            float leftYNoise = CalculateStandardDeviation(
                leftBaseline.y, leftBaselineSquareSum.y, leftBaselineSamples);
            float rightYNoise = CalculateStandardDeviation(
                rightBaseline.y, rightBaselineSquareSum.y, rightBaselineSamples);
            leftAdaptiveActivationThreshold = Mathf.Clamp(
                Mathf.Max(alternationForceThreshold, leftYNoise * noiseStandardDeviationMultiplier),
                alternationForceThreshold,
                maximumAdaptiveNoiseThreshold);
            rightAdaptiveActivationThreshold = Mathf.Clamp(
                Mathf.Max(alternationForceThreshold, rightYNoise * noiseStandardDeviationMultiplier),
                alternationForceThreshold,
                maximumAdaptiveNoiseThreshold);
            leftAdaptiveReleaseThreshold = Mathf.Max(
                alternationReleaseThreshold, leftAdaptiveActivationThreshold * 0.6f);
            rightAdaptiveReleaseThreshold = Mathf.Max(
                alternationReleaseThreshold, rightAdaptiveActivationThreshold * 0.6f);
            hasBaseline = true;
            isTaring = false;
            Debug.Log(
                $"{nameof(DexterFrontLegIK)} calibration complete. " +
                $"Index baseline=({leftBaseline.x:F3}, {leftBaseline.y:F3}) N, " +
                $"Middle baseline=({rightBaseline.x:F3}, {rightBaseline.y:F3}) N. " +
                $"Y thresholds: Index={leftAdaptiveActivationThreshold:F3} N, " +
                $"Middle={rightAdaptiveActivationThreshold:F3} N.",
                this);
        }

        private void UpdateIpadMovementCalibration()
        {
            if ((!waitingForIpadMovementCalibration &&
                 !isIpadMovementCalibrating) || receiver == null)
                return;

            if (isIpadMovementCalibrating &&
                Time.realtimeSinceStartup - ipadMovementCalibrationStart >=
                calibrationDurationSeconds)
            {
                CompleteIpadMovementCalibration();
                return;
            }

            DexterForceFrame frame = receiver.LatestFrame;
            if (!IsRelayPositionFrame(frame) || !receiver.HasRecentFrame)
                return;

            if (!HasActiveIpadCalibrationFinger())
                return;

            if (waitingForIpadMovementCalibration)
            {
                waitingForIpadMovementCalibration = false;
                isIpadMovementCalibrating = true;
                ipadMovementCalibrationStart =
                    Time.realtimeSinceStartup;
                lastIpadMovementCalibrationSequence = long.MinValue;
                ipadMovementCalibrationSamples.Clear();
                ResetInputStateForMovementCalibration();
            }

            if (frame.sequence != lastIpadMovementCalibrationSequence)
            {
                lastIpadMovementCalibrationSequence = frame.sequence;
                float strongestY = 0f;
                bool hasDisplacement = false;
                if (TryReadForce(
                        leftLegFinger, out Vector2 leftCalibrationForce))
                {
                    strongestY = Mathf.Abs(leftCalibrationForce.y);
                    hasDisplacement = true;
                }
                if (TryReadForce(
                        rightLegFinger, out Vector2 rightCalibrationForce))
                {
                    strongestY = Mathf.Max(
                        strongestY,
                        Mathf.Abs(rightCalibrationForce.y));
                    hasDisplacement = true;
                }

                if (hasDisplacement && !float.IsNaN(strongestY) &&
                    !float.IsInfinity(strongestY))
                    ipadMovementCalibrationSamples.Add(strongestY);
            }
        }

        private void CompleteIpadMovementCalibration()
        {
            if (ipadMovementCalibrationSamples.Count > 0)
            {
                ipadMovementCalibrationSamples.Sort();
                int sampleIndex = Mathf.RoundToInt(
                    (ipadMovementCalibrationSamples.Count - 1) *
                    ipadCalibrationReferencePercentile);
                calibratedIpadReferenceDisplacement = Mathf.Max(
                    minimumIpadCalibrationDisplacement,
                    ipadMovementCalibrationSamples[
                        Mathf.Clamp(sampleIndex, 0,
                            ipadMovementCalibrationSamples.Count - 1)]);
            }
            else
            {
                calibratedIpadReferenceDisplacement = Mathf.Max(
                    minimumIpadCalibrationDisplacement,
                    minimumSpeedDisplacement);
            }

            isIpadMovementCalibrating = false;
            ipadMovementCalibrationComplete = true;
            ipadCalibrationCompleteMessageUntil =
                Time.realtimeSinceStartup + 1.5f;
            ResetInputStateForMovementCalibration();
            Debug.Log(
                $"{nameof(DexterFrontLegIK)} active iPad calibration " +
                $"complete: normal walking displacement=" +
                $"{calibratedIpadReferenceDisplacement:F3}, " +
                $"maximum-speed displacement=" +
                $"{GetActiveMaximumSpeedDisplacement():F3}.",
                this);
        }

        private void ResetInputStateForMovementCalibration()
        {
            smoothedLeftForce = Vector2.zero;
            smoothedRightForce = Vector2.zero;
            pressedFinger = 0;
            lastActiveFinger = 0;
            lastFingerPressTime = -1f;
            lastLeftFingerPressTime = -1f;
            lastRightFingerPressTime = -1f;
            leftFingerCadence = 0f;
            rightFingerCadence = 0f;
            alternatingDriveAuthorized = false;
            gaitDriveActive = false;
            lastCompletedAlternationTime = -1f;
            lastLiveFingerInputTime = -1f;
            forwardSpeed = 0f;
            turnInput = 0f;
            currentTurnSpeed = 0f;
            turnGaitWeight = 0f;
            wasTurnGestureActive = false;
            isRecoveringFromTurn = false;
            leftSideWaveInPlace = false;
            rightSideWaveInPlace = false;
            leftFingerMimicLift = 0f;
            rightFingerMimicLift = 0f;
            currentLeftLegPushDistance = 0f;
            currentRightLegPushDistance = 0f;
            targetForwardDistance = currentForwardDistance;
            lastAppliedForwardDistance = currentForwardDistance;
            ResetWorldFootTargets(leftLegs);
            ResetWorldFootTargets(rightLegs);
        }

        private static bool IsRelayPositionFrame(DexterForceFrame frame)
        {
            if (frame?.fingers == null)
                return false;

            string transport = frame.transport ?? string.Empty;
            return transport.IndexOf(
                       "ipad", StringComparison.OrdinalIgnoreCase) >= 0 ||
                   transport.IndexOf(
                       "touchpad", StringComparison.OrdinalIgnoreCase) >= 0;
        }

        private static bool IsIpadFrame(DexterForceFrame frame) =>
            IsRelayPositionFrame(frame);

        private bool HasActiveIpadCalibrationFinger()
        {
            return HasActivePositionMeasurement(
                       receiver?.GetFinger(leftLegFinger)) ||
                   HasActivePositionMeasurement(
                       receiver?.GetFinger(rightLegFinger));
        }

        private static bool HasActivePositionMeasurement(
            DexterFingerMeasurement measurement)
        {
            return measurement != null && measurement.has_data &&
                   measurement.force != null && measurement.force.Length >= 2;
        }

        private bool IsUsingCalibratedIpadInput()
        {
            return ipadMovementCalibrationComplete &&
                   IsIpadFrame(receiver?.LatestFrame);
        }

        private float GetActiveForceRangeScale()
        {
            if (!IsUsingCalibratedIpadInput())
                return 1f;

            return calibratedIpadReferenceDisplacement /
                   Mathf.Max(0.001f, minimumSpeedDisplacement);
        }

        private float GetActiveMinimumSpeedDisplacement()
        {
            return minimumSpeedDisplacement * GetActiveForceRangeScale();
        }

        private float GetActiveMaximumSpeedDisplacement()
        {
            return IsUsingCalibratedIpadInput()
                ? calibratedIpadReferenceDisplacement *
                  ipadMaximumSpeedDisplacementMultiplier
                : maximumSpeedDisplacement;
        }

        private float GetActiveTurnActivationForce()
        {
            return turnActivationForce * GetActiveForceRangeScale();
        }

        private float GetActiveTurnMinimumSpeedDisplacement()
        {
            return turnMinimumSpeedDisplacement *
                   GetActiveForceRangeScale();
        }

        private float GetActiveTurnMaximumSpeedDisplacement()
        {
            return IsUsingCalibratedIpadInput()
                ? GetActiveTurnMinimumSpeedDisplacement() *
                  ipadMaximumSpeedDisplacementMultiplier
                : turnForceForMaximumSpeed;
        }

        private static float CalculateStandardDeviation(float mean, float squareSum, int samples)
        {
            if (samples <= 1)
                return 0f;
            float variance = Mathf.Max(0f, squareSum / samples - mean * mean);
            return Mathf.Sqrt(variance);
        }

        private void UpdateDisplacements()
        {
            Vector2 leftForce = Vector2.zero;
            Vector2 rightForce = Vector2.zero;

            if (receiver.HasRecentFrame && !IsTaring &&
                !isIpadMovementCalibrating)
            {
                if (TryReadForce(leftLegFinger, out Vector2 measuredLeftForce))
                    leftForce = measuredLeftForce - (hasBaseline ? leftBaseline : Vector2.zero);
                if (TryReadForce(rightLegFinger, out Vector2 measuredRightForce))
                    rightForce = measuredRightForce - (hasBaseline ? rightBaseline : Vector2.zero);
            }

            if (temporalSmoothingWindow > 0f)
            {
                float smoothingBlend = 1f - Mathf.Exp(-Time.deltaTime / temporalSmoothingWindow);
                smoothedLeftForce = Vector2.Lerp(smoothedLeftForce, leftForce, smoothingBlend);
                smoothedRightForce = Vector2.Lerp(smoothedRightForce, rightForce, smoothingBlend);
            }
            else
            {
                smoothedLeftForce = leftForce;
                smoothedRightForce = rightForce;
            }

            bool isTurning = UpdateTurning(smoothedLeftForce, smoothedRightForce);
            UpdateAlternatingLocomotion(smoothedLeftForce, smoothedRightForce, isTurning);
            ApplyLocomotion();

            UpdateAlternatingGait();
        }

        private void UpdateAlternatingLocomotion(Vector2 leftForce, Vector2 rightForce, bool isTurning)
        {
            float leftActivity = Mathf.Abs(leftForce.y);
            float rightActivity = Mathf.Abs(rightForce.y);
            int activeFinger = 0;
            bool pressIntervalElapsed = lastFingerPressTime < 0f ||
                                        Time.time - lastFingerPressTime >= minimumPressInterval;

            // Locomotion is intentionally forward-only. Returning a finger to
            // neutral must never be interpreted as a request to walk backward.
            travelDirection = 1;

            // A neutral release rearms the same finger. The opposite finger may
            // also take over immediately once it is clearly dominant; requiring
            // the previous finger to cross zero caused multi-second freezes while
            // the user was already performing a valid alternating motion.
            if (isTurning)
            {
                pressedFinger = 0;
            }
            else if (pressedFinger != 0)
            {
                bool pressedFingerWasReleased = pressedFinger == 1
                    ? leftActivity <= leftAdaptiveReleaseThreshold
                    : rightActivity <= rightAdaptiveReleaseThreshold;
                if (pressedFingerWasReleased)
                    pressedFinger = 0;
                else if (pressIntervalElapsed && pressedFinger == 1 &&
                         rightActivity >= rightAdaptiveActivationThreshold &&
                         rightActivity >= leftActivity + alternationDominanceMargin)
                    activeFinger = 2;
                else if (pressIntervalElapsed && pressedFinger == 2 &&
                         leftActivity >= leftAdaptiveActivationThreshold &&
                         leftActivity >= rightActivity + alternationDominanceMargin)
                    activeFinger = 1;
            }

            if (!isTurning && pressedFinger == 0 && activeFinger == 0 &&
                leftActivity >= leftAdaptiveActivationThreshold &&
                leftActivity >= rightActivity + alternationDominanceMargin)
            {
                activeFinger = 1;
            }
            else if (!isTurning && pressedFinger == 0 && activeFinger == 0 &&
                     rightActivity >= rightAdaptiveActivationThreshold &&
                     rightActivity >= leftActivity + alternationDominanceMargin)
            {
                activeFinger = 2;
            }

            if (!isTurning && pressIntervalElapsed && activeFinger != 0 && activeFinger != pressedFinger)
            {
                RegisterFingerPress(activeFinger);
                pressedFinger = activeFinger;
            }

            // Alternation only unlocks walking. Live displacement—not the
            // alternation interval—drives speed and stride on every frame.
            float liveDisplacement = Mathf.Max(leftActivity, rightActivity);
            bool hasLiveFingerInput = leftActivity >= leftAdaptiveActivationThreshold ||
                                      rightActivity >= rightAdaptiveActivationThreshold;
            if (hasLiveFingerInput)
                lastLiveFingerInputTime = Time.time;

            bool inputRecentlyActive = lastLiveFingerInputTime >= 0f &&
                Time.time - lastLiveFingerInputTime <= inputDriveReleaseDelay;
            if (!inputRecentlyActive)
            {
                alternatingDriveAuthorized = false;
                lastActiveFinger = 0;
                leftSideWaveInPlace = false;
                rightSideWaveInPlace = false;
            }

            bool alternationRecentlyConfirmed =
                lastCompletedAlternationTime >= 0f &&
                Time.time - lastCompletedAlternationTime <=
                alternatingDriveSustainTime;
            if (!alternationRecentlyConfirmed)
                alternatingDriveAuthorized = false;
            gaitDriveActive = alternatingDriveAuthorized &&
                              alternationRecentlyConfirmed &&
                              inputRecentlyActive &&
                              !isTurning &&
                              !isRecoveringFromTurn;

            float activeMinimumSpeedDisplacement =
                GetActiveMinimumSpeedDisplacement();
            float leftMimicTarget = leftSideWaveInPlace
                ? Mathf.InverseLerp(0f, activeMinimumSpeedDisplacement,
                    Mathf.Max(0f, smoothedLeftForce.y)) * fingerMimicLiftHeight
                : 0f;
            float rightMimicTarget = rightSideWaveInPlace
                ? Mathf.InverseLerp(0f, activeMinimumSpeedDisplacement,
                    Mathf.Max(0f, smoothedRightForce.y)) * fingerMimicLiftHeight
                : 0f;
            leftFingerMimicLift = Mathf.MoveTowards(
                leftFingerMimicLift, leftMimicTarget,
                fingerMimicResponse * Time.deltaTime);
            rightFingerMimicLift = Mathf.MoveTowards(
                rightFingerMimicLift, rightMimicTarget,
                fingerMimicResponse * Time.deltaTime);
            float leftPushTarget = GetTargetLegPushDistance(
                smoothedLeftForce.y, leftGaitWeight, leftSideWaveInPlace);
            float rightPushTarget = GetTargetLegPushDistance(
                smoothedRightForce.y, rightGaitWeight, rightSideWaveInPlace);
            currentLeftLegPushDistance = Mathf.MoveTowards(
                currentLeftLegPushDistance, leftPushTarget,
                legPushResponse * maximumLegPushDistance * Time.deltaTime);
            currentRightLegPushDistance = Mathf.MoveTowards(
                currentRightLegPushDistance, rightPushTarget,
                legPushResponse * maximumLegPushDistance * Time.deltaTime);

            activeForceSpeedMultiplier = GetForceStepSpeedMultiplier(liveDisplacement);
            activeForceStrideLength = GetForceStrideLength(liveDisplacement);
            float requestedGaitPhaseSpeed =
                0.5f / referenceHalfStepDuration *
                activeForceSpeedMultiplier *
                overallMovementSpeedMultiplier;
            float phaseSpeedBlend = 1f - Mathf.Exp(
                -gaitBlendSpeed * Time.deltaTime);
            gaitPhaseSpeed = Mathf.Lerp(
                gaitPhaseSpeed, requestedGaitPhaseSpeed, phaseSpeedBlend);
            activeForceLiftMultiplier = Mathf.Lerp(
                activeForceLiftMultiplier,
                GetForceLiftMultiplier(liveDisplacement),
                phaseSpeedBlend);
            float locomotionStride = Mathf.Clamp(
                Mathf.Max(minimumStrideLength, activeForceStrideLength),
                minimumStrideLength,
                maximumStrideLength) * walkingStrideEmphasis;
            float strideMatchedSpeed = gaitPhaseSpeed * locomotionStride;
            float forceSpeedCeiling = normalForceTravelSpeed *
                                      activeForceSpeedMultiplier *
                                      overallMovementSpeedMultiplier;
            float requestedForwardSpeed = gaitDriveActive
                ? Mathf.Min(forceSpeedCeiling, strideMatchedSpeed)
                : 0f;
            if (environmentForces != null)
                requestedForwardSpeed *= environmentForces.GetMovementMultiplier(
                    locomotionWorldPosition);
            requestedForwardSpeed *= GetPlantedPushDriveStrength();
            forwardSpeed = terrainForces != null
                ? terrainForces.MoveWalkingSpeed(
                    forwardSpeed, requestedForwardSpeed, Time.deltaTime)
                : Mathf.MoveTowards(
                    forwardSpeed, requestedForwardSpeed, Time.deltaTime * 6f);
            currentForwardDistance += forwardSpeed * Time.deltaTime;
            targetForwardDistance = currentForwardDistance;

            if (lastLeftFingerPressTime < 0f ||
                Time.time - lastLeftFingerPressTime > 0.75f)
                leftFingerCadence = Mathf.MoveTowards(
                    leftFingerCadence, 0f, Time.deltaTime * 3f);
            if (lastRightFingerPressTime < 0f ||
                Time.time - lastRightFingerPressTime > 0.75f)
                rightFingerCadence = Mathf.MoveTowards(
                    rightFingerCadence, 0f, Time.deltaTime * 3f);
        }

        private void RegisterFingerPress(int activeFinger)
        {
            float now = Time.time;
            bool completesAlternation = lastActiveFinger != 0 &&
                                        activeFinger != lastActiveFinger &&
                                        lastFingerPressTime >= 0f &&
                                        now - lastFingerPressTime <=
                                        maximumAlternationInterval;

            if (activeFinger == 1)
            {
                leftSideWaveInPlace = !completesAlternation;
                if (lastLeftFingerPressTime >= 0f)
                {
                    float cadence = 1f / Mathf.Max(
                        0.08f, now - lastLeftFingerPressTime);
                    leftFingerCadence = Mathf.Lerp(
                        leftFingerCadence, cadence, 0.5f);
                }
                else
                {
                    leftFingerCadence = cadenceForFullSupport;
                }
                lastLeftFingerPressTime = now;
            }
            else
            {
                rightSideWaveInPlace = !completesAlternation;
                if (lastRightFingerPressTime >= 0f)
                {
                    float cadence = 1f / Mathf.Max(
                        0.08f, now - lastRightFingerPressTime);
                    rightFingerCadence = Mathf.Lerp(
                        rightFingerCadence, cadence, 0.5f);
                }
                else
                {
                    rightFingerCadence = cadenceForFullSupport;
                }
                lastRightFingerPressTime = now;
            }

            // Before alternation, the selected side follows finger height
            // directly. Once alternating, presses advance normal walking phases.
            if (completesAlternation)
            {
                leftSideWaveInPlace = false;
                rightSideWaveInPlace = false;
                // Alternation authorizes the gait; it does not add a fixed
                // movement unit. Live displacement controls cadence and stride
                // continuously while valid alternations keep refreshing the
                // authorization window.
                float synchronizedPhase = Mathf.Max(
                    leftGaitPhase, rightGaitPhase);
                leftGaitPhase = synchronizedPhase;
                rightGaitPhase = synchronizedPhase;
                targetLeftGaitPhase = synchronizedPhase;
                targetRightGaitPhase = synchronizedPhase;
                alternatingDriveAuthorized = true;
                lastCompletedAlternationTime = now;
            }

            lastActiveFinger = activeFinger;
            lastFingerPressTime = now;
        }

        private float GetForceStepSpeedMultiplier(float fingerForce)
        {
            float displacementStrength = GetForceDisplacementStrength(
                fingerForce);
            return Mathf.Lerp(
                minimumStepSpeedMultiplier,
                maximumStepSpeedMultiplier,
                displacementStrength);
        }

        private float GetForceStrideLength(float fingerForce)
        {
            float displacementStrength = GetForceDisplacementStrength(
                fingerForce);
            return Mathf.Lerp(
                minimumForceStrideLength,
                maximumForceStrideLength,
                displacementStrength);
        }

        private float GetForceLiftMultiplier(float fingerForce)
        {
            return Mathf.Lerp(
                minimumForceLiftMultiplier,
                maximumForceLiftMultiplier,
                GetForceDisplacementStrength(fingerForce));
        }

        private float GetForceDisplacementStrength(float fingerForce)
        {
            return Mathf.InverseLerp(
                GetActiveMinimumSpeedDisplacement(),
                GetActiveMaximumSpeedDisplacement(),
                Mathf.Max(0f, fingerForce));
        }

        private void UpdateAlternatingGait()
        {
            if (isRecoveringFromTurn)
            {
                UpdateTurnRecovery();
                return;
            }

            if (turnGaitWeight > 0.01f)
            {
                UpdateTurningGait();
                return;
            }

            bool leftIsMoving = gaitDriveActive;
            bool rightIsMoving = gaitDriveActive;
            if (gaitDriveActive)
            {
                float gaitTravelStride = Mathf.Clamp(
                    Mathf.Max(minimumStrideLength, activeForceStrideLength),
                    minimumStrideLength,
                    maximumStrideLength) * walkingStrideEmphasis;
                float surfaceSpeedMultiplier = terrainForces != null
                    ? terrainForces.GetMovementMultiplier(
                        GetTerrainSamplingPosition())
                    : 1f;
                // One complete gait cycle advances the body by one stride.
                // Deriving phase speed from actual friction/slope-adjusted
                // travel keeps the feet from cycling faster than the spider.
                float travelMatchedPhaseSpeed = gaitTravelStride > 0.0001f
                    ? Mathf.Abs(forwardSpeed) * surfaceSpeedMultiplier /
                      gaitTravelStride
                    : 0f;
                float synchronizedPhase = Mathf.Max(
                    leftGaitPhase, rightGaitPhase) +
                    travelMatchedPhaseSpeed * Time.deltaTime;
                leftGaitPhase = synchronizedPhase;
                rightGaitPhase = synchronizedPhase;
                targetLeftGaitPhase = synchronizedPhase;
                targetRightGaitPhase = synchronizedPhase;
            }
            else
            {
                targetLeftGaitPhase = leftGaitPhase;
                targetRightGaitPhase = rightGaitPhase;
            }

            float gaitBlend = 1f - Mathf.Exp(-gaitBlendSpeed * Time.deltaTime);
            leftGaitWeight = Mathf.Lerp(leftGaitWeight, leftIsMoving ? 1f : 0f, gaitBlend);
            rightGaitWeight = Mathf.Lerp(rightGaitWeight, rightIsMoving ? 1f : 0f, gaitBlend);

            float strideLength = Mathf.Clamp(
                Mathf.Max(minimumStrideLength, activeForceStrideLength),
                minimumStrideLength,
                maximumStrideLength);
            UpdateSideWorldTargets(
                leftLegs, true, Mathf.Repeat(leftGaitPhase, 1f),
                leftGaitWeight,
                leftSideWaveInPlace ? 0f : strideLength * walkingStrideEmphasis,
                1);
            // Tetrapod gait: the right side is half a cycle out of phase.
            // Left front + third move with right second + back, followed by
            // the complementary four-leg group.
            UpdateSideWorldTargets(
                rightLegs, false, Mathf.Repeat(rightGaitPhase, 1f),
                rightGaitWeight,
                rightSideWaveInPlace ? 0f : strideLength * walkingStrideEmphasis,
                1);
        }

        private bool UpdateTurning(Vector2 leftForce, Vector2 rightForce)
        {
            float xSign = invertFx ? -1f : 1f;
            float leftX = leftForce.x * xSign;
            float rightX = rightForce.x * xSign;
            float requestedTurn = 0f;
            float activeTurnActivationForce =
                GetActiveTurnActivationForce();
            bool leftTurnReady = Mathf.Abs(leftX) >=
                                 activeTurnActivationForce &&
                                 Mathf.Abs(leftX) >=
                                 Mathf.Abs(leftForce.y) * turnAxisDominanceRatio;
            bool rightTurnReady = Mathf.Abs(rightX) >=
                                  activeTurnActivationForce &&
                                  Mathf.Abs(rightX) >=
                                  Mathf.Abs(rightForce.y) * turnAxisDominanceRatio;

            if (leftTurnReady && rightTurnReady && leftX > 0f && rightX > 0f)
                requestedTurn = Mathf.Min(leftX, rightX);
            else if (leftTurnReady && rightTurnReady && leftX < 0f && rightX < 0f)
                requestedTurn = -Mathf.Min(-leftX, -rightX);

            float blend = 1f - Mathf.Exp(-turnResponseSpeed * Time.deltaTime);
            turnInput = Mathf.Lerp(turnInput, requestedTurn, blend);
            float magnitude = Mathf.Abs(turnInput);
            float strength = Mathf.InverseLerp(
                GetActiveTurnMinimumSpeedDisplacement(),
                GetActiveTurnMaximumSpeedDisplacement(),
                magnitude);
            float requestedSpeed = requestedTurn == 0f
                ? 0f
                : Mathf.Sign(requestedTurn) * Mathf.Lerp(
                    minimumTurnDegreesPerSecond,
                    maximumTurnDegreesPerSecond,
                    Mathf.Sqrt(strength));
            currentTurnSpeed = Mathf.Lerp(currentTurnSpeed, requestedSpeed, blend);
            currentYawDegrees += currentTurnSpeed * Time.deltaTime;
            turnGaitWeight = Mathf.Lerp(
                turnGaitWeight, requestedTurn == 0f ? 0f : 1f, blend);

            bool hasTurnGesture = requestedTurn != 0f;
            if (hasTurnGesture)
            {
                isRecoveringFromTurn = false;
                float cyclesPerSecond = Mathf.Lerp(
                    minimumGaitCyclesPerSecond,
                    maximumGaitCyclesPerSecond,
                    Mathf.Sqrt(strength));
                turnGaitPhase = Mathf.Repeat(
                    turnGaitPhase + cyclesPerSecond * Time.deltaTime, 1f);
            }

            if (!hasTurnGesture && wasTurnGestureActive)
                BeginTurnRecovery();
            wasTurnGestureActive = hasTurnGesture;

            return hasTurnGesture;
        }

        private void UpdateTurningGait()
        {
            int direction = currentTurnSpeed >= 0f ? 1 : -1;
            float strength = Mathf.InverseLerp(
                GetActiveTurnMinimumSpeedDisplacement(),
                GetActiveTurnMaximumSpeedDisplacement(),
                Mathf.Abs(turnInput));
            float baseStride = Mathf.Lerp(
                minimumStrideLength,
                maximumStrideLength,
                Mathf.Sqrt(strength));
            float leftStride = baseStride * (direction > 0
                ? outsideTurnStrideMultiplier
                : insideTurnStrideMultiplier);
            float rightStride = baseStride * (direction < 0
                ? outsideTurnStrideMultiplier
                : insideTurnStrideMultiplier);

            // Right turn: left/outside legs advance and right/inside legs
            // counter-stride. Left turn is the exact mirrored pattern.
            UpdateSideWorldTargets(
                leftLegs, true, turnGaitPhase,
                turnGaitWeight, leftStride, direction);
            UpdateSideWorldTargets(
                rightLegs, false, turnGaitPhase,
                turnGaitWeight, rightStride, -direction);
        }

        private void BeginTurnRecovery()
        {
            isRecoveringFromTurn = true;
            turnRecoveryElapsed = 0f;
            CaptureRecoveryStarts(leftLegs);
            CaptureRecoveryStarts(rightLegs);
        }

        private static void CaptureRecoveryStarts(LegChain[] legs)
        {
            if (legs == null)
                return;
            for (int i = 0; i < legs.Length; i++)
            {
                if (legs[i] == null)
                    continue;
                legs[i].SwingStartWorldTarget = legs[i].WorldTarget;
                legs[i].WasSwinging = false;
            }
        }

        private void UpdateTurnRecovery()
        {
            turnRecoveryElapsed += Time.deltaTime;
            UpdateRecoverySide(leftLegs, true);
            UpdateRecoverySide(rightLegs, false);

            if (turnRecoveryElapsed < turnRecoveryDuration)
                return;

            isRecoveringFromTurn = false;
            turnGaitWeight = 0f;
            leftGaitWeight = 0f;
            rightGaitWeight = 0f;
            targetLeftGaitPhase = leftGaitPhase;
            targetRightGaitPhase = rightGaitPhase;
        }

        private void UpdateRecoverySide(LegChain[] legs, bool isLeftSide)
        {
            if (legs == null || rigSpace == null)
                return;
            float doubledProgress = turnRecoveryElapsed /
                                    Mathf.Max(0.1f, turnRecoveryDuration) * 2f;
            for (int row = 0; row < legs.Length; row++)
            {
                LegChain leg = legs[row];
                if (leg == null)
                    continue;
                // Recover in the same diagonal tetrapod groups used for walking.
                bool firstGroup = isLeftSide ? row % 2 == 0 : row % 2 != 0;
                float progress = Mathf.Clamp01(
                    doubledProgress - (firstGroup ? 0f : 1f));
                float eased = SmoothStep01(progress);
                Vector3 naturalLanding = rigSpace.TransformPoint(leg.RestRigLocalTarget);
                if (TryGetFootSurfaceTarget(
                        naturalLanding, leg.GroundClearance,
                        out Vector3 groundedLanding, out Vector3 surfaceNormal))
                    naturalLanding = groundedLanding;
                else
                {
                    naturalLanding.y = leg.RestWorldTarget.y;
                    surfaceNormal = Vector3.up;
                }

                leg.WorldTarget = Vector3.Lerp(
                    leg.SwingStartWorldTarget, naturalLanding, eased);
                leg.WorldTarget += surfaceNormal *
                    (Mathf.Sin(progress * Mathf.PI) *
                     stepLiftHeight * 0.45f);
                leg.SwingEndWorldTarget = naturalLanding;
                leg.SwingSurfaceNormal = surfaceNormal;
            }
        }

        private void UpdateSideWorldTargets(
            LegChain[] side,
            bool isLeftSide,
            float phase,
            float weight,
            float strideLength,
            int direction)
        {
            Vector3 rigForward = terrainForces != null
                ? terrainForces.GetSurfaceTravelDirection(
                    GetTerrainSamplingPosition(), GetRigForward())
                : GetRigForward();
            for (int row = 0; row < side.Length; row++)
            {
                LegChain leg = side[row];
                float legPhase = GetTetrapodLegPhase(
                    phase, isLeftSide, row);
                bool isSwinging = weight > 0.01f && legPhase < swingPhaseFraction;

                if (isSwinging && !leg.WasSwinging)
                {
                    leg.SwingStartWorldTarget = leg.WorldTarget;
                    Vector3 desiredLanding =
                        rigSpace.TransformPoint(leg.RestRigLocalTarget) +
                        rigForward * (strideLength * 0.5f * direction);
                    if (TryGetFootSurfaceTarget(
                            desiredLanding, leg.GroundClearance,
                            out Vector3 naturalLanding,
                            out Vector3 surfaceNormal))
                    {
                        leg.SwingEndWorldTarget = naturalLanding;
                        leg.SwingSurfaceNormal = surfaceNormal;
                    }
                    else
                    {
                        desiredLanding.y = leg.RestWorldTarget.y;
                        leg.SwingEndWorldTarget = desiredLanding;
                        leg.SwingSurfaceNormal = Vector3.up;
                    }
                }

                if (isSwinging)
                {
                    leg.IsRecoveringLag = false;
                    float swingTime = Mathf.Clamp01(
                        legPhase / swingPhaseFraction);
                    // Parametric half-ellipse: horizontal travel uses a cosine
                    // ease while height follows one clean sine arch. Smoothly
                    // warping time gives the foot zero takeoff/landing velocity
                    // without changing the visible sinusoidal path.
                    float easedSwingTime = SmoothStep01(swingTime);
                    float arcProgress = 0.5f -
                                        0.5f * Mathf.Cos(
                                            easedSwingTime * Mathf.PI);
                    float arcLift = Mathf.Sin(
                        easedSwingTime * Mathf.PI);
                    leg.WorldTarget = Vector3.Lerp(
                        leg.SwingStartWorldTarget,
                        leg.SwingEndWorldTarget,
                        arcProgress);
                    leg.WorldTarget += leg.SwingSurfaceNormal *
                        (arcLift * stepLiftHeight *
                         walkingLiftEmphasis *
                         activeForceLiftMultiplier *
                         Mathf.Clamp01(weight));
                }
                else if (leg.WasSwinging)
                {
                    // Once planted, this point remains fixed in world space while
                    // the body moves, producing the visible stance/pull motion.
                    leg.WorldTarget = leg.SwingEndWorldTarget;
                }

                if (!isSwinging && weight <= 0.01f)
                {
                    // Once walking has stopped, recover any remaining reach
                    // gently. During an active gait the scheduled tetrapod
                    // swing owns recovery, preventing isolated legs from
                    // stepping out of their 1/3 or 2/4 group.
                    UpdateLagRecovery(leg);
                }
                else if (!isSwinging)
                {
                    leg.IsRecoveringLag = false;
                    leg.LagRecoveryProgress = 0f;
                }

                // A planted world-space target may encounter rising terrain as
                // the body advances. Never allow the solved foot below the local
                // surface while it waits for its next swing.
                if (TryGetFootSurfaceTarget(
                        leg.WorldTarget, leg.GroundClearance,
                        out Vector3 stanceContact, out Vector3 stanceNormal))
                {
                    float penetration = Vector3.Dot(
                        stanceContact - leg.WorldTarget, stanceNormal);
                    if (penetration > 0f)
                        leg.WorldTarget += stanceNormal * penetration;
                }

                leg.WasSwinging = isSwinging;
            }
        }

        private void UpdateLagRecovery(LegChain leg)
        {
            Vector3 naturalLanding = rigSpace.TransformPoint(leg.RestRigLocalTarget);
            if (TryGetFootSurfaceTarget(
                    naturalLanding, leg.GroundClearance,
                    out Vector3 groundedLanding, out Vector3 surfaceNormal))
                naturalLanding = groundedLanding;
            else
            {
                naturalLanding.y = leg.RestWorldTarget.y;
                surfaceNormal = Vector3.up;
            }

            Vector3 surfaceLag = Vector3.ProjectOnPlane(
                naturalLanding - leg.WorldTarget, surfaceNormal);
            if (!leg.IsRecoveringLag && surfaceLag.magnitude > maximumPlantedFootLag)
            {
                leg.IsRecoveringLag = true;
                leg.LagRecoveryProgress = 0f;
                leg.LagRecoveryStart = leg.WorldTarget;
            }

            if (!leg.IsRecoveringLag)
                return;

            leg.LagRecoveryProgress = Mathf.Clamp01(
                leg.LagRecoveryProgress + Time.deltaTime /
                Mathf.Max(0.05f, lagRecoveryDuration));
            float eased = SmoothStep01(leg.LagRecoveryProgress);
            leg.WorldTarget = Vector3.Lerp(
                leg.LagRecoveryStart, naturalLanding, eased);
            leg.WorldTarget += surfaceNormal *
                (Mathf.Sin(leg.LagRecoveryProgress * Mathf.PI) *
                 stepLiftHeight * 0.70f);
            leg.SwingEndWorldTarget = naturalLanding;
            leg.SwingSurfaceNormal = surfaceNormal;

            if (leg.LagRecoveryProgress >= 1f)
                leg.IsRecoveringLag = false;
        }

        private void UpdateTravelDirection(float leftYForce, float rightYForce)
        {
            int requestedDirection = 0;
            // Direction is a deliberate two-finger gesture. Single-finger cadence
            // strokes, returns to neutral, and opposite-signed forces cannot alter
            // the persistent direction latch.
            if (leftYForce >= directionChangeForceThreshold &&
                rightYForce >= directionChangeForceThreshold)
                requestedDirection = 1;
            else if (leftYForce <= -directionChangeForceThreshold &&
                     rightYForce <= -directionChangeForceThreshold)
                requestedDirection = -1;

            // Resting, weak forces, and forces in the already-selected direction
            // cannot disturb the direction latch.
            if (requestedDirection == 0 || requestedDirection == travelDirection)
            {
                pendingTravelDirection = 0;
                pendingDirectionStartTime = -1f;
                return;
            }

            if (requestedDirection != pendingTravelDirection)
            {
                pendingTravelDirection = requestedDirection;
                pendingDirectionStartTime = Time.time;
                return;
            }

            if (Time.time - pendingDirectionStartTime < directionChangeHoldTime)
                return;

            travelDirection = pendingTravelDirection;
            pendingTravelDirection = 0;
            pendingDirectionStartTime = -1f;
        }

        private void ApplyLocomotion()
        {
            if (!hasLocomotionWorldPosition)
            {
                locomotionWorldPosition = transform.position;
                hasLocomotionWorldPosition = true;
            }

            transform.localRotation = rootLocalRotation *
                                      Quaternion.Euler(0f, currentYawDegrees, 0f);
            Vector3 movementStart = locomotionWorldPosition;
            float forwardDelta = currentForwardDistance - lastAppliedForwardDistance;
            Vector3 surfaceSample = GetTerrainSamplingPosition();
            float terrainMovementMultiplier = terrainForces != null
                ? terrainForces.GetMovementMultiplier(surfaceSample)
                : 1f;
            Vector3 surfaceForward = terrainForces != null
                ? terrainForces.GetSurfaceTravelDirection(
                    surfaceSample, GetRigForward())
                : GetRigForward();
            Vector3 movedSurfaceSample = surfaceSample + surfaceForward *
                (forwardDelta * terrainMovementMultiplier);
            lastAppliedForwardDistance = currentForwardDistance;
            if (terrainForces != null)
                movedSurfaceSample = terrainForces.ApplyForces(
                    movedSurfaceSample);

            Vector3 surfacePoint = movedSurfaceSample;
            Vector3 surfaceNormal = Vector3.up;
            bool foundSurfaceFrame = terrainForces != null &&
                terrainForces.TryGetTerrainSurfaceFrame(
                    movedSurfaceSample,
                    out surfacePoint,
                    out surfaceNormal);
            bool enteredClimb = foundSurfaceFrame &&
                                terrainForces.IsClimbableSurfaceNormal(
                                    surfaceNormal);
            // Clear the wall anchor as soon as the root reaches ordinary
            // terrain. Retaining it on a merely sloped ground surface keeps
            // the previous wall normal and leaves the torso partially sunk.
            bool remainSurfaceAnchored = hasClimbSurfaceAnchor &&
                                         foundSurfaceFrame &&
                                         terrainForces.IsClimbableSurfaceNormal(
                                             surfaceNormal);

            if (enteredClimb || remainSurfaceAnchored)
            {
                hasClimbSurfaceAnchor = true;
                climbSurfaceAnchor = surfacePoint;
                Vector3 desiredRoot = surfacePoint +
                                      surfaceNormal * terrainRootClearance;
                locomotionWorldPosition = ResolveObstacleMovement(
                    movementStart, desiredRoot);
            }
            else
            {
                hasClimbSurfaceAnchor = false;
                locomotionWorldPosition = ResolveObstacleMovement(
                    movementStart, movedSurfaceSample);
            }
            transform.position = locomotionWorldPosition;

            if (!hasClimbSurfaceAnchor &&
                hasTerrainRootClearance && terrainForces != null &&
                terrainForces.TryGetRequiredBodyHeight(
                    transform.position,
                    GetRigForward(),
                    rigSpace.right,
                    terrainRootClearance,
                    out float requiredBodyHeight))
            {
                Vector3 worldPosition = transform.position;
                worldPosition.y = requiredBodyHeight;
                transform.position = worldPosition;
                locomotionWorldPosition = worldPosition;
            }

            if (terrainForces != null)
            {
                Vector3 facingDirection = surfaceForward;
                Vector3 actualSurfaceTravel = Vector3.ProjectOnPlane(
                    transform.position - movementStart,
                    foundSurfaceFrame ? surfaceNormal : Vector3.up);
                if (Mathf.Abs(forwardDelta) > 0.00001f &&
                    actualSurfaceTravel.sqrMagnitude > 0.00000025f)
                {
                    // Use the direction the root actually advanced after
                    // terrain projection, body clearance, and collision
                    // resolution. This keeps the face forward even on a
                    // diagonal or near-vertical surface.
                    facingDirection = actualSurfaceTravel.normalized;
                }

                transform.rotation = terrainForces.GetSlopeAlignedRootRotation(
                    GetTerrainSamplingPosition(),
                    transform.rotation,
                    facingDirection);
            }

        }

        private Vector3 GetTerrainSamplingPosition()
        {
            return hasClimbSurfaceAnchor
                ? climbSurfaceAnchor
                : locomotionWorldPosition;
        }

        private Vector3 ResolveObstacleMovement(Vector3 start, Vector3 desired)
        {
            Vector3 horizontalDelta = Vector3.ProjectOnPlane(
                desired - start, Vector3.up);
            float distance = horizontalDelta.magnitude;
            if (distance < 0.0001f)
                return desired;

            Vector3 castOrigin = start + Vector3.up * bodyCollisionHeight;
            RaycastHit[] hits = Physics.SphereCastAll(
                castOrigin,
                bodyCollisionRadius,
                horizontalDelta / distance,
                distance + obstacleSkin,
                obstacleLayers,
                QueryTriggerInteraction.Ignore);
            float permittedDistance = distance;
            for (int i = 0; i < hits.Length; i++)
            {
                Collider hitCollider = hits[i].collider;
                if (hitCollider == null ||
                    IsWalkableTerrainContact(hits[i]) ||
                    IsSoftFoliageCollider(hitCollider) ||
                    hitCollider.transform.IsChildOf(transform))
                    continue;
                permittedDistance = Mathf.Min(
                    permittedDistance,
                    Mathf.Max(0f, hits[i].distance - obstacleSkin));
            }

            if (permittedDistance >= distance)
                return desired;

            terrainForces?.StopOnCollision();
            Vector3 resolved = start + horizontalDelta.normalized * permittedDistance;
            resolved.y = desired.y;
            return resolved;
        }

        private bool IsWalkableTerrainContact(RaycastHit hit)
        {
            // Terrain-painted tree trunks may be reported through the same
            // TerrainCollider as the ground. A steep contact is ignored only
            // when its normal agrees with the actual TerrainData wall normal;
            // vertical tree/trunk contacts therefore remain blocking.
            return hit.collider is TerrainCollider &&
                   (hit.normal.y >= 0.55f ||
                    (terrainForces != null &&
                     terrainForces.IsClimbableTerrainContact(
                         hit.point, hit.normal)));
        }

        private static bool IsSoftFoliageCollider(Collider collider)
        {
            Transform current = collider.transform;
            bool foundSoftName = false;
            for (int depth = 0; current != null && depth < 5; depth++, current = current.parent)
            {
                string objectName = current.name.ToLowerInvariant();
                if (objectName.Contains("tree") ||
                    objectName.Contains("poplar") ||
                    objectName.Contains("trunk") ||
                    objectName.Contains("stump") ||
                    objectName.Contains("timber") ||
                    objectName.Contains("wood") ||
                    objectName.Contains("rock") ||
                    objectName.Contains("cliff") ||
                    objectName.Contains("fence"))
                    return false;

                if (objectName.Contains("grass") ||
                    objectName.Contains("flower") ||
                    objectName.Contains("bush") ||
                    objectName.Contains("shrub") ||
                    objectName.Contains("leaf") ||
                    objectName.Contains("leaves") ||
                    objectName.Contains("plant"))
                    foundSoftName = true;
            }

            return foundSoftName;
        }

        private float GetGroundClearance(Vector3 worldPosition)
        {
            return TrySampleTerrainHeight(worldPosition, out float terrainHeight)
                ? Mathf.Max(0f, worldPosition.y - terrainHeight)
                : 0f;
        }

        private bool TryGetFootSurfaceTarget(
            Vector3 desired,
            float clearance,
            out Vector3 surfaceTarget,
            out Vector3 surfaceNormal)
        {
            if (TryGetActiveClimbNormal(out Vector3 activeClimbNormal) &&
                terrainForces.TryProjectFootToClimbSurface(
                    desired, activeClimbNormal, clearance,
                    out surfaceTarget, out surfaceNormal))
                return true;

            if (terrainForces != null &&
                terrainForces.TryGetClimbSurfaceContact(
                    desired, clearance, out surfaceTarget, out surfaceNormal))
                return true;

            surfaceTarget = desired;
            surfaceNormal = Vector3.up;
            if (!TrySampleTerrainHeight(desired, out float terrainHeight))
                return false;
            surfaceTarget.y = terrainHeight + clearance;
            return true;
        }

        private bool TryGetActiveClimbNormal(out Vector3 surfaceNormal)
        {
            surfaceNormal = Vector3.up;
            return hasClimbSurfaceAnchor &&
                   terrainForces != null &&
                   terrainForces.TryGetTerrainSurfaceFrame(
                       climbSurfaceAnchor, out _, out surfaceNormal) &&
                   terrainForces.IsClimbableSurfaceNormal(surfaceNormal);
        }

        private bool TrySampleTerrainHeight(Vector3 worldPosition, out float height)
        {
            height = 0f;
            bool foundSurface = false;
            if (activeTerrain != null && activeTerrain.terrainData != null)
            {
                Vector3 origin = activeTerrain.transform.position;
                Vector3 size = activeTerrain.terrainData.size;
                if (worldPosition.x >= origin.x && worldPosition.x <= origin.x + size.x &&
                    worldPosition.z >= origin.z && worldPosition.z <= origin.z + size.z)
                {
                    height = activeTerrain.SampleHeight(worldPosition) + origin.y;
                    foundSurface = true;
                }
            }

            // Terrain sampling does not cover mesh ground, collider-only
            // terrain, or points just outside the TerrainData bounds. Use a
            // downward ray as a safe fallback so IK feet cannot pass through
            // the actual surface.
            Vector3 rayOrigin = worldPosition + Vector3.up * 2f;
            int hitCount = Physics.RaycastNonAlloc(
                rayOrigin, Vector3.down, groundHitBuffer, 6f,
                obstacleLayers, QueryTriggerInteraction.Ignore);
            for (int i = 0; i < hitCount; i++)
            {
                RaycastHit hit = groundHitBuffer[i];
                Collider hitCollider = hit.collider;
                if (hitCollider == null ||
                    hitCollider.transform.IsChildOf(transform) ||
                    IsSoftFoliageCollider(hitCollider))
                    continue;

                // Prefer the highest valid surface: on a climb this keeps a
                // foot from using a lower TerrainData sample when a mesh
                // collider or raised terrain surface is underneath it.
                if (!foundSurface || hit.point.y > height)
                    height = hit.point.y;
                foundSurface = true;
            }

            return foundSurface;
        }

        private void UpdateSideGait(
            LegChain[] side,
            Vector3 dexterInput,
            float forceMagnitudeNewtons,
            ref float phase,
            ref float weight,
            float responseBlend)
        {
            float inputActivity = dexterInput.magnitude;
            bool hasYDirection = Mathf.Abs(dexterInput.z) > gaitDirectionDeadZone;
            float activeTarget = hasYDirection && inputActivity > gaitActivationThreshold ? 1f : 0f;
            float gaitBlend = 1f - Mathf.Exp(-gaitBlendSpeed * Time.deltaTime);
            weight = Mathf.Lerp(weight, activeTarget, gaitBlend);

            if (activeTarget > 0f)
            {
                float forceStrength = Mathf.Clamp01(forceMagnitudeNewtons / forceForMaximumGaitSpeed);
                float cyclesPerSecond = Mathf.Lerp(
                    minimumGaitCyclesPerSecond,
                    maximumGaitCyclesPerSecond,
                    Mathf.Sqrt(forceStrength));
                float phaseDelta = cyclesPerSecond * Time.deltaTime;

                // +Y places the foot forward at the end of swing; -Y moves
                // it through stance to the rear placement. X contributes to
                // strength only after Y has supplied a movement direction.
                float targetPhase = dexterInput.z > 0f ? swingPhaseFraction : 0f;
                AdvancePhaseToward(ref phase, targetPhase, phaseDelta);
            }

            float strideStrength = Mathf.Clamp01(forceMagnitudeNewtons / forceForMaximumGaitSpeed);
            float strideLength = Mathf.Lerp(minimumStrideLength, maximumStrideLength, Mathf.Sqrt(strideStrength));
            UpdateSideDisplacements(
                side, dexterInput, phase, weight, strideLength, responseBlend, travelDirection);
        }

        private static void AdvancePhaseToward(ref float phase, float targetPhase, float maximumDelta)
        {
            float forwardDistance = Mathf.Repeat(targetPhase - phase, 1f);
            if (forwardDistance < 0.0001f)
            {
                phase = targetPhase;
                return;
            }

            phase = Mathf.Repeat(phase + Mathf.Min(maximumDelta, forwardDistance), 1f);
        }

        private void UpdateSideDisplacements(
            LegChain[] side,
            Vector3 dexterInput,
            float phase,
            float weight,
            float strideLength,
            float blend,
            int direction)
        {
            for (int row = 0; row < side.Length; row++)
            {
                LegChain leg = side[row];
                float groupOffset = row == 1 || row == 3 ? 0.5f : 0f;
                float legPhase = Mathf.Repeat(phase + groupOffset, 1f);
                float halfStride = strideLength * 0.5f;
                float foreAft;
                float lift;

                if (legPhase < swingPhaseFraction)
                {
                    float swingProgress = legPhase / swingPhaseFraction;
                    float easedSwing = SmoothStep01(swingProgress);
                    foreAft = Mathf.Lerp(-halfStride, halfStride, easedSwing);
                    lift = Mathf.Sin(easedSwing * Mathf.PI) * stepLiftHeight;
                }
                else
                {
                    // During stance, the foot moves backward relative to the body,
                    // representing a planted foot supporting forward travel.
                    float stanceProgress = (legPhase - swingPhaseFraction) / (1f - swingPhaseFraction);
                    foreAft = Mathf.Lerp(halfStride, -halfStride, SmoothStep01(stanceProgress));
                    lift = 0f;
                }

                var rowTarget = new Vector3(
                    0f,
                    0f,
                    foreAft * weight * direction);
                leg.TargetDisplacement = rowTarget;
                leg.CurrentDisplacement = Vector3.Lerp(leg.CurrentDisplacement, rowTarget, blend);
                leg.StepLift = lift * weight;
            }
        }

        private static float SmoothStep01(float value)
        {
            value = Mathf.Clamp01(value);
            return value * value * (3f - 2f * value);
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

            Vector3 rigForward = terrainForces != null
                ? terrainForces.GetSurfaceTravelDirection(
                    GetTerrainSamplingPosition(), GetRigForward())
                : GetRigForward();

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
            locomotionWorldPosition = transform.position;
            hasLocomotionWorldPosition = true;
            hasClimbSurfaceAnchor = false;
            lastAppliedForwardDistance = currentForwardDistance;
            if (TrySampleTerrainHeight(transform.position, out float terrainHeight))
            {
                terrainRootClearance = transform.position.y - terrainHeight;
                hasTerrainRootClearance = true;
            }
            else
            {
                hasTerrainRootClearance = false;
            }
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
            if (legs == null)
                return;

            for (int i = 0; i < legs.Length; i++)
            {
                if (legs[i] != null)
                    RestoreLegPose(legs[i]);
            }
        }

        private void SolveLegsWithBalancePosture(
            LegChain[] legs,
            bool isLeftSide,
            float cadenceImbalance,
            float directFingerLift,
            float pushDistance)
        {
            float imbalance = Mathf.Clamp(cadenceImbalance, -1f, 1f);
            bool thisSideIsFaster = isLeftSide
                ? imbalance < 0f
                : imbalance > 0f;
            float strength = Mathf.Abs(imbalance);
            float radialAmount = (thisSideIsFaster
                ? -balanceLegBend
                : balanceLegExtension) * strength;

            for (int i = 0; i < legs.Length; i++)
            {
                LegChain leg = legs[i];
                Vector3 supportNormal = Vector3.up;
                if (TryGetActiveClimbNormal(out Vector3 activeClimbNormal))
                {
                    supportNormal = activeClimbNormal;
                }
                else if (terrainForces != null &&
                    terrainForces.TryGetClimbSurfaceContact(
                        leg.WorldTarget, leg.GroundClearance,
                        out _, out Vector3 climbNormal))
                {
                    supportNormal = climbNormal;
                }
                Vector3 outward = Vector3.ProjectOnPlane(
                    leg.WorldTarget - body.position, supportNormal).normalized;
                if (outward.sqrMagnitude < 0.0001f)
                {
                    Vector3 rigRight = Vector3.ProjectOnPlane(
                        rigSpace.right, supportNormal).normalized;
                    outward = isLeftSide ? -rigRight : rigRight;
                }
                Vector3 rigForward = terrainForces != null
                    ? terrainForces.GetSurfaceTravelDirection(
                        GetTerrainSamplingPosition(), GetRigForward())
                    : Vector3.ProjectOnPlane(
                        rigSpace.forward, supportNormal).normalized;
                bool directSideWave = isLeftSide
                    ? leftSideWaveInPlace
                    : rightSideWaveInPlace;
                // During locomotion the planted WorldTarget is the friction
                // contact and must not move. The body travelling past that
                // fixed point creates the visible backward power stroke. Keep
                // the explicit backward offset only for the single-finger,
                // no-locomotion preparation gesture.
                float stancePush = directSideWave ? pushDistance : 0f;
                bool shouldPlantFoot = directFingerLift <= 0.001f;
                if (!leftSideWaveInPlace && !rightSideWaveInPlace)
                {
                    float phase = isLeftSide
                        ? leftGaitPhase
                        : rightGaitPhase;
                    float gaitWeight = isLeftSide ? leftGaitWeight : rightGaitWeight;
                    float stancePhase = GetTetrapodLegPhase(
                        phase, isLeftSide, i);
                    bool planted = stancePhase >= swingPhaseFraction;
                    shouldPlantFoot &= gaitWeight <= 0.01f || planted;
                }
                Vector3 pushOffset = -rigForward * stancePush;
                Vector3 solvedTarget = leg.WorldTarget + outward * radialAmount +
                                       supportNormal * directFingerLift *
                                       (i == 0 ? frontLegMimicLiftMultiplier : 1f) *
                                       1f +
                                       pushOffset;
                solvedTarget = ConstrainFootAgainstObstacles(
                    leg, solvedTarget, obstacleSkin);
                if (TryGetFootSurfaceTarget(
                        solvedTarget, leg.GroundClearance,
                        out Vector3 surfaceTarget, out Vector3 surfaceNormal))
                {
                    float penetration = Vector3.Dot(
                        surfaceTarget - solvedTarget, surfaceNormal);
                    if (shouldPlantFoot)
                        solvedTarget = surfaceTarget;
                    else if (penetration > 0f)
                        solvedTarget += surfaceNormal * penetration;
                }
                SolveCcd(leg, solvedTarget);
                if (TryGetFootSurfaceTarget(
                        leg.Effector.position, leg.GroundClearance,
                        out Vector3 solvedContact, out Vector3 solvedNormal))
                {
                    float penetration = Vector3.Dot(
                        solvedContact - leg.Effector.position, solvedNormal);
                    if (penetration > 0.001f)
                    {
                        // CCD can stop short when a climbing leg reaches its
                        // joint-angle limit. Re-solve from the measured foot
                        // position rather than translating the chain, which
                        // preserves leg lengths and prevents warping.
                        SolveCcd(leg,
                            solvedTarget + solvedNormal * penetration);
                    }
                }
            }
        }

        private Vector3 ConstrainFootAgainstObstacles(
            LegChain leg, Vector3 desired, float skin)
        {
            if (leg == null || leg.Effector == null)
                return desired;

            Vector3 start = leg.Effector.position;
            Vector3 delta = desired - start;
            float distance = delta.magnitude;
            if (distance < 0.001f)
                return desired;

            float radius = Mathf.Max(0.025f, skin * 1.5f);
            RaycastHit[] hits = Physics.SphereCastAll(
                start, radius, delta / distance, distance + skin,
                obstacleLayers, QueryTriggerInteraction.Ignore);
            float permitted = distance;
            Vector3 normal = Vector3.zero;
            for (int i = 0; i < hits.Length; i++)
            {
                Collider collider = hits[i].collider;
                if (collider == null || collider.transform.IsChildOf(transform) ||
                    IsSoftFoliageCollider(collider) ||
                    IsWalkableTerrainContact(hits[i]))
                    continue;
                if (hits[i].distance < permitted)
                {
                    permitted = Mathf.Max(0f, hits[i].distance - skin);
                    normal = hits[i].normal;
                }
            }

            if (permitted >= distance || normal.sqrMagnitude < 0.0001f)
                return desired;
            return start + delta.normalized * permitted + normal.normalized * skin;
        }

        private float GetTargetLegPushDistance(
            float yDisplacement,
            float gaitWeight,
            bool isDirectFingerGesture)
        {
            float forceStrength = Mathf.InverseLerp(
                GetActiveMinimumSpeedDisplacement(),
                GetActiveMaximumSpeedDisplacement(),
                Mathf.Max(0f, yDisplacement));
            float distance = Mathf.Lerp(
                minimumLegPushDistance,
                maximumLegPushDistance,
                forceStrength);
            if (isDirectFingerGesture)
            {
                // Let the foot lift first, then draw it backward into a
                // push-ready position. Releasing the finger drives this back
                // to zero through the existing response smoothing.
                float pushPreparation = Mathf.InverseLerp(0.25f, 1f,
                    forceStrength);
                return distance * pushPreparation;
            }

            return distance * gaitWeight;
        }

        private float GetPlantedPushDriveStrength()
        {
            if (!alternatingDriveAuthorized)
                return 0f;

            float activeWeight = leftGaitWeight + rightGaitWeight;
            float plantedPush = activeWeight > 0.01f
                ? (GetSidePlantedPush(leftGaitPhase, true) * leftGaitWeight +
                   GetSidePlantedPush(rightGaitPhase, false) *
                   rightGaitWeight) / activeWeight
                : 0f;
            plantedPush = 1f - Mathf.Pow(
                1f - Mathf.Clamp01(plantedPush), walkingPushEmphasis);
            return terrainForces != null
                ? terrainForces.GetWalkingTractionMultiplier(plantedPush)
                : Mathf.Lerp(0.35f, 1f, plantedPush);
        }

        private float GetSidePlantedPush(float phase, bool isLeftSide)
        {
            float loadedPush = 0f;
            for (int row = 0; row < 4; row++)
            {
                float legPhase = GetTetrapodLegPhase(
                    phase, isLeftSide, row);
                if (legPhase < swingPhaseFraction)
                    continue;

                float stanceProgress = Mathf.InverseLerp(
                    swingPhaseFraction, 1f, legPhase);
                loadedPush += Mathf.Sin(stanceProgress * Mathf.PI);
            }

            // A tetrapod group contains two legs on each side. Normalize those
            // two loaded feet to full support while preserving the handoff dip
            // that makes the plant-and-push rhythm visible in body motion.
            return Mathf.Clamp01(loadedPush * 0.5f);
        }

        private static float GetTetrapodLegPhase(
            float synchronizedPhase,
            bool isLeftSide,
            int row)
        {
            // The mirrored side is always half a cycle behind. Rows 2 and 4
            // receive the other half-cycle offset, forming the two groups:
            // L1/L3 + R2/R4, then R1/R3 + L2/L4.
            float sideOffset = isLeftSide ? 0f : 0.5f;
            float rowOffset = row == 1 || row == 3 ? 0.5f : 0f;
            return Mathf.Repeat(
                synchronizedPhase + sideOffset + rowOffset, 1f);
        }

        private static void ResetWorldFootTargets(LegChain[] legs)
        {
            if (legs == null)
                return;

            for (int i = 0; i < legs.Length; i++)
            {
                LegChain leg = legs[i];
                leg.WorldTarget = leg.RestWorldTarget;
                leg.SwingStartWorldTarget = leg.RestWorldTarget;
                leg.SwingEndWorldTarget = leg.RestWorldTarget;
                leg.WasSwinging = false;
                leg.IsRecoveringLag = false;
                leg.LagRecoveryProgress = 0f;
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
                Gizmos.DrawWireSphere(leg.WorldTarget, 0.03f);
            }
        }

        private void OnValidate()
        {
            leftLegFinger = DexterFinger.Index;
            rightLegFinger = DexterFinger.Middle;
            ApplyResponsiveMovementDefaults();
            calibrationDurationSeconds = Mathf.Max(0.25f, calibrationDurationSeconds);
            ipadCalibrationReferencePercentile = Mathf.Clamp(
                ipadCalibrationReferencePercentile, 0.25f, 0.8f);
            minimumIpadCalibrationDisplacement = Mathf.Max(
                0.01f, minimumIpadCalibrationDisplacement);
            ipadMaximumSpeedDisplacementMultiplier = Mathf.Clamp(
                ipadMaximumSpeedDisplacementMultiplier, 1.05f, 6f);
            displacementPerNewton.x = Mathf.Max(0f, displacementPerNewton.x);
            displacementPerNewton.y = Mathf.Max(0f, displacementPerNewton.y);
            forceDeadZone = Mathf.Max(0f, forceDeadZone);
            maximumDisplacement = Mathf.Max(0.01f, maximumDisplacement);
            responseSpeed = Mathf.Max(0.01f, responseSpeed);
            temporalSmoothingWindow = Mathf.Max(0f, temporalSmoothingWindow);
            maximumOutwardMovement = Mathf.Max(0f, maximumOutwardMovement);
            maximumInwardMovement = Mathf.Max(0f, maximumInwardMovement);
            maximumForeAftMovement = Mathf.Max(0f, maximumForeAftMovement);
            rowLaneFraction = Mathf.Clamp(rowLaneFraction, 0.1f, 0.49f);
            minimumRestReachRatio = Mathf.Clamp(minimumRestReachRatio, 0.5f, 1f);
            maximumRestReachRatio = Mathf.Clamp(maximumRestReachRatio, 1f, 1.25f);
            maximumPlantedFootLag = Mathf.Max(0.05f, maximumPlantedFootLag);
            lagRecoveryDuration = Mathf.Max(0.05f, lagRecoveryDuration);
            balanceLegExtension = Mathf.Max(0f, balanceLegExtension);
            balanceLegBend = Mathf.Max(0f, balanceLegBend);
            fingerMimicLiftHeight = Mathf.Max(0f, fingerMimicLiftHeight);
            fingerMimicResponse = Mathf.Max(0.01f, fingerMimicResponse);
            frontLegMimicLiftMultiplier = Mathf.Clamp(
                frontLegMimicLiftMultiplier, 1f, 3f);
            fingerLiftBalanceInfluence = Mathf.Clamp(
                fingerLiftBalanceInfluence, 0f, 2f);
            fingerLiftSupportLoss = Mathf.Clamp01(fingerLiftSupportLoss);
            minimumLegPushDistance = Mathf.Max(0f, minimumLegPushDistance);
            maximumLegPushDistance = Mathf.Max(
                minimumLegPushDistance, maximumLegPushDistance);
            legPushResponse = Mathf.Max(0.01f, legPushResponse);
            cadenceForFullSupport = Mathf.Max(0.1f, cadenceForFullSupport);
            balancePostureResponse = Mathf.Max(0.01f, balancePostureResponse);
            stepLiftHeight = Mathf.Max(0f, stepLiftHeight);
            gaitActivationThreshold = Mathf.Max(0f, gaitActivationThreshold);
            minimumStrideLength = Mathf.Max(0f, minimumStrideLength);
            maximumStrideLength = Mathf.Max(minimumStrideLength, maximumStrideLength);
            minimumGaitCyclesPerSecond = Mathf.Max(0.01f, minimumGaitCyclesPerSecond);
            maximumGaitCyclesPerSecond = Mathf.Max(minimumGaitCyclesPerSecond, maximumGaitCyclesPerSecond);
            forceForMaximumGaitSpeed = Mathf.Max(0.01f, forceForMaximumGaitSpeed);
            gaitDirectionDeadZone = Mathf.Max(0f, gaitDirectionDeadZone);
            swingPhaseFraction = Mathf.Clamp(swingPhaseFraction, 0.2f, 0.6f);
            gaitBlendSpeed = Mathf.Max(0.01f, gaitBlendSpeed);
            walkingLiftEmphasis = Mathf.Clamp(walkingLiftEmphasis, 1f, 3f);
            walkingStrideEmphasis = Mathf.Clamp(walkingStrideEmphasis, 1f, 2f);
            walkingPushEmphasis = Mathf.Clamp(walkingPushEmphasis, 1f, 3f);
            alternationForceThreshold = Mathf.Max(0f, alternationForceThreshold);
            alternationDominanceMargin = Mathf.Max(0f, alternationDominanceMargin);
            alternationReleaseThreshold = Mathf.Clamp(
                alternationReleaseThreshold, 0f, alternationForceThreshold);
            noiseStandardDeviationMultiplier = Mathf.Clamp(noiseStandardDeviationMultiplier, 2f, 8f);
            maximumAdaptiveNoiseThreshold = Mathf.Max(
                alternationForceThreshold, maximumAdaptiveNoiseThreshold);
            normalForceTravelSpeed = Mathf.Max(0f, normalForceTravelSpeed);
            inputDriveReleaseDelay = Mathf.Max(0.05f, inputDriveReleaseDelay);
            minimumPressInterval = Mathf.Max(0f, minimumPressInterval);
            maximumAlternationInterval = Mathf.Max(
                minimumPressInterval, maximumAlternationInterval);
            alternatingDriveSustainTime = Mathf.Max(
                maximumAlternationInterval, alternatingDriveSustainTime);
            locomotionSmoothTime = Mathf.Max(0.01f, locomotionSmoothTime);
            directionChangeForceThreshold = Mathf.Max(0f, directionChangeForceThreshold);
            directionChangeHoldTime = Mathf.Max(0f, directionChangeHoldTime);
            overallMovementSpeedMultiplier = Mathf.Clamp(
                overallMovementSpeedMultiplier, 0.5f, 2f);
            referenceHalfStepDuration = Mathf.Max(0.05f, referenceHalfStepDuration);
            minimumSpeedDisplacement = Mathf.Max(0.001f, minimumSpeedDisplacement);
            maximumSpeedDisplacement = Mathf.Max(
                minimumSpeedDisplacement + 0.001f, maximumSpeedDisplacement);
            minimumForceStrideLength = Mathf.Max(0f, minimumForceStrideLength);
            maximumForceStrideLength = Mathf.Max(
                minimumForceStrideLength, maximumForceStrideLength);
            minimumForceLiftMultiplier = Mathf.Clamp(
                minimumForceLiftMultiplier, 0.1f, 1f);
            maximumForceLiftMultiplier = Mathf.Clamp(
                maximumForceLiftMultiplier, 1f, 2f);
            minimumStepSpeedMultiplier = Mathf.Clamp(
                minimumStepSpeedMultiplier, 0.1f, 2f);
            maximumStepSpeedMultiplier = Mathf.Clamp(
                maximumStepSpeedMultiplier, 1f, 6f);
            turnActivationForce = Mathf.Max(0f, turnActivationForce);
            turnAxisDominanceRatio = Mathf.Clamp01(turnAxisDominanceRatio);
            turnMinimumSpeedDisplacement = Mathf.Max(
                turnActivationForce, turnMinimumSpeedDisplacement);
            turnForceForMaximumSpeed = Mathf.Max(
                turnMinimumSpeedDisplacement + 0.001f,
                turnForceForMaximumSpeed);
            turnForceForMaximumSpeed = Mathf.Max(
                turnActivationForce + 0.01f, turnForceForMaximumSpeed);
            minimumTurnDegreesPerSecond = Mathf.Max(0f, minimumTurnDegreesPerSecond);
            maximumTurnDegreesPerSecond = Mathf.Max(
                minimumTurnDegreesPerSecond, maximumTurnDegreesPerSecond);
            turnResponseSpeed = Mathf.Max(0.01f, turnResponseSpeed);
            outsideTurnStrideMultiplier = Mathf.Max(0f, outsideTurnStrideMultiplier);
            insideTurnStrideMultiplier = Mathf.Max(0f, insideTurnStrideMultiplier);
            turnRecoveryDuration = Mathf.Max(0.1f, turnRecoveryDuration);
            bodyCollisionRadius = Mathf.Max(0.05f, bodyCollisionRadius);
            bodyCollisionHeight = Mathf.Max(0f, bodyCollisionHeight);
            obstacleSkin = Mathf.Max(0f, obstacleSkin);
            baseJointLeadDegrees = Mathf.Clamp(baseJointLeadDegrees, 0f, 30f);
            maximumBaseDeviationDegrees = Mathf.Clamp(maximumBaseDeviationDegrees, 1f, 120f);
            maximumJointDeviationDegrees = Mathf.Clamp(maximumJointDeviationDegrees, 1f, 120f);
            solverIterations = Mathf.Clamp(solverIterations, 1, 32);
            positionTolerance = Mathf.Max(0.00001f, positionTolerance);
        }

        private void ApplyResponsiveMovementDefaults()
        {
            displacementPerNewton.x = Mathf.Max(1.25f, displacementPerNewton.x);
            displacementPerNewton.y = Mathf.Max(1.25f, displacementPerNewton.y);
            maximumDisplacement = Mathf.Max(0.75f, maximumDisplacement);
            maximumOutwardMovement = Mathf.Max(0.55f, maximumOutwardMovement);
            maximumInwardMovement = Mathf.Max(0.10f, maximumInwardMovement);
            maximumForeAftMovement = Mathf.Max(0.65f, maximumForeAftMovement);
            baseJointLeadDegrees = Mathf.Max(16f, baseJointLeadDegrees);
        }
    }
}
