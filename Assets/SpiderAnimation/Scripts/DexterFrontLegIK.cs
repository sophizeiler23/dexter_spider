using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using Dexter.Visualize;
using UnityEngine;

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
        [SerializeField, Min(0.25f)] private float calibrationDurationSeconds = 5f;

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
        [SerializeField, Range(0.2f, 0.6f)] private float swingPhaseFraction = 0.40f;
        [Tooltip("How quickly walking fades in and out as finger movement starts and stops.")]
        [SerializeField, Min(0.01f)] private float gaitBlendSpeed = 8f;

        [Header("Alternating Finger Locomotion")]
        [Tooltip("Minimum forward/back force in Newtons for a finger to register. Kept low for small finger forces.")]
        [SerializeField, Min(0f)] private float alternationForceThreshold = 0.012f;
        [Tooltip("Minimum Y-force lead over the other finger before an alternation registers. Prevents noise from rapidly swapping the active finger.")]
        [SerializeField, Min(0f)] private float alternationDominanceMargin = 0.012f;
        [Tooltip("Both fingers must return below this Y force before another press can register.")]
        [SerializeField, Min(0f)] private float alternationReleaseThreshold = 0.008f;
        [Tooltip("Tare noise standard deviations used to derive a separate threshold for each finger.")]
        [SerializeField, Range(2f, 8f)] private float noiseStandardDeviationMultiplier = 4f;
        [Tooltip("Maximum adaptive activation threshold, preserving sensitivity even with a noisy sensor.")]
        [SerializeField, Min(0.01f)] private float maximumAdaptiveNoiseThreshold = 0.03f;
        [Tooltip("How much the spider advances whenever the active finger changes Index <-> Middle.")]
        [SerializeField, Min(0f)] private float distancePerAlternation = 0.30f;
        [Tooltip("Shortest accepted time between finger changes. Rejects one-frame dominance noise without making deliberate alternation feel delayed.")]
        [SerializeField, Min(0f)] private float minimumPressInterval = 0.08f;
        [Tooltip("Time used to smooth each discrete forward movement.")]
        [SerializeField, Min(0.01f)] private float locomotionSmoothTime = 0.10f;
        [Tooltip("Combined Y force required to request a change of travel direction.")]
        [SerializeField, Min(0f)] private float directionChangeForceThreshold = 0.18f;
        [Tooltip("How long the reverse-direction force must be held before direction changes.")]
        [SerializeField, Min(0f)] private float directionChangeHoldTime = 0.30f;
        [Tooltip("Fastest time allowed for one finger-triggered half step.")]
        [SerializeField, Min(0.05f)] private float minimumStepDuration = 0.25f;
        [Tooltip("Slowest time allowed for one finger-triggered half step.")]
        [SerializeField, Min(0.05f)] private float maximumStepDuration = 0.45f;

        [Header("Two-Finger Turning")]
        [Tooltip("Each finger must exceed this signed X force in the same direction to turn.")]
        [SerializeField, Min(0f)] private float turnActivationForce = 0.08f;
        [Tooltip("X force at which turning reaches its maximum speed.")]
        [SerializeField, Min(0.01f)] private float turnForceForMaximumSpeed = 0.75f;
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

        [Header("Terrain Movement")]
        [Tooltip("Walking speed retained on the steepest traversable terrain.")]
        [SerializeField, Range(0.1f, 1f)] private float steepSlopeSpeedMultiplier = 0.45f;
        [SerializeField, Range(1f, 70f)] private float steepSlopeAngle = 38f;
        [Tooltip("Walking speed retained while moving through terrain detail grass.")]
        [SerializeField, Range(0.1f, 1f)] private float grassSpeedMultiplier = 0.90f;

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
        private float forwardDistanceVelocity;
        private float lastAlternationTime = -1f;
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
        private bool wasTurnGestureActive;
        private bool isRecoveringFromTurn;
        private float turnRecoveryElapsed;
        private SpiderTerrainForces terrainForces;
        private float cachedVegetationMultiplier = 1f;
        private float nextVegetationSampleTime;
        private float leftAdaptiveActivationThreshold;
        private float rightAdaptiveActivationThreshold;
        private float leftAdaptiveReleaseThreshold;
        private float rightAdaptiveReleaseThreshold;
        private Terrain activeTerrain;
        private float terrainRootClearance;
        private bool hasTerrainRootClearance;
        private StreamWriter traceWriter;
        private long lastTracedSequence = long.MinValue;
        private int unflushedTraceRows;
        private string traceFilePath;

        public bool IsReceiving => receiver != null && receiver.HasRecentFrame;
        public bool IsTaring => isTaring;
        public float ForwardSpeed => forwardSpeed;
        public string TraceFilePath => traceFilePath;

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
            else
                UseUntaredRelayDefaults();
            // Live CSV recording is intentionally disabled. Pre-recorded Dexter
            // force files can be replayed without creating a new trace per run.
        }

        private void UseUntaredRelayDefaults()
        {
            isTaring = false;
            hasBaseline = false;
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
            if (body != null)
            {
                body.localPosition = bodyLocalPosition;
                body.localRotation = bodyLocalRotation;
            }
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

            float cadenceScale = Mathf.Max(
                0.25f, Mathf.Max(leftFingerCadence, rightFingerCadence));
            float targetCadenceImbalance =
                (leftFingerCadence - rightFingerCadence) / cadenceScale;
            currentCadenceImbalance = Mathf.MoveTowards(
                currentCadenceImbalance,
                targetCadenceImbalance,
                balancePostureResponse * Time.deltaTime);

            SolveLegsWithBalancePosture(
                leftLegs, true, currentCadenceImbalance);
            SolveLegsWithBalancePosture(
                rightLegs, false, currentCadenceImbalance);

            bool bodyIsMoving = Mathf.Abs(forwardSpeed) > 0.01f ||
                                Mathf.Abs(currentTurnSpeed) > 0.5f ||
                                leftGaitWeight > 0.05f ||
                                rightGaitWeight > 0.05f ||
                                turnGaitWeight > 0.05f;
            terrainForces?.ApplyBodyWeightAfterIk(
                body,
                bodyLocalPosition,
                bodyLocalRotation,
                bodyIsMoving,
                currentCadenceImbalance,
                rigSpace.right,
                rigSpace.forward);
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
            row.Append("spider_x,spider_y,spider_z");
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
            }
        }

        private void WriteDiagnosticTraceRow()
        {
            DexterForceFrame frame = receiver != null ? receiver.LatestFrame : null;
            if (traceWriter == null || frame == null || frame.sequence == lastTracedSequence)
                return;

            lastTracedSequence = frame.sequence;
            var row = new StringBuilder(4096);
            row.Append(Time.realtimeSinceStartupAsDouble.ToString("R", CultureInfo.InvariantCulture));
            AppendNumber(row, Time.timeAsDouble);
            AppendInteger(row, Time.frameCount);
            AppendInteger(row, frame.sequence);
            AppendNumber(row, receiver.LastFrameAge);
            AppendBoolean(row, receiver.HasRecentFrame);
            AppendBoolean(row, isTaring);

            DexterFingerMeasurement thumb = receiver.GetFinger(DexterFinger.Thumb);
            DexterFingerMeasurement index = receiver.GetFinger(DexterFinger.Index);
            DexterFingerMeasurement middle = receiver.GetFinger(DexterFinger.Middle);
            DexterFingerMeasurement ring = receiver.GetFinger(DexterFinger.Ring);
            DexterFingerMeasurement pinky = receiver.GetFinger(DexterFinger.Pinky);
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

        private static void AppendFeet(StringBuilder row, LegChain[] legs)
        {
            if (legs == null)
                return;
            for (int i = 0; i < legs.Length; i++)
            {
                AppendVector3(row, legs[i].Effector.position);
                AppendVector3(row, legs[i].WorldTarget);
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
            targetForwardDistance = 0f;
            currentForwardDistance = 0f;
            forwardDistanceVelocity = 0f;
            lastAlternationTime = -1f;
            forwardSpeed = 0f;
            turnInput = 0f;
            turnGaitPhase = 0f;
            turnGaitWeight = 0f;
            currentTurnSpeed = 0f;
            currentYawDegrees = 0f;
            lastAppliedForwardDistance = 0f;
            locomotionWorldPosition = transform.position;
            hasLocomotionWorldPosition = true;
            wasTurnGestureActive = false;
            isRecoveringFromTurn = false;
            turnRecoveryElapsed = 0f;
            terrainForces?.ResetForces();
            cachedVegetationMultiplier = 1f;
            nextVegetationSampleTime = 0f;
            travelDirection = 1;
            pendingTravelDirection = 0;
            pendingDirectionStartTime = -1f;
            leftGaitPhase = 0f;
            rightGaitPhase = 0f;
            targetLeftGaitPhase = 0f;
            targetRightGaitPhase = 0f;
            gaitPhaseSpeed = 0.5f / 0.35f;
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
            if (terrainForces == null)
                terrainForces = gameObject.AddComponent<SpiderTerrainForces>();

            activeTerrain = Terrain.activeTerrain;
            if (activeTerrain == null)
                activeTerrain = FindFirstObjectByType<Terrain>();

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
                GroundClearance = GetGroundClearance(effector.position)
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

            if (receiver.HasRecentFrame && !IsTaring)
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

            // Keep body travel synchronized with the half-step without replacing
            // the authored alternating gait with a separate all-leg push phase.
            float bodyMovementSpeed = Mathf.Max(
                0.01f,
                distancePerAlternation * gaitPhaseSpeed * 2f);
            currentForwardDistance = Mathf.MoveTowards(
                currentForwardDistance,
                targetForwardDistance,
                bodyMovementSpeed * Time.deltaTime);

            if (lastAlternationTime >= 0f && Time.time - lastAlternationTime > 0.75f)
                forwardSpeed = Mathf.MoveTowards(forwardSpeed, 0f, Time.deltaTime * 4f);

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
            if (activeFinger == 1)
            {
                if (lastLeftFingerPressTime >= 0f)
                {
                    float cadence = 1f / Mathf.Max(
                        0.08f, now - lastLeftFingerPressTime);
                    leftFingerCadence = Mathf.Lerp(
                        leftFingerCadence, cadence, 0.5f);
                }
                else
                {
                    leftFingerCadence = gaitPhaseSpeed;
                }
                lastLeftFingerPressTime = now;
            }
            else
            {
                if (lastRightFingerPressTime >= 0f)
                {
                    float cadence = 1f / Mathf.Max(
                        0.08f, now - lastRightFingerPressTime);
                    rightFingerCadence = Mathf.Lerp(
                        rightFingerCadence, cadence, 0.5f);
                }
                else
                {
                    rightFingerCadence = gaitPhaseSpeed;
                }
                lastRightFingerPressTime = now;
            }
            if (lastFingerPressTime >= 0f)
            {
                float pressInterval = Mathf.Max(0.001f, now - lastFingerPressTime);
                float stepDuration = Mathf.Clamp(
                    pressInterval * 0.6f,
                    minimumStepDuration,
                    maximumStepDuration);
                gaitPhaseSpeed = 0.5f / stepDuration;
            }
            else
            {
                gaitPhaseSpeed = 0.5f / Mathf.Lerp(
                    minimumStepDuration, maximumStepDuration, 0.5f);
            }

            // A single finger always moves only its own side, with no body travel.
            if (activeFinger == 1)
                targetLeftGaitPhase = Mathf.Min(
                    targetLeftGaitPhase + 0.5f,
                    leftGaitPhase + 0.5f);
            else
                targetRightGaitPhase = Mathf.Min(
                    targetRightGaitPhase + 0.5f,
                    rightGaitPhase + 0.5f);

            // Body travel is earned only by a genuine Index <-> Middle alternation.
            if (lastActiveFinger != 0 && activeFinger != lastActiveFinger)
            {
                targetForwardDistance += distancePerAlternation;
                if (lastAlternationTime >= 0f)
                {
                    float interval = Mathf.Max(0.001f, now - lastAlternationTime);
                    forwardSpeed = distancePerAlternation / interval;
                }
                lastAlternationTime = now;
            }

            lastActiveFinger = activeFinger;
            lastFingerPressTime = now;
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

            bool leftIsMoving = leftGaitPhase < targetLeftGaitPhase - 0.0001f;
            bool rightIsMoving = rightGaitPhase < targetRightGaitPhase - 0.0001f;
            if (leftIsMoving)
                leftGaitPhase = Mathf.MoveTowards(
                    leftGaitPhase, targetLeftGaitPhase, gaitPhaseSpeed * Time.deltaTime);
            if (rightIsMoving)
                rightGaitPhase = Mathf.MoveTowards(
                    rightGaitPhase, targetRightGaitPhase, gaitPhaseSpeed * Time.deltaTime);

            float gaitBlend = 1f - Mathf.Exp(-gaitBlendSpeed * Time.deltaTime);
            leftGaitWeight = Mathf.Lerp(leftGaitWeight, leftIsMoving ? 1f : 0f, gaitBlend);
            rightGaitWeight = Mathf.Lerp(rightGaitWeight, rightIsMoving ? 1f : 0f, gaitBlend);

            float strideLength = Mathf.Clamp(
                Mathf.Max(minimumStrideLength, distancePerAlternation),
                minimumStrideLength,
                maximumStrideLength);
            UpdateSideWorldTargets(
                leftLegs, Mathf.Repeat(leftGaitPhase, 1f),
                leftGaitWeight, strideLength, 1);
            // Tetrapod gait: the right side is half a cycle out of phase.
            // Left front + third move with right second + back, followed by
            // the complementary four-leg group.
            UpdateSideWorldTargets(
                rightLegs, Mathf.Repeat(rightGaitPhase + 0.5f, 1f),
                rightGaitWeight, strideLength, 1);
        }

        private bool UpdateTurning(Vector2 leftForce, Vector2 rightForce)
        {
            float xSign = invertFx ? -1f : 1f;
            float leftX = leftForce.x * xSign;
            float rightX = rightForce.x * xSign;
            float requestedTurn = 0f;

            if (leftX >= turnActivationForce && rightX >= turnActivationForce)
                requestedTurn = Mathf.Min(leftX, rightX);
            else if (leftX <= -turnActivationForce && rightX <= -turnActivationForce)
                requestedTurn = -Mathf.Min(-leftX, -rightX);

            float blend = 1f - Mathf.Exp(-turnResponseSpeed * Time.deltaTime);
            turnInput = Mathf.Lerp(turnInput, requestedTurn, blend);
            float magnitude = Mathf.Abs(turnInput);
            float strength = Mathf.InverseLerp(
                turnActivationForce, turnForceForMaximumSpeed, magnitude);
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
                turnActivationForce,
                turnForceForMaximumSpeed,
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
                leftLegs, turnGaitPhase, turnGaitWeight, leftStride, direction);
            UpdateSideWorldTargets(
                rightLegs, Mathf.Repeat(turnGaitPhase + 0.5f, 1f),
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
            for (int i = 0; i < legs.Length; i++)
            {
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
            float doubledProgress = turnRecoveryElapsed /
                                    Mathf.Max(0.1f, turnRecoveryDuration) * 2f;
            for (int row = 0; row < legs.Length; row++)
            {
                LegChain leg = legs[row];
                // Recover in the same diagonal tetrapod groups used for walking.
                bool firstGroup = isLeftSide ? row % 2 == 0 : row % 2 != 0;
                float progress = Mathf.Clamp01(
                    doubledProgress - (firstGroup ? 0f : 1f));
                float eased = SmoothStep01(progress);
                Vector3 naturalLanding = rigSpace.TransformPoint(leg.RestRigLocalTarget);
                if (TrySampleTerrainHeight(naturalLanding, out float terrainHeight))
                    naturalLanding.y = terrainHeight + leg.GroundClearance;
                else
                    naturalLanding.y = leg.RestWorldTarget.y;

                leg.WorldTarget = Vector3.Lerp(
                    leg.SwingStartWorldTarget, naturalLanding, eased);
                leg.WorldTarget.y += Mathf.Sin(progress * Mathf.PI) *
                                     stepLiftHeight * 0.45f;
                leg.SwingEndWorldTarget = naturalLanding;
            }
        }

        private void UpdateSideWorldTargets(
            LegChain[] side,
            float phase,
            float weight,
            float strideLength,
            int direction)
        {
            Vector3 rigForward = GetRigForward();
            for (int row = 0; row < side.Length; row++)
            {
                LegChain leg = side[row];
                float groupOffset = row == 1 || row == 3 ? 0.5f : 0f;
                float legPhase = Mathf.Repeat(phase + groupOffset, 1f);
                bool isSwinging = weight > 0.01f && legPhase < swingPhaseFraction;

                if (isSwinging && !leg.WasSwinging)
                {
                    leg.SwingStartWorldTarget = leg.WorldTarget;
                    Vector3 naturalLanding = rigSpace.TransformPoint(leg.RestRigLocalTarget);
                    if (TrySampleTerrainHeight(naturalLanding, out float terrainHeight))
                        naturalLanding.y = terrainHeight + leg.GroundClearance;
                    else
                        naturalLanding.y = leg.RestWorldTarget.y;
                    leg.SwingEndWorldTarget = naturalLanding
                        + rigForward * (strideLength * 0.5f * direction);
                }

                if (isSwinging)
                {
                    leg.IsRecoveringLag = false;
                    float swingProgress = SmoothStep01(legPhase / swingPhaseFraction);
                    leg.WorldTarget = Vector3.Lerp(
                        leg.SwingStartWorldTarget,
                        leg.SwingEndWorldTarget,
                        swingProgress);
                    leg.WorldTarget.y = Mathf.Lerp(
                        leg.SwingStartWorldTarget.y,
                        leg.SwingEndWorldTarget.y,
                        swingProgress);
                    leg.WorldTarget.y += Mathf.Sin(swingProgress * Mathf.PI) * stepLiftHeight;
                }
                else if (leg.WasSwinging)
                {
                    // Once planted, this point remains fixed in world space while
                    // the body moves, producing the visible stance/pull motion.
                    leg.WorldTarget = leg.SwingEndWorldTarget;
                }

                if (!isSwinging)
                    UpdateLagRecovery(leg);

                // A planted world-space target may encounter rising terrain as
                // the body advances. Never allow the solved foot below the local
                // surface while it waits for its next swing.
                if (TrySampleTerrainHeight(leg.WorldTarget, out float footTerrainHeight))
                    leg.WorldTarget.y = Mathf.Max(
                        leg.WorldTarget.y,
                        footTerrainHeight + leg.GroundClearance);

                leg.WasSwinging = isSwinging;
            }
        }

        private void UpdateLagRecovery(LegChain leg)
        {
            Vector3 naturalLanding = rigSpace.TransformPoint(leg.RestRigLocalTarget);
            if (TrySampleTerrainHeight(naturalLanding, out float terrainHeight))
                naturalLanding.y = terrainHeight + leg.GroundClearance;
            else
                naturalLanding.y = leg.RestWorldTarget.y;

            Vector3 horizontalLag = Vector3.ProjectOnPlane(
                naturalLanding - leg.WorldTarget, Vector3.up);
            if (!leg.IsRecoveringLag && horizontalLag.magnitude > maximumPlantedFootLag)
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
            leg.WorldTarget.y += Mathf.Sin(leg.LagRecoveryProgress * Mathf.PI) *
                                 stepLiftHeight * 0.70f;
            leg.SwingEndWorldTarget = naturalLanding;

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
            float terrainMovementMultiplier = GetTerrainMovementMultiplier(
                locomotionWorldPosition);
            locomotionWorldPosition += GetRigForward() *
                                       (forwardDelta * terrainMovementMultiplier);
            lastAppliedForwardDistance = currentForwardDistance;
            if (terrainForces != null)
                locomotionWorldPosition = terrainForces.ApplyForces(
                    locomotionWorldPosition);
            locomotionWorldPosition = ResolveObstacleMovement(
                movementStart, locomotionWorldPosition);
            transform.position = locomotionWorldPosition;

            if (hasTerrainRootClearance && TrySampleTerrainHeight(transform.position, out float terrainHeight))
            {
                Vector3 worldPosition = transform.position;
                worldPosition.y = terrainHeight + terrainRootClearance;
                transform.position = worldPosition;
                locomotionWorldPosition = worldPosition;
            }

        }

        private float GetTerrainMovementMultiplier(Vector3 worldPosition)
        {
            float slopeMultiplier = 1f;
            if (TrySampleTerrainSurface(worldPosition, out _, out Vector3 normal))
            {
                float slope = Vector3.Angle(normal, Vector3.up);
                slopeMultiplier = Mathf.Lerp(
                    1f,
                    steepSlopeSpeedMultiplier,
                    Mathf.InverseLerp(0f, steepSlopeAngle, slope));
            }

            if (Time.time >= nextVegetationSampleTime)
            {
                cachedVegetationMultiplier = SampleVegetationMultiplier(worldPosition);
                nextVegetationSampleTime = Time.time + 0.20f;
            }

            return slopeMultiplier * cachedVegetationMultiplier;
        }

        private float SampleVegetationMultiplier(Vector3 worldPosition)
        {
            if (activeTerrain == null || activeTerrain.terrainData == null)
                return 1f;

            TerrainData data = activeTerrain.terrainData;
            if (data.detailPrototypes == null || data.detailPrototypes.Length == 0 ||
                data.detailWidth <= 0 || data.detailHeight <= 0)
                return 1f;

            Vector3 origin = activeTerrain.transform.position;
            Vector3 size = data.size;
            int x = Mathf.Clamp(
                Mathf.FloorToInt((worldPosition.x - origin.x) / size.x * data.detailWidth),
                0, data.detailWidth - 1);
            int z = Mathf.Clamp(
                Mathf.FloorToInt((worldPosition.z - origin.z) / size.z * data.detailHeight),
                0, data.detailHeight - 1);

            for (int layer = 0; layer < data.detailPrototypes.Length; layer++)
            {
                int[,] density = data.GetDetailLayer(x, z, 1, 1, layer);
                if (density[0, 0] > 0)
                    return grassSpeedMultiplier;
            }

            return 1f;
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

        private static bool IsWalkableTerrainContact(RaycastHit hit)
        {
            // Terrain-painted tree trunks may be reported through the same
            // TerrainCollider as the ground. Ignore only upward-facing ground;
            // vertical terrain/tree surfaces must still stop the spider.
            return hit.collider is TerrainCollider && hit.normal.y >= 0.55f;
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

        private bool TrySampleTerrainSurface(
            Vector3 worldPosition,
            out float height,
            out Vector3 normal)
        {
            normal = Vector3.up;
            if (!TrySampleTerrainHeight(worldPosition, out height))
                return false;

            Vector3 origin = activeTerrain.transform.position;
            Vector3 size = activeTerrain.terrainData.size;
            float normalizedX = Mathf.InverseLerp(
                origin.x, origin.x + size.x, worldPosition.x);
            float normalizedZ = Mathf.InverseLerp(
                origin.z, origin.z + size.z, worldPosition.z);
            normal = activeTerrain.terrainData.GetInterpolatedNormal(
                normalizedX, normalizedZ).normalized;
            return true;
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
            locomotionWorldPosition = transform.position;
            hasLocomotionWorldPosition = true;
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
            for (int i = 0; i < legs.Length; i++)
                RestoreLegPose(legs[i]);
        }

        private void SolveLegsWithBalancePosture(
            LegChain[] legs,
            bool isLeftSide,
            float cadenceImbalance)
        {
            float imbalance = Mathf.Clamp(cadenceImbalance, -1f, 1f);
            bool thisSideIsFaster = isLeftSide
                ? imbalance > 0f
                : imbalance < 0f;
            float strength = Mathf.Abs(imbalance);
            float radialAmount = (thisSideIsFaster
                ? -balanceLegBend
                : balanceLegExtension) * strength;

            for (int i = 0; i < legs.Length; i++)
            {
                LegChain leg = legs[i];
                Vector3 outward = Vector3.ProjectOnPlane(
                    leg.WorldTarget - body.position, Vector3.up).normalized;
                if (outward.sqrMagnitude < 0.0001f)
                {
                    Vector3 rigRight = Vector3.ProjectOnPlane(
                        rigSpace.right, Vector3.up).normalized;
                    outward = isLeftSide ? -rigRight : rigRight;
                }
                SolveCcd(leg, leg.WorldTarget + outward * radialAmount);
            }
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
            alternationForceThreshold = Mathf.Max(0f, alternationForceThreshold);
            alternationDominanceMargin = Mathf.Max(0f, alternationDominanceMargin);
            alternationReleaseThreshold = Mathf.Clamp(
                alternationReleaseThreshold, 0f, alternationForceThreshold);
            noiseStandardDeviationMultiplier = Mathf.Clamp(noiseStandardDeviationMultiplier, 2f, 8f);
            maximumAdaptiveNoiseThreshold = Mathf.Max(
                alternationForceThreshold, maximumAdaptiveNoiseThreshold);
            distancePerAlternation = Mathf.Max(0f, distancePerAlternation);
            minimumPressInterval = Mathf.Max(0f, minimumPressInterval);
            locomotionSmoothTime = Mathf.Max(0.01f, locomotionSmoothTime);
            directionChangeForceThreshold = Mathf.Max(0f, directionChangeForceThreshold);
            directionChangeHoldTime = Mathf.Max(0f, directionChangeHoldTime);
            minimumStepDuration = Mathf.Max(0.05f, minimumStepDuration);
            maximumStepDuration = Mathf.Max(minimumStepDuration, maximumStepDuration);
            turnActivationForce = Mathf.Max(0f, turnActivationForce);
            turnForceForMaximumSpeed = Mathf.Max(
                turnActivationForce + 0.01f, turnForceForMaximumSpeed);
            minimumTurnDegreesPerSecond = Mathf.Max(0f, minimumTurnDegreesPerSecond);
            maximumTurnDegreesPerSecond = Mathf.Max(
                minimumTurnDegreesPerSecond, maximumTurnDegreesPerSecond);
            turnResponseSpeed = Mathf.Max(0.01f, turnResponseSpeed);
            outsideTurnStrideMultiplier = Mathf.Max(0f, outsideTurnStrideMultiplier);
            insideTurnStrideMultiplier = Mathf.Max(0f, insideTurnStrideMultiplier);
            turnRecoveryDuration = Mathf.Max(0.1f, turnRecoveryDuration);
            steepSlopeSpeedMultiplier = Mathf.Clamp(
                steepSlopeSpeedMultiplier, 0.1f, 1f);
            steepSlopeAngle = Mathf.Clamp(steepSlopeAngle, 1f, 70f);
            grassSpeedMultiplier = Mathf.Clamp(grassSpeedMultiplier, 0.1f, 1f);
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
