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
        [SerializeField, Min(0.25f)] private float calibrationDurationSeconds = 10f;

        [Header("Dexter Active Movement Calibration")]
        [Tooltip("When physical Dexter raw samples are detected, pause movement and learn a comfortable walking force from the first calibration period. iPad/editor input is unaffected.")]
        [SerializeField] private bool calibrateDexterMovementOnStart = true;
        [Tooltip("Safety floor for a learned Dexter reference so an accidental no-input calibration cannot amplify sensor noise.")]
        [SerializeField, Min(0.01f)] private float minimumDexterCalibrationForce = 0.1f;
        [Tooltip("Minimum 10th-to-90th percentile force range along each learned walking axis. Calibration repeats when either finger did not actually wiggle.")]
        [SerializeField, Min(0.01f)] private float minimumCalibrationWiggleRange = 0.08f;
        [Tooltip("Learns the natural diagonal angle of each finger's calibration wiggle and treats that direction as pure forward/back input. This removes coupled X force from walking without removing deliberate sideways turning.")]
        [SerializeField] private bool alignWalkingAxisFromCalibration = true;
        [Tooltip("Safety limit for how far calibration may rotate a finger's forward axis away from device Y.")]
        [SerializeField, Range(0f, 80f)] private float maximumWalkingAxisCorrectionDegrees = 60f;
        [Tooltip("After axis alignment, removes the remaining lateral X component from force that is clearly inside the forward/back walking corridor. Set to 1 for a perfectly straight displayed walking vector.")]
        [SerializeField, Range(0f, 1f)] private float calibratedWalkingLateralSuppression = 1f;
        [Tooltip("Largest remaining |X|/|Y| ratio treated as walking and snapped onto Y. X-dominant two-finger gestures remain available for turning.")]
        [SerializeField, Range(0.1f, 0.75f)] private float calibratedWalkingCorridorRatio = 0.55f;
        [Tooltip("Physical Dexter force that reaches the high-speed walking point, as a multiple of the learned normal walking force.")]
        [SerializeField, Range(1.05f, 6f)] private float dexterMaximumSpeedForceMultiplier = 1.2f;
        [Tooltip("Walking-speed multiplier reached at the Dexter high-speed force point. This is separate from the iPad speed range.")]
        [SerializeField, Range(1f, 24f)] private float dexterHighForceWalkingSpeedMultiplier = 19.5f;
        [Tooltip("Absolute walking-speed multiplier at the calibrated normal-force reference.")]
        [SerializeField, Range(0.1f, 6f)] private float dexterBaselineWalkingSpeedMultiplier = 3f;
        [Tooltip("Walking-speed multiplier at very light Dexter force. The calibrated reference itself is always 1x.")]
        [SerializeField, Range(0.1f, 1f)] private float dexterMinimumWalkingSpeedMultiplier = 0.5f;
        [Tooltip("How long the calibrated speed-force envelope takes to release between alternating finger presses. Force increases remain immediate so short alternating presses retain their full strength; body acceleration is smoothed separately.")]
        [SerializeField, Range(0.05f, 0.6f)] private float dexterSpeedForceReleaseSeconds = 0.50f;
        [Tooltip("Maximum acceleration-response scale for physical Dexter walking. This prevents the 19.5x speed command from also multiplying acceleration by 19.5.")]
        [SerializeField, Range(0.5f, 6f)] private float dexterWalkingAccelerationResponseScale = 2.25f;
        [Tooltip("Braking-response scale for physical Dexter walking. Lower values produce a softer transition back to rest.")]
        [SerializeField, Range(0.5f, 6f)] private float dexterWalkingBrakingResponseScale = 0.75f;
        [Tooltip("Visible stride at the learned normal Dexter walking force. Increasing this makes each planted step carry the spider farther without increasing foot cadence.")]
        [SerializeField, Min(0.05f)] private float dexterBaselineStrideLength = 0.45f;
        [Tooltip("Visible stride at the Dexter high-speed force point. Keep this within the rig's reachable fore/aft range.")]
        [SerializeField, Min(0.05f)] private float dexterHighForceStrideLength = 1.50f;
        [Tooltip("Maximum Dexter gait cycles per second. Extra force above this cadence is expressed as a longer stride instead of faster, jittery foot cycling.")]
        [SerializeField, Range(0.5f, 8f)] private float dexterMaximumGaitCyclesPerSecond = 4f;
        [Tooltip("Maximum sensitivity boost applied to very light physical Dexter index/middle input. The boost tapers to 1 at maximum-speed force so small movements stay responsive without compressing the high-force control range. iPad controls are unaffected.")]
        [SerializeField, Range(1f, 3f)] private float dexterFingerInputSensitivity = 1.75f;

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
        [SerializeField, Min(0.05f)] private float inputDriveReleaseDelay = 0.60f;
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

        [Header("Thumb Jump")]
        [Tooltip("Dexter finger whose force magnitude triggers a forward jump.")]
        [SerializeField] private DexterFinger jumpFinger = DexterFinger.Thumb;
        [Tooltip("Thumb force magnitude required to start a jump.")]
        [SerializeField, Min(0.001f)] private float minimumJumpForce = 0.15f;
        [Tooltip("Fraction of the calibrated normal-jump force required to begin sampling a press. This reduces sensitivity while retaining small jumps.")]
        [SerializeField, Range(0.05f, 0.75f)] private float calibratedJumpActivationFraction = 0.40f;
        [Tooltip("Thumb force magnitude that produces the maximum jump distance and height.")]
        [SerializeField, Min(0.001f)] private float forceForMaximumJump = 1.50f;
        [Tooltip("Percentile of deliberate thumb movements used as the normal calibrated jump force.")]
        [SerializeField, Range(0.5f, 0.9f)] private float thumbJumpCalibrationReferencePercentile = 0.65f;
        [Tooltip("Final calibration time reserved for measuring the fully released thumb position.")]
        [SerializeField, Min(0.5f)] private float thumbNeutralCalibrationSeconds = 2f;
        [Tooltip("Safety multiplier above incidental thumb movement measured while index and middle continue walking during the released-thumb calibration period.")]
        [SerializeField, Min(1f)] private float thumbIncidentalForceSafetyMultiplier = 1.35f;
        [Tooltip("How long thumb force must remain above activation before the press is accepted as an intentional jump.")]
        [SerializeField, Min(0f)] private float jumpActivationHoldSeconds = 0.10f;
        [Tooltip("Additional thumb force required per Newton of active index/middle walking force. This rejects mechanically coupled thumb motion without disabling deliberate jumps during walking.")]
        [SerializeField, Min(0f)] private float walkingInputJumpActivationMargin = 0.25f;
        [Tooltip("Thumb force must return below this value before another jump can trigger.")]
        [SerializeField, Min(0f)] private float jumpReleaseForce = 0.08f;
        [Tooltip("Release threshold as a fraction of the calibrated base activation force. Kept below one for hysteresis while allowing realistic Dexter rest force to rearm jumping.")]
        [SerializeField, Range(0.5f, 0.98f)] private float jumpReleaseActivationFraction = 0.90f;
        [Tooltip("Short time used to capture the peak of a thumb press before committing to jump distance.")]
        [SerializeField, Min(0.02f)] private float jumpForceSamplingWindow = 0.20f;
        [SerializeField, Min(0f)] private float minimumJumpDistance = 0.25f;
        [SerializeField, Min(0f)] private float maximumJumpDistance = 2.50f;
        [SerializeField, Min(0f)] private float minimumJumpHeight = 0.40f;
        [SerializeField, Min(0f)] private float maximumJumpHeight = 1.20f;
        [Tooltip("Additional applied thumb force above the calibrated normal-jump force required for the strong jump.")]
        [SerializeField, Min(0f)] private float strongJumpForceAboveCalibration = 0.30f;
        [Tooltip("Forward-distance multiplier when applied thumb force exceeds the strong-jump threshold above calibrated rest.")]
        [SerializeField, Min(1f)] private float strongJumpDistanceMultiplier = 2f;
        [Tooltip("Air time at minimum thumb force. Stronger jumps remain airborne slightly longer.")]
        [SerializeField, Min(0.1f)] private float minimumJumpDuration = 0.48f;
        [Tooltip("Air time at maximum thumb force.")]
        [SerializeField, Min(0.1f)] private float maximumJumpDuration = 0.78f;
        [Tooltip("How far the rear legs extend behind the body during push-off.")]
        [SerializeField, Min(0f)] private float airborneRearLegStretch = 0.80f;
        [Tooltip("Brief lockout after landing before a new thumb press may jump.")]
        [SerializeField, Min(0f)] private float jumpCooldown = 0.20f;
        [Tooltip("Thumb must remain released for this long after landing before another jump can arm.")]
        [SerializeField, Min(0f)] private float jumpRearmReleaseHoldSeconds = 0.18f;

        [Header("Two-Finger Turning")]
        [Tooltip("Physical Dexter-only sensitivity for turn activation and speed range. Higher values require less X force without changing iPad turning.")]
        [SerializeField, Range(1f, 3f)] private float dexterTurningSensitivity = 1.5f;
        [Tooltip("Dexter X force, as a multiple of the learned walking reference, that reaches maximum turn speed. A wider range prevents immediate full-speed turning.")]
        [SerializeField, Range(1.5f, 8f)] private float dexterTurnMaximumSpeedReferenceMultiplier = 4f;
        [Tooltip("Minimum post-scaled X input on both fingers before an iPad turn registers.")]
        [SerializeField, Min(0f)] private float turnActivationForce = 1f;
        [Tooltip("X must be at least this fraction of Y on both fingers. This prevents a mostly vertical walking gesture from being mistaken for a turn.")]
        [SerializeField, Range(0f, 1f)] private float turnAxisDominanceRatio = 0.30f;
        [Tooltip("Same-direction X must remain deliberate for this long before it owns locomotion.")]
        [SerializeField, Min(0f)] private float turnIntentHoldSeconds = 0.09f;
        [Tooltip("How long coherent constant X must persist before it may take control directly from a recently established walking rhythm.")]
        [SerializeField, Min(0f)] private float turnWalkingTakeoverSeconds = 0.09f;
        [Tooltip("Maximum relative change allowed while X is proving that it is a constant turn gesture. Alternating X pulses restart the hold timer.")]
        [SerializeField, Range(0.05f, 1f)] private float turnConstantForceTolerance = 0.45f;
        [Tooltip("A recently completed Y alternation takes priority over mechanically coupled X force.")]
        [SerializeField, Min(0f)] private float walkingPriorityAfterAlternation = 0.65f;
        [Tooltip("Continuous opposite changes in the two Y channels must persist this long to confirm walking intent. This recognizes alternation even when neither finger fully releases.")]
        [SerializeField, Min(0.02f)] private float walkingIntentConfirmationSeconds = 0.10f;
        [Tooltip("Smallest per-frame Y change used by the walking-intent detector, expressed as a fraction of the calibrated walking force.")]
        [SerializeField, Range(0.001f, 0.1f)] private float walkingIntentDeltaFraction = 0.01f;
        [Tooltip("How long an established multi-handoff walking rhythm owns intent between successive finger exchanges.")]
        [SerializeField, Range(0.6f, 1.5f)] private float establishedWalkingIntentHoldSeconds = 1.0f;
        [Tooltip("Post-scaled iPad X input at or below minimum turn speed.")]
        [SerializeField, Min(0.001f)] private float turnMinimumSpeedDisplacement = 5f;
        [Tooltip("Post-scaled iPad X input that reaches maximum turn speed.")]
        [SerializeField, Min(0.01f)] private float turnForceForMaximumSpeed = 25f;
        [SerializeField, Min(0f)] private float minimumTurnDegreesPerSecond = 36f;
        [SerializeField, Min(0f)] private float maximumTurnDegreesPerSecond = 228f;
        [Tooltip("How quickly rotation and the turning gait blend in and out.")]
        [SerializeField, Min(0.01f)] private float turnResponseSpeed = 6f;
        [Tooltip("Maximum rate at which turning speed increases, in degrees per second squared.")]
        [SerializeField, Min(1f)] private float turnAngularAcceleration = 360f;
        [Tooltip("Maximum rate at which turning slows or reverses, in degrees per second squared.")]
        [SerializeField, Min(1f)] private float turnAngularDeceleration = 540f;
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
        [Tooltip("How quickly the root rounds a sharp floor/wall corner instead of snapping to the new support face.")]
        [SerializeField, Min(0.01f)] private float cornerRootPositionResponse = 2.5f;
        [Tooltip("Absolute root-speed limit while rounding a terrain corner. This prevents a discontinuous heightmap sample from teleporting the spider.")]
        [SerializeField, Min(0.01f)] private float cornerRootMaximumSpeed = 3f;
        [Tooltip("Largest unresolved gap allowed between a sampled surface pose and the root pose before the climb anchor pauses. This prevents a high-speed command from advancing the virtual surface far ahead of the physical spider.")]
        [SerializeField, Min(0.05f)] private float maximumSurfaceAnchorTrackingError = 0.85f;
        [Tooltip("Minimum time the root, torso, and feet share one surface-transition frame.")]
        [SerializeField, Min(0f)] private float minimumSurfaceTransitionDuration = 0.20f;
        [Tooltip("Maximum time a surface transition may remain active if terrain samples never fully settle.")]
        [SerializeField, Min(0.1f)] private float maximumSurfaceTransitionDuration = 1.5f;
        [Tooltip("Required agreement between the raw terrain normal and smoothed support normal before a transition completes.")]
        [SerializeField, Range(1f, 20f)] private float surfaceTransitionNormalTolerance = 8f;
        [Tooltip("How long the surface normals must remain settled before returning to ordinary ground or wall handling.")]
        [SerializeField, Min(0f)] private float surfaceTransitionStableDuration = 0.15f;

        [Header("Safe Foot Workspace")]
        [Tooltip("Maximum movement farther away from the body.")]
        [SerializeField, Min(0f)] private float maximumOutwardMovement = 0.55f;
        [Tooltip("Maximum movement toward the body. Kept small to prevent crossing the body.")]
        [SerializeField, Min(0f)] private float maximumInwardMovement = 0.10f;
        [Tooltip("Maximum movement parallel to the side of the body.")]
        [SerializeField, Min(0f)] private float maximumForeAftMovement = 0.65f;
        [Tooltip("Largest terrain-height difference an individual foot may follow before the root enters a shared surface transition.")]
        [SerializeField, Min(0.05f)] private float maximumPreTransitionFootHeightDifference = 0.75f;
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
        [Tooltip("The other finger must remain below this fraction of the active finger for the input to count as an isolated side lift.")]
        [SerializeField, Range(0.05f, 0.75f)] private float singleFingerIsolationRatio = 0.35f;
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
        private Vector2 thumbJumpBaseline;
        private readonly List<Vector2> thumbJumpCalibrationSamples =
            new List<Vector2>(1024);
        private readonly List<Vector2> thumbNeutralCalibrationSamples =
            new List<Vector2>(256);
        private float calibratedThumbJumpReferenceForce;
        private float calibratedThumbIncidentalForce;
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
        private float smoothedLeftDexterSpeedForce;
        private float smoothedRightDexterSpeedForce;
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
        private float lastAlternatingYIntentTime = -1f;
        private float alternatingYIntentConfidence;
        private float previousLeftYActivity;
        private float previousRightYActivity;
        private bool hasPreviousYActivity;
        private int previousYDominantFinger;
        private float lastYDominanceEventTime = -1f;
        private float lastQualifiedYHandoffTime = -1f;
        private float lastConfirmedWalkingPatternTime = -1f;
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
        private int pendingTurnDirection;
        private float pendingTurnStartedAt = -1f;
        private float pendingTurnReferenceMagnitude;
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
        private SpiderRigidbodyDynamics rigidbodyDynamics;
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
        private bool rootPositionTransitionActive;
        private bool wasOnClimbSurface;
        private bool transitionTargetIsClimb;
        private bool transitionAwaitingClimbEntry;
        private float surfaceTransitionStartedAt = -1f;
        private float surfaceTransitionStableSince = -1f;
        private bool traceFoundSurfaceFrame;
        private bool traceEnteredClimb;
        private bool traceCornerTransition;
        private bool traceRootTransitionActive;
        private Vector3 traceMovementStart;
        private Vector3 traceSurfaceSample;
        private Vector3 traceMovedSurfaceSample;
        private Vector3 traceSurfaceForward;
        private Vector3 traceSurfacePoint;
        private Vector3 traceRawSurfaceNormal = Vector3.up;
        private Vector3 traceSupportNormal = Vector3.up;
        private Vector3 traceClearedRootTarget;
        private StreamWriter traceWriter;
        private long lastTracedSequence = long.MinValue;
        private int unflushedTraceRows;
        private string traceFilePath;
        private readonly List<Vector2> dexterLeftMovementCalibrationSamples =
            new List<Vector2>(1024);
        private readonly List<Vector2> dexterRightMovementCalibrationSamples =
            new List<Vector2>(1024);
        private bool waitingForDexterMovementCalibration;
        private bool dexterCalibrationStartConfirmed;
        private bool isDexterMovementCalibrating;
        private bool dexterMovementCalibrationComplete;
        private bool dexterCalibrationNeedsRetry;
        private float dexterMovementCalibrationStart = -1f;
        private float dexterCalibrationCompleteMessageUntil = -1f;
        private float calibratedDexterReferenceForce = 1f;
        private Vector2 calibratedLeftWalkingAxis = Vector2.up;
        private Vector2 calibratedRightWalkingAxis = Vector2.up;
        private long lastDexterMovementCalibrationSequence = long.MinValue;
        private bool isJumping;
        private bool jumpInputArmed;
        private float jumpStartedAt = -1f;
        private float activeJumpDuration;
        private float activeJumpHeight;
        private float activeJumpForce;
        private float activeJumpDistance;
        private bool activeJumpIsStrong;
        private Vector2 currentThumbAppliedForce;
        private int jumpGateState = 1;
        private int jumpId;
        private int jumpEventThisFrame;
        private float lastCompletedJumpDistance;
        private bool isSamplingJumpForce;
        private float jumpForceSamplingStartedAt = -1f;
        private float sampledJumpPeakForce;
        private Vector2 sampledJumpPeakDirection;
        private bool jumpPressQualified;
        private float lastJumpLandedAt = float.NegativeInfinity;
        private float jumpReleaseBeganAt = -1f;
        private Vector3 jumpStartPosition;
        private Vector3 jumpLandingPosition;
        private Vector3 jumpForward = Vector3.forward;
        private Vector3 jumpUp = Vector3.up;
        private Quaternion jumpStartRotation = Quaternion.identity;
        private Quaternion jumpTargetRotation = Quaternion.identity;
        private bool debugWalkingIntentActive;
        private bool debugCoherentTurnCandidate;
        private string debugInputIntent = "NEUTRAL";
        private string debugTurnDecision = "No turn input";

        public bool IsReceiving => receiver != null && receiver.HasRecentFrame;
        public bool IsTaring => isTaring;
        public bool IsDexterMovementCalibrating =>
            isDexterMovementCalibrating;
        public float ForwardSpeed => forwardSpeed;
        public float TurnInput => turnInput;
        public float TurnSpeedDegreesPerSecond => currentTurnSpeed;
        public Vector2 IndexForce => smoothedLeftForce;
        public Vector2 MiddleForce => smoothedRightForce;
        public Vector2 ThumbForce => currentThumbAppliedForce;
        public float ActiveJumpForce => activeJumpForce;
        public Vector3 JumpDirection => jumpForward;
        public Vector3 JumpUp => jumpUp;
        public Vector3 SupportNormal => traceSupportNormal;
        public bool WalkingIntentActive => debugWalkingIntentActive;
        public bool CoherentTurnCandidate => debugCoherentTurnCandidate;
        public string InputIntentState => debugInputIntent;
        public string TurnDecisionReason => debugTurnDecision;
        public bool IsJumping => isJumping;
        public string TraceFilePath => traceFilePath;

        private void Awake()
        {
            InitializeRig();
        }

        private void OnEnable()
        {
            if (!initialized || body == null ||
                leftLegs == null || rightLegs == null)
                InitializeRig();
            PrepareDexterMovementCalibration();
            if (tareOnEnable && !calibrateDexterMovementOnStart)
                BeginTare();
            else
                UseUntaredRelayDefaults();
            ResetJumpState();
            BeginDiagnosticTrace();
        }

        private void PrepareDexterMovementCalibration()
        {
            waitingForDexterMovementCalibration =
                calibrateDexterMovementOnStart;
            dexterCalibrationStartConfirmed = false;
            isDexterMovementCalibrating = false;
            dexterMovementCalibrationComplete = false;
            dexterCalibrationNeedsRetry = false;
            dexterMovementCalibrationStart = -1f;
            dexterCalibrationCompleteMessageUntil = -1f;
            lastDexterMovementCalibrationSequence = long.MinValue;
            dexterLeftMovementCalibrationSamples.Clear();
            dexterRightMovementCalibrationSamples.Clear();
            thumbJumpBaseline = Vector2.zero;
            thumbJumpCalibrationSamples.Clear();
            thumbNeutralCalibrationSamples.Clear();
            calibratedThumbJumpReferenceForce = forceForMaximumJump;
            calibratedThumbIncidentalForce = 0f;
            calibratedDexterReferenceForce = Mathf.Max(
                minimumDexterCalibrationForce,
                minimumSpeedDisplacement / 5f);
            calibratedLeftWalkingAxis = Vector2.up;
            calibratedRightWalkingAxis = Vector2.up;
        }

        private void UseUntaredRelayDefaults()
        {
            isTaring = false;
            hasBaseline = false;
            alternatingDriveAuthorized = false;
            gaitDriveActive = false;
            lastCompletedAlternationTime = -1f;
            ResetWalkingIntentDetection();
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
            UpdateDexterMovementCalibration();
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
                rigSpace.forward,
                rootPositionTransitionActive,
                traceSupportNormal);

            SolveLegsWithBalancePosture(
                leftLegs, true, currentCadenceImbalance, leftFingerMimicLift,
                currentLeftLegPushDistance);
            SolveLegsWithBalancePosture(
                rightLegs, false, currentCadenceImbalance, rightFingerMimicLift,
                currentRightLegPushDistance);

            WriteDiagnosticTraceRow();
            jumpEventThisFrame = 0;
        }

        private void OnGUI()
        {
            bool waitingForDevice = waitingForDexterMovementCalibration &&
                (receiver == null || !receiver.HasRecentFrame ||
                 !IsPhysicalDexterFrame(receiver.LatestFrame));
            bool waitingForStart = waitingForDexterMovementCalibration &&
                !waitingForDevice && !dexterCalibrationStartConfirmed;
            bool showComplete = dexterMovementCalibrationComplete &&
                Time.realtimeSinceStartup <=
                dexterCalibrationCompleteMessageUntil;
            if (!waitingForDevice && !waitingForStart &&
                !isDexterMovementCalibrating &&
                !showComplete)
                return;

            float panelWidth = Mathf.Min(620f, Screen.width - 40f);
            float panelHeight = waitingForStart ? 220f : 150f;
            var panel = new Rect(
                (Screen.width - panelWidth) * 0.5f,
                Mathf.Max(20f, Screen.height * 0.12f),
                panelWidth,
                panelHeight);
            GUI.Box(panel, GUIContent.none);

            var titleStyle = new GUIStyle(GUI.skin.label)
            {
                alignment = TextAnchor.MiddleCenter,
                fontSize = Mathf.Clamp(Screen.height / 36, 20, 32),
                fontStyle = FontStyle.Bold,
                normal = { textColor = Color.white }
            };
            var messageStyle = new GUIStyle(GUI.skin.label)
            {
                alignment = TextAnchor.MiddleCenter,
                fontSize = Mathf.Clamp(Screen.height / 50, 16, 24),
                wordWrap = true,
                normal = { textColor = Color.white }
            };

            if (waitingForDevice)
            {
                GUI.Label(
                    new Rect(panel.x + 20f, panel.y + 12f,
                        panel.width - 40f, 42f),
                    "WAITING FOR DEXTER",
                    titleStyle);
                GUI.Label(
                    new Rect(panel.x + 30f, panel.y + 58f,
                        panel.width - 60f, 72f),
                    "No live Dexter force frames are arriving yet. Check the device/relay connection; calibration will begin automatically when data arrives.",
                    messageStyle);
            }
            else if (waitingForStart)
            {
                GUI.Label(
                    new Rect(panel.x + 20f, panel.y + 10f,
                        panel.width - 40f, 42f),
                    dexterCalibrationNeedsRetry
                        ? "CALIBRATION NEEDS MORE MOVEMENT"
                        : "DEXTER READY",
                    titleStyle);
                GUI.Label(
                    new Rect(panel.x + 30f, panel.y + 52f,
                        panel.width - 60f, 72f),
                    dexterCalibrationNeedsRetry
                        ? "Index and Middle did not both produce a clear walking wiggle. Press Start, then alternate both fingers continuously until calibration completes."
                        : "Place your hand in the device with thumb, index, and middle fingers positioned comfortably. Calibration will not begin until you are ready.",
                    messageStyle);
                var buttonStyle = new GUIStyle(GUI.skin.button)
                {
                    fontSize = Mathf.Clamp(Screen.height / 48, 17, 25),
                    fontStyle = FontStyle.Bold
                };
                if (GUI.Button(
                        new Rect(panel.x + panel.width * 0.20f,
                            panel.y + 142f,
                            panel.width * 0.60f, 54f),
                        "START CALIBRATION",
                        buttonStyle))
                {
                    dexterCalibrationStartConfirmed = true;
                    dexterCalibrationNeedsRetry = false;
                }
            }
            else if (isDexterMovementCalibrating)
            {
                float elapsed = Mathf.Max(
                    0f,
                    Time.realtimeSinceStartup -
                    dexterMovementCalibrationStart);
                float remaining = Mathf.Max(
                    0f, calibrationDurationSeconds - elapsed);
                bool measuringThumbNeutral = remaining <=
                    Mathf.Min(
                        thumbNeutralCalibrationSeconds,
                        calibrationDurationSeconds * 0.5f);
                GUI.Label(
                    new Rect(panel.x + 20f, panel.y + 12f,
                        panel.width - 40f, 42f),
                    measuringThumbNeutral
                        ? $"RELEASE THUMB NOW  {remaining:0.0}s"
                        : $"CALIBRATING DEXTER  {remaining:0.0}s",
                    titleStyle);
                GUI.Label(
                    new Rect(panel.x + 30f, panel.y + 58f,
                        panel.width - 60f, 72f),
                    measuringThumbNeutral
                        ? "Keep the thumb fully released until calibration finishes. You may continue alternating index and middle."
                        : "Alternate index and middle as if walking. Repeatedly press and release the thumb using the force you want for a normal jump.",
                    messageStyle);
            }
            else
            {
                GUI.Label(
                    new Rect(panel.x + 20f, panel.y + 20f,
                        panel.width - 40f, 44f),
                    "DEXTER CALIBRATION COMPLETE",
                    titleStyle);
                GUI.Label(
                    new Rect(panel.x + 30f, panel.y + 68f,
                        panel.width - 60f, 52f),
                    $"Normal walking force: {calibratedDexterReferenceForce:0.00}",
                    messageStyle);
            }
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
            row.Append("unity_realtime_s,unity_time_s,unity_frame,sequence,frame_age_s,has_recent,is_taring,is_dexter_calibrating,dexter_calibration_complete,");
            row.Append("thumb_raw,index_raw,middle_raw,ring_raw,pinky_raw,");
            row.Append("thumb_fx,thumb_fy,index_fx,index_fy,middle_fx,middle_fy,ring_fx,ring_fy,pinky_fx,pinky_fy,");
            row.Append("index_baseline_x,index_baseline_y,middle_baseline_x,middle_baseline_y,thumb_jump_baseline_x,thumb_jump_baseline_y,");
            row.Append("index_activation_threshold,middle_activation_threshold,index_release_threshold,middle_release_threshold,");
            row.Append("calibrated_walking_reference_force,dexter_finger_input_sensitivity,index_walking_axis_x,index_walking_axis_y,middle_walking_axis_x,middle_walking_axis_y,index_speed_force,middle_speed_force,active_force_speed_multiplier,active_force_stride_length,index_scaled_smoothed_fx,index_scaled_smoothed_fy,middle_scaled_smoothed_fx,middle_scaled_smoothed_fy,");
            row.Append("pressed_finger,last_active_finger,maximum_alternation_interval_s,input_drive_release_delay_s,travel_direction,forward_speed,target_forward_distance,current_forward_distance,");
            row.Append("turn_input,turn_speed_deg_s,yaw_degrees,dexter_turning_sensitivity,turn_activation_force,turn_minimum_speed_force,turn_maximum_speed_force,turn_axis_dominance_ratio,walking_intent_active,coherent_turn_candidate,input_intent_state,turn_decision_reason,");
            row.Append("thumb_applied_fx,thumb_applied_fy,thumb_applied_magnitude,");
            row.Append("jump_input_armed,is_sampling_jump_force,jump_press_qualified,sampled_jump_peak_force,jump_sampling_elapsed_s,jump_activation_hold_s,jump_gate_state,jump_gate_reason,jump_activation_force,jump_base_activation_force,walking_input_force,walking_jump_activation_margin_per_n,walking_jump_activation_addition,jump_release_force,jump_release_activation_fraction,calibrated_jump_reference_force,calibrated_thumb_incidental_force,thumb_incidental_safety_multiplier,thumb_incidental_activation_floor,calibrated_jump_activation_fraction,strong_jump_delta_force,strong_jump_trigger_force,");
            row.Append("jump_id,jump_event,jump_event_name,is_jumping,jump_force,jump_progress,jump_distance,strong_jump,");
            row.Append("jump_start_x,jump_start_y,jump_start_z,jump_landing_x,jump_landing_y,jump_landing_z,");
            row.Append("jump_current_distance,last_completed_jump_distance,");
            row.Append("left_gait_phase,left_target_gait_phase,right_gait_phase,right_target_gait_phase,");
            row.Append("spider_x,spider_y,spider_z,body_x,body_y,body_z,body_pitch,body_roll,body_yaw");
            row.Append(",surface_found,entered_climb,corner_transition,root_transition_active,climb_anchor_active");
            row.Append(",movement_start_x,movement_start_y,movement_start_z");
            row.Append(",surface_sample_x,surface_sample_y,surface_sample_z");
            row.Append(",moved_surface_sample_x,moved_surface_sample_y,moved_surface_sample_z");
            row.Append(",surface_forward_x,surface_forward_y,surface_forward_z");
            row.Append(",surface_point_x,surface_point_y,surface_point_z");
            row.Append(",raw_surface_normal_x,raw_surface_normal_y,raw_surface_normal_z");
            row.Append(",support_normal_x,support_normal_y,support_normal_z");
            row.Append(",cleared_root_target_x,cleared_root_target_y,cleared_root_target_z");
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
                row.Append($",{side}_surface_{number}_x,{side}_surface_{number}_y,{side}_surface_{number}_z");
                row.Append($",{side}_surface_normal_{number}_x,{side}_surface_normal_{number}_y,{side}_surface_normal_{number}_z");
                row.Append($",{side}_surface_{number}_normal_clearance");
            }
        }

        private void WriteDiagnosticTraceRow()
        {
            DexterForceFrame frame = receiver != null ? receiver.LatestFrame : null;
            if (traceWriter == null)
                return;

            long sequence = frame != null ? frame.sequence : Time.frameCount;
            if (sequence == lastTracedSequence && jumpEventThisFrame == 0)
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
            AppendBoolean(row, isDexterMovementCalibrating);
            AppendBoolean(row, dexterMovementCalibrationComplete);

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
            AppendVector2(row, thumbJumpBaseline);
            AppendNumber(row, leftAdaptiveActivationThreshold);
            AppendNumber(row, rightAdaptiveActivationThreshold);
            AppendNumber(row, leftAdaptiveReleaseThreshold);
            AppendNumber(row, rightAdaptiveReleaseThreshold);
            AppendNumber(row, calibratedDexterReferenceForce);
            AppendNumber(row, dexterFingerInputSensitivity);
            AppendVector2(row, calibratedLeftWalkingAxis);
            AppendVector2(row, calibratedRightWalkingAxis);
            AppendNumber(row, smoothedLeftDexterSpeedForce);
            AppendNumber(row, smoothedRightDexterSpeedForce);
            AppendNumber(row, activeForceSpeedMultiplier);
            AppendNumber(row, activeForceStrideLength);
            AppendVector2(row, smoothedLeftForce);
            AppendVector2(row, smoothedRightForce);
            AppendInteger(row, pressedFinger);
            AppendInteger(row, lastActiveFinger);
            AppendNumber(row, maximumAlternationInterval);
            AppendNumber(row, inputDriveReleaseDelay);
            AppendInteger(row, travelDirection);
            AppendNumber(row, forwardSpeed);
            AppendNumber(row, targetForwardDistance);
            AppendNumber(row, currentForwardDistance);
            AppendNumber(row, turnInput);
            AppendNumber(row, currentTurnSpeed);
            AppendNumber(row, currentYawDegrees);
            AppendNumber(row, dexterTurningSensitivity);
            AppendNumber(row, GetActiveTurnActivationForce());
            AppendNumber(row, GetActiveTurnMinimumSpeedDisplacement());
            AppendNumber(row, GetActiveTurnMaximumSpeedDisplacement());
            AppendNumber(row, turnAxisDominanceRatio);
            AppendBoolean(row, debugWalkingIntentActive);
            AppendBoolean(row, debugCoherentTurnCandidate);
            AppendCsvText(row, debugInputIntent);
            AppendCsvText(row, debugTurnDecision);
            AppendVector2(row, currentThumbAppliedForce);
            AppendNumber(row, currentThumbAppliedForce.magnitude);
            AppendBoolean(row, jumpInputArmed);
            AppendBoolean(row, isSamplingJumpForce);
            AppendBoolean(row, jumpPressQualified);
            AppendNumber(row, sampledJumpPeakForce);
            AppendNumber(row, isSamplingJumpForce
                ? Mathf.Max(0f, Time.time - jumpForceSamplingStartedAt)
                : 0f);
            AppendNumber(row, jumpActivationHoldSeconds);
            AppendInteger(row, jumpGateState);
            AppendCsvText(row, GetJumpGateReason(jumpGateState));
            AppendNumber(row, GetActiveJumpActivationForce());
            AppendNumber(row, GetBaseJumpActivationForce());
            float walkingJumpInputForce =
                Mathf.Abs(smoothedLeftForce.y) +
                Mathf.Abs(smoothedRightForce.y);
            AppendNumber(row, walkingJumpInputForce);
            AppendNumber(row, walkingInputJumpActivationMargin);
            AppendNumber(row, walkingJumpInputForce *
                walkingInputJumpActivationMargin);
            AppendNumber(row, GetActiveJumpReleaseForce());
            AppendNumber(row, jumpReleaseActivationFraction);
            AppendNumber(row, calibratedThumbJumpReferenceForce);
            AppendNumber(row, calibratedThumbIncidentalForce);
            AppendNumber(row, thumbIncidentalForceSafetyMultiplier);
            AppendNumber(row, calibratedThumbIncidentalForce *
                thumbIncidentalForceSafetyMultiplier);
            AppendNumber(row, calibratedJumpActivationFraction);
            AppendNumber(row, strongJumpForceAboveCalibration);
            AppendNumber(row, GetStrongJumpTriggerForce());
            AppendInteger(row, jumpId);
            AppendInteger(row, jumpEventThisFrame);
            AppendCsvText(row, jumpEventThisFrame == 1
                ? "started"
                : jumpEventThisFrame == 2 ? "landed" : string.Empty);
            AppendBoolean(row, isJumping);
            AppendNumber(row, activeJumpForce);
            AppendNumber(row, isJumping
                ? Mathf.Clamp01((Time.time - jumpStartedAt) /
                    Mathf.Max(0.1f, activeJumpDuration))
                : 0f);
            AppendNumber(row, activeJumpDistance);
            AppendBoolean(row, activeJumpIsStrong);
            AppendVector3(row, jumpStartPosition);
            AppendVector3(row, jumpLandingPosition);
            AppendNumber(row, jumpId > 0
                ? Vector3.ProjectOnPlane(
                    transform.position - jumpStartPosition,
                    jumpUp).magnitude
                : 0f);
            AppendNumber(row, lastCompletedJumpDistance);
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
            AppendBoolean(row, traceFoundSurfaceFrame);
            AppendBoolean(row, traceEnteredClimb);
            AppendBoolean(row, traceCornerTransition);
            AppendBoolean(row, traceRootTransitionActive);
            AppendBoolean(row, hasClimbSurfaceAnchor);
            AppendVector3(row, traceMovementStart);
            AppendVector3(row, traceSurfaceSample);
            AppendVector3(row, traceMovedSurfaceSample);
            AppendVector3(row, traceSurfaceForward);
            AppendVector3(row, traceSurfacePoint);
            AppendVector3(row, traceRawSurfaceNormal);
            AppendVector3(row, traceSupportNormal);
            AppendVector3(row, traceClearedRootTarget);
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

                if (terrainForces != null &&
                    terrainForces.TryGetTerrainSurfaceFrame(
                        legs[i].Effector.position,
                        out Vector3 surfacePoint,
                        out Vector3 surfaceNormal))
                {
                    AppendVector3(row, surfacePoint);
                    AppendVector3(row, surfaceNormal);
                    AppendNumber(
                        row,
                        Vector3.Dot(
                            legs[i].Effector.position - surfacePoint,
                            surfaceNormal.normalized));
                }
                else
                {
                    for (int column = 0; column < 7; column++)
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

        private static void AppendCsvText(StringBuilder row, string value)
        {
            row.Append(',').Append('"').Append(
                (value ?? string.Empty).Replace("\"", "\"\"")).Append('"');
        }

        private static string GetJumpGateReason(int state)
        {
            switch (state)
            {
                case 0: return "triggered";
                case 1: return "no_physical_thumb_sample";
                case 2: return "below_activation_force";
                case 3: return "waiting_for_release";
                case 4: return "landing_cooldown";
                case 5: return "already_airborne";
                case 6: return "sampling_press_peak";
                case 8: return "confirming_intentional_press";
                default: return "unknown";
            }
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
            smoothedLeftDexterSpeedForce = 0f;
            smoothedRightDexterSpeedForce = 0f;
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
            ResetWalkingIntentDetection();
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
            rootPositionTransitionActive = false;
            wasOnClimbSurface = false;
            transitionTargetIsClimb = false;
            transitionAwaitingClimbEntry = false;
            surfaceTransitionStartedAt = -1f;
            surfaceTransitionStableSince = -1f;
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
            ResetJumpState();
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
            ConfigureRigidbodyDynamics();
            if (GetComponent<SpiderForceDebugOverlay>() == null)
                gameObject.AddComponent<SpiderForceDebugOverlay>();
            if (GetComponent<SpiderFollowingLight>() == null)
                gameObject.AddComponent<SpiderFollowingLight>();
            initialized = true;
            Debug.Log(
                $"{nameof(DexterFrontLegIK)} ready: {leftLegFinger} drives the left tetrapod gait, " +
                $"{rightLegFinger} drives the right tetrapod gait.",
                this);
        }

        private void ConfigureRigidbodyDynamics()
        {
            rigidbodyDynamics = GetComponent<SpiderRigidbodyDynamics>();
            if (rigidbodyDynamics == null)
                rigidbodyDynamics = gameObject.AddComponent<SpiderRigidbodyDynamics>();

            var feet = new Transform[leftLegs.Length + rightLegs.Length];
            for (int i = 0; i < leftLegs.Length; i++)
            {
                feet[i] = leftLegs[i].Effector;
                feet[i + leftLegs.Length] = rightLegs[i].Effector;
            }
            rigidbodyDynamics.Initialize(feet, body);
            rigidbodyDynamics.SetDesiredPose(
                transform.position,
                transform.rotation,
                Vector3.up,
                0f,
                false);
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

        private void UpdateDexterMovementCalibration()
        {
            if ((!waitingForDexterMovementCalibration &&
                 !isDexterMovementCalibrating) || receiver == null)
                return;

            DexterForceFrame frame = receiver.LatestFrame;
            if (!IsPhysicalDexterFrame(frame) || !receiver.HasRecentFrame)
                return;

            if (waitingForDexterMovementCalibration)
            {
                if (!dexterCalibrationStartConfirmed)
                    return;
                waitingForDexterMovementCalibration = false;
                isDexterMovementCalibrating = true;
                dexterMovementCalibrationStart =
                    Time.realtimeSinceStartup;
                lastDexterMovementCalibrationSequence = long.MinValue;
                dexterLeftMovementCalibrationSamples.Clear();
                dexterRightMovementCalibrationSamples.Clear();
                ResetInputStateForMovementCalibration();
            }

            if (frame.sequence != lastDexterMovementCalibrationSequence)
            {
                lastDexterMovementCalibrationSequence = frame.sequence;
                if (TryReadForce(
                        leftLegFinger, out Vector2 leftCalibrationForce))
                {
                    dexterLeftMovementCalibrationSamples.Add(
                        leftCalibrationForce);
                }
                if (TryReadForce(
                        rightLegFinger, out Vector2 rightCalibrationForce))
                {
                    dexterRightMovementCalibrationSamples.Add(
                        rightCalibrationForce);
                }
                if (TryReadForce(
                        jumpFinger, out Vector2 thumbCalibrationForce))
                {
                    thumbJumpCalibrationSamples.Add(
                        thumbCalibrationForce);
                    float elapsed = Time.realtimeSinceStartup -
                                    dexterMovementCalibrationStart;
                    float neutralWindow = Mathf.Min(
                        thumbNeutralCalibrationSeconds,
                        calibrationDurationSeconds * 0.5f);
                    if (elapsed >= calibrationDurationSeconds -
                        neutralWindow)
                        thumbNeutralCalibrationSamples.Add(
                            thumbCalibrationForce);
                }

                // The per-finger samples are retained so calibration can
                // separate the stable device preload from deliberate motion.
            }

            if (Time.realtimeSinceStartup -
                dexterMovementCalibrationStart <
                calibrationDurationSeconds)
                return;

            CompleteDexterMovementCalibration();
        }

        private void CompleteDexterMovementCalibration()
        {
            Vector2 learnedLeftAxis = alignWalkingAxisFromCalibration
                ? CalculateCalibrationWalkingAxis(
                    dexterLeftMovementCalibrationSamples)
                : Vector2.up;
            Vector2 learnedRightAxis = alignWalkingAxisFromCalibration
                ? CalculateCalibrationWalkingAxis(
                    dexterRightMovementCalibrationSamples)
                : Vector2.up;
            learnedLeftAxis = LimitWalkingAxisCorrection(learnedLeftAxis);
            learnedRightAxis = LimitWalkingAxisCorrection(learnedRightAxis);
            float leftWiggleRange = CalculateCalibrationWiggleRange(
                dexterLeftMovementCalibrationSamples, learnedLeftAxis);
            float rightWiggleRange = CalculateCalibrationWiggleRange(
                dexterRightMovementCalibrationSamples, learnedRightAxis);
            if (leftWiggleRange < minimumCalibrationWiggleRange ||
                rightWiggleRange < minimumCalibrationWiggleRange)
            {
                isDexterMovementCalibrating = false;
                dexterMovementCalibrationComplete = false;
                waitingForDexterMovementCalibration = true;
                dexterCalibrationStartConfirmed = false;
                dexterCalibrationNeedsRetry = true;
                dexterMovementCalibrationStart = -1f;
                ResetInputStateForMovementCalibration();
                Debug.LogWarning(
                    $"{nameof(DexterFrontLegIK)} rejected Dexter " +
                    $"calibration because walking motion was too small. " +
                    $"Index walking-axis range={leftWiggleRange:F3} N, " +
                    $"Middle walking-axis range={rightWiggleRange:F3} N; " +
                    $"required={minimumCalibrationWiggleRange:F3} N.",
                    this);
                return;
            }

            CompleteThumbJumpCalibration();
            leftBaseline = EstimateStableDexterBaseline(
                dexterLeftMovementCalibrationSamples);
            rightBaseline = EstimateStableDexterBaseline(
                dexterRightMovementCalibrationSamples);
            calibratedLeftWalkingAxis = learnedLeftAxis;
            calibratedRightWalkingAxis = learnedRightAxis;
            hasBaseline = dexterLeftMovementCalibrationSamples.Count > 0 ||
                          dexterRightMovementCalibrationSamples.Count > 0;

            var activeWalkingSamples = new List<float>(
                dexterLeftMovementCalibrationSamples.Count +
                dexterRightMovementCalibrationSamples.Count);
            for (int i = 0;
                 i < dexterLeftMovementCalibrationSamples.Count; i++)
            {
                activeWalkingSamples.Add(Mathf.Abs(
                    AlignForceToWalkingAxis(
                        dexterLeftMovementCalibrationSamples[i] -
                        leftBaseline,
                        calibratedLeftWalkingAxis).y));
            }
            for (int i = 0;
                 i < dexterRightMovementCalibrationSamples.Count; i++)
            {
                activeWalkingSamples.Add(Mathf.Abs(
                    AlignForceToWalkingAxis(
                        dexterRightMovementCalibrationSamples[i] -
                        rightBaseline,
                        calibratedRightWalkingAxis).y));
            }

            if (activeWalkingSamples.Count > 0)
            {
                activeWalkingSamples.Sort();
                // Use a mean of the active upper portion. This keeps the
                // requested mean-style calibration while excluding the many
                // released samples clustered around the learned preload.
                int activeStart = Mathf.Clamp(
                    Mathf.FloorToInt(activeWalkingSamples.Count * 0.60f),
                    0, activeWalkingSamples.Count - 1);
                float forceSum = 0f;
                for (int i = activeStart;
                     i < activeWalkingSamples.Count; i++)
                    forceSum += activeWalkingSamples[i];
                calibratedDexterReferenceForce = Mathf.Max(
                    minimumDexterCalibrationForce,
                    forceSum /
                    Mathf.Max(1, activeWalkingSamples.Count - activeStart));
            }
            else
            {
                calibratedDexterReferenceForce = Mathf.Max(
                    minimumDexterCalibrationForce,
                minimumSpeedDisplacement / 5f);
            }

            float calibratedActivationThreshold = Mathf.Clamp(
                calibratedDexterReferenceForce * 0.04f,
                alternationForceThreshold,
                maximumAdaptiveNoiseThreshold);
            leftAdaptiveActivationThreshold = calibratedActivationThreshold;
            rightAdaptiveActivationThreshold = calibratedActivationThreshold;
            leftAdaptiveReleaseThreshold = Mathf.Max(
                alternationReleaseThreshold,
                leftAdaptiveActivationThreshold * 0.6f);
            rightAdaptiveReleaseThreshold = Mathf.Max(
                alternationReleaseThreshold,
                rightAdaptiveActivationThreshold * 0.6f);

            isDexterMovementCalibrating = false;
            dexterMovementCalibrationComplete = true;
            dexterCalibrationCompleteMessageUntil =
                Time.realtimeSinceStartup + 1.5f;
            ResetInputStateForMovementCalibration();
            Debug.Log(
                $"{nameof(DexterFrontLegIK)} active Dexter calibration " +
                $"complete: normal walking force=" +
                $"{calibratedDexterReferenceForce:F3}, " +
                $"Index neutral=({leftBaseline.x:F3}, " +
                $"{leftBaseline.y:F3}), Middle neutral=" +
                $"({rightBaseline.x:F3}, {rightBaseline.y:F3}), " +
                $"walking-axis correction Index=" +
                $"{GetWalkingAxisCorrectionDegrees(calibratedLeftWalkingAxis):F1}°, " +
                $"Middle=" +
                $"{GetWalkingAxisCorrectionDegrees(calibratedRightWalkingAxis):F1}°, " +
                $"maximum-speed force=" +
                $"{GetActiveMaximumSpeedDisplacement():F3}, " +
                $"thumb rest=({thumbJumpBaseline.x:F3}, " +
                $"{thumbJumpBaseline.y:F3}), " +
                $"normal thumb jump=" +
                $"{calibratedThumbJumpReferenceForce:F3}, " +
                $"2x threshold={GetStrongJumpTriggerForce():F3}.",
                this);
        }

        private static float CalculateCalibrationWiggleRange(
            List<Vector2> samples, Vector2 walkingAxis)
        {
            if (samples == null || samples.Count < 10)
                return 0f;
            var projectedValues = new List<float>(samples.Count);
            for (int i = 0; i < samples.Count; i++)
            {
                float value = Vector2.Dot(samples[i], walkingAxis);
                if (!float.IsNaN(value) && !float.IsInfinity(value))
                    projectedValues.Add(value);
            }
            if (projectedValues.Count < 10)
                return 0f;
            projectedValues.Sort();
            int lowIndex = Mathf.Clamp(
                Mathf.FloorToInt((projectedValues.Count - 1) * 0.10f),
                0, projectedValues.Count - 1);
            int highIndex = Mathf.Clamp(
                Mathf.CeilToInt((projectedValues.Count - 1) * 0.90f),
                0, projectedValues.Count - 1);
            return Mathf.Abs(
                projectedValues[highIndex] - projectedValues[lowIndex]);
        }

        private static Vector2 CalculateCalibrationWalkingAxis(
            List<Vector2> samples)
        {
            if (samples == null || samples.Count < 10)
                return Vector2.up;

            Vector2 mean = Vector2.zero;
            int validSamples = 0;
            for (int i = 0; i < samples.Count; i++)
            {
                Vector2 sample = samples[i];
                if (!IsFinite(sample))
                    continue;
                mean += sample;
                validSamples++;
            }
            if (validSamples < 10)
                return Vector2.up;
            mean /= validSamples;

            float xx = 0f;
            float xy = 0f;
            float yy = 0f;
            for (int i = 0; i < samples.Count; i++)
            {
                Vector2 sample = samples[i];
                if (!IsFinite(sample))
                    continue;
                Vector2 delta = sample - mean;
                xx += delta.x * delta.x;
                xy += delta.x * delta.y;
                yy += delta.y * delta.y;
            }

            if (xx + yy < 0.000001f)
                return Vector2.up;
            float angle = 0.5f * Mathf.Atan2(2f * xy, xx - yy);
            Vector2 axis = new Vector2(
                Mathf.Cos(angle), Mathf.Sin(angle)).normalized;
            if (Vector2.Dot(axis, Vector2.up) < 0f)
                axis = -axis;
            return axis;
        }

        private Vector2 LimitWalkingAxisCorrection(Vector2 axis)
        {
            if (!IsFinite(axis) || axis.sqrMagnitude < 0.001f)
                return Vector2.up;
            float correction = Mathf.Clamp(
                GetWalkingAxisCorrectionDegrees(axis),
                -maximumWalkingAxisCorrectionDegrees,
                maximumWalkingAxisCorrectionDegrees);
            float radians = correction * Mathf.Deg2Rad;
            return new Vector2(
                Mathf.Sin(radians), Mathf.Cos(radians)).normalized;
        }

        private static float GetWalkingAxisCorrectionDegrees(Vector2 axis)
        {
            return Mathf.Atan2(axis.x, axis.y) * Mathf.Rad2Deg;
        }

        private static Vector2 AlignForceToWalkingAxis(
            Vector2 force, Vector2 walkingAxis)
        {
            if (!IsFinite(walkingAxis) ||
                walkingAxis.sqrMagnitude < 0.001f)
                walkingAxis = Vector2.up;
            walkingAxis.Normalize();
            Vector2 lateralAxis = new Vector2(
                walkingAxis.y, -walkingAxis.x);
            return new Vector2(
                Vector2.Dot(force, lateralAxis),
                Vector2.Dot(force, walkingAxis));
        }

        private static bool IsFinite(Vector2 value)
        {
            return !float.IsNaN(value.x) &&
                   !float.IsInfinity(value.x) &&
                   !float.IsNaN(value.y) &&
                   !float.IsInfinity(value.y);
        }

        private void CompleteThumbJumpCalibration()
        {
            if (thumbJumpCalibrationSamples.Count == 0)
            {
                thumbJumpBaseline = Vector2.zero;
                calibratedThumbJumpReferenceForce =
                    Mathf.Max(minimumJumpForce, forceForMaximumJump);
                return;
            }

            List<Vector2> neutralSource =
                thumbNeutralCalibrationSamples.Count > 0
                    ? thumbNeutralCalibrationSamples
                    : thumbJumpCalibrationSamples;
            int neutralStart = thumbNeutralCalibrationSamples.Count > 0
                ? 0
                : Mathf.FloorToInt(neutralSource.Count * 0.8f);
            var xSamples = new List<float>(
                neutralSource.Count - neutralStart);
            var ySamples = new List<float>(
                neutralSource.Count - neutralStart);
            for (int i = neutralStart; i < neutralSource.Count; i++)
            {
                xSamples.Add(neutralSource[i].x);
                ySamples.Add(neutralSource[i].y);
            }
            xSamples.Sort();
            ySamples.Sort();
            thumbJumpBaseline = new Vector2(
                GetSortedMedian(xSamples),
                GetSortedMedian(ySamples));

            var incidentalSamples = new List<float>(neutralSource.Count);
            for (int i = 0; i < neutralSource.Count; i++)
            {
                incidentalSamples.Add(Vector2.Distance(
                    neutralSource[i], thumbJumpBaseline));
            }
            incidentalSamples.Sort();
            int incidentalIndex = Mathf.RoundToInt(
                (incidentalSamples.Count - 1) * 0.95f);
            calibratedThumbIncidentalForce = incidentalSamples.Count > 0
                ? incidentalSamples[Mathf.Clamp(
                    incidentalIndex, 0, incidentalSamples.Count - 1)]
                : 0f;

            var appliedSamples = new List<float>(
                thumbJumpCalibrationSamples.Count);
            for (int i = 0; i < thumbJumpCalibrationSamples.Count; i++)
            {
                float appliedMagnitude = Vector2.Distance(
                    thumbJumpCalibrationSamples[i],
                    thumbJumpBaseline);
                if (appliedMagnitude >= minimumJumpForce)
                    appliedSamples.Add(appliedMagnitude);
            }
            appliedSamples.Sort();
            if (appliedSamples.Count == 0)
            {
                calibratedThumbJumpReferenceForce =
                    Mathf.Max(minimumJumpForce, forceForMaximumJump);
                return;
            }

            int referenceIndex = Mathf.RoundToInt(
                (appliedSamples.Count - 1) *
                thumbJumpCalibrationReferencePercentile);
            calibratedThumbJumpReferenceForce = Mathf.Max(
                minimumJumpForce,
                appliedSamples[Mathf.Clamp(
                    referenceIndex, 0, appliedSamples.Count - 1)]);
        }

        private static float GetSortedMedian(List<float> values)
        {
            if (values == null || values.Count == 0)
                return 0f;
            int middle = values.Count / 2;
            return values.Count % 2 == 0
                ? (values[middle - 1] + values[middle]) * 0.5f
                : values[middle];
        }

        private static Vector2 EstimateStableDexterBaseline(
            List<Vector2> samples)
        {
            if (samples == null || samples.Count == 0)
                return Vector2.zero;

            var xValues = new List<float>(samples.Count);
            var yValues = new List<float>(samples.Count);
            for (int i = 0; i < samples.Count; i++)
            {
                if (float.IsNaN(samples[i].x) ||
                    float.IsInfinity(samples[i].x) ||
                    float.IsNaN(samples[i].y) ||
                    float.IsInfinity(samples[i].y))
                    continue;
                xValues.Add(samples[i].x);
                yValues.Add(samples[i].y);
            }

            return new Vector2(
                EstimateStableDexterComponent(xValues),
                EstimateStableDexterComponent(yValues));
        }

        private static float EstimateStableDexterComponent(
            List<float> values)
        {
            if (values == null || values.Count == 0)
                return 0f;

            values.Sort();
            int windowSize = Mathf.Clamp(
                Mathf.RoundToInt(values.Count * 0.20f),
                Mathf.Min(8, values.Count), values.Count);
            int bestStart = 0;
            float narrowestSpan = float.PositiveInfinity;
            for (int start = 0;
                 start + windowSize <= values.Count; start++)
            {
                float span = values[start + windowSize - 1] -
                             values[start];
                if (span >= narrowestSpan)
                    continue;
                narrowestSpan = span;
                bestStart = start;
            }

            float sum = 0f;
            for (int i = bestStart; i < bestStart + windowSize; i++)
                sum += values[i];
            return sum / windowSize;
        }

        private void ResetInputStateForMovementCalibration()
        {
            smoothedLeftForce = Vector2.zero;
            smoothedRightForce = Vector2.zero;
            smoothedLeftDexterSpeedForce = 0f;
            smoothedRightDexterSpeedForce = 0f;
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
            ResetWalkingIntentDetection();
            lastLiveFingerInputTime = -1f;
            forwardSpeed = 0f;
            turnInput = 0f;
            currentTurnSpeed = 0f;
            turnGaitWeight = 0f;
            pendingTurnDirection = 0;
            pendingTurnStartedAt = -1f;
            pendingTurnReferenceMagnitude = 0f;
            wasTurnGestureActive = false;
            isRecoveringFromTurn = false;
            leftSideWaveInPlace = false;
            rightSideWaveInPlace = false;
            leftFingerMimicLift = 0f;
            rightFingerMimicLift = 0f;
            currentLeftLegPushDistance = 0f;
            currentRightLegPushDistance = 0f;
            ResetJumpState();
            targetForwardDistance = currentForwardDistance;
            lastAppliedForwardDistance = currentForwardDistance;
            ResetWorldFootTargets(leftLegs);
            ResetWorldFootTargets(rightLegs);
        }

        private static bool IsPhysicalDexterFrame(DexterForceFrame frame)
        {
            if (frame?.fingers == null)
                return false;

            string transport = frame.transport ?? string.Empty;
            if (transport.IndexOf(
                    "ipad", StringComparison.OrdinalIgnoreCase) >= 0 ||
                transport.IndexOf(
                    "editor", StringComparison.OrdinalIgnoreCase) >= 0)
                return false;

            return HasRawSamples(frame.fingers.index) ||
                   HasRawSamples(frame.fingers.middle);
        }

        private void ResetJumpState()
        {
            isJumping = false;
            // A real below-threshold sample must arm the first jump. This
            // prevents residual load at startup from launching the spider.
            jumpInputArmed = false;
            jumpStartedAt = -1f;
            activeJumpDuration = 0f;
            activeJumpHeight = 0f;
            activeJumpForce = 0f;
            activeJumpDistance = 0f;
            activeJumpIsStrong = false;
            currentThumbAppliedForce = Vector2.zero;
            jumpGateState = 1;
            jumpId = 0;
            jumpEventThisFrame = 0;
            lastCompletedJumpDistance = 0f;
            isSamplingJumpForce = false;
            jumpForceSamplingStartedAt = -1f;
            sampledJumpPeakForce = 0f;
            sampledJumpPeakDirection = Vector2.zero;
            jumpPressQualified = false;
            lastJumpLandedAt = float.NegativeInfinity;
            jumpReleaseBeganAt = -1f;
            jumpStartRotation = transform.rotation;
            jumpTargetRotation = transform.rotation;
        }

        private static bool HasRawSamples(
            DexterFingerMeasurement measurement)
        {
            return measurement?.raw != null && measurement.raw.Length > 0;
        }

        private bool IsUsingCalibratedDexterInput()
        {
            return dexterMovementCalibrationComplete &&
                   IsPhysicalDexterFrame(receiver?.LatestFrame);
        }

        private float GetActiveForceRangeScale()
        {
            if (!IsUsingCalibratedDexterInput())
                return 1f;

            return calibratedDexterReferenceForce /
                   Mathf.Max(0.001f, minimumSpeedDisplacement);
        }

        private float GetActiveMinimumSpeedDisplacement()
        {
            return minimumSpeedDisplacement * GetActiveForceRangeScale();
        }

        private float GetActiveMaximumSpeedDisplacement()
        {
            return IsUsingCalibratedDexterInput()
                ? calibratedDexterReferenceForce *
                  dexterMaximumSpeedForceMultiplier
                : maximumSpeedDisplacement;
        }

        private float GetActiveTurnActivationForce()
        {
            float threshold = turnActivationForce *
                              GetActiveForceRangeScale();
            return IsUsingCalibratedDexterInput()
                ? threshold / dexterTurningSensitivity
                : threshold;
        }

        private float GetActiveTurnMinimumSpeedDisplacement()
        {
            float minimumForce = turnMinimumSpeedDisplacement *
                                 GetActiveForceRangeScale();
            return IsUsingCalibratedDexterInput()
                ? minimumForce / dexterTurningSensitivity
                : minimumForce;
        }

        private float GetActiveTurnMaximumSpeedDisplacement()
        {
            return IsUsingCalibratedDexterInput()
                ? Mathf.Max(
                    GetActiveTurnMinimumSpeedDisplacement() + 0.001f,
                    calibratedDexterReferenceForce *
                    dexterTurnMaximumSpeedReferenceMultiplier)
                : turnForceForMaximumSpeed;
        }

        private Vector2 ApplyDexterFingerSensitivity(Vector2 force)
        {
            float fullScaleForce = Mathf.Max(
                0.001f, GetActiveMaximumSpeedDisplacement());
            float forceStrength = Mathf.Clamp01(
                force.magnitude / fullScaleForce);
            float taperedBoost = Mathf.Lerp(
                dexterFingerInputSensitivity, 1f, forceStrength);
            return force * taperedBoost;
        }

        private Vector2 SnapCalibratedWalkingForceToY(Vector2 force)
        {
            float absoluteY = Mathf.Abs(force.y);
            if (absoluteY < 0.0001f ||
                Mathf.Abs(force.x) >
                absoluteY * calibratedWalkingCorridorRatio)
                return force;

            force.x = Mathf.Lerp(
                force.x, 0f, calibratedWalkingLateralSuppression);
            return force;
        }

        private float UpdateDexterSpeedForceEnvelope(
            float current, float target)
        {
            if (target >= current)
                return target;
            float releaseRate = Mathf.Max(
                calibratedDexterReferenceForce,
                current) / Mathf.Max(
                0.05f, dexterSpeedForceReleaseSeconds);
            return Mathf.MoveTowards(
                current, target, releaseRate * Time.deltaTime);
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
            jumpEventThisFrame = 0;
            Vector2 leftForce = Vector2.zero;
            Vector2 rightForce = Vector2.zero;
            Vector2 jumpForce = Vector2.zero;
            float leftDexterSpeedForce = 0f;
            float rightDexterSpeedForce = 0f;
            bool hasJumpForce = false;
            bool usingPhysicalDexterInput =
                IsPhysicalDexterFrame(receiver?.LatestFrame);

            if (receiver.HasRecentFrame && !IsTaring &&
                !isDexterMovementCalibrating)
            {
                bool physicalDexterFrame =
                    IsPhysicalDexterFrame(receiver.LatestFrame);
                if (TryReadForce(leftLegFinger, out Vector2 measuredLeftForce))
                    leftForce = measuredLeftForce - (hasBaseline ? leftBaseline : Vector2.zero);
                if (TryReadForce(rightLegFinger, out Vector2 measuredRightForce))
                    rightForce = measuredRightForce - (hasBaseline ? rightBaseline : Vector2.zero);
                if (physicalDexterFrame)
                {
                    leftForce = AlignForceToWalkingAxis(
                        leftForce, calibratedLeftWalkingAxis);
                    rightForce = AlignForceToWalkingAxis(
                        rightForce, calibratedRightWalkingAxis);
                    // Speed uses the calibrated physical force before the
                    // light-input sensitivity boost. This keeps 1x calibrated
                    // force equal to exactly 1x walking speed.
                    leftDexterSpeedForce = Mathf.Abs(leftForce.y);
                    rightDexterSpeedForce = Mathf.Abs(rightForce.y);
                    leftForce = ApplyDexterFingerSensitivity(leftForce);
                    rightForce = ApplyDexterFingerSensitivity(rightForce);
                    hasJumpForce = TryReadForce(
                        jumpFinger, out Vector2 measuredJumpForce);
                    if (hasJumpForce)
                        jumpForce = measuredJumpForce - thumbJumpBaseline;
                }
            }

            if (temporalSmoothingWindow > 0f)
            {
                float smoothingBlend = 1f - Mathf.Exp(-Time.deltaTime / temporalSmoothingWindow);
                smoothedLeftForce = Vector2.Lerp(smoothedLeftForce, leftForce, smoothingBlend);
                smoothedRightForce = Vector2.Lerp(smoothedRightForce, rightForce, smoothingBlend);
                smoothedLeftDexterSpeedForce =
                    UpdateDexterSpeedForceEnvelope(
                        smoothedLeftDexterSpeedForce,
                        leftDexterSpeedForce);
                smoothedRightDexterSpeedForce =
                    UpdateDexterSpeedForceEnvelope(
                        smoothedRightDexterSpeedForce,
                        rightDexterSpeedForce);
            }
            else
            {
                smoothedLeftForce = leftForce;
                smoothedRightForce = rightForce;
                smoothedLeftDexterSpeedForce = leftDexterSpeedForce;
                smoothedRightDexterSpeedForce = rightDexterSpeedForce;
            }

            if (usingPhysicalDexterInput &&
                dexterMovementCalibrationComplete)
            {
                // Axis calibration removes the repeatable diagonal angle.
                // Snap the small remaining lateral component completely to
                // zero so a walking press is visibly and numerically pure Y.
                // X-dominant input remains available for deliberate turning.
                smoothedLeftForce = SnapCalibratedWalkingForceToY(
                    smoothedLeftForce);
                smoothedRightForce = SnapCalibratedWalkingForceToY(
                    smoothedRightForce);
            }

            // Resolve intent before either movement system runs. Dexter finger
            // presses inevitably create coupled X force, so temporal Y
            // alternation owns locomotion even when that X is large enough to
            // satisfy the static turn thresholds.
            bool walkingIntentActive = UpdateWalkingIntentDetection(
                smoothedLeftForce, smoothedRightForce);
            float coherentTurnCandidate = GetCoherentTurnCandidate(
                smoothedLeftForce, smoothedRightForce);
            bool hasCoherentTurnCandidate =
                Mathf.Abs(coherentTurnCandidate) > 0f;
            int singleFingerLiftSide =
                walkingIntentActive || hasCoherentTurnCandidate
                ? 0
                : GetSingleFingerLiftSide(
                    smoothedLeftForce, smoothedRightForce);
            debugWalkingIntentActive = walkingIntentActive;
            debugCoherentTurnCandidate = hasCoherentTurnCandidate;
            debugInputIntent = walkingIntentActive
                ? "WALK"
                : singleFingerLiftSide == 1
                    ? "LEFT LIFT"
                    : singleFingerLiftSide == 2
                        ? "RIGHT LIFT"
                        : hasCoherentTurnCandidate
                            ? "TURN CANDIDATE"
                            : "NEUTRAL";
            bool isTurning = UpdateTurning(
                smoothedLeftForce, smoothedRightForce,
                walkingIntentActive);
            if (isTurning)
                debugInputIntent = "TURNING";
            UpdateAlternatingLocomotion(
                smoothedLeftForce, smoothedRightForce, isTurning,
                singleFingerLiftSide);
            if (hasJumpForce)
            {
                currentThumbAppliedForce = jumpForce;
                UpdateThumbJump(jumpForce);
            }
            else
            {
                currentThumbAppliedForce = Vector2.zero;
                jumpGateState = 1;
            }
            if (isJumping)
                ApplyJumpLocomotion();
            else
                ApplyLocomotion();

            UpdateAlternatingGait();
            if (isJumping)
                UpdateAirborneLegTargets();
        }

        private void UpdateThumbJump(Vector2 appliedForce)
        {
            float forceMagnitude = appliedForce.magnitude;
            float releaseForce = GetActiveJumpReleaseForce();
            float activationForce = GetActiveJumpActivationForce();

            if (isJumping)
            {
                // A release during flight must not pre-arm another jump. The
                // thumb has to be released deliberately after touchdown.
                jumpInputArmed = false;
                jumpReleaseBeganAt = -1f;
                jumpGateState = 5;
                return;
            }

            if (isSamplingJumpForce)
            {
                if (forceMagnitude > sampledJumpPeakForce)
                {
                    sampledJumpPeakForce = forceMagnitude;
                    sampledJumpPeakDirection = appliedForce;
                }
                if (!jumpPressQualified)
                {
                    if (forceMagnitude < activationForce)
                    {
                        isSamplingJumpForce = false;
                        jumpForceSamplingStartedAt = -1f;
                        sampledJumpPeakForce = 0f;
                        sampledJumpPeakDirection = Vector2.zero;
                        jumpPressQualified = false;
                        // This press never became a jump, so dropping back
                        // below activation must leave the detector ready for
                        // the next deliberate press. Requiring the much lower
                        // post-jump release threshold here caused permanent
                        // waiting after ordinary short thumb movements.
                        jumpInputArmed = true;
                        jumpGateState = 2;
                        return;
                    }
                    if (Time.time - jumpForceSamplingStartedAt <
                        jumpActivationHoldSeconds)
                    {
                        jumpGateState = 8;
                        return;
                    }
                    jumpPressQualified = true;
                }
                bool wasReleased = forceMagnitude <= releaseForce;
                bool samplingComplete = Time.time -
                    jumpForceSamplingStartedAt >=
                    jumpForceSamplingWindow;
                if (!wasReleased && !samplingComplete)
                {
                    jumpGateState = 6;
                    return;
                }

                float capturedPeak = sampledJumpPeakForce;
                Vector2 capturedDirection = sampledJumpPeakDirection;
                isSamplingJumpForce = false;
                jumpForceSamplingStartedAt = -1f;
                sampledJumpPeakForce = 0f;
                sampledJumpPeakDirection = Vector2.zero;
                jumpPressQualified = false;
                jumpInputArmed = wasReleased;
                if (capturedPeak < activationForce)
                {
                    jumpInputArmed = true;
                    jumpGateState = 2;
                    return;
                }

                float capturedStrength = Mathf.InverseLerp(
                    activationForce,
                    GetStrongJumpTriggerForce(),
                    capturedPeak);
                jumpGateState = 0;
                StartJump(
                    capturedPeak, capturedStrength, capturedDirection);
                return;
            }

            if (!jumpInputArmed)
            {
                if (forceMagnitude <= releaseForce)
                {
                    if (jumpReleaseBeganAt < 0f)
                        jumpReleaseBeganAt = Time.time;
                    if (Time.time - jumpReleaseBeganAt >=
                        jumpRearmReleaseHoldSeconds)
                    {
                        jumpInputArmed = true;
                        jumpReleaseBeganAt = -1f;
                    }
                }
                else
                {
                    jumpReleaseBeganAt = -1f;
                }
                jumpGateState = 3;
                return;
            }
            if (forceMagnitude < activationForce)
            {
                jumpGateState = 2;
                return;
            }
            if (Time.time - lastJumpLandedAt < jumpCooldown)
            {
                jumpGateState = 4;
                return;
            }

            // Do not commit distance on the noisy rising-edge sample. Capture
            // the peak of this deliberate press, or launch early on release.
            isSamplingJumpForce = true;
            jumpForceSamplingStartedAt = Time.time;
            sampledJumpPeakForce = forceMagnitude;
            sampledJumpPeakDirection = appliedForce;
            jumpPressQualified = jumpActivationHoldSeconds <= 0f;
            jumpGateState = jumpPressQualified ? 6 : 8;
        }

        private float GetStrongJumpTriggerForce()
        {
            float reference = dexterMovementCalibrationComplete
                ? calibratedThumbJumpReferenceForce
                : forceForMaximumJump;
            return Mathf.Max(
                GetActiveJumpActivationForce() + 0.001f,
                reference + strongJumpForceAboveCalibration);
        }

        private float GetActiveJumpActivationForce()
        {
            float walkingForce = Mathf.Abs(smoothedLeftForce.y) +
                                 Mathf.Abs(smoothedRightForce.y);
            return GetBaseJumpActivationForce() +
                   walkingForce * walkingInputJumpActivationMargin;
        }

        private float GetBaseJumpActivationForce()
        {
            float reference = dexterMovementCalibrationComplete
                ? calibratedThumbJumpReferenceForce
                : forceForMaximumJump;
            return Mathf.Max(
                minimumJumpForce,
                reference * calibratedJumpActivationFraction,
                calibratedThumbIncidentalForce *
                thumbIncidentalForceSafetyMultiplier);
        }

        private float GetActiveJumpReleaseForce()
        {
            return Mathf.Max(
                jumpReleaseForce,
                GetActiveJumpActivationForce() *
                jumpReleaseActivationFraction);
        }

        private void StartJump(
            float forceMagnitude,
            float strength,
            Vector2 forceDirection)
        {
            if (!hasLocomotionWorldPosition)
            {
                locomotionWorldPosition = transform.position;
                hasLocomotionWorldPosition = true;
            }

            jumpInputArmed = false;
            jumpReleaseBeganAt = -1f;
            isJumping = true;
            jumpId++;
            jumpEventThisFrame = 1;
            activeJumpForce = forceMagnitude;
            jumpStartedAt = Time.time;
            activeJumpDuration = Mathf.Lerp(
                minimumJumpDuration, maximumJumpDuration, strength);
            activeJumpHeight = Mathf.Lerp(
                minimumJumpHeight, maximumJumpHeight, strength);
            float jumpDistance = Mathf.Lerp(
                minimumJumpDistance, maximumJumpDistance, strength);
            activeJumpIsStrong = forceMagnitude >=
                                 GetStrongJumpTriggerForce();
            if (activeJumpIsStrong)
                jumpDistance *= strongJumpDistanceMultiplier;
            activeJumpDistance = jumpDistance;

            // A physics-owned jump starts from the actual body position, not
            // from a possibly leading procedural target.
            jumpStartPosition = transform.position;
            locomotionWorldPosition = jumpStartPosition;
            jumpUp = traceSupportNormal.sqrMagnitude > 0.25f
                ? traceSupportNormal.normalized
                : transform.up;
            if (jumpUp.sqrMagnitude < 0.25f)
                jumpUp = Vector3.up;
            Vector3 surfaceForward = terrainForces != null
                ? terrainForces.GetSurfaceTravelDirection(
                    GetTerrainSamplingPosition(), GetRigForward())
                : GetRigForward();
            surfaceForward = Vector3.ProjectOnPlane(
                surfaceForward, jumpUp).normalized;
            if (surfaceForward.sqrMagnitude < 0.25f)
                surfaceForward = Vector3.ProjectOnPlane(
                    GetRigForward(), jumpUp).normalized;

            Vector3 surfaceRight = Vector3.Cross(
                jumpUp, surfaceForward).normalized;
            Vector3 mappedForce = MapForce(forceDirection);
            float forwardForce = Mathf.Max(
                Mathf.Abs(mappedForce.z),
                Mathf.Abs(mappedForce.x) * 0.5f);
            jumpForward = Vector3.ProjectOnPlane(
                surfaceRight * mappedForce.x +
                surfaceForward * forwardForce,
                jumpUp).normalized;
            if (jumpForward.sqrMagnitude < 0.25f)
                jumpForward = surfaceForward;

            // Every jump retains forward travel, while thumb X steers left or
            // right. Persist the heading for locomotion after landing, but
            // visually turn toward it during the airborne arc.
            float jumpHeadingChange = Vector3.SignedAngle(
                surfaceForward, jumpForward, jumpUp);
            currentYawDegrees += jumpHeadingChange;
            jumpStartRotation = transform.rotation;
            jumpTargetRotation = terrainForces != null
                ? terrainForces.GetSlopeAlignedRootRotation(
                    GetTerrainSamplingPosition(),
                    transform.rotation,
                    jumpForward)
                : Quaternion.LookRotation(jumpForward, jumpUp);

            jumpLandingPosition = jumpStartPosition +
                                  jumpForward * jumpDistance;
            if (terrainForces != null &&
                terrainForces.TryGetTerrainSurfaceFrame(
                    jumpLandingPosition,
                    out Vector3 landingSurface,
                    out Vector3 landingNormal))
            {
                jumpLandingPosition = landingSurface +
                    landingNormal * terrainRootClearance;
            }

            if (rigidbodyDynamics != null &&
                rigidbodyDynamics.isActiveAndEnabled)
            {
                // Thumb force selects distance/height; the Rigidbody receives
                // exactly one velocity-change impulse. No airborne position
                // drive is allowed to add energy afterward.
                activeJumpDuration = rigidbodyDynamics.ApplyJumpImpulse(
                    jumpForward,
                    jumpUp,
                    activeJumpDistance,
                    activeJumpHeight);
            }

            // The jump owns root translation until landing. Discard walking
            // distance accumulated in the air so it cannot cause a landing pop.
            forwardSpeed = 0f;
            lastAppliedForwardDistance = currentForwardDistance;
            gaitDriveActive = false;
            hasClimbSurfaceAnchor = false;
            rootPositionTransitionActive = false;
            CaptureRecoveryStarts(leftLegs);
            CaptureRecoveryStarts(rightLegs);
        }

        private void ApplyJumpLocomotion()
        {
            if (rigidbodyDynamics != null &&
                rigidbodyDynamics.isActiveAndEnabled)
            {
                ApplyImpulseJumpLocomotion();
                return;
            }

            float duration = Mathf.Max(0.1f, activeJumpDuration);
            float progress = Mathf.Clamp01(
                (Time.time - jumpStartedAt) / duration);
            float travelProgress = SmoothStep01(progress);
            Vector3 basePosition = Vector3.Lerp(
                jumpStartPosition, jumpLandingPosition, travelProgress);
            Vector3 desiredPosition = basePosition + jumpUp *
                (4f * activeJumpHeight * progress * (1f - progress));
            Vector3 resolvedPosition = ResolveObstacleMovement(
                locomotionWorldPosition, desiredPosition);
            resolvedPosition = EnforceJumpSurfaceClearance(
                resolvedPosition);
            if ((resolvedPosition - desiredPosition).sqrMagnitude > 0.0001f)
            {
                // Retarget the landing to the blocked side of a solid object;
                // otherwise the final frame could bypass the collision guard.
                jumpLandingPosition.x = resolvedPosition.x;
                jumpLandingPosition.z = resolvedPosition.z;
            }
            if (rigidbodyDynamics != null &&
                rigidbodyDynamics.isActiveAndEnabled)
            {
                // The physics lead limit is not an obstacle. Apply it only
                // after obstacle retargeting so it cannot shorten the jump's
                // actual landing destination every frame.
                resolvedPosition = rigidbodyDynamics.ConstrainDesiredPosition(
                    resolvedPosition, true);
            }
            locomotionWorldPosition = resolvedPosition;
            Quaternion airborneRotation = Quaternion.Slerp(
                jumpStartRotation,
                jumpTargetRotation,
                travelProgress);
            ApplyPhysicsOwnedRootPose(
                resolvedPosition,
                airborneRotation,
                jumpUp,
                0f,
                true);
            lastAppliedForwardDistance = currentForwardDistance;

            if (progress < 1f)
                return;

            isJumping = false;
            lastJumpLandedAt = Time.time;
            PrepareImmediatePostJumpGait();
            Vector3 finalLandingPosition = resolvedPosition;
            if (terrainForces != null &&
                terrainForces.TryGetContinuousRootSurfacePose(
                    resolvedPosition,
                    terrainRootClearance,
                    out _,
                    out _,
                    out _,
                    out Vector3 clearedLandingRoot,
                    out _))
            {
                finalLandingPosition = ResolveObstacleMovement(
                    resolvedPosition, clearedLandingRoot);
            }
            if (rigidbodyDynamics != null &&
                rigidbodyDynamics.isActiveAndEnabled)
                finalLandingPosition =
                    rigidbodyDynamics.ConstrainDesiredPosition(
                        finalLandingPosition, true);
            jumpLandingPosition = finalLandingPosition;
            locomotionWorldPosition = finalLandingPosition;
            jumpTargetRotation = terrainForces != null
                ? terrainForces.GetSlopeAlignedRootRotation(
                    GetTerrainSamplingPosition(),
                    jumpTargetRotation,
                    jumpForward)
                : Quaternion.LookRotation(jumpForward, jumpUp);
            ApplyPhysicsOwnedRootPose(
                finalLandingPosition,
                jumpTargetRotation,
                jumpUp,
                0f,
                false);
            // Make the in-flight facing the new neutral root orientation.
            // The following walking frame therefore begins from exactly the
            // landing heading instead of reconstructing the pre-jump heading.
            rootLocalRotation = transform.parent != null
                ? Quaternion.Inverse(transform.parent.rotation) * jumpTargetRotation
                : jumpTargetRotation;
            currentYawDegrees = 0f;
            lastCompletedJumpDistance = Vector3.ProjectOnPlane(
                transform.position - jumpStartPosition,
                jumpUp).magnitude;
            jumpEventThisFrame = 2;
            jumpGateState = 3;
            PlaceFeetAtCurrentStance(leftLegs);
            PlaceFeetAtCurrentStance(rightLegs);
        }

        private void ApplyImpulseJumpLocomotion()
        {
            float elapsed = Time.time - jumpStartedAt;
            float duration = Mathf.Max(0.2f, activeJumpDuration);
            float progress = Mathf.Clamp01(elapsed / duration);
            float travelProgress = SmoothStep01(progress);
            Quaternion airborneRotation = Quaternion.Slerp(
                jumpStartRotation,
                jumpTargetRotation,
                travelProgress);

            // The Rigidbody is authoritative throughout the flight. This
            // target carries only orientation and the takeoff-surface gravity
            // direction; it never chases the old procedural parabola.
            locomotionWorldPosition = transform.position;
            rigidbodyDynamics.SetDesiredPose(
                transform.position,
                airborneRotation,
                jumpUp,
                0f,
                true);
            lastAppliedForwardDistance = currentForwardDistance;

            Rigidbody jumpBody = rigidbodyDynamics.PhysicsBody;
            bool descending = jumpBody == null ||
                Vector3.Dot(jumpBody.linearVelocity, jumpUp) <= 0f;
            bool reachedSurface = progress >= 0.80f && descending &&
                (rigidbodyDynamics.HasBodySupportContact(jumpUp) ||
                 rigidbodyDynamics.HasSupportContact(jumpUp));
            bool safetyTimeout = elapsed >= duration + 1.25f;
            if (!reachedSurface && !safetyTimeout)
                return;

            isJumping = false;
            lastJumpLandedAt = Time.time;
            PrepareImmediatePostJumpGait();
            rigidbodyDynamics.CompleteJumpLanding(jumpUp);
            Vector3 actualLandingPosition = transform.position;
            Vector3 groundedTarget = actualLandingPosition;
            Vector3 landingNormal = jumpUp;
            if (terrainForces != null &&
                terrainForces.TryGetContinuousRootSurfacePose(
                    actualLandingPosition,
                    terrainRootClearance,
                    out _,
                    out _,
                    out Vector3 clearedSupportNormal,
                    out Vector3 clearedLandingRoot,
                    out _))
            {
                groundedTarget = ResolveObstacleMovement(
                    actualLandingPosition,
                    clearedLandingRoot);
                landingNormal = clearedSupportNormal;
            }

            jumpLandingPosition = actualLandingPosition;
            locomotionWorldPosition = groundedTarget;
            jumpTargetRotation = terrainForces != null
                ? terrainForces.GetSlopeAlignedRootRotation(
                    GetTerrainSamplingPosition(),
                    jumpTargetRotation,
                    jumpForward)
                : Quaternion.LookRotation(jumpForward, landingNormal);
            rigidbodyDynamics.SetDesiredPose(
                groundedTarget,
                jumpTargetRotation,
                landingNormal,
                0f,
                false);
            rootLocalRotation = transform.parent != null
                ? Quaternion.Inverse(transform.parent.rotation) *
                  jumpTargetRotation
                : jumpTargetRotation;
            currentYawDegrees = 0f;
            lastCompletedJumpDistance = Vector3.ProjectOnPlane(
                actualLandingPosition - jumpStartPosition,
                jumpUp).magnitude;
            jumpEventThisFrame = 2;
            jumpGateState = 3;
            PlaceFeetAtCurrentStance(leftLegs);
            PlaceFeetAtCurrentStance(rightLegs);
        }

        private void PrepareImmediatePostJumpGait()
        {
            jumpInputArmed = false;
            jumpReleaseBeganAt = -1f;
            forwardSpeed = 0f;
            leftGaitWeight = 0f;
            rightGaitWeight = 0f;
            leftSideWaveInPlace = false;
            rightSideWaveInPlace = false;
            leftFingerMimicLift = 0f;
            rightFingerMimicLift = 0f;
            currentLeftLegPushDistance = 0f;
            currentRightLegPushDistance = 0f;
            pressedFinger = 0;
            targetLeftGaitPhase = leftGaitPhase;
            targetRightGaitPhase = rightGaitPhase;
            lastAppliedForwardDistance = currentForwardDistance;
        }

        private Vector3 EnforceJumpSurfaceClearance(Vector3 position)
        {
            if (terrainForces == null ||
                !terrainForces.TryGetTerrainSurfaceFrame(
                    position,
                    out Vector3 surfacePoint,
                    out Vector3 surfaceNormal))
                return position;

            surfaceNormal = surfaceNormal.normalized;
            float currentClearance = Vector3.Dot(
                position - surfacePoint, surfaceNormal);
            float missingClearance = terrainRootClearance -
                                     currentClearance;
            return missingClearance > 0f
                ? position + surfaceNormal * missingClearance
                : position;
        }

        private void UpdateAirborneLegTargets()
        {
            float progress = Mathf.Clamp01(
                (Time.time - jumpStartedAt) /
                Mathf.Max(0.1f, activeJumpDuration));
            float poseOut = 1f - SmoothStep01(
                Mathf.InverseLerp(0.72f, 1f, progress));
            float poseStrength = poseOut;
            PoseAirborneSide(leftLegs, poseStrength);
            PoseAirborneSide(rightLegs, poseStrength);
        }

        private void PoseAirborneSide(LegChain[] legs, float strength)
        {
            if (legs == null)
                return;

            for (int row = 0; row < legs.Length; row++)
            {
                LegChain leg = legs[row];
                Vector3 naturalTarget = rigSpace.TransformPoint(
                    leg.RestRigLocalTarget);
                float rearWeight = row == legs.Length - 1
                    ? 1f
                    : row == legs.Length - 2 ? 0.45f : 0f;
                Vector3 stretchedTarget = naturalTarget;
                if (rearWeight > 0f && leg.Joints.Length > 0)
                {
                    // Build the airborne reach from the hip attached to the
                    // torso. A distant target along this complete-chain line
                    // makes every joint open together instead of moving only
                    // the lower leg beyond its knee.
                    Vector3 hip = leg.Joints[0].position;
                    Vector3 authoredDirection =
                        (naturalTarget - hip).normalized;
                    Vector3 torsoOutward = Vector3.ProjectOnPlane(
                        hip - body.position, jumpUp).normalized;
                    Vector3 straightDirection =
                        (authoredDirection + torsoOutward * 0.45f -
                         jumpForward * (0.65f * rearWeight)).normalized;
                    if (straightDirection.sqrMagnitude < 0.25f)
                        straightDirection = authoredDirection;
                    float fullLegLength = GetLegChainLength(leg);
                    float straightReach = fullLegLength +
                        airborneRearLegStretch * rearWeight * strength;
                    stretchedTarget = hip +
                                      straightDirection * straightReach;
                }
                // The target may sit just beyond ordinary stance reach so the
                // fixed-length IK chain straightens instead of remaining bent.
                leg.WorldTarget = stretchedTarget;
                leg.WasSwinging = false;
                leg.IsRecoveringLag = false;
            }
        }

        private static float GetLegChainLength(LegChain leg)
        {
            if (leg == null || leg.Joints == null ||
                leg.Joints.Length == 0 || leg.Effector == null)
                return 0f;

            float length = 0f;
            for (int i = 0; i < leg.Joints.Length - 1; i++)
                length += Vector3.Distance(
                    leg.Joints[i].position,
                    leg.Joints[i + 1].position);
            length += Vector3.Distance(
                leg.Joints[leg.Joints.Length - 1].position,
                leg.Effector.position);
            return length;
        }

        private void PlaceFeetAtCurrentStance(LegChain[] legs)
        {
            if (legs == null)
                return;

            for (int i = 0; i < legs.Length; i++)
            {
                LegChain leg = legs[i];
                Vector3 target = rigSpace.TransformPoint(
                    leg.RestRigLocalTarget);
                if (TryGetFootSurfaceTarget(
                        target, leg.GroundClearance,
                        out Vector3 groundedTarget, out _))
                    target = groundedTarget;
                leg.WorldTarget = target;
                leg.SwingStartWorldTarget = target;
                leg.SwingEndWorldTarget = target;
                leg.WasSwinging = false;
                leg.IsRecoveringLag = false;
                leg.LagRecoveryProgress = 0f;
            }
        }

        private void UpdateAlternatingLocomotion(
            Vector2 leftForce, Vector2 rightForce, bool isTurning,
            int singleFingerLiftSide)
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
            if (pressedFinger != 0)
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

            if (pressedFinger == 0 && activeFinger == 0 &&
                leftActivity >= leftAdaptiveActivationThreshold &&
                leftActivity >= rightActivity + alternationDominanceMargin)
            {
                activeFinger = 1;
            }
            else if (pressedFinger == 0 && activeFinger == 0 &&
                     rightActivity >= rightAdaptiveActivationThreshold &&
                     rightActivity >= leftActivity + alternationDominanceMargin)
            {
                activeFinger = 2;
            }

            if (pressIntervalElapsed && activeFinger != 0 && activeFinger != pressedFinger)
            {
                RegisterFingerPress(activeFinger);
                pressedFinger = activeFinger;
            }

            // Alternation only unlocks walking. Live displacement—not the
            // alternation interval—drives speed and stride on every frame.
            float liveDisplacement = IsUsingCalibratedDexterInput()
                ? Mathf.Max(
                    smoothedLeftDexterSpeedForce,
                    smoothedRightDexterSpeedForce)
                : Mathf.Max(leftActivity, rightActivity);
            bool hasLiveFingerInput = leftActivity >= leftAdaptiveActivationThreshold ||
                                      rightActivity >= rightAdaptiveActivationThreshold;
            if (hasLiveFingerInput)
                lastLiveFingerInputTime = Time.time;
            else if (leftActivity <= leftAdaptiveReleaseThreshold &&
                     rightActivity <= rightAdaptiveReleaseThreshold)
            {
                // Direct side lifts must end with the physical press, not with
                // the longer gait release grace period.
                leftSideWaveInPlace = false;
                rightSideWaveInPlace = false;
            }

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
                              !isJumping &&
                              !isTurning &&
                              !isRecoveringFromTurn;

            if (singleFingerLiftSide != 0 && !isTurning)
            {
                // An isolated finger directly owns its matching side. It can
                // prepare/lift the legs and influence body support, but it does
                // not authorize forward locomotion by itself.
                leftSideWaveInPlace = singleFingerLiftSide == 1;
                rightSideWaveInPlace = singleFingerLiftSide == 2;
                gaitDriveActive = false;
            }

            float activeMinimumSpeedDisplacement =
                GetActiveMinimumSpeedDisplacement();
            float leftMimicTarget = leftSideWaveInPlace
                ? Mathf.InverseLerp(0f, activeMinimumSpeedDisplacement,
                    leftActivity) * fingerMimicLiftHeight
                : 0f;
            float rightMimicTarget = rightSideWaveInPlace
                ? Mathf.InverseLerp(0f, activeMinimumSpeedDisplacement,
                    rightActivity) * fingerMimicLiftHeight
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
            if (IsUsingCalibratedDexterInput())
            {
                // Let stronger input increase the distance carried by each
                // planted step first. Cadence supplies only the remaining
                // speed so the legs do not jitter at the high-force endpoint.
                float baselineStride = Mathf.Max(
                    0.001f, minimumForceStrideLength);
                requestedGaitPhaseSpeed *= baselineStride /
                    Mathf.Max(baselineStride, activeForceStrideLength);
                requestedGaitPhaseSpeed = Mathf.Min(
                    requestedGaitPhaseSpeed,
                    dexterMaximumGaitCyclesPerSecond);
            }
            float phaseSpeedBlend = 1f - Mathf.Exp(
                -gaitBlendSpeed * Time.deltaTime);
            gaitPhaseSpeed = Mathf.Lerp(
                gaitPhaseSpeed, requestedGaitPhaseSpeed, phaseSpeedBlend);
            activeForceLiftMultiplier = Mathf.Lerp(
                activeForceLiftMultiplier,
                GetForceLiftMultiplier(liveDisplacement),
                phaseSpeedBlend);
            float locomotionStride =
                GetActiveForceDrivenStrideLength() * walkingStrideEmphasis;
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
            bool isAccelerating = Mathf.Abs(requestedForwardSpeed) >
                                  Mathf.Abs(forwardSpeed);
            float walkingSpeedResponseScale =
                IsUsingCalibratedDexterInput()
                    ? (isAccelerating
                        ? dexterWalkingAccelerationResponseScale
                        : dexterWalkingBrakingResponseScale)
                    : 1f;
            forwardSpeed = terrainForces != null
                ? terrainForces.MoveWalkingSpeed(
                    forwardSpeed,
                    requestedForwardSpeed,
                    Time.deltaTime * walkingSpeedResponseScale)
                : Mathf.MoveTowards(
                    forwardSpeed,
                    requestedForwardSpeed,
                    Time.deltaTime * 6f *
                    walkingSpeedResponseScale);
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
            bool consistentAlternation = completesAlternation &&
                lastConfirmedWalkingPatternTime >= 0f &&
                now - lastConfirmedWalkingPatternTime <=
                establishedWalkingIntentHoldSeconds;

            if (activeFinger == 1)
            {
                leftSideWaveInPlace = !consistentAlternation;
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
                rightSideWaveInPlace = !consistentAlternation;
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
            if (consistentAlternation)
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
            if (IsUsingCalibratedDexterInput())
            {
                float reference = Mathf.Max(
                    0.001f, calibratedDexterReferenceForce);
                float forceRatio = Mathf.Max(0f, fingerForce) / reference;
                if (forceRatio <= 1f)
                {
                    return Mathf.Lerp(
                        dexterMinimumWalkingSpeedMultiplier *
                        dexterBaselineWalkingSpeedMultiplier,
                        dexterBaselineWalkingSpeedMultiplier,
                        Mathf.Clamp01(forceRatio));
                }

                return Mathf.Lerp(
                    dexterBaselineWalkingSpeedMultiplier,
                    dexterHighForceWalkingSpeedMultiplier,
                    Mathf.InverseLerp(
                        1f,
                        dexterMaximumSpeedForceMultiplier,
                        forceRatio));
            }

            float displacementStrength = GetForceDisplacementStrength(
                fingerForce);
            return Mathf.Lerp(
                minimumStepSpeedMultiplier,
                maximumStepSpeedMultiplier,
                displacementStrength);
        }

        private float GetForceStrideLength(float fingerForce)
        {
            if (IsUsingCalibratedDexterInput())
            {
                float reference = Mathf.Max(
                    0.001f, calibratedDexterReferenceForce);
                float forceRatio = Mathf.Max(0f, fingerForce) / reference;
                if (forceRatio <= 1f)
                {
                    return Mathf.Lerp(
                        minimumForceStrideLength,
                        dexterBaselineStrideLength,
                        Mathf.Clamp01(forceRatio));
                }

                return Mathf.Lerp(
                    dexterBaselineStrideLength,
                    dexterHighForceStrideLength,
                    Mathf.InverseLerp(
                        1f,
                        dexterMaximumSpeedForceMultiplier,
                        forceRatio));
            }

            float displacementStrength = GetForceDisplacementStrength(
                fingerForce);
            return Mathf.Lerp(
                minimumForceStrideLength,
                maximumForceStrideLength,
                displacementStrength);
        }

        private float GetActiveForceDrivenStrideLength()
        {
            float maximumReach = IsUsingCalibratedDexterInput()
                ? Mathf.Min(
                    dexterHighForceStrideLength,
                    maximumForeAftMovement)
                : maximumStrideLength;
            return Mathf.Clamp(
                Mathf.Max(minimumStrideLength, activeForceStrideLength),
                minimumStrideLength,
                Mathf.Max(minimumStrideLength, maximumReach));
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
                float gaitTravelStride =
                    GetActiveForceDrivenStrideLength() *
                    walkingStrideEmphasis;
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

            float strideLength = GetActiveForceDrivenStrideLength();
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

        private void ResetWalkingIntentDetection()
        {
            lastAlternatingYIntentTime = -1f;
            alternatingYIntentConfidence = 0f;
            previousLeftYActivity = 0f;
            previousRightYActivity = 0f;
            hasPreviousYActivity = false;
            previousYDominantFinger = 0;
            lastYDominanceEventTime = -1f;
            lastQualifiedYHandoffTime = -1f;
            lastConfirmedWalkingPatternTime = -1f;
        }

        private int GetSingleFingerLiftSide(
            Vector2 leftForce, Vector2 rightForce)
        {
            float leftActivity = Mathf.Abs(leftForce.y);
            float rightActivity = Mathf.Abs(rightForce.y);
            bool leftActive = leftActivity >= leftAdaptiveActivationThreshold;
            bool rightActive = rightActivity >= rightAdaptiveActivationThreshold;
            float isolationRatio = Mathf.Clamp(
                singleFingerIsolationRatio, 0.05f, 0.75f);

            bool leftIsolated = leftActive &&
                (!rightActive || rightActivity <= leftActivity * isolationRatio);
            bool rightIsolated = rightActive &&
                (!leftActive || leftActivity <= rightActivity * isolationRatio);
            if (leftIsolated == rightIsolated)
                return 0;
            return leftIsolated ? 1 : 2;
        }

        private bool UpdateWalkingIntentDetection(
            Vector2 leftForce, Vector2 rightForce)
        {
            float leftActivity = Mathf.Abs(leftForce.y);
            float rightActivity = Mathf.Abs(rightForce.y);
            float minimumDelta = Mathf.Max(
                0.001f,
                calibratedDexterReferenceForce * walkingIntentDeltaFraction);
            bool alternatingMotion = false;

            if (hasPreviousYActivity)
            {
                float leftDelta = leftActivity - previousLeftYActivity;
                float rightDelta = rightActivity - previousRightYActivity;
                alternatingMotion =
                    Mathf.Abs(leftDelta) >= minimumDelta &&
                    Mathf.Abs(rightDelta) >= minimumDelta &&
                    leftDelta * rightDelta < 0f;
            }

            int dominantFinger = 0;
            if (leftActivity >= leftAdaptiveActivationThreshold &&
                leftActivity >= rightActivity + alternationDominanceMargin)
                dominantFinger = 1;
            else if (rightActivity >= rightAdaptiveActivationThreshold &&
                     rightActivity >= leftActivity + alternationDominanceMargin)
                dominantFinger = 2;

            bool bothFingersReleased =
                leftActivity <= leftAdaptiveReleaseThreshold &&
                rightActivity <= rightAdaptiveReleaseThreshold;
            if (bothFingersReleased)
            {
                // Do not connect a new isolated press to a dominant finger
                // remembered from an earlier walk. After neutral, the first
                // finger is a side-lift gesture until a new alternation occurs.
                previousYDominantFinger = 0;
                lastYDominanceEventTime = -1f;
                lastQualifiedYHandoffTime = -1f;
                lastConfirmedWalkingPatternTime = -1f;
                alternatingYIntentConfidence = 0f;
                lastAlternatingYIntentTime = -1f;
                lastCompletedAlternationTime = -1f;
                alternatingDriveAuthorized = false;
            }

            bool dominanceChanged = dominantFinger != 0 &&
                previousYDominantFinger != 0 &&
                dominantFinger != previousYDominantFinger;
            bool dominanceChangedInTime = dominanceChanged &&
                lastYDominanceEventTime >= 0f &&
                Time.time - lastYDominanceEventTime <=
                maximumAlternationInterval;
            if (dominantFinger != 0 &&
                (previousYDominantFinger == 0 || dominanceChanged))
                lastYDominanceEventTime = Time.time;
            bool completesConsistentPattern = dominanceChangedInTime &&
                lastQualifiedYHandoffTime >= 0f &&
                Time.time - lastQualifiedYHandoffTime <=
                maximumAlternationInterval;
            if (dominanceChangedInTime)
            {
                lastQualifiedYHandoffTime = Time.time;
                if (completesConsistentPattern)
                    lastConfirmedWalkingPatternTime = Time.time;
            }
            if (dominantFinger != 0)
                previousYDominantFinger = dominantFinger;

            bool hasEstablishedWalkingPattern =
                lastConfirmedWalkingPatternTime >= 0f &&
                Time.time - lastConfirmedWalkingPatternTime <=
                establishedWalkingIntentHoldSeconds;
            alternatingMotion &= hasEstablishedWalkingPattern;

            float confirmationTime = Mathf.Max(
                0.02f, walkingIntentConfirmationSeconds);
            if (alternatingMotion)
            {
                alternatingYIntentConfidence = Mathf.MoveTowards(
                    alternatingYIntentConfidence, 1f,
                    Time.deltaTime / confirmationTime);
            }
            else
            {
                // Decay more slowly than confidence rises so the zero-velocity
                // peaks of a deliberate finger cycle do not break detection.
                alternatingYIntentConfidence = Mathf.MoveTowards(
                    alternatingYIntentConfidence, 0f,
                    Time.deltaTime / (confirmationTime * 2f));
            }

            if (completesConsistentPattern)
                alternatingYIntentConfidence = 1f;

            if (alternatingYIntentConfidence >= 0.75f)
            {
                lastAlternatingYIntentTime = Time.time;
                lastConfirmedWalkingPatternTime = Time.time;
                // The rolling detector is deliberately allowed to authorize
                // the same gait as a discrete finger swap. This avoids losing
                // walking when both loaded fingers never cross their release
                // thresholds, which is common with the physical Dexter device.
                alternatingDriveAuthorized = true;
                lastCompletedAlternationTime = Time.time;
                leftSideWaveInPlace = false;
                rightSideWaveInPlace = false;
            }

            previousLeftYActivity = leftActivity;
            previousRightYActivity = rightActivity;
            hasPreviousYActivity = true;

            float intentHold = Mathf.Max(
                establishedWalkingIntentHoldSeconds,
                walkingPriorityAfterAlternation);
            bool rollingAlternationIsRecent =
                lastAlternatingYIntentTime >= 0f &&
                Time.time - lastAlternatingYIntentTime <= intentHold;
            bool discreteAlternationIsRecent =
                lastCompletedAlternationTime >= 0f &&
                Time.time - lastCompletedAlternationTime <= intentHold;
            return rollingAlternationIsRecent ||
                   (hasEstablishedWalkingPattern &&
                    discreteAlternationIsRecent);
        }

        private bool UpdateTurning(
            Vector2 leftForce, Vector2 rightForce,
            bool walkingIntentActive)
        {
            float requestedTurn = 0f;
            float activeTurnActivationForce =
                GetActiveTurnActivationForce();
            float candidateTurn = GetCoherentTurnCandidate(
                leftForce, rightForce);

            if (candidateTurn == 0f)
            {
                debugTurnDecision = walkingIntentActive
                    ? "Walking: no coherent constant X takeover"
                    : "Rejected: X is not coherent on both fingers";
            }

            int candidateDirection = candidateTurn > 0f
                ? 1
                : candidateTurn < 0f ? -1 : 0;
            float candidateMagnitude = Mathf.Abs(candidateTurn);
            if (candidateDirection == 0)
            {
                pendingTurnDirection = 0;
                pendingTurnStartedAt = -1f;
                pendingTurnReferenceMagnitude = 0f;
            }
            else if (candidateDirection != pendingTurnDirection)
            {
                pendingTurnDirection = candidateDirection;
                pendingTurnStartedAt = Time.time;
                pendingTurnReferenceMagnitude = candidateMagnitude;
                debugTurnDecision = "Confirming constant X";
            }
            else
            {
                float permittedVariation = Mathf.Max(
                    activeTurnActivationForce * 0.35f,
                    pendingTurnReferenceMagnitude *
                    turnConstantForceTolerance);
                if (candidateMagnitude <
                    pendingTurnReferenceMagnitude - permittedVariation)
                {
                    // A substantial drop marks the end of an X pulse and must
                    // prove itself again. A same-direction rise is expected
                    // when the user begins pressing to turn, so it must not
                    // restart the confirmation timer.
                    pendingTurnStartedAt = Time.time;
                    pendingTurnReferenceMagnitude = candidateMagnitude;
                    debugTurnDecision =
                        "Waiting: X dropped too quickly";
                }
                else
                {
                    pendingTurnReferenceMagnitude = Mathf.Lerp(
                        pendingTurnReferenceMagnitude,
                        candidateMagnitude,
                        1f - Mathf.Exp(-5f * Time.deltaTime));
                    float requiredHold = walkingIntentActive
                        ? Mathf.Max(
                            turnIntentHoldSeconds,
                            turnWalkingTakeoverSeconds)
                        : turnIntentHoldSeconds;
                    if (pendingTurnStartedAt >= 0f &&
                        Time.time - pendingTurnStartedAt >=
                        requiredHold)
                    {
                        requestedTurn = candidateTurn;
                        if (walkingIntentActive)
                        {
                            // A proven constant-X gesture may take control
                            // directly instead of waiting for the full
                            // established-walk ownership window to expire.
                            ResetWalkingIntentDetection();
                            alternatingDriveAuthorized = false;
                            gaitDriveActive = false;
                            lastCompletedAlternationTime = -1f;
                            debugTurnDecision =
                                "Accepted: constant X took over from walk";
                        }
                        else
                            debugTurnDecision =
                                "Accepted: constant X turn";
                    }
                    else
                        debugTurnDecision = walkingIntentActive
                            ? "Confirming constant X takeover from walk"
                            : "Confirming constant X";
                }
            }

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
            bool turnIsSlowing =
                Mathf.Abs(requestedSpeed) < Mathf.Abs(currentTurnSpeed) ||
                requestedSpeed != 0f &&
                currentTurnSpeed != 0f &&
                Mathf.Sign(requestedSpeed) != Mathf.Sign(currentTurnSpeed);
            float angularResponse = turnIsSlowing
                ? turnAngularDeceleration
                : turnAngularAcceleration;
            currentTurnSpeed = Mathf.MoveTowards(
                currentTurnSpeed,
                requestedSpeed,
                angularResponse * Time.deltaTime);
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

        private float GetCoherentTurnCandidate(
            Vector2 leftForce, Vector2 rightForce)
        {
            float xSign = invertFx ? -1f : 1f;
            float leftX = leftForce.x * xSign;
            float rightX = rightForce.x * xSign;
            float activation = GetActiveTurnActivationForce();
            bool leftReady = Mathf.Abs(leftX) >= activation &&
                Mathf.Abs(leftX) >=
                Mathf.Abs(leftForce.y) * turnAxisDominanceRatio;
            bool rightReady = Mathf.Abs(rightX) >= activation &&
                Mathf.Abs(rightX) >=
                Mathf.Abs(rightForce.y) * turnAxisDominanceRatio;
            if (leftReady && rightReady &&
                Mathf.Sign(leftX) == Mathf.Sign(rightX))
            {
                return Mathf.Sign(leftX) *
                       Mathf.Min(Mathf.Abs(leftX), Mathf.Abs(rightX));
            }
            return 0f;
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
                        if (rootPositionTransitionActive)
                            naturalLanding = ConstrainTargetReach(
                                leg, naturalLanding);
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
                if (rootPositionTransitionActive)
                    leg.WorldTarget = ConstrainTargetReach(
                        leg, leg.WorldTarget);

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
            if (rigidbodyDynamics != null &&
                rigidbodyDynamics.isActiveAndEnabled)
                locomotionWorldPosition =
                    rigidbodyDynamics.ConstrainDesiredPosition(
                        locomotionWorldPosition);

            Quaternion desiredLocalRotation = rootLocalRotation *
                Quaternion.Euler(0f, currentYawDegrees, 0f);
            Quaternion desiredRootRotation = transform.parent != null
                ? transform.parent.rotation * desiredLocalRotation
                : desiredLocalRotation;
            Vector3 commandedForward = desiredRootRotation * Vector3.forward;
            Vector3 commandedRight = desiredRootRotation * Vector3.right;
            Vector3 movementStart = locomotionWorldPosition;
            bool hadSurfaceMotionAnchor = hasClimbSurfaceAnchor;
            Vector3 previousSurfaceMotionAnchor = climbSurfaceAnchor;
            float forwardDelta = currentForwardDistance - lastAppliedForwardDistance;
            Vector3 surfaceSample = GetTerrainSamplingPosition();
            float terrainMovementMultiplier = terrainForces != null
                ? terrainForces.GetMovementMultiplier(surfaceSample)
                : 1f;
            Vector3 surfaceForward = terrainForces != null
                ? terrainForces.GetSurfaceTravelDirection(
                    surfaceSample, commandedForward)
                : commandedForward;
            Vector3 movedSurfaceSample = surfaceSample + surfaceForward *
                (forwardDelta * terrainMovementMultiplier);
            lastAppliedForwardDistance = currentForwardDistance;
            // Passive transform-space gravity is the legacy fallback. A
            // dynamic body receives gravity, friction, and adhesion in
            // FixedUpdate and must not have them added to its target as well.
            if (terrainForces != null &&
                (rigidbodyDynamics == null ||
                 !rigidbodyDynamics.isActiveAndEnabled))
                movedSurfaceSample = terrainForces.ApplyForces(
                    movedSurfaceSample);

            Vector3 surfacePoint = movedSurfaceSample;
            Vector3 surfaceNormal = Vector3.up;
            Vector3 supportNormal = Vector3.up;
            Vector3 surfaceClearedRoot = movedSurfaceSample;
            bool isCornerTransition = false;
            bool foundSurfaceFrame = terrainForces != null &&
                terrainForces.TryGetContinuousRootSurfacePose(
                    movedSurfaceSample,
                    terrainRootClearance,
                    out surfacePoint,
                    out surfaceNormal,
                    out supportNormal,
                    out surfaceClearedRoot,
                    out isCornerTransition);
            bool enteredClimb = foundSurfaceFrame &&
                terrainForces.IsClimbableSurfaceNormal(surfaceNormal);
            bool anticipatedClimbTransition = foundSurfaceFrame &&
                !enteredClimb &&
                terrainForces.ShouldAnticipateClimbTransition(
                    surfaceSample,
                    commandedForward,
                    surfaceNormal, supportNormal);
            if (foundSurfaceFrame && enteredClimb != wasOnClimbSurface)
                BeginSurfaceTransition(enteredClimb);
            else if (anticipatedClimbTransition &&
                     !rootPositionTransitionActive)
                BeginSurfaceTransition(true, true);
            else if (foundSurfaceFrame && isCornerTransition &&
                     !rootPositionTransitionActive)
                BeginSurfaceTransition(enteredClimb);
            wasOnClimbSurface = enteredClimb;

            bool useSurfaceMotionAnchor = enteredClimb ||
                                          isCornerTransition ||
                                          rootPositionTransitionActive;
            if (foundSurfaceFrame && useSurfaceMotionAnchor)
            {
                // Keep sampling from the actual surface point throughout the
                // corner. Sampling from the clearance-offset root would add
                // that offset again on the next frame and create lateral drift.
                hasClimbSurfaceAnchor = true;
                Vector3 clearedMovementTarget = surfaceClearedRoot;
                if (rootPositionTransitionActive)
                {
                    // A heightmap can change from floor to wall (or back) in
                    // one sample. The clearance solver must move the root out
                    // of both faces, but applying that entire correction in a
                    // single frame looks like a teleport. Follow the safe pose
                    // with a frame-rate-independent response so the torso
                    // rounds the edge while the gait continues advancing.
                    float cornerBlend = 1f - Mathf.Exp(
                        -cornerRootPositionResponse * Time.deltaTime);
                    Vector3 responsiveTarget = Vector3.Lerp(
                        movementStart, surfaceClearedRoot, cornerBlend);
                    clearedMovementTarget = Vector3.MoveTowards(
                        movementStart,
                        responsiveTarget,
                        cornerRootMaximumSpeed * Time.deltaTime);
                }
                locomotionWorldPosition = ResolveObstacleMovement(
                    movementStart, clearedMovementTarget);
            }
            else
            {
                hasClimbSurfaceAnchor = false;
                locomotionWorldPosition = ResolveObstacleMovement(
                    movementStart, movedSurfaceSample);
            }
            if (!hasClimbSurfaceAnchor &&
                hasTerrainRootClearance && terrainForces != null &&
                terrainForces.TryGetRequiredBodyHeight(
                    locomotionWorldPosition,
                    commandedForward,
                    commandedRight,
                    terrainRootClearance,
                out float requiredBodyHeight))
            {
                Vector3 worldPosition = locomotionWorldPosition;
                // Raising remains immediate so the body cannot enter rising
                // terrain. A downward height discontinuity is safe to follow
                // gradually and avoids a visible drop at a floor/wall lip.
                worldPosition.y = requiredBodyHeight >= worldPosition.y
                    ? requiredBodyHeight
                    : Mathf.MoveTowards(
                        worldPosition.y,
                        requiredBodyHeight,
                        cornerRootMaximumSpeed * Time.deltaTime);
                locomotionWorldPosition = worldPosition;
            }

            if (rigidbodyDynamics != null &&
                rigidbodyDynamics.isActiveAndEnabled)
                locomotionWorldPosition =
                    rigidbodyDynamics.ConstrainDesiredPosition(
                        locomotionWorldPosition);

            if (foundSurfaceFrame && useSurfaceMotionAnchor)
            {
                Vector3 trackedRootPosition =
                    rigidbodyDynamics != null &&
                    rigidbodyDynamics.isActiveAndEnabled &&
                    rigidbodyDynamics.PhysicsBody != null
                        ? rigidbodyDynamics.PhysicsBody.position
                        : locomotionWorldPosition;
                float unresolvedRootDistance = Vector3.Distance(
                    trackedRootPosition, surfaceClearedRoot);
                // A steep heightfield can move its sampled contact much
                // farther than the Rigidbody or corner solver can follow in
                // one frame. Do not commit that virtual contact until the
                // torso has caught up to its corresponding cleared pose.
                // Otherwise every new command starts from an increasingly
                // distant point and the spider appears to float across gaps.
                climbSurfaceAnchor =
                    hadSurfaceMotionAnchor &&
                    unresolvedRootDistance >
                    maximumSurfaceAnchorTrackingError
                        ? previousSurfaceMotionAnchor
                        : surfacePoint;
            }

            UpdateSurfaceTransition(
                foundSurfaceFrame,
                enteredClimb,
                surfaceNormal,
                supportNormal);

            traceFoundSurfaceFrame = foundSurfaceFrame;
            traceEnteredClimb = enteredClimb;
            traceCornerTransition = isCornerTransition;
            traceRootTransitionActive = rootPositionTransitionActive;
            traceMovementStart = movementStart;
            traceSurfaceSample = surfaceSample;
            traceMovedSurfaceSample = movedSurfaceSample;
            traceSurfaceForward = surfaceForward;
            traceSurfacePoint = surfacePoint;
            traceRawSurfaceNormal = surfaceNormal;
            traceSupportNormal = supportNormal;
            traceClearedRootTarget = surfaceClearedRoot;

            if (terrainForces != null)
            {
                Vector3 facingDirection = surfaceForward;
                Vector3 actualSurfaceTravel = Vector3.ProjectOnPlane(
                    locomotionWorldPosition - movementStart,
                    foundSurfaceFrame ? supportNormal : Vector3.up);
                if (Mathf.Abs(forwardDelta) > 0.00001f &&
                    actualSurfaceTravel.sqrMagnitude > 0.00000025f)
                {
                    // Terrain projection may add a small diagonal component,
                    // but lateral Rigidbody drift must not become the new
                    // heading and create a self-reinforcing spin. Accept only
                    // travel that remains close to commanded surface-forward,
                    // and blend it conservatively.
                    Vector3 actualDirection =
                        actualSurfaceTravel.normalized;
                    if (Vector3.Dot(actualDirection, surfaceForward) >=
                        Mathf.Cos(30f * Mathf.Deg2Rad))
                    {
                        facingDirection = Vector3.Slerp(
                            surfaceForward,
                            actualDirection,
                            0.25f).normalized;
                    }
                }

                desiredRootRotation = terrainForces.GetSlopeAlignedRootRotation(
                    GetTerrainSamplingPosition(),
                    desiredRootRotation,
                    facingDirection);
            }

            ApplyPhysicsOwnedRootPose(
                locomotionWorldPosition,
                desiredRootRotation,
                foundSurfaceFrame ? supportNormal : Vector3.up,
                GetPlantedPushDriveStrength(),
                false);

        }

        private void ApplyPhysicsOwnedRootPose(
            Vector3 desiredPosition,
            Quaternion desiredRotation,
            Vector3 supportNormal,
            float plantedDrive,
            bool airborne)
        {
            if (rigidbodyDynamics != null && rigidbodyDynamics.isActiveAndEnabled)
            {
                rigidbodyDynamics.SetDesiredPose(
                    desiredPosition,
                    desiredRotation,
                    supportNormal,
                    plantedDrive,
                    airborne);
                return;
            }

            transform.SetPositionAndRotation(desiredPosition, desiredRotation);
        }

        private void BeginSurfaceTransition(
            bool targetIsClimb,
            bool awaitingClimbEntry = false)
        {
            rootPositionTransitionActive = true;
            transitionTargetIsClimb = targetIsClimb;
            transitionAwaitingClimbEntry = awaitingClimbEntry;
            surfaceTransitionStartedAt = Time.time;
            surfaceTransitionStableSince = -1f;
        }

        private void UpdateSurfaceTransition(
            bool foundSurfaceFrame,
            bool enteredClimb,
            Vector3 rawSurfaceNormal,
            Vector3 supportNormal)
        {
            if (!rootPositionTransitionActive)
                return;

            float elapsed = Mathf.Max(0f, Time.time - surfaceTransitionStartedAt);
            float maximumDuration = Mathf.Max(
                minimumSurfaceTransitionDuration,
                maximumSurfaceTransitionDuration);
            if (transitionAwaitingClimbEntry)
            {
                if (enteredClimb)
                {
                    transitionAwaitingClimbEntry = false;
                    surfaceTransitionStableSince = -1f;
                }
                else
                {
                    if (elapsed >= maximumDuration)
                    {
                        rootPositionTransitionActive = false;
                        transitionAwaitingClimbEntry = false;
                        surfaceTransitionStartedAt = -1f;
                    }
                    return;
                }
            }

            if (foundSurfaceFrame && enteredClimb != transitionTargetIsClimb)
            {
                BeginSurfaceTransition(enteredClimb);
                return;
            }

            bool normalsSettled = foundSurfaceFrame &&
                enteredClimb == transitionTargetIsClimb &&
                Vector3.Angle(rawSurfaceNormal, supportNormal) <=
                surfaceTransitionNormalTolerance;
            if (normalsSettled)
            {
                if (surfaceTransitionStableSince < 0f)
                    surfaceTransitionStableSince = Time.time;
            }
            else
            {
                surfaceTransitionStableSince = -1f;
            }

            bool stableLongEnough = surfaceTransitionStableSince >= 0f &&
                Time.time - surfaceTransitionStableSince >=
                surfaceTransitionStableDuration;
            if ((elapsed >= minimumSurfaceTransitionDuration &&
                 stableLongEnough) || elapsed >= maximumDuration)
            {
                rootPositionTransitionActive = false;
                transitionAwaitingClimbEntry = false;
                surfaceTransitionStartedAt = -1f;
                surfaceTransitionStableSince = -1f;
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
                    IsWalkableTerrainContact(
                        hits[i], horizontalDelta / distance) ||
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

        private bool IsWalkableTerrainContact(
            RaycastHit hit,
            Vector3 approachDirection,
            bool allowSteepTerrain = true)
        {
            // Terrain-painted tree trunks may be reported through the same
            // TerrainCollider as the ground. A steep contact is ignored only
            // when its normal agrees with the actual TerrainData wall normal;
            // vertical tree/trunk contacts therefore remain blocking.
            return hit.collider is TerrainCollider &&
                   (hit.normal.y >= 0.55f ||
                    (allowSteepTerrain && terrainForces != null &&
                     terrainForces.IsClimbableTerrainContact(
                         hit.point, hit.normal, approachDirection)));
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
                    out surfaceTarget, out surfaceNormal,
                    rootPositionTransitionActive))
                return true;

            bool allowLocalClimbContact = rootPositionTransitionActive ||
                                          hasClimbSurfaceAnchor ||
                                          wasOnClimbSurface;
            if (allowLocalClimbContact && terrainForces != null &&
                terrainForces.TryGetClimbSurfaceContact(
                    desired, clearance, out surfaceTarget, out surfaceNormal))
                return true;

            surfaceTarget = desired;
            surfaceNormal = Vector3.up;
            if (!TrySampleTerrainHeight(desired, out float terrainHeight))
                return false;
            if (!allowLocalClimbContact && traceFoundSurfaceFrame &&
                Mathf.Abs(terrainHeight - traceSurfacePoint.y) >
                maximumPreTransitionFootHeightDifference)
            {
                terrainHeight = traceSurfacePoint.y;
            }
            surfaceTarget.y = terrainHeight + clearance;
            return true;
        }

        private bool TryGetActiveClimbNormal(out Vector3 surfaceNormal)
        {
            if (rootPositionTransitionActive && traceFoundSurfaceFrame &&
                traceSupportNormal.sqrMagnitude > 0.0001f)
            {
                surfaceNormal = traceSupportNormal.normalized;
                return true;
            }

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
            Vector3 naturalRestTarget = rigSpace != null
                ? rigSpace.TransformPoint(leg.RestRigLocalTarget)
                : leg.RestWorldTarget;
            float restingReach = Vector3.Distance(hip, naturalRestTarget);
            Vector3 hipToTarget = target - hip;
            if (restingReach < 0.0001f || hipToTarget.sqrMagnitude < 0.0000001f)
                return naturalRestTarget;

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
            rootPositionTransitionActive = false;
            wasOnClimbSurface = false;
            transitionTargetIsClimb = false;
            transitionAwaitingClimbEntry = false;
            surfaceTransitionStartedAt = -1f;
            surfaceTransitionStableSince = -1f;
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
            // A dynamic Rigidbody owns the root pose. Restoring the authored
            // root every LateUpdate would erase velocity and cancel all forces.
            if (rigidbodyDynamics == null ||
                !rigidbodyDynamics.isActiveAndEnabled)
            {
                transform.localPosition = rootLocalPosition;
                transform.localRotation = rootLocalRotation;
            }
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
                else if ((rootPositionTransitionActive ||
                          hasClimbSurfaceAnchor ||
                          wasOnClimbSurface) &&
                    terrainForces != null &&
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
                if (!isJumping && TryGetFootSurfaceTarget(
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
                if (rootPositionTransitionActive)
                    solvedTarget = ConstrainTargetReach(leg, solvedTarget);
                SolveCcd(leg, solvedTarget);
                if (!isJumping && TryGetFootSurfaceTarget(
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
                        Vector3 correctedTarget =
                            solvedTarget + solvedNormal * penetration;
                        if (rootPositionTransitionActive)
                            correctedTarget = ConstrainTargetReach(
                                leg, correctedTarget);
                        SolveCcd(leg, correctedTarget);
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
                    IsWalkableTerrainContact(
                        hits[i],
                        delta / distance,
                        hasClimbSurfaceAnchor ||
                        wasOnClimbSurface ||
                        (rootPositionTransitionActive &&
                         terrainForces != null &&
                         terrainForces.IsClimbableSurfaceNormal(
                             traceSupportNormal))))
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
            jumpFinger = DexterFinger.Thumb;
            ApplyResponsiveMovementDefaults();
            calibrationDurationSeconds = Mathf.Max(0.25f, calibrationDurationSeconds);
            minimumDexterCalibrationForce = Mathf.Max(
                0.01f, minimumDexterCalibrationForce);
            minimumCalibrationWiggleRange = Mathf.Max(
                0.01f, minimumCalibrationWiggleRange);
            maximumWalkingAxisCorrectionDegrees = Mathf.Clamp(
                maximumWalkingAxisCorrectionDegrees, 0f, 80f);
            calibratedWalkingLateralSuppression = Mathf.Clamp01(
                calibratedWalkingLateralSuppression);
            calibratedWalkingCorridorRatio = Mathf.Clamp(
                calibratedWalkingCorridorRatio, 0.1f, 0.75f);
            dexterMaximumSpeedForceMultiplier = Mathf.Clamp(
                dexterMaximumSpeedForceMultiplier, 1.05f, 6f);
            dexterHighForceWalkingSpeedMultiplier = Mathf.Clamp(
                dexterHighForceWalkingSpeedMultiplier, 1f, 24f);
            dexterBaselineWalkingSpeedMultiplier = Mathf.Clamp(
                dexterBaselineWalkingSpeedMultiplier, 0.1f, 6f);
            dexterMinimumWalkingSpeedMultiplier = Mathf.Clamp(
                dexterMinimumWalkingSpeedMultiplier, 0.1f, 1f);
            dexterSpeedForceReleaseSeconds = Mathf.Clamp(
                dexterSpeedForceReleaseSeconds, 0.05f, 0.6f);
            dexterWalkingAccelerationResponseScale = Mathf.Clamp(
                dexterWalkingAccelerationResponseScale, 0.5f, 6f);
            dexterWalkingBrakingResponseScale = Mathf.Clamp(
                dexterWalkingBrakingResponseScale, 0.5f, 6f);
            dexterBaselineStrideLength = Mathf.Clamp(
                dexterBaselineStrideLength,
                0.05f,
                Mathf.Max(0.05f, maximumForeAftMovement));
            dexterHighForceStrideLength = Mathf.Clamp(
                dexterHighForceStrideLength,
                dexterBaselineStrideLength,
                Mathf.Max(dexterBaselineStrideLength,
                    maximumForeAftMovement));
            dexterMaximumGaitCyclesPerSecond = Mathf.Clamp(
                dexterMaximumGaitCyclesPerSecond, 0.5f, 8f);
            dexterFingerInputSensitivity = Mathf.Clamp(
                dexterFingerInputSensitivity, 1f, 3f);
            dexterTurningSensitivity = Mathf.Clamp(
                dexterTurningSensitivity, 1f, 3f);
            dexterTurnMaximumSpeedReferenceMultiplier = Mathf.Clamp(
                dexterTurnMaximumSpeedReferenceMultiplier, 1.5f, 8f);
            turnIntentHoldSeconds = Mathf.Max(0f, turnIntentHoldSeconds);
            turnWalkingTakeoverSeconds = Mathf.Max(
                0f, turnWalkingTakeoverSeconds);
            turnConstantForceTolerance = Mathf.Clamp(
                turnConstantForceTolerance, 0.05f, 1f);
            walkingPriorityAfterAlternation = Mathf.Max(
                maximumAlternationInterval,
                walkingPriorityAfterAlternation);
            walkingIntentConfirmationSeconds = Mathf.Max(
                0.02f, walkingIntentConfirmationSeconds);
            walkingIntentDeltaFraction = Mathf.Clamp(
                walkingIntentDeltaFraction, 0.001f, 0.1f);
            establishedWalkingIntentHoldSeconds = Mathf.Clamp(
                establishedWalkingIntentHoldSeconds, 0.6f, 1.5f);
            singleFingerIsolationRatio = Mathf.Clamp(
                singleFingerIsolationRatio, 0.05f, 0.75f);
            displacementPerNewton.x = Mathf.Max(0f, displacementPerNewton.x);
            displacementPerNewton.y = Mathf.Max(0f, displacementPerNewton.y);
            forceDeadZone = Mathf.Max(0f, forceDeadZone);
            maximumDisplacement = Mathf.Max(0.01f, maximumDisplacement);
            responseSpeed = Mathf.Max(0.01f, responseSpeed);
            temporalSmoothingWindow = Mathf.Max(0f, temporalSmoothingWindow);
            maximumOutwardMovement = Mathf.Max(0f, maximumOutwardMovement);
            maximumInwardMovement = Mathf.Max(0f, maximumInwardMovement);
            maximumForeAftMovement = Mathf.Max(0f, maximumForeAftMovement);
            maximumPreTransitionFootHeightDifference = Mathf.Max(
                0.05f, maximumPreTransitionFootHeightDifference);
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
            maximumAlternationInterval = Mathf.Clamp(
                maximumAlternationInterval,
                Mathf.Max(0.05f, minimumPressInterval), 0.6f);
            alternatingDriveSustainTime = Mathf.Clamp(
                alternatingDriveSustainTime,
                maximumAlternationInterval,
                establishedWalkingIntentHoldSeconds);
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
            minimumJumpForce = Mathf.Max(0.001f, minimumJumpForce);
            calibratedJumpActivationFraction = Mathf.Clamp(
                calibratedJumpActivationFraction, 0.05f, 0.75f);
            forceForMaximumJump = Mathf.Max(
                minimumJumpForce + 0.001f, forceForMaximumJump);
            thumbJumpCalibrationReferencePercentile = Mathf.Clamp(
                thumbJumpCalibrationReferencePercentile, 0.5f, 0.9f);
            thumbNeutralCalibrationSeconds = Mathf.Clamp(
                thumbNeutralCalibrationSeconds,
                0.5f,
                calibrationDurationSeconds * 0.5f);
            thumbIncidentalForceSafetyMultiplier = Mathf.Max(
                1f, thumbIncidentalForceSafetyMultiplier);
            jumpActivationHoldSeconds = Mathf.Max(
                0f, jumpActivationHoldSeconds);
            walkingInputJumpActivationMargin = Mathf.Max(
                0f, walkingInputJumpActivationMargin);
            jumpReleaseForce = Mathf.Clamp(
                jumpReleaseForce, 0f, minimumJumpForce * 0.95f);
            jumpReleaseActivationFraction = Mathf.Clamp(
                jumpReleaseActivationFraction, 0.5f, 0.98f);
            jumpForceSamplingWindow = Mathf.Max(
                0.02f, jumpForceSamplingWindow);
            minimumJumpDistance = Mathf.Max(0f, minimumJumpDistance);
            maximumJumpDistance = Mathf.Max(
                minimumJumpDistance, maximumJumpDistance);
            minimumJumpHeight = Mathf.Max(0f, minimumJumpHeight);
            maximumJumpHeight = Mathf.Max(
                minimumJumpHeight, maximumJumpHeight);
            strongJumpForceAboveCalibration = Mathf.Max(
                0f, strongJumpForceAboveCalibration);
            strongJumpDistanceMultiplier = Mathf.Max(
                1f, strongJumpDistanceMultiplier);
            minimumJumpDuration = Mathf.Max(0.1f, minimumJumpDuration);
            maximumJumpDuration = Mathf.Max(
                minimumJumpDuration, maximumJumpDuration);
            airborneRearLegStretch = Mathf.Max(0f, airborneRearLegStretch);
            jumpCooldown = Mathf.Max(0f, jumpCooldown);
            jumpRearmReleaseHoldSeconds = Mathf.Max(
                0f, jumpRearmReleaseHoldSeconds);
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
            turnAngularAcceleration = Mathf.Max(
                1f, turnAngularAcceleration);
            turnAngularDeceleration = Mathf.Max(
                1f, turnAngularDeceleration);
            outsideTurnStrideMultiplier = Mathf.Max(0f, outsideTurnStrideMultiplier);
            insideTurnStrideMultiplier = Mathf.Max(0f, insideTurnStrideMultiplier);
            turnRecoveryDuration = Mathf.Max(0.1f, turnRecoveryDuration);
            bodyCollisionRadius = Mathf.Max(0.05f, bodyCollisionRadius);
            bodyCollisionHeight = Mathf.Max(0f, bodyCollisionHeight);
            obstacleSkin = Mathf.Max(0f, obstacleSkin);
            cornerRootPositionResponse = Mathf.Max(
                0.01f, cornerRootPositionResponse);
            cornerRootMaximumSpeed = Mathf.Max(0.01f, cornerRootMaximumSpeed);
            maximumSurfaceAnchorTrackingError = Mathf.Max(
                0.05f, maximumSurfaceAnchorTrackingError);
            minimumSurfaceTransitionDuration = Mathf.Max(
                0f, minimumSurfaceTransitionDuration);
            maximumSurfaceTransitionDuration = Mathf.Max(
                Mathf.Max(0.1f, minimumSurfaceTransitionDuration),
                maximumSurfaceTransitionDuration);
            surfaceTransitionNormalTolerance = Mathf.Clamp(
                surfaceTransitionNormalTolerance, 1f, 20f);
            surfaceTransitionStableDuration = Mathf.Max(
                0f, surfaceTransitionStableDuration);
            baseJointLeadDegrees = Mathf.Clamp(baseJointLeadDegrees, 0f, 30f);
            maximumBaseDeviationDegrees = Mathf.Clamp(maximumBaseDeviationDegrees, 1f, 120f);
            maximumJointDeviationDegrees = Mathf.Clamp(maximumJointDeviationDegrees, 1f, 120f);
            solverIterations = Mathf.Clamp(solverIterations, 1, 32);
            positionTolerance = Mathf.Max(0.00001f, positionTolerance);
        }

        private void ApplyResponsiveMovementDefaults()
        {
            // Dexter X and Y channels are mechanically coupled. The latest
            // physical trace showed valid same-direction X turn input being
            // rejected only because one finger also carried a much larger Y
            // load. Preserve the two-finger/same-sign requirement while making
            // turning consistent with the calibrated walking sensitivity.
            dexterTurningSensitivity = Mathf.Max(
                2.25f, dexterTurningSensitivity);
            dexterMaximumSpeedForceMultiplier = 1.2f;
            dexterHighForceWalkingSpeedMultiplier = Mathf.Max(
                19.5f, dexterHighForceWalkingSpeedMultiplier);
            dexterBaselineWalkingSpeedMultiplier = Mathf.Max(
                3f, dexterBaselineWalkingSpeedMultiplier);
            dexterHighForceStrideLength = Mathf.Max(
                1.5f, dexterHighForceStrideLength);
            dexterSpeedForceReleaseSeconds = Mathf.Max(
                0.5f, dexterSpeedForceReleaseSeconds);
            dexterWalkingBrakingResponseScale = Mathf.Min(
                0.75f, dexterWalkingBrakingResponseScale);
            maximumSurfaceAnchorTrackingError = Mathf.Max(
                0.85f, maximumSurfaceAnchorTrackingError);
            turnIntentHoldSeconds = Mathf.Min(
                0.09f, turnIntentHoldSeconds);
            turnWalkingTakeoverSeconds = Mathf.Min(
                0.09f, turnWalkingTakeoverSeconds);
            turnConstantForceTolerance = Mathf.Max(
                0.45f, turnConstantForceTolerance);
            turnResponseSpeed = Mathf.Max(
                6f, turnResponseSpeed);
            maximumAlternationInterval = Mathf.Clamp(
                maximumAlternationInterval,
                Mathf.Max(0.05f, minimumPressInterval), 0.6f);
            establishedWalkingIntentHoldSeconds = Mathf.Clamp(
                establishedWalkingIntentHoldSeconds, 0.6f, 1.5f);
            alternatingDriveSustainTime = Mathf.Clamp(
                alternatingDriveSustainTime,
                maximumAlternationInterval,
                establishedWalkingIntentHoldSeconds);
            walkingPriorityAfterAlternation =
                establishedWalkingIntentHoldSeconds;
            turnAxisDominanceRatio = Mathf.Clamp(
                turnAxisDominanceRatio, 0.12f, 0.30f);
            minimumTurnDegreesPerSecond = Mathf.Max(
                36f, minimumTurnDegreesPerSecond);
            maximumTurnDegreesPerSecond = Mathf.Max(
                228f, maximumTurnDegreesPerSecond);
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
