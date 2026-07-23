using Dexter.Visualize;
using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// Drives <see cref="SpiderWebAttack"/> from a ring-finger extension gesture: a short rise of the
    /// finger's Y axis above the rest baseline, followed by a return back to baseline. Only positive-Y
    /// movement is ever considered an extension. The (x, y) sample with the largest Y seen during the
    /// excursion is treated as the gesture's "peak" vector and used to derive the shot:
    /// - Azimuth (left/right aim off the spider's forward) = angle of the peak vector from the Y axis.
    /// - Elevation is fixed on <see cref="SpiderWebAttack"/>, independent of finger input.
    /// - Launch speed = a range mapped from the peak vector's magnitude (its "norm").
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class RingFingerShotAimer : MonoBehaviour
    {
        private enum GestureState
        {
            Idle,
            Tracking,
            CancelledAwaitingRelease
        }

        [Header("References")]
        [Tooltip("Relay receiver to read the ring finger's raw (x, y) from. Auto-resolved on enable (this object, its parent, then the whole scene) if left empty.")]
        [SerializeField] private DexterRelayUdpReceiver receiver;
        [Tooltip("Leg IK component whose 'Enable Calibration Phase' checkbox gates whether this component auto-tares its own baseline on enable. Auto-resolved on enable if left empty.")]
        [SerializeField] private DexterFrontLegIK legIK;
        [Tooltip("Web attack component to fire through once a gesture completes. Auto-resolved on enable if left empty.")]
        [SerializeField] private SpiderWebAttack webAttack;

        [Header("Baseline")]
        [Tooltip("How long to sample the ring finger's rest position before gesture detection begins. Skipped entirely (baseline = 0) while the leg IK's calibration phase is disabled.")]
        [SerializeField, Min(0.1f)] private float baselineDurationSeconds = 1.5f;

        [Header("Gesture Thresholds")]
        [Tooltip("Relative Y the ring finger must rise above (from baseline) to start tracking an extension.")]
        [SerializeField, Min(0f)] private float activationThreshold = 0.15f;
        [Tooltip("Relative Y the finger must fall back below to count as 'returned to baseline' and complete or re-arm the gesture. Should be lower than the activation threshold (hysteresis).")]
        [SerializeField, Min(0f)] private float releaseThreshold = 0.08f;
        [Tooltip("Peak vector magnitude below this is treated as noise and never fires a shot.")]
        [SerializeField, Min(0f)] private float minPeakMagnitude = 0.2f;
        [Tooltip("If the finger stays extended longer than this without returning to baseline, the gesture is cancelled (it wasn't a short flick) and must fully release before it can re-arm.")]
        [SerializeField, Min(0.05f)] private float maxExtensionSeconds = 1.2f;

        [Header("Azimuth Mapping")]
        [Tooltip("Peak-vector angle from the Y axis is clamped to +/- this many degrees before being applied as the shot's azimuth.")]
        [SerializeField, Range(0f, 180f)] private float maxAzimuthDegrees = 60f;
        [Tooltip("Flips the left/right steering direction. Enable if aiming left with the finger sends the shot right, or vice versa.")]
        [SerializeField] private bool invertAzimuth;

        [Header("Speed Mapping")]
        [Tooltip("Peak magnitude at or below this maps to the minimum launch speed. Weaker extensions than this still fire (as long as they clear 'Min Peak Magnitude' above) but always at minimum speed.")]
        [SerializeField, Min(0f)] private float minPeakMagnitudeForMinSpeed = 0.2f;
        [Tooltip("Peak magnitude at or above this maps to the maximum launch speed. Stronger extensions are clamped to the same maximum speed.")]
        [SerializeField, Min(0.01f)] private float maxPeakMagnitudeForMaxSpeed = 1.5f;
        [Tooltip("Launch speed used for the weakest gesture that still clears 'Min Peak Magnitude'.")]
        [SerializeField, Min(0f)] private float minLaunchSpeed = 6f;
        [Tooltip("Launch speed used for a gesture whose peak magnitude reaches 'Max Peak Magnitude For Max Speed' or beyond.")]
        [SerializeField, Min(0f)] private float maxLaunchSpeed = 16f;

        [Header("Debug")]
        [Tooltip("Logs baseline taring, gesture cancellations, filtered gestures, and fired shots.")]
        [SerializeField] private bool logGestureDebugInfo;

        private GestureState state = GestureState.Idle;
        private Vector2 peak;
        private float trackingElapsed;

        private Vector2 ringBaseline;
        private bool hasBaseline;
        private bool isTaring;

        /// <summary>True once the ring finger's rest baseline has been established (or was skipped because calibration is disabled).</summary>
        public bool HasBaseline => hasBaseline;
        /// <summary>
        /// Ring finger force after baseline subtraction — the exact signal the gesture state machine
        /// acts on, as opposed to the raw value coming straight off the relay.
        /// </summary>
        public Vector2 ProcessedForce { get; private set; }

        private Vector2 baselineSum;
        private int baselineSamples;
        private float tareStartRealtime = -1f;
        private long lastTareSequence = long.MinValue;

        private void OnValidate()
        {
            releaseThreshold = Mathf.Min(releaseThreshold, activationThreshold);
            maxPeakMagnitudeForMaxSpeed = Mathf.Max(maxPeakMagnitudeForMaxSpeed, minPeakMagnitudeForMinSpeed + 0.01f);
        }

        private void OnEnable()
        {
            ResolveReferences();

            if (ShouldAutoTareBaseline())
            {
                BeginTare();
            }
            else
            {
                ringBaseline = Vector2.zero;
                hasBaseline = true;
                isTaring = false;
            }
        }

        private void ResolveReferences()
        {
            if (receiver == null)
                receiver = GetComponent<DexterRelayUdpReceiver>();
            if (receiver == null)
                receiver = GetComponentInParent<DexterRelayUdpReceiver>();
            if (receiver == null)
                receiver = FindAnyObjectByType<DexterRelayUdpReceiver>();

            if (legIK == null)
                legIK = GetComponent<DexterFrontLegIK>();
            if (legIK == null)
                legIK = GetComponentInParent<DexterFrontLegIK>();
            if (legIK == null)
                legIK = FindAnyObjectByType<DexterFrontLegIK>();

            if (webAttack == null)
                webAttack = GetComponent<SpiderWebAttack>();
            if (webAttack == null)
                webAttack = GetComponentInParent<SpiderWebAttack>();
            if (webAttack == null)
                webAttack = FindAnyObjectByType<SpiderWebAttack>();
        }

        private bool ShouldAutoTareBaseline()
        {
            return legIK == null || legIK.IsCalibrationPhaseEnabled;
        }

        [ContextMenu("Tare Ring Finger")]
        public void BeginTare()
        {
            baselineSum = Vector2.zero;
            baselineSamples = 0;
            tareStartRealtime = -1f;
            lastTareSequence = long.MinValue;
            isTaring = true;
            hasBaseline = false;
            state = GestureState.Idle;
            ProcessedForce = Vector2.zero;
        }

        private void Update()
        {
            UpdateBaselineTare();

            if (!hasBaseline || receiver == null || webAttack == null)
                return;

            if (!TryReadRingForce(out Vector2 raw))
                return;

            ProcessedForce = raw - ringBaseline;
            UpdateGesture(ProcessedForce);
        }

        private bool TryReadRingForce(out Vector2 force)
        {
            force = Vector2.zero;
            DexterFingerMeasurement measurement = receiver.GetFinger(DexterFinger.Ring);
            if (measurement == null || !measurement.has_data || measurement.force == null || measurement.force.Length < 2)
                return false;

            force = new Vector2(measurement.force[0], measurement.force[1]);
            return true;
        }

        private void UpdateBaselineTare()
        {
            if (!isTaring)
                return;

            DexterForceFrame frame = receiver != null ? receiver.LatestFrame : null;
            if (frame == null || frame.sequence == lastTareSequence)
                return;

            if (!TryReadRingForce(out Vector2 force))
                return;

            lastTareSequence = frame.sequence;
            baselineSum += force;
            baselineSamples++;

            if (tareStartRealtime < 0f)
                tareStartRealtime = Time.realtimeSinceStartup;
            if (Time.realtimeSinceStartup - tareStartRealtime < baselineDurationSeconds)
                return;

            ringBaseline = baselineSamples > 0 ? baselineSum / baselineSamples : Vector2.zero;
            hasBaseline = true;
            isTaring = false;

            if (logGestureDebugInfo)
            {
                Debug.Log(
                    $"{nameof(RingFingerShotAimer)}: ring baseline tared to " +
                    $"({ringBaseline.x:F3}, {ringBaseline.y:F3}) over {baselineSamples} samples.",
                    this);
            }
        }

        private void UpdateGesture(Vector2 relative)
        {
            switch (state)
            {
                case GestureState.Idle:
                    if (relative.y > activationThreshold)
                    {
                        state = GestureState.Tracking;
                        peak = relative;
                        trackingElapsed = 0f;
                    }
                    break;

                case GestureState.Tracking:
                    trackingElapsed += Time.deltaTime;
                    if (relative.y > peak.y)
                        peak = relative;

                    if (relative.y <= releaseThreshold)
                    {
                        CompleteGesture();
                        state = GestureState.Idle;
                    }
                    else if (trackingElapsed >= maxExtensionSeconds)
                    {
                        if (logGestureDebugInfo)
                        {
                            Debug.Log(
                                $"{nameof(RingFingerShotAimer)}: gesture cancelled, held extended for " +
                                $"{trackingElapsed:F2}s (> {maxExtensionSeconds:F2}s).",
                                this);
                        }
                        state = GestureState.CancelledAwaitingRelease;
                    }
                    break;

                case GestureState.CancelledAwaitingRelease:
                    if (relative.y <= releaseThreshold)
                        state = GestureState.Idle;
                    break;
            }
        }

        private void CompleteGesture()
        {
            float magnitude = peak.magnitude;
            if (magnitude < minPeakMagnitude)
            {
                if (logGestureDebugInfo)
                {
                    Debug.Log(
                        $"{nameof(RingFingerShotAimer)}: gesture filtered, peak magnitude {magnitude:F3} " +
                        $"below minimum {minPeakMagnitude:F3}.",
                        this);
                }
                return;
            }

            float azimuthDegrees = Mathf.Atan2(peak.x, peak.y) * Mathf.Rad2Deg;
            if (invertAzimuth)
                azimuthDegrees = -azimuthDegrees;
            azimuthDegrees = Mathf.Clamp(azimuthDegrees, -maxAzimuthDegrees, maxAzimuthDegrees);

            float speedT = Mathf.InverseLerp(minPeakMagnitudeForMinSpeed, maxPeakMagnitudeForMaxSpeed, magnitude);
            float launchSpeed = Mathf.Lerp(minLaunchSpeed, maxLaunchSpeed, Mathf.Clamp01(speedT));

            if (logGestureDebugInfo)
            {
                Debug.Log(
                    $"{nameof(RingFingerShotAimer)}: firing shot. peak=({peak.x:F3}, {peak.y:F3}) " +
                    $"azimuth={azimuthDegrees:F1} deg, speed={launchSpeed:F2}.",
                    this);
            }

            webAttack.FireWebShot(azimuthDegrees, launchSpeed);
        }
    }
}
