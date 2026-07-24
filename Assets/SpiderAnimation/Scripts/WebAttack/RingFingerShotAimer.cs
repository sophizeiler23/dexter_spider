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
    /// Only acts while <see cref="SpiderWebAttack.AimingMode"/> is <see cref="SpiderWebAttack.WebAimingMode.PeakGesture"/>
    /// (see <see cref="RingFingerAimerBase.IsActive"/>); otherwise sits idle so it can be left attached
    /// alongside <see cref="RingFingerTrajectoryAimer"/> and switched between via that single setting.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class RingFingerShotAimer : RingFingerAimerBase
    {
        private enum GestureState
        {
            Idle,
            Tracking,
            CancelledAwaitingRelease
        }

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

        private GestureState state = GestureState.Idle;
        private Vector2 peak;
        private float trackingElapsed;

        protected override SpiderWebAttack.WebAimingMode RequiredMode => SpiderWebAttack.WebAimingMode.PeakGesture;

        private void OnValidate()
        {
            releaseThreshold = Mathf.Min(releaseThreshold, activationThreshold);
            maxPeakMagnitudeForMaxSpeed = Mathf.Max(maxPeakMagnitudeForMaxSpeed, minPeakMagnitudeForMinSpeed + 0.01f);
        }

        protected override void OnInactive()
        {
            // Switching away mid-gesture shouldn't leave a stale partial extension armed for when
            // this mode becomes active again.
            state = GestureState.Idle;
        }

        protected override void OnActiveUpdate(Vector2 relative)
        {
            UpdateGesture(relative);
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
                        if (LogGestureDebugInfo)
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
                if (LogGestureDebugInfo)
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

            if (LogGestureDebugInfo)
            {
                Debug.Log(
                    $"{nameof(RingFingerShotAimer)}: firing shot. peak=({peak.x:F3}, {peak.y:F3}) " +
                    $"azimuth={azimuthDegrees:F1} deg, speed={launchSpeed:F2}.",
                    this);
            }

            WebAttack.FireWebShot(azimuthDegrees, launchSpeed);
        }
    }
}
