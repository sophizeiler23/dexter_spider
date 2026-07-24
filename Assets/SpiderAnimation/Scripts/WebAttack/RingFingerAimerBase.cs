using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// Shared plumbing for ring-finger-driven aiming systems: resolving the relay/leg-IK/attack
    /// references, taring the finger's rest baseline (reusing <see cref="DexterFrontLegIK"/>'s
    /// calibration-phase toggle exactly like <see cref="RingFingerShotAimer"/> always has), and
    /// exposing the baseline-subtracted force for both gameplay and the force overlay to read.
    /// Concrete subclasses (<see cref="RingFingerShotAimer"/>, <see cref="RingFingerTrajectoryAimer"/>)
    /// only need to implement what happens with that force while they're the active aiming system.
    /// </summary>
    public abstract class RingFingerAimerBase : MonoBehaviour
    {
        [Header("References")]
        [Tooltip("Relay receiver to read the ring finger's raw (x, y) from. Auto-resolved on enable (this object, its parent, then the whole scene) if left empty.")]
        [SerializeField] private DexterRelayUdpReceiver receiver;
        [Tooltip("Leg IK component whose 'Enable Calibration Phase' checkbox gates whether this component auto-tares its own baseline on enable. Auto-resolved on enable if left empty.")]
        [SerializeField] private DexterFrontLegIK legIK;
        [Tooltip("Web attack component this aiming system fires through, and whose 'Aiming Mode' selects whether this component is currently active. Auto-resolved on enable if left empty.")]
        [SerializeField] private SpiderWebAttack webAttack;

        [Header("Baseline")]
        [Tooltip("How long to sample the ring finger's rest position before gesture detection begins. Skipped entirely (baseline = 0) while the leg IK's calibration phase is disabled.")]
        [SerializeField, Min(0.1f)] private float baselineDurationSeconds = 1.5f;

        [Header("Debug")]
        [Tooltip("Logs baseline taring, gesture cancellations, filtered gestures, and fired shots.")]
        [SerializeField] private bool logGestureDebugInfo;

        private Vector2 ringBaseline;
        private bool hasBaseline;
        private bool isTaring;
        private Vector2 baselineSum;
        private int baselineSamples;
        private float tareStartRealtime = -1f;
        private long lastTareSequence = long.MinValue;

        /// <summary>True once the ring finger's rest baseline has been established (or was skipped because calibration is disabled).</summary>
        public bool HasBaseline => hasBaseline;

        /// <summary>
        /// Ring finger force after baseline subtraction — the exact signal this aimer's gesture/preview
        /// logic acts on, as opposed to the raw value coming straight off the relay.
        /// </summary>
        public Vector2 ProcessedForce { get; private set; }

        /// <summary>True while this is the aiming system <see cref="SpiderWebAttack.AimingMode"/> currently selects.</summary>
        public bool IsActive => webAttack != null && webAttack.AimingMode == RequiredMode;

        /// <summary>Logs debug info when true; shared flag exposed to subclasses for consistent logging.</summary>
        protected bool LogGestureDebugInfo => logGestureDebugInfo;

        /// <summary>The <see cref="SpiderWebAttack.WebAimingMode"/> this concrete aimer implements.</summary>
        protected abstract SpiderWebAttack.WebAimingMode RequiredMode { get; }

        /// <summary>Web attack this aimer fires shots through. Null until references resolve.</summary>
        protected SpiderWebAttack WebAttack => webAttack;

        /// <summary>
        /// Called every frame with the current baseline-subtracted force while <see cref="IsActive"/>
        /// is true and a baseline has been established.
        /// </summary>
        protected abstract void OnActiveUpdate(Vector2 relative);

        /// <summary>
        /// Called every frame instead of <see cref="OnActiveUpdate"/> while this aimer is not the
        /// active aiming system, so it can cancel/hide any in-progress gesture or preview. Default is
        /// a no-op.
        /// </summary>
        protected virtual void OnInactive()
        {
        }

        protected virtual void OnEnable()
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
            ProcessedForce = Vector2.zero;
            OnInactive();
        }

        private void Update()
        {
            UpdateBaselineTare();

            if (!hasBaseline || receiver == null || webAttack == null)
                return;

            if (!TryReadRingForce(out Vector2 raw))
                return;

            ProcessedForce = raw - ringBaseline;

            if (IsActive)
                OnActiveUpdate(ProcessedForce);
            else
                OnInactive();
        }

        protected bool TryReadRingForce(out Vector2 force)
        {
            force = Vector2.zero;
            if (receiver == null)
                return false;

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
                    $"{GetType().Name}: ring baseline tared to " +
                    $"({ringBaseline.x:F3}, {ringBaseline.y:F3}) over {baselineSamples} samples.",
                    this);
            }
        }
    }
}
