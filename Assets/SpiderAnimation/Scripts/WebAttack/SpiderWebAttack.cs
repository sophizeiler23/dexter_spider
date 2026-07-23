using UnityEngine;
using UnityEngine.InputSystem;

namespace Dexter.Spider
{
    /// <summary>
    /// Fires web shots from the spider's front. The primary trigger is a ring-finger extension
    /// gesture (see <see cref="RingFingerShotAimer"/>), which supplies an azimuth and launch speed
    /// via <see cref="FireWebShot(float, float)"/>. Space is kept as a debug-only fallback that
    /// always fires a flat, straight-ahead shot at the default speed so the attack can be tested
    /// without any Dexter hardware connected.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class SpiderWebAttack : MonoBehaviour
    {
        [Tooltip("Transform the shot launches from. If left empty, falls back to the bone named 'Body Bone Name' below, or this object's own transform if that bone isn't found.")]
        [SerializeField] private Transform launchAnchor;
        [Tooltip("Currently unused by aiming (azimuth/elevation are computed from this object's own forward direction instead) — reserved for a future aim reference. If left empty, falls back the same way as 'Launch Anchor'.")]
        [SerializeField] private Transform aimReference;
        [Tooltip("Name of the bone to search for (among this object's children) when 'Launch Anchor'/'Aim Reference' are left empty.")]
        [SerializeField] private string bodyBoneName = "body";

        [Header("Shot")]
        [Tooltip("Height above the launch anchor the shot spawns at.")]
        [SerializeField, Min(0f)] private float launchHeight = 0.35f;
        [Tooltip("Distance in front of the launch anchor (along the aim direction) the shot spawns at, so it doesn't immediately collide with the spider itself.")]
        [SerializeField, Min(0f)] private float launchForwardOffset = 0.45f;
        [Tooltip("Minimum time between shots, enforced for both the debug Space trigger and gesture-driven shots.")]
        [SerializeField, Min(0f)] private float cooldownSeconds = 0.8f;

        [Header("Gesture Aim")]
        [Tooltip("Elevation angle above the horizontal used for every ring-finger-gesture shot. Fixed, regardless of finger input.")]
        [SerializeField, Range(0f, 90f)] private float gestureLaunchElevationDegrees = 45f;

        private float cooldownTimer;
        private Transform resolvedLaunchAnchor;
        private Transform resolvedAimReference;

        private void OnValidate()
        {
            cooldownSeconds = Mathf.Max(0f, cooldownSeconds);
            launchHeight = Mathf.Max(0f, launchHeight);
            launchForwardOffset = Mathf.Max(0f, launchForwardOffset);
        }

        private void Awake()
        {
            ResolveReferences();
        }

        private void Update()
        {
            if (cooldownTimer > 0f)
                cooldownTimer -= Time.deltaTime;

            if (!WasDebugAttackPressedThisFrame() || cooldownTimer > 0f)
                return;

            FireWebShot();
            cooldownTimer = cooldownSeconds;
        }

        private static bool WasDebugAttackPressedThisFrame()
        {
            Keyboard keyboard = Keyboard.current;
            return keyboard != null && keyboard.spaceKey.wasPressedThisFrame;
        }

        /// <summary>Debug-only trigger: fires a flat, straight-ahead shot at the default speed.</summary>
        public void FireWebShot()
        {
            Fire(azimuthDegrees: 0f, elevationDegrees: 0f, speedOverride: null);
        }

        /// <summary>
        /// Fires a shot aimed by a ring-finger gesture: <paramref name="azimuthDegrees"/> steers left/right
        /// off the spider's forward direction, elevation is the fixed <see cref="gestureLaunchElevationDegrees"/>,
        /// and <paramref name="launchSpeed"/> overrides the projectile's default speed for this shot.
        /// </summary>
        public void FireWebShot(float azimuthDegrees, float launchSpeed)
        {
            if (cooldownTimer > 0f)
                return;

            Fire(azimuthDegrees, gestureLaunchElevationDegrees, launchSpeed);
            cooldownTimer = cooldownSeconds;
        }

        private void Fire(float azimuthDegrees, float elevationDegrees, float? speedOverride)
        {
            ResolveReferences();

            Transform anchor = resolvedLaunchAnchor != null ? resolvedLaunchAnchor : transform;
            Vector3 direction = ResolveAimDirection(azimuthDegrees, elevationDegrees);

            Vector3 origin = anchor.position
                + Vector3.up * launchHeight
                + direction * launchForwardOffset;

            GameObject shotObject = new("SpiderWebShot");
            SpiderWebShot shot = shotObject.AddComponent<SpiderWebShot>();
            shot.Launch(origin, direction, transform, speedOverride);
        }

        private void ResolveReferences()
        {
            if (launchAnchor != null)
                resolvedLaunchAnchor = launchAnchor;
            else
                resolvedLaunchAnchor = FindBone(bodyBoneName) ?? transform;

            if (aimReference != null)
                resolvedAimReference = aimReference;
            else
                resolvedAimReference = FindBone(bodyBoneName) ?? transform;
        }

        /// <summary>
        /// Builds the launch direction from the spider's forward (the sagittal-plane reference, azimuth 0),
        /// rotated left/right by <paramref name="azimuthDegrees"/> around world-up, then tilted upward by
        /// <paramref name="elevationDegrees"/>.
        /// </summary>
        private Vector3 ResolveAimDirection(float azimuthDegrees, float elevationDegrees)
        {
            Vector3 forward = transform.forward;
            forward.y = 0f;
            if (forward.sqrMagnitude < 0.0001f)
                forward = Vector3.forward;
            forward.Normalize();

            Vector3 horizontal = Quaternion.AngleAxis(azimuthDegrees, Vector3.up) * forward;

            float elevationRadians = elevationDegrees * Mathf.Deg2Rad;
            Vector3 direction = horizontal * Mathf.Cos(elevationRadians) + Vector3.up * Mathf.Sin(elevationRadians);
            return direction.normalized;
        }

        private Transform FindBone(string boneName)
        {
            if (string.IsNullOrWhiteSpace(boneName))
                return null;

            Transform[] descendants = GetComponentsInChildren<Transform>(true);
            for (int i = 0; i < descendants.Length; i++)
            {
                if (descendants[i].name == boneName)
                    return descendants[i];
            }

            return null;
        }

        private void OnDrawGizmosSelected()
        {
            ResolveReferences();

            Transform anchor = resolvedLaunchAnchor != null ? resolvedLaunchAnchor : transform;

            Gizmos.color = Color.cyan;
            Vector3 debugDirection = ResolveAimDirection(0f, 0f);
            Vector3 debugOrigin = anchor.position + Vector3.up * launchHeight + debugDirection * launchForwardOffset;
            Gizmos.DrawSphere(debugOrigin, 0.05f);
            Gizmos.DrawRay(debugOrigin, debugDirection);

            Gizmos.color = Color.magenta;
            Vector3 gestureDirection = ResolveAimDirection(0f, gestureLaunchElevationDegrees);
            Vector3 gestureOrigin = anchor.position + Vector3.up * launchHeight + gestureDirection * launchForwardOffset;
            Gizmos.DrawSphere(gestureOrigin, 0.05f);
            Gizmos.DrawRay(gestureOrigin, gestureDirection);
        }
    }
}
