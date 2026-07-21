using UnityEngine;
using UnityEngine.InputSystem;

namespace Dexter.Spider
{
    /// <summary>
    /// Fires web shots from the spider's front. Press Space or left mouse button to attack in the test scene.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class SpiderWebAttack : MonoBehaviour
    {
        [SerializeField] private Transform launchAnchor;
        [SerializeField] private Transform aimReference;
        [SerializeField] private string bodyBoneName = "body";

        [Header("Shot")]
        [SerializeField, Min(0f)] private float launchHeight = 0.35f;
        [SerializeField, Min(0f)] private float launchForwardOffset = 0.45f;
        [SerializeField, Range(0f, 0.35f)] private float launchArcLift = 0.12f;
        [SerializeField, Min(0f)] private float cooldownSeconds = 0.8f;

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

            if (!WasAttackPressedThisFrame() || cooldownTimer > 0f)
                return;

            FireWebShot();
            cooldownTimer = cooldownSeconds;
        }

        private static bool WasAttackPressedThisFrame()
        {
            Keyboard keyboard = Keyboard.current;
            if (keyboard != null && keyboard.spaceKey.wasPressedThisFrame)
                return true;

            Mouse mouse = Mouse.current;
            return mouse != null && mouse.leftButton.wasPressedThisFrame;
        }

        public void FireWebShot()
        {
            ResolveReferences();

            Transform anchor = resolvedLaunchAnchor != null ? resolvedLaunchAnchor : transform;
            Vector3 direction = ResolveAimDirection();

            Vector3 origin = anchor.position
                + Vector3.up * launchHeight
                + direction * launchForwardOffset;

            GameObject shotObject = new("SpiderWebShot");
            SpiderWebShot shot = shotObject.AddComponent<SpiderWebShot>();
            shot.Launch(origin, direction);
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

        private Vector3 ResolveAimDirection()
        {
            Vector3 forward = transform.forward;
            forward.y = 0f;
            if (forward.sqrMagnitude < 0.0001f)
            {
                forward = transform.forward;
                forward.y = 0f;
            }

            forward.Normalize();
            return (forward + Vector3.up * launchArcLift).normalized;
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
            Vector3 direction = ResolveAimDirection();

            Vector3 origin = anchor.position
                + Vector3.up * launchHeight
                + direction * launchForwardOffset;

            Gizmos.color = Color.cyan;
            Gizmos.DrawSphere(origin, 0.05f);
            Gizmos.DrawRay(origin, direction);
        }
    }
}
