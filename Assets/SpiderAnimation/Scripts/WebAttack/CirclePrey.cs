using Dexter.Butterfly;
using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// Prey target for web shots. On impact the butterfly stops animating and falls with physics.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class CirclePrey : MonoBehaviour
    {
        [SerializeField, Min(0.01f)] private float radius = 0.35f;

        [Header("Captured")]
        [SerializeField] private Color normalColor = new(0.95f, 0.15f, 0.1f, 1f);
        [SerializeField] private Color capturedColor = new(0.55f, 0.55f, 0.6f, 1f);

        [Header("Knockdown")]
        [SerializeField, Min(0.0001f)] private float bodyMassKg = 0.0008f;
        [SerializeField, Min(0f)] private float knockdownImpulse = 0.35f;
        [SerializeField, Min(0f)] private float downwardImpulse = 0.12f;
        [SerializeField, Min(0f)] private float tumbleTorque = 0.08f;
        [SerializeField, Min(0f)] private float linearDrag = 0.2f;
        [SerializeField, Min(0f)] private float angularDrag = 0.5f;

        private Renderer[] preyRenderers;
        private bool isCaptured;
        private Rigidbody body;

        public bool IsCaptured => isCaptured;
        public float Radius => radius;

        /// <summary>World-space size used to fit a capture web around this prey.</summary>
        public float CaptureExtent
        {
            get
            {
                float extent = radius;
                Renderer[] renderers = preyRenderers;
                if (renderers == null || renderers.Length == 0)
                    renderers = GetComponentsInChildren<Renderer>();

                for (int i = 0; i < renderers.Length; i++)
                {
                    Renderer renderer = renderers[i];
                    if (renderer == null)
                        continue;

                    Bounds bounds = renderer.bounds;
                    float horizontal = new Vector2(bounds.extents.x, bounds.extents.z).magnitude;
                    extent = Mathf.Max(extent, horizontal, bounds.extents.y);
                }

                return extent;
            }
        }

        private void Awake()
        {
            preyRenderers = GetComponentsInChildren<Renderer>();
            if (GetComponent<Renderer>() != null)
                ApplyColor(normalColor);
        }

        private void OnValidate()
        {
            radius = Mathf.Max(0.01f, radius);
        }

        public void KnockDownFromWeb(Vector3 impactVelocity, Vector3 impactPoint)
        {
            if (isCaptured)
                return;

            isCaptured = true;
            ApplyColor(capturedColor);
            StopButterflyMotion();
            EnableKnockdownPhysics(impactVelocity, impactPoint);
        }

        private void StopButterflyMotion()
        {
            ButterflyBehavior behavior = GetComponent<ButterflyBehavior>();
            if (behavior != null)
                behavior.enabled = false;

            ButterflyWingFlapPreview wingFlap = GetComponent<ButterflyWingFlapPreview>();
            if (wingFlap != null)
            {
                wingFlap.StopFlapping();
                wingFlap.enabled = false;
            }

            Animator animator = GetComponent<Animator>();
            if (animator != null)
                animator.enabled = false;
        }

        private void EnableKnockdownPhysics(Vector3 impactVelocity, Vector3 impactPoint)
        {
            body = GetComponent<Rigidbody>();
            if (body == null)
                body = gameObject.AddComponent<Rigidbody>();

            body.mass = bodyMassKg;
            body.linearDamping = linearDrag;
            body.angularDamping = angularDrag;
            body.useGravity = true;
            body.interpolation = RigidbodyInterpolation.Interpolate;
            body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;

            Collider collider = GetComponent<Collider>();
            if (collider == null)
                collider = GetComponentInChildren<Collider>();

            if (collider != null && !collider.enabled)
                collider.enabled = true;

            Vector3 pushDirection = impactVelocity.sqrMagnitude > 0.01f
                ? impactVelocity.normalized
                : transform.position - impactPoint;

            if (pushDirection.sqrMagnitude < 0.0001f)
                pushDirection = Vector3.forward;

            pushDirection.y = 0f;
            if (pushDirection.sqrMagnitude < 0.0001f)
                pushDirection = Vector3.down;
            else
                pushDirection.Normalize();

            Vector3 impulse = pushDirection * knockdownImpulse +
                Vector3.down * downwardImpulse;
            body.AddForceAtPosition(impulse, impactPoint, ForceMode.Impulse);
            body.AddTorque(
                Random.insideUnitSphere * tumbleTorque,
                ForceMode.Impulse);
        }

        private void ApplyColor(Color color)
        {
            if (preyRenderers == null || preyRenderers.Length == 0)
                preyRenderers = GetComponentsInChildren<Renderer>();

            for (int i = 0; i < preyRenderers.Length; i++)
            {
                Renderer renderer = preyRenderers[i];
                if (renderer == null)
                    continue;

                Material material = renderer.material;
                if (material.HasProperty("_BaseColor"))
                    material.SetColor("_BaseColor", color);
                else
                    material.color = color;
            }
        }

        private void OnDrawGizmosSelected()
        {
            Gizmos.color = isCaptured ? Color.gray : Color.red;
            Gizmos.DrawWireSphere(transform.position, radius);
        }
    }
}
