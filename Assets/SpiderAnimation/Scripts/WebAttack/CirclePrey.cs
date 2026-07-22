using Dexter.Butterfly;
using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// Prey target for web shots. On impact the prey switches to the Hit state: its animation stops,
    /// a white cocoon grows around it, and gravity/physics take over.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class CirclePrey : MonoBehaviour
    {
        public enum PreyState
        {
            Alive,
            Hit
        }

        [SerializeField, Min(0.01f)] private float radius = 0.35f;

        [Header("Captured")]
        [SerializeField] private Color normalColor = new(0.95f, 0.15f, 0.1f, 1f);
        [SerializeField] private Color capturedColor = new(0.55f, 0.55f, 0.6f, 1f);

        [Header("Knockdown")]
        [SerializeField, Min(0.0001f)] private float bodyMassKg = 0.0008f;
        [SerializeField, Min(0f)] private float knockdownSpeed = 0.35f;
        [SerializeField, Min(0f)] private float downwardSpeed = 0.12f;
        [SerializeField, Min(0f)] private float tumbleAngularSpeed = 0.08f;
        [SerializeField, Min(0f)] private float linearDrag = 0.2f;
        [SerializeField, Min(0f)] private float angularDrag = 0.5f;

        [Header("Cocoon")]
        [SerializeField] private Color cocoonColor = new(0.97f, 0.97f, 0.95f, 1f);
        [SerializeField, Min(0.05f)] private float cocoonGrowDuration = 1.1f;
        [SerializeField, Range(1f, 3f)] private float cocoonPadding = 1.35f;
        [SerializeField, Range(0.8f, 1.6f)] private float cocoonLengthStretch = 1.15f;

        private Renderer[] preyRenderers;
        private PreyState state = PreyState.Alive;
        private Rigidbody body;

        private Transform cocoonTransform;
        private float cocoonTimer;
        private bool cocoonGrowing;
        private float cocoonTargetDiameter;

        public PreyState State => state;
        public bool IsCaptured => state == PreyState.Hit;
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

        private void Update()
        {
            if (!cocoonGrowing)
                return;

            cocoonTimer += Time.deltaTime;
            float t = cocoonGrowDuration > 0f ? Mathf.Clamp01(cocoonTimer / cocoonGrowDuration) : 1f;
            float eased = 1f - Mathf.Pow(1f - t, 3f);
            ApplyCocoonScale(eased);

            if (t >= 1f)
                cocoonGrowing = false;
        }

        public void KnockDownFromWeb(Vector3 impactVelocity, Vector3 impactPoint)
        {
            if (state == PreyState.Hit)
                return;

            state = PreyState.Hit;
            ApplyColor(capturedColor);
            StopButterflyMotion();
            BeginCocoonGrowth();
            EnableKnockdownPhysics(impactVelocity, impactPoint);
        }

        private void BeginCocoonGrowth()
        {
            cocoonTargetDiameter = Mathf.Max(0.05f, CaptureExtent * 2f * cocoonPadding);
            cocoonTransform = CreateCocoonSphere();
            cocoonTimer = 0f;
            cocoonGrowing = true;
            ApplyCocoonScale(0f);
        }

        private Transform CreateCocoonSphere()
        {
            GameObject cocoon = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            cocoon.name = "WebCocoon";
            cocoon.transform.SetParent(transform, false);
            cocoon.transform.localPosition = ComputeLocalBoundsCenter();
            cocoon.transform.localRotation = Quaternion.identity;

            Collider cocoonCollider = cocoon.GetComponent<Collider>();
            if (cocoonCollider != null)
                Destroy(cocoonCollider);

            MeshRenderer cocoonRenderer = cocoon.GetComponent<MeshRenderer>();
            cocoonRenderer.material = SpiderWebMaterialFactory.CreateCocoonMaterial(cocoonColor);
            cocoonRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            cocoonRenderer.receiveShadows = false;

            return cocoon.transform;
        }

        private Vector3 ComputeLocalBoundsCenter()
        {
            Renderer[] renderers = preyRenderers;
            if (renderers == null || renderers.Length == 0)
                renderers = GetComponentsInChildren<Renderer>();

            if (renderers.Length == 0)
                return Vector3.zero;

            Bounds bounds = renderers[0].bounds;
            for (int i = 1; i < renderers.Length; i++)
                bounds.Encapsulate(renderers[i].bounds);

            return transform.InverseTransformPoint(bounds.center);
        }

        private void ApplyCocoonScale(float progress)
        {
            if (cocoonTransform == null)
                return;

            float diameter = cocoonTargetDiameter * Mathf.Max(0.001f, progress);

            // cocoonTargetDiameter is a world-space size, but localScale is multiplied by the
            // parent's world scale (butterflies are often spawned scaled up, e.g. x10), so divide
            // it back out here to keep the cocoon's actual world size correct.
            Vector3 parentScale = transform.lossyScale;
            float scaleX = Mathf.Approximately(parentScale.x, 0f) ? 1f : parentScale.x;
            float scaleY = Mathf.Approximately(parentScale.y, 0f) ? 1f : parentScale.y;
            float scaleZ = Mathf.Approximately(parentScale.z, 0f) ? 1f : parentScale.z;

            cocoonTransform.localScale = new Vector3(
                diameter / scaleX,
                diameter * cocoonLengthStretch / scaleY,
                diameter / scaleZ);
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

            // VelocityChange ignores mass/inertia, so these stay sane, gentle knockdown
            // speeds even though the butterfly's mass and inertia tensor are tiny
            // (ForceMode.Impulse would divide by mass and launch it at hundreds of m/s).
            Vector3 velocityChange = pushDirection * knockdownSpeed +
                Vector3.down * downwardSpeed;
            body.AddForceAtPosition(velocityChange, impactPoint, ForceMode.VelocityChange);
            body.AddTorque(
                Random.insideUnitSphere * tumbleAngularSpeed,
                ForceMode.VelocityChange);
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
            Gizmos.color = state == PreyState.Hit ? Color.gray : Color.red;
            Gizmos.DrawWireSphere(transform.position, radius);
        }
    }
}
