using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// Placeholder prey represented as a flat circle. Captured when enveloped by an expanding web net.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class CirclePrey : MonoBehaviour
    {
        [SerializeField, Min(0.01f)] private float radius = 0.35f;

        [Header("Captured")]
        [SerializeField] private Color normalColor = new(0.95f, 0.15f, 0.1f, 1f);
        [SerializeField] private Color capturedColor = new(0.55f, 0.55f, 0.6f, 1f);

        private Renderer[] preyRenderers;
        private bool isCaptured;

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

        public void Capture(Transform webAnchor)
        {
            if (isCaptured || webAnchor == null)
                return;

            isCaptured = true;
            ApplyColor(capturedColor);

            Transform preyTransform = transform;
            preyTransform.SetParent(webAnchor, true);

            Vector3 pullTarget = webAnchor.position;
            pullTarget.y = preyTransform.position.y;
            preyTransform.position = Vector3.Lerp(preyTransform.position, pullTarget, 0.65f);
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
