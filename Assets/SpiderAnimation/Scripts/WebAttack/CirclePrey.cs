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

        private Renderer preyRenderer;
        private bool isCaptured;

        public bool IsCaptured => isCaptured;
        public float Radius => radius;

        private void Awake()
        {
            preyRenderer = GetComponent<Renderer>();
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
            if (preyRenderer == null)
                preyRenderer = GetComponent<Renderer>();

            if (preyRenderer == null)
                return;

            Material material = preyRenderer.material;
            if (material.HasProperty("_BaseColor"))
                material.SetColor("_BaseColor", color);
            else
                material.color = color;
        }

        private void OnDrawGizmosSelected()
        {
            Gizmos.color = isCaptured ? Color.gray : Color.red;
            Gizmos.DrawWireSphere(transform.position, radius);
        }
    }
}
