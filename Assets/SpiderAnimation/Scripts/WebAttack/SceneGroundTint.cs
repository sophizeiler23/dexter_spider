using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// Applies a distinct ground tint so prey and floor are easy to tell apart in test scenes.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class SceneGroundTint : MonoBehaviour
    {
        [SerializeField] private Color groundColor = new(0.28f, 0.42f, 0.24f, 1f);

        private void Awake()
        {
            Renderer renderer = GetComponent<Renderer>();
            if (renderer == null)
                return;

            Material material = renderer.material;
            if (material.HasProperty("_BaseColor"))
                material.SetColor("_BaseColor", groundColor);
            else
                material.color = groundColor;
        }
    }
}
