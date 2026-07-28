using UnityEngine;

namespace Dexter.Spider
{
    [ExecuteAlways]
    [DisallowMultipleComponent]
    public sealed class SpiderMaterialVariant : MonoBehaviour
    {
        [SerializeField] private Material bodyMaterial;
        [SerializeField] private Material legMaterial;

        private void OnEnable() => Apply();
        private void OnValidate() => Apply();

        public void Apply()
        {
            if (bodyMaterial == null && legMaterial == null) return;
            foreach (var renderer in GetComponentsInChildren<Renderer>(true))
            {
                if (renderer is ParticleSystemRenderer) continue;
                var material = renderer.transform.name.ToLowerInvariant().Contains("body")
                    ? bodyMaterial : legMaterial;
                if (material == null) continue;
                var materials = renderer.sharedMaterials;
                for (var i = 0; i < materials.Length; i++) materials[i] = material;
                renderer.sharedMaterials = materials;
            }
        }
    }
}
