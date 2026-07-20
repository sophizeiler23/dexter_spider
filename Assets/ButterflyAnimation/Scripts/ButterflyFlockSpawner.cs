using Dexter.Spider;
using UnityEngine;

namespace Dexter.Butterfly
{
    /// <summary>
    /// Spawns additional butterflies around a center point without modifying existing scene instances.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class ButterflyFlockSpawner : MonoBehaviour
    {
        [SerializeField] private GameObject butterflyPrefab;
        [SerializeField] private Transform spawnCenter;
        [SerializeField] private Terrain terrain;
        [SerializeField, Min(1)] private int butterflyCount = 10;
        [SerializeField, Min(1f)] private float spawnRadius = 20f;
        [SerializeField] private float butterflyScale = 10f;
        [SerializeField, Min(0.01f)] private float preyColliderRadius = 0.035f;

        private void Start()
        {
            if (butterflyPrefab == null)
            {
                Debug.LogWarning("ButterflyFlockSpawner: butterfly prefab is not assigned.");
                return;
            }

            if (spawnCenter == null)
                spawnCenter = transform;

            if (terrain == null)
                terrain = Terrain.activeTerrain;

            for (int i = 0; i < butterflyCount; i++)
                SpawnButterfly(i);
        }

        private void SpawnButterfly(int index)
        {
            Vector2 offset = Random.insideUnitCircle * spawnRadius;
            Vector3 position = spawnCenter.position + new Vector3(offset.x, 0f, offset.y);

            if (terrain != null)
                position.y = terrain.SampleHeight(position) + terrain.transform.position.y + Random.Range(0.8f, 2f);

            GameObject instance = Instantiate(
                butterflyPrefab,
                position,
                Quaternion.Euler(0f, Random.Range(0f, 360f), 0f));

            instance.name = $"Butterfly_Flock_{index + 1:00}";
            instance.transform.localScale = Vector3.one * butterflyScale;

            if (instance.GetComponent<ButterflyBehavior>() == null)
                instance.AddComponent<ButterflyBehavior>();

            EnsurePreyComponents(instance);
        }

        private void EnsurePreyComponents(GameObject instance)
        {
            if (instance.GetComponent<CirclePrey>() != null)
                return;

            SphereCollider collider = instance.GetComponent<SphereCollider>();
            if (collider == null)
                collider = instance.AddComponent<SphereCollider>();

            collider.radius = preyColliderRadius;
            collider.center = Vector3.zero;

            CirclePrey prey = instance.AddComponent<CirclePrey>();
        }
    }
}
