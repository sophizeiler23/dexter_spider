using UnityEngine;

namespace Dexter.Spider
{
    [DisallowMultipleComponent]
    public sealed class SpiderColliderRig : MonoBehaviour
    {
        [SerializeField] private float bodyRadius = 0.42f;
        [SerializeField] private float bodyHeight = 0.55f;
        [SerializeField] private float legRadius = 0.055f;

        private void Awake() => Build();

        private void Build()
        {
            CapsuleCollider body = GetComponent<CapsuleCollider>();
            if (body == null) body = gameObject.AddComponent<CapsuleCollider>();
            body.radius = bodyRadius;
            body.height = bodyHeight;
            body.direction = 1;
            body.center = Vector3.up * bodyHeight * 0.5f;
            body.isTrigger = false;

            foreach (Transform child in GetComponentsInChildren<Transform>(true))
            {
                if (child == transform || !child.name.ToLowerInvariant().Contains("leg")) continue;
                if (child.GetComponent<Collider>() != null) continue;
                CapsuleCollider leg = child.gameObject.AddComponent<CapsuleCollider>();
                leg.radius = legRadius;
                leg.height = Mathf.Max(legRadius * 2f, child.localScale.y);
                leg.direction = 1;
                leg.isTrigger = false;
                Physics.IgnoreCollision(body, leg, true);
            }
        }
    }
}
