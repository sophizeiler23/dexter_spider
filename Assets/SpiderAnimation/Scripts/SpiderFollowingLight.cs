using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// Runtime fill light that follows the spider relative to its current
    /// walking surface, keeping the body readable in dark terrain areas.
    /// </summary>
    [DisallowMultipleComponent]
    [DefaultExecutionOrder(1100)]
    public sealed class SpiderFollowingLight : MonoBehaviour
    {
        [SerializeField, Min(0f)] private float surfaceHeight = 1.45f;
        [SerializeField, Min(0f)] private float backwardOffset = 0.35f;
        [SerializeField, Min(0f)] private float intensity = 4.5f;
        [SerializeField, Min(0.1f)] private float range = 9f;
        [SerializeField] private Color lightColor =
            new Color(1f, 0.84f, 0.68f);
        [SerializeField, Min(0.01f)] private float followResponse = 12f;

        private DexterFrontLegIK controller;
        private Transform lightTransform;
        private Light followLight;

        private void Awake()
        {
            controller = GetComponent<DexterFrontLegIK>();
            EnsureLight();
            SnapToTarget();
        }

        private void LateUpdate()
        {
            EnsureLight();
            Vector3 target = GetTargetPosition();
            float blend = 1f - Mathf.Exp(
                -followResponse * Time.deltaTime);
            lightTransform.position = Vector3.Lerp(
                lightTransform.position, target, blend);
            ApplyLightSettings();
        }

        private void EnsureLight()
        {
            if (followLight != null)
                return;
            Transform existing = transform.Find("Spider Follow Light");
            GameObject lightObject;
            if (existing != null)
                lightObject = existing.gameObject;
            else
            {
                lightObject = new GameObject("Spider Follow Light");
                lightObject.transform.SetParent(transform, false);
            }
            lightTransform = lightObject.transform;
            followLight = lightObject.GetComponent<Light>();
            if (followLight == null)
                followLight = lightObject.AddComponent<Light>();
            followLight.type = LightType.Point;
            followLight.shadows = LightShadows.None;
            followLight.renderMode = LightRenderMode.Auto;
            ApplyLightSettings();
        }

        private void ApplyLightSettings()
        {
            if (followLight == null)
                return;
            followLight.color = lightColor;
            followLight.intensity = intensity;
            followLight.range = range;
        }

        private Vector3 GetTargetPosition()
        {
            Vector3 normal = controller != null
                ? controller.SupportNormal
                : transform.up;
            if (normal.sqrMagnitude < 0.001f)
                normal = transform.up;
            normal.Normalize();
            Vector3 forward = Vector3.ProjectOnPlane(
                transform.forward, normal).normalized;
            if (forward.sqrMagnitude < 0.001f)
                forward = transform.forward;
            return transform.position +
                   normal * surfaceHeight -
                   forward * backwardOffset;
        }

        private void SnapToTarget()
        {
            if (lightTransform != null)
                lightTransform.position = GetTargetPosition();
        }

        private void OnValidate()
        {
            surfaceHeight = Mathf.Max(0f, surfaceHeight);
            backwardOffset = Mathf.Max(0f, backwardOffset);
            intensity = Mathf.Max(0f, intensity);
            range = Mathf.Max(0.1f, range);
            followResponse = Mathf.Max(0.01f, followResponse);
            ApplyLightSettings();
        }
    }
}
