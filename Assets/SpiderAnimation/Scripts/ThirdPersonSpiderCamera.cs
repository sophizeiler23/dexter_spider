using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// Keeps a camera smoothly positioned behind and above a moving spider root.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class ThirdPersonSpiderCamera : MonoBehaviour
    {
        [SerializeField] private Transform target;

        [Header("Position")]
        [SerializeField, Min(0f)] private float distanceBehind = 5.5f;
        [SerializeField] private float height = 3.5f;
        [SerializeField, Min(0.01f)] private float positionSmoothTime = 0.15f;

        [Header("View")]
        [SerializeField] private float lookHeight = 0.9f;
        [SerializeField, Min(0f)] private float rotationSpeed = 10f;
        [SerializeField] private bool snapBehindTargetOnEnable = true;

        [Header("Terrain-Aware Framing")]
        [Tooltip("Uses the terrain surface frame to keep the camera behind and outside the wall on steep terrain.")]
        [SerializeField] private bool adjustForSteepSurfaces = true;
        [SerializeField, Range(0f, 85f)] private float steepSurfaceStartAngle = 25f;
        [SerializeField, Range(1f, 85f)] private float steepSurfaceFullAngle = 75f;
        [Tooltip("Distance behind the spider along its wall-travel direction at full steepness.")]
        [SerializeField, Min(0f)] private float steepSurfaceBehindDistance = 2.75f;
        [Tooltip("Distance outward from the wall at full steepness.")]
        [SerializeField, Min(0f)] private float steepSurfaceOutwardDistance = 4.5f;
        [Tooltip("Minimum vertical gap between the camera and the terrain heightmap.")]
        [SerializeField, Min(0f)] private float minimumTerrainClearance = 1.25f;
        [Tooltip("Prevents the camera from entering terrain, trees, or other solid scenery while looking at the spider.")]
        [SerializeField, Min(0.01f)] private float collisionProbeRadius = 0.18f;
        [SerializeField] private LayerMask collisionLayers = ~0;
        [SerializeField, Min(0f)] private float collisionSkin = 0.08f;

        private Vector3 positionVelocity;
        private Terrain activeTerrain;

        private void OnEnable()
        {
            positionVelocity = Vector3.zero;
            if (snapBehindTargetOnEnable && target != null)
                SnapBehindTarget();
        }

        private void LateUpdate()
        {
            if (target == null)
                return;

            Vector3 lookTarget = GetLookTarget();
            Vector3 desiredPosition = GetDesiredPosition();
            desiredPosition = KeepCameraClearOfTerrain(desiredPosition);
            desiredPosition = ResolveViewObstruction(lookTarget, desiredPosition);

            transform.position = Vector3.SmoothDamp(
                transform.position,
                desiredPosition,
                ref positionVelocity,
                positionSmoothTime);
            transform.position = KeepCameraClearOfTerrain(transform.position);

            Vector3 lookDirection = lookTarget - transform.position;
            if (lookDirection.sqrMagnitude < 0.0001f)
                return;

            Quaternion desiredRotation = Quaternion.LookRotation(lookDirection, Vector3.up);
            transform.rotation = Quaternion.Slerp(
                transform.rotation,
                desiredRotation,
                rotationSpeed * Time.deltaTime);
        }

        private Vector3 GetDesiredPosition()
        {
            Vector3 groundForward = GetTargetPlanarForward();
            Vector3 groundPosition = target.position -
                groundForward * distanceBehind + Vector3.up * height;
            if (!TryGetTargetSurfaceFrame(
                    out _, out Vector3 surfaceNormal, out float steepness) ||
                steepness <= 0f)
                return groundPosition;

            Vector3 surfaceRight = Vector3.ProjectOnPlane(
                target.right, surfaceNormal).normalized;
            Vector3 surfaceForward = Vector3.Cross(
                surfaceRight, surfaceNormal).normalized;
            if (surfaceForward.sqrMagnitude < 0.0001f)
                surfaceForward = Vector3.ProjectOnPlane(
                    groundForward, surfaceNormal).normalized;
            if (Vector3.Dot(surfaceForward, target.forward) < 0f)
                surfaceForward = -surfaceForward;

            // On a vertical descent, "behind" is uphill along the wall and
            // "above" is outward from the wall. Building the camera in this
            // surface frame keeps the spider visible instead of looking at the
            // ledge above it.
            Vector3 wallPosition = target.position -
                surfaceForward * steepSurfaceBehindDistance +
                surfaceNormal * steepSurfaceOutwardDistance;
            return Vector3.Lerp(groundPosition, wallPosition, steepness);
        }

        private Vector3 GetLookTarget()
        {
            if (!TryGetTargetSurfaceFrame(
                    out _, out Vector3 surfaceNormal, out float steepness))
                return target.position + Vector3.up * lookHeight;

            Vector3 lookOffset = Vector3.Lerp(
                Vector3.up * lookHeight,
                surfaceNormal * (lookHeight * 0.2f),
                steepness);
            return target.position + lookOffset;
        }

        private float GetSteepSurfaceStrength(Vector3 worldPosition)
        {
            if (!adjustForSteepSurfaces || !TryGetTerrainNormal(
                    worldPosition, out Vector3 normal))
                return 0f;

            float fullAngle = Mathf.Max(
                steepSurfaceStartAngle + 0.01f, steepSurfaceFullAngle);
            return Mathf.InverseLerp(
                steepSurfaceStartAngle,
                fullAngle,
                Vector3.Angle(normal, Vector3.up));
        }

        private Vector3 KeepCameraClearOfTerrain(Vector3 position)
        {
            if (TryGetTargetSurfaceFrame(
                    out Vector3 surfacePoint,
                    out Vector3 surfaceNormal,
                    out float steepness) && steepness > 0.01f)
            {
                float currentClearance = Vector3.Dot(
                    position - surfacePoint, surfaceNormal);
                float requiredClearance = Mathf.Lerp(
                    0f, minimumTerrainClearance, steepness);
                if (currentClearance < requiredClearance)
                    position += surfaceNormal *
                        (requiredClearance - currentClearance);
                return position;
            }

            if (TryGetTerrainHeight(position, out float terrainHeight))
                position.y = Mathf.Max(
                    position.y, terrainHeight + minimumTerrainClearance);
            return position;
        }

        private Vector3 ResolveViewObstruction(
            Vector3 lookTarget,
            Vector3 desiredPosition)
        {
            Vector3 offset = desiredPosition - lookTarget;
            float distance = offset.magnitude;
            if (distance < 0.001f)
                return desiredPosition;

            RaycastHit[] hits = Physics.SphereCastAll(
                lookTarget,
                collisionProbeRadius,
                offset / distance,
                distance,
                collisionLayers,
                QueryTriggerInteraction.Ignore);
            float safeDistance = distance;
            for (int i = 0; i < hits.Length; i++)
            {
                Collider hitCollider = hits[i].collider;
                if (hitCollider == null ||
                    hitCollider.transform.IsChildOf(target))
                    continue;

                if (hitCollider is TerrainCollider)
                    continue;
                safeDistance = Mathf.Min(
                    safeDistance,
                    Mathf.Max(0f, hits[i].distance - collisionSkin));
            }

            return safeDistance < distance
                ? lookTarget + offset / distance * safeDistance
                : desiredPosition;
        }

        private Vector3 GetTargetPlanarForward()
        {
            if (target == null)
                return Vector3.forward;

            Vector3 planarForward = Vector3.ProjectOnPlane(
                target.forward, Vector3.up);
            return planarForward.sqrMagnitude > 0.0001f
                ? planarForward.normalized
                : Vector3.forward;
        }

        private bool TryGetTargetSurfaceFrame(
            out Vector3 surfacePoint,
            out Vector3 surfaceNormal,
            out float steepness)
        {
            surfacePoint = target != null ? target.position : Vector3.zero;
            surfaceNormal = Vector3.up;
            steepness = 0f;
            if (target == null ||
                !TryGetTerrainHeight(target.position, out float height) ||
                !TryGetTerrainNormal(target.position, out surfaceNormal))
                return false;

            surfacePoint = new Vector3(
                target.position.x, height, target.position.z);
            steepness = adjustForSteepSurfaces
                ? GetSteepSurfaceStrength(target.position)
                : 0f;
            return true;
        }

        private bool TryGetTerrainHeight(
            Vector3 worldPosition,
            out float height)
        {
            height = 0f;
            Terrain terrain = GetTerrain();
            if (terrain == null || terrain.terrainData == null)
                return false;

            Vector3 origin = terrain.transform.position;
            Vector3 size = terrain.terrainData.size;
            if (worldPosition.x < origin.x ||
                worldPosition.x > origin.x + size.x ||
                worldPosition.z < origin.z ||
                worldPosition.z > origin.z + size.z)
                return false;

            height = terrain.SampleHeight(worldPosition) + origin.y;
            return true;
        }

        private bool TryGetTerrainNormal(
            Vector3 worldPosition,
            out Vector3 normal)
        {
            normal = Vector3.up;
            Terrain terrain = GetTerrain();
            if (terrain == null || terrain.terrainData == null)
                return false;

            Vector3 origin = terrain.transform.position;
            Vector3 size = terrain.terrainData.size;
            if (size.x <= 0f || size.z <= 0f ||
                worldPosition.x < origin.x ||
                worldPosition.x > origin.x + size.x ||
                worldPosition.z < origin.z ||
                worldPosition.z > origin.z + size.z)
                return false;

            float x = Mathf.InverseLerp(origin.x, origin.x + size.x,
                worldPosition.x);
            float z = Mathf.InverseLerp(origin.z, origin.z + size.z,
                worldPosition.z);
            normal = terrain.terrainData.GetInterpolatedNormal(x, z).normalized;
            return true;
        }

        private Terrain GetTerrain()
        {
            if (activeTerrain == null)
                activeTerrain = Terrain.activeTerrain ??
                    FindAnyObjectByType<Terrain>();
            return activeTerrain;
        }

        private void SnapBehindTarget()
        {
            transform.position = GetDesiredPosition();
            transform.position = KeepCameraClearOfTerrain(transform.position);
            Vector3 lookTarget = GetLookTarget();
            Vector3 lookDirection = lookTarget - transform.position;
            if (lookDirection.sqrMagnitude > 0.0001f)
                transform.rotation = Quaternion.LookRotation(
                    lookDirection, Vector3.up);
        }
    }
}
