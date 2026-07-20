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
        [SerializeField] private bool snapBehindTargetOnEnable = true;
        [Tooltip("If the spider moves farther than this in one frame, snap to a new safe rear view instead of crossing terrain from the old location.")]
        [SerializeField, Min(0.1f)] private float teleportSnapDistance = 3f;

        [Header("Automatic Orientation")]
        [Tooltip("How quickly the automatic behind/above frame follows changes in the spider and terrain. Lower values produce gentler ledge transitions.")]
        [SerializeField, Min(0.01f)] private float surfaceFrameResponse = 4f;
        [Tooltip("How quickly the screen horizon settles toward world-up. The previous camera orientation is preserved while looking almost straight up or down, preventing flips.")]
        [SerializeField, Min(0.01f)] private float cameraUpResponse = 6f;
        [Tooltip("Looks slightly ahead along the spider's path so the spider stays in the lower part of the frame and upcoming terrain remains visible.")]
        [SerializeField, Min(0f)] private float lookAheadDistance = 0.65f;
        [Tooltip("Below this world-up projection the camera preserves its previous horizon instead of choosing an unstable new one.")]
        [SerializeField, Range(0.01f, 0.95f)] private float verticalHorizonThreshold = 0.25f;

        [Header("Terrain-Aware Framing")]
        [Tooltip("Uses the terrain surface frame to keep the camera behind and outside the wall on steep terrain.")]
        [SerializeField] private bool adjustForSteepSurfaces = true;
        [SerializeField, Range(0f, 85f)] private float steepSurfaceStartAngle = 25f;
        [SerializeField, Range(1f, 85f)] private float steepSurfaceFullAngle = 75f;
        [Tooltip("Distance behind the spider along its wall-travel direction at full steepness.")]
        [SerializeField, Min(0f)] private float steepSurfaceBehindDistance = 6f;
        [Tooltip("Distance outward from the wall at full steepness.")]
        [SerializeField, Min(0f)] private float steepSurfaceOutwardDistance = 4f;
        [Tooltip("Minimum vertical gap between the camera and the terrain heightmap.")]
        [SerializeField, Min(0f)] private float minimumTerrainClearance = 1.25f;
        [Tooltip("Prevents the camera from entering terrain, trees, or other solid scenery while looking at the spider.")]
        [SerializeField, Min(0.01f)] private float collisionProbeRadius = 0.18f;
        [SerializeField] private LayerMask collisionLayers = ~0;
        [SerializeField, Min(0f)] private float collisionSkin = 0.08f;

        [Header("Automatic Zoom")]
        [Tooltip("Preferred closest framing distance. A solid obstruction may temporarily require a closer view to keep the camera outside it.")]
        [SerializeField, Min(0.1f)] private float minimumZoomDistance = 2.25f;
        [Tooltip("Farthest distance the camera may use while keeping the spider centered.")]
        [SerializeField, Min(0.1f)] private float maximumZoomDistance = 7.5f;
        [Tooltip("How smoothly the camera zooms back out after terrain or scenery clears. Zooming inward for safety is immediate.")]
        [SerializeField, Min(0.01f)] private float zoomOutSmoothTime = 0.3f;

        private Vector3 positionVelocity;
        private float currentZoomDistance;
        private float zoomVelocity;
        private Terrain activeTerrain;
        private Vector3 lastTargetPosition;
        private bool hasLastTargetPosition;
        private Vector3 followForward = Vector3.forward;
        private Vector3 followUp = Vector3.up;
        private Vector3 cameraUp = Vector3.up;
        private float followSteepness;
        private bool hasFollowFrame;

        private void OnEnable()
        {
            positionVelocity = Vector3.zero;
            zoomVelocity = 0f;
            currentZoomDistance = 0f;
            hasLastTargetPosition = target != null;
            if (target != null)
            {
                lastTargetPosition = target.position;
                ResetAutomaticFrame();
            }
            if (snapBehindTargetOnEnable && target != null)
                SnapBehindTarget();
        }

        private void LateUpdate()
        {
            if (target == null)
                return;

            bool targetTeleported = hasLastTargetPosition &&
                Vector3.Distance(target.position, lastTargetPosition) >=
                teleportSnapDistance;
            if (targetTeleported)
            {
                ResetAutomaticFrame();
                currentZoomDistance = 0f;
            }
            else
            {
                UpdateAutomaticFrame();
            }

            Vector3 lookTarget = GetLookTarget();
            Vector3 preferredPosition = GetDesiredPosition();
            Vector3 desiredPosition = GetZoomedPosition(
                lookTarget, preferredPosition);

            Vector3 smoothedPosition;
            if (targetTeleported)
            {
                smoothedPosition = desiredPosition;
                positionVelocity = Vector3.zero;
                zoomVelocity = 0f;
            }
            else
            {
                smoothedPosition = Vector3.SmoothDamp(
                    transform.position,
                    desiredPosition,
                    ref positionVelocity,
                    positionSmoothTime);
            }
            // Smoothing can cut across a ledge even when both endpoints are
            // safe. Re-test the actual position every frame, including terrain,
            // then enforce height/surface clearance as the final hard guard.
            smoothedPosition = ResolveViewObstruction(
                lookTarget, smoothedPosition);
            smoothedPosition = KeepCameraClearOfTerrain(smoothedPosition);
            smoothedPosition = ResolveViewObstruction(
                lookTarget, smoothedPosition);
            transform.position = KeepCameraClearOfTerrain(smoothedPosition);
            lastTargetPosition = target.position;
            hasLastTargetPosition = true;

            Vector3 lookDirection = lookTarget - transform.position;
            if (lookDirection.sqrMagnitude < 0.0001f)
                return;

            Vector3 viewForward = lookDirection.normalized;
            Vector3 stableCameraUp = GetStableCameraUp(
                viewForward, Time.deltaTime);

            Quaternion desiredRotation = Quaternion.LookRotation(
                viewForward, stableCameraUp);
            // The target must remain in frame even when the rear surface frame
            // changes rapidly at a ledge. Position and zoom remain smoothed;
            // aim directly at the spider so rotational lag cannot look at the
            // old ledge or move the spider off-screen.
            transform.rotation = desiredRotation;
        }

        private Vector3 GetDesiredPosition()
        {
            float behindDistance = Mathf.Lerp(
                distanceBehind,
                steepSurfaceBehindDistance,
                followSteepness);
            float aboveDistance = Mathf.Lerp(
                height,
                steepSurfaceOutwardDistance,
                followSteepness);
            return target.position - followForward * behindDistance +
                   followUp * aboveDistance;
        }

        private Vector3 GetLookTarget()
        {
            float surfaceLookHeight = Mathf.Lerp(
                lookHeight, lookHeight * 0.2f, followSteepness);
            return target.position + followUp * surfaceLookHeight +
                   followForward * lookAheadDistance;
        }

        private void ResetAutomaticFrame()
        {
            Vector3 desiredUp = Vector3.up;
            float desiredSteepness = 0f;
            if (TryGetTargetSurfaceFrame(
                    out _, out Vector3 surfaceNormal, out float steepness))
            {
                desiredUp = surfaceNormal;
                desiredSteepness = steepness;
            }

            Vector3 desiredForward = GetTargetSurfaceForward(desiredUp);
            followUp = desiredUp;
            followForward = desiredForward;
            followSteepness = desiredSteepness;
            cameraUp = Vector3.up;
            hasFollowFrame = true;
        }

        private void UpdateAutomaticFrame()
        {
            Vector3 desiredUp = Vector3.up;
            float desiredSteepness = 0f;
            if (TryGetTargetSurfaceFrame(
                    out _, out Vector3 surfaceNormal, out float steepness))
            {
                desiredUp = surfaceNormal;
                desiredSteepness = steepness;
            }

            Vector3 desiredForward = GetTargetSurfaceForward(desiredUp);
            if (!hasFollowFrame)
            {
                followUp = desiredUp;
                followForward = desiredForward;
                followSteepness = desiredSteepness;
                cameraUp = Vector3.up;
                hasFollowFrame = true;
                return;
            }

            // A one-frame 180-degree change is a surface-frame ambiguity, not
            // a real player turn. Real turning changes gradually, so retaining
            // the current hemisphere removes ledge flips without blocking it.
            if (Vector3.Dot(followForward, desiredForward) < -0.5f)
                desiredForward = -desiredForward;

            float blend = 1f - Mathf.Exp(
                -surfaceFrameResponse * Time.deltaTime);
            followUp = Vector3.Slerp(
                followUp, desiredUp, blend).normalized;
            desiredForward = Vector3.ProjectOnPlane(
                desiredForward, followUp).normalized;
            if (desiredForward.sqrMagnitude < 0.0001f)
                desiredForward = followForward;
            followForward = Vector3.Slerp(
                followForward, desiredForward, blend).normalized;
            followForward = Vector3.ProjectOnPlane(
                followForward, followUp).normalized;
            if (followForward.sqrMagnitude < 0.0001f)
                followForward = desiredForward;
            followSteepness = Mathf.Lerp(
                followSteepness, desiredSteepness, blend);
        }

        private Vector3 GetStableCameraUp(
            Vector3 viewForward,
            float deltaTime)
        {
            // Parallel-transport the previous screen-up onto the new view
            // plane. This retains a continuous horizon through vertical views,
            // where LookRotation(viewForward, Vector3.up) is under-defined.
            Vector3 transportedUp = Vector3.ProjectOnPlane(
                cameraUp, viewForward);
            if (transportedUp.sqrMagnitude < 0.0001f)
                transportedUp = Vector3.ProjectOnPlane(
                    transform.up, viewForward);
            if (transportedUp.sqrMagnitude < 0.0001f)
                transportedUp = Vector3.ProjectOnPlane(
                    followUp, viewForward);
            if (transportedUp.sqrMagnitude < 0.0001f)
                transportedUp = Vector3.Cross(
                    viewForward, transform.right);
            transportedUp.Normalize();

            Vector3 projectedWorldUp = Vector3.ProjectOnPlane(
                Vector3.up, viewForward);
            float worldUpStrength = projectedWorldUp.magnitude;
            Vector3 desiredUp = transportedUp;
            if (worldUpStrength > 0.0001f)
            {
                projectedWorldUp /= worldUpStrength;
                if (Vector3.Dot(transportedUp, projectedWorldUp) < 0f)
                    projectedWorldUp = -projectedWorldUp;

                // Near a straight-up/down view, fade toward the transported
                // horizon instead of allowing world-up to choose either side.
                float horizonWeight = Mathf.SmoothStep(
                    0f,
                    1f,
                    Mathf.InverseLerp(
                        verticalHorizonThreshold,
                        Mathf.Min(1f, verticalHorizonThreshold + 0.35f),
                        worldUpStrength));
                desiredUp = Vector3.Slerp(
                    transportedUp, projectedWorldUp, horizonWeight).normalized;
            }

            float upBlend = 1f - Mathf.Exp(
                -cameraUpResponse * Mathf.Max(0f, deltaTime));
            cameraUp = Vector3.Slerp(
                transportedUp, desiredUp, upBlend).normalized;
            cameraUp = Vector3.ProjectOnPlane(
                cameraUp, viewForward).normalized;
            return cameraUp.sqrMagnitude > 0.0001f
                ? cameraUp
                : transportedUp;
        }

        private Vector3 GetTargetSurfaceForward(Vector3 surfaceUp)
        {
            Vector3 desiredForward = Vector3.ProjectOnPlane(
                target.forward, surfaceUp).normalized;
            if (desiredForward.sqrMagnitude >= 0.0001f)
                return desiredForward;

            if (hasLastTargetPosition)
            {
                Vector3 movementForward = Vector3.ProjectOnPlane(
                    target.position - lastTargetPosition,
                    surfaceUp).normalized;
                if (movementForward.sqrMagnitude >= 0.0001f)
                    return movementForward;
            }

            desiredForward = Vector3.ProjectOnPlane(
                followForward, surfaceUp).normalized;
            if (desiredForward.sqrMagnitude >= 0.0001f)
                return desiredForward;

            desiredForward = Vector3.ProjectOnPlane(
                GetTargetPlanarForward(), surfaceUp).normalized;
            return desiredForward.sqrMagnitude >= 0.0001f
                ? desiredForward
                : Vector3.forward;
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
            }

            // Always test the terrain directly below the camera too. The old
            // steep-surface early return skipped this check and allowed the
            // camera to remain underneath the heightfield after a descent.
            if (TryGetTerrainHeight(position, out float terrainHeight))
                position.y = Mathf.Max(
                    position.y, terrainHeight + minimumTerrainClearance);
            return position;
        }

        private Vector3 GetZoomedPosition(
            Vector3 lookTarget,
            Vector3 preferredPosition)
        {
            preferredPosition = KeepCameraClearOfTerrain(preferredPosition);
            Vector3 offset = preferredPosition - lookTarget;
            float offsetDistance = offset.magnitude;
            if (offsetDistance < 0.001f)
                return preferredPosition;

            Vector3 viewDirection = offset / offsetDistance;
            float maximumDistance = Mathf.Max(
                minimumZoomDistance, maximumZoomDistance);
            float preferredDistance = Mathf.Clamp(
                offsetDistance, minimumZoomDistance, maximumDistance);
            preferredPosition = lookTarget +
                viewDirection * preferredDistance;

            Vector3 unobstructedPosition = ResolveViewObstruction(
                lookTarget, preferredPosition);
            float availableDistance = Vector3.Distance(
                lookTarget, unobstructedPosition);
            float targetZoomDistance = Mathf.Min(
                preferredDistance, availableDistance);

            if (currentZoomDistance <= 0f)
                currentZoomDistance = targetZoomDistance;
            else if (targetZoomDistance < currentZoomDistance)
            {
                // Moving inward is a collision response, so it cannot lag.
                currentZoomDistance = targetZoomDistance;
                zoomVelocity = 0f;
            }
            else
            {
                currentZoomDistance = Mathf.SmoothDamp(
                    currentZoomDistance,
                    targetZoomDistance,
                    ref zoomVelocity,
                    zoomOutSmoothTime);
            }

            currentZoomDistance = Mathf.Min(
                currentZoomDistance, availableDistance);
            return lookTarget + viewDirection * currentZoomDistance;
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

            // The target's forward axis points almost vertically while it is
            // wall-aligned, so projecting that axis can collapse to zero. Its
            // lateral axis remains horizontal; reconstructing yaw from it is
            // the same stable method used by the locomotion gait.
            Vector3 planarRight = Vector3.ProjectOnPlane(
                target.right, Vector3.up).normalized;
            if (planarRight.sqrMagnitude < 0.5f)
                planarRight = Vector3.right;

            Vector3 planarForward = Vector3.Cross(
                Vector3.up, planarRight).normalized;
            if (Vector3.Dot(planarForward, target.forward) < 0f)
                planarForward = -planarForward;
            return planarForward;
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
            Vector3 lookTarget = GetLookTarget();
            transform.position = GetZoomedPosition(
                lookTarget, GetDesiredPosition());
            transform.position = KeepCameraClearOfTerrain(transform.position);
            Vector3 lookDirection = lookTarget - transform.position;
            if (lookDirection.sqrMagnitude > 0.0001f)
            {
                Vector3 viewForward = lookDirection.normalized;
                cameraUp = Vector3.up;
                Vector3 stableCameraUp = GetStableCameraUp(
                    viewForward, 1f);
                transform.rotation = Quaternion.LookRotation(
                    viewForward, stableCameraUp);
            }
        }
    }
}
