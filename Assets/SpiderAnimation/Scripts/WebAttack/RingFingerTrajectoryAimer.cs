using System.Collections.Generic;
using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// Alternative ring-finger aiming system: instead of a quick flick gesture, this aims
    /// continuously while the finger stays extended past a small threshold, drawing a live
    /// semi-transparent dotted parabola (ending in a cross where it would land) that always
    /// mirrors the exact shot the current finger vector would produce. The shot fires using the
    /// last-aimed vector the instant the finger drops back below the threshold (release).
    /// Uses the same azimuth/speed mapping as <see cref="RingFingerShotAimer"/>: azimuth is the
    /// angle of the finger vector from the Y axis, speed is mapped from the vector's magnitude, and
    /// elevation is <see cref="SpiderWebAttack"/>'s fixed gesture elevation.
    /// Only acts while <see cref="SpiderWebAttack.AimingMode"/> is
    /// <see cref="SpiderWebAttack.WebAimingMode.TrajectoryPreview"/> (see
    /// <see cref="RingFingerAimerBase.IsActive"/>); otherwise sits idle so it can be left attached
    /// alongside <see cref="RingFingerShotAimer"/> and switched between via that single setting.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class RingFingerTrajectoryAimer : RingFingerAimerBase
    {
        [Header("Preview Thresholds")]
        [Tooltip("Relative Y the ring finger must rise above (from baseline) before the trajectory preview appears and this aimer starts tracking the current vector.")]
        [SerializeField, Min(0f)] private float previewActivationThreshold = 0.08f;
        [Tooltip("Relative Y the finger must fall back to/below to count as 'released' — this fires the shot using the last vector the preview showed. Should be lower than the activation threshold (hysteresis).")]
        [SerializeField, Min(0f)] private float previewReleaseThreshold = 0.05f;
        [Tooltip("The last-aimed vector's magnitude must reach at least this for a release to actually fire a shot; otherwise the preview just disappears (treated as an aborted aim).")]
        [SerializeField, Min(0f)] private float minFireMagnitude = 0.15f;

        [Header("Azimuth Mapping")]
        [Tooltip("Current-vector angle from the Y axis is clamped to +/- this many degrees before being applied as the shot's azimuth.")]
        [SerializeField, Range(0f, 180f)] private float maxAzimuthDegrees = 60f;
        [Tooltip("Flips the left/right steering direction. Enable if aiming left with the finger sends the shot right, or vice versa.")]
        [SerializeField] private bool invertAzimuth;

        [Header("Speed Mapping")]
        [Tooltip("Vector magnitude at or below this maps to the minimum launch speed (and the shortest previewed arc).")]
        [SerializeField, Min(0f)] private float minMagnitudeForMinSpeed = 0.2f;
        [Tooltip("Vector magnitude at or above this maps to the maximum launch speed (and the longest previewed arc). Stronger extensions are clamped to the same maximum.")]
        [SerializeField, Min(0.01f)] private float maxMagnitudeForMaxSpeed = 1.5f;
        [Tooltip("Launch speed (and preview arc) used for the weakest aim that still clears 'Preview Activation Threshold'.")]
        [SerializeField, Min(0f)] private float minLaunchSpeed = 6f;
        [Tooltip("Launch speed (and preview arc) used once the vector magnitude reaches 'Max Magnitude For Max Speed' or beyond.")]
        [SerializeField, Min(0f)] private float maxLaunchSpeed = 16f;

        [Header("Trajectory Preview Visual")]
        [Tooltip("Color (including alpha) of the dotted trajectory line and landing cross. Keep alpha well below 1 so it clearly reads as a preview rather than an actual web.")]
        [SerializeField] private Color previewColor = new(0.85f, 0.93f, 1f, 0.55f);
        [Tooltip("Line width of the dotted trajectory preview.")]
        [SerializeField, Min(0.001f)] private float lineWidth = 0.035f;
        [Tooltip("World-space length of one dash+gap tile along the trajectory line. Smaller values pack more, shorter dashes into the same arc.")]
        [SerializeField, Min(0.05f)] private float dashTileLength = 0.35f;
        [Tooltip("Half-length of each of the two strokes forming the 'X' landing marker.")]
        [SerializeField, Min(0.01f)] private float crossSize = 0.25f;
        [Tooltip("Line width of the landing marker's strokes.")]
        [SerializeField, Min(0.001f)] private float crossLineWidth = 0.04f;
        [Tooltip("Time step used to numerically integrate the preview's simulated arc. Smaller is smoother but samples more points; must be small enough that fast shots don't visibly skip through thin geometry.")]
        [SerializeField, Range(0.005f, 0.05f)] private float simulationTimeStep = 0.02f;
        [Tooltip("Safety cap on how long (in simulated seconds) the preview keeps integrating the arc if it never hits anything and never exceeds the shot's max range.")]
        [SerializeField, Min(0.2f)] private float maxSimulationSeconds = 3f;
        [Tooltip("Layers the previewed arc can land on. Should match whatever SpiderWebShot's own flight collision effectively hits (defaults to everything).")]
        [SerializeField] private LayerMask hitMask = ~0;

        private bool isPreviewing;
        private Vector2 lastAimedRelative;

        private Transform previewRoot;
        private LineRenderer trajectoryLine;
        private LineRenderer crossStrokeA;
        private LineRenderer crossStrokeB;
        private Material trajectoryMaterial;
        private readonly List<Vector3> trajectoryPointsScratch = new();

        protected override SpiderWebAttack.WebAimingMode RequiredMode => SpiderWebAttack.WebAimingMode.TrajectoryPreview;

        private void OnValidate()
        {
            previewReleaseThreshold = Mathf.Min(previewReleaseThreshold, previewActivationThreshold);
            maxMagnitudeForMaxSpeed = Mathf.Max(maxMagnitudeForMaxSpeed, minMagnitudeForMinSpeed + 0.01f);
        }

        private void OnDisable()
        {
            isPreviewing = false;
            HidePreview();
        }

        protected override void OnInactive()
        {
            // Switching away mid-aim cancels the preview without firing — only a release while this
            // is the active aiming system fires a shot.
            if (isPreviewing)
            {
                isPreviewing = false;
                HidePreview();
            }
        }

        protected override void OnActiveUpdate(Vector2 relative)
        {
            if (isPreviewing)
            {
                if (relative.y > previewReleaseThreshold)
                {
                    lastAimedRelative = relative;
                    ShowPreview(relative);
                }
                else
                {
                    Vector2 releaseVector = lastAimedRelative;
                    isPreviewing = false;
                    HidePreview();
                    TryFire(releaseVector);
                }

                return;
            }

            if (relative.y > previewActivationThreshold)
            {
                isPreviewing = true;
                lastAimedRelative = relative;
                ShowPreview(relative);
            }
        }

        private void TryFire(Vector2 relative)
        {
            float magnitude = relative.magnitude;
            if (magnitude < minFireMagnitude)
            {
                if (LogGestureDebugInfo)
                {
                    Debug.Log(
                        $"{nameof(RingFingerTrajectoryAimer)}: release filtered, magnitude {magnitude:F3} " +
                        $"below minimum {minFireMagnitude:F3}.",
                        this);
                }
                return;
            }

            ComputeAim(relative, out float azimuthDegrees, out float launchSpeed);

            if (LogGestureDebugInfo)
            {
                Debug.Log(
                    $"{nameof(RingFingerTrajectoryAimer)}: firing shot on release. vector=({relative.x:F3}, {relative.y:F3}) " +
                    $"azimuth={azimuthDegrees:F1} deg, speed={launchSpeed:F2}.",
                    this);
            }

            WebAttack.FireWebShot(azimuthDegrees, launchSpeed);
        }

        /// <summary>Same azimuth/speed mapping as <see cref="RingFingerShotAimer"/>, applied to the current (not peak) vector.</summary>
        private void ComputeAim(Vector2 relative, out float azimuthDegrees, out float launchSpeed)
        {
            azimuthDegrees = Mathf.Atan2(relative.x, relative.y) * Mathf.Rad2Deg;
            if (invertAzimuth)
                azimuthDegrees = -azimuthDegrees;
            azimuthDegrees = Mathf.Clamp(azimuthDegrees, -maxAzimuthDegrees, maxAzimuthDegrees);

            float speedT = Mathf.InverseLerp(minMagnitudeForMinSpeed, maxMagnitudeForMaxSpeed, relative.magnitude);
            launchSpeed = Mathf.Lerp(minLaunchSpeed, maxLaunchSpeed, Mathf.Clamp01(speedT));
        }

        private void ShowPreview(Vector2 relative)
        {
            if (WebAttack == null)
                return;

            EnsurePreviewVisual();

            ComputeAim(relative, out float azimuthDegrees, out float launchSpeed);
            Vector3 origin = WebAttack.ComputeLaunchPose(azimuthDegrees, out Vector3 direction);

            SimulateTrajectory(origin, direction * launchSpeed, out Vector3 endPoint, out Vector3 endNormal);
            DrawTrajectoryLine();
            PositionCross(endPoint, endNormal);

            SetPreviewVisible(true);
        }

        private void HidePreview()
        {
            SetPreviewVisible(false);
        }

        private void SetPreviewVisible(bool visible)
        {
            if (previewRoot != null)
                previewRoot.gameObject.SetActive(visible);
        }

        private void EnsurePreviewVisual()
        {
            if (previewRoot != null)
                return;

            GameObject rootObject = new("RingFingerTrajectoryPreview");
            previewRoot = rootObject.transform;
            previewRoot.SetParent(transform, false);

            trajectoryMaterial = SpiderWebMaterialFactory.CreateDottedLineMaterial(previewColor);
            trajectoryLine = CreateLine("TrajectoryDots", trajectoryMaterial, lineWidth);
            trajectoryLine.textureMode = LineTextureMode.Tile;

            Material crossMaterial = SpiderWebMaterialFactory.CreateDottedLineMaterial(previewColor);
            crossStrokeA = CreateLine("LandingCrossA", crossMaterial, crossLineWidth);
            crossStrokeB = CreateLine("LandingCrossB", crossMaterial, crossLineWidth);
            crossStrokeA.textureMode = LineTextureMode.Stretch;
            crossStrokeB.textureMode = LineTextureMode.Stretch;
        }

        private LineRenderer CreateLine(string lineName, Material material, float width)
        {
            GameObject lineObject = new(lineName);
            lineObject.transform.SetParent(previewRoot, false);

            LineRenderer line = lineObject.AddComponent<LineRenderer>();
            line.sharedMaterial = material;
            line.startColor = Color.white;
            line.endColor = Color.white;
            line.startWidth = width;
            line.endWidth = width;
            line.widthMultiplier = 1f;
            line.numCapVertices = 4;
            line.numCornerVertices = 4;
            line.alignment = LineAlignment.View;
            line.useWorldSpace = true;
            line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            line.receiveShadows = false;
            line.allowOcclusionWhenDynamic = false;
            line.generateLightingData = false;
            line.loop = false;
            return line;
        }

        /// <summary>
        /// Numerically integrates the exact same ballistic arc <see cref="SpiderWebShot"/> flies
        /// (same gravity/max range constants, see <see cref="SpiderWebShot.DefaultGravity"/>), stopping
        /// at the first raycast hit along the path or once it exceeds the shot's max range/this
        /// preview's simulation time cap, whichever comes first.
        /// </summary>
        private void SimulateTrajectory(Vector3 origin, Vector3 initialVelocity, out Vector3 endPoint, out Vector3 endNormal)
        {
            trajectoryPointsScratch.Clear();
            trajectoryPointsScratch.Add(origin);

            Vector3 position = origin;
            Vector3 velocity = initialVelocity;
            endPoint = origin;
            endNormal = Vector3.up;

            int steps = Mathf.Max(1, Mathf.CeilToInt(maxSimulationSeconds / simulationTimeStep));
            for (int i = 0; i < steps; i++)
            {
                Vector3 previous = position;
                velocity += Vector3.down * SpiderWebShot.DefaultGravity * simulationTimeStep;
                position += velocity * simulationTimeStep;

                Vector3 segment = position - previous;
                float segmentLength = segment.magnitude;
                if (segmentLength > 0.0001f &&
                    Physics.Raycast(previous, segment.normalized, out RaycastHit hit, segmentLength, hitMask, QueryTriggerInteraction.Collide))
                {
                    trajectoryPointsScratch.Add(hit.point);
                    endPoint = hit.point;
                    endNormal = hit.normal.sqrMagnitude > 0.0001f ? hit.normal.normalized : Vector3.up;
                    return;
                }

                trajectoryPointsScratch.Add(position);
                endPoint = position;

                if (Vector3.Distance(origin, position) >= SpiderWebShot.DefaultMaxShotRange)
                    return;
            }
        }

        private void DrawTrajectoryLine()
        {
            trajectoryLine.positionCount = trajectoryPointsScratch.Count;
            trajectoryLine.SetPositions(trajectoryPointsScratch.ToArray());

            float length = 0f;
            for (int i = 1; i < trajectoryPointsScratch.Count; i++)
                length += Vector3.Distance(trajectoryPointsScratch[i - 1], trajectoryPointsScratch[i]);

            float tiles = Mathf.Max(1f, length / Mathf.Max(0.01f, dashTileLength));
            trajectoryMaterial.mainTextureScale = new Vector2(tiles, 1f);
        }

        private void PositionCross(Vector3 point, Vector3 normal)
        {
            // Two perpendicular tangent axes on the landing surface so the cross lies flat against
            // it (rather than always facing a fixed world axis), lifted slightly to avoid z-fighting.
            Vector3 reference = Mathf.Abs(Vector3.Dot(normal, Vector3.up)) > 0.95f ? Vector3.forward : Vector3.up;
            Vector3 tangentA = Vector3.Cross(normal, reference).normalized;
            Vector3 tangentB = Vector3.Cross(normal, tangentA).normalized;
            Vector3 offset = point + normal * 0.01f;

            crossStrokeA.positionCount = 2;
            crossStrokeA.SetPosition(0, offset - tangentA * crossSize + tangentB * crossSize);
            crossStrokeA.SetPosition(1, offset + tangentA * crossSize - tangentB * crossSize);

            crossStrokeB.positionCount = 2;
            crossStrokeB.SetPosition(0, offset + tangentA * crossSize + tangentB * crossSize);
            crossStrokeB.SetPosition(1, offset - tangentA * crossSize - tangentB * crossSize);
        }
    }
}
