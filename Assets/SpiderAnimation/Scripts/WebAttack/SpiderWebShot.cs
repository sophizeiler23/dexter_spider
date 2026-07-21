using System.Collections.Generic;
using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// A single web shot: a compact orb-web projectile flies forward in a ballistic arc, then blooms on impact.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class SpiderWebShot : MonoBehaviour
    {
        private enum Phase
        {
            Flying,
            Expanding,
            Holding,
            Finished
        }

        [Header("Projectile")]
        [SerializeField, Min(0.5f)] private float maxShotRange = 8f;
        [SerializeField, Min(1f)] private float launchSpeed = 12f;
        [SerializeField, Min(0f)] private float gravity = 9.81f;
        [SerializeField, Min(0.01f)] private float projectileHitRadius = 0.15f;
        [SerializeField, Min(0.1f)] private float projectileWebRadius = 0.38f;
        [SerializeField, Min(4)] private int trajectoryPointCount = 20;

        [Header("Flight Trail")]
        [SerializeField, Min(0.001f)] private float trailWidth = 0.018f;
        [SerializeField, Range(0f, 0.35f)] private float trailSag = 0.06f;

        [Header("Net Phase")]
        [SerializeField, Min(0.1f)] private float defaultNetRadius = 0.6f;
        [SerializeField, Range(1f, 2.5f)] private float netRadiusPadding = 1.2f;
        [SerializeField, Range(0.05f, 0.5f)] private float netExpandDuration = 0.18f;
        [SerializeField, Range(6, 28)] private int spokeCount = 18;
        [SerializeField, Range(3, 12)] private int ringCount = 8;
        [SerializeField, Range(0.2f, 1.2f)] private float ringSpacingPower = 0.55f;
        [SerializeField, Range(0f, 0.12f)] private float ringJitter = 0.035f;
        [SerializeField, Range(0f, 0.35f)] private float spiralBulge = 0.14f;
        [SerializeField, Range(4, 16)] private int curveSegments = 10;
        [SerializeField, Range(0, 4)] private int auxiliarySpiralTurns = 2;

        [Header("Silk Width")]
        [SerializeField, Min(0.001f)] private float radialThreadWidth = 0.011f;
        [SerializeField, Min(0.001f)] private float captureThreadWidth = 0.0065f;
        [SerializeField, Min(0.001f)] private float frameThreadWidth = 0.01f;
        [SerializeField, Min(0.001f)] private float auxiliaryThreadWidth = 0.004f;
        [SerializeField, Min(0.001f)] private float projectileThreadWidth = 0.014f;

        [Header("Wind")]
        [SerializeField, Range(0f, 0.08f)] private float windSwayAmplitude = 0.025f;
        [SerializeField, Min(0f)] private float windSwayFrequency = 2.4f;

        [Header("Droplets")]
        [SerializeField, Range(0, 80)] private int dropletCount = 28;
        [SerializeField, Range(0.002f, 0.02f)] private float dropletRadius = 0.006f;

        [Header("Lifetime")]
        [SerializeField, Min(0f)] private float holdDuration = 2.5f;

        private Phase phase = Phase.Flying;
        private Vector3 origin;
        private Vector3 launchDirection = Vector3.forward;
        private Vector3 tipWorldPosition;
        private Vector3 previousTipWorldPosition;
        private Vector3 tipVelocity;
        private float activeNetRadius;
        private float currentNetRadius;
        private float phaseTimer;
        private int randomSeed;
        private CirclePrey hitPrey;
        private bool netBuilt;
        private float netWidthScale = 1f;

        private Transform projectileRoot;
        private LineRenderer trailRenderer;
        private readonly List<Vector3> trajectoryPoints = new();
        private readonly List<LineRenderer> projectileStrandRenderers = new();
        private readonly List<Vector3[]> projectileStrandPoints = new();
        private IReadOnlyList<WebStrand> projectileStrands;

        private Transform netRoot;
        private readonly List<LineRenderer> strandRenderers = new();
        private IReadOnlyList<WebStrand> fullWebStrands;
        private readonly List<Vector3[]> baseStrandPoints = new();
        private readonly List<Vector3[]> workingStrandPoints = new();
        private readonly List<DropletAnchor> dropletAnchors = new();
        private Material lineMaterial;

        private readonly struct DropletAnchor
        {
            public DropletAnchor(Transform transform, int strandIndex, int pointIndex, float scale)
            {
                Transform = transform;
                StrandIndex = strandIndex;
                PointIndex = pointIndex;
                Scale = scale;
            }

            public Transform Transform { get; }
            public int StrandIndex { get; }
            public int PointIndex { get; }
            public float Scale { get; }
        }

        public void Launch(Vector3 launchOrigin, Vector3 launchDirection)
        {
            origin = launchOrigin;
            Vector3 normalizedDirection = launchDirection.sqrMagnitude > 0.0001f
                ? launchDirection.normalized
                : Vector3.forward;

            this.launchDirection = normalizedDirection;
            randomSeed = Random.Range(0, int.MaxValue);
            lineMaterial = SpiderWebMaterialFactory.GetLineMaterial();

            tipWorldPosition = origin;
            previousTipWorldPosition = origin;
            tipVelocity = normalizedDirection * launchSpeed;

            trajectoryPoints.Clear();
            trajectoryPoints.Add(origin);

            CreateProjectileVisual();
            UpdateProjectileVisual(applyWind: false);
        }

        private void Update()
        {
            switch (phase)
            {
                case Phase.Flying:
                    UpdateFlying();
                    break;
                case Phase.Expanding:
                    UpdateExpanding();
                    break;
                case Phase.Holding:
                    UpdateHolding();
                    break;
                case Phase.Finished:
                    Destroy(gameObject);
                    break;
            }
        }

        private void UpdateFlying()
        {
            previousTipWorldPosition = tipWorldPosition;

            tipVelocity += Vector3.down * gravity * Time.deltaTime;
            tipWorldPosition += tipVelocity * Time.deltaTime;

            RecordTrajectoryPoint(tipWorldPosition);
            UpdateProjectileVisual(applyWind: true);

            if (TryProjectileHit(out CirclePrey prey, out Vector3 hitPoint))
            {
                hitPrey = prey;
                tipWorldPosition = hitPoint;
                RecordTrajectoryPoint(hitPoint);
                prey.KnockDownFromWeb(tipVelocity, hitPoint);
                BeginNetExpansion();
                return;
            }

            if (Vector3.Distance(origin, tipWorldPosition) >= maxShotRange)
                BeginNetExpansion();
        }

        private void RecordTrajectoryPoint(Vector3 point)
        {
            if (trajectoryPoints.Count == 0 ||
                Vector3.Distance(trajectoryPoints[^1], point) > 0.02f)
                trajectoryPoints.Add(point);

            while (trajectoryPoints.Count > trajectoryPointCount)
                trajectoryPoints.RemoveAt(0);
        }

        private bool TryProjectileHit(
            out CirclePrey prey,
            out Vector3 hitPoint)
        {
            prey = null;
            hitPoint = tipWorldPosition;

            Vector3 displacement = tipWorldPosition - previousTipWorldPosition;
            float distance = displacement.magnitude;
            if (distance <= 0.0001f)
                return false;

            Vector3 direction = displacement / distance;
            RaycastHit[] hits = Physics.SphereCastAll(
                previousTipWorldPosition,
                projectileHitRadius,
                direction,
                distance,
                ~0,
                QueryTriggerInteraction.Collide);

            float closestDistance = float.MaxValue;
            for (int i = 0; i < hits.Length; i++)
            {
                RaycastHit hit = hits[i];
                CirclePrey candidate = hit.collider.GetComponent<CirclePrey>();
                if (candidate == null)
                    candidate = hit.collider.GetComponentInParent<CirclePrey>();
                if (candidate == null || candidate.IsCaptured)
                    continue;

                if (hit.distance >= closestDistance)
                    continue;

                closestDistance = hit.distance;
                prey = candidate;
                hitPoint = hit.point;
            }

            return prey != null;
        }

        private void BeginNetExpansion()
        {
            DestroyProjectileVisual();
            activeNetRadius = ResolveNetRadius();
            BuildNetAtTip();
            phase = Phase.Expanding;
        }

        private float ResolveNetRadius()
        {
            if (hitPrey != null)
                return Mathf.Max(0.15f, hitPrey.CaptureExtent * netRadiusPadding);

            return defaultNetRadius;
        }

        private void UpdateExpanding()
        {
            float expandSpeed = activeNetRadius / Mathf.Max(0.05f, netExpandDuration);
            currentNetRadius = Mathf.MoveTowards(
                currentNetRadius,
                activeNetRadius,
                expandSpeed * Time.deltaTime);

            float expansion01 = activeNetRadius > 0.0001f
                ? currentNetRadius / activeNetRadius
                : 1f;

            UpdateWorkingStrands(expansion01, applyWind: true);
            UpdateNetLines(expansion01);

            if (Mathf.Approximately(currentNetRadius, activeNetRadius))
            {
                phase = Phase.Holding;
                phaseTimer = 0f;
            }
        }

        private void UpdateHolding()
        {
            phaseTimer += Time.deltaTime;
            UpdateWorkingStrands(1f, applyWind: true);
            UpdateNetLines(1f);

            if (phaseTimer >= holdDuration)
                phase = Phase.Finished;
        }

        private void CreateProjectileVisual()
        {
            GameObject projectileObject = new("WebProjectile");
            projectileRoot = projectileObject.transform;
            projectileRoot.SetParent(transform, true);
            projectileRoot.position = tipWorldPosition;

            OrbWebSettings settings = new(
                projectileWebRadius,
                10,
                4,
                0.55f,
                0.02f,
                0.08f * projectileWebRadius,
                6,
                1,
                0.04f,
                0.03f,
                randomSeed);

            projectileStrands = ProceduralOrbWeb.Generate(settings);
            projectileStrandPoints.Clear();
            projectileStrandRenderers.Clear();

            for (int i = 0; i < projectileStrands.Count; i++)
            {
                WebStrand strand = projectileStrands[i];
                GameObject strandObject = new($"ProjectileStrand_{strand.Type}_{i}");
                strandObject.transform.SetParent(projectileRoot, false);

                LineRenderer line = strandObject.AddComponent<LineRenderer>();
                float width = strand.Type == WebStrandType.Radial
                    ? projectileThreadWidth
                    : captureThreadWidth * 1.2f;
                ConfigureLineRenderer(line, width, StrandColor(strand.Type));
                line.useWorldSpace = true;
                projectileStrandRenderers.Add(line);
                projectileStrandPoints.Add((Vector3[])strand.Points.Clone());
            }

            GameObject trailObject = new("FlightTrail");
            trailObject.transform.SetParent(projectileRoot, false);
            trailRenderer = trailObject.AddComponent<LineRenderer>();
            ConfigureLineRenderer(
                trailRenderer,
                trailWidth,
                new Color(0.94f, 0.97f, 1f, 0.85f));
            trailRenderer.useWorldSpace = true;
            trailRenderer.widthCurve = BuildTrailWidthCurve();
        }

        private static AnimationCurve BuildTrailWidthCurve()
        {
            return new AnimationCurve(
                new Keyframe(0f, 0.15f),
                new Keyframe(0.25f, 0.55f),
                new Keyframe(1f, 1f));
        }

        private void UpdateProjectileVisual(bool applyWind)
        {
            if (projectileRoot == null)
                return;

            projectileRoot.position = tipWorldPosition;

            if (tipVelocity.sqrMagnitude > 0.01f)
            {
                Vector3 up = Mathf.Abs(Vector3.Dot(tipVelocity.normalized, Vector3.up)) > 0.95f
                    ? Vector3.forward
                    : Vector3.up;
                projectileRoot.rotation = Quaternion.LookRotation(tipVelocity.normalized, up);
            }

            UpdateProjectileStrands(applyWind);
            UpdateFlightTrail(applyWind);
        }

        private void UpdateProjectileStrands(bool applyWind)
        {
            float time = Time.time * windSwayFrequency;
            Vector3 center = tipWorldPosition;

            for (int i = 0; i < projectileStrandRenderers.Count; i++)
            {
                Vector3[] localPoints = projectileStrandPoints[i];
                Vector3[] worldPoints = new Vector3[localPoints.Length];

                for (int pointIndex = 0; pointIndex < localPoints.Length; pointIndex++)
                {
                    Vector3 point = center + localPoints[pointIndex];

                    if (applyWind && windSwayAmplitude > 0f)
                    {
                        float phase = time + i * 0.31f + pointIndex * 0.17f;
                        point += projectileRoot.right * (Mathf.Sin(phase) * windSwayAmplitude);
                        point += projectileRoot.up * (Mathf.Cos(phase * 0.85f) * windSwayAmplitude * 0.35f);
                    }

                    worldPoints[pointIndex] = point;
                }

                LineRenderer line = projectileStrandRenderers[i];
                line.positionCount = worldPoints.Length;
                line.SetPositions(worldPoints);
            }
        }

        private void UpdateFlightTrail(bool applyWind)
        {
            if (trailRenderer == null || trajectoryPoints.Count < 2)
            {
                if (trailRenderer != null)
                    trailRenderer.positionCount = 0;
                return;
            }

            int count = trajectoryPoints.Count;
            trailRenderer.positionCount = count;

            for (int i = 0; i < count; i++)
            {
                float t = i / (float)(count - 1);
                Vector3 point = trajectoryPoints[i];

                float sag = trailSag * 4f * t * (1f - t);
                point.y -= sag;

                if (applyWind && windSwayAmplitude > 0f)
                {
                    float envelope = 4f * t * (1f - t);
                    float phase = Time.time * windSwayFrequency + t * 4f;
                    point += Vector3.right * (Mathf.Sin(phase) * windSwayAmplitude * envelope * 2f);
                }

                trailRenderer.SetPosition(i, point);
            }
        }

        private void DestroyProjectileVisual()
        {
            if (projectileRoot != null)
            {
                Destroy(projectileRoot.gameObject);
                projectileRoot = null;
            }

            trailRenderer = null;
            projectileStrandRenderers.Clear();
            projectileStrandPoints.Clear();
        }

        private void BuildNetAtTip()
        {
            if (netBuilt)
                return;

            netWidthScale = Mathf.Clamp(activeNetRadius / defaultNetRadius, 0.35f, 1.5f);
            BuildFullWebData();
            CreateNetLines();
            CreateDroplets();
            PositionNetAtTip();
            SetNetVisible(true);
            netBuilt = true;
        }

        private void BuildFullWebData()
        {
            OrbWebSettings settings = new(
                activeNetRadius,
                spokeCount,
                ringCount,
                ringSpacingPower,
                ringJitter * netWidthScale,
                spiralBulge * activeNetRadius,
                curveSegments,
                auxiliarySpiralTurns,
                0.08f * netWidthScale,
                0.06f * netWidthScale,
                randomSeed);

            fullWebStrands = ProceduralOrbWeb.Generate(settings);
            baseStrandPoints.Clear();
            workingStrandPoints.Clear();

            for (int i = 0; i < fullWebStrands.Count; i++)
            {
                Vector3[] points = (Vector3[])fullWebStrands[i].Points.Clone();
                baseStrandPoints.Add(points);
                workingStrandPoints.Add((Vector3[])points.Clone());
            }
        }

        private void CreateNetLines()
        {
            GameObject netObject = new("OrbWeb");
            netRoot = netObject.transform;
            netRoot.SetParent(transform, true);

            for (int i = 0; i < fullWebStrands.Count; i++)
            {
                WebStrand strand = fullWebStrands[i];
                GameObject strandObject = new($"Strand_{strand.Type}_{i}");
                strandObject.transform.SetParent(netRoot, false);

                LineRenderer line = strandObject.AddComponent<LineRenderer>();
                float width = StrandWidth(strand.Type) * netWidthScale;
                Color color = StrandColor(strand.Type);
                ConfigureLineRenderer(line, width, color);
                line.useWorldSpace = true;
                strandRenderers.Add(line);
            }
        }

        private void CreateDroplets()
        {
            float radiusScale = netWidthScale;
            int scaledDropletCount = Mathf.RoundToInt(dropletCount * radiusScale * radiusScale);
            scaledDropletCount = Mathf.Clamp(scaledDropletCount, 0, dropletCount);

            int created = 0;
            int safety = 0;

            while (created < scaledDropletCount && safety < scaledDropletCount * 8)
            {
                safety++;
                int strandIndex = Random.Range(0, fullWebStrands.Count);
                WebStrand strand = fullWebStrands[strandIndex];
                if (strand.Type != WebStrandType.CaptureSpiral && strand.Type != WebStrandType.AuxiliarySpiral)
                    continue;

                if (strand.Points.Length < 2)
                    continue;

                int pointIndex = Random.Range(0, strand.Points.Length);
                float scale = Random.Range(0.65f, 1.35f);

                GameObject droplet = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                droplet.name = $"Droplet_{created}";
                droplet.transform.SetParent(netRoot, false);
                droplet.transform.localScale = Vector3.one * dropletRadius * 2f * scale * radiusScale;

                Collider collider = droplet.GetComponent<Collider>();
                if (collider != null)
                    Destroy(collider);

                MeshRenderer renderer = droplet.GetComponent<MeshRenderer>();
                renderer.sharedMaterial = lineMaterial;

                dropletAnchors.Add(new DropletAnchor(droplet.transform, strandIndex, pointIndex, scale));
                created++;
            }
        }

        private void ConfigureLineRenderer(LineRenderer line, float width, Color color)
        {
            line.sharedMaterial = lineMaterial;
            line.startColor = color;
            line.endColor = color;
            line.startWidth = width;
            line.endWidth = width;
            line.widthMultiplier = 1f;
            line.numCapVertices = 10;
            line.numCornerVertices = 10;
            line.alignment = LineAlignment.View;
            line.textureMode = LineTextureMode.Tile;
            line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            line.receiveShadows = false;
            line.allowOcclusionWhenDynamic = false;
            line.generateLightingData = false;
            line.loop = false;
        }

        private void UpdateWorkingStrands(float expansion01, bool applyWind)
        {
            float expansion = Mathf.Clamp01(expansion01);
            float sway = applyWind ? windSwayAmplitude * expansion : 0f;
            float time = Time.time * windSwayFrequency;

            for (int strandIndex = 0; strandIndex < baseStrandPoints.Count; strandIndex++)
            {
                Vector3[] source = baseStrandPoints[strandIndex];
                Vector3[] target = workingStrandPoints[strandIndex];

                for (int pointIndex = 0; pointIndex < source.Length; pointIndex++)
                {
                    Vector3 point = source[pointIndex] * expansion;

                    if (sway > 0f)
                    {
                        float phaseOffset = strandIndex * 0.37f + pointIndex * 0.19f;
                        float lateral = Mathf.Sin(time + phaseOffset) * sway;
                        float vertical = Mathf.Cos(time * 0.8f + phaseOffset) * sway * 0.35f;
                        point += Vector3.right * lateral + Vector3.up * vertical;
                    }

                    target[pointIndex] = point;
                }
            }

            UpdateDropletPositions();
        }

        private void UpdateNetLines(float expansion01)
        {
            if (netRoot == null)
                return;

            Vector3 netCenter = netRoot.position;
            for (int i = 0; i < strandRenderers.Count; i++)
            {
                LineRenderer line = strandRenderers[i];
                Vector3[] localPoints = DensifyPolyline(workingStrandPoints[i], 0.03f);
                Vector3[] worldPoints = new Vector3[localPoints.Length];
                for (int pointIndex = 0; pointIndex < localPoints.Length; pointIndex++)
                    worldPoints[pointIndex] = netCenter + localPoints[pointIndex];

                line.positionCount = worldPoints.Length;
                line.SetPositions(worldPoints);

                float width = StrandWidth(fullWebStrands[i].Type) * netWidthScale * Mathf.Lerp(0.2f, 1f, expansion01);
                line.startWidth = width;
                line.endWidth = width;
            }
        }

        private static Vector3[] DensifyPolyline(IReadOnlyList<Vector3> points, float maxSegmentLength)
        {
            if (points.Count < 2 || maxSegmentLength <= 0f)
            {
                Vector3[] copy = new Vector3[points.Count];
                for (int i = 0; i < points.Count; i++)
                    copy[i] = points[i];
                return copy;
            }

            List<Vector3> densified = new() { points[0] };

            for (int i = 0; i < points.Count - 1; i++)
            {
                Vector3 start = points[i];
                Vector3 end = points[i + 1];
                float length = Vector3.Distance(start, end);
                int divisions = Mathf.Max(1, Mathf.CeilToInt(length / maxSegmentLength));

                for (int division = 1; division <= divisions; division++)
                {
                    float t = division / (float)divisions;
                    densified.Add(Vector3.Lerp(start, end, t));
                }
            }

            return densified.ToArray();
        }

        private void UpdateDropletPositions()
        {
            if (netRoot == null)
                return;

            Vector3 netCenter = netRoot.position;
            for (int i = 0; i < dropletAnchors.Count; i++)
            {
                DropletAnchor anchor = dropletAnchors[i];
                if (anchor.Transform == null)
                    continue;

                Vector3[] strand = workingStrandPoints[anchor.StrandIndex];
                int pointIndex = Mathf.Clamp(anchor.PointIndex, 0, strand.Length - 1);
                anchor.Transform.position = netCenter + strand[pointIndex];
            }
        }

        private float StrandWidth(WebStrandType type)
        {
            return type switch
            {
                WebStrandType.Radial => radialThreadWidth,
                WebStrandType.Frame => frameThreadWidth,
                WebStrandType.AuxiliarySpiral => auxiliaryThreadWidth,
                _ => captureThreadWidth
            };
        }

        private static Color StrandColor(WebStrandType type)
        {
            return type switch
            {
                WebStrandType.Radial => new Color(0.94f, 0.97f, 1f, 0.92f),
                WebStrandType.Frame => new Color(0.92f, 0.96f, 1f, 0.88f),
                WebStrandType.AuxiliarySpiral => new Color(0.9f, 0.94f, 1f, 0.45f),
                _ => new Color(0.96f, 0.98f, 1f, 0.82f)
            };
        }

        private void PositionNetAtTip()
        {
            if (netRoot == null)
                return;

            netRoot.position = tipWorldPosition;
            netRoot.rotation = Quaternion.identity;
        }

        private void SetNetVisible(bool visible)
        {
            if (netRoot != null)
                netRoot.gameObject.SetActive(visible);
        }
    }
}
