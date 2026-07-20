using System.Collections.Generic;
using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// A single web shot: dragline silk extends forward until it hits prey, then an orb-web net blooms on the target.
    /// Uses view-aligned line renderers for smooth, anti-aliased silk strands.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class SpiderWebShot : MonoBehaviour
    {
        private enum Phase
        {
            Extending,
            Expanding,
            Holding,
            Finished
        }

        [Header("Line Phase")]
        [SerializeField, Min(0.5f)] private float maxShotRange = 5f;
        [SerializeField, Min(0.1f)] private float lineExtendSpeed = 4f;
        [SerializeField, Min(0.001f)] private float draglineWidth = 0.012f;
        [SerializeField, Range(0f, 0.2f)] private float draglineSag = 0.03f;

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

        [Header("Motion")]
        [SerializeField, Range(0f, 0.03f)] private float windSwayAmplitude = 0.008f;
        [SerializeField, Min(0f)] private float windSwayFrequency = 2.4f;

        [Header("Droplets")]
        [SerializeField, Range(0, 80)] private int dropletCount = 28;
        [SerializeField, Range(0.002f, 0.02f)] private float dropletRadius = 0.006f;

        [Header("Lifetime")]
        [SerializeField, Min(0f)] private float holdDuration = 2.5f;

        private Phase phase = Phase.Extending;
        private Vector3 origin;
        private Vector3 direction = Vector3.forward;
        private float currentLineLength;
        private float targetLineLength;
        private float activeNetRadius;
        private float currentNetRadius;
        private float phaseTimer;
        private int randomSeed;
        private CirclePrey targetPrey;
        private bool netBuilt;
        private float netWidthScale = 1f;

        private LineRenderer draglineRenderer;
        private Transform netRoot;
        private readonly List<LineRenderer> strandRenderers = new();
        private IReadOnlyList<WebStrand> fullWebStrands;
        private readonly List<Vector3[]> baseStrandPoints = new();
        private readonly List<Vector3[]> workingStrandPoints = new();
        private readonly List<DropletAnchor> dropletAnchors = new();
        private CirclePrey capturedPrey;
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

        public void Launch(Vector3 launchOrigin, Vector3 launchDirection, CirclePrey preyTarget = null)
        {
            origin = launchOrigin;
            direction = launchDirection.sqrMagnitude > 0.0001f
                ? launchDirection.normalized
                : Vector3.forward;

            targetPrey = preyTarget;
            randomSeed = Random.Range(0, int.MaxValue);
            lineMaterial = SpiderWebMaterialFactory.GetLineMaterial();
            transform.position = origin;
            transform.rotation = Quaternion.LookRotation(direction, Vector3.up);

            targetLineLength = ResolveTargetLineLength();
            activeNetRadius = ResolveNetRadius();

            CreateDragline();
            UpdateDraglineLine();
        }

        private void Update()
        {
            switch (phase)
            {
                case Phase.Extending:
                    UpdateExtending();
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

        private float ResolveTargetLineLength()
        {
            if (targetPrey == null || targetPrey.IsCaptured)
                return maxShotRange;

            Vector3 toPrey = targetPrey.transform.position - origin;
            float alongRay = Vector3.Dot(toPrey, direction);
            return Mathf.Clamp(alongRay, 0.1f, maxShotRange);
        }

        private float ResolveNetRadius()
        {
            if (targetPrey != null && !targetPrey.IsCaptured)
                return Mathf.Max(0.15f, targetPrey.CaptureExtent * netRadiusPadding);

            return defaultNetRadius;
        }

        private void UpdateExtending()
        {
            currentLineLength = Mathf.MoveTowards(
                currentLineLength,
                targetLineLength,
                lineExtendSpeed * Time.deltaTime);

            UpdateDraglineLine();

            if (HasReachedPrey() || Mathf.Approximately(currentLineLength, targetLineLength))
                BeginNetExpansion();
        }

        private bool HasReachedPrey()
        {
            if (targetPrey == null || targetPrey.IsCaptured)
                return false;

            Vector3 tip = origin + direction * currentLineLength;
            return Vector3.Distance(tip, targetPrey.transform.position) <= targetPrey.CaptureExtent;
        }

        private void BeginNetExpansion()
        {
            if (targetPrey != null && !targetPrey.IsCaptured)
            {
                Vector3 preyPosition = targetPrey.transform.position;
                currentLineLength = Mathf.Max(0.1f, Vector3.Dot(preyPosition - origin, direction));
                activeNetRadius = ResolveNetRadius();
            }

            UpdateDraglineLine();
            BuildNetAtTip();
            phase = Phase.Expanding;
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
            TryCapturePrey();

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

        private void CreateDragline()
        {
            GameObject draglineObject = new("Dragline");
            draglineObject.transform.SetParent(transform, false);
            draglineRenderer = draglineObject.AddComponent<LineRenderer>();
            ConfigureLineRenderer(draglineRenderer, draglineWidth, new Color(0.94f, 0.97f, 1f, 0.95f));
            draglineRenderer.useWorldSpace = false;
        }

        private void CreateNetLines()
        {
            GameObject netObject = new("OrbWeb");
            netRoot = netObject.transform;
            netRoot.SetParent(transform, false);

            for (int i = 0; i < fullWebStrands.Count; i++)
            {
                WebStrand strand = fullWebStrands[i];
                GameObject strandObject = new($"Strand_{strand.Type}_{i}");
                strandObject.transform.SetParent(netRoot, false);

                LineRenderer line = strandObject.AddComponent<LineRenderer>();
                float width = StrandWidth(strand.Type) * netWidthScale;
                Color color = StrandColor(strand.Type);
                ConfigureLineRenderer(line, width, color);
                line.useWorldSpace = false;
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

        private void UpdateDraglineLine()
        {
            if (draglineRenderer == null)
                return;

            const int samples = 24;
            draglineRenderer.positionCount = samples + 1;
            Vector3 end = direction * currentLineLength;

            for (int i = 0; i <= samples; i++)
            {
                float t = i / (float)samples;
                Vector3 point = end * t;
                point.y -= draglineSag * 4f * t * (1f - t);
                draglineRenderer.SetPosition(i, point);
            }
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
            for (int i = 0; i < strandRenderers.Count; i++)
            {
                LineRenderer line = strandRenderers[i];
                Vector3[] points = DensifyPolyline(workingStrandPoints[i], 0.03f);
                line.positionCount = points.Length;
                line.SetPositions(points);

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
            for (int i = 0; i < dropletAnchors.Count; i++)
            {
                DropletAnchor anchor = dropletAnchors[i];
                if (anchor.Transform == null)
                    continue;

                Vector3[] strand = workingStrandPoints[anchor.StrandIndex];
                int pointIndex = Mathf.Clamp(anchor.PointIndex, 0, strand.Length - 1);
                anchor.Transform.localPosition = strand[pointIndex];
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

            if (targetPrey != null && !targetPrey.IsCaptured)
                netRoot.position = targetPrey.transform.position;
            else
                netRoot.localPosition = direction * currentLineLength;

            netRoot.localRotation = Quaternion.identity;
        }

        private void SetNetVisible(bool visible)
        {
            if (netRoot != null)
                netRoot.gameObject.SetActive(visible);
        }

        private void TryCapturePrey()
        {
            if (capturedPrey != null || netRoot == null)
                return;

            Vector3 netCenter = netRoot.position;
            Collider[] overlaps = Physics.OverlapSphere(netCenter, currentNetRadius);

            for (int i = 0; i < overlaps.Length; i++)
            {
                CirclePrey prey = overlaps[i].GetComponent<CirclePrey>();
                if (prey == null || prey.IsCaptured)
                    continue;

                float distance = Vector3.Distance(netCenter, prey.transform.position);
                if (distance <= currentNetRadius + prey.CaptureExtent * 0.15f)
                {
                    capturedPrey = prey;
                    prey.Capture(netRoot);
                    break;
                }
            }
        }
    }
}
