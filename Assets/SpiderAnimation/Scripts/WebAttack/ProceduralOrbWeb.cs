using System;
using System.Collections.Generic;
using UnityEngine;

namespace Dexter.Spider
{
    public enum WebStrandType
    {
        Radial,
        Frame,
        AuxiliarySpiral,
        CaptureSpiral
    }

    public readonly struct WebStrand
    {
        public WebStrand(WebStrandType type, Vector3[] points)
        {
            Type = type;
            Points = points;
        }

        public WebStrandType Type { get; }
        public Vector3[] Points { get; }
    }

    public readonly struct OrbWebSettings
    {
        public OrbWebSettings(
            float radius,
            int spokeCount,
            int ringCount,
            float ringSpacingPower,
            float ringJitter,
            float spiralBulge,
            int curveSegments,
            int auxiliarySpiralTurns,
            float hubRadiusFraction,
            float frameOvershoot,
            int randomSeed)
        {
            Radius = radius;
            SpokeCount = spokeCount;
            RingCount = ringCount;
            RingSpacingPower = ringSpacingPower;
            RingJitter = ringJitter;
            SpiralBulge = spiralBulge;
            CurveSegments = curveSegments;
            AuxiliarySpiralTurns = auxiliarySpiralTurns;
            HubRadiusFraction = hubRadiusFraction;
            FrameOvershoot = frameOvershoot;
            RandomSeed = randomSeed;
        }

        public float Radius { get; }
        public int SpokeCount { get; }
        public int RingCount { get; }
        public float RingSpacingPower { get; }
        public float RingJitter { get; }
        public float SpiralBulge { get; }
        public int CurveSegments { get; }
        public int AuxiliarySpiralTurns { get; }
        public float HubRadiusFraction { get; }
        public float FrameOvershoot { get; }
        public int RandomSeed { get; }
    }

    /// <summary>
    /// Builds an orb-web style net from flattened sphere topology:
    /// radial spokes, outer frame, auxiliary scaffold spiral, and inward-bulging capture spirals.
    /// </summary>
    public static class ProceduralOrbWeb
    {
        public static IReadOnlyList<WebStrand> Generate(OrbWebSettings settings)
        {
            int spokeCount = Mathf.Max(6, settings.SpokeCount);
            int ringCount = Mathf.Max(2, settings.RingCount);
            float radius = Mathf.Max(0.1f, settings.Radius);
            float hubRadius = radius * Mathf.Clamp(settings.HubRadiusFraction, 0.02f, 0.2f);
            float frameRadius = radius * (1f + Mathf.Max(0f, settings.FrameOvershoot));

            System.Random random = new(settings.RandomSeed);
            float[] ringRadii = BuildRingRadii(ringCount, radius, settings.RingSpacingPower, settings.RingJitter, random);
            List<WebStrand> strands = new();

            AddRadialStrands(strands, spokeCount, hubRadius, frameRadius);
            AddFrameStrand(strands, spokeCount, frameRadius);
            AddAuxiliarySpiral(strands, spokeCount, ringRadii, settings.AuxiliarySpiralTurns);
            AddCaptureSpiralStrands(strands, spokeCount, ringRadii, settings.SpiralBulge, settings.CurveSegments);

            return strands;
        }

        private static float[] BuildRingRadii(
            int ringCount,
            float maxRadius,
            float spacingPower,
            float jitter,
            System.Random random)
        {
            float[] ringRadii = new float[ringCount];
            float power = Mathf.Clamp(spacingPower, 0.25f, 1.5f);

            for (int ring = 0; ring < ringCount; ring++)
            {
                float normalized = (ring + 1f) / (ringCount + 1f);
                float radius = maxRadius * Mathf.Pow(normalized, power);
                float jitterAmount = (float)(random.NextDouble() * 2.0 - 1.0) * jitter * maxRadius;
                ringRadii[ring] = Mathf.Clamp(radius + jitterAmount, maxRadius * 0.08f, maxRadius * 0.98f);
            }

            return ringRadii;
        }

        private static void AddRadialStrands(List<WebStrand> strands, int spokeCount, float hubRadius, float frameRadius)
        {
            float step = Mathf.PI * 2f / spokeCount;

            for (int spoke = 0; spoke < spokeCount; spoke++)
            {
                float angle = step * spoke;
                Vector3 hub = Polar(angle, hubRadius);
                Vector3 frame = Polar(angle, frameRadius);
                strands.Add(new WebStrand(WebStrandType.Radial, new[] { hub, frame }));
            }
        }

        private static void AddFrameStrand(List<WebStrand> strands, int spokeCount, float frameRadius)
        {
            Vector3[] framePoints = new Vector3[spokeCount + 1];
            float step = Mathf.PI * 2f / spokeCount;

            for (int spoke = 0; spoke <= spokeCount; spoke++)
            {
                float angle = step * spoke;
                framePoints[spoke] = Polar(angle, frameRadius);
            }

            strands.Add(new WebStrand(WebStrandType.Frame, framePoints));
        }

        private static void AddAuxiliarySpiral(
            List<WebStrand> strands,
            int spokeCount,
            float[] ringRadii,
            int spiralTurns)
        {
            if (spiralTurns <= 0 || ringRadii.Length == 0)
                return;

            int samples = Mathf.Max(spokeCount * spiralTurns, 12);
            Vector3[] spiralPoints = new Vector3[samples + 1];
            float maxRadius = ringRadii[ringRadii.Length - 1];

            for (int sample = 0; sample <= samples; sample++)
            {
                float t = sample / (float)samples;
                float angle = t * spiralTurns * Mathf.PI * 2f;
                float radius = Mathf.Lerp(ringRadii[0], maxRadius, t);
                spiralPoints[sample] = Polar(angle, radius);
            }

            strands.Add(new WebStrand(WebStrandType.AuxiliarySpiral, spiralPoints));
        }

        private static void AddCaptureSpiralStrands(
            List<WebStrand> strands,
            int spokeCount,
            float[] ringRadii,
            float spiralBulge,
            int curveSegments)
        {
            int segments = Mathf.Max(3, curveSegments);
            float step = Mathf.PI * 2f / spokeCount;

            for (int ring = 0; ring < ringRadii.Length; ring++)
            {
                float ringRadius = ringRadii[ring];
                float bulge = spiralBulge * (1f - ring / (float)ringRadii.Length);

                for (int spoke = 0; spoke < spokeCount; spoke++)
                {
                    float angleA = step * spoke;
                    float angleB = step * (spoke + 1);
                    Vector3 start = Polar(angleA, ringRadius);
                    Vector3 end = Polar(angleB, ringRadius);

                    Vector3[] curve = new Vector3[segments + 1];
                    for (int segment = 0; segment <= segments; segment++)
                    {
                        float t = segment / (float)segments;
                        curve[segment] = BulgingArcPoint(start, end, t, bulge);
                    }

                    strands.Add(new WebStrand(WebStrandType.CaptureSpiral, curve));
                }
            }
        }

        private static Vector3 BulgingArcPoint(Vector3 start, Vector3 end, float t, float bulge)
        {
            Vector3 point = Vector3.Lerp(start, end, t);
            if (point.sqrMagnitude < 0.000001f)
                return point;

            Vector3 inward = -point.normalized;
            float arc = Mathf.Sin(t * Mathf.PI);
            return point + inward * (bulge * arc);
        }

        private static Vector3 Polar(float angle, float radius)
        {
            return new Vector3(Mathf.Cos(angle) * radius, Mathf.Sin(angle) * radius, 0f);
        }
    }
}
