using System.Collections.Generic;
using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// Converts procedural web strands into a single tube mesh for thin, glossy silk threads.
    /// </summary>
    public static class SpiderWebMeshBuilder
    {
        public static Mesh Build(
            IReadOnlyList<WebStrand> strands,
            float radialRadius,
            float captureRadius,
            float frameRadius,
            float auxiliaryRadius,
            int tubeSides = 10,
            float maxSegmentLength = 0.04f)
        {
            List<Vector3> vertices = new();
            List<Vector3> normals = new();
            List<Vector2> uvs = new();
            List<Color> colors = new();
            List<int> triangles = new();

            int sides = Mathf.Clamp(tubeSides, 6, 16);

            for (int strandIndex = 0; strandIndex < strands.Count; strandIndex++)
            {
                WebStrand strand = strands[strandIndex];
                if (strand.Points == null || strand.Points.Length < 2)
                    continue;

                float radius = StrandRadius(strand.Type, radialRadius, captureRadius, frameRadius, auxiliaryRadius);
                Color vertexColor = StrandColor(strand.Type);
                Vector3[] resampledPoints = ResamplePolyline(strand.Points, maxSegmentLength);
                AppendTube(
                    resampledPoints,
                    radius,
                    sides,
                    vertexColor,
                    vertices,
                    normals,
                    uvs,
                    colors,
                    triangles);
            }

            Mesh mesh = new()
            {
                name = "SpiderWebMesh",
                indexFormat = vertices.Count > 65000
                    ? UnityEngine.Rendering.IndexFormat.UInt32
                    : UnityEngine.Rendering.IndexFormat.UInt16
            };

            mesh.SetVertices(vertices);
            mesh.SetNormals(normals);
            mesh.SetUVs(0, uvs);
            mesh.SetColors(colors);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateTangents();
            mesh.RecalculateBounds();
            return mesh;
        }

        private static float StrandRadius(
            WebStrandType type,
            float radialRadius,
            float captureRadius,
            float frameRadius,
            float auxiliaryRadius)
        {
            return type switch
            {
                WebStrandType.Radial => radialRadius,
                WebStrandType.Frame => frameRadius,
                WebStrandType.AuxiliarySpiral => auxiliaryRadius,
                _ => captureRadius
            };
        }

        private static Color StrandColor(WebStrandType type)
        {
            return type switch
            {
                WebStrandType.Radial => new Color(0.92f, 0.96f, 1f, 0.85f),
                WebStrandType.Frame => new Color(0.9f, 0.94f, 1f, 0.8f),
                WebStrandType.AuxiliarySpiral => new Color(0.88f, 0.92f, 1f, 0.35f),
                _ => new Color(0.95f, 0.98f, 1f, 0.7f)
            };
        }

        private static void AppendTube(
            IReadOnlyList<Vector3> points,
            float radius,
            int sides,
            Color vertexColor,
            List<Vector3> vertices,
            List<Vector3> normals,
            List<Vector2> uvs,
            List<Color> colors,
            List<int> triangles)
        {
            int ringCount = points.Count;
            int baseVertex = vertices.Count;

            for (int ring = 0; ring < ringCount; ring++)
            {
                Vector3 center = points[ring];
                Vector3 tangent = GetTangent(points, ring);

                Vector3 normal = tangent.sqrMagnitude > 0.0001f
                    ? Vector3.Cross(tangent, Vector3.forward).normalized
                    : Vector3.up;

                if (normal.sqrMagnitude < 0.0001f)
                    normal = Vector3.up;

                Vector3 bitangent = Vector3.Cross(tangent, normal).normalized;
                normal = Vector3.Cross(bitangent, tangent).normalized;

                for (int side = 0; side < sides; side++)
                {
                    float angle = (Mathf.PI * 2f * side) / sides;
                    Vector3 offset = normal * Mathf.Cos(angle) + bitangent * Mathf.Sin(angle);
                    vertices.Add(center + offset * radius);
                    normals.Add(offset.normalized);
                    uvs.Add(new Vector2(side / (float)sides, ring / (float)(ringCount - 1)));
                    colors.Add(vertexColor);
                }
            }

            for (int ring = 0; ring < ringCount - 1; ring++)
            {
                for (int side = 0; side < sides; side++)
                {
                    int current = baseVertex + ring * sides + side;
                    int next = baseVertex + ring * sides + ((side + 1) % sides);
                    int currentNextRing = baseVertex + (ring + 1) * sides + side;
                    int nextNextRing = baseVertex + (ring + 1) * sides + ((side + 1) % sides);

                    triangles.Add(current);
                    triangles.Add(currentNextRing);
                    triangles.Add(next);

                    triangles.Add(next);
                    triangles.Add(currentNextRing);
                    triangles.Add(nextNextRing);
                }
            }
        }

        private static Vector3[] ResamplePolyline(IReadOnlyList<Vector3> points, float maxSegmentLength)
        {
            if (points.Count < 2 || maxSegmentLength <= 0f)
            {
                Vector3[] copy = new Vector3[points.Count];
                for (int i = 0; i < points.Count; i++)
                    copy[i] = points[i];
                return copy;
            }

            List<Vector3> resampled = new() { points[0] };

            for (int i = 0; i < points.Count - 1; i++)
            {
                Vector3 start = points[i];
                Vector3 end = points[i + 1];
                float length = Vector3.Distance(start, end);
                int divisions = Mathf.Max(1, Mathf.CeilToInt(length / maxSegmentLength));

                for (int division = 1; division <= divisions; division++)
                {
                    float t = division / (float)divisions;
                    resampled.Add(Vector3.Lerp(start, end, t));
                }
            }

            return resampled.ToArray();
        }

        private static Vector3 GetTangent(IReadOnlyList<Vector3> points, int index)
        {
            if (points.Count < 2)
                return Vector3.right;

            if (index == 0)
                return points[1] - points[0];

            if (index == points.Count - 1)
                return points[index] - points[index - 1];

            return points[index + 1] - points[index - 1];
        }
    }
}
