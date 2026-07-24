using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// Materials for view-aligned silk lines that stay smooth at any camera distance.
    /// </summary>
    public static class SpiderWebMaterialFactory
    {
        private static Material sharedLineMaterial;
        private static Texture2D lineFalloffTexture;
        private static Texture2D dottedLineTexture;
        private static Shader cocoonShader;
        private static Shader lineShader;

        /// <summary>Creates a standalone opaque white-silk material for a cocoon instance.</summary>
        public static Material CreateCocoonMaterial(Color color)
        {
            if (cocoonShader == null)
            {
                cocoonShader = Shader.Find("Universal Render Pipeline/Lit");
                if (cocoonShader == null)
                    cocoonShader = Shader.Find("Standard");
                if (cocoonShader == null)
                    cocoonShader = Shader.Find("Sprites/Default");
            }

            Material material = new(cocoonShader) { name = "SpiderWebCocoonRuntime" };

            if (material.HasProperty("_BaseColor"))
                material.SetColor("_BaseColor", color);
            else
                material.color = color;

            if (material.HasProperty("_Smoothness"))
                material.SetFloat("_Smoothness", 0.1f);
            if (material.HasProperty("_Glossiness"))
                material.SetFloat("_Glossiness", 0.1f);

            return material;
        }

        public static Material GetLineMaterial()
        {
            if (sharedLineMaterial != null)
                return sharedLineMaterial;

            sharedLineMaterial = new Material(GetLineShader())
            {
                name = "SpiderSilkLineRuntime",
                renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent
            };

            sharedLineMaterial.mainTexture = GetLineFalloffTexture();
            sharedLineMaterial.color = new Color(0.94f, 0.97f, 1f, 0.92f);
            return sharedLineMaterial;
        }

        /// <summary>
        /// Creates a standalone (non-shared) dashed-look material for a trajectory preview line, so
        /// each caller can independently tint it and tile the dash pattern (via
        /// <c>LineRenderer.textureMode = Tile</c> plus <c>material.mainTextureScale</c>) without
        /// disturbing <see cref="GetLineMaterial"/>'s shared silk material.
        /// </summary>
        public static Material CreateDottedLineMaterial(Color color)
        {
            Material material = new(GetLineShader())
            {
                name = "SpiderWebTrajectoryPreviewRuntime",
                renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent
            };

            material.mainTexture = GetDottedLineTexture();
            material.color = color;
            return material;
        }

        private static Shader GetLineShader()
        {
            if (lineShader != null)
                return lineShader;

            lineShader = Shader.Find("Sprites/Default");
            if (lineShader == null)
                lineShader = Shader.Find("Universal Render Pipeline/Unlit");
            return lineShader;
        }

        private static Texture2D GetLineFalloffTexture()
        {
            if (lineFalloffTexture != null)
                return lineFalloffTexture;

            const int width = 128;
            const int height = 8;
            lineFalloffTexture = new Texture2D(width, height, TextureFormat.RGBA32, false)
            {
                name = "SpiderSilkLineFalloff",
                wrapMode = TextureWrapMode.Clamp,
                filterMode = FilterMode.Bilinear
            };

            for (int x = 0; x < width; x++)
            {
                float t = x / (width - 1f);
                float edge = 1f - Mathf.Abs(t - 0.5f) * 2f;
                float alpha = Mathf.Pow(Mathf.Clamp01(edge), 0.45f);

                for (int y = 0; y < height; y++)
                    lineFalloffTexture.SetPixel(x, y, new Color(1f, 1f, 1f, alpha));
            }

            lineFalloffTexture.Apply();
            return lineFalloffTexture;
        }

        /// <summary>
        /// A single dash-and-gap tile, meant to be repeated along a line's length via
        /// <c>LineRenderer.textureMode = Tile</c> and a <c>mainTextureScale.x</c> proportional to the
        /// line's length, to get a dotted/dashed appearance out of an otherwise-solid LineRenderer.
        /// </summary>
        private static Texture2D GetDottedLineTexture()
        {
            if (dottedLineTexture != null)
                return dottedLineTexture;

            const int width = 64;
            const int height = 8;
            const float dashFraction = 0.55f;

            dottedLineTexture = new Texture2D(width, height, TextureFormat.RGBA32, false)
            {
                name = "SpiderWebTrajectoryDotted",
                wrapMode = TextureWrapMode.Repeat,
                filterMode = FilterMode.Bilinear
            };

            for (int x = 0; x < width; x++)
            {
                float t = x / (float)width;
                // Soft-edged dash so it doesn't alias/flicker at a distance, faded to nothing past dashFraction.
                float edge = Mathf.Clamp01((dashFraction - t) / 0.08f);
                float alpha = t < dashFraction ? Mathf.Clamp01(edge + 0.4f) : 0f;

                for (int y = 0; y < height; y++)
                    dottedLineTexture.SetPixel(x, y, new Color(1f, 1f, 1f, alpha));
            }

            dottedLineTexture.Apply();
            return dottedLineTexture;
        }
    }
}
