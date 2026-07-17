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

        public static Material GetLineMaterial()
        {
            if (sharedLineMaterial != null)
                return sharedLineMaterial;

            Shader shader = Shader.Find("Sprites/Default");
            if (shader == null)
                shader = Shader.Find("Universal Render Pipeline/Unlit");

            sharedLineMaterial = new Material(shader)
            {
                name = "SpiderSilkLineRuntime",
                renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent
            };

            sharedLineMaterial.mainTexture = GetLineFalloffTexture();
            sharedLineMaterial.color = new Color(0.94f, 0.97f, 1f, 0.92f);
            return sharedLineMaterial;
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
    }
}
