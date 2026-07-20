#if UNITY_EDITOR
using System.IO;
using Dexter.Butterfly;
using UnityEditor;
using UnityEngine;

namespace Dexter.Butterfly.Editor
{
    [InitializeOnLoad]
    internal static class ButterflyPrefabAutoBuilder
    {
        static ButterflyPrefabAutoBuilder()
        {
            EditorApplication.delayCall += TryBuildOnce;
        }

        private static void TryBuildOnce()
        {
            if (EditorApplication.isPlayingOrWillChangePlaymode)
                return;

            if (AssetDatabase.LoadAssetAtPath<GameObject>(ButterflyPrefabBuilder.PrefabAssetPath) != null)
                return;

            ButterflyPrefabBuilder.Build();
        }
    }

    public static class ButterflyPrefabBuilder
    {
        public const string PrefabAssetPath = "Assets/ButterflyAnimation/Prefabs/Butterfly.prefab";

        private const string ModelPath = "Assets/Meshes/butterfly_rig.fbx";
        private const string TexturesFolder = "Assets/ButterflyAnimation/Textures";
        private const string MaterialsFolder = "Assets/ButterflyAnimation/Materials";
        private const string PrefabsFolder = "Assets/ButterflyAnimation/Prefabs";
        private const string PrefabPath = PrefabAssetPath;
        private const string BodyMaterialPath = MaterialsFolder + "/ButterflyBody.mat";
        private const string WingMaterialPath = MaterialsFolder + "/ButterflyWings.mat";

        [MenuItem("Dexter/Build Butterfly Prefab")]
        public static void Build()
        {
            EnsureFolder("Assets/ButterflyAnimation/Materials");
            EnsureFolder("Assets/ButterflyAnimation/Prefabs");

            ConfigureTextureImportSettings();

            Material bodyMaterial = CreateOrUpdateBodyMaterial();
            Material wingMaterial = CreateOrUpdateWingMaterial();

            GameObject sourceModel = AssetDatabase.LoadAssetAtPath<GameObject>(ModelPath);
            if (sourceModel == null)
            {
                Debug.LogError("Could not load butterfly model at " + ModelPath);
                return;
            }

            GameObject instance = Object.Instantiate(sourceModel);
            instance.name = "Butterfly";

            AssignMaterials(instance, bodyMaterial, wingMaterial);
            ConfigureRig(instance);

            if (File.Exists(PrefabPath))
                AssetDatabase.DeleteAsset(PrefabPath);

            PrefabUtility.SaveAsPrefabAsset(instance, PrefabPath);
            Object.DestroyImmediate(instance);

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log("Butterfly prefab created at " + PrefabPath, AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath));
        }

        public static void BuildFromCommandLine()
        {
            Build();
            EditorApplication.Exit(0);
        }

        private static void ConfigureTextureImportSettings()
        {
            ConfigureNormalMap(TexturesFolder + "/body_normalmap.jpg");
            ConfigureNormalMap(TexturesFolder + "/wing_normalmap.jpg");
            ConfigureAlphaMap(TexturesFolder + "/wing_alpha.jpg");
        }

        private static void ConfigureAlphaMap(string path)
        {
            TextureImporter importer = AssetImporter.GetAtPath(path) as TextureImporter;
            if (importer == null)
                return;

            bool changed = false;
            if (importer.sRGBTexture)
            {
                importer.sRGBTexture = false;
                changed = true;
            }

            TextureImporterSettings settings = new();
            importer.ReadTextureSettings(settings);
            if (!settings.mipmapEnabled || !settings.mipMapsPreserveCoverage)
            {
                settings.mipmapEnabled = true;
                settings.mipMapsPreserveCoverage = true;
                settings.alphaTestReferenceValue = 0.5f;
                importer.SetTextureSettings(settings);
                changed = true;
            }

            if (changed)
                importer.SaveAndReimport();
        }

        private static void ConfigureNormalMap(string path)
        {
            TextureImporter importer = AssetImporter.GetAtPath(path) as TextureImporter;
            if (importer == null)
                return;

            bool changed = false;
            if (importer.textureType != TextureImporterType.NormalMap)
            {
                importer.textureType = TextureImporterType.NormalMap;
                changed = true;
            }

            if (!importer.sRGBTexture)
            {
                importer.sRGBTexture = false;
                changed = true;
            }

            if (changed)
                importer.SaveAndReimport();
        }

        private static Material CreateOrUpdateBodyMaterial()
        {
            Material material = AssetDatabase.LoadAssetAtPath<Material>(BodyMaterialPath);
            Shader shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null)
                shader = Shader.Find("Lit");

            if (material == null)
            {
                material = new Material(shader) { name = "ButterflyBody" };
                AssetDatabase.CreateAsset(material, BodyMaterialPath);
            }
            else
            {
                material.shader = shader;
            }

            material.SetTexture("_BaseMap", LoadTexture("body_albedo.jpg"));
            material.SetTexture("_BumpMap", LoadTexture("body_normalmap.jpg"));
            material.SetFloat("_Metallic", 0f);
            material.SetFloat("_Smoothness", 0.38f);
            material.EnableKeyword("_NORMALMAP");
            material.doubleSidedGI = false;

            EditorUtility.SetDirty(material);
            return material;
        }

        private static Material CreateOrUpdateWingMaterial()
        {
            Material material = AssetDatabase.LoadAssetAtPath<Material>(WingMaterialPath);
            Shader shader = Shader.Find("Dexter/ButterflyWing");
            if (shader == null)
            {
                Debug.LogError("Dexter/ButterflyWing shader was not found.");
                return material;
            }

            if (material == null)
            {
                material = new Material(shader) { name = "ButterflyWings" };
                AssetDatabase.CreateAsset(material, WingMaterialPath);
            }
            else
            {
                material.shader = shader;
            }

            material.SetTexture("_BaseMap", LoadTexture("wing_albedo.jpg"));
            material.SetTexture("_AlphaMap", LoadTexture("wing_alpha.jpg"));
            material.SetTexture("_BumpMap", LoadTexture("wing_normalmap.jpg"));
            material.SetFloat("_Cutoff", 0.05f);
            material.SetFloat("_Smoothness", 0.45f);
            material.SetFloat("_Metallic", 0f);
            material.doubleSidedGI = true;

            EditorUtility.SetDirty(material);
            return material;
        }

        private static Texture2D LoadTexture(string fileName)
        {
            return AssetDatabase.LoadAssetAtPath<Texture2D>(TexturesFolder + "/" + fileName);
        }

        private static void AssignMaterials(GameObject root, Material bodyMaterial, Material wingMaterial)
        {
            foreach (Renderer renderer in root.GetComponentsInChildren<Renderer>(true))
            {
                string objectName = renderer.gameObject.name.ToLowerInvariant();
                Material targetMaterial = null;

                if (objectName == "wings")
                    targetMaterial = wingMaterial;
                else if (objectName == "body" || objectName == "legs")
                    targetMaterial = bodyMaterial;

                if (targetMaterial == null)
                    continue;

                Material[] materials = renderer.sharedMaterials;
                for (int i = 0; i < materials.Length; i++)
                    materials[i] = targetMaterial;

                renderer.sharedMaterials = materials;
            }
        }

        private static void ConfigureRig(GameObject root)
        {
            foreach (SkinnedMeshRenderer skinnedMesh in root.GetComponentsInChildren<SkinnedMeshRenderer>(true))
            {
                skinnedMesh.updateWhenOffscreen = true;
                skinnedMesh.receiveShadows = true;
            }

            Animator animator = root.GetComponent<Animator>();
            if (animator == null)
                animator = root.AddComponent<Animator>();

            animator.applyRootMotion = false;
            animator.cullingMode = AnimatorCullingMode.AlwaysAnimate;

            Avatar avatar = null;
            foreach (Object asset in AssetDatabase.LoadAllAssetsAtPath(ModelPath))
            {
                if (asset is Avatar foundAvatar)
                {
                    avatar = foundAvatar;
                    break;
                }
            }

            if (avatar != null)
                animator.avatar = avatar;

            if (root.GetComponent<ButterflyWingFlapPreview>() == null)
                root.AddComponent<ButterflyWingFlapPreview>();
        }

        private static void EnsureFolder(string folderPath)
        {
            if (AssetDatabase.IsValidFolder(folderPath))
                return;

            string parent = Path.GetDirectoryName(folderPath)?.Replace('\\', '/');
            string child = Path.GetFileName(folderPath);
            if (!string.IsNullOrEmpty(parent) && !string.IsNullOrEmpty(child))
                AssetDatabase.CreateFolder(parent, child);
        }
    }
}
#endif
