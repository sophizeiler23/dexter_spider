#if UNITY_EDITOR
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using UnityEngine.SceneManagement;

namespace Dexter.Spider.Editor
{
    /// <summary>
    /// Applies NatureManufacture Meadow URP configuration from the 17.3 support-pack README.
    /// </summary>
    internal static class MeadowUrSetup
    {
        private const string SupportPackFolder =
            "Assets/NatureManufacture Assets/Meadow Environment Dynamic Nature/HD and URP Support Packs";
        private const string UrpSupportPackFileName = "URP 17.3 Unity 6.3 Meadow Environment.unitypackage";
        private const string UrpSupportPackPath = SupportPackFolder + "/" + UrpSupportPackFileName;
        private const string PcPipelineAssetPath = "Assets/Settings/PC_RPAsset.asset";
        private const string PcRendererAssetPath = "Assets/Settings/PC_Renderer.asset";
        private const string MeadowVolumeProfilePath =
            "Assets/NatureManufacture Assets/Meadow Environment Dynamic Nature/Demo Scenes/Meadow Post Process URP.asset";
        private const string TerrainMaterialPath =
            "Assets/NatureManufacture Assets/Meadow Environment Dynamic Nature/Demo Scenes/Terrain Material.mat";
        private const string MeadowDemoScenePath =
            "Assets/NatureManufacture Assets/Meadow Environment Dynamic Nature/Demo Scenes/Unity URP Demo Scene.unity";
        private const string SpiderBodyMaterialPath = "Assets/SpiderAnimation/Materials/Spider Body.mat";
        private const string SpiderLegsMaterialPath = "Assets/SpiderAnimation/Materials/Spider Legs.mat";
        private const string SpiderModelPath = "Assets/Meshes/spider_rig.fbx";

        [MenuItem("Dexter/Configure Meadow URP")]
        public static void ConfigureFromMenu()
        {
            ConfigureAll(logToConsole: true, showDialogs: true);
        }

        public static void ConfigureAllBatch()
        {
            ConfigureAll(logToConsole: true, showDialogs: false);
            EditorApplication.Exit(0);
        }

        private static void ConfigureAll(bool logToConsole, bool showDialogs)
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
                return;

            bool importedSupportPack = TryImportSupportPack(logToConsole);
            ConfigurePipelineAssets(logToConsole);
            FixTerrainMaterial(logToConsole);
            FixSpiderMaterials(logToConsole);
            ApplySceneRenderSettings(logToConsole);

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            string summary =
                "Meadow URP configuration applied.\n\n" +
                (importedSupportPack
                    ? "- Imported the Meadow URP support pack.\n"
                    : "- URP support pack not found. Re-import it from the Asset Store package folder:\n  " +
                      SupportPackFolder + "\n\n") +
                "- Updated PC pipeline settings (shadow distance, HDR, Meadow volume profile).\n" +
                "- Refreshed terrain material shader.\n" +
                "- Applied Meadow sky/fog/lighting to the active scene.\n" +
                "- Assigned spider URP materials where possible.\n\n" +
                "Open the Meadow URP demo scene to verify the environment, then return to your spider scene.";

            if (logToConsole)
                Debug.Log(summary);

            if (showDialogs)
                EditorUtility.DisplayDialog("Meadow URP configured", summary, "OK");
        }

        private static bool TryImportSupportPack(bool logToConsole)
        {
            if (!File.Exists(UrpSupportPackPath))
            {
                if (logToConsole)
                {
                    Debug.LogWarning(
                        "Meadow URP support pack not found at:\n" + UrpSupportPackPath + "\n" +
                        "Download it from the Asset Store package (HD and URP Support Packs folder) and import it there.");
                }

                return false;
            }

            AssetDatabase.ImportPackage(UrpSupportPackPath, false);
            if (logToConsole)
                Debug.Log("Imported Meadow URP support pack: " + UrpSupportPackPath);

            return true;
        }

        private static void ConfigurePipelineAssets(bool logToConsole)
        {
            UniversalRenderPipelineAsset pipelineAsset =
                AssetDatabase.LoadAssetAtPath<UniversalRenderPipelineAsset>(PcPipelineAssetPath);
            if (pipelineAsset != null)
            {
                SerializedObject pipeline = new(pipelineAsset);
                SerializedProperty shadowDistance = pipeline.FindProperty("m_ShadowDistance");
                if (shadowDistance != null && shadowDistance.floatValue < 150f)
                    shadowDistance.floatValue = 150f;

                SerializedProperty hdr = pipeline.FindProperty("m_SupportsHDR");
                if (hdr != null)
                    hdr.boolValue = true;

                SerializedProperty mainShadowResolution = pipeline.FindProperty("m_MainLightShadowmapResolution");
                if (mainShadowResolution != null && mainShadowResolution.intValue < 2048)
                    mainShadowResolution.intValue = 2048;

                VolumeProfile meadowVolume =
                    AssetDatabase.LoadAssetAtPath<VolumeProfile>(MeadowVolumeProfilePath);
                if (meadowVolume != null)
                {
                    SerializedProperty volumeProfile = pipeline.FindProperty("m_VolumeProfile");
                    if (volumeProfile != null)
                        volumeProfile.objectReferenceValue = meadowVolume;
                }

                pipeline.ApplyModifiedPropertiesWithoutUndo();
                EditorUtility.SetDirty(pipelineAsset);
            }

            UniversalRendererData rendererData =
                AssetDatabase.LoadAssetAtPath<UniversalRendererData>(PcRendererAssetPath);
            if (rendererData != null)
            {
                SerializedObject renderer = new(rendererData);
                SerializedProperty renderingMode = renderer.FindProperty("m_RenderingMode");
                if (renderingMode != null && renderingMode.intValue == (int)RenderingMode.Forward)
                    renderingMode.intValue = (int)RenderingMode.ForwardPlus;

                renderer.ApplyModifiedPropertiesWithoutUndo();
                EditorUtility.SetDirty(rendererData);
            }

            if (logToConsole)
                Debug.Log("Updated PC URP pipeline assets for Meadow.");
        }

        private static void FixTerrainMaterial(bool logToConsole)
        {
            Material terrainMaterial = AssetDatabase.LoadAssetAtPath<Material>(TerrainMaterialPath);
            if (terrainMaterial == null)
            {
                if (logToConsole)
                    Debug.LogWarning("Terrain Material not found at " + TerrainMaterialPath);
                return;
            }

            Shader terrainShader = Shader.Find("Universal Render Pipeline/Terrain/Lit");
            if (terrainShader == null)
            {
                if (logToConsole)
                    Debug.LogError("URP Terrain/Lit shader was not found. Is com.unity.render-pipelines.universal installed?");
                return;
            }

            if (terrainMaterial.shader != terrainShader)
            {
                terrainMaterial.shader = terrainShader;
                EditorUtility.SetDirty(terrainMaterial);
            }

            Terrain terrain = Object.FindFirstObjectByType<Terrain>();
            if (terrain != null)
            {
                terrain.materialTemplate = terrainMaterial;
                terrain.drawInstanced = true;
                EditorUtility.SetDirty(terrain);
                EditorSceneManager.MarkSceneDirty(terrain.gameObject.scene);
            }

            if (logToConsole)
                Debug.Log("Terrain material bound to URP Terrain/Lit.");
        }

        private static void FixSpiderMaterials(bool logToConsole)
        {
            Material bodyMaterial = AssetDatabase.LoadAssetAtPath<Material>(SpiderBodyMaterialPath);
            Material legsMaterial = AssetDatabase.LoadAssetAtPath<Material>(SpiderLegsMaterialPath);
            if (bodyMaterial == null && legsMaterial == null)
                return;

            Shader urpLit = Shader.Find("Universal Render Pipeline/Lit");
            if (urpLit != null)
            {
                if (bodyMaterial != null && bodyMaterial.shader != urpLit)
                {
                    bodyMaterial.shader = urpLit;
                    EditorUtility.SetDirty(bodyMaterial);
                }

                if (legsMaterial != null && legsMaterial.shader != urpLit)
                {
                    legsMaterial.shader = urpLit;
                    EditorUtility.SetDirty(legsMaterial);
                }
            }

            foreach (Renderer renderer in Object.FindObjectsByType<Renderer>(FindObjectsSortMode.None))
            {
                if (renderer.gameObject.name != "spider_rig" &&
                    !renderer.transform.root.name.Contains("spider_rig"))
                    continue;

                Material[] sharedMaterials = renderer.sharedMaterials;
                bool changed = false;
                for (int i = 0; i < sharedMaterials.Length; i++)
                {
                    Material replacement = PickSpiderMaterial(sharedMaterials[i], bodyMaterial, legsMaterial);
                    if (replacement == null || sharedMaterials[i] == replacement)
                        continue;

                    sharedMaterials[i] = replacement;
                    changed = true;
                }

                if (changed)
                {
                    renderer.sharedMaterials = sharedMaterials;
                    EditorUtility.SetDirty(renderer);
                    EditorSceneManager.MarkSceneDirty(renderer.gameObject.scene);
                }
            }

            if (logToConsole)
                Debug.Log("Spider renderers updated to use URP Lit materials.");
        }

        private static Material PickSpiderMaterial(
            Material current,
            Material bodyMaterial,
            Material legsMaterial)
        {
            if (current == null)
                return bodyMaterial ?? legsMaterial;

            string name = current.name.ToLowerInvariant();
            if (name.Contains("leg"))
                return legsMaterial ?? bodyMaterial;

            if (name.Contains("body") || name.Contains("default") || name.Contains("material"))
                return bodyMaterial ?? legsMaterial;

            if (current.shader == null || current.shader.name.Contains("Error"))
                return bodyMaterial ?? legsMaterial;

            return null;
        }

        private static void ApplySceneRenderSettings(bool logToConsole)
        {
            Scene activeScene = SceneManager.GetActiveScene();
            if (!activeScene.IsValid())
                return;

            RenderSettings.skybox = null;
            RenderSettings.ambientIntensity = 1.2f;
            RenderSettings.fog = true;
            RenderSettings.fogMode = FogMode.Linear;
            RenderSettings.fogColor = new Color(0.6797686f, 0.7973164f, 0.9338235f, 1f);
            RenderSettings.fogStartDistance = 0f;
            RenderSettings.fogEndDistance = 2300f;

            Light directionalLight = FindDirectionalLight();
            if (directionalLight != null)
                RenderSettings.sun = directionalLight;

            EditorSceneManager.MarkSceneDirty(activeScene);

            if (logToConsole)
                Debug.Log("Applied Meadow render settings to scene " + activeScene.name + ".");
        }

        private static Light FindDirectionalLight()
        {
            Light[] lights = Object.FindObjectsByType<Light>(FindObjectsSortMode.None);
            for (int i = 0; i < lights.Length; i++)
            {
                if (lights[i].type == LightType.Directional)
                    return lights[i];
            }

            return null;
        }

        [MenuItem("Dexter/Open Meadow URP Demo Scene")]
        private static void OpenMeadowDemoScene()
        {
            if (File.Exists(MeadowDemoScenePath))
                EditorSceneManager.OpenScene(MeadowDemoScenePath);
        }
    }
}
#endif
