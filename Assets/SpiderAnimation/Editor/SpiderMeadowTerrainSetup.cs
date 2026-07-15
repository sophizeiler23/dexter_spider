#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Dexter.Spider.Editor
{
    internal static class SpiderMeadowTerrainSetup
    {
        private const string SceneName = "SpiderBabyStepsScene";
        private const string LayerFolder = "Assets/SpiderAnimation/Terrain";
        private const string LayerPath = LayerFolder + "/Meadow Grass.terrainlayer";
        private const string AlbedoPath = "Assets/NatureManufacture Assets/Meadow Environment Dynamic Nature/Ground/T_ground_meadow_grass_01_A_SM.tga";
        private const string NormalPath = "Assets/NatureManufacture Assets/Meadow Environment Dynamic Nature/Ground/T_ground_meadow_grass_01_N.png";

        [MenuItem("Dexter/Apply Meadow Terrain")]
        private static void ApplyMeadowTerrain()
        {
            Scene scene = SceneManager.GetActiveScene();
            if (!scene.IsValid() || scene.name != SceneName)
                return;

            Terrain terrain = Object.FindFirstObjectByType<Terrain>();
            if (terrain == null || terrain.terrainData == null)
                return;

            TerrainLayer meadowLayer = AssetDatabase.LoadAssetAtPath<TerrainLayer>(LayerPath);
            if (meadowLayer == null)
            {
                EnsureFolder(LayerFolder);
                meadowLayer = new TerrainLayer
                {
                    name = "Meadow Grass",
                    diffuseTexture = AssetDatabase.LoadAssetAtPath<Texture2D>(AlbedoPath),
                    normalMapTexture = AssetDatabase.LoadAssetAtPath<Texture2D>(NormalPath),
                    tileSize = new Vector2(8f, 8f),
                    normalScale = 1f,
                    smoothness = 0.2f
                };
                AssetDatabase.CreateAsset(meadowLayer, LayerPath);
            }

            TerrainLayer[] layers = terrain.terrainData.terrainLayers;
            if (layers.Length == 1 && layers[0] == meadowLayer)
                return;

            Undo.RecordObject(terrain.terrainData, "Apply Meadow Terrain");
            terrain.terrainData.terrainLayers = new[] { meadowLayer };
            EditorUtility.SetDirty(terrain.terrainData);
            EditorSceneManager.MarkSceneDirty(scene);
            AssetDatabase.SaveAssets();
            Debug.Log("Applied NatureManufacture meadow grass to the spider terrain.", terrain);
        }

        private static void EnsureFolder(string folderPath)
        {
            string[] parts = folderPath.Split('/');
            string current = parts[0];
            for (int i = 1; i < parts.Length; i++)
            {
                string next = current + "/" + parts[i];
                if (!AssetDatabase.IsValidFolder(next))
                    AssetDatabase.CreateFolder(current, parts[i]);
                current = next;
            }
        }
    }
}
#endif
