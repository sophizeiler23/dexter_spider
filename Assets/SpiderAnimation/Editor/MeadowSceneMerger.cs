#if UNITY_EDITOR
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Dexter.Spider.Editor
{
    internal static class MeadowSceneMerger
    {
        private const string BabyScenePath = "Assets/Scenes/SpiderBabyStepsScene.unity";
        private const string BackupScenePath = "Assets/Scenes/SpiderBabyStepsScene Before Meadow.unity";
        private const string MeadowScenePath =
            "Assets/NatureManufacture Assets/Meadow Environment Dynamic Nature/Demo Scenes/Unity URP Demo Scene.unity";
        private const string EnvironmentRootName = "URP Meadow Environment";

        [MenuItem("Dexter/Merge URP Meadow Into Baby Steps Scene")]
        private static void MergeMeadowIntoBabySteps()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
                return;

            if (!File.Exists(BabyScenePath) || !File.Exists(MeadowScenePath))
            {
                Debug.LogError("The Baby Steps or URP Meadow source scene could not be found.");
                return;
            }

            if (!File.Exists(BackupScenePath))
            {
                AssetDatabase.CopyAsset(BabyScenePath, BackupScenePath);
                AssetDatabase.ImportAsset(BackupScenePath);
            }

            Scene babyScene = EditorSceneManager.OpenScene(BabyScenePath, OpenSceneMode.Single);
            GameObject existingEnvironment = FindRoot(babyScene, EnvironmentRootName);
            if (existingEnvironment != null)
            {
                Debug.LogWarning("The URP Meadow environment is already present in Baby Steps.");
                return;
            }

            GameObject spider = FindRoot(babyScene, "spider_rig");
            RemoveOldEnvironmentRoots(babyScene);

            Scene meadowScene = EditorSceneManager.OpenScene(MeadowScenePath, OpenSceneMode.Additive);
            Camera meadowCamera = FindSceneCamera(meadowScene);
            Vector3 preferredSpawn = meadowCamera != null ? meadowCamera.transform.position : Vector3.zero;

            var environmentRoot = new GameObject(EnvironmentRootName);
            SceneManager.MoveGameObjectToScene(environmentRoot, babyScene);

            GameObject[] meadowRoots = meadowScene.GetRootGameObjects();
            var rootsToMove = new List<GameObject>(meadowRoots.Length);
            for (int i = 0; i < meadowRoots.Length; i++)
            {
                GameObject root = meadowRoots[i];
                if (root.GetComponentInChildren<Camera>(true) != null ||
                    root.GetComponentInChildren<AudioListener>(true) != null)
                    continue;
                rootsToMove.Add(root);
            }

            for (int i = 0; i < rootsToMove.Count; i++)
            {
                GameObject root = rootsToMove[i];
                SceneManager.MoveGameObjectToScene(root, babyScene);
                root.transform.SetParent(environmentRoot.transform, true);
            }

            Terrain meadowTerrain = environmentRoot.GetComponentInChildren<Terrain>(true);
            if (spider != null && meadowTerrain != null)
                PlaceSpiderOnTerrain(spider.transform, meadowTerrain, preferredSpawn);

            EditorSceneManager.CloseScene(meadowScene, true);
            EditorSceneManager.MarkSceneDirty(babyScene);
            EditorSceneManager.SaveScene(babyScene);
            Selection.activeGameObject = environmentRoot;
            Debug.Log(
                $"Merged {rootsToMove.Count} URP Meadow root objects into Baby Steps. " +
                $"Backup: {BackupScenePath}",
                environmentRoot);
        }

        private static void RemoveOldEnvironmentRoots(Scene scene)
        {
            string[] oldRootNames = { "Terrain", "Global Volume", "Directional Light" };
            for (int i = 0; i < oldRootNames.Length; i++)
            {
                GameObject root = FindRoot(scene, oldRootNames[i]);
                if (root != null)
                    Object.DestroyImmediate(root);
            }
        }

        private static void PlaceSpiderOnTerrain(
            Transform spider,
            Terrain terrain,
            Vector3 preferredSpawn)
        {
            Vector3 terrainPosition = terrain.transform.position;
            Vector3 terrainSize = terrain.terrainData.size;
            bool preferredIsOnTerrain =
                preferredSpawn.x >= terrainPosition.x &&
                preferredSpawn.x <= terrainPosition.x + terrainSize.x &&
                preferredSpawn.z >= terrainPosition.z &&
                preferredSpawn.z <= terrainPosition.z + terrainSize.z;

            Vector3 spawn = preferredIsOnTerrain
                ? preferredSpawn
                : terrainPosition + new Vector3(terrainSize.x * 0.5f, 0f, terrainSize.z * 0.5f);
            spawn.y = terrain.SampleHeight(spawn) + terrainPosition.y + 0.3f;
            Undo.RecordObject(spider, "Place spider on meadow terrain");
            spider.position = spawn;
        }

        private static Camera FindSceneCamera(Scene scene)
        {
            GameObject[] roots = scene.GetRootGameObjects();
            for (int i = 0; i < roots.Length; i++)
            {
                Camera camera = roots[i].GetComponentInChildren<Camera>(true);
                if (camera != null)
                    return camera;
            }
            return null;
        }

        private static GameObject FindRoot(Scene scene, string objectName)
        {
            GameObject[] roots = scene.GetRootGameObjects();
            for (int i = 0; i < roots.Length; i++)
            {
                if (roots[i].name == objectName)
                    return roots[i];
            }
            return null;
        }
    }
}
#endif
