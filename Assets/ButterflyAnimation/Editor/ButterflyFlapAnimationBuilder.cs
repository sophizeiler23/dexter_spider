#if UNITY_EDITOR
using System.Collections.Generic;
using System.IO;
using Dexter.Butterfly;
using UnityEditor;
using UnityEditor.Animations;
using UnityEngine;

namespace Dexter.Butterfly.Editor
{
    public static class ButterflyFlapAnimationBuilder
    {
        public const string ClipAssetPath = "Assets/ButterflyAnimation/Animations/ButterflyFlap.anim";
        public const string ControllerAssetPath = "Assets/ButterflyAnimation/Animations/ButterflyFlap.controller";

        private const string ModelPath = "Assets/Meshes/butterfly_rig.fbx";
        private const float FlapAngleDegrees = 28f;
        private const float CycleDurationSeconds = 1.047f;

        private static readonly WingBone[] WingBones =
        {
            new("wing_upper.L", 1f, 1f),
            new("wing_upper.R", 1f, -1f),
            new("wing_lower.L", 0.55f, 1f),
            new("wing_lower.R", 0.55f, -1f),
        };

        [MenuItem("Dexter/Build Butterfly Flap Animation")]
        public static void BuildForExistingPrefab()
        {
            BuildFlapAnimationInternal();
        }

        public static void BuildFromCommandLine()
        {
            BuildFlapAnimationInternal();
            EditorApplication.Exit(0);
        }

        private static void BuildFlapAnimationInternal()
        {
            GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>(ButterflyPrefabBuilder.PrefabAssetPath);
            if (prefab == null)
            {
                Debug.LogError("Butterfly prefab not found. Run Dexter/Build Butterfly Prefab first.");
                return;
            }

            GameObject instance = PrefabUtility.InstantiatePrefab(prefab) as GameObject;
            if (instance == null)
            {
                Debug.LogError("Could not instantiate Butterfly prefab.");
                return;
            }

            AssignFlapAnimation(instance);
            PrefabUtility.SaveAsPrefabAsset(instance, ButterflyPrefabBuilder.PrefabAssetPath);
            Object.DestroyImmediate(instance);

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log("Butterfly flap animation assigned to " + ButterflyPrefabBuilder.PrefabAssetPath);
        }

        public static void AssignFlapAnimation(GameObject root)
        {
            EnsureFolder("Assets/ButterflyAnimation");
            EnsureFolder("Assets/ButterflyAnimation/Animations");

            AnimationClip clip = CreateOrUpdateFlapClip(root);
            AnimatorController controller = CreateOrUpdateFlapController(clip);

            Animator animator = root.GetComponent<Animator>();
            if (animator == null)
                animator = root.AddComponent<Animator>();

            Avatar avatar = LoadModelAvatar();
            if (avatar != null)
                animator.avatar = avatar;

            animator.runtimeAnimatorController = controller;
            animator.applyRootMotion = false;
            animator.cullingMode = AnimatorCullingMode.AlwaysAnimate;

            ButterflyWingFlapPreview preview = root.GetComponent<ButterflyWingFlapPreview>();
            if (preview != null)
                Object.DestroyImmediate(preview);
        }

        private static AnimationClip CreateOrUpdateFlapClip(GameObject root)
        {
            AnimationClip clip = AssetDatabase.LoadAssetAtPath<AnimationClip>(ClipAssetPath);
            if (clip == null)
            {
                clip = new AnimationClip { name = "ButterflyFlap" };
                AssetDatabase.CreateAsset(clip, ClipAssetPath);
            }

            clip.ClearCurves();

            Dictionary<string, Transform> bonesByName = new();
            foreach (Transform child in root.GetComponentsInChildren<Transform>(true))
                bonesByName[child.name] = child;

            foreach (WingBone wingBone in WingBones)
            {
                if (!bonesByName.TryGetValue(wingBone.Name, out Transform bone))
                {
                    Debug.LogWarning("Butterfly wing bone not found: " + wingBone.Name);
                    continue;
                }

                string path = AnimationUtility.CalculateTransformPath(bone, root.transform);
                AddFlapRotationCurves(clip, path, bone.localRotation, wingBone.AngleScale * wingBone.Sign);
            }

            AnimationClipSettings settings = AnimationUtility.GetAnimationClipSettings(clip);
            settings.loopTime = true;
            settings.loopBlend = false;
            AnimationUtility.SetAnimationClipSettings(clip, settings);

            EditorUtility.SetDirty(clip);
            return clip;
        }

        private static void AddFlapRotationCurves(
            AnimationClip clip,
            string path,
            Quaternion restRotation,
            float signedAngleScale)
        {
            float[] keyTimes =
            {
                0f,
                CycleDurationSeconds * 0.25f,
                CycleDurationSeconds * 0.5f,
                CycleDurationSeconds * 0.75f,
                CycleDurationSeconds,
            };

            float[] keyAngles =
            {
                0f,
                FlapAngleDegrees * signedAngleScale,
                0f,
                -FlapAngleDegrees * signedAngleScale,
                0f,
            };

            AnimationCurve curveX = new();
            AnimationCurve curveY = new();
            AnimationCurve curveZ = new();
            AnimationCurve curveW = new();

            for (int i = 0; i < keyTimes.Length; i++)
            {
                Quaternion rotation = restRotation * Quaternion.Euler(0f, 0f, keyAngles[i]);
                curveX.AddKey(keyTimes[i], rotation.x);
                curveY.AddKey(keyTimes[i], rotation.y);
                curveZ.AddKey(keyTimes[i], rotation.z);
                curveW.AddKey(keyTimes[i], rotation.w);
            }

            clip.SetCurve(path, typeof(Transform), "localRotation.x", curveX);
            clip.SetCurve(path, typeof(Transform), "localRotation.y", curveY);
            clip.SetCurve(path, typeof(Transform), "localRotation.z", curveZ);
            clip.SetCurve(path, typeof(Transform), "localRotation.w", curveW);
        }

        private static AnimatorController CreateOrUpdateFlapController(AnimationClip clip)
        {
            AnimatorController controller = AssetDatabase.LoadAssetAtPath<AnimatorController>(ControllerAssetPath);
            if (controller == null)
                controller = AnimatorController.CreateAnimatorControllerAtPath(ControllerAssetPath);

            AnimatorStateMachine stateMachine = controller.layers[0].stateMachine;
            AnimatorState flapState = FindState(stateMachine, "Flap");
            if (flapState == null)
            {
                flapState = stateMachine.AddState("Flap", new Vector3(300f, 0f, 0f));
                stateMachine.defaultState = flapState;
            }

            flapState.motion = clip;
            EditorUtility.SetDirty(controller);
            return controller;
        }

        private static AnimatorState FindState(AnimatorStateMachine stateMachine, string stateName)
        {
            foreach (ChildAnimatorState childState in stateMachine.states)
            {
                if (childState.state.name == stateName)
                    return childState.state;
            }

            return null;
        }

        private static Avatar LoadModelAvatar()
        {
            foreach (Object asset in AssetDatabase.LoadAllAssetsAtPath(ModelPath))
            {
                if (asset is Avatar avatar)
                    return avatar;
            }

            return null;
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

        private readonly struct WingBone
        {
            public WingBone(string name, float angleScale, float sign)
            {
                Name = name;
                AngleScale = angleScale;
                Sign = sign;
            }

            public string Name { get; }
            public float AngleScale { get; }
            public float Sign { get; }
        }
    }
}
#endif
