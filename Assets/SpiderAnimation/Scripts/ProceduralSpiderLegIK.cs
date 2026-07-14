using System;
using System.Collections.Generic;
using UnityEngine;

namespace Dexter.Spider
{
    /// <summary>
    /// Generates smooth random foot targets and solves each spider leg with CCD IK.
    /// The model root and body bone are restored to their captured poses every frame.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class ProceduralSpiderLegIK : MonoBehaviour
    {
        [Header("Rig Discovery")]
        [SerializeField] private string bodyBoneName = "body";

        [Header("Leaf Displacement (body-local units)")]
        [SerializeField] private Vector3 maximumDisplacement = new Vector3(0.16f, 0.10f, 0.16f);
        [SerializeField, Min(0.05f)] private float minimumMoveDuration = 0.55f;
        [SerializeField, Min(0.05f)] private float maximumMoveDuration = 1.25f;
        [SerializeField] private int randomSeed = 7391;

        [Header("IK")]
        [SerializeField, Range(1, 32)] private int solverIterations = 12;
        [SerializeField, Min(0.00001f)] private float positionTolerance = 0.001f;
        [SerializeField, Range(1f, 180f)] private float maximumJointStepDegrees = 28f;
        [SerializeField] private bool drawTargets = true;

        private sealed class LegChain
        {
            public string Name;
            public Transform[] Joints;
            public Transform Effector;
            public Quaternion[] RestLocalRotations;
            public Vector3 RestTarget;
            public Vector3 PreviousTarget;
            public Vector3 NextTarget;
            public Vector3 DisplayTarget;
            public float MoveStartedAt;
            public float MoveDuration;
        }

        private readonly List<LegChain> legs = new List<LegChain>(8);
        private System.Random random;
        private Transform body;
        private Vector3 rootLocalPosition;
        private Quaternion rootLocalRotation;
        private Vector3 rootLocalScale;
        private Vector3 bodyLocalPosition;
        private Quaternion bodyLocalRotation;
        private Vector3 bodyLocalScale;
        private bool initialized;

        public int LegCount => legs.Count;

        private void Awake()
        {
            InitializeRig();
        }

        private void OnEnable()
        {
            if (!initialized)
                InitializeRig();
        }

        private void LateUpdate()
        {
            if (!initialized)
                return;

            RestoreFixedTransforms();

            float now = Time.time;
            for (int i = 0; i < legs.Count; i++)
            {
                LegChain leg = legs[i];
                RestoreLegPose(leg);
                UpdateTarget(leg, now);
                SolveCcd(leg, body.TransformPoint(leg.DisplayTarget));
            }

            // CCD only rotates leg descendants, but restore these again as a hard invariant.
            RestoreFixedTransforms();
        }

        [ContextMenu("Rebuild Leg Chains")]
        public void InitializeRig()
        {
            initialized = false;
            legs.Clear();
            body = FindDescendant(transform, bodyBoneName);

            if (body == null)
            {
                Debug.LogError($"{nameof(ProceduralSpiderLegIK)} could not find a '{bodyBoneName}' bone below {name}.", this);
                enabled = false;
                return;
            }

            CaptureFixedTransforms();
            random = new System.Random(randomSeed);

            for (int i = 0; i < body.childCount; i++)
            {
                Transform legRoot = body.GetChild(i);
                if (!IsLegRoot(legRoot.name))
                    continue;

                LegChain chain = BuildChain(legRoot);
                if (chain != null)
                    legs.Add(chain);
            }

            legs.Sort((left, right) => string.CompareOrdinal(left.Name, right.Name));
            if (legs.Count == 0)
            {
                Debug.LogError($"{nameof(ProceduralSpiderLegIK)} found the body but no front/mid/back leg chains.", this);
                enabled = false;
                return;
            }

            float now = Time.time;
            for (int i = 0; i < legs.Count; i++)
            {
                LegChain leg = legs[i];
                leg.PreviousTarget = leg.RestTarget;
                leg.NextTarget = CreateRandomTarget(leg);
                leg.DisplayTarget = leg.RestTarget;
                leg.MoveStartedAt = now + i * 0.07f;
                leg.MoveDuration = RandomDuration();
            }

            initialized = true;
            Debug.Log($"{nameof(ProceduralSpiderLegIK)} initialized {legs.Count} leg chains under {body.name}.", this);
        }

        private LegChain BuildChain(Transform legRoot)
        {
            var transforms = new List<Transform>(6);
            Transform current = legRoot;
            transforms.Add(current);

            while (current.childCount == 1 && transforms.Count < 16)
            {
                current = current.GetChild(0);
                transforms.Add(current);
            }

            if (transforms.Count < 3)
            {
                Debug.LogWarning($"Ignoring short leg chain {legRoot.name}.", legRoot);
                return null;
            }

            Transform effector = transforms[transforms.Count - 1];
            transforms.RemoveAt(transforms.Count - 1);
            var restRotations = new Quaternion[transforms.Count];
            for (int i = 0; i < transforms.Count; i++)
                restRotations[i] = transforms[i].localRotation;

            Vector3 restTarget = body.InverseTransformPoint(effector.position);
            return new LegChain
            {
                Name = legRoot.name,
                Joints = transforms.ToArray(),
                Effector = effector,
                RestLocalRotations = restRotations,
                RestTarget = restTarget
            };
        }

        private void UpdateTarget(LegChain leg, float now)
        {
            if (now >= leg.MoveStartedAt + leg.MoveDuration)
            {
                leg.PreviousTarget = leg.NextTarget;
                leg.NextTarget = CreateRandomTarget(leg);
                leg.MoveStartedAt = now;
                leg.MoveDuration = RandomDuration();
            }

            float progress = Mathf.InverseLerp(leg.MoveStartedAt, leg.MoveStartedAt + leg.MoveDuration, now);
            float smoothProgress = progress * progress * (3f - 2f * progress);
            leg.DisplayTarget = Vector3.LerpUnclamped(leg.PreviousTarget, leg.NextTarget, smoothProgress);
        }

        private Vector3 CreateRandomTarget(LegChain leg)
        {
            Vector3 offset = new Vector3(
                RandomSigned() * maximumDisplacement.x,
                RandomSigned() * maximumDisplacement.y,
                RandomSigned() * maximumDisplacement.z);

            // Bias toward smaller natural motions while still occasionally reaching the full range.
            offset *= Mathf.Sqrt((float)random.NextDouble());
            return leg.RestTarget + offset;
        }

        private void SolveCcd(LegChain leg, Vector3 worldTarget)
        {
            float toleranceSquared = positionTolerance * positionTolerance;

            for (int iteration = 0; iteration < solverIterations; iteration++)
            {
                if ((leg.Effector.position - worldTarget).sqrMagnitude <= toleranceSquared)
                    return;

                for (int jointIndex = leg.Joints.Length - 1; jointIndex >= 0; jointIndex--)
                {
                    Transform joint = leg.Joints[jointIndex];
                    Vector3 toEffector = leg.Effector.position - joint.position;
                    Vector3 toTarget = worldTarget - joint.position;
                    if (toEffector.sqrMagnitude < 0.0000001f || toTarget.sqrMagnitude < 0.0000001f)
                        continue;

                    Quaternion fullDelta = Quaternion.FromToRotation(toEffector, toTarget);
                    Quaternion limitedDelta = Quaternion.RotateTowards(
                        Quaternion.identity,
                        fullDelta,
                        maximumJointStepDegrees);
                    joint.rotation = limitedDelta * joint.rotation;
                }
            }
        }

        private void CaptureFixedTransforms()
        {
            rootLocalPosition = transform.localPosition;
            rootLocalRotation = transform.localRotation;
            rootLocalScale = transform.localScale;
            bodyLocalPosition = body.localPosition;
            bodyLocalRotation = body.localRotation;
            bodyLocalScale = body.localScale;
        }

        private void RestoreFixedTransforms()
        {
            transform.localPosition = rootLocalPosition;
            transform.localRotation = rootLocalRotation;
            transform.localScale = rootLocalScale;
            body.localPosition = bodyLocalPosition;
            body.localRotation = bodyLocalRotation;
            body.localScale = bodyLocalScale;
        }

        private static void RestoreLegPose(LegChain leg)
        {
            for (int i = 0; i < leg.Joints.Length; i++)
                leg.Joints[i].localRotation = leg.RestLocalRotations[i];
        }

        private static bool IsLegRoot(string boneName)
        {
            return boneName.StartsWith("front", StringComparison.OrdinalIgnoreCase)
                || boneName.StartsWith("mid", StringComparison.OrdinalIgnoreCase)
                || boneName.StartsWith("back", StringComparison.OrdinalIgnoreCase);
        }

        private static Transform FindDescendant(Transform root, string targetName)
        {
            Transform[] descendants = root.GetComponentsInChildren<Transform>(true);
            for (int i = 0; i < descendants.Length; i++)
            {
                if (string.Equals(descendants[i].name, targetName, StringComparison.OrdinalIgnoreCase))
                    return descendants[i];
            }
            return null;
        }

        private float RandomDuration()
        {
            float minimum = Mathf.Max(0.05f, minimumMoveDuration);
            float maximum = Mathf.Max(minimum, maximumMoveDuration);
            return Mathf.Lerp(minimum, maximum, (float)random.NextDouble());
        }

        private float RandomSigned()
        {
            return (float)random.NextDouble() * 2f - 1f;
        }

        private void OnDrawGizmosSelected()
        {
            if (!drawTargets || !initialized || body == null)
                return;

            Gizmos.color = new Color(0.2f, 0.9f, 1f, 0.9f);
            for (int i = 0; i < legs.Count; i++)
            {
                Vector3 target = body.TransformPoint(legs[i].DisplayTarget);
                Gizmos.DrawWireSphere(target, 0.025f);
                Gizmos.DrawLine(legs[i].Effector.position, target);
            }
        }

        private void OnValidate()
        {
            maximumDisplacement.x = Mathf.Max(0f, maximumDisplacement.x);
            maximumDisplacement.y = Mathf.Max(0f, maximumDisplacement.y);
            maximumDisplacement.z = Mathf.Max(0f, maximumDisplacement.z);
            minimumMoveDuration = Mathf.Max(0.05f, minimumMoveDuration);
            maximumMoveDuration = Mathf.Max(minimumMoveDuration, maximumMoveDuration);
            solverIterations = Mathf.Clamp(solverIterations, 1, 32);
            positionTolerance = Mathf.Max(0.00001f, positionTolerance);
        }
    }
}
