using UnityEngine;
using UnityEngine.Rendering;

namespace Dexter.Spider
{
    /// <summary>
    /// Read-only, runtime visualization of Dexter commands and the forces
    /// applied by SpiderRigidbodyDynamics.
    /// </summary>
    [DisallowMultipleComponent]
    [DefaultExecutionOrder(1000)]
    public sealed class SpiderForceDebugOverlay : MonoBehaviour
    {
        [Header("Overlay")]
        [SerializeField] private bool showOverlay = true;
        [SerializeField] private KeyCode toggleKey = KeyCode.F3;
        [SerializeField] private bool showNumericValues = true;

        [Header("World Arrows")]
        [SerializeField, Min(0f)] private float anchorHeight = 0.10f;
        [SerializeField, Min(0.001f)] private float forceVectorScale = 0.045f;
        [SerializeField, Min(0.01f)] private float inputVectorScale = 0.40f;
        [SerializeField, Min(0.01f)] private float turnVectorScale = 1.50f;
        [SerializeField, Min(0.1f)] private float vectorLengthMultiplier = 5f;
        [SerializeField, Min(0.1f)] private float maximumArrowLength = 20f;
        [SerializeField, Min(0f)] private float rawInputSideOffset = 0.42f;

        private static readonly Color DriveColor =
            new Color(0.20f, 1f, 0.25f);
        private static readonly Color LateralCorrectionColor =
            new Color(0.10f, 0.85f, 1f);
        private static readonly Color TurnColor =
            new Color(1f, 0.85f, 0.10f);
        private static readonly Color TorqueColor =
            new Color(1f, 0.20f, 0.90f);
        private static readonly Color FrictionColor =
            new Color(1f, 0.48f, 0.08f);
        private static readonly Color AdhesionColor =
            new Color(0.65f, 0.25f, 1f);
        private static readonly Color IndexForceColor =
            new Color(0.55f, 1f, 0.10f);
        private static readonly Color MiddleForceColor =
            new Color(1f, 0.35f, 0.62f);

        private readonly LineRenderer[] vectorLines = new LineRenderer[8];
        private DexterFrontLegIK controller;
        private SpiderRigidbodyDynamics dynamics;
        private Rigidbody physicsBody;
        private Transform vectorLineRoot;
        private Material vectorLineMaterial;
        private GUIStyle legendStyle;
        private GUIStyle titleStyle;
        private GUIStyle buttonStyle;

        private void Awake()
        {
            controller = GetComponent<DexterFrontLegIK>();
            dynamics = GetComponent<SpiderRigidbodyDynamics>();
            physicsBody = dynamics != null
                ? dynamics.PhysicsBody
                : GetComponent<Rigidbody>();
            EnsureVectorLines();
        }

        private void LateUpdate()
        {
            if (controller == null)
                controller = GetComponent<DexterFrontLegIK>();
            if (dynamics == null)
                dynamics = GetComponent<SpiderRigidbodyDynamics>();
            if (physicsBody == null)
                physicsBody = dynamics != null
                    ? dynamics.PhysicsBody
                    : GetComponent<Rigidbody>();
            UpdateVectorLines();
        }

        private void OnGUI()
        {
            Event currentEvent = Event.current;
            if (currentEvent != null &&
                currentEvent.type == EventType.KeyDown &&
                currentEvent.keyCode == toggleKey)
            {
                showOverlay = !showOverlay;
                currentEvent.Use();
            }

            EnsureGuiStyles();
            const float buttonWidth = 190f;
            Rect toggleRect = new Rect(
                Screen.width - buttonWidth - 14f, 14f,
                buttonWidth, 28f);
            if (GUI.Button(
                    toggleRect,
                    $"[{toggleKey}] FORCE OVERLAY: " +
                    (showOverlay ? "ON" : "OFF"),
                    buttonStyle))
                showOverlay = !showOverlay;
            if (!showOverlay)
                return;

            const float panelWidth = 278f;
            float panelHeight = showNumericValues ? 294f : 232f;
            Rect panel = new Rect(
                Screen.width - panelWidth - 14f,
                toggleRect.yMax + 8f,
                panelWidth, panelHeight);
            GUI.Box(panel, GUIContent.none);
            GUI.Label(
                new Rect(panel.x + 12f, panel.y + 8f,
                    panel.width - 24f, 26f),
                "SPIDER FORCE / COMMAND VECTORS",
                titleStyle);

            float y = panel.y + 40f;
            DrawLegendRow(panel.x + 12f, ref y, DriveColor,
                "Forward propulsion", GetMagnitude(
                    dynamics?.DebugDriveForce), "N");
            DrawLegendRow(panel.x + 12f, ref y, LateralCorrectionColor,
                "Lateral correction", GetMagnitude(
                    dynamics?.DebugLateralCorrectionForce), "N");
            DrawLegendRow(panel.x + 12f, ref y, TurnColor,
                "Turn command / yaw axis",
                controller != null
                    ? controller.TurnSpeedDegreesPerSecond
                    : 0f,
                "deg/s");
            DrawLegendRow(panel.x + 12f, ref y, IndexForceColor,
                "Index aligned X/Y",
                controller != null ? controller.IndexForce.magnitude : 0f,
                "N");
            DrawLegendRow(panel.x + 12f, ref y, MiddleForceColor,
                "Middle aligned X/Y",
                controller != null ? controller.MiddleForce.magnitude : 0f,
                "N");
            DrawLegendRow(panel.x + 12f, ref y, TorqueColor,
                "Applied rotation torque", GetMagnitude(
                    dynamics?.DebugRotationTorque), "N·m");
            DrawLegendRow(panel.x + 12f, ref y, FrictionColor,
                "Foot friction force", GetMagnitude(
                    dynamics?.DebugFrictionForce), "N");
            DrawLegendRow(panel.x + 12f, ref y, AdhesionColor,
                "Adhesion / static grip", GetMagnitude(
                    dynamics?.DebugAdhesionForce), "N");
            if (showNumericValues && controller != null)
            {
                y += 3f;
                GUI.Label(
                    new Rect(panel.x + 12f, y,
                        panel.width - 24f, 58f),
                    $"Intent: {controller.InputIntentState}\n" +
                    $"{controller.TurnDecisionReason}\n" +
                    $"I {FormatVector2(controller.IndexForce)}  " +
                    $"M {FormatVector2(controller.MiddleForce)}",
                    legendStyle);
            }
        }

        private void UpdateVectorLines()
        {
            EnsureVectorLines();
            Vector3 normal = GetSupportNormal();
            Vector3 anchor = (physicsBody != null
                ? physicsBody.worldCenterOfMass
                : transform.position) + normal * anchorHeight;

            SetVectorLine(
                0, anchor,
                dynamics != null
                    ? dynamics.DebugDriveForce * forceVectorScale
                    : Vector3.zero,
                DriveColor);
            SetVectorLine(
                1, anchor,
                dynamics != null
                    ? dynamics.DebugLateralCorrectionForce *
                      forceVectorScale
                    : Vector3.zero,
                LateralCorrectionColor);
            float normalizedTurn = controller != null
                ? Mathf.Clamp(
                    controller.TurnSpeedDegreesPerSecond / 30f,
                    -1.5f, 1.5f)
                : 0f;
            SetVectorLine(
                2, anchor,
                normal * (normalizedTurn * turnVectorScale),
                TurnColor);
            SetCurvedTorqueLine(
                3,
                anchor,
                dynamics != null
                    ? dynamics.DebugRotationTorque
                    : Vector3.zero,
                normal,
                TorqueColor);
            SetVectorLine(
                4, anchor,
                dynamics != null
                    ? dynamics.DebugFrictionForce * forceVectorScale
                    : Vector3.zero,
                FrictionColor);
            SetVectorLine(
                5, anchor,
                dynamics != null
                    ? dynamics.DebugAdhesionForce * forceVectorScale
                    : Vector3.zero,
                AdhesionColor);

            Vector3 surfaceRight = Vector3.ProjectOnPlane(
                transform.right, normal).normalized;
            Vector3 surfaceForward = Vector3.ProjectOnPlane(
                transform.forward, normal).normalized;
            if (surfaceRight.sqrMagnitude < 0.001f)
                surfaceRight = transform.right;
            if (surfaceForward.sqrMagnitude < 0.001f)
                surfaceForward = transform.forward;
            Vector2 indexForce = controller != null
                ? controller.IndexForce
                : Vector2.zero;
            Vector2 middleForce = controller != null
                ? controller.MiddleForce
                : Vector2.zero;
            SetVectorLine(
                6, anchor - surfaceRight * rawInputSideOffset,
                (surfaceRight * indexForce.x +
                 surfaceForward * indexForce.y) * inputVectorScale,
                IndexForceColor);
            SetVectorLine(
                7, anchor + surfaceRight * rawInputSideOffset,
                (surfaceRight * middleForce.x +
                 surfaceForward * middleForce.y) * inputVectorScale,
                MiddleForceColor);
        }

        private void EnsureVectorLines()
        {
            if (vectorLineRoot != null)
                return;
            var rootObject = new GameObject("Force Vector Overlay");
            rootObject.hideFlags = HideFlags.DontSave;
            vectorLineRoot = rootObject.transform;
            vectorLineRoot.SetParent(transform, false);

            Shader shader = Shader.Find("Sprites/Default");
            if (shader == null)
                shader = Shader.Find("Universal Render Pipeline/Unlit");
            if (shader != null)
            {
                vectorLineMaterial = new Material(shader)
                {
                    hideFlags = HideFlags.HideAndDontSave
                };
            }

            for (int i = 0; i < vectorLines.Length; i++)
            {
                var lineObject = new GameObject($"Vector {i + 1}");
                lineObject.hideFlags = HideFlags.DontSave;
                lineObject.transform.SetParent(vectorLineRoot, false);
                LineRenderer line = lineObject.AddComponent<LineRenderer>();
                line.useWorldSpace = true;
                line.positionCount = 5;
                line.startWidth = 0.060f;
                line.endWidth = 0.045f;
                line.numCapVertices = 2;
                line.numCornerVertices = 2;
                line.alignment = LineAlignment.View;
                line.shadowCastingMode = ShadowCastingMode.Off;
                line.receiveShadows = false;
                line.sortingOrder = 1000;
                if (vectorLineMaterial != null)
                    line.sharedMaterial = vectorLineMaterial;
                vectorLines[i] = line;
            }
        }

        private void SetVectorLine(
            int index, Vector3 origin, Vector3 vector, Color color)
        {
            LineRenderer line = vectorLines[index];
            if (line == null)
                return;
            line.positionCount = 5;
            vector *= vectorLengthMultiplier;
            vector = Vector3.ClampMagnitude(
                vector, maximumArrowLength);
            float length = vector.magnitude;
            line.enabled = showOverlay && length >= 0.005f;
            if (!line.enabled)
                return;

            Vector3 direction = vector / length;
            Vector3 tip = origin + vector;
            Camera camera = Camera.main;
            Vector3 viewDirection = camera != null
                ? camera.transform.forward
                : Vector3.forward;
            Vector3 side = Vector3.Cross(
                direction, viewDirection).normalized;
            if (side.sqrMagnitude < 0.001f)
                side = Vector3.Cross(direction, Vector3.up).normalized;
            if (side.sqrMagnitude < 0.001f)
                side = Vector3.right;
            float headLength = Mathf.Min(0.24f, length * 0.30f);
            float headWidth = headLength * 0.55f;
            Vector3 headBase = tip - direction * headLength;
            line.startColor = color;
            line.endColor = color;
            line.SetPosition(0, origin);
            line.SetPosition(1, tip);
            line.SetPosition(2, headBase + side * headWidth);
            line.SetPosition(3, tip);
            line.SetPosition(4, headBase - side * headWidth);
        }

        private void SetCurvedTorqueLine(
            int index,
            Vector3 center,
            Vector3 torque,
            Vector3 normal,
            Color color)
        {
            LineRenderer line = vectorLines[index];
            if (line == null)
                return;
            float signedTorque = Vector3.Dot(torque, normal);
            line.enabled = showOverlay &&
                           Mathf.Abs(signedTorque) >= 0.01f;
            if (!line.enabled)
                return;

            Vector3 surfaceRight = Vector3.ProjectOnPlane(
                transform.right, normal).normalized;
            if (surfaceRight.sqrMagnitude < 0.001f)
                surfaceRight = Vector3.Cross(
                    normal, transform.forward).normalized;
            Vector3 surfaceForward = Vector3.Cross(
                normal, surfaceRight).normalized;
            float direction = Mathf.Sign(signedTorque);
            float radius = Mathf.Clamp(
                0.34f + Mathf.Sqrt(Mathf.Abs(signedTorque)) * 0.08f,
                0.34f,
                1.15f);
            const int arcSegments = 22;
            const float arcDegrees = 230f;
            line.positionCount = arcSegments + 4;
            line.startColor = color;
            line.endColor = color;

            float startRadians = -115f * Mathf.Deg2Rad;
            Vector3 end = center;
            Vector3 endTangent = surfaceForward;
            Vector3 endRadial = surfaceRight;
            for (int i = 0; i <= arcSegments; i++)
            {
                float progress = i / (float)arcSegments;
                float angle = startRadians +
                    direction * arcDegrees * Mathf.Deg2Rad * progress;
                Vector3 radial =
                    surfaceRight * Mathf.Cos(angle) +
                    surfaceForward * Mathf.Sin(angle);
                Vector3 tangent = direction *
                    (-surfaceRight * Mathf.Sin(angle) +
                     surfaceForward * Mathf.Cos(angle));
                Vector3 point = center + radial * radius;
                line.SetPosition(i, point);
                if (i == arcSegments)
                {
                    end = point;
                    endTangent = tangent.normalized;
                    endRadial = radial.normalized;
                }
            }

            float headLength = Mathf.Min(0.28f, radius * 0.38f);
            float headWidth = headLength * 0.55f;
            Vector3 headBase = end - endTangent * headLength;
            line.SetPosition(
                arcSegments + 1,
                headBase + endRadial * headWidth);
            line.SetPosition(arcSegments + 2, end);
            line.SetPosition(
                arcSegments + 3,
                headBase - endRadial * headWidth);
        }

        private Vector3 GetSupportNormal()
        {
            Vector3 normal = controller != null
                ? controller.SupportNormal
                : Vector3.up;
            if (normal.sqrMagnitude < 0.001f && dynamics != null)
                normal = dynamics.SurfaceNormal;
            return normal.sqrMagnitude > 0.001f
                ? normal.normalized
                : Vector3.up;
        }

        private void EnsureGuiStyles()
        {
            if (legendStyle != null)
                return;
            legendStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = 11,
                normal = { textColor = Color.white }
            };
            titleStyle = new GUIStyle(legendStyle)
            {
                fontSize = 12,
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleCenter
            };
            buttonStyle = new GUIStyle(GUI.skin.button)
            {
                fontSize = 11,
                fontStyle = FontStyle.Bold
            };
        }

        private void DrawLegendRow(
            float x, ref float y, Color color, string label,
            float value, string units)
        {
            Color previousColor = GUI.color;
            GUI.color = color;
            GUI.DrawTexture(
                new Rect(x, y + 3f, 12f, 12f),
                Texture2D.whiteTexture);
            GUI.color = previousColor;
            string valueText = showNumericValues
                ? $"  {value:0.00} {units}"
                : string.Empty;
            GUI.Label(
                new Rect(x + 18f, y, 240f, 18f),
                label + valueText,
                legendStyle);
            y += 20f;
        }

        private static float GetMagnitude(Vector3? vector)
        {
            return vector.HasValue ? vector.Value.magnitude : 0f;
        }

        private static string FormatVector2(Vector2 value)
        {
            return $"({value.x:0.00}, {value.y:0.00})";
        }

        private void OnDestroy()
        {
            if (vectorLineRoot != null)
                Destroy(vectorLineRoot.gameObject);
            if (vectorLineMaterial != null)
                Destroy(vectorLineMaterial);
        }

        private void OnValidate()
        {
            anchorHeight = Mathf.Max(0f, anchorHeight);
            forceVectorScale = Mathf.Max(0.001f, forceVectorScale);
            inputVectorScale = Mathf.Max(0.01f, inputVectorScale);
            turnVectorScale = Mathf.Max(0.01f, turnVectorScale);
            vectorLengthMultiplier = Mathf.Max(
                0.1f, vectorLengthMultiplier);
            maximumArrowLength = Mathf.Max(0.1f, maximumArrowLength);
            rawInputSideOffset = Mathf.Max(0f, rawInputSideOffset);
        }
    }
}
