using Dexter.Spider;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Dexter.Visualize
{
    /// <summary>
    /// Draws the latest five two-axis finger forces as a responsive 2D overlay. For fingers that
    /// actually drive gameplay (the leg IK's Index/Middle, the ring-finger web aimer's Ring), this
    /// shows the real post-processing signal those systems act on (baseline-subtracted, and for legs
    /// also temporally smoothed) rather than the raw relay value, so what you see matches what the
    /// spider receives. Thumb/Pinky (and Ring with no aimer wired in) have no gameplay pipeline to
    /// mirror, so they fall back to raw force with this overlay's own optional tare.
    /// </summary>
    [RequireComponent(typeof(DexterRelayUdpReceiver))]
    public sealed class DexterForceOverlay : MonoBehaviour
    {
        public enum DisplayMode
        {
            FullScreen,
            EmbeddedPanel
        }

        private const int FingerCount = 5;
        private const int TareSampleCount = 20;

        private static readonly string[] FingerNames = { "Thumb", "Index", "Middle", "Ring", "Pinky" };
        private static readonly DexterFinger[] Fingers =
        {
            DexterFinger.Thumb,
            DexterFinger.Index,
            DexterFinger.Middle,
            DexterFinger.Ring,
            DexterFinger.Pinky
        };

        private static readonly Vector2[] HandLayout =
        {
            new Vector2(0.13f, 0.73f),
            new Vector2(0.32f, 0.32f),
            new Vector2(0.49f, 0.23f),
            new Vector2(0.66f, 0.28f),
            new Vector2(0.83f, 0.39f)
        };

        private static readonly Color[] FingerColors =
        {
            new Color32(232, 73, 73, 255),
            new Color32(239, 145, 36, 255),
            new Color32(65, 190, 91, 255),
            new Color32(69, 117, 236, 255),
            new Color32(177, 72, 218, 255)
        };

        [Tooltip("Master switch for this overlay. Uncheck to hide it completely (equivalent to disabling the component, but easier to find/toggle than the component checkbox in the Inspector header). Also toggled in-game by the 'Toggle Key' below.")]
        [SerializeField] private bool showOverlay = true;
        [Tooltip("Key that toggles 'Show Overlay' on/off at runtime.")]
        [SerializeField] private Key toggleKey = Key.Q;
        [SerializeField] private DexterRelayUdpReceiver receiver;
        [SerializeField] private DexterFrontLegIK frontLegIk;
        [SerializeField] private RingFingerShotAimer ringFingerAimer;
        [SerializeField] private DisplayMode displayMode = DisplayMode.FullScreen;
        [SerializeField, Min(1f)] private float pixelsPerNewton = 55f;
        [SerializeField, Min(20f)] private float maximumArrowPixels = 175f;
        [Tooltip("Opacity the embedded panel fades to (instead of disappearing) once the calibration HUD is no longer showing, so finger input stays visible for reference.")]
        [SerializeField, Range(0f, 1f)] private float postCalibrationAlpha = 0.35f;

        private float currentAlphaMultiplier = 1f;

        private readonly Vector2[] baseline = new Vector2[FingerCount];
        private readonly Vector2[] baselineSum = new Vector2[FingerCount];
        private readonly int[] baselineCounts = new int[FingerCount];
        private int tareSamplesRemaining;
        private long lastTareSequence = long.MinValue;
        private bool hasBaseline;

        private Texture2D circleTexture;
        private GUIStyle titleStyle;
        private GUIStyle compactTitleStyle;
        private GUIStyle statusStyle;
        private GUIStyle compactStatusStyle;
        private GUIStyle fingerNameStyle;
        private GUIStyle compactFingerNameStyle;
        private GUIStyle valueStyle;

        private void Awake()
        {
            if (receiver == null)
                receiver = GetComponent<DexterRelayUdpReceiver>();
            if (frontLegIk == null)
                frontLegIk = GetComponent<DexterFrontLegIK>();
            if (ringFingerAimer == null)
                ringFingerAimer = GetComponent<RingFingerShotAimer>();
            circleTexture = CreateCircleTexture(32);
        }

        private void OnDestroy()
        {
            if (circleTexture != null)
                Destroy(circleTexture);
        }

        private void Update()
        {
            CaptureTareSample();
            HandleToggleKey();
        }

        private void HandleToggleKey()
        {
            Keyboard keyboard = Keyboard.current;
            if (keyboard == null)
                return;

            if (keyboard[toggleKey].wasPressedThisFrame)
                showOverlay = !showOverlay;
        }

        private void OnGUI()
        {
            if (!showOverlay)
                return;

            EnsureStyles();

            if (displayMode == DisplayMode.EmbeddedPanel)
            {
                currentAlphaMultiplier = ResolvePanelAlpha();
                DrawEmbeddedPanel(DexterSpiderHudLayout.ForcePanel);
                return;
            }

            currentAlphaMultiplier = 1f;
            DrawRect(new Rect(0f, 0f, Screen.width, Screen.height), new Color(0.055f, 0.065f, 0.085f, 1f));

            float width = Mathf.Min(820f, Screen.width - 32f);
            float height = Mathf.Min(650f, Screen.height - 32f);
            var panel = new Rect((Screen.width - width) * 0.5f, (Screen.height - height) * 0.5f, width, height);
            DrawPanel(panel, showFooter: true, compact: false);
        }

        /// <summary>
        /// Full opacity while the calibration HUD is showing (matches its previous always-visible
        /// behavior); fades to <see cref="postCalibrationAlpha"/> afterward instead of disappearing,
        /// so live finger input stays visible for reference. Always full opacity if there's no
        /// leg IK reference to check (no calibration concept to gate on).
        /// </summary>
        private float ResolvePanelAlpha()
        {
            if (frontLegIk == null)
                return 1f;

            return frontLegIk.ShouldShowCalibrationHud ? 1f : postCalibrationAlpha;
        }

        public void BeginTare()
        {
            for (int i = 0; i < FingerCount; i++)
            {
                baselineSum[i] = Vector2.zero;
                baselineCounts[i] = 0;
            }

            tareSamplesRemaining = TareSampleCount;
            lastTareSequence = long.MinValue;
        }

        private void DrawEmbeddedPanel(Rect panel)
        {
            DrawRect(panel, new Color(0.09f, 0.105f, 0.135f, 0.94f));
            DrawOutline(panel, new Color(0.25f, 0.29f, 0.37f, 1f), 1f);
            DrawLabel(
                new Rect(panel.x + 12f, panel.y + 6f, panel.width - 84f, 20f),
                "DEXTER FORCES",
                compactTitleStyle);

            if (DrawButton(new Rect(panel.xMax - 68f, panel.y + 4f, 56f, 22f),
                    tareSamplesRemaining > 0 ? "Taring…" : "Tare"))
                BeginTare();

            var statusPanel = new Rect(panel.x, panel.y + 24f, panel.width, 18f);
            DrawStatus(statusPanel, compact: true);

            var plot = new Rect(panel.x + 10f, panel.y + 44f, panel.width - 20f, panel.height - 52f);
            DrawForcePlot(plot, compact: true);
        }

        private void DrawPanel(Rect panel, bool showFooter, bool compact)
        {
            DrawRect(panel, new Color(0.09f, 0.105f, 0.135f, 0.98f));
            DrawOutline(panel, new Color(0.25f, 0.29f, 0.37f, 1f), 1f);

            GUIStyle headerStyle = compact ? compactTitleStyle : titleStyle;
            DrawLabel(
                new Rect(panel.x + 24f, panel.y + 18f, panel.width - 180f, 34f),
                "DEXTER FORCE VISUALIZER",
                headerStyle);

            if (DrawButton(
                    new Rect(panel.xMax - 118f, panel.y + 17f, 94f, 30f),
                    tareSamplesRemaining > 0 ? "Taring…" : "Tare"))
                BeginTare();

            var statusPanel = compact
                ? new Rect(panel.x + 12f, panel.y + 42f, panel.width - 24f, 18f)
                : new Rect(panel.x, panel.y, panel.width, panel.height);
            DrawStatus(statusPanel, compact);

            float plotTop = compact ? panel.y + 64f : panel.y + 92f;
            float plotBottomPad = compact ? 12f : 136f;
            var plot = new Rect(
                panel.x + (compact ? 10f : 28f),
                plotTop,
                panel.width - (compact ? 20f : 56f),
                panel.height - plotBottomPad);
            DrawForcePlot(plot, compact);

            if (showFooter)
            {
                DrawLabel(
                    new Rect(panel.x + 24f, panel.yMax - 35f, panel.width - 48f, 22f),
                    "Arrow direction = Fx / Fy    •    Arrow length = force magnitude    •    Units: N",
                    statusStyle);
            }
        }

        private void DrawForcePlot(Rect plot, bool compact)
        {
            DrawRect(plot, new Color(0.045f, 0.052f, 0.068f, 1f));
            DrawOutline(plot, new Color(0.16f, 0.19f, 0.24f, 1f), 1f);

            Vector2 palm = new Vector2(plot.x + plot.width * 0.49f, plot.y + plot.height * 0.76f);
            float connectorWidth = compact ? 2f : 3f;
            for (int i = 0; i < FingerCount; i++)
            {
                Vector2 origin = PlotPoint(plot, HandLayout[i]);
                DrawLine(palm, origin, new Color(0.24f, 0.27f, 0.34f, 0.75f), connectorWidth);
            }

            for (int i = 0; i < FingerCount; i++)
                DrawFinger(plot, i, compact);
        }

        private void CaptureTareSample()
        {
            DexterForceFrame frame = receiver != null ? receiver.LatestFrame : null;
            if (tareSamplesRemaining <= 0 || frame == null || frame.sequence == lastTareSequence)
                return;

            lastTareSequence = frame.sequence;
            for (int i = 0; i < FingerCount; i++)
            {
                if (!TryGetForce(i, out Vector2 force))
                    continue;
                baselineSum[i] += force;
                baselineCounts[i]++;
            }

            tareSamplesRemaining--;
            if (tareSamplesRemaining > 0)
                return;

            for (int i = 0; i < FingerCount; i++)
            {
                if (baselineCounts[i] > 0)
                    baseline[i] = baselineSum[i] / baselineCounts[i];
            }
            hasBaseline = true;
        }

        private void DrawStatus(Rect panel, bool compact)
        {
            string text;
            Color color;

            if (receiver == null)
            {
                text = "Receiver component missing";
                color = new Color32(239, 95, 95, 255);
            }
            else if (!string.IsNullOrEmpty(receiver.LastError))
            {
                text = $"Relay {receiver.ServerLabel}  •  {receiver.LastError}";
                color = new Color32(239, 95, 95, 255);
            }
            else if (receiver.LatestFrame == null)
            {
                text = $"Waiting for relay at {receiver.ServerLabel}…";
                color = new Color32(241, 181, 74, 255);
            }
            else
            {
                string state = receiver.HasRecentFrame ? "LIVE" : "STALE";
                text = compact
                    ? $"{state}  •  {receiver.LatestFrame.transport}  •  seq {receiver.LatestFrame.sequence}"
                    : $"{state}  •  {receiver.ServerLabel}  •  {receiver.LatestFrame.transport}  •  seq {receiver.LatestFrame.sequence}";
                color = receiver.HasRecentFrame
                    ? new Color32(82, 210, 126, 255)
                    : new Color32(241, 181, 74, 255);
            }

            GUIStyle style = compact ? compactStatusStyle : statusStyle;
            style.normal.textColor = color;
            float x = compact ? panel.x + 12f : panel.x + 25f;
            float y = compact ? panel.y : panel.y + 54f;
            float width = compact ? panel.width - 24f : panel.width - 50f;
            DrawLabel(new Rect(x, y, width, compact ? 18f : 24f), text, style);
        }

        private void DrawFinger(Rect plot, int index, bool compact)
        {
            Vector2 origin = PlotPoint(plot, HandLayout[index]);
            bool hasForce = TryGetDisplayForce(index, out Vector2 force, out bool isProcessed);

            float scale = compact ? 0.55f : 1f;
            float magnitude = hasForce ? force.magnitude : 0f;
            Vector2 arrow = new Vector2(force.x, -force.y) * pixelsPerNewton * scale;
            float maxArrow = maximumArrowPixels * scale;
            if (arrow.magnitude > maxArrow)
                arrow = arrow.normalized * maxArrow;

            Color color = FingerColors[index];
            if (receiver == null || !receiver.HasRecentFrame || !hasForce)
                color.a = 0.45f;

            float nodeSize = 16f * scale;
            float coreSize = 7f * scale;
            DrawCircle(origin, nodeSize, color);
            DrawCircle(origin, coreSize, Color.white);

            if (arrow.sqrMagnitude >= 1f)
                DrawArrow(origin, origin + arrow, color, 4f * scale);

            if (compact)
                return;

            fingerNameStyle.normal.textColor = color;
            DrawLabel(new Rect(origin.x - 65f, origin.y + 15f, 130f, 22f), FingerNames[index], fingerNameStyle);
            string valueText = hasForce
                ? $"{magnitude:F2} N  ({force.x:F2}, {force.y:F2}){(isProcessed ? "  •  live" : string.Empty)}"
                : "waiting";
            DrawLabel(
                new Rect(origin.x - 75f, origin.y + 36f, 150f, 22f),
                valueText,
                valueStyle);
        }

        /// <summary>
        /// Resolves the value to display for a given finger. For Index/Middle (whichever are actually
        /// assigned as the leg IK's driving fingers) and Ring (when a <see cref="RingFingerShotAimer"/>
        /// is wired in), this is the real post-baseline signal that gameplay acts on ("live"). Every
        /// other finger has no gameplay pipeline, so it falls back to raw force with this overlay's
        /// own optional tare.
        /// </summary>
        private bool TryGetDisplayForce(int index, out Vector2 force, out bool isProcessed)
        {
            DexterFinger finger = Fingers[index];
            isProcessed = true;

            if (frontLegIk != null && finger == frontLegIk.LeftLegFinger)
            {
                force = frontLegIk.ProcessedLeftForce;
                return frontLegIk.IsReceiving;
            }

            if (frontLegIk != null && finger == frontLegIk.RightLegFinger)
            {
                force = frontLegIk.ProcessedRightForce;
                return frontLegIk.IsReceiving;
            }

            if (finger == DexterFinger.Ring && ringFingerAimer != null)
            {
                force = ringFingerAimer.ProcessedForce;
                return ringFingerAimer.HasBaseline && receiver != null && receiver.HasRecentFrame;
            }

            isProcessed = false;
            bool hasForce = TryGetForce(index, out force);
            if (hasForce && hasBaseline)
                force -= baseline[index];
            return hasForce;
        }

        private bool TryGetForce(int index, out Vector2 force)
        {
            force = Vector2.zero;
            DexterFingerMeasurement measurement = receiver != null ? receiver.GetFinger(Fingers[index]) : null;
            if (measurement == null || !measurement.has_data || measurement.force == null || measurement.force.Length < 2)
                return false;

            force = new Vector2(measurement.force[0], measurement.force[1]);
            return true;
        }

        private static Vector2 PlotPoint(Rect plot, Vector2 normalized)
        {
            return new Vector2(plot.x + plot.width * normalized.x, plot.y + plot.height * normalized.y);
        }

        private void EnsureStyles()
        {
            if (titleStyle != null)
                return;

            titleStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = 20,
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleLeft
            };
            titleStyle.normal.textColor = new Color32(225, 231, 242, 255);

            compactTitleStyle = new GUIStyle(titleStyle)
            {
                fontSize = 12,
                alignment = TextAnchor.MiddleLeft
            };

            statusStyle = new GUIStyle(GUI.skin.label) { fontSize = 12, alignment = TextAnchor.MiddleLeft };
            compactStatusStyle = new GUIStyle(statusStyle) { fontSize = 10 };

            fingerNameStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = 14,
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.UpperCenter
            };
            compactFingerNameStyle = new GUIStyle(fingerNameStyle) { fontSize = 10 };

            valueStyle = new GUIStyle(GUI.skin.label) { fontSize = 11, alignment = TextAnchor.UpperCenter };
            valueStyle.normal.textColor = new Color32(188, 197, 214, 255);
        }

        private static Texture2D CreateCircleTexture(int size)
        {
            var texture = new Texture2D(size, size, TextureFormat.RGBA32, false)
            {
                name = "Dexter Overlay Circle",
                hideFlags = HideFlags.HideAndDontSave,
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp
            };

            var pixels = new Color[size * size];
            float center = (size - 1) * 0.5f;
            float radius = size * 0.48f;
            for (int y = 0; y < size; y++)
            for (int x = 0; x < size; x++)
            {
                float distance = Vector2.Distance(new Vector2(x, y), new Vector2(center, center));
                pixels[y * size + x] = new Color(1f, 1f, 1f, Mathf.Clamp01(radius - distance + 0.8f));
            }
            texture.SetPixels(pixels);
            texture.Apply(false, true);
            return texture;
        }

        private void DrawArrow(Vector2 start, Vector2 end, Color color, float width)
        {
            DrawLine(start, end, color, width);
            Vector2 direction = (end - start).normalized;
            float headLength = Mathf.Min(15f, Vector2.Distance(start, end) * 0.45f);
            Vector2 perpendicular = new Vector2(-direction.y, direction.x);
            Vector2 headBase = end - direction * headLength;
            DrawLine(end, headBase + perpendicular * headLength * 0.55f, color, width);
            DrawLine(end, headBase - perpendicular * headLength * 0.55f, color, width);
        }

        private void DrawLine(Vector2 start, Vector2 end, Color color, float width)
        {
            Vector2 delta = end - start;
            float length = delta.magnitude;
            if (length <= 0.01f)
                return;

            color.a *= currentAlphaMultiplier;

            Matrix4x4 previousMatrix = GUI.matrix;
            Color previousColor = GUI.color;
            GUI.color = color;
            GUIUtility.RotateAroundPivot(Mathf.Atan2(delta.y, delta.x) * Mathf.Rad2Deg, start);
            GUI.DrawTexture(new Rect(start.x, start.y - width * 0.5f, length, width), Texture2D.whiteTexture);
            GUI.matrix = previousMatrix;
            GUI.color = previousColor;
        }

        private void DrawRect(Rect rect, Color color)
        {
            color.a *= currentAlphaMultiplier;

            Color previous = GUI.color;
            GUI.color = color;
            GUI.DrawTexture(rect, Texture2D.whiteTexture);
            GUI.color = previous;
        }

        private void DrawOutline(Rect rect, Color color, float width)
        {
            DrawRect(new Rect(rect.x, rect.y, rect.width, width), color);
            DrawRect(new Rect(rect.x, rect.yMax - width, rect.width, width), color);
            DrawRect(new Rect(rect.x, rect.y, width, rect.height), color);
            DrawRect(new Rect(rect.xMax - width, rect.y, width, rect.height), color);
        }

        private void DrawCircle(Vector2 center, float diameter, Color color)
        {
            color.a *= currentAlphaMultiplier;

            Color previous = GUI.color;
            GUI.color = color;
            GUI.DrawTexture(new Rect(center.x - diameter * 0.5f, center.y - diameter * 0.5f, diameter, diameter), circleTexture);
            GUI.color = previous;
        }

        /// <summary>
        /// Wraps a Label/Button draw with an ambient GUI.color alpha tint. Unlike the shape helpers
        /// above (which set an explicit opaque color and so must bake the alpha in directly), Unity's
        /// GUIStyle-based Label/Button content is tinted multiplicatively by whatever GUI.color is
        /// ambient at draw time, so this is the correct way to fade text/buttons.
        /// </summary>
        private void DrawLabel(Rect rect, string text, GUIStyle style)
        {
            Color previous = GUI.color;
            GUI.color = new Color(1f, 1f, 1f, currentAlphaMultiplier);
            GUI.Label(rect, text, style);
            GUI.color = previous;
        }

        private bool DrawButton(Rect rect, string text)
        {
            Color previous = GUI.color;
            GUI.color = new Color(1f, 1f, 1f, currentAlphaMultiplier);
            bool clicked = GUI.Button(rect, text);
            GUI.color = previous;
            return clicked;
        }
    }
}
