using UnityEngine;

namespace Dexter.Visualize
{
    /// <summary>Shared screen-space layout for spider calibration and force HUD panels.</summary>
    public static class DexterSpiderHudLayout
    {
        public const float PanelWidth = 460f;
        public const float CalibrationPanelHeight = 92f;
        public const float ForcePanelHeight = 168f;
        public const float PanelGap = 8f;

        public static float TopY => Mathf.Max(14f, Screen.height * 0.08f);

        public static Rect CalibrationPanel
        {
            get
            {
                float width = Mathf.Min(PanelWidth, Screen.width - 32f);
                return new Rect(
                    (Screen.width - width) * 0.5f,
                    TopY,
                    width,
                    CalibrationPanelHeight);
            }
        }

        public static Rect ForcePanel
        {
            get
            {
                Rect calibration = CalibrationPanel;
                return new Rect(
                    calibration.x,
                    calibration.yMax + PanelGap,
                    calibration.width,
                    ForcePanelHeight);
            }
        }
    }
}
