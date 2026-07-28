using System.IO;
using UnityEngine;

namespace Dexter.Spider
{
    [DisallowMultipleComponent]
    public sealed class SpiderDebugCapture : MonoBehaviour
    {
        [SerializeField] private Camera captureCamera;
        [SerializeField, Min(0.25f)] private float captureIntervalSeconds = 1f;
        [SerializeField] private bool captureScreenshots = true;
        [SerializeField] private bool drawGroundMarkers = true;
        private float nextCaptureTime;
        private string outputDirectory;

        private void OnEnable()
        {
            outputDirectory = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Logs"));
            Directory.CreateDirectory(outputDirectory);
            nextCaptureTime = Time.realtimeSinceStartup + captureIntervalSeconds;
            Debug.Log($"Spider debug captures: {outputDirectory}", this);
        }

        private void LateUpdate()
        {
            if (!captureScreenshots || captureCamera == null ||
                Time.realtimeSinceStartup < nextCaptureTime)
                return;
            nextCaptureTime = Time.realtimeSinceStartup + captureIntervalSeconds;
            string path = Path.Combine(outputDirectory,
                $"SpiderDebug_{System.DateTime.Now:yyyyMMdd_HHmmss_fff}.png");
            ScreenCapture.CaptureScreenshot(path, 1);
        }

        private void OnDrawGizmos()
        {
            if (!drawGroundMarkers) return;
            Gizmos.color = Color.red;
            foreach (Transform t in GetComponentsInChildren<Transform>(true))
            {
                string n = t.name.ToLowerInvariant();
                if (n.Contains("foot") || n.Contains("toe"))
                    Gizmos.DrawWireSphere(t.position, 0.06f);
            }
        }
    }
}
