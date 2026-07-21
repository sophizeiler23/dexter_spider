using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using System.IO;
using System.Reflection;
using Dexter.Visualize;

namespace Dexter.Spider.EditorTools
{
    [InitializeOnLoad]
    public static class SpiderPlayModeCapture
    {
        private static double stopAt = -1;
        private static double nextCommandCheck;
        private static bool captureTimerRunning;
        private static bool forwardWalkRunning;
        private static double forwardWalkStartedAt;
        private static double syntheticWalkDuration = 8.5;
        private static double syntheticWalkStart = 1.0;
        private static double syntheticInputEnd = 7.5;
        private static float syntheticWalkForce = 8f;
        private static double syntheticTurnStart;
        private static double syntheticTurnEnd;
        private static float syntheticTurnForce;
        private static long syntheticSequence;
        private static DexterRelayUdpReceiver syntheticReceiver;
        private static readonly PropertyInfo LatestFrameProperty =
            typeof(DexterRelayUdpReceiver).GetProperty(
                "LatestFrame", BindingFlags.Instance | BindingFlags.Public);
        private static readonly FieldInfo LastFrameRealtimeField =
            typeof(DexterRelayUdpReceiver).GetField(
                "lastFrameRealtime", BindingFlags.Instance | BindingFlags.NonPublic);
        private static string commandPath => Path.Combine(Application.dataPath, "..", "Logs", "spider_editor_command.txt");
        private const string SpiderScenePath = "Assets/Scenes/SpiderBabyStepsScene.unity";
        private const string CapturePendingKey = "Dexter.Spider.CapturePending";
        private const string CaptureDurationKey = "Dexter.Spider.CaptureDuration";
        private const string OpenCapturePendingKey = "Dexter.Spider.OpenCapturePending";
        private const string ForwardWalkPendingKey = "Dexter.Spider.ForwardWalkPending";
        private const string ForwardWalkDurationKey = "Dexter.Spider.ForwardWalkDuration";
        private const string ForwardWalkStartKey = "Dexter.Spider.ForwardWalkStart";
        private const string ForwardWalkInputEndKey = "Dexter.Spider.ForwardWalkInputEnd";
        private const string ForwardWalkForceKey = "Dexter.Spider.ForwardWalkForce";
        private const string ForwardTurnStartKey = "Dexter.Spider.ForwardTurnStart";
        private const string ForwardTurnEndKey = "Dexter.Spider.ForwardTurnEnd";
        private const string ForwardTurnForceKey = "Dexter.Spider.ForwardTurnForce";
        private const string WallAscentTeleportKey = "Dexter.Spider.WallAscentTeleport";
        private const string ConcaveCornerTeleportKey =
            "Dexter.Spider.ConcaveCornerTeleport";
        private const string SyntheticReceiverDisabledKey =
            "Dexter.Spider.SyntheticReceiverDisabled";

        static SpiderPlayModeCapture()
        {
            EditorApplication.update += Update;
            EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
        }

        [MenuItem("Spider/Play Mode/Capture 5 Seconds")]
        public static void CaptureFiveSeconds()
        {
            SessionState.SetFloat(CaptureDurationKey, 5f);
            SessionState.SetBool(CapturePendingKey, true);
            stopAt = -1;
            captureTimerRunning = false;
            EditorApplication.isPlaying = true;
            Debug.Log("Spider capture started; Play Mode will stop after 5 seconds.");
        }

        [MenuItem("Spider/Play Mode/Capture 12 Seconds")]
        public static void CaptureTwelveSeconds()
        {
            SessionState.SetFloat(CaptureDurationKey, 12f);
            SessionState.SetBool(CapturePendingKey, true);
            stopAt = -1;
            captureTimerRunning = false;
            EditorApplication.isPlaying = true;
            Debug.Log("Spider capture started; Play Mode will stop after 12 seconds.");
        }

        [MenuItem("Spider/Play Mode/Stop")]
        public static void Stop()
        {
            SessionState.SetBool(CapturePendingKey, false);
            SessionState.SetBool(ForwardWalkPendingKey, false);
            SessionState.SetBool(WallAscentTeleportKey, false);
            SessionState.SetBool(ConcaveCornerTeleportKey, false);
            stopAt = -1;
            captureTimerRunning = false;
            forwardWalkRunning = false;
            RestorePhysicalReceiverAfterSyntheticTest();
            EditorApplication.isPlaying = false;
        }

        [MenuItem("Spider/Play Mode/Test Forward Walk 8 Seconds")]
        public static void TestForwardWalkEightSeconds()
        {
            syntheticWalkDuration = 8.5;
            syntheticWalkStart = 1.0;
            syntheticInputEnd = 7.5;
            syntheticWalkForce = 8f;
            ConfigureSyntheticTest(8.5f, 1f, 7.5f, 8f, 0f, 0f, 0f);
            SessionState.SetBool(WallAscentTeleportKey, false);
            SessionState.SetBool(CapturePendingKey, false);
            SessionState.SetBool(ForwardWalkPendingKey, true);
            stopAt = -1;
            captureTimerRunning = false;
            forwardWalkRunning = false;
            EditorApplication.isPlaying = true;
            Debug.Log("Synthetic forward-walk capture requested.");
        }

        [MenuItem("Spider/Play Mode/Test Forward Walk 20 Seconds")]
        public static void TestForwardWalkTwentySeconds()
        {
            syntheticWalkDuration = 20.5;
            syntheticWalkStart = 1.0;
            syntheticInputEnd = 19.5;
            syntheticWalkForce = 25f;
            ConfigureSyntheticTest(20.5f, 1f, 19.5f, 25f, 0f, 0f, 0f);
            SessionState.SetBool(WallAscentTeleportKey, false);
            RequestSyntheticPlay();
            Debug.Log("Extended synthetic forward-walk capture requested.");
        }

        [MenuItem("Spider/Play Mode/Test Trough Descent 46 Seconds")]
        public static void TestTroughDescent()
        {
            // Matches the heading and travel distance of the user trace that
            // reaches the steep trough rim: turn about -138 degrees, then walk.
            syntheticWalkDuration = 46f;
            syntheticWalkStart = 4.2f;
            syntheticInputEnd = 44.5f;
            syntheticWalkForce = 25f;
            syntheticTurnStart = 0.5f;
            syntheticTurnEnd = 3.82f;
            syntheticTurnForce = -25f;
            ConfigureSyntheticTest(
                46f, 4.2f, 44.5f, 25f, 0.5f, 3.82f, -25f);
            SessionState.SetBool(WallAscentTeleportKey, false);
            RequestSyntheticPlay();
            Debug.Log("Synthetic steep-trough descent capture requested.");
        }

        [MenuItem("Spider/Play Mode/Test Full Trough Traverse 75 Seconds")]
        public static void TestFullTroughTraverse()
        {
            syntheticWalkDuration = 75f;
            syntheticWalkStart = 4.2f;
            syntheticInputEnd = 73.5f;
            syntheticWalkForce = 25f;
            syntheticTurnStart = 0.5f;
            syntheticTurnEnd = 3.82f;
            syntheticTurnForce = -25f;
            ConfigureSyntheticTest(
                75f, 4.2f, 73.5f, 25f, 0.5f, 3.82f, -25f);
            SessionState.SetBool(WallAscentTeleportKey, false);
            RequestSyntheticPlay();
            Debug.Log("Synthetic full-trough traverse capture requested.");
        }

        [MenuItem("Spider/Play Mode/Test Trough Wall Ascent 80 Seconds")]
        public static void TestTroughWallAscent()
        {
            syntheticWalkDuration = 80f;
            syntheticWalkStart = 1f;
            syntheticInputEnd = 78.5f;
            syntheticWalkForce = 25f;
            syntheticTurnStart = 0f;
            syntheticTurnEnd = 0f;
            syntheticTurnForce = 0f;
            ConfigureSyntheticTest(80f, 1f, 78.5f, 25f, 0f, 0f, 0f);
            SessionState.SetBool(WallAscentTeleportKey, true);
            RequestSyntheticPlay();
            Debug.Log("Synthetic trough-wall ascent capture requested.");
        }

        [MenuItem("Spider/Play Mode/Test Trough Wall Ascent 25 Seconds")]
        public static void TestTroughWallAscentTwentyFiveSeconds()
        {
            ConfigureSyntheticTest(25f, 1f, 23.5f, 25f, 0f, 0f, 0f);
            SessionState.SetBool(WallAscentTeleportKey, true);
            RequestSyntheticPlay();
            Debug.Log("Short synthetic trough-wall ascent capture requested.");
        }

        [MenuItem("Spider/Play Mode/Test Concave Corner Approach 15 Seconds")]
        public static void TestConcaveCornerApproach()
        {
            ConfigureSyntheticTest(15f, 1f, 13.5f, 25f, 0f, 0f, 0f);
            SessionState.SetBool(WallAscentTeleportKey, false);
            SessionState.SetBool(ConcaveCornerTeleportKey, true);
            RequestSyntheticPlay();
            Debug.Log("Synthetic concave-corner regression capture requested.");
        }

        private static void ConfigureSyntheticTest(
            float duration,
            float walkStart,
            float inputEnd,
            float walkForce,
            float turnStart,
            float turnEnd,
            float turnForce)
        {
            SessionState.SetFloat(ForwardWalkDurationKey, duration);
            SessionState.SetFloat(ForwardWalkStartKey, walkStart);
            SessionState.SetFloat(ForwardWalkInputEndKey, inputEnd);
            SessionState.SetFloat(ForwardWalkForceKey, walkForce);
            SessionState.SetFloat(ForwardTurnStartKey, turnStart);
            SessionState.SetFloat(ForwardTurnEndKey, turnEnd);
            SessionState.SetFloat(ForwardTurnForceKey, turnForce);
        }

        private static void RequestSyntheticPlay()
        {
            SessionState.SetBool(CapturePendingKey, false);
            SessionState.SetBool(ForwardWalkPendingKey, true);
            stopAt = -1;
            captureTimerRunning = false;
            forwardWalkRunning = false;
            EditorApplication.isPlaying = true;
        }

        [MenuItem("Spider/Play Mode/Open Spider Scene And Capture")]
        public static void OpenSpiderSceneAndCapture()
        {
            if (EditorApplication.isPlaying)
            {
                SessionState.SetBool(OpenCapturePendingKey, true);
                EditorApplication.isPlaying = false;
                return;
            }

            SessionState.SetBool(OpenCapturePendingKey, false);
            OpenSpiderSceneAndCaptureFromEditMode();
        }

        private static void OpenSpiderSceneAndCaptureFromEditMode()
        {
            EditorSceneManager.SaveOpenScenes();
            AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);
            EditorSceneManager.OpenScene(SpiderScenePath, OpenSceneMode.Single);
            EditorApplication.delayCall += CaptureFiveSeconds;
        }

        private static void Update()
        {
            if (EditorApplication.timeSinceStartup >= nextCommandCheck)
            {
                nextCommandCheck = EditorApplication.timeSinceStartup + 0.25;
                ProcessExternalCommand();
            }
            if (SessionState.GetBool(CapturePendingKey, false) &&
                EditorApplication.isPlaying && !captureTimerRunning)
            {
                stopAt = EditorApplication.timeSinceStartup +
                    SessionState.GetFloat(CaptureDurationKey, 5f);
                captureTimerRunning = true;
            }
            if (SessionState.GetBool(ForwardWalkPendingKey, false) &&
                EditorApplication.isPlaying && !forwardWalkRunning)
            {
                BeginSyntheticForwardWalk();
            }
            if (forwardWalkRunning && EditorApplication.isPlaying)
                UpdateSyntheticForwardWalk();
            if (stopAt < 0 || !EditorApplication.isPlaying)
                return;
            if (EditorApplication.timeSinceStartup >= stopAt)
            {
                SessionState.SetBool(CapturePendingKey, false);
                SessionState.SetBool(ForwardWalkPendingKey, false);
                stopAt = -1;
                captureTimerRunning = false;
                forwardWalkRunning = false;
                RestorePhysicalReceiverAfterSyntheticTest();
                EditorApplication.isPlaying = false;
                Debug.Log("Spider capture stopped; screenshots and CSV were saved.");
            }
        }

        private static void OnPlayModeStateChanged(PlayModeStateChange state)
        {
            if (state == PlayModeStateChange.EnteredEditMode)
            {
                bool openCaptureRequested = SessionState.GetBool(
                    OpenCapturePendingKey, false);
                // A manual stop or script reload must never leave an old
                // synthetic walk queued for the user's next normal Play run.
                SessionState.SetBool(CapturePendingKey, false);
                SessionState.SetBool(ForwardWalkPendingKey, false);
                SessionState.SetBool(WallAscentTeleportKey, false);
                SessionState.SetBool(ConcaveCornerTeleportKey, false);
                forwardWalkRunning = false;
                RestorePhysicalReceiverAfterSyntheticTest();
                if (openCaptureRequested)
                {
                    SessionState.SetBool(OpenCapturePendingKey, false);
                    EditorApplication.delayCall +=
                        OpenSpiderSceneAndCaptureFromEditMode;
                }
                return;
            }

            if (state == PlayModeStateChange.EnteredPlayMode &&
                SessionState.GetBool(ForwardWalkPendingKey, false))
            {
                BeginSyntheticForwardWalk();
                return;
            }

            if (state == PlayModeStateChange.EnteredPlayMode &&
                SessionState.GetBool(CapturePendingKey, false))
            {
                // Scene loading and domain reloads can take several seconds;
                // start the requested capture duration only once Play Mode is live.
                stopAt = EditorApplication.timeSinceStartup +
                    SessionState.GetFloat(CaptureDurationKey, 5f);
                captureTimerRunning = true;
                return;
            }

            if (state == PlayModeStateChange.EnteredPlayMode)
                RestorePhysicalReceiverAfterSyntheticTest();
        }

        private static void BeginSyntheticForwardWalk()
        {
            syntheticWalkDuration = SessionState.GetFloat(
                ForwardWalkDurationKey, 8.5f);
            syntheticWalkStart = SessionState.GetFloat(
                ForwardWalkStartKey, 1f);
            syntheticInputEnd = SessionState.GetFloat(
                ForwardWalkInputEndKey, 7.5f);
            syntheticWalkForce = SessionState.GetFloat(
                ForwardWalkForceKey, 8f);
            syntheticTurnStart = SessionState.GetFloat(
                ForwardTurnStartKey, 0f);
            syntheticTurnEnd = SessionState.GetFloat(
                ForwardTurnEndKey, 0f);
            syntheticTurnForce = SessionState.GetFloat(
                ForwardTurnForceKey, 0f);
            forwardWalkStartedAt = EditorApplication.timeSinceStartup;
            stopAt = forwardWalkStartedAt + syntheticWalkDuration;
            captureTimerRunning = true;
            forwardWalkRunning = true;
            syntheticSequence = 0;
            syntheticReceiver = Object.FindAnyObjectByType<DexterRelayUdpReceiver>();
            if (syntheticReceiver != null)
            {
                SessionState.SetBool(SyntheticReceiverDisabledKey, true);
                syntheticReceiver.enabled = false;
            }
            if (SessionState.GetBool(WallAscentTeleportKey, false))
                PrepareWallAscentStart();
            else if (SessionState.GetBool(ConcaveCornerTeleportKey, false))
                PrepareConcaveCornerStart();
        }

        private static void PrepareWallAscentStart()
        {
            DexterFrontLegIK spider = Object.FindAnyObjectByType<DexterFrontLegIK>();
            Terrain terrain = Terrain.activeTerrain ?? Object.FindAnyObjectByType<Terrain>();
            if (spider == null || terrain == null || terrain.terrainData == null)
                return;

            Vector3 sample = new Vector3(398.38f, 4.73f, 217.97f);
            Vector3 origin = terrain.transform.position;
            Vector3 size = terrain.terrainData.size;
            float nx = Mathf.InverseLerp(origin.x, origin.x + size.x, sample.x);
            float nz = Mathf.InverseLerp(origin.z, origin.z + size.z, sample.z);
            Vector3 normal = terrain.terrainData.GetInterpolatedNormal(nx, nz).normalized;
            Vector3 surfacePoint = new Vector3(
                sample.x, terrain.SampleHeight(sample) + origin.y, sample.z);

            const BindingFlags flags = BindingFlags.Instance | BindingFlags.NonPublic;
            System.Type type = typeof(DexterFrontLegIK);
            FieldInfo clearanceField = type.GetField("terrainRootClearance", flags);
            float clearance = clearanceField != null
                ? (float)clearanceField.GetValue(spider)
                : 1.5f;
            Vector3 rootPosition = surfacePoint + normal * clearance;
            spider.transform.position = rootPosition;
            type.GetMethod("InitializeRig", flags)?.Invoke(spider, null);
            clearanceField?.SetValue(spider, clearance);
            type.GetField("locomotionWorldPosition", flags)?.SetValue(
                spider, rootPosition);
            type.GetField("climbSurfaceAnchor", flags)?.SetValue(
                spider, surfacePoint);
            type.GetField("hasClimbSurfaceAnchor", flags)?.SetValue(
                spider, true);
            type.GetField("currentYawDegrees", flags)?.SetValue(
                spider, 34.6f);
        }

        private static void PrepareConcaveCornerStart()
        {
            DexterFrontLegIK spider = Object.FindAnyObjectByType<DexterFrontLegIK>();
            if (spider == null)
                return;

            Vector3 rootPosition = new Vector3(
                388.41f, 1.0500002f, 215.57f);
            spider.transform.position = rootPosition;
            const BindingFlags flags = BindingFlags.Instance | BindingFlags.NonPublic;
            System.Type type = typeof(DexterFrontLegIK);
            type.GetMethod("InitializeRig", flags)?.Invoke(spider, null);
            type.GetField("locomotionWorldPosition", flags)?.SetValue(
                spider, rootPosition);
            type.GetField("hasClimbSurfaceAnchor", flags)?.SetValue(
                spider, false);
            type.GetField("currentYawDegrees", flags)?.SetValue(
                spider, -129.4122f);
        }

        private static void UpdateSyntheticForwardWalk()
        {
            if (syntheticReceiver == null)
            {
                syntheticReceiver = Object.FindAnyObjectByType<DexterRelayUdpReceiver>();
                if (syntheticReceiver == null)
                    return;
                syntheticReceiver.enabled = false;
            }

            double elapsed = EditorApplication.timeSinceStartup -
                             forwardWalkStartedAt;
            float leftX = 0f;
            float rightX = 0f;
            float leftY = 0f;
            float rightY = 0f;
            if (elapsed >= syntheticTurnStart && elapsed < syntheticTurnEnd)
            {
                leftX = syntheticTurnForce;
                rightX = syntheticTurnForce;
            }
            if (elapsed >= syntheticWalkStart && elapsed < syntheticInputEnd)
            {
                double cycle = (elapsed - syntheticWalkStart) % 0.8;
                if (cycle < 0.2)
                    leftY = syntheticWalkForce;
                else if (cycle >= 0.4 && cycle < 0.6)
                    rightY = syntheticWalkForce;
            }

            var frame = new DexterForceFrame
            {
                type = "force",
                version = 1,
                sequence = ++syntheticSequence,
                transport = "editor-walk-test",
                fingers = new DexterFingerMeasurements
                {
                    thumb = CreateSyntheticFinger(0f, 0f),
                    index = CreateSyntheticFinger(leftX, leftY),
                    middle = CreateSyntheticFinger(rightX, rightY),
                    ring = CreateSyntheticFinger(0f, 0f),
                    pinky = CreateSyntheticFinger(0f, 0f)
                }
            };
            LatestFrameProperty?.SetValue(syntheticReceiver, frame);
            LastFrameRealtimeField?.SetValue(
                syntheticReceiver, Time.realtimeSinceStartup);
        }

        private static DexterFingerMeasurement CreateSyntheticFinger(float x, float y)
        {
            return new DexterFingerMeasurement
            {
                raw = System.Array.Empty<int>(),
                force = new[] { x, y },
                channels = 2,
                has_data = true
            };
        }

        private static void RestorePhysicalReceiverAfterSyntheticTest()
        {
            if (!SessionState.GetBool(SyntheticReceiverDisabledKey, false) &&
                syntheticReceiver == null)
                return;

            if (syntheticReceiver != null)
                syntheticReceiver.enabled = true;

            // SessionState survives assembly/domain reloads while the cached
            // component reference does not. Find the scene receiver as a
            // fallback so a synthetic test can never leave real Dexter input
            // disabled for the user's next Play run.
            DexterRelayUdpReceiver[] receivers =
                Object.FindObjectsByType<DexterRelayUdpReceiver>(
                    FindObjectsInactive.Include,
                    FindObjectsSortMode.None);
            foreach (DexterRelayUdpReceiver receiver in receivers)
            {
                if (receiver != null && receiver.gameObject.activeInHierarchy)
                    receiver.enabled = true;
            }

            SessionState.SetBool(SyntheticReceiverDisabledKey, false);
            syntheticReceiver = null;
        }

        private static void ProcessExternalCommand()
        {
            if (!File.Exists(commandPath)) return;
            string command = File.ReadAllText(commandPath).Trim().ToUpperInvariant();
            File.Delete(commandPath);
            if (command == "PLAY5") CaptureFiveSeconds();
            else if (command == "PLAY12") CaptureTwelveSeconds();
            else if (command == "OPEN_PLAY5") OpenSpiderSceneAndCapture();
            else if (command == "WALK8") TestForwardWalkEightSeconds();
            else if (command == "WALK20") TestForwardWalkTwentySeconds();
            else if (command == "TROUGH46") TestTroughDescent();
            else if (command == "TROUGH75") TestFullTroughTraverse();
            else if (command == "ASCEND65") TestTroughWallAscent();
            else if (command == "ASCEND25") TestTroughWallAscentTwentyFiveSeconds();
            else if (command == "CORNER15") TestConcaveCornerApproach();
            else if (command == "STOP") Stop();
        }
    }
}
