using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

namespace Dexter.Visualize
{
    public enum DexterFinger
    {
        Thumb,
        Index,
        Middle,
        Ring,
        Pinky
    }

    [Serializable]
    public sealed class DexterForceFrame
    {
        public string type;
        public int version;
        public long sequence;
        public string transport;
        public DexterFingerMeasurements fingers;
    }

    [Serializable]
    public sealed class DexterFingerMeasurements
    {
        public DexterFingerMeasurement thumb;
        public DexterFingerMeasurement index;
        public DexterFingerMeasurement middle;
        public DexterFingerMeasurement ring;
        public DexterFingerMeasurement pinky;
    }

    [Serializable]
    public sealed class DexterFingerMeasurement
    {
        public int[] raw = Array.Empty<int>();
        public float[] force = Array.Empty<float>();
        public int channels;
        public bool has_data;
    }

    /// <summary>
    /// Subscribes to dexter-relay and moves received JSON frames onto Unity's main thread.
    /// </summary>
    public sealed class DexterRelayUdpReceiver : MonoBehaviour
    {
        [Header("Relay")]
        [SerializeField] private string serverHost = "127.0.0.1";
        [SerializeField] private int serverPort = 45678;

        [Header("Protocol")]
        [SerializeField, Min(0.25f)] private float subscribeIntervalSeconds = 2f;
        [SerializeField, Min(0.25f)] private float staleAfterSeconds = 2.5f;

        private const int MaxPendingFrames = 128;
        private readonly Queue<string> pendingJson = new Queue<string>();
        private readonly object pendingLock = new object();

        private UdpClient udp;
        private Thread receiveThread;
        private volatile bool running;
        private volatile string lastError;
        private string clientId;
        private float nextSubscribeTime;
        private float lastFrameRealtime = float.NegativeInfinity;

        public DexterForceFrame LatestFrame { get; private set; }
        public string ServerLabel => $"{serverHost}:{serverPort}";
        public string LastError => lastError;
        public bool HasRecentFrame => LatestFrame != null && LastFrameAge <= staleAfterSeconds;
        public float LastFrameAge => Time.realtimeSinceStartup - lastFrameRealtime;

        private void OnEnable()
        {
            StartReceiver();
        }

        private void Update()
        {
            if (!running)
                return;

            if (Time.realtimeSinceStartup >= nextSubscribeTime)
            {
                SendSubscribe();
                nextSubscribeTime = Time.realtimeSinceStartup + Mathf.Max(0.25f, subscribeIntervalSeconds);
            }

            DrainPendingDatagrams();
        }

        private void OnDisable()
        {
            StopReceiver();
        }

        private void OnApplicationQuit()
        {
            StopReceiver();
        }

        public DexterFingerMeasurement GetFinger(DexterFinger finger)
        {
            DexterFingerMeasurements values = LatestFrame?.fingers;
            if (values == null)
                return null;

            switch (finger)
            {
                case DexterFinger.Thumb: return values.thumb;
                case DexterFinger.Index: return values.index;
                case DexterFinger.Middle: return values.middle;
                case DexterFinger.Ring: return values.ring;
                case DexterFinger.Pinky: return values.pinky;
                default: return null;
            }
        }

        private void StartReceiver()
        {
            StopReceiver();
            clientId = Guid.NewGuid().ToString("N");
            lastError = null;

            try
            {
                udp = new UdpClient(0);
                udp.Connect(serverHost, serverPort);
                udp.Client.ReceiveTimeout = 250;

                running = true;
                receiveThread = new Thread(ReceiveLoop)
                {
                    IsBackground = true,
                    Name = "Dexter Relay UDP Receiver"
                };
                receiveThread.Start();
                nextSubscribeTime = 0f;
            }
            catch (Exception exception)
            {
                lastError = exception.Message;
                running = false;
                udp?.Close();
                udp = null;
            }
        }

        private void StopReceiver()
        {
            if (!running && udp == null)
                return;

            SendPacket(BuildPacketJson("unsubscribe"), false);
            running = false;

            try
            {
                udp?.Close();
            }
            catch (SocketException)
            {
                // The socket is already shutting down.
            }

            if (receiveThread != null && receiveThread.IsAlive)
                receiveThread.Join(500);

            receiveThread = null;
            udp = null;
            lock (pendingLock)
                pendingJson.Clear();
        }

        private void ReceiveLoop()
        {
            var remote = new IPEndPoint(IPAddress.Any, 0);

            while (running)
            {
                try
                {
                    byte[] data = udp.Receive(ref remote);
                    string json = Encoding.UTF8.GetString(data);
                    lock (pendingLock)
                    {
                        if (pendingJson.Count >= MaxPendingFrames)
                            pendingJson.Dequeue();
                        pendingJson.Enqueue(json);
                    }
                }
                catch (SocketException exception)
                {
                    if (exception.SocketErrorCode != SocketError.TimedOut && running)
                        lastError = exception.Message;
                }
                catch (ObjectDisposedException)
                {
                    return;
                }
                catch (Exception exception)
                {
                    if (running)
                        lastError = exception.Message;
                }
            }
        }

        private void DrainPendingDatagrams()
        {
            while (true)
            {
                string json;
                lock (pendingLock)
                {
                    if (pendingJson.Count == 0)
                        return;
                    json = pendingJson.Dequeue();
                }

                if (!json.Contains("\"type\":\"force\"") && !json.Contains("\"type\": \"force\""))
                    continue;

                try
                {
                    DexterForceFrame frame = JsonUtility.FromJson<DexterForceFrame>(json);
                    if (frame != null && frame.type == "force" && frame.fingers != null)
                    {
                        LatestFrame = frame;
                        lastFrameRealtime = Time.realtimeSinceStartup;
                        lastError = null;
                    }
                }
                catch (Exception exception)
                {
                    lastError = exception.Message;
                }
            }
        }

        private void SendSubscribe()
        {
            SendPacket(BuildPacketJson("subscribe"), true);
        }

        private void SendPacket(string json, bool reportErrors)
        {
            if (udp == null)
                return;

            try
            {
                byte[] data = Encoding.UTF8.GetBytes(json);
                udp.Send(data, data.Length);
            }
            catch (Exception exception)
            {
                if (reportErrors && running)
                    lastError = exception.Message;
            }
        }

        private string BuildPacketJson(string type)
        {
            return "{\"type\":\"" + type + "\",\"version\":1,\"client\":\"dexter-unity-visualizer\",\"client_id\":\"" + clientId + "\"}";
        }

        private void OnValidate()
        {
            serverPort = Mathf.Clamp(serverPort, 1, 65535);
            subscribeIntervalSeconds = Mathf.Max(0.25f, subscribeIntervalSeconds);
            staleAfterSeconds = Mathf.Max(0.25f, staleAfterSeconds);
        }
    }
}
