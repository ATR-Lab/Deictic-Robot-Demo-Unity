using System;
using System.Collections;
using System.Text;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEngine;
using UnityEngine.Networking;

namespace Deictic
{
    /// <summary>Authenticated, serialized operator input to a loopback SSH-forwarded runtime.</summary>
    public sealed class TransitionOperatorClient : MonoBehaviour
    {
        public TransitionPresentation Presentation { get; } = new TransitionPresentation();
        public TransitionDisplayClock DisplayClock { get; } = new TransitionDisplayClock();
        public bool Connected { get; private set; }
        public bool Busy { get; private set; }
        public string Feedback { get; private set; } = "Connecting to transition runtime";
        public string History { get; private set; }
        public event Action Changed;
        string address, token;
        bool initialized;
        float nextPoll, lastStateAt = float.NegativeInfinity;
        float nextClockPoll;
        UnityWebRequest activeRequest;
        public bool Fresh => Connected && Time.realtimeSinceStartup - lastStateAt < 3;

        public void Initialize(string url)
        {
            TransitionRuntimeConfig.ValidateOperatorUrl(url);
            address = url.TrimEnd('/'); initialized = true;
        }

        void Update()
        {
            if (initialized && !Busy && Time.realtimeSinceStartup >= nextPoll) StartCoroutine(Refresh());
        }

        IEnumerator Refresh()
        {
            Busy = true;
            if (!Connected)
            {
                yield return Request("GET", "/api/session", null, session =>
                {
                    if (session["schema_version"]?.Value<int>() != 1 || string.IsNullOrEmpty((string)session["session_token"]))
                        throw new InvalidOperationException("Unsupported operator session");
                    bool changed = Presentation.BeginSession((string)session["run_id"]);
                    if (changed) { DisplayClock.Reset(Presentation.RunId); nextClockPoll = 0; }
                    token = (string)session["session_token"];
                    Connected = true;
                    if (changed) Feedback = "Connected. Grant and acknowledge task authority explicitly.";
                });
            }
            if (Connected)
                yield return Request("GET", "/api/state", null, state =>
                {
                    Presentation.AcceptState(state);
                    lastStateAt = Time.realtimeSinceStartup;
                });
            if (Connected && Time.realtimeSinceStartup >= nextClockPoll)
            {
                nextClockPoll = Time.realtimeSinceStartup + .5f;
                yield return Request("GET", "/api/time", null, reply => { }, timedSuccess: (reply, sentUtc, sentMono, receivedUtc, receivedMono) =>
                {
                    bool accepted = DisplayClock.TryAccept(reply, sentUtc, sentMono, receivedUtc, receivedMono);
                    nextClockPoll = Time.realtimeSinceStartup + (accepted ? 2 : .5f);
                }, disconnectOnFailure: false);
            }
            Busy = false; nextPoll = Time.realtimeSinceStartup + (Connected ? .4f : 1f); Changed?.Invoke();
        }

        public bool Send(string operation, JObject data = null)
        {
            if (!Fresh || Busy) return false;
            StartCoroutine(Operate(operation, data ?? new JObject()));
            return true;
        }

        public bool Choose(string kind, string target)
        {
            if (!Fresh || Busy || !Presentation.CanRespond) return false;
            return Send("respond", Presentation.Choose(kind, target));
        }

        public bool RetryDecision() => Presentation.HasPendingDecision && Send("respond", Presentation.PendingDecision);

        public void Revoke()
        {
            // A task-level stop has priority over a state/history GET, but cannot be queued
            // behind an unbounded client queue. Abort that read and issue one explicit request.
            if (Busy && activeRequest != null && activeRequest.method == "GET")
            {
                activeRequest.Abort(); StopAllCoroutines(); activeRequest.Dispose(); activeRequest = null; Busy = false;
            }
            if (!Send("revoke")) Feedback = "Revoke was not sent. Wait for the current request or use the runtime console.";
            Changed?.Invoke();
        }

        IEnumerator Operate(string operation, JObject data)
        {
            Busy = true; Changed?.Invoke();
            bool accepted = false;
            bool notCaptured = false;
            yield return Request("POST", "/api/operator", new JObject { ["operation"] = operation, ["data"] = data.DeepClone() }, reply =>
            {
                if (operation == "displayed")
                    Presentation.ConfirmDisplayed((string)data["lease_id"], (string)data["snapshot_hash"]);
                if (operation == "respond" && !Presentation.ConfirmDecision(reply))
                    throw new InvalidOperationException("Decision response has no matching identity; retry the same decision");
                Feedback = operation == "respond" ? "First decision recorded: " + (string)reply["label"] : "Recorded: " + operation;
                if (operation == "reconcile") History = reply.ToString(Formatting.Indented);
                accepted = true;
            }, rejection =>
            {
                if (operation == "respond") notCaptured = Presentation.ConfirmNotCaptured(rejection);
            });
            if (!accepted && operation == "respond") Feedback += notCaptured ?
                " · Server confirms this choice was not captured." : " · Decision outcome uncertain; retry keeps its original ID.";
            Busy = false; nextPoll = 0; Changed?.Invoke();
        }

        public void FetchHistory()
        {
            if (Fresh && !Busy) StartCoroutine(ReadHistory());
        }

        IEnumerator ReadHistory()
        {
            Busy = true;
            yield return Request("GET", "/api/events", null, reply => History = reply.ToString(Formatting.Indented));
            Busy = false; Changed?.Invoke();
        }

        IEnumerator Request(string method, string path, JObject payload, Action<JObject> success, Action<JObject> rejected = null,
            Action<JObject, double, double, double, double> timedSuccess = null, bool disconnectOnFailure = true)
        {
            using (var request = new UnityWebRequest(address + path, method))
            {
                activeRequest = request;
                request.timeout = timedSuccess == null ? 5 : 1;
                request.redirectLimit = 0; // Never forward the session credential to a redirect.
                request.downloadHandler = new DownloadHandlerBuffer();
                if (payload != null)
                {
                    request.uploadHandler = new UploadHandlerRaw(Encoding.UTF8.GetBytes(payload.ToString(Formatting.None)));
                    request.SetRequestHeader("Content-Type", "application/json");
                    request.SetRequestHeader("X-Session-Token", token);
                }
                double sentUtc = RosFrames.LocalNow, sentMonotonic = Time.realtimeSinceStartupAsDouble;
                yield return request.SendWebRequest();
                double receivedUtc = RosFrames.LocalNow, receivedMonotonic = Time.realtimeSinceStartupAsDouble;
                try
                {
                    if (request.downloadedBytes > 2 * 1024 * 1024) throw new InvalidOperationException("Operator response too large");
                    if (request.result == UnityWebRequest.Result.ConnectionError || string.IsNullOrEmpty(request.downloadHandler.text))
                        throw new InvalidOperationException(request.error ?? "Empty operator response");
                    var parsed = JToken.Parse(request.downloadHandler.text, new JsonLoadSettings { DuplicatePropertyNameHandling = DuplicatePropertyNameHandling.Error });
                    // Events is an array; keep the original entries inside a named object for display.
                    var response = parsed as JObject ?? new JObject { ["events"] = parsed };
                    if (request.result != UnityWebRequest.Result.Success)
                    {
                        rejected?.Invoke(response);
                        throw new InvalidOperationException((string)response["error"] ?? request.error);
                    }
                    success(response);
                    timedSuccess?.Invoke(response, sentUtc, sentMonotonic, receivedUtc, receivedMonotonic);
                }
                catch (Exception e)
                {
                    if (disconnectOnFailure)
                    {
                        Feedback = "Operator connection: " + e.Message;
                        Connected = false;
                    }
                    DisplayClock.Reset(Presentation.RunId);
                }
                activeRequest = null;
            }
        }

        void OnDisable()
        {
            StopAllCoroutines(); activeRequest?.Abort(); activeRequest?.Dispose(); activeRequest = null;
            Busy = Connected = false; token = null;
            DisplayClock.Reset(Presentation.RunId);
            // The server's lease/watchdog owns disconnect behavior; never claim a verified stop here.
        }
    }
}
