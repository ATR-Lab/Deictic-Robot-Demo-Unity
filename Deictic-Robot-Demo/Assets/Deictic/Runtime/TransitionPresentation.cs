using System;
using Newtonsoft.Json.Linq;

namespace Deictic
{
    /// <summary>Client-side identity bookkeeping only. The server owns authority, clocks and scoring.</summary>
    public sealed class TransitionPresentation
    {
        public string RunId { get; private set; }
        public JObject State { get; private set; }
        public string LeaseId { get; private set; }
        public string SnapshotHash { get; private set; }
        JObject snapshot;
        JObject pendingDecision;
        bool displayed;
        string decidedLease;
        public bool HasSnapshot => snapshot != null;
        public JObject Snapshot => snapshot == null ? null : (JObject)snapshot.DeepClone();
        public JObject PendingDecision => pendingDecision == null ? null : (JObject)pendingDecision.DeepClone();
        public bool HasPendingDecision => pendingDecision != null;
        public bool Exposed => (string)(State?["lease"] as JObject)?["status"] == "exposed" && HasSnapshot;
        public bool NeedsDisplayAcknowledgement => Exposed && !displayed && decidedLease != LeaseId;
        public bool CanRespond => Exposed && displayed && pendingDecision == null && decidedLease != LeaseId;

        public bool BeginSession(string runId)
        {
            if (string.IsNullOrEmpty(runId)) throw new ArgumentException("Session has no run ID");
            if (RunId == runId) return false;
            RunId = runId; State = null; LeaseId = SnapshotHash = null;
            snapshot = pendingDecision = null; displayed = false; decidedLease = null;
            return true;
        }

        public void AcceptState(JObject state)
        {
            if ((string)state["run_id"] != RunId) throw new InvalidOperationException("Runtime session changed");
            if (!(state["facts"] is JObject)) throw new ArgumentException("Runtime state has no fact map");
            var lease = state["lease"] as JObject;
            var candidate = lease?["snapshot"] as JObject;
            if (candidate != null && !string.IsNullOrEmpty((string)lease?["snapshot_hash"]))
            {
                string id = (string)lease["id"], hash = (string)lease["snapshot_hash"];
                if (string.IsNullOrEmpty(id) || hash == null || hash.Length != 64 || !IsHex(hash))
                    throw new ArgumentException("Snapshot is missing its lease/hash binding");
                if (id == LeaseId && (SnapshotHash != hash || !JToken.DeepEquals(snapshot, candidate)))
                    throw new InvalidOperationException("Immutable lease snapshot changed");
                if (id != LeaseId)
                {
                    LeaseId = id; SnapshotHash = hash; snapshot = (JObject)candidate.DeepClone();
                    displayed = false;
                }
            }
            else if ((string)lease?["id"] != LeaseId)
            {
                LeaseId = SnapshotHash = null; snapshot = null; displayed = false;
            }
            State = (JObject)state.DeepClone();
            // A display ACK from another client never proves that Unity rendered this snapshot.
        }

        static bool IsHex(string value)
        {
            foreach (char c in value) if (!(c >= '0' && c <= '9') && !(c >= 'a' && c <= 'f')) return false;
            return true;
        }

        public JObject DisplayAcknowledgement() => new JObject { ["lease_id"] = LeaseId, ["snapshot_hash"] = SnapshotHash };

        public void ConfirmDisplayed(string leaseId, string hash)
        {
            if (leaseId == LeaseId && hash == SnapshotHash && Exposed) displayed = true;
        }

        public JObject Choose(string kind, string target)
        {
            if (!CanRespond) throw new InvalidOperationException("Render and acknowledge the current snapshot before deciding");
            if (kind != "execute" && kind != "defer" && kind != "request_evidence") throw new ArgumentException("Unknown decision kind");
            if (string.IsNullOrWhiteSpace(target)) throw new ArgumentException("A decision target is required");
            pendingDecision = new JObject { ["lease_id"] = LeaseId, ["snapshot_hash"] = SnapshotHash,
                ["decision_id"] = Guid.NewGuid().ToString("D"), ["kind"] = kind, ["target"] = target };
            return PendingDecision;
        }

        public bool ConfirmDecision(JObject response)
        {
            if (!MatchesPending(response)) return false;
            decidedLease = (string)pendingDecision["lease_id"];
            pendingDecision = null;
            return true;
        }

        public bool ConfirmNotCaptured(JObject response)
        {
            // HTTP status alone cannot prove absence of a durable capture: storage or
            // scoring errors can surface after the first choice has already committed.
            if (!MatchesPending(response) || response["response_captured"]?.Type != JTokenType.Boolean ||
                (bool)response["response_captured"]) return false;
            pendingDecision = null;
            return true;
        }

        bool MatchesPending(JObject response) => pendingDecision != null &&
            (string)response["decision_id"] == (string)pendingDecision["decision_id"] &&
            (string)response["lease_id"] == (string)pendingDecision["lease_id"] &&
            (string)response["snapshot_hash"] == (string)pendingDecision["snapshot_hash"];
    }

    /// <summary>A click is a press/release over the same control. A two-trigger chord cancels it.</summary>
    public sealed class TransitionClickIntent
    {
        int pressed = -1;
        bool chordHeld;
        public bool BlockedUntilRelease => chordHeld;
        public int Step(bool rightPressed, bool rightReleased, bool leftHeld, bool rightHeld, int hovered)
        {
            if (leftHeld && rightHeld) { chordHeld = true; pressed = -1; }
            if (chordHeld)
            {
                if (!leftHeld && !rightHeld) chordHeld = false;
                return -1;
            }
            if (rightPressed) pressed = hovered;
            if (!rightReleased) return -1;
            int result = pressed >= 0 && pressed == hovered ? pressed : -1;
            pressed = -1;
            return result;
        }
        public void Cancel() { pressed = -1; chordHeld = true; }
    }
}
