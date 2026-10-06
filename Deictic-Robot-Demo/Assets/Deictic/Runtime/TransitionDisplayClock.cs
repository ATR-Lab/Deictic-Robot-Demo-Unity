using System;
using Newtonsoft.Json.Linq;

namespace Deictic
{
    /// <summary>Bounded UTC estimate for simulated image display only. Never a command or task clock.</summary>
    public sealed class TransitionDisplayClock
    {
        public const double MaximumAgeSeconds = 12;
        const double MaximumExchangeSeconds = .5;
        const double MaximumClockIntervalError = .005;
        readonly RosClockSync estimator = new RosClockSync();
        string runId;
        double acceptedUtc, acceptedMonotonic;
        public int Epoch { get; private set; }
        public double OffsetSeconds => estimator.OffsetSeconds;
        // Include the two accepted UTC/monotonic interval tolerances, not only network asymmetry.
        public double UncertaintySeconds => estimator.HasEstimate ? estimator.RoundTripSeconds / 2 + .01 : double.PositiveInfinity;

        public void Reset(string run = null) { estimator.Reset(); runId = run; Epoch++; }

        public bool IsReady(double localUtc, double monotonic)
        {
            if (!estimator.HasEstimate || !double.IsFinite(localUtc) || !double.IsFinite(monotonic)) return false;
            double since = monotonic - acceptedMonotonic;
            // A desktop wall-clock adjustment or suspend cannot silently retime a displayed image.
            if (since < 0 || Math.Abs((localUtc - acceptedUtc) - since) > MaximumClockIntervalError)
            { Reset(runId); return false; }
            return monotonic >= estimator.SelectedAt && monotonic - estimator.SelectedAt < MaximumAgeSeconds;
        }

        public bool TryAccept(JObject response, double sentUtc, double sentMonotonic, double receivedUtc, double receivedMonotonic)
        {
            if (response == null || response["schema_version"]?.Type != JTokenType.Integer || (int)response["schema_version"] != 1 ||
                string.IsNullOrEmpty(runId) || (string)response["run_id"] != runId ||
                (string)response["scope"] != "display_only" || (string)response["clock_domain"] != "server_system_utc") return false;
            if (!Number(response["server_receive_utc_seconds"], out double serverReceive) ||
                !Number(response["server_send_utc_seconds"], out double serverSend) ||
                !Number(response["server_processing_seconds"], out double serverProcessing) ||
                !double.IsFinite(sentUtc) || !double.IsFinite(sentMonotonic) ||
                !double.IsFinite(receivedUtc) || !double.IsFinite(receivedMonotonic)) return false;
            double elapsed = receivedMonotonic - sentMonotonic;
            double residual = elapsed - serverProcessing;
            if (elapsed < 0 || elapsed > MaximumExchangeSeconds || serverProcessing < 0 || serverSend < serverReceive ||
                residual < 0 || residual > RosClockSync.MaximumRoundTripSeconds ||
                Math.Abs((receivedUtc - sentUtc) - elapsed) > MaximumClockIntervalError ||
                Math.Abs((serverSend - serverReceive) - serverProcessing) > MaximumClockIntervalError) return false;
            IsReady(receivedUtc, receivedMonotonic); // Discard pre-step samples before considering a new epoch.
            double offset = ((serverReceive - sentUtc) + (serverSend - receivedUtc)) / 2;
            if (estimator.HasEstimate && Math.Abs(offset - estimator.OffsetSeconds) > .1) Epoch++;
            // Use the latest bounded sample so an older minimum-RTT estimate cannot
            // mask a smaller server clock adjustment between HTTP exchanges.
            estimator.Reset();
            if (!estimator.TryAccept(new[] { sentUtc, serverReceive, serverSend }, receivedUtc, sentUtc, receivedMonotonic)) return false;
            acceptedUtc = receivedUtc; acceptedMonotonic = receivedMonotonic;
            return true;
        }

        static bool Number(JToken token, out double value)
        {
            value = 0;
            if (token == null || (token.Type != JTokenType.Float && token.Type != JTokenType.Integer)) return false;
            value = (double)token; return double.IsFinite(value);
        }
    }
}
