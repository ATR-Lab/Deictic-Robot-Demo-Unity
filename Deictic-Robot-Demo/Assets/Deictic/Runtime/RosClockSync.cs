using System;
using System.Collections.Generic;

namespace Deictic
{
    /// <summary>Four-timestamp synchronization with a bounded uncertainty and recent minimum RTT.</summary>
    public sealed class RosClockSync
    {
        // The controller rejects goals more than 50 ms in its future. An 80 ms
        // round trip bounds offset uncertainty to 40 ms, leaving 10 ms margin.
        public const double MaximumRoundTripSeconds = .08;
        const double SampleWindowSeconds = 10;
        readonly List<Sample> samples = new List<Sample>();
        public bool HasEstimate { get; private set; }
        public double OffsetSeconds { get; private set; }
        public double RoundTripSeconds { get; private set; }
        public double SelectedAt { get; private set; }

        struct Sample
        {
            public double offset, roundTrip, observedAt;
        }

        public void Reset()
        {
            samples.Clear();
            HasEstimate = false;
        }

        public bool TryAccept(double[] timestamps, double received, double request, double observedAt)
        {
            if (timestamps == null || timestamps.Length != 3 || timestamps[0] != request ||
                !double.IsFinite(received) || !double.IsFinite(observedAt)) return false;
            foreach (double value in timestamps) if (!double.IsFinite(value)) return false;
            double roundTrip = received - timestamps[0] - (timestamps[2] - timestamps[1]);
            if (timestamps[2] < timestamps[1] || !double.IsFinite(roundTrip) ||
                roundTrip < 0 || roundTrip > MaximumRoundTripSeconds)
                return false;
            double offset = ((timestamps[1] - timestamps[0]) + (timestamps[2] - received)) * .5;
            if (!double.IsFinite(offset)) return false;
            // A real clock step must not be hidden by a lower-RTT sample from
            // the preceding clock epoch.
            if (HasEstimate && Math.Abs(offset - OffsetSeconds) > .1) Reset();
            samples.RemoveAll(sample => observedAt - sample.observedAt > SampleWindowSeconds);
            if (samples.Count == 32) samples.RemoveAt(0);
            samples.Add(new Sample { offset = offset, roundTrip = roundTrip, observedAt = observedAt });
            Sample best = samples[0];
            foreach (Sample sample in samples) if (sample.roundTrip < best.roundTrip) best = sample;
            OffsetSeconds = best.offset;
            RoundTripSeconds = best.roundTrip;
            SelectedAt = best.observedAt;
            HasEstimate = true;
            return true;
        }
    }
}
