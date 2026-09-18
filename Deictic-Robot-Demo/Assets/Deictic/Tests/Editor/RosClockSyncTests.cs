using Deictic;
using NUnit.Framework;

public sealed class RosClockSyncTests
{
    static bool Sample(RosClockSync sync, double outward, double inward, double offset = .2, double at = 0)
    {
        const double sent = 100, processing = .001;
        return sync.TryAccept(new[] { sent, sent + outward + offset, sent + outward + offset + processing },
            sent + outward + processing + inward, sent, at);
    }

    [Test]
    public void AsymmetricSlowReplyCannotCreateAnOutOfBudgetCommandClock()
    {
        var sync = new RosClockSync();
        Assert.That(Sample(sync, .176, .012), Is.False);
        Assert.That(sync.HasEstimate, Is.False);
        Assert.That(Sample(sync, .037, .037), Is.True);
        Assert.That(sync.OffsetSeconds, Is.EqualTo(.2).Within(1e-10));
    }

    [TestCase(.001, .078)]
    [TestCase(.078, .001)]
    public void AcceptedAsymmetryBoundsOffsetErrorBelowControllerFutureLimit(double outward, double inward)
    {
        var sync = new RosClockSync();
        Assert.That(Sample(sync, outward, inward), Is.True);
        Assert.That(System.Math.Abs(sync.OffsetSeconds - .2), Is.LessThan(.04));
    }

    [Test]
    public void RecentMinimumRttResistsJitterWithoutRefreshingItsAge()
    {
        var sync = new RosClockSync();
        Assert.That(Sample(sync, .02, .02, at: 1), Is.True);
        Assert.That(Sample(sync, .06, .01, at: 6), Is.True);
        Assert.That(sync.OffsetSeconds, Is.EqualTo(.2).Within(1e-10));
        Assert.That(sync.SelectedAt, Is.EqualTo(1));
        Assert.That(Sample(sync, .03, .03, at: 17), Is.True);
        Assert.That(sync.SelectedAt, Is.EqualTo(17));
    }

    [Test]
    public void ClockStepAndResumeDiscardOldSamples()
    {
        var sync = new RosClockSync();
        Sample(sync, .01, .01, at: 1);
        Sample(sync, .03, .03, offset: .5, at: 2);
        Assert.That(sync.OffsetSeconds, Is.EqualTo(.5).Within(1e-10));
        Assert.That(sync.SelectedAt, Is.EqualTo(2));
        sync.Reset();
        Assert.That(sync.HasEstimate, Is.False);
        Sample(sync, .03, .03, offset: .2, at: 3);
        Assert.That(sync.OffsetSeconds, Is.EqualTo(.2).Within(1e-10));
    }

    [Test]
    public void InvalidOrUncorrelatedRepliesLeaveEstimateUnchanged()
    {
        var sync = new RosClockSync();
        Sample(sync, .02, .02);
        Assert.That(sync.TryAccept(new[] { 100d, 100.22, 100.221 }, 100.041, 101, 1), Is.False);
        Assert.That(sync.TryAccept(new[] { 100d, 100.22, 100.21 }, 100.04, 100, 1), Is.False);
        Assert.That(sync.TryAccept(new[] { 100d, double.NaN, 100.221 }, 100.041, 100, 1), Is.False);
        Assert.That(sync.TryAccept(null, 100.041, 100, 1), Is.False);
        Assert.That(sync.OffsetSeconds, Is.EqualTo(.2).Within(1e-10));
        Assert.That(sync.SelectedAt, Is.Zero);
    }
}
