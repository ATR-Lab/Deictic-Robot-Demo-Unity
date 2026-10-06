using Deictic;
using Newtonsoft.Json.Linq;
using NUnit.Framework;

public sealed class TransitionDisplayClockTests
{
    static JObject Reply(double received, double sent, double processing = .001, string run = "run") =>
        new JObject { ["schema_version"] = 1, ["run_id"] = run, ["scope"] = "display_only",
            ["clock_domain"] = "server_system_utc", ["server_receive_utc_seconds"] = received,
            ["server_send_utc_seconds"] = sent, ["server_processing_seconds"] = processing };

    static bool Sample(TransitionDisplayClock clock, double outward = .02, double inward = .02, double offset = 42.5,
        double sentUtc = 1000, double sentMono = 10) => clock.TryAccept(
            Reply(sentUtc + offset + outward, sentUtc + offset + outward + .001),
            sentUtc, sentMono, sentUtc + outward + .001 + inward, sentMono + outward + .001 + inward);

    [Test]
    public void BoundedReadOnlyExchangeCalibratesDisplayWithoutChangingCommandClock()
    {
        var display = new TransitionDisplayClock(); display.Reset("run");
        double commandOffset = RosFrames.ClockOffsetSeconds;
        Assert.That(Sample(display), Is.True);
        Assert.That(display.IsReady(1001, 11), Is.True);
        Assert.That(display.OffsetSeconds, Is.EqualTo(42.5).Within(1e-8));
        Assert.That(display.UncertaintySeconds, Is.EqualTo(.03).Within(1e-8));
        Assert.That(RosFrames.ClockOffsetSeconds, Is.EqualTo(commandOffset));
    }

    [Test]
    public void HighRttCannotEstablishDisplayClock()
    {
        var display = new TransitionDisplayClock(); display.Reset("run");
        Assert.That(Sample(display, outward: .176, inward: .012), Is.False);
        Assert.That(display.IsReady(1000.189, 10.189), Is.False);
    }

    [Test]
    public void RejectedSlowSampleCannotExtendCalibrationAge()
    {
        var display = new TransitionDisplayClock(); display.Reset("run");
        Assert.That(Sample(display, outward: .001, inward: .001), Is.True);
        Assert.That(Sample(display, inward: .1, sentUtc: 1009, sentMono: 19), Is.False);
        Assert.That(display.IsReady(1013, 23), Is.False);
    }

    [Test]
    public void ClientClockJumpOrRunChangeInvalidatesImagesImmediately()
    {
        var display = new TransitionDisplayClock(); display.Reset("run"); Sample(display);
        Assert.That(display.IsReady(1002, 11), Is.False);
        Assert.That(Sample(display, sentUtc: 1002, sentMono: 11), Is.True);
        display.Reset("new-run");
        Assert.That(Sample(display), Is.False, "Previous run's timing metadata cannot calibrate the new run");
        Assert.That(display.IsReady(1001, 11), Is.False);
    }

    [TestCase("client_step")]
    [TestCase("server_step")]
    [TestCase("boolean_processing")]
    [TestCase("nonfinite")]
    [TestCase("wrong_scope")]
    [TestCase("backward")]
    public void InvalidTimingCannotEstablishCalibration(string fault)
    {
        var display = new TransitionDisplayClock(); display.Reset("run");
        var response = Reply(1042.52, 1042.521);
        double receivedUtc = 1000.041;
        switch (fault)
        {
            case "client_step": receivedUtc += 1; break;
            case "server_step": response["server_send_utc_seconds"] = 1043.521; break;
            case "boolean_processing": response["server_processing_seconds"] = false; break;
            case "nonfinite": response["server_receive_utc_seconds"] = double.NaN; break;
            case "wrong_scope": response["scope"] = "authority"; break;
            case "backward": response["server_send_utc_seconds"] = 1042.51; break;
        }
        Assert.That(display.TryAccept(response, 1000, 10, receivedUtc, 10.041), Is.False);
        Assert.That(display.IsReady(1000.041, 10.041), Is.False);
    }
}
