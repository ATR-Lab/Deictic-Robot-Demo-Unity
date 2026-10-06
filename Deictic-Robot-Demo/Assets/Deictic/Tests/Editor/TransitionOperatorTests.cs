using System;
using Deictic;
using Newtonsoft.Json.Linq;
using NUnit.Framework;
using UnityEngine;

public sealed class TransitionOperatorTests
{
    static readonly string Hash = new string('a', 64);
    static JObject State(string status = "exposed", string leaseId = "lease-one") => JObject.Parse(@"{
        'run_id':'run-one','facts':{'remote.pointed_target':{'value':'B','status':'known','version':2}},
        'lease':{'id':'" + leaseId + "','status':'" + status + "','snapshot_hash':'" + Hash + @"',
        'displayed_at':20,'snapshot':{'remote.pointed_target':{'value':'A','status':'known','version':1}}}}");
    static TransitionPresentation Presentation()
    {
        var p = new TransitionPresentation(); p.BeginSession("run-one"); p.AcceptState(State()); return p;
    }

    [Test]
    public void LeaseRendersDetachedSnapshotAndNeverUsesLiveFacts()
    {
        var p = Presentation();
        Assert.That((string)p.Snapshot["remote.pointed_target"]["value"], Is.EqualTo("A"));
        var copy = p.Snapshot; copy["remote.pointed_target"]["value"] = "tampered";
        Assert.That((string)p.Snapshot["remote.pointed_target"]["value"], Is.EqualTo("A"));
        Assert.That(p.CanRespond, Is.False, "Another client's displayed_at is not a Unity render ACK");
        Assert.Throws<InvalidOperationException>(() => p.Choose("execute", "select_A"));
        p.ConfirmDisplayed("wrong-lease", Hash);
        Assert.That(p.CanRespond, Is.False);
        p.ConfirmDisplayed("lease-one", new string('b', 64));
        Assert.That(p.CanRespond, Is.False);
        p.ConfirmDisplayed("lease-one", Hash);
        Assert.That(p.CanRespond, Is.True);
    }

    [Test]
    public void SameLeaseCannotSilentlyChangeItsSnapshot()
    {
        var p = Presentation(); var changed = State();
        changed["lease"]["snapshot"]["remote.pointed_target"]["value"] = "B";
        Assert.Throws<InvalidOperationException>(() => p.AcceptState(changed));
        Assert.That((string)p.Snapshot["remote.pointed_target"]["value"], Is.EqualTo("A"));
    }

    [Test]
    public void DrainingLeaseHasNoRenderableSnapshot()
    {
        var p = Presentation();
        var state = State("draining", "lease-two");
        state["lease"]["snapshot"] = new JObject(); state["lease"]["snapshot_hash"] = null;
        p.AcceptState(state);
        Assert.That(p.HasSnapshot, Is.False);
        Assert.That(p.CanRespond, Is.False);
    }

    [Test]
    public void RetryKeepsFirstChoiceAcrossClosedLeaseAndLaterLease()
    {
        var p = Presentation(); p.ConfirmDisplayed("lease-one", Hash);
        JObject choice = p.Choose("execute", "select_A");
        p.AcceptState(State("closed"));
        p.AcceptState(State("exposed", "lease-two"));
        p.ConfirmDisplayed("lease-two", Hash);
        Assert.That(p.CanRespond, Is.False, "Do not replace a choice whose reply was lost");
        Assert.That(JToken.DeepEquals(choice, p.PendingDecision), Is.True);
        p.PendingDecision["target"] = "select_B";
        Assert.That((string)p.PendingDecision["target"], Is.EqualTo("select_A"));
        Assert.That(p.ConfirmDecision(new JObject { ["decision_id"] = "unrelated" }), Is.False);
        Assert.That(p.ConfirmDecision(choice), Is.True);
        Assert.That(p.HasPendingDecision, Is.False);
    }

    [Test]
    public void NewSessionDropsPendingInputAndRequiresFreshState()
    {
        var p = Presentation(); p.ConfirmDisplayed("lease-one", Hash); p.Choose("defer", "sequence_complete");
        Assert.That(p.BeginSession("run-one"), Is.False);
        Assert.That(p.HasPendingDecision, Is.True);
        Assert.That(p.BeginSession("run-two"), Is.True);
        Assert.That(p.HasPendingDecision, Is.False);
        Assert.That(p.State, Is.Null);
        Assert.Throws<InvalidOperationException>(() => p.AcceptState(State()));
    }

    [Test]
    public void OnlyExplicitCorrelatedUncapturedReceiptCanReleaseAnUncertainChoice()
    {
        var p = Presentation(); p.ConfirmDisplayed("lease-one", Hash);
        var choice = p.Choose("execute", "select_A");
        Assert.That(p.ConfirmNotCaptured(new JObject { ["error"] = "conflict" }), Is.False);
        choice["response_captured"] = null;
        Assert.That(p.ConfirmNotCaptured(choice), Is.False);
        choice["response_captured"] = true;
        Assert.That(p.ConfirmNotCaptured(choice), Is.False);
        choice["response_captured"] = false;
        choice["lease_id"] = "unrelated";
        Assert.That(p.ConfirmNotCaptured(choice), Is.False);
        choice["lease_id"] = "lease-one";
        Assert.That(p.ConfirmNotCaptured(choice), Is.True);
        Assert.That(p.HasPendingDecision, Is.False);
    }

    [Test]
    public void TwoTriggerChordCannotBecomeAUiClickAfterEitherRelease()
    {
        var click = new TransitionClickIntent();
        Assert.That(click.Step(true, false, false, true, 4), Is.EqualTo(-1));
        Assert.That(click.Step(false, false, true, true, 4), Is.EqualTo(-1));
        Assert.That(click.Step(false, true, true, false, 4), Is.EqualTo(-1));
        Assert.That(click.Step(false, false, false, false, 4), Is.EqualTo(-1));
        Assert.That(click.BlockedUntilRelease, Is.False);
        Assert.That(click.Step(true, false, false, true, 4), Is.EqualTo(-1));
        Assert.That(click.Step(false, true, false, false, 4), Is.EqualTo(4));
        Assert.That(click.Step(false, true, false, false, 4), Is.EqualTo(-1));
    }

    [Test]
    public void PressAndReleaseMustTargetSameControl()
    {
        var click = new TransitionClickIntent(); click.Step(true, false, false, true, 2);
        Assert.That(click.Step(false, true, false, false, 3), Is.EqualTo(-1));
        click.Step(true, false, false, true, 2); click.Cancel();
        Assert.That(click.Step(false, true, false, false, 2), Is.EqualTo(-1));
    }

    [TestCase("http://192.168.10.102:8766")]
    [TestCase("http://localhost:8766/redirect")]
    [TestCase("http://user:password@127.0.0.1:8766")]
    [TestCase("https://example.com")]
    public void OperatorCredentialsCannotLeaveTheLoopbackTunnel(string address)
        => Assert.Throws<ArgumentException>(() => TransitionRuntimeConfig.ValidateOperatorUrl(address));

    [TestCase("hardware_observation", DeicticControlMode.HardwareObservation)]
    [TestCase("transition_simulation", DeicticControlMode.TransitionSimulation)]
    public void TransitionModesDisableEveryLegacyMotionPath(string mode, DeicticControlMode expected)
    {
        var config = ScriptableObject.CreateInstance<DeicticSettings>();
        var host = new GameObject("Transition command isolation");
        try
        {
            TransitionRuntimeConfig.Apply(config, "{\"schema_version\":1,\"control_mode\":\"" + mode + "\"}");
            config.connectOnStart = false;
            Assert.That(config.controlMode, Is.EqualTo(expected));
            var bridge = host.AddComponent<DeicticBridge>(); bridge.Initialize(config);
            Assert.That(bridge.DirectCommandsAllowed, Is.False);
            config.controlMode = DeicticControlMode.ManualSimulation;
            Assert.That(bridge.DirectCommandsAllowed, Is.False, "Control ownership is immutable within a run");
            Assert.That(bridge.CanTrackRobotHead || bridge.CanCommit || bridge.CanTeleoperate, Is.False);
            Assert.That(bridge.PublishRobotHead(true, true, Quaternion.identity), Is.False);
            Assert.That(bridge.PublishRobotHead(false, false, Quaternion.identity), Is.False);
            Assert.That(bridge.PublishTeleop(true, true, true, Vector3.zero, Quaternion.identity, Vector3.zero, Quaternion.identity), Is.False);
            Assert.That(bridge.PublishTeleop(false, false, false, Vector3.zero, Quaternion.identity, Vector3.zero, Quaternion.identity), Is.False);
            Assert.That(bridge.Commit(Vector3.zero, Vector3.up), Is.False);
            Assert.That(bridge.Execute(), Is.False);
        }
        finally { UnityEngine.Object.DestroyImmediate(host); UnityEngine.Object.DestroyImmediate(config); }
    }

    [TestCase("{'schema_version':true,'control_mode':'manual_simulation'}")]
    [TestCase("{'schema_version':1,'control_mode':'manual_simulation','control_mode':'hardware_observation'}")]
    [TestCase("{'schema_version':1,'control_mode':'manual_simulation','ros_port':false}")]
    [TestCase("{'schema_version':1,'control_mode':'hardware_actuation'}")]
    [TestCase("{'schema_version':1,'control_mode':'manual_simulation','force':true}")]
    public void RuntimeConfigRejectsMalformedOrUnknownFieldsAtomically(string json)
    {
        var config = ScriptableObject.CreateInstance<DeicticSettings>();
        config.controlMode = DeicticControlMode.HardwareObservation;
        try
        {
            Assert.Catch(() => TransitionRuntimeConfig.Apply(config, json));
            Assert.That(config.controlMode, Is.EqualTo(DeicticControlMode.HardwareObservation));
        }
        finally { UnityEngine.Object.DestroyImmediate(config); }
    }
}
