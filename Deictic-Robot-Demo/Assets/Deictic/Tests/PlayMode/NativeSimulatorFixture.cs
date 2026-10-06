using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using UnityEngine.XR;
using UnityEngine.XR.Management;
using UnityEngine.XR.OpenXR;
#if UNITY_EDITOR
using UnityEditor;
#endif

/// <summary>Optional native Meta XR Simulator session for scripted ROS tests.
/// Requires a process-scoped runtime selection and never changes the OS runtime.</summary>
public sealed class NativeSimulatorFixture
{
    XRGeneralSettings previousSettings;
    XRManagerSettings manager;
    bool changedSettings, initializedHere, startedHere;
    public XRDisplaySubsystem Display { get; private set; }

    public IEnumerator Initialize()
    {
        if (Environment.GetEnvironmentVariable("DEICTIC_NATIVE_SIMULATOR") != "1") yield break;
        string runtime = Environment.GetEnvironmentVariable("XR_RUNTIME_JSON");
        Assert.That(runtime, Is.Not.Null.And.Not.Empty,
            "Native simulator tests require process-scoped XR_RUNTIME_JSON");
        Assert.That(File.Exists(runtime), Is.True, "Meta XR Simulator runtime must exist: " + runtime);
        Assert.That(Path.GetFileName(runtime), Is.EqualTo("meta_openxr_simulator.json"),
            "This fixture must not initialize a physical headset runtime");

        // The simulator reports unsupported optional Meta extensions during
        // loader startup. Keep those messages in the Unity log; restore normal
        // error assertions before the caller exercises any production behavior.
        bool previousIgnore = LogAssert.ignoreFailingMessages;
        LogAssert.ignoreFailingMessages = true;
        try
        {
            previousSettings = XRGeneralSettings.Instance;
            XRGeneralSettings settings = previousSettings;
#if UNITY_EDITOR
            if (!settings)
            {
                settings = AssetDatabase.LoadAllAssetsAtPath("Assets/XR/XRGeneralSettingsPerBuildTarget.asset")
                    .OfType<XRGeneralSettings>().FirstOrDefault(value => value.name == "Standalone Settings");
                if (settings)
                {
                    XRGeneralSettings.Instance = settings;
                    changedSettings = true;
                }
            }
#endif
            Assert.That(settings, Is.Not.Null, "Standalone XR settings are required");
            manager = settings.Manager;
            Assert.That(manager, Is.Not.Null);
            if (!manager.activeLoader)
            {
                yield return manager.InitializeLoader();
                initializedHere = manager.activeLoader != null;
            }
            Assert.That(manager.activeLoader, Is.Not.Null, "Meta XR Simulator OpenXR loader did not initialize");
            Assert.That(manager.activeLoader.GetType().Name, Does.Contain("OpenXR"));
            // A reused loader can still belong to Quest Link. Environment selection alone
            // does not establish the identity of an already initialized native instance.
            Assert.That(OpenXRRuntime.name, Is.EqualTo("Meta XR Simulator"),
                "The active native runtime is not Meta XR Simulator; no physical headset test is permitted here");
            Assert.That(OpenXRRuntime.version, Is.Not.Null.And.Not.Empty);
            Debug.Log("Verified native runtime: " + OpenXRRuntime.name + " " + OpenXRRuntime.version);
            if (!FindRunningDisplay())
            {
                manager.StartSubsystems();
                startedHere = true;
            }
            float deadline = Time.realtimeSinceStartup + 20;
            while (!FindRunningDisplay() && Time.realtimeSinceStartup < deadline) yield return null;
            Assert.That(Display, Is.Not.Null, "Native OpenXR display must start");
            Assert.That(Display.running, Is.True);
            yield return new WaitForSecondsRealtime(2);
            Assert.That(FindRunningDisplay(), Is.True, "Native display must remain running after startup");
            Debug.Log("Native Meta XR Simulator display running: " + Display.subsystemDescriptor.id);
        }
        finally { LogAssert.ignoreFailingMessages = previousIgnore; }
    }

    bool FindRunningDisplay()
    {
        var displays = new List<XRDisplaySubsystem>();
        SubsystemManager.GetSubsystems(displays);
        Display = displays.FirstOrDefault(value => value.running &&
            value.subsystemDescriptor.id.IndexOf("OpenXR", StringComparison.OrdinalIgnoreCase) >= 0);
        return Display != null;
    }

    public IEnumerator Cleanup()
    {
        bool previousIgnore = LogAssert.ignoreFailingMessages;
        LogAssert.ignoreFailingMessages = true;
        try
        {
            if (startedHere && manager != null) manager.StopSubsystems();
            if (initializedHere && manager != null) manager.DeinitializeLoader();
#if UNITY_EDITOR
            if (changedSettings) XRGeneralSettings.Instance = previousSettings;
#endif
            startedHere = initializedHere = changedSettings = false;
            Display = null;
            yield return null;
        }
        finally { LogAssert.ignoreFailingMessages = previousIgnore; }
    }
}
