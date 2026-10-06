using System;
using System.IO;
using Deictic;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.XR.OpenXR;

/// <summary>Bounded passive simulator input diagnosis; never changes input or sends commands.</summary>
[InitializeOnLoad]
public static class TransitionInputTraceMenu
{
    const string Menu = "Tools/Deictic/Transition XR Tests/Record simulator input (30 sec)";
    static DeicticInput input;
    static StreamWriter output;
    static string path, previous;
    static double deadline, nextHeartbeat;
    static int frame = -1, records;

    static TransitionInputTraceMenu()
    {
        EditorApplication.update += Sample;
        EditorApplication.playModeStateChanged += state => { if (state == PlayModeStateChange.ExitingPlayMode) Finish("play_mode_ended"); };
        AssemblyReloadEvents.beforeAssemblyReload += () => Finish("assembly_reload");
        EditorApplication.quitting += () => Finish("editor_exit");
    }

    [MenuItem(Menu, true)]
    static bool CanRecord() => EditorApplication.isPlaying && output == null;

    [MenuItem(Menu)]
    public static void Record()
    {
        if (!CanRecord()) throw new InvalidOperationException("Enter Play mode first; only one passive input trace can run.");
        if (OpenXRRuntime.name != "Meta XR Simulator") throw new InvalidOperationException("Input trace requires the active native Meta XR Simulator.");
        input = UnityEngine.Object.FindAnyObjectByType<DeicticInput>();
        if (!input || !input.isActiveAndEnabled || !input.bridge ||
            input.bridge.settings.controlMode != DeicticControlMode.TransitionSimulation || input.bridge.DirectCommandsAllowed)
            throw new InvalidOperationException("Input trace requires the transition simulation application with direct commands disabled.");
        string directory = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "output",
            "transition-input-" + DateTime.UtcNow.ToString("yyyyMMddTHHmmssfffZ")));
        Directory.CreateDirectory(directory); path = Path.Combine(directory, "input.jsonl");
        output = new StreamWriter(path, false) { AutoFlush = true };
        deadline = EditorApplication.timeSinceStartup + 30; nextHeartbeat = 0; records = 0; frame = -1; previous = null;
        Write(new JObject { ["event"] = "trace_start", ["runtime"] = OpenXRRuntime.name, ["runtime_version"] = OpenXRRuntime.version,
            ["scope"] = "Passive simulator input reads on changed state or one-second heartbeat; no commands or network reads",
            ["max_seconds"] = 30, ["max_samples"] = 4096 });
        Debug.Log("Recording passive simulator input for 30 seconds: " + path);
    }

    static void Sample()
    {
        if (output == null) return;
        if (EditorApplication.timeSinceStartup >= deadline || records >= 4096) { Finish("bounded_capture_complete"); return; }
        if (!EditorApplication.isPlaying || !input || !input.isActiveAndEnabled) { Finish("application_unavailable"); return; }
        if (Time.frameCount == frame) return;
        frame = Time.frameCount;
        try
        {
            if (OpenXRRuntime.name != "Meta XR Simulator" || input.bridge.settings.controlMode != DeicticControlMode.TransitionSimulation ||
                input.bridge.DirectCommandsAllowed) { Finish("simulation_scope_changed"); return; }
            bool xrActive = OVRManager.instance != null && OVRManager.isHmdPresent;
            bool connected = OVRInput.IsControllerConnected(OVRInput.Controller.RTouch);
            bool tracked = xrActive && input.pointer && connected && OVRInput.GetControllerPositionTracked(OVRInput.Controller.RTouch);
            Ray ray = tracked ? new Ray(input.pointer.position, input.pointer.forward) : new Ray(input.head.position, input.head.forward);
            string raySource = tracked ? "controller" : "head";
            var mouse = Mouse.current;
            if (!tracked && input.viewCamera && mouse != null && input.synthetic)
            { ray = input.viewCamera.ScreenPointToRay(mouse.position.ReadValue()); raySource = "desktop_mouse"; }
            bool overToggle = input.cameraView && input.cameraView.PointAtToggle(ray, out _);
            int taskControl = -1;
            bool overTask = input.transitionPanel && input.transitionPanel.PointAtControl(ray, out taskControl, out _);
            var sample = new JObject {
                ["xr_active"] = xrActive, ["application_focused"] = Application.isFocused,
                ["ovr_input_focus"] = OVRManager.hasInputFocus,
                ["connected_controllers"] = OVRInput.GetConnectedControllers().ToString(),
                ["active_controller"] = OVRInput.GetActiveController().ToString(),
                ["right_connected"] = connected, ["right_position_tracked"] = tracked,
                ["right_raw_axis"] = OVRInput.Get(OVRInput.RawAxis1D.RIndexTrigger, OVRInput.Controller.RTouch),
                ["active_right_raw_axis"] = OVRInput.Get(OVRInput.RawAxis1D.RIndexTrigger, OVRInput.Controller.Active),
                ["left_raw_axis"] = OVRInput.Get(OVRInput.RawAxis1D.LIndexTrigger, OVRInput.Controller.LTouch),
                ["right_raw_button"] = OVRInput.Get(OVRInput.RawButton.RIndexTrigger, OVRInput.Controller.RTouch),
                ["right_raw_pressed"] = OVRInput.GetDown(OVRInput.RawButton.RIndexTrigger, OVRInput.Controller.RTouch),
                ["right_raw_released"] = OVRInput.GetUp(OVRInput.RawButton.RIndexTrigger, OVRInput.Controller.RTouch),
                ["mouse_left_held"] = mouse != null && mouse.leftButton.isPressed,
                ["mouse_left_pressed"] = mouse != null && mouse.leftButton.wasPressedThisFrame,
                ["mouse_left_released"] = mouse != null && mouse.leftButton.wasReleasedThisFrame,
                ["robot_view"] = input.cameraView && input.cameraView.RobotView,
                ["ray_source"] = raySource, ["ray_over_toggle"] = overToggle,
                ["ray_over_task"] = overTask, ["task_control_index"] = taskControl, ["input_source"] = input.InputSource
            };
            string comparison = sample.ToString(Formatting.None);
            if (comparison == previous && EditorApplication.timeSinceStartup < nextHeartbeat) return;
            previous = comparison; nextHeartbeat = EditorApplication.timeSinceStartup + 1;
            sample["event"] = "sample"; sample["frame"] = frame;
            Write(sample); records++;
        }
        catch (Exception exception) { Finish("read_failed:" + exception.GetType().Name); }
    }

    static void Write(JObject record)
    {
        record["utc"] = DateTime.UtcNow.ToString("O");
        record["editor_time"] = EditorApplication.timeSinceStartup;
        output.WriteLine(record.ToString(Formatting.None));
    }

    static void Finish(string reason)
    {
        if (output == null) return;
        try { Write(new JObject { ["event"] = "trace_end", ["reason"] = reason, ["sample_count"] = records }); }
        finally { output.Dispose(); output = null; input = null; }
        Debug.Log("Passive simulator input trace ended: " + reason + ". " + path);
    }
}
