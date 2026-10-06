using System;
using System.IO;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEditor;
using UnityEditor.TestTools.TestRunner.Api;
using UnityEngine;

/// <summary>Runs bounded transition tests in the existing editor using its installed simulator.</summary>
[InitializeOnLoad]
public static class TransitionXrTestMenu
{
    const string Prefix = "Deictic.TransitionXrTest.";
    const string Menu = "Tools/Deictic/Transition XR Tests/";
    static readonly string[] Variables = {
        "XR_RUNTIME_JSON", "XR_SELECTED_RUNTIME_JSON", "OTHER_XR_RUNTIME_JSON",
        "DEICTIC_NATIVE_SIMULATOR", "DEICTIC_TRANSITION_TEST_URL", "DEICTIC_TRANSITION_TEST_BACKEND",
        "DEICTIC_PANEL_CAPTURE", "DEICTIC_TRANSITION_TEST_EVIDENCE_DIR"
    };
    static readonly Results callbacks = new Results();
    static bool Active => SessionState.GetBool(Prefix + "active", false);
    static string ProjectRoot => Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
    static string Marker => Path.Combine(ProjectRoot, "Library", "transition-xr-active-run.json");
    static string ProcessStamp
    {
        get
        {
            using (var process = System.Diagnostics.Process.GetCurrentProcess())
                return process.Id + "/" + process.StartTime.ToUniversalTime().ToString("O");
        }
    }

    static TransitionXrTestMenu()
    {
        // PlayMode tests reload the domain. SessionState retains the original process
        // environment and this initializer registers a new callback after that reload.
        TestRunnerApi.RegisterTestCallback(callbacks);
        Application.logMessageReceived += CaptureLog;
        EditorApplication.quitting += OnEditorQuit;
        EditorApplication.delayCall += RecoverInterruptedRun;
    }

    [MenuItem(Menu + "Fixture only (Meta XR)")]
    public static void FixtureOnly() => Run(false);

    [MenuItem(Menu + "Disposable Isaac workflow (Meta XR)")]
    public static void DisposableIsaac() => Run(true);

    [MenuItem(Menu + "Fixture only (Meta XR)", true)]
    [MenuItem(Menu + "Disposable Isaac workflow (Meta XR)", true)]
    static bool CanRun() => !Active && !EditorApplication.isCompiling && !EditorApplication.isPlayingOrWillChangePlaymode;

    static void Run(bool isaac)
    {
        if (!CanRun()) throw new InvalidOperationException("Stop Play mode and finish the current test run before starting another.");
        string runtime = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ProgramFiles),
            "MetaXRSimulator", "v205.0", "meta_openxr_simulator.json");
        string selected = Environment.GetEnvironmentVariable("XR_RUNTIME_JSON");
        if (File.Exists(selected) && Path.GetFileName(selected) == "meta_openxr_simulator.json") runtime = selected;
        if (!File.Exists(runtime)) throw new FileNotFoundException("Installed Meta XR Simulator manifest was not found.", runtime);
        string directory = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "output",
            "transition-xr-" + DateTime.UtcNow.ToString("yyyyMMddTHHmmssfffZ") + "-" + (isaac ? "isaac" : "fixture")));
        Directory.CreateDirectory(directory);
        var previous = new JObject();
        foreach (string variable in Variables) previous[variable] = Environment.GetEnvironmentVariable(variable);
        SessionState.SetString(Prefix + "environment", previous.ToString(Formatting.None));
        SessionState.SetString(Prefix + "directory", directory);
        SessionState.SetBool(Prefix + "active", true);
        try
        {
            foreach (string variable in new[] { "XR_RUNTIME_JSON", "XR_SELECTED_RUNTIME_JSON", "OTHER_XR_RUNTIME_JSON" })
                Environment.SetEnvironmentVariable(variable, runtime);
            Environment.SetEnvironmentVariable("DEICTIC_NATIVE_SIMULATOR", "1");
            Environment.SetEnvironmentVariable("DEICTIC_TRANSITION_TEST_URL", isaac ? "http://127.0.0.1:8766" : null);
            Environment.SetEnvironmentVariable("DEICTIC_TRANSITION_TEST_BACKEND", isaac ? "isaac_sim_k1" : null);
            Environment.SetEnvironmentVariable("DEICTIC_PANEL_CAPTURE", Path.Combine(directory, "frozen-snapshot.png"));
            Environment.SetEnvironmentVariable("DEICTIC_TRANSITION_TEST_EVIDENCE_DIR", directory);
            WriteJson(Path.Combine(directory, "scope.json"), new JObject {
                ["schema_version"] = 1, ["started_utc"] = DateTime.UtcNow.ToString("O"),
                ["status"] = "running", ["editor_process"] = ProcessStamp,
                ["runtime_manifest"] = runtime, ["mode"] = isaac ? "disposable_isaac" : "fixture_only",
                ["scope"] = "Scripted panel render and HTTP workflow with native simulator loader; not participant or physical evidence",
                ["native_integration"] = "project_meta_native_features",
                ["operator_url"] = isaac ? "http://127.0.0.1:8766" : null
            });
            WriteJson(Marker, new JObject { ["editor_process"] = ProcessStamp, ["directory"] = directory });
            Debug.Log("Transition XR tests: " + directory);
            var filter = new Filter { testMode = TestMode.PlayMode, assemblyNames = new[] { "Deictic.IntegrationTests" },
                testNames = isaac ? new[] { "TransitionPanelTests.RealRuntimeAcceptsExplicitTaskWorkflowAndScoresTheRenderedSnapshot" } :
                    new[] { "TransitionPanelTests.FrozenSnapshotNeedsVisibleEyeFrameAndAcceptedAckBeforeDecision",
                        "TransitionPanelTests.HardwareObserverHasNoHttpSessionAndNoMutationControls" } };
            var api = ScriptableObject.CreateInstance<TestRunnerApi>();
            api.Execute(new ExecutionSettings(filter));
            UnityEngine.Object.DestroyImmediate(api);
        }
        catch
        {
            try { UpdateScope("launch_error"); }
            finally { RestoreEnvironment(); }
            throw;
        }
    }

    static void WriteJson(string path, JObject value)
    {
        string temporary = path + ".tmp";
        File.WriteAllText(temporary, value.ToString(Formatting.Indented));
        if (File.Exists(path)) File.Replace(temporary, path, null);
        else File.Move(temporary, path);
    }

    static void UpdateScope(string status, ITestResultAdaptor result = null)
    {
        if (!Active) return;
        string path = Path.Combine(SessionState.GetString(Prefix + "directory", ""), "scope.json");
        if (!File.Exists(path)) return;
        var scope = JObject.Parse(File.ReadAllText(path));
        scope["status"] = status; scope["finished_utc"] = DateTime.UtcNow.ToString("O");
        if (result != null)
        { scope["passed"] = result.PassCount; scope["failed"] = result.FailCount; scope["skipped"] = result.SkipCount; }
        WriteJson(path, scope);
    }

    static void RecoverInterruptedRun()
    {
        if (!File.Exists(Marker)) return;
        try { RecoverReadableMarker(); }
        catch (Exception exception) when (exception is JsonException || exception is IOException ||
            exception is ArgumentException || exception is UnauthorizedAccessException)
        {
            // A crash can interrupt the initial write too. Retain the original marker
            // and record unreadable recovery without following an unverified path.
            string directory = Path.GetFullPath(Path.Combine(ProjectRoot, "..", "output",
                "transition-xr-recovery-" + DateTime.UtcNow.ToString("yyyyMMddTHHmmssfffZ")));
            try
            {
                Directory.CreateDirectory(directory);
                WriteJson(Path.Combine(directory, "scope.json"), new JObject {
                    ["status"] = "interrupted_marker_unreadable", ["exception_type"] = exception.GetType().Name,
                    ["scope"] = "No completion result can be recovered; no saved environment or feature state was applied"
                });
            }
            finally
            {
                SessionState.SetBool(Prefix + "active", false);
                SessionState.EraseString(Prefix + "environment"); SessionState.EraseString(Prefix + "directory");
            }
            Debug.LogWarning("Transition XR recovery marker was unreadable. No previous run is claimed active or successful. " + directory);
        }
    }

    static void RecoverReadableMarker()
    {
        var marker = JObject.Parse(File.ReadAllText(Marker));
        if (string.IsNullOrEmpty((string)marker["editor_process"]) || string.IsNullOrEmpty((string)marker["directory"]))
            throw new JsonException("Incomplete transition XR recovery marker");
        if ((string)marker["editor_process"] == ProcessStamp) return;
        string directory = Path.GetFullPath((string)marker["directory"] ?? "");
        string output = Path.GetFullPath(Path.Combine(ProjectRoot, "..", "output")) + Path.DirectorySeparatorChar;
        if (directory.StartsWith(output, StringComparison.OrdinalIgnoreCase) && Path.GetFileName(directory).StartsWith("transition-xr-", StringComparison.Ordinal))
        {
            string path = Path.Combine(directory, "scope.json");
            if (File.Exists(path))
            {
                var scope = JObject.Parse(File.ReadAllText(path));
                if ((string)scope["status"] == "running")
                {
                    scope["status"] = "interrupted";
                    scope["interruption_detected_utc"] = DateTime.UtcNow.ToString("O");
                    scope["interruption_reason"] = "Previous editor process ended without a test completion receipt";
                    WriteJson(path, scope);
                }
            }
        }
        // A new editor has its own environment. Never restore values from a crashed process.
        SessionState.SetBool(Prefix + "active", false);
        SessionState.EraseString(Prefix + "environment"); SessionState.EraseString(Prefix + "directory");
        File.Delete(Marker);
    }

    static void OnEditorQuit()
    {
        try { UpdateScope("interrupted"); }
        finally { RestoreEnvironment(); }
    }

    static void CaptureLog(string message, string stack, LogType type)
    {
        if (!Active) return;
        string path = Path.Combine(SessionState.GetString(Prefix + "directory", ""), "editor-test.log");
        try
        {
            // Native extension startup can be noisy; preserve a bounded diagnostic log.
            if (File.Exists(path) && new FileInfo(path).Length >= 1024 * 1024) return;
            File.AppendAllText(path, DateTime.UtcNow.ToString("O") + " " + type + " " + message + "\n" +
                (type == LogType.Exception || type == LogType.Error ? stack + "\n" : ""));
        }
        catch (IOException) { }
    }

    static void RestoreEnvironment()
    {
        if (!Active) return;
        var previous = JObject.Parse(SessionState.GetString(Prefix + "environment", "{}"));
        foreach (string variable in Variables) Environment.SetEnvironmentVariable(variable, (string)previous[variable]);
        SessionState.SetBool(Prefix + "active", false);
        SessionState.EraseString(Prefix + "environment");
        SessionState.EraseString(Prefix + "directory");
        if (File.Exists(Marker)) File.Delete(Marker);
    }

    sealed class Results : IErrorCallbacks
    {
        public void OnError(string message)
        {
            if (!Active) return;
            try { UpdateScope("run_error"); Debug.LogError("Transition XR run failed before completion: " + message); }
            finally { RestoreEnvironment(); }
        }
        public void RunStarted(ITestAdaptor testsToRun) { }
        public void TestStarted(ITestAdaptor test) { }
        public void TestFinished(ITestResultAdaptor result) { }
        public void RunFinished(ITestResultAdaptor result)
        {
            if (!Active) return;
            string directory = SessionState.GetString(Prefix + "directory", "");
            try
            {
                TestRunnerApi.SaveResultToFile(result, Path.Combine(directory, "results.xml"));
                UpdateScope("completed", result);
                Debug.Log("Transition XR results: " + result.PassCount + " passed, " + result.FailCount +
                    " failed, " + result.SkipCount + " skipped. " + directory);
            }
            finally { RestoreEnvironment(); }
        }
    }
}
