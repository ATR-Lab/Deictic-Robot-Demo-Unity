using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Xml.Linq;
using Oculus.Interaction.Locomotion;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEditor.XR.Management;
using UnityEditor.XR.Management.Metadata;
using UnityEditor.XR.OpenXR.Features;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.XR.Management;
using UnityEngine.XR.OpenXR;
using UnityEngine.XR.OpenXR.Features.Interactions;
using UnityEngine.XR.OpenXR.Features.MetaQuestSupport;

namespace Deictic.Editor
{
    public static class DemoSetup
    {
        const string Output = "Assets/Deictic/Generated";
        [MenuItem("Deictic/Build Simulation Assets")]
        public static void BuildAssets()
        {
            string repo = Path.GetFullPath(Path.Combine(Application.dataPath, "../.."));
            string source = Path.Combine(repo, "models/K1/K1_22dof.urdf");
            if (!File.Exists(source)) throw new FileNotFoundException("Run sim/fetch_k1_assets.py, or keep models/K1 adjacent to the Unity project", source);
            Directory.CreateDirectory(Output + "/Meshes");
            Directory.CreateDirectory("Assets/Resources/Deictic/Materials");
            AssetDatabase.Refresh();
            RetainUnlitMaterial(DemoVisuals.OpaqueTemplate, Color.white);
            RetainUnlitMaterial(DemoVisuals.TransparentTemplate, new Color(1, 1, 1, .3f));
            var document = XDocument.Load(source);
            var robot = new GameObject("K1 URDF visual");
            var links = new Dictionary<string, Transform>();
            foreach (XElement element in document.Root.Elements("link"))
            {
                var link = new GameObject((string)element.Attribute("name"));
                link.transform.SetParent(robot.transform, false);
                links.Add(link.name, link.transform);
            }
            foreach (XElement joint in document.Root.Elements("joint"))
            {
                Transform child = links[(string)joint.Element("child").Attribute("link")];
                child.SetParent(links[(string)joint.Element("parent").Attribute("link")], false);
                SetOrigin(child, joint.Element("origin"));
                if ((string)joint.Attribute("type") == "revolute" || (string)joint.Attribute("type") == "continuous")
                {
                    K1Joint component = child.gameObject.AddComponent<K1Joint>();
                    component.jointName = (string)joint.Attribute("name");
                    component.axis = FLU.ConvertToRUF(Vector((string)joint.Element("axis")?.Attribute("xyz"), Vector3.right));
                    component.restRotation = child.localRotation;
                }
            }
            Material material = AssetDatabase.LoadAssetAtPath<Material>(Output + "/Robot.mat");
            if (!material) { material = DemoVisuals.CreateEditorMaterial(new Color(.78f, .82f, .88f), true); AssetDatabase.CreateAsset(material, Output + "/Robot.mat"); }
            material.shader = Shader.Find("Universal Render Pipeline/Lit") ?? material.shader;
            material.SetFloat("_Smoothness", .35f);
            EditorUtility.SetDirty(material);
            var meshCache = new Dictionary<string, Mesh>();
            foreach (XElement element in document.Root.Elements("link"))
            {
                foreach (XElement visual in element.Elements("visual"))
                {
                    XElement meshTag = visual.Element("geometry")?.Element("mesh");
                    if (meshTag == null) continue;
                    string meshFile = (string)meshTag.Attribute("filename");
                    string meshPath = Path.Combine(Path.GetDirectoryName(source), meshFile);
                    if (!meshCache.TryGetValue(meshFile, out Mesh mesh))
                    {
                        string assetPath = Output + "/Meshes/" + Path.GetFileNameWithoutExtension(meshFile) + ".asset";
                        mesh = AssetDatabase.LoadAssetAtPath<Mesh>(assetPath);
                        if (!mesh) { mesh = ReadStl(meshPath); AssetDatabase.CreateAsset(mesh, assetPath); }
                        meshCache[meshFile] = mesh;
                    }
                    var model = new GameObject("Visual"); model.transform.SetParent(links[(string)element.Attribute("name")], false);
                    SetOrigin(model.transform, visual.Element("origin"));
                    Vector3 scale = Vector((string)meshTag.Attribute("scale"), Vector3.one);
                    model.transform.localScale = new Vector3(scale.y, scale.z, scale.x);
                    model.AddComponent<MeshFilter>().sharedMesh = mesh;
                    model.AddComponent<MeshRenderer>().sharedMaterial = material;
                }
            }
            // Synthetic reach point attached to right hand, matching the Python URDF chain.
            Transform terminal = links.Values.FirstOrDefault(t => t.name == "right_elbow_yaw_link");
            if (terminal)
            {
                var tool = new GameObject("right_tool"); tool.transform.SetParent(terminal, false);
                tool.transform.localPosition = FLU.ConvertToRUF(new Vector3(0, -.1f, 0));
            }
            GameObject prefab = PrefabUtility.SaveAsPrefabAsset(robot, "Assets/Resources/K1Robot.prefab");
            UnityEngine.Object.DestroyImmediate(robot);
            var settings = AssetDatabase.LoadAssetAtPath<DeicticSettings>("Assets/Resources/DeicticSettings.asset");
            if (!settings) { settings = ScriptableObject.CreateInstance<DeicticSettings>(); AssetDatabase.CreateAsset(settings, "Assets/Resources/DeicticSettings.asset"); }
            settings.robotVisual = prefab;
            EditorUtility.SetDirty(settings);
            EnableRos2();
            AssetDatabase.SaveAssets();
            Debug.Log("Deictic K1 assets built from supplied pinned URDF.");
        }
        static void RetainUnlitMaterial(string resourcePath, Color color)
        {
            string path = "Assets/Resources/" + resourcePath + ".mat";
            Material configured = DemoVisuals.CreateEditorMaterial(color);
            Material retained = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (retained)
            {
                retained.shader = configured.shader;
                retained.CopyPropertiesFromMaterial(configured);
                UnityEngine.Object.DestroyImmediate(configured);
                EditorUtility.SetDirty(retained);
            }
            else AssetDatabase.CreateAsset(configured, path);
        }
        [MenuItem("Deictic/Create Demo Scene")]
        public static void CreateScene()
        {
            BuildAssets();
            var scene = EditorSceneManager.OpenScene("Assets/Scenes/SampleScene.unity", OpenSceneMode.Single);
            if (!UnityEngine.Object.FindFirstObjectByType<DeicticDemo>()) new GameObject("Deictic K1 demo").AddComponent<DeicticDemo>();
            ConfigureStationaryRig();
            EditorSceneManager.SaveScene(scene, "Assets/Scenes/DeicticDemo.unity");
            EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene("Assets/Scenes/DeicticDemo.unity", true) };
            Debug.Log("Deictic demo scene ready. Play with ROS endpoint + simulation running.");
        }
        [MenuItem("Deictic/Configure Stationary Demo Rig")]
        public static void ConfigureDemoRig()
        {
            if (EditorApplication.isPlaying) throw new InvalidOperationException("Stop Play mode before configuring the demo rig.");
            var scene = UnityEngine.SceneManagement.SceneManager.GetActiveScene();
            if (scene.path != "Assets/Scenes/DeicticDemo.unity")
                throw new InvalidOperationException("Open DeicticDemo.unity before configuring its rig.");
            ConfigureStationaryRig();
            EditorSceneManager.SaveScene(scene);
        }
        static void ConfigureStationaryRig()
        {
            var rig = UnityEngine.Object.FindAnyObjectByType<OVRCameraRig>();
            if (!rig) throw new InvalidOperationException("The demo needs an OVRCameraRig.");
            // The supplied comprehensive interaction prefab includes virtual locomotion.
            // MR reaching uses physical tracking; its origin must not fall under gravity
            // or move through teleport/joystick actions while registering the workcell.
            foreach (var locomotor in rig.GetComponentsInChildren<FirstPersonLocomotor>(true))
            {
                Transform subtree = locomotor.transform.parent;
                GameObject root = subtree && subtree.name == "Locomotor" ? subtree.gameObject : locomotor.gameObject;
                root.SetActive(false);
                PrefabUtility.RecordPrefabInstancePropertyModifications(root);
            }
            var manager = rig.GetComponent<OVRManager>();
            if (manager)
            {
                var serialized = new SerializedObject(manager);
                serialized.FindProperty("_trackingOriginType").intValue = (int)OVRManager.TrackingOrigin.FloorLevel;
                serialized.ApplyModifiedPropertiesWithoutUndo();
                PrefabUtility.RecordPrefabInstancePropertyModifications(manager);
            }
            // Meta XR Simulator owns emulated pose/input; the legacy Ctrl/mouse
            // emulator would apply a second offset while using editor shortcuts.
            var emulator = rig.GetComponent<OVRHeadsetEmulator>();
            if (emulator)
            {
                emulator.opMode = OVRHeadsetEmulator.OpMode.Off;
                PrefabUtility.RecordPrefabInstancePropertyModifications(emulator);
            }
            EditorSceneManager.MarkSceneDirty(rig.gameObject.scene);
            Debug.Log("DEICTIC_STATIONARY_RIG_CONFIGURED floor origin, virtual locomotor disabled, Meta XR Simulator pose input.");
        }
        public static void EnableRos2()
        {
            PlayerSettings.Android.forceInternetPermission = true;
            // The authenticated operator client accepts loopback URLs only; SSH protects the remote hop.
            PlayerSettings.insecureHttpOption = InsecureHttpOption.AlwaysAllowed;
            foreach (NamedBuildTarget target in new[] { NamedBuildTarget.Standalone, NamedBuildTarget.Android })
            {
                var defines = PlayerSettings.GetScriptingDefineSymbols(target).Split(';').Where(x => !string.IsNullOrWhiteSpace(x)).ToList();
                if (!defines.Contains("ROS2")) { defines.Add("ROS2"); PlayerSettings.SetScriptingDefineSymbols(target, string.Join(";", defines)); }
            }
        }
        [MenuItem("Deictic/Build Quest Development APK")]
        public static void BuildQuest()
        {
            BuildAssets();
            ConfigureQuestXR();
            EditorSceneManager.OpenScene("Assets/Scenes/DeicticDemo.unity", OpenSceneMode.Single);
            ConfigureDemoRig();
            string destination = Environment.GetEnvironmentVariable("DEICTIC_ANDROID_OUTPUT");
            if (string.IsNullOrEmpty(destination))
                destination = Path.GetFullPath(Path.Combine(Application.dataPath, "../../output/DeicticK1.apk"));
            Directory.CreateDirectory(Path.GetDirectoryName(destination));
            PlayerSettings.SetScriptingBackend(NamedBuildTarget.Android, ScriptingImplementation.IL2CPP);
            PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARM64;
            BuildReport report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
            {
                scenes = new[] { "Assets/Scenes/DeicticDemo.unity" },
                locationPathName = destination,
                target = BuildTarget.Android,
                options = BuildOptions.Development
            });
            if (report.summary.result != BuildResult.Succeeded)
                throw new InvalidOperationException("Quest build failed: " + report.summary.result);
            Debug.Log("DEICTIC_QUEST_BUILD_SUCCEEDED " + destination);
        }
        [MenuItem("Deictic/Configure Quest Android XR")]
        public static void ConfigureQuestXR()
        {
            const BuildTargetGroup target = BuildTargetGroup.Android;
            EditorBuildSettings.TryGetConfigObject(XRGeneralSettings.settingsKey, out XRGeneralSettingsPerBuildTarget targets);
            if (!targets)
            {
                targets = AssetDatabase.FindAssets("t:XRGeneralSettingsPerBuildTarget")
                    .Select(guid => AssetDatabase.LoadAssetAtPath<XRGeneralSettingsPerBuildTarget>(AssetDatabase.GUIDToAssetPath(guid)))
                    .FirstOrDefault();
                if (!targets)
                {
                    Directory.CreateDirectory("Assets/XR");
                    AssetDatabase.Refresh();
                    targets = ScriptableObject.CreateInstance<XRGeneralSettingsPerBuildTarget>();
                    AssetDatabase.CreateAsset(targets, "Assets/XR/XRGeneralSettingsPerBuildTarget.asset");
                }
                EditorBuildSettings.AddConfigObject(XRGeneralSettings.settingsKey, targets, true);
            }

            // Create only Android entries; Standalone's editor/simulator lifecycle is independent.
            if (!targets.HasSettingsForBuildTarget(target)) targets.CreateDefaultSettingsForBuildTarget(target);
            if (!targets.HasManagerSettingsForBuildTarget(target)) targets.CreateDefaultManagerSettingsForBuildTarget(target);
            XRGeneralSettings android = targets.SettingsForBuildTarget(target);
            android.InitManagerOnStart = true;
            android.Manager.automaticLoading = true;
            android.Manager.automaticRunning = true;
            if (!XRPackageMetadataStore.AssignLoader(android.Manager, typeof(OpenXRLoader).FullName, target)
                || !android.Manager.activeLoaders.Any(loader => loader is OpenXRLoader))
                throw new InvalidOperationException("Could not configure the Android OpenXR loader.");

            FeatureHelpers.RefreshFeatures(target);
            OpenXRSettings settings = OpenXRSettings.GetSettingsForBuildTargetGroup(target);
            MetaQuestFeature quest = settings ? settings.GetFeature<MetaQuestFeature>() : null;
            OculusTouchControllerProfile touch = settings ? settings.GetFeature<OculusTouchControllerProfile>() : null;
            if (!quest || !touch) throw new InvalidOperationException("Android OpenXR Meta Quest/Touch features are unavailable.");
            quest.enabled = true;
            quest.ForceRemoveInternetPermission = false;
            quest.AddTargetDevice("quest3s", "Quest 3S", true);
            // OpenXR 1.18 exposes AddTargetDevice publicly, but existing enabled flags only via serialization.
            var serializedQuest = new SerializedObject(quest);
            SerializedProperty devices = serializedQuest.FindProperty("targetDevices");
            bool enabledQuest3S = false;
            for (int i = 0; devices != null && i < devices.arraySize; i++)
            {
                SerializedProperty device = devices.GetArrayElementAtIndex(i);
                if (device.FindPropertyRelative("manifestName").stringValue != "quest3s") continue;
                device.FindPropertyRelative("enabled").boolValue = true;
                enabledQuest3S = true;
            }
            if (!enabledQuest3S) throw new InvalidOperationException("OpenXR Meta Quest Support has no Quest 3S target entry.");
            serializedQuest.ApplyModifiedPropertiesWithoutUndo();
            touch.enabled = true;
            var metaConfig = OVRProjectConfig.CachedProjectConfig;
            metaConfig.insightPassthroughSupport = OVRProjectConfig.FeatureSupport.Supported;
            OVRProjectConfig.CommitProjectConfig(metaConfig);

            // Meta XR SDK's Android rendering setup recommends this exact Vulkan configuration.
            PlayerSettings.SetUseDefaultGraphicsAPIs(BuildTarget.Android, false);
            PlayerSettings.SetGraphicsAPIs(BuildTarget.Android, new[] { GraphicsDeviceType.Vulkan });
            PlayerSettings.Android.forceInternetPermission = true;
            foreach (UnityEngine.Object changed in new UnityEngine.Object[] { targets, android, android.Manager, settings, quest, touch })
                EditorUtility.SetDirty(changed);
            AssetDatabase.SaveAssets();
            Debug.Log("DEICTIC_QUEST_XR_CONFIGURED Android OpenXR, automatic initialization, Quest 3S, Touch, Vulkan.");
        }
        static Vector3 Vector(string text, Vector3 fallback)
        {
            if (string.IsNullOrWhiteSpace(text)) return fallback;
            float[] values = text.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries).Select(v => float.Parse(v, CultureInfo.InvariantCulture)).ToArray();
            return new Vector3(values[0], values[1], values[2]);
        }
        static void SetOrigin(Transform transform, XElement origin)
        {
            transform.localPosition = FLU.ConvertToRUF(Vector((string)origin?.Attribute("xyz"), Vector3.zero));
            Vector3 rpy = Vector((string)origin?.Attribute("rpy"), Vector3.zero) * Mathf.Rad2Deg;
            Quaternion q = Quaternion.AngleAxis(rpy.z, Vector3.forward) * Quaternion.AngleAxis(rpy.y, Vector3.up) * Quaternion.AngleAxis(rpy.x, Vector3.right);
            transform.localRotation = FLU.ConvertToRUF(q);
        }
        static Mesh ReadStl(string path)
        {
            using var reader = new BinaryReader(File.OpenRead(path));
            reader.ReadBytes(80);
            int triangles = checked((int)reader.ReadUInt32());
            if (reader.BaseStream.Length != 84L + triangles * 50L) throw new InvalidDataException("Expected binary STL: " + path);
            var vertices = new Vector3[triangles * 3]; var indices = new int[triangles * 3];
            for (int i = 0; i < triangles; i++)
            {
                reader.ReadBytes(12);
                for (int j = 0; j < 3; j++) vertices[i * 3 + j] = FLU.ConvertToRUF(new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle()));
                reader.ReadUInt16();
                indices[i * 3] = i * 3; indices[i * 3 + 1] = i * 3 + 2; indices[i * 3 + 2] = i * 3 + 1;
            }
            var mesh = new Mesh { name = Path.GetFileNameWithoutExtension(path), indexFormat = IndexFormat.UInt32, vertices = vertices, triangles = indices };
            mesh.RecalculateNormals(); mesh.RecalculateBounds(); return mesh;
        }
    }
}
