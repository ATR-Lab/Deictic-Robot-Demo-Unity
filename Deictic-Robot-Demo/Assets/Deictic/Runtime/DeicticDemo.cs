using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

namespace Deictic
{
    public sealed class DeicticDemo : MonoBehaviour
    {
        public DeicticSettings settings;
        DeicticBridge bridge;
        DeicticInput input;
        DeicticCameraView cameraView;
        TextMesh hud;
        void Start()
        {
            if (!settings) settings = Resources.Load<DeicticSettings>("DeicticSettings");
            if (!settings) { Debug.LogError("Run Deictic > Build Simulation Assets first."); enabled = false; return; }
            OVRCameraRig rig = FindFirstObjectByType<OVRCameraRig>();
            Transform head = rig ? rig.centerEyeAnchor : Camera.main ? Camera.main.transform : null;
            if (!head) { Debug.LogError("An OVRCameraRig or MainCamera is required."); enabled = false; return; }
            bridge = gameObject.AddComponent<DeicticBridge>();
            bridge.Initialize(settings);
            input = gameObject.AddComponent<DeicticInput>();
            input.bridge = bridge; input.head = head; input.pointer = rig ? rig.rightControllerAnchor : null;
            input.viewCamera = head.GetComponent<Camera>(); input.synthetic = settings.syntheticScene;
            if (settings.bimanualTeleop)
            {
                var teleop = gameObject.AddComponent<BimanualTeleop>();
                teleop.bridge = bridge; teleop.head = head;
                teleop.leftController = rig ? rig.leftControllerAnchor : null;
                teleop.rightController = rig ? rig.rightControllerAnchor : null;
                input.teleop = teleop;
            }
            cameraView = gameObject.AddComponent<DeicticCameraView>();
            cameraView.Initialize(head, bridge);
            input.cameraView = cameraView;
            if (settings.syntheticScene) BuildWorkcell();
            if (settings.syntheticScene && !FindFirstObjectByType<Light>())
            {
                var light = new GameObject("Workcell light").AddComponent<Light>();
                light.type = LightType.Directional; light.intensity = 1.4f;
                light.transform.rotation = Quaternion.Euler(40, -25, 0);
            }
            if (settings.robotVisual)
            {
                for (int i = 0; i < 2; i++)
                {
                    var robot = Instantiate(settings.robotVisual, new Vector3(0, .9f, 1.2f), Quaternion.identity);
                    robot.name = i == 0 ? "K1 measured state" : "K1 trajectory preview";
                    var view = robot.AddComponent<K1RobotView>(); view.bridge = bridge; view.preview = i == 1;
                }
            }
            var label = new GameObject("Deictic status");
            label.transform.SetParent(head, false);
            label.transform.localPosition = new Vector3(-.41f, .27f, .75f);
            hud = label.AddComponent<TextMesh>();
            hud.characterSize = .0036f; hud.fontSize = 42; hud.color = Color.white;
            if (settings.publishHeadsetCamera && !settings.syntheticScene)
            {
                var publisher = gameObject.AddComponent<QuestCameraPublisher>(); publisher.bridge = bridge; publisher.surface = input;
            }
        }
        void Update()
        {
            if (!hud || bridge == null) return;
            hud.gameObject.SetActive(!cameraView.RobotView || bridge.TeleopControlsBusy || !string.IsNullOrEmpty(bridge.LastTeleopFault));
            string alignment = bridge.CanCommit ? "ALIGNED" : "COMMITS BLOCKED";
            var status = bridge.Status;
            hud.color = bridge.CanCommit ? new Color(.5f, 1, .8f) : new Color(1, .7f, .25f);
            hud.text = "DEICTIC / BOOSTER K1\n" + (settings.syntheticScene ? "SIMULATION - fixed-base K1\n" : "MR sensor mode\n") +
                alignment + (status != null ? $"   confidence {status.confidence:F2}\n{status.mode} / {status.backend}" : "\nWaiting for ROS 2") +
                "\n" + bridge.Feedback + "\n" + input.InputSource +
                "\nRight trigger release / Space: select   A / Enter: execute\nB / Escape: cancel" +
                (input.teleop ? "\n" + input.teleop.Feedback : "");
        }
        void BuildWorkcell()
        {
            Transform cell = new GameObject("Synthetic workcell (known calibration)").transform;
            cell.position = new Vector3(0, .9f, 1.2f);
            GameObject table = GameObject.CreatePrimitive(PrimitiveType.Cube);
            table.name = "Table"; table.transform.SetParent(cell, false);
            table.transform.localPosition = FLU.ConvertToRUF(new Vector3(.28f, -.3f, -.19f));
            table.transform.localScale = new Vector3(.5f, .08f, .55f);
            table.GetComponent<Renderer>().sharedMaterial = DemoVisuals.Material(new Color(.15f, .2f, .26f));
            table.AddComponent<DeicticSurface>();
            // Raised reaching targets, above the conservative table plane; no grasping/contact.
            for (int i = 0; i < 5; i++)
            {
                var target = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                target.name = "Reach target " + (i + 1);
                target.transform.SetParent(cell, false);
                target.transform.localPosition = FLU.ConvertToRUF(new Vector3(.08f + .02f * i, -.25f, .02f));
                target.transform.localScale = Vector3.one * .025f;
                target.GetComponent<Renderer>().sharedMaterial = DemoVisuals.Material(new Color(.2f, .65f + .05f * i, .9f));
                target.AddComponent<DeicticSurface>();
            }
        }
    }
}
