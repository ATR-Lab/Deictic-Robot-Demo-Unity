using UnityEngine;
using UnityEngine.Rendering.Universal;

namespace Deictic
{
    /// <summary>
    /// Uses URP's XR-aware fullscreen pass before transparent return controls.
    /// Only the bound headset camera is affected; Scene view and other cameras
    /// keep their normal rendering. No scene geometry represents the video.
    /// </summary>
    public sealed class RobotPovRendererFeature : FullScreenPassRendererFeature
    {
        public static Camera ActiveCamera { get; private set; }
        static Material activeMaterial;

        public static void Show(Camera camera, Material material)
        {
            ActiveCamera = camera;
            activeMaterial = material;
        }

        public static void Hide(Camera camera)
        {
            if (ActiveCamera != camera) return;
            ActiveCamera = null;
            activeMaterial = null;
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        static void ResetBinding()
        {
            ActiveCamera = null;
            activeMaterial = null;
        }

        public override void Create()
        {
            injectionPoint = InjectionPoint.BeforeRenderingTransparents;
            fetchColorBuffer = false;
            requirements = ScriptableRenderPassInput.None;
            bindDepthStencilAttachment = false;
            base.Create();
        }

        public override void AddRenderPasses(ScriptableRenderer renderer, ref RenderingData renderingData)
        {
            if (!ActiveCamera || !activeMaterial || renderingData.cameraData.camera != ActiveCamera) return;
            passMaterial = activeMaterial;
            var descriptor = renderingData.cameraData.cameraTargetDescriptor;
            // Instanced/multiview XR targets use one array slice per eye.
            passMaterial.SetFloat("_ViewportAspect", (float)descriptor.width / Mathf.Max(1, descriptor.height));
            base.AddRenderPasses(renderer, ref renderingData);
        }
    }
}
