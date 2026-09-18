using UnityEngine;
namespace Deictic
{
    public static class DemoVisuals
    {
        public const string OpaqueTemplate = "Deictic/Materials/UnlitOpaque";
        public const string TransparentTemplate = "Deictic/Materials/UnlitTransparent";

        public static Material Material(Color color)
        {
            // Resource templates retain both URP shader variants in Android builds.
            var template = Resources.Load<Material>(color.a < 1 ? TransparentTemplate : OpaqueTemplate);
            Material material = template ? new Material(template) : null;
#if UNITY_EDITOR
            // The first asset-generation pass has no persisted templates yet.
            if (!material) material = CreateEditorMaterial(color);
#else
            if (!material) throw new System.InvalidOperationException("Missing retained demo material. Run Deictic/Build Simulation Assets before building the player.");
#endif
            material.color = color;
            return material;
        }

#if UNITY_EDITOR
        public static Material CreateEditorMaterial(Color color, bool lit = false)
        {
            // Direct creation avoids loading the asset currently being generated.
            Shader shader = Shader.Find(lit ? "Universal Render Pipeline/Lit" : "Universal Render Pipeline/Unlit");
            if (!shader) throw new System.InvalidOperationException("The demo requires Universal Render Pipeline shaders.");
            var material = new Material(shader);
            material.color = color;
            if (color.a < 1)
            {
                material.SetFloat("_Surface", 1);
                material.SetOverrideTag("RenderType", "Transparent");
                material.SetFloat("_SrcBlend", (float)UnityEngine.Rendering.BlendMode.SrcAlpha);
                material.SetFloat("_DstBlend", (float)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
                material.SetFloat("_SrcBlendAlpha", (float)UnityEngine.Rendering.BlendMode.One);
                material.SetFloat("_DstBlendAlpha", (float)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
                material.SetFloat("_ZWrite", 0);
                material.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
                material.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent;
            }
            return material;
        }
#endif
        public static LineRenderer Line(string name, Color color, float width)
        {
            var line = new GameObject(name).AddComponent<LineRenderer>();
            line.sharedMaterial = Material(color);
            line.startWidth = line.endWidth = width;
            line.positionCount = 0;
            return line;
        }
    }
}
