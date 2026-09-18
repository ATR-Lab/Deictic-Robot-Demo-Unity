Shader "Deictic/Camera View Overlay"
{
    Properties
    {
        [PerRendererData] _MainTex ("Texture", 2D) = "white" {}
        _Color ("Tint", Color) = (1,1,1,1)
        [Toggle] _StereoSideBySide ("Left/right head stereo", Float) = 0
        [Enum(UnityEngine.Rendering.CompareFunction)] _ZTest ("Depth test", Float) = 4
    }
    SubShader
    {
        Tags { "Queue"="Transparent" "RenderType"="Transparent" "IgnoreProjector"="True" "CanUseSpriteAtlas"="True" "RenderPipeline"="UniversalPipeline" }
        Cull Off
        ZWrite Off
        ZTest [_ZTest]
        Blend SrcAlpha OneMinusSrcAlpha
        Pass
        {
            HLSLPROGRAM
            #pragma target 3.5
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            #pragma multi_compile _ STEREO_INSTANCING_ON STEREO_MULTIVIEW_ON
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            struct Attributes
            {
                float4 vertex : POSITION;
                float4 color : COLOR;
                float2 uv : TEXCOORD0;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };
            struct Varyings
            {
                float4 vertex : SV_POSITION;
                half4 color : COLOR;
                float2 uv : TEXCOORD0;
                UNITY_VERTEX_OUTPUT_STEREO
            };
            // This is a normal SBS texture, not an XR render-target array.
            TEXTURE2D(_MainTex);
            SAMPLER(sampler_MainTex);
            float4 _MainTex_TexelSize;
            half4 _Color;
            half4 _TextureSampleAdd;
            float _StereoSideBySide;
            Varyings vert(Attributes input)
            {
                Varyings output = (Varyings)0;
                UNITY_SETUP_INSTANCE_ID(input);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(output);
                output.vertex = TransformObjectToHClip(input.vertex.xyz);
                output.color = input.color * _Color;
                output.uv = input.uv;
                return output;
            }
            half4 frag(Varyings input) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(input);
                float2 uv = input.uv;
                if (_StereoSideBySide > .5)
                {
                    // The desktop/mono camera shows the left eye. XR selects
                    // the corresponding half for every draw instance/view.
                    float eye = unity_StereoEyeIndex;
                    float left = eye * .5;
                    uv.x = clamp(uv.x * .5 + left,
                        left + .5 * _MainTex_TexelSize.x,
                        left + .5 - .5 * _MainTex_TexelSize.x);
                }
                return (SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, uv) + _TextureSampleAdd) * input.color;
            }
            ENDHLSL
        }
    }
}
