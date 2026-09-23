Shader "Deictic/Robot POV Fullscreen"
{
    Properties
    {
        _MainTex ("Atomic head-camera pair", 2D) = "black" {}
        _StereoSideBySide ("Left/right stereo", Float) = 1
        _HasFrame ("Fresh camera frame", Float) = 0
        _SourceAspect ("Source eye aspect", Float) = 1.333333
        _ViewportAspect ("Viewport eye aspect", Float) = 1
        _RosTopLeft ("Source row zero is ROS top row", Float) = 1
    }
    SubShader
    {
        Tags { "RenderPipeline"="UniversalPipeline" }
        Pass
        {
            Name "Robot POV"
            Cull Off ZWrite Off ZTest Always Blend Off
            HLSLPROGRAM
            #pragma target 3.5
            #pragma vertex Vert
            #pragma fragment RobotView
            #pragma multi_compile_instancing
            #pragma multi_compile _ STEREO_INSTANCING_ON STEREO_MULTIVIEW_ON
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.core/Runtime/Utilities/Blit.hlsl"

            // A regular 2D SBS source, NOT an XR render-target texture array.
            TEXTURE2D(_MainTex);
            SAMPLER(sampler_MainTex);
            float4 _MainTex_TexelSize;
            float _StereoSideBySide, _HasFrame, _SourceAspect, _ViewportAspect, _RosTopLeft;

            half4 RobotView(Varyings input) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(input);
                // Opaque fallback covers passthrough/the scene, never a frozen
                // old image. The world-fixed status/return UI draws afterward.
                if (_HasFrame < .5) return half4(.008, .012, .02, 1);
                float2 uv = input.texcoord;
                // Raw ROS pixels need a flip; LoadImage JPEG textures already
                // have Unity's bottom-left origin. Avoid a CPU row copy/upload.
                if (_RosTopLeft > .5) uv.y = 1 - uv.y;
                // Center crop to fill the entire eye without changing aspect.
                float sourceAspect = max(.001, _SourceAspect);
                float viewportAspect = max(.001, _ViewportAspect);
                float2 scale = float2(min(1, viewportAspect / sourceAspect), min(1, sourceAspect / viewportAspect));
                uv = (uv - .5) * scale + .5;
                float eye = _StereoSideBySide > .5 ? unity_StereoEyeIndex : 0;
                float eyeWidth = _StereoSideBySide > .5 ? .5 : 1;
                float left = eye * eyeWidth;
                uv.x = clamp(uv.x * eyeWidth + left,
                    left + .5 * _MainTex_TexelSize.x,
                    left + eyeWidth - .5 * _MainTex_TexelSize.x);
                uv.y = clamp(uv.y, .5 * _MainTex_TexelSize.y, 1 - .5 * _MainTex_TexelSize.y);
                return half4(SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, uv).rgb, 1);
            }
            ENDHLSL
        }
    }
}
