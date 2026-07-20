Shader "Dexter/ButterflyWing"
{
    Properties
    {
        _BaseMap ("Albedo", 2D) = "white" {}
        _AlphaMap ("Alpha", 2D) = "white" {}
        _BumpMap ("Normal", 2D) = "bump" {}
        _Cutoff ("Alpha Cutoff", Range(0, 1)) = 0.05
        _Smoothness ("Smoothness", Range(0, 1)) = 0.45
        _Metallic ("Metallic", Range(0, 1)) = 0
    }

    SubShader
    {
        Tags
        {
            "RenderType" = "TransparentCutout"
            "Queue" = "AlphaTest"
            "RenderPipeline" = "UniversalPipeline"
            "IgnoreProjector" = "True"
        }

        HLSLINCLUDE
        #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
        #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"

        TEXTURE2D(_BaseMap);
        SAMPLER(sampler_BaseMap);
        TEXTURE2D(_AlphaMap);
        SAMPLER(sampler_AlphaMap);

        CBUFFER_START(UnityPerMaterial)
            float4 _BaseMap_ST;
            float4 _AlphaMap_ST;
            float _Cutoff;
            float _Smoothness;
            float _Metallic;
        CBUFFER_END

        struct Attributes
        {
            float4 positionOS : POSITION;
            float3 normalOS : NORMAL;
            float2 uv : TEXCOORD0;
        };

        struct Varyings
        {
            float4 positionCS : SV_POSITION;
            float2 uv : TEXCOORD0;
            float3 normalWS : TEXCOORD1;
            float fogFactor : TEXCOORD2;
        };

        half ButterflyAlpha(float2 uv, half3 albedoRgb)
        {
            float2 alphaUv = TRANSFORM_TEX(uv, _AlphaMap);
            half alphaMask = SAMPLE_TEXTURE2D(_AlphaMap, sampler_AlphaMap, alphaUv).r;
            half albedoLuma = dot(albedoRgb, half3(0.2126, 0.7152, 0.0722));
            return max(alphaMask, saturate(albedoLuma - 0.04h));
        }

        Varyings LitVert(Attributes input)
        {
            Varyings output;
            VertexPositionInputs positionInputs = GetVertexPositionInputs(input.positionOS.xyz);
            VertexNormalInputs normalInputs = GetVertexNormalInputs(input.normalOS);

            output.positionCS = positionInputs.positionCS;
            output.normalWS = normalInputs.normalWS;
            output.uv = input.uv;
            output.fogFactor = ComputeFogFactor(positionInputs.positionCS.z);
            return output;
        }

        half4 LitFrag(Varyings input) : SV_Target
        {
            float2 albedoUv = TRANSFORM_TEX(input.uv, _BaseMap);
            half4 albedo = SAMPLE_TEXTURE2D(_BaseMap, sampler_BaseMap, albedoUv);
            half alpha = ButterflyAlpha(input.uv, albedo.rgb);
            clip(alpha - _Cutoff);

            half3 color = albedo.rgb;
            Light mainLight = GetMainLight();
            half ndotl = saturate(dot(normalize(input.normalWS), mainLight.direction));
            color *= mainLight.color * (ndotl * 0.85h + 0.15h);
            color += SampleSH(input.normalWS) * albedo.rgb;
            color = MixFog(color, input.fogFactor);
            return half4(color, 1);
        }

        Varyings DepthVert(Attributes input)
        {
            Varyings output;
            output.positionCS = TransformObjectToHClip(input.positionOS.xyz);
            output.uv = input.uv;
            output.normalWS = 0;
            output.fogFactor = 0;
            return output;
        }

        half4 DepthFrag(Varyings input) : SV_Target
        {
            float2 albedoUv = TRANSFORM_TEX(input.uv, _BaseMap);
            half3 albedo = SAMPLE_TEXTURE2D(_BaseMap, sampler_BaseMap, albedoUv).rgb;
            clip(ButterflyAlpha(input.uv, albedo) - _Cutoff);
            return 0;
        }
        ENDHLSL

        Pass
        {
            Name "ForwardLit"
            Tags { "LightMode" = "UniversalForwardOnly" }

            Cull Off
            ZWrite On

            HLSLPROGRAM
            #pragma target 3.0
            #pragma vertex LitVert
            #pragma fragment LitFrag
            #pragma multi_compile_fog
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS _MAIN_LIGHT_SHADOWS_CASCADE
            ENDHLSL
        }

        Pass
        {
            Name "ForwardLitCompat"
            Tags { "LightMode" = "UniversalForward" }

            Cull Off
            ZWrite On

            HLSLPROGRAM
            #pragma target 3.0
            #pragma vertex LitVert
            #pragma fragment LitFrag
            #pragma multi_compile_fog
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS _MAIN_LIGHT_SHADOWS_CASCADE
            ENDHLSL
        }

        Pass
        {
            Name "DepthOnly"
            Tags { "LightMode" = "DepthOnly" }

            Cull Off
            ZWrite On
            ColorMask 0

            HLSLPROGRAM
            #pragma target 3.0
            #pragma vertex DepthVert
            #pragma fragment DepthFrag
            ENDHLSL
        }
    }

    FallBack "Hidden/Universal Render Pipeline/FallbackError"
}
