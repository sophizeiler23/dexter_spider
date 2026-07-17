Shader "Dexter/SpiderSilk"
{
    Properties
    {
        _BaseColor ("Base Color", Color) = (0.92, 0.96, 1, 0.42)
        _IridescenceStrength ("Iridescence Strength", Range(0, 1)) = 0.55
        _FresnelStrength ("Fresnel Strength", Range(0, 2)) = 0.75
        _EdgeSoftness ("Edge Softness", Range(0, 4)) = 1.6
    }

    SubShader
    {
        Tags
        {
            "RenderType" = "Transparent"
            "Queue" = "Transparent"
            "RenderPipeline" = "UniversalPipeline"
        }

        Pass
        {
            Name "ForwardLit"
            Tags { "LightMode" = "UniversalForward" }

            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite Off
            Cull Off

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_fog
            #pragma target 3.0

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            struct Attributes
            {
                float4 positionOS : POSITION;
                float3 normalOS : NORMAL;
                float4 color : COLOR;
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float3 normalWS : TEXCOORD0;
                float3 viewDirWS : TEXCOORD1;
                float4 color : COLOR;
                float fogFactor : TEXCOORD2;
            };

            CBUFFER_START(UnityPerMaterial)
                float4 _BaseColor;
                float _IridescenceStrength;
                float _FresnelStrength;
                float _EdgeSoftness;
            CBUFFER_END

            float3 IridescentTint(float viewDotNormal)
            {
                float wave = viewDotNormal * 2.4 + 0.2;
                float3 cyan = float3(0.15, 0.75, 1.0);
                float3 magenta = float3(0.9, 0.2, 0.75);
                float3 orange = float3(1.0, 0.55, 0.15);
                float3 tint = lerp(cyan, magenta, smoothstep(0.15, 0.55, wave));
                tint = lerp(tint, orange, smoothstep(0.55, 0.95, wave));
                return tint;
            }

            Varyings vert(Attributes input)
            {
                Varyings output;
                VertexPositionInputs positionInputs = GetVertexPositionInputs(input.positionOS.xyz);
                VertexNormalInputs normalInputs = GetVertexNormalInputs(input.normalOS);

                output.positionCS = positionInputs.positionCS;
                output.normalWS = normalInputs.normalWS;
                output.viewDirWS = GetWorldSpaceViewDir(positionInputs.positionWS);
                output.color = input.color;
                output.fogFactor = ComputeFogFactor(positionInputs.positionCS.z);
                return output;
            }

            half4 frag(Varyings input) : SV_Target
            {
                float3 normalWS = normalize(input.normalWS);
                float3 viewDirWS = normalize(input.viewDirWS);
                float viewDotNormal = saturate(abs(dot(normalWS, viewDirWS)));

                float fresnel = pow(1.0 - viewDotNormal, 2.0) * _FresnelStrength;
                float3 iridescence = IridescentTint(viewDotNormal) * _IridescenceStrength;

                float3 baseRgb = _BaseColor.rgb * input.color.rgb;
                float3 finalRgb = baseRgb + iridescence + fresnel * float3(0.8, 0.9, 1.0);
                float alpha = saturate(_BaseColor.a * input.color.a + fresnel * 0.2);

                float alphaGradient = max(fwidth(alpha), 0.001);
                alpha = smoothstep(0.0, alphaGradient * _EdgeSoftness, alpha);

                half4 color = half4(finalRgb, alpha);
                color.rgb = MixFog(color.rgb, input.fogFactor);
                return color;
            }
            ENDHLSL
        }
    }

    FallBack Off
}
