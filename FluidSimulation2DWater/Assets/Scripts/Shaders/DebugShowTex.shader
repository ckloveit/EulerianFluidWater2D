Shader "FluidSimulation/DebugShowTex"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                UNITY_FOG_COORDS(1)
                float4 vertex : SV_POSITION;
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;
			float _GridSize;
            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = TRANSFORM_TEX(v.uv, _MainTex);
                return o;
            }

            float4 frag (v2f i) : SV_Target
            {
                // sample the texture
				float4 col = 0;
				float value = tex2D(_MainTex, i.uv).r;
				/*if (value == 1)
				{
					col.rgb = float3(1,0,0);
				}
				else if (value = 0)
				{
					col.rgb = float3(0,0,0.5);
				}
				else*/
				{
					/*if (value < 0)
						col.rgb = 0.5;
					else*/
					//if (value == 1)
					//{
					//	col.rgb = float3(1, 0, 0);
					//}
					//else if (value == 2)
					//{
					//	col.rgb = float3(1, 1, 0);
					//}
					//else if (value == 0)
					//{
					//	col.rgb = float3(0, 1, 0);//(value);////value;//

					//}
					int shade = int(abs(-value) * 255.0);
					if (value <= 0)
					{
						col.rgb = float3(1, 1, 1) * abs(value);
					}
					//else
					//{
					//	col.rgb = float3(1, 0, 0) * abs(value / 50);
					//}
					//col.rgb =  (-value / 10);
				}

                return col;
            }
            ENDCG
        }
    }
}
