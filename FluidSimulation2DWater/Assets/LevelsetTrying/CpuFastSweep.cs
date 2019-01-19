using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CpuFastSweep : MonoBehaviour
{
    public float colorScale = 1f;

    public const int n = 256;
    float[][] sdf;
    float gridSpacing = 1f / n;
    const float max = 10000f;
    Texture2D result;

    // Use this for initialization
    void Start()
    {

        sdf = new float[n + 2][];
        {
            for (int i = 0; i < sdf.Length; ++i)
            {
                sdf[i] = new float[n + 2];
                for (int j = 0; j < sdf[i].Length; ++j)
                    sdf[i][j] = max;
            }
        }
        sdf[n / 2 + 1][n / 2 + 1] = 0;

        {
            int i = 0, j = 0;
            int istep = 0, jstep = 0;
            int istart = 0, jstart = 0;
            float a = 0f, b = 0f, ubar = max;
            for (int k = 0; k < 4; ++k)
            {
                if (k == 0) { istart = n; jstart = n; istep = -1; jstep = -1; }
                if (k == 1) { istart = n; jstart = 1; istep = -1; jstep = 1; }
                if (k == 2) { istart = 1; jstart = n; istep = 1; jstep = -1; }
                if (k == 3) { istart = 1; jstart = 1; istep = 1; jstep = 1; }
                for (i = istart; i >= 1 && i <= n; i += istep)
                    for (j = jstart; j >= 1 && j <= n; j += jstep)
                    {
                        a = Mathf.Min(sdf[i - 1][j], sdf[i + 1][j]);
                        b = Mathf.Min(sdf[i][j - 1], sdf[i][j + 1]);
                        if (Mathf.Abs(a - b) >= gridSpacing)
                            ubar = Mathf.Min(a, b) + gridSpacing;
                        else
                            ubar = (a + b + Mathf.Sqrt(2f * gridSpacing * gridSpacing - (a - b) * (a - b))) * 0.5f;

                        sdf[i][j] = Mathf.Min(ubar, sdf[i][j]);
                    }
            }
        }

        result = new Texture2D(n, n, TextureFormat.ARGB32, false, true);
    }

    private void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        {
            for (int i = 1; i <= n; ++i)
            {
                for (int j = 1; j <= n; ++j)
                {
                    result.SetPixel(i - 1, j - 1, new Color(sdf[i][j] * colorScale, sdf[i][j] * colorScale, sdf[i][j] * colorScale));
                    //result.SetPixel(i - 1, j - 1, Color.red);
                }
            }
        }
        result.Apply();
        Graphics.Blit(result, destination);
        //Graphics.Blit(null, destination);
    }

    private void OnDestroy()
    {
        DestroyImmediate(result);
    }
}

