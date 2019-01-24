using UnityEngine;
using System.Collections;

public class FluidCore
{
    /* Safe Release */
    public static void SafeRelease(RenderTexture2D renderTexture)
    {
        if (renderTexture != null)
            renderTexture.Release();
    }
    /* Safe Release */
    public static void SafeRelease(RenderTexture renderTexture)
    {
        if (renderTexture != null)
            renderTexture.Release();
    }




}
