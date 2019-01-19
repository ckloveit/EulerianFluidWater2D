﻿using UnityEngine;
using System.Collections;

public class RenderTexture2D
{
    private RenderTexture renderTexture;
    public RenderTexture Source { get { return renderTexture; } }

    public RenderTexture2D(int xSize,int ySize, string texName = "", RenderTextureFormat renderTextureFormat = RenderTextureFormat.RFloat, bool cpuCanRead = true)
    {
        renderTexture = new RenderTexture(xSize, ySize, 0, renderTextureFormat, RenderTextureReadWrite.Linear);
        renderTexture.enableRandomWrite = cpuCanRead;
        renderTexture.name = texName;
        renderTexture.filterMode = FilterMode.Bilinear;
        renderTexture.Create();
    }

    public void Release()
    {
        if (renderTexture != null)
            renderTexture.Release();
    }

}
