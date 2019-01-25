using UnityEngine;
using System.Collections;

public class MultiGridLinearSolver2D
{
    public enum RestrictType
    {
        Normal, // restrict for normal quantity (float type)
        Marker  // restrict for cell marker quantity (uint type)
    }
    private const int mGroupThreadSizeX = 8;

    private ComputeShader mComputePressure;
    private ComputeShader mComputeRestrict;
    private ComputeShader mComputeResidual;
    private ComputeShader mComputePrologation;
    private ComputeShader mComputeCorrect;

    private RenderTexture2D[] mResidualTexArray;
    private RenderTexture2D[] mErrorTexArray;
    private RenderTexture2D[] mErrorTexSwapArray;
    private RenderTexture2D[] mRightPoissonTexArray;

    private RenderTexture2D[] mCellMakerArray;
    private RenderTexture2D[] mLevelsetArray;

    private int mMaxLevel = 0;

    public static MultiGridLinearSolver2D Build(int maxLevel,int finerWidth,int finerHeight,
        RenderTexture2D level0Pressure,RenderTexture2D level0PressureSwap,
        RenderTexture2D level0Marker,RenderTexture2D level0Divergence,
        RenderTexture2D level0Levelset)
    {
        MultiGridLinearSolver2D multiGridSolver = new MultiGridLinearSolver2D();
        multiGridSolver.mMaxLevel = maxLevel;
        multiGridSolver.mCellMakerArray       = new RenderTexture2D[maxLevel];
        multiGridSolver.mResidualTexArray     = new RenderTexture2D[maxLevel];
        multiGridSolver.mErrorTexArray        = new RenderTexture2D[maxLevel];
        multiGridSolver.mErrorTexSwapArray    = new RenderTexture2D[maxLevel];
        multiGridSolver.mRightPoissonTexArray = new RenderTexture2D[maxLevel];
        multiGridSolver.mLevelsetArray        = new RenderTexture2D[maxLevel];
        // sets level0 
        multiGridSolver.mRightPoissonTexArray[0] = level0Divergence;
        multiGridSolver.mCellMakerArray[0] = level0Marker;
        multiGridSolver.mResidualTexArray[0] = new RenderTexture2D(finerWidth, finerHeight, "MG_Residual" + 0.ToString());
        multiGridSolver.mErrorTexArray[0] = level0Pressure;
        multiGridSolver.mErrorTexSwapArray[0] = level0PressureSwap;
        multiGridSolver.mLevelsetArray[0] = level0Levelset;
        for (int i = 1;i<maxLevel;i++)
        {
            int currentWidth  = finerWidth >> i;
            int currentHeight = finerHeight >> i;
            multiGridSolver.mCellMakerArray[i] = new RenderTexture2D(currentWidth, currentHeight, "MG_CellMarker" + i.ToString(), RenderTextureFormat.RInt);
            multiGridSolver.mLevelsetArray[i] = new RenderTexture2D(currentWidth, currentHeight, "MG_Levelset" + i.ToString());
            multiGridSolver.mResidualTexArray[i] = new RenderTexture2D(currentWidth, currentHeight, "MG_Residual" + i.ToString());
            multiGridSolver.mErrorTexArray[i] = new RenderTexture2D(currentWidth, currentHeight, "MG_Error" + i.ToString());
            multiGridSolver.mErrorTexSwapArray[i] = new RenderTexture2D(currentWidth, currentHeight, "MG_ErrorSwap" + i.ToString());
            multiGridSolver.mRightPoissonTexArray[i] = new RenderTexture2D(currentWidth, currentHeight, "MG_RightPoisson" + i.ToString());
        }
        return multiGridSolver;
    }

    public void SetMGComputeShaders(ComputeShader computePressure, ComputeShader computeRestrict, 
        ComputeShader computeResidual,ComputeShader computePrologation,
        ComputeShader computeCorrect)
    {
        mComputePressure = computePressure;
        mComputeRestrict = computeRestrict;
        mComputeResidual = computeResidual;
        mComputePrologation = computePrologation;
        mComputeCorrect = computeCorrect;
    }

    public void MultigridSolveVCycle()
    {
        VCycle(0);
        // do more smoothing
        Smooth(0, 2.0f / 3.0f, 20);
    }

    public void MultigridSolveFMG()
    {
        for(int curLevel = 0; curLevel < mMaxLevel - 2; curLevel++)
        {
            Residual(curLevel);
            // Restrict
            Restrict(mResidualTexArray[curLevel], mRightPoissonTexArray[curLevel + 1], RestrictType.Normal);
            Restrict(mCellMakerArray[curLevel], mCellMakerArray[curLevel + 1], RestrictType.Marker);
            Restrict(mLevelsetArray[curLevel], mLevelsetArray[curLevel + 1], RestrictType.Normal);
        }
        Smooth(mMaxLevel - 1, 1.0f, 20); // caution : if we iterator 5 ,the error is large,so use the Multigrid seem can't solve
        for(int curLevel =mMaxLevel - 2; curLevel >= 0; curLevel--)
        {
            // Prologation
            Prologation(curLevel, mErrorTexArray[curLevel + 1], mErrorTexSwapArray[curLevel]);
            VCycle(curLevel);
        }
        VCycle(0);
    }


    private void VCycle(int curLevel)
    {
        // PreSmoothing
        Smooth(curLevel, 1.0f, 20); // caution : if we iterator 5 ,the error is large,so use the Multigrid seem can't solve
        // Compute Residual
        Residual(curLevel);

        // Restrict
        Restrict(mResidualTexArray[curLevel], mRightPoissonTexArray[curLevel + 1], RestrictType.Normal);
        Restrict(mCellMakerArray[curLevel], mCellMakerArray[curLevel + 1], RestrictType.Marker);
        Restrict(mLevelsetArray[curLevel], mLevelsetArray[curLevel + 1], RestrictType.Normal);

        if (curLevel == mMaxLevel - 2)
        {
            // smooth
            Smooth(curLevel + 1, 1.0f, 20);
        }
        else
        {
            VCycle(curLevel + 1);
        }

        // Prologation
        Prologation(curLevel, mErrorTexArray[curLevel + 1], mErrorTexSwapArray[curLevel]);
        // correct
        Correct(mErrorTexArray[curLevel], mErrorTexSwapArray[curLevel]);
        // post-smoothing
        Smooth(curLevel, 1.0f, 20);

    }

    public void UpdateLevel0Tex(RenderTexture2D level0Levelset)
    {
        this.mLevelsetArray[0] = level0Levelset;
    }

    private void Smooth(int curLevel, float SORWeight, int numLoop)
    {
        int jacobiSORKernel = mComputePressure.FindKernel("ComputeJacobiPressureSOR");
        mComputePressure.SetFloat("_SORWeight", SORWeight);
        int WIDTH = mErrorTexArray[curLevel].Width;
        int HEIGHT = mErrorTexArray[curLevel].Height;
        mComputePressure.SetInt("_currentXSize", WIDTH);
        mComputePressure.SetInt("_currentYSize", HEIGHT);
        mComputePressure.SetInt("_currentMGLevel", curLevel);
        RenderTexture2D pressureRead = mErrorTexArray[curLevel];
        RenderTexture2D pressureWrite = mErrorTexSwapArray[curLevel];

        //int clearTexKernl = mComputePressure.FindKernel("ClearTex");
        //mComputePressure.SetTexture(clearTexKernl, "gPressure", pressureRead.Source);
        //mComputePressure.Dispatch(clearTexKernl, Mathf.CeilToInt(WIDTH * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(HEIGHT * 1.0f / mGroupThreadSizeX), 1);

        for (int i = 0; i < numLoop; i++)
        {
            pressureRead = i % 2 == 0 ? mErrorTexArray[curLevel] : mErrorTexSwapArray[curLevel];
            pressureWrite = i % 2 == 0 ? mErrorTexSwapArray[curLevel] : mErrorTexArray[curLevel];
            mComputePressure.SetTexture(jacobiSORKernel, "gPressure", pressureRead.Source);
            mComputePressure.SetTexture(jacobiSORKernel, "gPressureWrite", pressureWrite.Source);
            mComputePressure.SetTexture(jacobiSORKernel, "gVelocityDivergence", mRightPoissonTexArray[curLevel].Source);
            mComputePressure.SetTexture(jacobiSORKernel, "gLevelSet", mLevelsetArray[curLevel].Source);
            mComputePressure.SetTexture(jacobiSORKernel, "gCurentLevelGridMarker", mCellMakerArray[curLevel].Source);
            mComputePressure.Dispatch(jacobiSORKernel, Mathf.CeilToInt(WIDTH * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(HEIGHT * 1.0f / mGroupThreadSizeX), 1);
        }
        if (pressureRead != mErrorTexArray[curLevel])
            Graphics.CopyTexture(pressureRead.Source, mErrorTexArray[curLevel].Source);

    }

    private void Residual(int curLevel)
    {
        int MGResidualKernel = mComputeResidual.FindKernel("MGResidual");
        int WIDTH = mErrorTexArray[curLevel].Width;
        int HEIGHT = mErrorTexArray[curLevel].Height;
        mComputeResidual.SetInt("_currentXSize", WIDTH);
        mComputeResidual.SetInt("_currentYSize", HEIGHT);

        mComputeResidual.SetTexture(MGResidualKernel, "gRightPoissonTex", mRightPoissonTexArray[curLevel].Source);
        mComputeResidual.SetTexture(MGResidualKernel, "gCurrentTex", mErrorTexArray[curLevel].Source);
        mComputeResidual.SetTexture(MGResidualKernel, "gResidualTex", mResidualTexArray[curLevel].Source);
        mComputeResidual.SetTexture(MGResidualKernel, "gCurentLevelGridMarker", mCellMakerArray[curLevel].Source);
        mComputeResidual.SetTexture(MGResidualKernel, "gCurrentLevelLevelSet", mLevelsetArray[curLevel].Source);
        mComputeResidual.Dispatch(MGResidualKernel, Mathf.CeilToInt(WIDTH * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(HEIGHT * 1.0f / mGroupThreadSizeX), 1);

    }

    private void Restrict(RenderTexture2D sourceTex,RenderTexture2D destTex, RestrictType restrictType)
    {
        if(restrictType == RestrictType.Normal)
        {
            int MGRestrictionHalfWeightingKernel = mComputeRestrict.FindKernel("MGRestrictionHalfWeighting");
            int WIDTH = destTex.Width;
            int HEIGHT = destTex.Height;

            mComputeRestrict.SetInt("_currentXSize", WIDTH);
            mComputeRestrict.SetInt("_currentYSize", HEIGHT);
            mComputeRestrict.SetTexture(MGRestrictionHalfWeightingKernel, "gSourceTex", sourceTex.Source);
            mComputeRestrict.SetTexture(MGRestrictionHalfWeightingKernel, "gDestinationHalfTex", destTex.Source);
            mComputeRestrict.Dispatch(MGRestrictionHalfWeightingKernel, Mathf.CeilToInt(WIDTH * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(HEIGHT * 1.0f / mGroupThreadSizeX), 1);

        }else if(restrictType == RestrictType.Marker)
        {
            int MGRestrictionForCellMarkerKernel = mComputeRestrict.FindKernel("MGRestrictionForCellMarker");
            int WIDTH = destTex.Width;
            int HEIGHT = destTex.Height;

            mComputeRestrict.SetInt("_currentXSize", WIDTH);
            mComputeRestrict.SetInt("_currentYSize", HEIGHT);
            mComputeRestrict.SetTexture(MGRestrictionForCellMarkerKernel, "gSourceMarkerTex", sourceTex.Source);
            mComputeRestrict.SetTexture(MGRestrictionForCellMarkerKernel, "gDestinationHalfMarkerTex", destTex.Source);
            mComputeRestrict.Dispatch(MGRestrictionForCellMarkerKernel, Mathf.CeilToInt(WIDTH * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(HEIGHT * 1.0f / mGroupThreadSizeX), 1);

        }

    }

    private void Prologation(int destLevel,RenderTexture2D sourceHalfTex,RenderTexture2D destFullTex)
    {
        int MGProlongationKernel = mComputePrologation.FindKernel("MGProlongation");
        int WIDTH = destFullTex.Width;
        int HEIGHT = destFullTex.Height;
        mComputePrologation.SetInt("_currentXSize", WIDTH);
        mComputePrologation.SetInt("_currentYSize", HEIGHT);

        mComputePrologation.SetTexture(MGProlongationKernel, "gSourceHalfTex", sourceHalfTex.Source);
        mComputePrologation.SetTexture(MGProlongationKernel, "gDestinationFullTex", destFullTex.Source);
        mComputePrologation.SetTexture(MGProlongationKernel, "gGridMarker", mCellMakerArray[destLevel].Source);
        mComputePrologation.Dispatch(MGProlongationKernel, Mathf.CeilToInt(WIDTH * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(HEIGHT * 1.0f / mGroupThreadSizeX), 1);

    }

    private void Correct(RenderTexture2D curNeedCorrectTex, RenderTexture2D errorTex)
    {
        int MGCorrectKernel = mComputeCorrect.FindKernel("MGCorrect");
        int WIDTH = curNeedCorrectTex.Width;
        int HEIGHT = curNeedCorrectTex.Height;
        mComputeCorrect.SetInt("_currentXSize", WIDTH);
        mComputeCorrect.SetInt("_currentYSize", HEIGHT);

        mComputeCorrect.SetTexture(MGCorrectKernel, "gNeedCorrectTex", curNeedCorrectTex.Source);
        mComputeCorrect.SetTexture(MGCorrectKernel, "gErrorTex", errorTex.Source);
        mComputeCorrect.Dispatch(MGCorrectKernel, Mathf.CeilToInt(WIDTH * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(HEIGHT * 1.0f / mGroupThreadSizeX), 1);

    }

    private void Swap(RenderTexture2D tex0,RenderTexture2D tex1)
    {
        RenderTexture2D temp = tex0;
        tex0 = tex1;
        tex1 = temp;
    }

    public void DebugComputePressure()
    {
        int jacobiKernel = mComputePressure.FindKernel("ComputeJacobiPressureSOR");
        mComputePressure.SetFloat("_SORWeight", 1.0f);
        int WIDTH = mErrorTexArray[0].Width;
        int HEIGHT = mErrorTexArray[0].Height;
        mComputePressure.SetInt("_currentXSize", WIDTH);
        mComputePressure.SetInt("_currentYSize", HEIGHT);
        // 1. compute pressure
        int numLoop = 100;
        RenderTexture2D pressureRead = mErrorTexArray[0];
        RenderTexture2D pressureWrite = mErrorTexSwapArray[0];
        for (int i = 0; i < numLoop; i++)
        {
            pressureRead = i % 2 == 0 ? mErrorTexArray[0] : mErrorTexSwapArray[0];
            pressureWrite = i % 2 == 0 ? mErrorTexSwapArray[0] : mErrorTexArray[0];
            mComputePressure.SetTexture(jacobiKernel, "gPressure", pressureRead.Source);
            mComputePressure.SetTexture(jacobiKernel, "gPressureWrite", pressureWrite.Source);
            mComputePressure.SetTexture(jacobiKernel, "gVelocityDivergence", mRightPoissonTexArray[0].Source);
            mComputePressure.SetTexture(jacobiKernel, "gLevelSet", mLevelsetArray[0].Source);
            mComputePressure.SetTexture(jacobiKernel, "gCurentLevelGridMarker", mCellMakerArray[0].Source);
            mComputePressure.Dispatch(jacobiKernel, Mathf.CeilToInt(mErrorTexArray[0].Width * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mErrorTexArray[0].Height * 1.0f / mGroupThreadSizeX), 1);
            //Swap(pressureRead, pressureWrite);
        }
        if (pressureRead != mErrorTexArray[0])
            Graphics.CopyTexture(pressureRead.Source, mErrorTexArray[0].Source);
    }

    public void Release()
    {
        for(int i=0;i<mMaxLevel;i++)
        {
            FluidCore.SafeRelease(mResidualTexArray[i]);
            FluidCore.SafeRelease(mErrorTexArray[i]);
            FluidCore.SafeRelease(mErrorTexSwapArray[i]);
            FluidCore.SafeRelease(mRightPoissonTexArray[i]);
            FluidCore.SafeRelease(mCellMakerArray[i]);
            FluidCore.SafeRelease(mLevelsetArray[i]);
        }
    }
}
