using UnityEngine;
using System.Collections;

public class FluidSolver2D : MonoBehaviour
{
    private int READ = 0;
    private int WRITE = 1;
    private const int mGroupThreadSizeX = 8;

    [HideInInspector]
    public ComputeShader mComputeInitialize; // fluid initialize status
    [HideInInspector]
    public ComputeShader mComputeFluidEmitter;// sphere liquid emitter
    [HideInInspector]
    public ComputeShader mComputeDivergence; // compute divergence
    [HideInInspector]
    public ComputeShader mComputeProjection; // projection velocity
    [HideInInspector]
    public ComputeShader mComputeExtrapolateToAir; // extropolate velocity to air
    [HideInInspector]
    public ComputeShader mComputeAdvect;// advect 
    [HideInInspector]
    public ComputeShader mComputeAddForces; // add forces
    [HideInInspector]
    public ComputeShader mComputeRedistanceLevelset;// redistance levelset
    [HideInInspector]
    public ComputeShader mComputeKeepBoundary; // keep boundary process
    [HideInInspector]
    public ComputeShader mComputeUpdateFluidMarker;// update grid marker
    [HideInInspector]
    public ComputeShader mComputeCopyTex; // copy texture


    /* Fluid quantities */
    private RenderTexture2D[] mVelocityU;
    private RenderTexture2D[] mVelocityV;
    private RenderTexture2D[] mLevelSet;

    /* Fluid Marker */
    private RenderTexture2D mGridMarker;
    private RenderTexture2D[] mGridValidU;
    private RenderTexture2D[] mGridValidV;

    /* Middle Result  */
    private RenderTexture2D mDivergence;
    private RenderTexture2D mPressure;

    /* Width and height */
    public int mWidth;
    public int mHeight;

    /* Grid cell size and fluid density */
    private float mGridSize;
    public float mDensity = 1.0f;

    /* for emitter */
    private uint mEmitterCounter = 0;
    private uint mCounter = 0;

    /* for fastsweep redistance levelset */
    private uint mRedistanceCounter = 0;
    private uint mFsmNumDir = 4;
    private int[,] mFsmConfig;

    /* for standard redistaning levelset */
    private int mRedistanceLevelsetCounter = 0;



    /* Show debug */
    public bool enabledDebug = true;
    public Material debugShowTexMaterial;

    public int mSimulateFPS = 10;

    public bool mEnabledRedistanceLevelset = true;

    private System.DateTime currentTime;
    private float simulateStepSeconds;

    private void Start()
    {
        InitResource(mWidth, mHeight, mDensity, mSimulateFPS);
    }

    public void InitResource(int _width,int _height,float density, float numSimulateFPS = 10)
    {
        mWidth = _width;
        mHeight = _height;
        mGridSize = 1.0f / Mathf.Min(mWidth, mHeight);
        mLevelSet = new RenderTexture2D[2];
        mVelocityU = new RenderTexture2D[2];
        mVelocityV = new RenderTexture2D[2];

        mLevelSet[READ] = new RenderTexture2D(mWidth, mHeight,"levelset0");
        mLevelSet[WRITE] = new RenderTexture2D(mWidth, mHeight, "levelset0");
        mVelocityU[READ] = new RenderTexture2D(mWidth + 1, mHeight, "velocityU0");
        mVelocityU[WRITE] = new RenderTexture2D(mWidth + 1, mHeight, "velocityU1");
        mVelocityV[READ] = new RenderTexture2D(mWidth, mHeight + 1, "velocityV0");
        mVelocityV[WRITE] = new RenderTexture2D(mWidth, mHeight + 1, "velocityV1");

        mDivergence = new RenderTexture2D(mWidth, mHeight,"divergence");
        mPressure = new RenderTexture2D(mWidth, mHeight,"pressure");

        mGridMarker = new RenderTexture2D(mWidth, mHeight,"gridMarker",RenderTextureFormat.RInt);
        mGridValidU = new RenderTexture2D[2];
        mGridValidV = new RenderTexture2D[2];

        mGridValidU[READ]   = new RenderTexture2D(mWidth + 1, mHeight, "gridValidU0", RenderTextureFormat.RInt);
        mGridValidU[WRITE]  = new RenderTexture2D(mWidth + 1, mHeight, "gridValidU1", RenderTextureFormat.RInt);
        mGridValidV[READ]   = new RenderTexture2D(mWidth, mHeight + 1, "gridValidV0",RenderTextureFormat.RInt);
        mGridValidV[WRITE]  = new RenderTexture2D(mWidth, mHeight + 1, "gridValidV1", RenderTextureFormat.RInt);

        currentTime = System.DateTime.Now;
        simulateStepSeconds = 1.0f / numSimulateFPS;

        // init fast sweep
        mFsmConfig = new int[4 , 4]
        {
            {1,          mWidth,           1,  mHeight },
            {mWidth - 2,     -1,           1,  mHeight },
            {1,          mWidth, mHeight - 2,       -1 },
            {mWidth - 2,    -1,  mHeight - 2,       -1 },
        };

        // init start states
        InitFluidStartStage();

    }

    private void UploadGlobalParameters(ComputeShader compute)
    {
        compute.SetInt("_xSize", mWidth);
        compute.SetInt("_ySize", mHeight);
        compute.SetFloat("_timeStep", Time.deltaTime);//compute.SetFloat("_timeStep", simulateStepSeconds);//
        compute.SetFloat("_gridSpace", mGridSize);
        compute.SetFloat("_density", mDensity);
    }

    private void InitFluidStartStage()
    {
        int initKernel = mComputeInitialize.FindKernel("InitFluid");
        UploadGlobalParameters(mComputeInitialize);
        mComputeInitialize.SetTexture(initKernel, "gLevelSet", mLevelSet[READ].Source);
        mComputeInitialize.SetTexture(initKernel, "gVelocityU", mVelocityU[READ].Source);
        mComputeInitialize.SetTexture(initKernel, "gVelocityV", mVelocityV[READ].Source);
        mComputeInitialize.Dispatch(initKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);

        UpdateFluidGridMarker();//
    }

    // nSteps : every frame 
    public void Simulate() 
    {
        System.DateTime nowTime = System.DateTime.Now;
        double seconds = nowTime.Subtract(currentTime).TotalSeconds;
        int simulateSteps = (int)(seconds / simulateStepSeconds);
        currentTime = currentTime.AddSeconds(simulateSteps * simulateStepSeconds);
        if(simulateSteps>0)
        {
            for(int i=0;i<simulateSteps;i++)
            {
                SimulateStep();
            }
        }
        /// pseudo time to reinitialize levelset
    }
    private void Update()
    {
        //Simulate();
        SimulateStep();
    }

    private void SimulateStep()
    {
        // for emitter
        if(mEmitterCounter++ % 500 == 0)
        {
            mCounter++;
            if (mCounter <= 4)
                EmitterFluid();
        }
        // 1. add forces
        AddExternalForcesGPU();
        // 2. keep boundary
        // 2.1 process boundary for levelset
        KeepBoundaryForLevelset();
        //// 3. update grid marker
        UpdateFluidGridMarker();
        // 4. kepp boundary for velocity
        KeepBoundaryForVelocity();


        //5.applying project
        //5.1 calculate divergence
        ComputeDivergence();
        // 5.2 calculate pressure (use jaccobin , red-black gauss-sibel,multi-grid ),subtract divergence pressure
        ComputePressure();

        //// 6. extrapolate velocity to air with two grid(need to improve)
        ExtrapolateVelocityToAirGPU();

        // 7. keep boundary velocity
        KeepBoundaryForVelocity();

        // 8. advect quantities
        AdvectGPU();

        KeepBoundaryForVelocity();
        // 5. redistance levelset
        RedistanceLevelSet();
        // 6. volume rendering

        Swap();
    }
    private void EmitterFluid()
    {
        int SphereEmitterKernel = mComputeFluidEmitter.FindKernel("SphereEmitter");
        UploadGlobalParameters(mComputeFluidEmitter);
        mComputeFluidEmitter.SetTexture(SphereEmitterKernel, "gLevelSet", mLevelSet[READ].Source);
        mComputeFluidEmitter.SetTexture(SphereEmitterKernel, "gVelocityU", mVelocityU[READ].Source);
        mComputeFluidEmitter.SetTexture(SphereEmitterKernel, "gVelocityV", mVelocityV[READ].Source);
        mComputeFluidEmitter.SetFloat("sphereEmitterPosX", (0.5f /*+ 0.8f * Random.value*/) * mWidth);
        mComputeFluidEmitter.SetFloat("sphereEmitterPosY", (0.8f /*+ 0.4f * Random.value*/) * mHeight);
        mComputeFluidEmitter.SetFloat("sphereEmitterRadius", 2 * 0.08f * Mathf.Min(mWidth, mHeight));
        mComputeFluidEmitter.Dispatch(SphereEmitterKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);
    }

    private void UpdateFluidGridMarker()
    {
        UploadGlobalParameters(mComputeUpdateFluidMarker);
        // 1. update Grid marker
        int UpdateGridMarkerKernel = mComputeUpdateFluidMarker.FindKernel("UpdateGridMarker");
        mComputeUpdateFluidMarker.SetTexture(UpdateGridMarkerKernel, "gLevelSet", mLevelSet[READ].Source);
        mComputeUpdateFluidMarker.SetTexture(UpdateGridMarkerKernel, "gGridMarker", mGridMarker.Source);
        mComputeUpdateFluidMarker.Dispatch(UpdateGridMarkerKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);
    }
    private void KeepBoundaryForLevelset()
    {
        // 2. process level set boundary
        UploadGlobalParameters(mComputeKeepBoundary);
        int ProcessLevelsetBoundaryKernel = mComputeKeepBoundary.FindKernel("ProcessLevelsetBoundary");
        mComputeKeepBoundary.SetTexture(ProcessLevelsetBoundaryKernel, "gLevelSetRead", mLevelSet[READ].Source);
        mComputeKeepBoundary.SetTexture(ProcessLevelsetBoundaryKernel, "gLevelSetWrite", mLevelSet[WRITE].Source);
        mComputeKeepBoundary.SetTexture(ProcessLevelsetBoundaryKernel, "gGridMarker", mGridMarker.Source);
        mComputeKeepBoundary.Dispatch(ProcessLevelsetBoundaryKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);
        Graphics.CopyTexture(mLevelSet[WRITE].Source, mLevelSet[READ].Source);
    }
    private void KeepBoundaryForVelocity()
    {
        UploadGlobalParameters(mComputeKeepBoundary);
        int SolidBoundaryForVelocityKernel = mComputeKeepBoundary.FindKernel("SolidBoundaryForVelocity");
        mComputeKeepBoundary.SetTexture(SolidBoundaryForVelocityKernel, "gGridMarker", mGridMarker.Source);
        mComputeKeepBoundary.SetTexture(SolidBoundaryForVelocityKernel, "gVelocityU", mVelocityU[READ].Source);
        mComputeKeepBoundary.SetTexture(SolidBoundaryForVelocityKernel, "gVelocityV", mVelocityV[READ].Source);
        mComputeKeepBoundary.Dispatch(SolidBoundaryForVelocityKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);

    }

    private void ExtrapolateVelocityToAirGPU()
    {
        UploadGlobalParameters(mComputeExtrapolateToAir);
        // 1. set valid flag
        int UpdateGridValidUKernel = mComputeExtrapolateToAir.FindKernel("UpdateGridValidU");
        mComputeExtrapolateToAir.SetTexture(UpdateGridValidUKernel, "gGridMarker", mGridMarker.Source);
        mComputeExtrapolateToAir.SetTexture(UpdateGridValidUKernel, "gGridValidURead", mGridValidU[READ].Source);
        mComputeExtrapolateToAir.SetTexture(UpdateGridValidUKernel, "gGridValidUWrite", mGridValidU[WRITE].Source);
        mComputeExtrapolateToAir.Dispatch(UpdateGridValidUKernel, Mathf.CeilToInt((mWidth + 1) * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);
        int UpdateGridValidVKernel = mComputeExtrapolateToAir.FindKernel("UpdateGridValidV");
        mComputeExtrapolateToAir.SetTexture(UpdateGridValidVKernel, "gGridMarker", mGridMarker.Source);
        mComputeExtrapolateToAir.SetTexture(UpdateGridValidVKernel, "gGridValidVRead", mGridValidV[READ].Source);
        mComputeExtrapolateToAir.SetTexture(UpdateGridValidVKernel, "gGridValidVWrite", mGridValidV[WRITE].Source);
        mComputeExtrapolateToAir.Dispatch(UpdateGridValidVKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt((mHeight + 1) * 1.0f / mGroupThreadSizeX), 1);

        // 2. extrapolate velocity to air
        UploadGlobalParameters(mComputeCopyTex);
        int CopyVelocityUTexKernel = mComputeCopyTex.FindKernel("CopyVelocityUTex");
        int CopyVelocityVTexKernel = mComputeCopyTex.FindKernel("CopyVelocityVTex");
        int CopyUintTexKernel = mComputeCopyTex.FindKernel("CopyUintTex");

        int numIterates = 10;
        for (int i = 0; i < numIterates; i++)
        {
            //// copy tex
            //mComputeCopyTex.SetTexture(CopyVelocityUTexKernel,"gSourceU", mVelocityU[READ].Source);
            //mComputeCopyTex.SetTexture(CopyVelocityUTexKernel, "gDestinationU", mVelocityU[WRITE].Source);
            //mComputeCopyTex.Dispatch(CopyVelocityUTexKernel,Mathf.CeilToInt((mWidth + 1) * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);

            Graphics.CopyTexture(mVelocityU[READ].Source, mVelocityU[WRITE].Source);
            int ExtrapolateUKernel = mComputeExtrapolateToAir.FindKernel("ExtrapolateU");
            mComputeExtrapolateToAir.SetTexture(ExtrapolateUKernel, "gVelocityURead", mVelocityU[READ].Source);
            mComputeExtrapolateToAir.SetTexture(ExtrapolateUKernel, "gVelocityUWrite", mVelocityU[WRITE].Source);
            mComputeExtrapolateToAir.SetTexture(ExtrapolateUKernel, "gGridValidURead", mGridValidU[READ].Source);
            mComputeExtrapolateToAir.SetTexture(ExtrapolateUKernel, "gGridValidUWrite", mGridValidU[WRITE].Source);
            mComputeExtrapolateToAir.Dispatch(ExtrapolateUKernel, Mathf.CeilToInt((mWidth + 1) * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);
            Graphics.CopyTexture(mGridValidU[WRITE].Source, mGridValidU[READ].Source);

            //mComputeCopyTex.SetTexture(CopyVelocityUTexKernel, "gSourceU", mVelocityU[WRITE].Source);
            //mComputeCopyTex.SetTexture(CopyVelocityUTexKernel, "gDestinationU", mVelocityU[READ].Source);
            //mComputeCopyTex.Dispatch(CopyVelocityUTexKernel, Mathf.CeilToInt((mWidth + 1) * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);

            Graphics.CopyTexture(mVelocityU[WRITE].Source, mVelocityU[READ].Source);

            //mComputeCopyTex.SetTexture(CopyVelocityVTexKernel, "gSourceV", mVelocityV[READ].Source);
            //mComputeCopyTex.SetTexture(CopyVelocityVTexKernel, "gDestinationV", mVelocityV[WRITE].Source);
            //mComputeCopyTex.Dispatch(CopyVelocityVTexKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt((mHeight + 1) * 1.0f / mGroupThreadSizeX), 1);

            Graphics.CopyTexture(mVelocityV[READ].Source, mVelocityV[WRITE].Source);
            int ExtrapolateVKernel = mComputeExtrapolateToAir.FindKernel("ExtrapolateV");
            mComputeExtrapolateToAir.SetTexture(ExtrapolateVKernel, "gVelocityVRead", mVelocityV[READ].Source);
            mComputeExtrapolateToAir.SetTexture(ExtrapolateVKernel, "gVelocityVWrite", mVelocityV[WRITE].Source);
            mComputeExtrapolateToAir.SetTexture(ExtrapolateVKernel, "gGridValidVRead", mGridValidV[READ].Source);
            mComputeExtrapolateToAir.SetTexture(ExtrapolateVKernel, "gGridValidVWrite", mGridValidV[WRITE].Source);
            mComputeExtrapolateToAir.Dispatch(ExtrapolateVKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt((mHeight + 1) * 1.0f / mGroupThreadSizeX), 1);
            Graphics.CopyTexture(mGridValidV[WRITE].Source, mGridValidV[READ].Source);
            Graphics.CopyTexture(mVelocityV[WRITE].Source, mVelocityV[READ].Source); 
            //mComputeCopyTex.SetTexture(CopyVelocityVTexKernel, "gSourceV", mVelocityV[WRITE].Source);
            //mComputeCopyTex.SetTexture(CopyVelocityVTexKernel, "gDestinationV", mVelocityV[READ].Source);
            //mComputeCopyTex.Dispatch(CopyVelocityVTexKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt((mHeight + 1) * 1.0f / mGroupThreadSizeX), 1);

        }

    }

    private void AdvectGPU()
    {
        UploadGlobalParameters(mComputeAdvect);
        int advectLevelsetKernel = mComputeAdvect.FindKernel("AdvectLevelset");
        int advectVelocityUKernel = mComputeAdvect.FindKernel("AdvectVelocityU");
        int advectVelocityVKernel = mComputeAdvect.FindKernel("AdvectVelocityV");

        // 1. first advect levelset
        mComputeAdvect.SetTexture(advectLevelsetKernel, "gLevelSetRead", mLevelSet[READ].Source);
        mComputeAdvect.SetTexture(advectLevelsetKernel, "gVelocityReadU", mVelocityU[READ].Source);
        mComputeAdvect.SetTexture(advectLevelsetKernel, "gVelocityReadV", mVelocityV[READ].Source);
        mComputeAdvect.SetTexture(advectLevelsetKernel, "gLevelSetWrite", mLevelSet[WRITE].Source);
        mComputeAdvect.Dispatch(advectLevelsetKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);

        // 2. second advect velocity
        mComputeAdvect.SetTexture(advectVelocityUKernel, "gVelocityReadU", mVelocityU[READ].Source);
        mComputeAdvect.SetTexture(advectVelocityUKernel, "gVelocityReadV", mVelocityV[READ].Source);
        mComputeAdvect.SetTexture(advectVelocityUKernel, "gVelocityWriteU", mVelocityU[WRITE].Source);
        mComputeAdvect.Dispatch(advectVelocityUKernel, Mathf.CeilToInt((mWidth + 1) * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);

        mComputeAdvect.SetTexture(advectVelocityVKernel, "gVelocityReadU", mVelocityU[READ].Source);
        mComputeAdvect.SetTexture(advectVelocityVKernel, "gVelocityReadV", mVelocityV[READ].Source);
        mComputeAdvect.SetTexture(advectVelocityVKernel, "gVelocityWriteV", mVelocityV[WRITE].Source);
        mComputeAdvect.Dispatch(advectVelocityVKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt((mHeight + 1) * 1.0f / mGroupThreadSizeX), 1);

    }

    private void AddExternalForcesGPU()
    {
        UploadGlobalParameters(mComputeAddForces);
        int addGravityKernel = mComputeAddForces.FindKernel("AddGravity");
        mComputeAddForces.SetTexture(addGravityKernel, "gVelocityWriteV", mVelocityV[READ].Source);
        mComputeAddForces.SetTexture(addGravityKernel, "gLevelSet", mLevelSet[READ].Source);
        mComputeAddForces.Dispatch(addGravityKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt((mHeight + 1.0f) / mGroupThreadSizeX), 1);
    }

    public void ComputeDivergence()
    {
        UploadGlobalParameters(mComputeDivergence);
        int computeDivergenceKernel = mComputeDivergence.FindKernel("ComputeDivergence");
        mComputeDivergence.SetTexture(computeDivergenceKernel, "gVelocityU", mVelocityU[READ].Source);
        mComputeDivergence.SetTexture(computeDivergenceKernel, "gVelocityV", mVelocityV[READ].Source);
        mComputeDivergence.SetTexture(computeDivergenceKernel, "gGridMarker", mGridMarker.Source);
        mComputeDivergence.SetTexture(computeDivergenceKernel, "gVelocityDivergence", mDivergence.Source);
        mComputeDivergence.Dispatch(computeDivergenceKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);
    }

    public void ComputePressure()
    {
        UploadGlobalParameters(mComputeProjection);
        int jacobiKernel = mComputeProjection.FindKernel("ComputeJacobiPressure");
        // 1. compute pressure
        int numLoop = 100;
        for (int i=0;i< numLoop; i++)
        {
            mComputeProjection.SetTexture(jacobiKernel, "gPressure", mPressure.Source);
            mComputeProjection.SetTexture(jacobiKernel, "gVelocityDivergence", mDivergence.Source);
            mComputeProjection.SetTexture(jacobiKernel, "gGridMarker", mGridMarker.Source);
            mComputeProjection.Dispatch(jacobiKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);
        }
        int applyPressureKernel = mComputeProjection.FindKernel("ApplyPressure");
        // 2. project (subtract divergence of pressure)
        mComputeProjection.SetTexture(applyPressureKernel, "gPressure", mPressure.Source);
        mComputeProjection.SetTexture(applyPressureKernel, "gVelocityU", mVelocityU[READ].Source);
        mComputeProjection.SetTexture(applyPressureKernel, "gVelocityV", mVelocityV[READ].Source);
        mComputeProjection.Dispatch(applyPressureKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);

    }

    private void RedistanceLevelSet()
    {
        if (mEnabledRedistanceLevelset == false)
            return;
        mRedistanceLevelsetCounter++;
       // if(mRedistanceLevelsetCounter == 1)
        //if (mRedistanceLevelsetCounter % 9 == 0)
        {
            int redistancingLevelsetKernel = mComputeRedistanceLevelset.FindKernel("RedistancingLevelset");
            UploadGlobalParameters(mComputeRedistanceLevelset);
            mComputeRedistanceLevelset.SetFloat("_dtau", simulateStepSeconds);
            int numberOfIterations = 10;
            int currentRead = WRITE;
            int currentWrite = READ;

            for(int i = 0;i<numberOfIterations;i++)
            {
                mComputeRedistanceLevelset.SetTexture(redistancingLevelsetKernel, "gPhiLevelSetRead", mLevelSet[currentRead].Source);
                mComputeRedistanceLevelset.SetTexture(redistancingLevelsetKernel, "gPhiLevelSetWrite", mLevelSet[currentWrite].Source);
                mComputeRedistanceLevelset.Dispatch(redistancingLevelsetKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt((mHeight + 1) * 1.0f / mGroupThreadSizeX), 1);
                int temp = currentRead;
                currentRead = currentWrite;
                currentWrite = temp;
            }
        }
    }


    private void Swap()
    {
        int temp = READ;
        READ = WRITE;
        WRITE = temp;
    }

    private void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        if(!enabledDebug)
        {
            Graphics.Blit(source, destination);
            return;
        }
        //if(mEnabledRedistanceLevelset)
            debugShowTexMaterial.SetTexture("_MainTex", mLevelSet[WRITE].Source);
        //else
        //    debugShowTexMaterial.SetTexture("_MainTex", mLevelSet[READ].Source);
        debugShowTexMaterial.SetFloat("_GridSize", 1.0f / mGridSize);
        //debugShowTexMaterial.SetTexture("_MainTex", mVelocityV[READ].Source/*mLevelSet[WRITE].Source*/);
        Graphics.Blit(null, destination, debugShowTexMaterial);
    }


}
