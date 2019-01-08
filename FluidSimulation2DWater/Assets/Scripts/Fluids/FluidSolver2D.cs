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

    /* Fluid quantities */
    private RenderTexture2D[] mVelocityU;
    private RenderTexture2D[] mVelocityV;
    private RenderTexture2D[] mLevelSet;

    /* Fluid Marker */
    private RenderTexture2D mGridMarker;
    private RenderTexture2D mGridValidU;
    private RenderTexture2D mGridValidV;

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

    /* Show debug */
    public bool enabledDebug = true;
    public Material debugShowTexMaterial;

    public int mSimulateFPS = 10;

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

        mLevelSet[READ] = new RenderTexture2D(mWidth, mHeight);
        mLevelSet[WRITE] = new RenderTexture2D(mWidth, mHeight);
        mVelocityU[READ] = new RenderTexture2D(mWidth + 1, mHeight);
        mVelocityU[WRITE] = new RenderTexture2D(mWidth + 1, mHeight);
        mVelocityV[READ] = new RenderTexture2D(mWidth, mHeight + 1);
        mVelocityV[WRITE] = new RenderTexture2D(mWidth, mHeight + 1);

        mDivergence = new RenderTexture2D(mWidth, mHeight);
        mPressure = new RenderTexture2D(mWidth, mHeight);

        mGridMarker = new RenderTexture2D(mWidth, mHeight,RenderTextureFormat.RInt);
        mGridValidU = new RenderTexture2D(mWidth + 1, mHeight, RenderTextureFormat.RInt);
        mGridValidV = new RenderTexture2D(mWidth, mHeight + 1, RenderTextureFormat.RInt);

        currentTime = System.DateTime.Now;
        simulateStepSeconds = 1.0f / numSimulateFPS;
        
        // init start states
        InitFluidStartStage();

    }

    private void UploadGlobalParameters(ComputeShader compute)
    {
        compute.SetInt("_xSize", mWidth);
        compute.SetInt("_ySize", mHeight);
        compute.SetFloat("_timeStep", simulateStepSeconds);//compute.SetFloat("_timeStep", Time.deltaTime);//
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
        Simulate();
        //SimulateStep();
    }

    private void SimulateStep()
    {
        // for emitter
        if(mEmitterCounter++ % 10 == 0)
        {
            mCounter++;
            if(mCounter < 20)
                EmitterFluid();
        }
        // 1. add forces
        AddExternalForcesGPU();

        // 2. extrapolate velocity to air with two grid(need to improve)
        ExtrapolateVelocityToAirGPU();

        // 3. applying project
        // 3.1 calculate divergence
        ComputeDivergence();
        // 3.2 calculate pressure (use jaccobin , red-black gauss-sibel,multi-grid ),subtract divergence pressure
        ComputePressure();

        // 4. advect quantities
        AdvectGPU();
        
        // 5. volume rendering

        Swap();
    }
    private void EmitterFluid()
    {
        int SphereEmitterKernel = mComputeFluidEmitter.FindKernel("SphereEmitter");
        UploadGlobalParameters(mComputeFluidEmitter);
        mComputeFluidEmitter.SetTexture(SphereEmitterKernel, "gLevelSet", mLevelSet[READ].Source);
        mComputeFluidEmitter.SetTexture(SphereEmitterKernel, "gVelocityU", mVelocityU[READ].Source);
        mComputeFluidEmitter.SetTexture(SphereEmitterKernel, "gVelocityV", mVelocityV[READ].Source);
        mComputeFluidEmitter.SetFloat("sphereEmitterPosX", (0.1f + 0.8f * Random.value) * mWidth);
        mComputeFluidEmitter.SetFloat("sphereEmitterPosY", (0.5f + 0.4f * Random.value) * mHeight);
        mComputeFluidEmitter.SetFloat("sphereEmitterRadius", 0.08f * Mathf.Min(mWidth, mHeight));
        mComputeFluidEmitter.Dispatch(SphereEmitterKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);
    }



    private void ExtrapolateVelocityToAirGPU()
    {
        UploadGlobalParameters(mComputeExtrapolateToAir);
        // 1. update Grid marker
        int UpdateGridMarkerKernel = mComputeExtrapolateToAir.FindKernel("UpdateGridMarker");
        mComputeExtrapolateToAir.SetTexture(UpdateGridMarkerKernel, "gLevelSet", mLevelSet[READ].Source);
        mComputeExtrapolateToAir.SetTexture(UpdateGridMarkerKernel, "gGridMarker", mGridMarker.Source);
        mComputeExtrapolateToAir.Dispatch(UpdateGridMarkerKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);

        // 2. update Grid Valid U and V

        int UpdateGridValidUKernel = mComputeExtrapolateToAir.FindKernel("UpdateGridValidU");
        mComputeExtrapolateToAir.SetTexture(UpdateGridValidUKernel, "gGridMarker", mGridMarker.Source);
        mComputeExtrapolateToAir.SetTexture(UpdateGridValidUKernel, "gGridValidU", mGridValidU.Source);
        mComputeExtrapolateToAir.Dispatch(UpdateGridValidUKernel, Mathf.CeilToInt((mWidth + 1) * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);
        int UpdateGridValidVKernel = mComputeExtrapolateToAir.FindKernel("UpdateGridValidV");
        mComputeExtrapolateToAir.SetTexture(UpdateGridValidVKernel, "gGridMarker", mLevelSet[READ].Source);
        mComputeExtrapolateToAir.SetTexture(UpdateGridValidVKernel, "gGridValidV", mGridValidV.Source);
        mComputeExtrapolateToAir.Dispatch(UpdateGridValidVKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt((mHeight + 1) * 1.0f / mGroupThreadSizeX), 1);

        // 3. extrapolate velocity to air
        int numIterates = 15;
        int ExtrapolateUKernel = mComputeExtrapolateToAir.FindKernel("ExtrapolateU");
        for (int i = 0; i < numIterates; i++)
        {
            mComputeExtrapolateToAir.SetTexture(ExtrapolateUKernel, "gVelocityU", mVelocityU[READ].Source);
            mComputeExtrapolateToAir.SetTexture(ExtrapolateUKernel, "gGridValidU", mGridValidU.Source);
            mComputeExtrapolateToAir.Dispatch(ExtrapolateUKernel, Mathf.CeilToInt((mWidth + 1) * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);
        }
        int ExtrapolateVKernel = mComputeExtrapolateToAir.FindKernel("ExtrapolateV");
        for (int i = 0; i < numIterates; i++)
        {
            mComputeExtrapolateToAir.SetTexture(ExtrapolateVKernel, "gVelocityV", mVelocityV[READ].Source);
            mComputeExtrapolateToAir.SetTexture(ExtrapolateVKernel, "gGridValidV", mGridValidV.Source);
            mComputeExtrapolateToAir.Dispatch(ExtrapolateVKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt((mHeight + 1) * 1.0f / mGroupThreadSizeX), 1);
        }

        // 4. keep solid boundary for velocity
        int SolidBoundaryForVelocityKernel = mComputeExtrapolateToAir.FindKernel("SolidBoundaryForVelocity");
        mComputeExtrapolateToAir.SetTexture(SolidBoundaryForVelocityKernel, "gGridMarker", mGridMarker.Source);
        mComputeExtrapolateToAir.SetTexture(SolidBoundaryForVelocityKernel, "gVelocityU", mVelocityU[READ].Source);
        mComputeExtrapolateToAir.SetTexture(SolidBoundaryForVelocityKernel, "gVelocityV", mVelocityV[READ].Source);
        mComputeExtrapolateToAir.Dispatch(SolidBoundaryForVelocityKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);

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
        mComputeDivergence.SetTexture(computeDivergenceKernel, "gVelocityDivergence", mDivergence.Source);
        mComputeDivergence.Dispatch(computeDivergenceKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);
    }

    public void ComputePressure()
    {
        UploadGlobalParameters(mComputeProjection);
        int jacobiKernel = mComputeProjection.FindKernel("ComputeJacobiPressure");
        // 1. compute pressure
        int numLoop = 50;
        
        for (int i=0;i< numLoop; i++)
        {
            mComputeProjection.SetTexture(jacobiKernel, "gPressure", mPressure.Source);
            mComputeProjection.SetTexture(jacobiKernel, "gVelocityDivergence", mDivergence.Source);
            mComputeProjection.Dispatch(jacobiKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);
        }
        int applyPressureKernel = mComputeProjection.FindKernel("ApplyPressure");
        // 2. project (subtract divergence of pressure
        mComputeProjection.SetTexture(applyPressureKernel, "gPressure", mPressure.Source);
        mComputeProjection.SetTexture(applyPressureKernel, "gVelocityU", mVelocityU[READ].Source);
        mComputeProjection.SetTexture(applyPressureKernel, "gVelocityV", mVelocityV[READ].Source);
        mComputeProjection.Dispatch(applyPressureKernel, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1);

        // 3. keep boundary velocity
        int keepBoundaryU = mComputeProjection.FindKernel("KeepBoundaryU");
        mComputeProjection.SetTexture(keepBoundaryU, "gVelocityU", mVelocityU[READ].Source);
        mComputeProjection.Dispatch(keepBoundaryU, Mathf.CeilToInt(mHeight * 1.0f / mGroupThreadSizeX), 1, 1);

        int keepBoundaryV = mComputeProjection.FindKernel("KeepBoundaryV");
        mComputeProjection.SetTexture(keepBoundaryV, "gVelocityV", mVelocityV[READ].Source);
        mComputeProjection.Dispatch(keepBoundaryV, Mathf.CeilToInt(mWidth * 1.0f / mGroupThreadSizeX), 1, 1);

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
        debugShowTexMaterial.SetTexture("_MainTex", mLevelSet[WRITE].Source);
        //debugShowTexMaterial.SetTexture("_MainTex", mVelocityV[READ].Source/*mLevelSet[WRITE].Source*/);
        Graphics.Blit(null, destination, debugShowTexMaterial);
    }


}
