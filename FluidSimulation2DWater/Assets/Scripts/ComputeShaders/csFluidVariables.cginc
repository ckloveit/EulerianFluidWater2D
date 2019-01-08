uint _xSize;
uint _ySize;
uint _zSize;

float _timeStep;
float _gridSpace;

float _density;
// samplerState
SamplerState g_linear_clamp_sampler;

static uint mGroupThreadSizeX = 8;

const uint CELL_AIR = 0;
const uint CELL_LIQUID = 1;
const uint CELL_SOLID = 2;
//static int AIR = 0;
//static int LIQUID = 1;
//static int SOLID = 2;