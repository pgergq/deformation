//--------------------------------------------------------------------------------------
// File: Deformation.cpp
//
// Project Deformation
// Object deformation with mass-spring systems
//
// @(Copyright (c) Microsoft Corporation. All rights reserved. - NBodyGravityCS11 project)
// @Modified by pgq
//--------------------------------------------------------------------------------------

#include "DXUT.h"
#include "DXUTgui.h"
#include "SDKmisc.h"
#include "DXUTcamera.h"
#include "DXUTsettingsdlg.h"
#include <DirectXMath.h>
#include <commdlg.h>
#include "resource.h"
#include "WaitDlg.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
#include <time.h>


#define GET_X_LPARAM(lp)                        ((int)(short)LOWORD(lp))
#define GET_Y_LPARAM(lp)                        ((int)(short)HIWORD(lp))

using namespace DirectX;

//--------------------------------------------------------------------------------------
// Global variables
//--------------------------------------------------------------------------------------
CDXUTDialogResourceManager          g_DialogResourceManager;    // manager for shared resources of dialogs
CModelViewerCamera                  g_Camera;                   // A model viewing camera
CD3DSettingsDlg                     g_D3DSettingsDlg;           // Device settings dialog
CDXUTDialog                         g_HUD;                      // dialog for standard controls
CDXUTDialog                         g_SampleUI;                 // dialog for sample specific controls
CDXUTTextHelper*                    g_pTxtHelper = nullptr;

ID3D11VertexShader*                 g_pRenderParticlesVS = nullptr;
ID3D11GeometryShader*               g_pRenderParticlesGS = nullptr;
ID3D11PixelShader*                  g_pRenderParticlesPS = nullptr;
ID3D11PixelShader*                  g_pModelPS1 = nullptr;
ID3D11PixelShader*                  g_pModelPS2 = nullptr;
ID3D11SamplerState*                 g_pSampleStateLinear = nullptr;
ID3D11BlendState*                   g_pBlendingStateParticle = nullptr;
ID3D11DepthStencilState*            g_pDepthStencilState = nullptr;

ID3D11ComputeShader*                g_pCompute1CS = nullptr;
ID3D11ComputeShader*                g_pCompute2CS = nullptr;
ID3D11Buffer*                       g_pcbCS = nullptr;

ID3D11ComputeShader*				g_pUdateCS = nullptr;

ID3D11Buffer*						g_pIndexCube = nullptr;
ID3D11ShaderResourceView*			g_pIndexCubeRV = nullptr;

ID3D11Buffer*						g_pVolCube1 = nullptr;
ID3D11Buffer*						g_pVolCube1c = nullptr;
ID3D11Buffer*						g_pVolCube2 = nullptr;
ID3D11Buffer*						g_pVolCube2c = nullptr;
ID3D11ShaderResourceView*			g_pVolCube1RV = nullptr;
ID3D11ShaderResourceView*			g_pVolCube1cRV = nullptr;
ID3D11ShaderResourceView*			g_pVolCube2RV = nullptr;
ID3D11ShaderResourceView*			g_pVolCube2cRV = nullptr;
ID3D11UnorderedAccessView*			g_pVolCube1UAV = nullptr;
ID3D11UnorderedAccessView*			g_pVolCube1cUAV = nullptr;
ID3D11UnorderedAccessView*			g_pVolCube2UAV = nullptr;
ID3D11UnorderedAccessView*			g_pVolCube2cUAV = nullptr;

ID3D11Buffer*                       g_pParticleArray0 = nullptr;
ID3D11Buffer*                       g_pParticleArray1 = nullptr;
ID3D11ShaderResourceView*           g_pParticleArrayRV0 = nullptr;
ID3D11ShaderResourceView*           g_pParticleArrayRV1 = nullptr;
ID3D11UnorderedAccessView*          g_pParticleArrayUAV0 = nullptr;
ID3D11UnorderedAccessView*          g_pParticleArrayUAV1 = nullptr;
ID3D11Buffer*                       g_pParticleBuffer = nullptr;
ID3D11InputLayout*                  g_pParticleVertexLayout = nullptr;

ID3D11Texture2D*					g_pModelTex[2];
ID3D11RenderTargetView*				g_pModelRTV[2];
ID3D11ShaderResourceView*			g_pModelSRV[2];

ID3D11Buffer*                       g_pcbGS = nullptr;

ID3D11ShaderResourceView*           g_pParticleTexRV = nullptr;

float                               g_fSpread = 400.0f;
float								g_fStiffness = 200.0f;
float								g_fDamping = -3.0f;
float								g_fInvMass = 1.0f;
int									g_nWindowWidth = 800;
int									g_nWindowHeight = 600;
int									g_nCurrentMouseX;
int									g_nCurrentMouseY;
int									g_nPickOriginX;
int									g_nPickOriginY;

std::ofstream						debug;
std::vector<std::vector<float>>		g_vVertices;					// file import data > model vertices
std::vector<std::vector<float>>		g_vNormals;						// file import data > model vertex normals
std::vector<std::vector<int>>		g_vFaces;						// file import data > model faces (index from 1)
bool								g_bPicking = false;				// RBUTTON is pressed
bool								g_bRM2Texture = false;			// render model to texture for picking
XMFLOAT3							g_vVolCubeO;					// offset verctor added to every volumetric masspoint
XMFLOAT3							g_vModelO(100, 100, 100);			// offset vector for the model
int									g_nNumParticles;				// number of particles in the loaded model
int									g_nVolCubeCell;					// cell size of volcube

#define VCUBEWIDTH		19											// n*n*n inner cube, (n+1)*(n+1)*(n+1) outer cube
#define VOLCUBE1_COUNT	VCUBEWIDTH*VCUBEWIDTH*VCUBEWIDTH			// number of masspoints in the smaller volumetric cube
#define VOLCUBE2_COUNT	(VCUBEWIDTH+1)*(VCUBEWIDTH+1)*(VCUBEWIDTH+1) // number of masspoints in the bigger volumetric cube

// volcube neighbouring data
#define NB_SAME_LEFT			0x20								// 0010 0000, has left neighbour
#define NB_SAME_RIGHT			0x10								// 0001 0000, has right neighbour
#define NB_SAME_DOWN			0x08								// 0000 1000, ...
#define NB_SAME_UP				0x04								// 0000 0100
#define NB_SAME_FRONT			0x02								// 0000 0010
#define NB_SAME_BACK			0x01								// 0000 0001
// NEAR = lower end of Z axis, nearer to the viewer
// FAR = higher Z values, farther into the screen
#define NB_OTHER_NEAR_BOT_LEFT	0x80								// 1000 0000
#define NB_OTHER_NEAR_BOT_RIGHT	0x40								// 0100 0000
#define NB_OTHER_NEAR_TOP_LEFT	0x20								// 0010 0000
#define NB_OTHER_NEAR_TOP_RIGHT	0x10								// 0001 0000
#define NB_OTHER_FAR_BOT_LEFT	0x08								// 0000 1000
#define NB_OTHER_FAR_BOT_RIGHT	0x04								// 0000 0100
#define NB_OTHER_FAR_TOP_LEFT	0x02								// 0000 0010
#define NB_OTHER_FAR_TOP_RIGHT	0x01								// 0000 0001

struct PARTICLE_VERTEX
{
    XMFLOAT4 Color;
};

struct CB_GS
{
    XMFLOAT4X4 m_WorldViewProj;
    XMFLOAT4X4 m_InvView;
};

struct CB_CS
{
    UINT vcwidth;			// number of masspoint in the smaller volcube in one row
    UINT vccell;			// size of one volcube cell
    UINT numparticles;		// total count of model vertices

    UINT is_picking;		// bool for mouse picking
    UINT pickOriginX;		// pickorigin
    UINT pickOriginY;		// pickorigin
    UINT dummy1;			// dummy
    UINT dummy2;

    XMFLOAT4 pickDir;		// picking vector direction
    XMFLOAT4 eyePos;		// eye position

    float stiffness;		// stiffness
    float damping;			// damping, negative!
    float dt;				// delta time
    float im;				// inverse mass of masspoints
};

struct MASSPOINT
{
    XMFLOAT4 oldpos;		// previous position of masspoint
    XMFLOAT4 newpos;		// current position of masspoint
    XMFLOAT4 acc;			// masspoint acceleration
    unsigned int neighbour_same;	// neighbour_data mask in the same volcube
    unsigned int neighbour_other;	// neighbour_data mask in the other volcube
};

struct PARTICLE
{
    XMFLOAT4 pos;			// model vertex position in world space
    XMFLOAT4 normal;		// model vertex normal
    XMFLOAT4 mpid1;			// model masscube ID (#1)
    XMFLOAT4 mpid2;			// model masscube ID (#2)
};

struct INDEXER
{
    XMFLOAT3 vc1index;
    float w1[8];
    XMFLOAT3 vc2index;
    float w2[8];
};
INDEXER* indexcube;

//--------------------------------------------------------------------------------------
// UI control IDs
//--------------------------------------------------------------------------------------
#define IDC_TOGGLEFULLSCREEN    1
#define IDC_TOGGLEREF           3
#define IDC_CHANGEDEVICE        4
#define IDC_RESETPARTICLES      5
#define IDC_POKEMODEL			6

//--------------------------------------------------------------------------------------
// Forward declarations 
//--------------------------------------------------------------------------------------
bool CALLBACK ModifyDeviceSettings(DXUTDeviceSettings* pDeviceSettings, void* pUserContext);
void CALLBACK OnKeyboard(UINT nChar, bool bKeyDown, bool bAltDown, void* pUserContext);
void CALLBACK OnFrameMove(double fTime, float fElapsedTime, void* pUserContext);
LRESULT CALLBACK MsgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam, bool* pbNoFurtherProcessing, void* pUserContext);
void CALLBACK OnGUIEvent(UINT nEvent, int nControlID, CDXUTControl* pControl, void* pUserContext);
bool CALLBACK IsD3D11DeviceAcceptable(const CD3D11EnumAdapterInfo *AdapterInfo, UINT Output, const CD3D11EnumDeviceInfo *DeviceInfo, DXGI_FORMAT BackBufferFormat, bool bWindowed, void* pUserContext);
HRESULT CALLBACK OnD3D11CreateDevice(ID3D11Device* pd3dDevice, const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc, void* pUserContext);
HRESULT CALLBACK OnD3D11ResizedSwapChain(ID3D11Device* pd3dDevice, IDXGISwapChain* pSwapChain, const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc, void* pUserContext);
void CALLBACK OnD3D11ReleasingSwapChain(void* pUserContext);
void CALLBACK OnD3D11DestroyDevice(void* pUserContext);
void CALLBACK OnD3D11FrameRender(ID3D11Device* pd3dDevice, ID3D11DeviceContext* pd3dImmediateContext, double fTime, float fElapsedTime, void* pUserContext);

void InitApp();
void RenderText();
void ModelRenderBuffers(ID3D11Device*, int, int);
bool RenderModel(ID3D11DeviceContext*, CXMMATRIX, CXMMATRIX);
void LoadModel(PARTICLE*);
void LoadVolumetricCubes(MASSPOINT*, MASSPOINT*, PARTICLE*);
void print_debug(const char*);




//--------------------------------------------------------------------------------------
// Entry point to the program. Initializes everything and goes into a message processing 
// loop. Idle time is used to render the scene.
//--------------------------------------------------------------------------------------
int WINAPI wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow)
{

    // Enable run-time memory check for debug builds.
#if defined(DEBUG) | defined(_DEBUG)
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

    DXUTSetCallbackDeviceChanging(ModifyDeviceSettings);
    DXUTSetCallbackMsgProc(MsgProc);
    DXUTSetCallbackFrameMove(OnFrameMove);
    DXUTSetCallbackKeyboard(OnKeyboard);

    DXUTSetCallbackD3D11DeviceAcceptable(IsD3D11DeviceAcceptable);
    DXUTSetCallbackD3D11DeviceCreated(OnD3D11CreateDevice);
    DXUTSetCallbackD3D11SwapChainResized(OnD3D11ResizedSwapChain);
    DXUTSetCallbackD3D11FrameRender(OnD3D11FrameRender);
    DXUTSetCallbackD3D11SwapChainReleasing(OnD3D11ReleasingSwapChain);
    DXUTSetCallbackD3D11DeviceDestroyed(OnD3D11DestroyDevice);

    InitApp();

    DXUTInit(true, true);                 // Use this line instead to try to create a hardware device

    DXUTSetCursorSettings(true, true);	// Show the cursor and clip it when in full screen
    DXUTCreateWindow(L"Deformation");
    DXUTCreateDevice(D3D_FEATURE_LEVEL_11_0, true, 800, 600);
    DXUTMainLoop();                      // Enter into the DXUT render loop

    return DXUTGetExitCode();
}

//--------------------------------------------------------------------------------------
// Initialize the app 
//--------------------------------------------------------------------------------------
void InitApp()
{

    g_D3DSettingsDlg.Init(&g_DialogResourceManager);
    g_HUD.Init(&g_DialogResourceManager);
    g_SampleUI.Init(&g_DialogResourceManager);

    g_HUD.SetCallback(OnGUIEvent); int iY = 10;
    g_HUD.AddButton(IDC_TOGGLEFULLSCREEN, L"Toggle full screen", 0, iY, 170, 23);
    g_HUD.AddButton(IDC_TOGGLEREF, L"Toggle REF (F3)", 0, iY += 26, 170, 23, VK_F3);
    g_HUD.AddButton(IDC_CHANGEDEVICE, L"Change device (F2)", 0, iY += 26, 170, 23, VK_F2);
    g_HUD.AddButton(IDC_RESETPARTICLES, L"Reset particles", 0, iY += 26, 170, 22, VK_F2);
    g_HUD.AddButton(IDC_POKEMODEL, L"Poke", 0, iY += 26, 170, 22, VK_F12);
    g_SampleUI.SetCallback(OnGUIEvent);

}

//--------------------------------------------------------------------------------------
// Create buffer for particles (+volumetric vertices), fill it with colour data
//--------------------------------------------------------------------------------------
HRESULT CreateParticleBuffer(ID3D11Device* pd3dDevice)
{

    HRESULT hr = S_OK;

    D3D11_BUFFER_DESC vbdesc =
    {
        (g_nNumParticles + VOLCUBE1_COUNT + VOLCUBE2_COUNT) * sizeof(PARTICLE_VERTEX),
        D3D11_USAGE_DEFAULT,
        D3D11_BIND_VERTEX_BUFFER,
        0,
        0
    };
    D3D11_SUBRESOURCE_DATA vbInitData;
    ZeroMemory(&vbInitData, sizeof(D3D11_SUBRESOURCE_DATA));

    PARTICLE_VERTEX* pVertices = new PARTICLE_VERTEX[(g_nNumParticles + VOLCUBE1_COUNT + VOLCUBE2_COUNT)];
    if (!pVertices)
        return E_OUTOFMEMORY;

    // particles, green dots
    for (int i = 0; i < g_nNumParticles; i++)
    {
        pVertices[i].Color = XMFLOAT4(0.1f, 1.0f, 0.1f, 1);
    }
    // first volcube, red dots
    for (int i = g_nNumParticles; i < g_nNumParticles + VOLCUBE1_COUNT; i++)
    {
        pVertices[i].Color = XMFLOAT4(1.0f, 0.1f, 0.1f, 0.4f);
    }
    // second volcube, blue dots
    for (int i = g_nNumParticles + VOLCUBE1_COUNT; i < g_nNumParticles + VOLCUBE1_COUNT + VOLCUBE2_COUNT; i++)
    {
        pVertices[i].Color = XMFLOAT4(0.1f, 0.1f, 1.0f, 0.4f);
    }

    vbInitData.pSysMem = pVertices;
    V_RETURN(pd3dDevice->CreateBuffer(&vbdesc, &vbInitData, &g_pParticleBuffer));
    DXUT_SetDebugName(g_pParticleBuffer, "Particle colours");
    SAFE_DELETE_ARRAY(pVertices);

    return hr;
}

//--------------------------------------------------------------------------------------
// Load model file, set global volcube parameters
//-------------------------------------------------------------------------------------
void ReadModel()
{

    // Import file
    std::ifstream input;
    input.open("bunny_res3_scaled.obj");	//scaled + rotated + added_normals

    while (!input.eof()){
        // get line
        std::string line;
        getline(input, line);

        // split at spaces
        std::vector<float> v;
        std::vector<float> n;
        std::vector<int> f;
        std::istringstream buf(line);
        std::istream_iterator<std::string> start(buf), end;
        std::vector<std::string> parts(start, end);

        // store vertex
        if (line[0] == 'v' && line[1] != 'n'){
            // parse
            for (auto& s : parts){
                v.push_back((float)atof(s.c_str()));
            }
            //store
            g_vVertices.push_back(v);
        }
        // store normal
        else if (line[0] == 'v' && line[1] == 'n'){
            // parse
            for (auto& s : parts){
                n.push_back((float)atof(s.c_str()));
            }
            //store
            g_vNormals.push_back(n);
        }
        // store face
        else if (line[0] == 'f'){
            // parse
            for (auto& s : parts){
                f.push_back((float)atoi(s.c_str()));
            }
            //store
            g_vFaces.push_back(f);
        }
    }

    // get/set space division parameters
    float minx, miny, minz, maxx, maxy, maxz;
    maxx = minx = g_vVertices[0][1];
    maxy = miny = g_vVertices[0][2];
    maxz = minz = g_vVertices[0][3];
    for (unsigned int i = 0; i < g_vVertices.size(); i++){
        if (g_vVertices[i][1] < minx)
            minx = g_vVertices[i][1];
        if (g_vVertices[i][1] > maxx)
            maxx = g_vVertices[i][1];
        if (g_vVertices[i][2] < miny)
            miny = g_vVertices[i][2];
        if (g_vVertices[i][2] > maxy)
            maxy = g_vVertices[i][2];
        if (g_vVertices[i][3] < minz)
            minz = g_vVertices[i][3];
        if (g_vVertices[i][3] > maxz)
            maxz = g_vVertices[i][3];
    }

    // tmp = greatest length in any direction (x|y|z)
    float tmp = std::max(abs(ceil(maxx) - floor(minx)), std::max(abs(ceil(maxy) - floor(miny)), abs(ceil(maxz) - floor(minz)))) + 1;
    // ceil((float)tmp/VCUBEWIDTH) = "tight" value of VOLCUBECELL
    // ceil((float)(tight+50)/100)*100 = upper 100 neighbour of tight
    tmp = ceil((float)tmp / VCUBEWIDTH);
    tmp = ceil((float)(tmp + 50) / 100) * 100;
    g_nVolCubeCell = (int)tmp;

    // Set global volumetric cube offsets to align the model
    g_vVolCubeO = XMFLOAT3(floor(minx), floor(miny), floor(minz));

    // Set model vertex count
    g_nNumParticles = g_vVertices.size();

}

//--------------------------------------------------------------------------------------
// Load particle buffer with initial position
//--------------------------------------------------------------------------------------
void LoadModel(PARTICLE* pParticles)
{

    // Load particle buffer with default data
    for (int i = 0; i < VOLCUBE1_COUNT + VOLCUBE2_COUNT + g_nNumParticles; i++){
        pParticles[i].pos = XMFLOAT4(0, 0, 0, 1);
        pParticles[i].normal = XMFLOAT4(0, 0, 0, 1);
        pParticles[i].mpid1 = XMFLOAT4(0, 0, 0, 0);
        pParticles[i].mpid2 = XMFLOAT4(0, 0, 0, 0);
    }

    // Load model vertices + normals
    for (int i = 0; i < g_nNumParticles; i++)
    {
        XMVECTOR tmp = XMVectorAdd(XMVectorSet(g_vVertices[i][1], g_vVertices[i][2], g_vVertices[i][3], 1),
            XMVectorSet(g_vModelO.x, g_vModelO.y, g_vModelO.z, 0));
        XMStoreFloat4(&pParticles[i].pos, tmp);

        // normalized
        float len = g_vNormals[i][1] * g_vNormals[i][1] + g_vNormals[i][2] * g_vNormals[i][2] + g_vNormals[i][3] * g_vNormals[i][3];
        len = (len == 0 ? 0 : sqrtf(len));
        XMVECTOR tmp2 = XMVectorSet((float)g_vNormals[i][1] / len, (float)g_vNormals[i][2] / len, (float)g_vNormals[i][3] / len, 1);
        XMStoreFloat4(&pParticles[i].normal, XMVector3Normalize(tmp2));
    }
}

//--------------------------------------------------------------------------------------
// Load volumetric cube buffers with data, runs after LoadModel
//--------------------------------------------------------------------------------------
void LoadVolumetricCubes(MASSPOINT* volcube1, MASSPOINT* volcube2, PARTICLE* particles)
{

    int ind;
    unsigned int mask = 0x00;
    // Load first volumetric cube
    for (int i = 0; i < VCUBEWIDTH; i++)
    {
        for (int j = 0; j < VCUBEWIDTH; j++)
        {
            for (int k = 0; k < VCUBEWIDTH; k++)
            {
                ind = i*VCUBEWIDTH*VCUBEWIDTH + j*VCUBEWIDTH + k;
                XMVECTOR tmp = XMVectorAdd(XMVectorSet(k * g_nVolCubeCell, j * g_nVolCubeCell, i * g_nVolCubeCell, 1), XMVectorSet(g_vVolCubeO.x, g_vVolCubeO.y, g_vVolCubeO.z, 0));
                XMStoreFloat4(&volcube1[ind].newpos, tmp);
                XMStoreFloat4(&volcube1[ind].oldpos, tmp);
                volcube1[ind].acc = XMFLOAT4(0, 0, 0, 0);
            }
        }
    }
    // Load second volumetric cube
    for (int i = 0; i < VCUBEWIDTH + 1; i++)
    {
        for (int j = 0; j < VCUBEWIDTH + 1; j++)
        {
            for (int k = 0; k < VCUBEWIDTH + 1; k++)
            {
                ind = i*(VCUBEWIDTH + 1)*(VCUBEWIDTH + 1) + j*(VCUBEWIDTH + 1) + k;
                XMVECTOR tmp = XMVectorAdd(XMVectorSet(k * g_nVolCubeCell - 0.5f * g_nVolCubeCell, j * g_nVolCubeCell - 0.5f * g_nVolCubeCell, i * g_nVolCubeCell - 0.5f * g_nVolCubeCell, 1), XMVectorSet(g_vVolCubeO.x, g_vVolCubeO.y, g_vVolCubeO.z, 0));
                XMStoreFloat4(&volcube2[ind].oldpos, tmp);
                XMStoreFloat4(&volcube2[ind].newpos, tmp);
                volcube2[ind].acc = XMFLOAT4(0, 0, 0, 0);
            }
        }
    }

    // Load indexer cube
    indexcube = new INDEXER[g_nNumParticles];
    XMFLOAT3 vc_pos1(volcube1[0].oldpos.x, volcube1[0].oldpos.y, volcube1[0].oldpos.z);
    XMFLOAT3 vc_pos2(volcube2[0].oldpos.x, volcube2[0].oldpos.y, volcube2[0].oldpos.z);
    int vind; XMFLOAT3 vertex;
    for (int i = 0; i < g_nNumParticles; i++)
    {
        // Get indices and weights in first volumetric cube
        vertex = XMFLOAT3(particles[i].pos.x, particles[i].pos.y, particles[i].pos.z);
        int x = (std::max(vertex.x, vc_pos1.x) - std::min(vertex.x, vc_pos1.x)) / g_nVolCubeCell;
        int y = (std::max(vertex.y, vc_pos1.y) - std::min(vertex.y, vc_pos1.y)) / g_nVolCubeCell;
        int z = (std::max(vertex.z, vc_pos1.z) - std::min(vertex.z, vc_pos1.z)) / g_nVolCubeCell;
        XMStoreFloat3(&indexcube[i].vc1index, XMVectorSet(x, y, z, 0));
        XMStoreFloat4(&particles[i].mpid1, XMVectorSet(x, y, z, 1));

        // trilinear interpolation
        vind = z*VCUBEWIDTH*VCUBEWIDTH + y*VCUBEWIDTH + x;
        float wx = (vertex.x - volcube1[vind].newpos.x) / g_nVolCubeCell;
        float dwx = 1.0f - wx;
        float wy = (vertex.y - volcube1[vind].newpos.y) / g_nVolCubeCell;
        float dwy = 1.0f - wy;
        float wz = (vertex.z - volcube1[vind].newpos.z) / g_nVolCubeCell;
        float dwz = 1.0f - wz;
        indexcube[i].w1[0] = dwx*dwy*dwz; indexcube[i].w1[1] = wx*dwy*dwz; indexcube[i].w1[2] = dwx*wy*dwz; indexcube[i].w1[3] = wx*wy*dwz;
        indexcube[i].w1[4] = dwx*dwy*wz; indexcube[i].w1[5] = wx*dwy*wz; indexcube[i].w1[6] = dwx*wy*wz; indexcube[i].w1[7] = wx*wy*wz;

        // Fill second indexer
        x = (std::max(vertex.x, vc_pos2.x) - std::min(vertex.x, vc_pos2.x)) / g_nVolCubeCell;
        y = (std::max(vertex.y, vc_pos2.y) - std::min(vertex.y, vc_pos2.y)) / g_nVolCubeCell;
        z = (std::max(vertex.z, vc_pos2.z) - std::min(vertex.z, vc_pos2.z)) / g_nVolCubeCell;
        XMStoreFloat3(&indexcube[i].vc2index, XMVectorSet(x, y, z, 0));
        XMStoreFloat4(&particles[i].mpid2, XMVectorSet(x, y, z, 1));

        vind = z*(VCUBEWIDTH + 1)*(VCUBEWIDTH + 1) + y*(VCUBEWIDTH + 1) + x;
        wx = (vertex.x - volcube2[vind].newpos.x) / g_nVolCubeCell;
        dwx = 1.0f - wx;
        wy = (vertex.y - volcube2[vind].newpos.y) / g_nVolCubeCell;
        dwy = 1.0f - wy;
        wz = (vertex.z - volcube2[vind].newpos.z) / g_nVolCubeCell;
        dwz = 1.0f - wz;
        indexcube[i].w2[0] = dwx*dwy*dwz; indexcube[i].w2[1] = wx*dwy*dwz; indexcube[i].w2[2] = dwx*wy*dwz; indexcube[i].w2[3] = wx*wy*dwz;
        indexcube[i].w2[4] = dwx*dwy*wz; indexcube[i].w2[5] = wx*dwy*wz; indexcube[i].w2[6] = dwx*wy*wz; indexcube[i].w2[7] = wx*wy*wz;

    }


    /// Set proper neighbouring data (disable masspoints with no model points)
    UINT nvc1[VCUBEWIDTH][VCUBEWIDTH][VCUBEWIDTH];
    UINT nvc2[VCUBEWIDTH + 1][VCUBEWIDTH + 1][VCUBEWIDTH + 1];
    for (int z = 0; z < VCUBEWIDTH; z++){
        for (int y = 0; y < VCUBEWIDTH; y++){
            for (int x = 0; x < VCUBEWIDTH; x++){
                nvc1[z][y][x] = 0;
            }
        }
    }
    for (int z = 0; z < VCUBEWIDTH + 1; z++){
        for (int y = 0; y < VCUBEWIDTH + 1; y++){
            for (int x = 0; x < VCUBEWIDTH + 1; x++){
                nvc2[z][y][x] = 0;
            }
        }
    }
    XMFLOAT3 x;
    // Set edge masspoints to 1
    for (int i = 0; i < g_nNumParticles; i++)
    {
        x = indexcube[i].vc1index;
        nvc1[(int)x.z][(int)x.y][(int)x.x] = 1; nvc1[(int)x.z][(int)x.y][(int)x.x + 1] = 1; nvc1[(int)x.z][(int)x.y + 1][(int)x.x] = 1; nvc1[(int)x.z][(int)x.y + 1][(int)x.x + 1] = 1;
        nvc1[(int)x.z + 1][(int)x.y][(int)x.x] = 1; nvc1[(int)x.z + 1][(int)x.y][(int)x.x + 1] = 1; nvc1[(int)x.z + 1][(int)x.y + 1][(int)x.x] = 1; nvc1[(int)x.z + 1][(int)x.y + 1][(int)x.x + 1] = 1;
        x = indexcube[i].vc2index;
        nvc2[(int)x.z][(int)x.y][(int)x.x] = 1; nvc2[(int)x.z][(int)x.y][(int)x.x + 1] = 1; nvc2[(int)x.z][(int)x.y + 1][(int)x.x] = 1; nvc2[(int)x.z][(int)x.y + 1][(int)x.x + 1] = 1;
        nvc2[(int)x.z + 1][(int)x.y][(int)x.x] = 1; nvc2[(int)x.z + 1][(int)x.y][(int)x.x + 1] = 1; nvc2[(int)x.z + 1][(int)x.y + 1][(int)x.x] = 1; nvc2[(int)x.z + 1][(int)x.y + 1][(int)x.x + 1] = 1;
    }

    // Set outer masspoints to 2 - 1st volcube
    //left+
    for (int z = 0; z < VCUBEWIDTH; z++){
        for (int y = 0; y < VCUBEWIDTH; y++){
            for (int x = 0; x < VCUBEWIDTH; x++){
                if (nvc1[z][y][x] == 1) break;
                else nvc1[z][y][x] = 2;
            }
        }
    }
    //right+
    for (int z = 0; z < VCUBEWIDTH; z++){
        for (int y = 0; y < VCUBEWIDTH; y++){
            for (int x = VCUBEWIDTH - 1; x >= 0; x--){
                if (nvc1[z][y][x] == 1) break;
                else nvc1[z][y][x] = 2;
            }
        }
    }
    //bottom+
    for (int z = 0; z < VCUBEWIDTH; z++){
        for (int x = 0; x < VCUBEWIDTH; x++){
            for (int y = 0; y < VCUBEWIDTH; y++){
                if (nvc1[z][y][x] == 1) break;
                else nvc1[z][y][x] = 2;
            }
        }
    }
    //up+
    for (int z = 0; z < VCUBEWIDTH; z++){
        for (int x = 0; x < VCUBEWIDTH; x++){
            for (int y = VCUBEWIDTH - 1; y >= 0; y--){
                if (nvc1[z][y][x] == 1) break;
                else nvc1[z][y][x] = 2;
            }
        }
    }
    //front+
    for (int y = 0; y < VCUBEWIDTH; y++){
        for (int x = 0; x < VCUBEWIDTH; x++){
            for (int z = 0; z < VCUBEWIDTH; z++){
                if (nvc1[z][y][x] == 1) break;
                else nvc1[z][y][x] = 2;
            }
        }
    }
    //back+
    for (int y = 0; y < VCUBEWIDTH; y++){
        for (int x = 0; x < VCUBEWIDTH; x++){
            for (int z = VCUBEWIDTH - 1; z >= 0; z--){
                if (nvc1[z][y][x] == 1) break;
                else nvc1[z][y][x] = 2;
            }
        }
    }
    //Set outer masspoints to 2 - 2nd volcube
    //left+
    for (int z = 0; z < VCUBEWIDTH + 1; z++){
        for (int y = 0; y < VCUBEWIDTH + 1; y++){
            for (int x = 0; x < VCUBEWIDTH + 1; x++){
                if (nvc2[z][y][x] == 1) break;
                else nvc2[z][y][x] = 2;
            }
        }
    }
    //right+
    for (int z = 0; z < VCUBEWIDTH + 1; z++){
        for (int y = 0; y < VCUBEWIDTH + 1; y++){
            for (int x = VCUBEWIDTH; x >= 0; x--){
                if (nvc2[z][y][x] == 1) break;
                else nvc2[z][y][x] = 2;
            }
        }
    }
    //bottom+
    for (int z = 0; z < VCUBEWIDTH + 1; z++){
        for (int x = 0; x < VCUBEWIDTH + 1; x++){
            for (int y = 0; y < VCUBEWIDTH + 1; y++){
                if (nvc2[z][y][x] == 1) break;
                else nvc2[z][y][x] = 2;
            }
        }
    }
    //up+
    for (int z = 0; z < VCUBEWIDTH + 1; z++){
        for (int x = 0; x < VCUBEWIDTH + 1; x++){
            for (int y = VCUBEWIDTH; y >= 0; y--){
                if (nvc2[z][y][x] == 1) break;
                else nvc2[z][y][x] = 2;
            }
        }
    }
    //front+
    for (int y = 0; y < VCUBEWIDTH + 1; y++){
        for (int x = 0; x < VCUBEWIDTH + 1; x++){
            for (int z = 0; z < VCUBEWIDTH + 1; z++){
                if (nvc2[z][y][x] == 1) break;
                else nvc2[z][y][x] = 2;
            }
        }
    }
    //back+
    for (int y = 0; y < VCUBEWIDTH + 1; y++){
        for (int x = 0; x < VCUBEWIDTH + 1; x++){
            for (int z = VCUBEWIDTH; z >= 0; z--){
                if (nvc2[z][y][x] == 1) break;
                else nvc2[z][y][x] = 2;
            }
        }
    }

    // Correct neighbouring in 1st volcube
    for (int i = 0; i < VCUBEWIDTH; i++)
    {
        for (int j = 0; j < VCUBEWIDTH; j++)
        {
            for (int k = 0; k < VCUBEWIDTH; k++)
            {
                ind = i*VCUBEWIDTH*VCUBEWIDTH + j*VCUBEWIDTH + k;

                // same volcube bitmasks
                mask = 0x00;
                mask = k != 0 && nvc1[i][j][k - 1] != 2 ? mask | NB_SAME_LEFT : mask;
                mask = k != (VCUBEWIDTH - 1) && nvc1[i][j][k + 1] != 2 ? mask | NB_SAME_RIGHT : mask;
                mask = j != 0 && nvc1[i][j - 1][k] != 2 ? mask | NB_SAME_DOWN : mask;
                mask = j != (VCUBEWIDTH - 1) && nvc1[i][j + 1][k] != 2 ? mask | NB_SAME_UP : mask;
                mask = i != 0 && nvc1[i - 1][j][k] != 2 ? mask | NB_SAME_FRONT : mask;
                mask = i != (VCUBEWIDTH - 1) && nvc1[i + 1][j][k] != 2 ? mask | NB_SAME_BACK : mask;
                volcube1[ind].neighbour_same = mask;

                // other volcube bitmasks
                mask = 0x00;
                mask = nvc2[i][j][k] != 2 ? mask | NB_OTHER_NEAR_BOT_LEFT : mask;
                mask = nvc2[i][j][k + 1] != 2 ? mask | NB_OTHER_NEAR_BOT_RIGHT : mask;
                mask = nvc2[i][j + 1][k] != 2 ? mask | NB_OTHER_NEAR_TOP_LEFT : mask;
                mask = nvc2[i][j + 1][k + 1] != 2 ? mask | NB_OTHER_NEAR_TOP_RIGHT : mask;
                mask = nvc2[i + 1][j][k] != 2 ? mask | NB_OTHER_FAR_BOT_LEFT : mask;
                mask = nvc2[i + 1][j][k + 1] != 2 ? mask | NB_OTHER_FAR_BOT_RIGHT : mask;
                mask = nvc2[i + 1][j + 1][k] != 2 ? mask | NB_OTHER_FAR_TOP_LEFT : mask;
                mask = nvc2[i + 1][j + 1][k + 1] != 2 ? mask | NB_OTHER_FAR_TOP_RIGHT : mask;
                volcube1[ind].neighbour_other = mask;
            }
        }
    }

    // Correct neighbouring in 2nd volcube
    for (int i = 0; i < VCUBEWIDTH + 1; i++)
    {
        for (int j = 0; j < VCUBEWIDTH + 1; j++)
        {
            for (int k = 0; k < VCUBEWIDTH + 1; k++)
            {
                ind = i*(VCUBEWIDTH + 1)*(VCUBEWIDTH + 1) + j*(VCUBEWIDTH + 1) + k;

                // same volcube bitmasks
                mask = 0x00;
                mask = k != 0 && nvc2[i][j][k - 1] != 2 ? mask | NB_SAME_LEFT : mask;
                mask = k != (VCUBEWIDTH) && nvc2[i][j][k + 1] != 2 ? mask | NB_SAME_RIGHT : mask;
                mask = j != 0 && nvc2[i][j - 1][k] != 2 ? mask | NB_SAME_DOWN : mask;
                mask = j != (VCUBEWIDTH) && nvc2[i][j + 1][k] != 2 ? mask | NB_SAME_UP : mask;
                mask = i != 0 && nvc2[i - 1][j][k] != 2 ? mask | NB_SAME_FRONT : mask;
                mask = i != (VCUBEWIDTH) && nvc2[i + 1][j][k] != 2 ? mask | NB_SAME_BACK : mask;
                volcube2[ind].neighbour_same = mask;

                //// other volcube bitmasks
                mask = 0x00;
                mask = (i != 0 && j != 0 && k != 0) && nvc1[i - 1][j - 1][k - 1] != 2 ? mask | NB_OTHER_NEAR_BOT_LEFT : mask;
                mask = (i != 0 && j != 0 && k != VCUBEWIDTH) && nvc1[i - 1][j - 1][k] != 2 ? mask | NB_OTHER_NEAR_BOT_RIGHT : mask;
                mask = (i != 0 && j != VCUBEWIDTH && k != 0) && nvc1[i - 1][j][k - 1] != 2 ? mask | NB_OTHER_NEAR_TOP_LEFT : mask;
                mask = (i != 0 && j != VCUBEWIDTH && k != VCUBEWIDTH) && nvc1[i - 1][j][k] != 2 ? mask | NB_OTHER_NEAR_TOP_RIGHT : mask;
                mask = (i != VCUBEWIDTH && j != 0 && k != 0) && nvc1[i][j - 1][k - 1] != 2 ? mask | NB_OTHER_FAR_BOT_LEFT : mask;
                mask = (i != VCUBEWIDTH && j != 0 && k != VCUBEWIDTH) && nvc1[i][j - 1][k] != 2 ? mask | NB_OTHER_FAR_BOT_RIGHT : mask;
                mask = (i != VCUBEWIDTH && j != VCUBEWIDTH && k != 0) && nvc1[i][j][k - 1] != 2 ? mask | NB_OTHER_FAR_TOP_LEFT : mask;
                mask = (i != VCUBEWIDTH && j != VCUBEWIDTH && k != VCUBEWIDTH) && nvc1[i][j][k] != 2 ? mask | NB_OTHER_FAR_TOP_RIGHT : mask;
                volcube2[ind].neighbour_other = mask;
            }
        }
    }
}

//--------------------------------------------------------------------------------------
// Create and initialize buffer structures
//--------------------------------------------------------------------------------------
HRESULT CreateParticlePosVeloBuffers(ID3D11Device* pd3dDevice)
{

    HRESULT hr = S_OK;

    // Desc for Particle buffers
    D3D11_BUFFER_DESC desc;
    ZeroMemory(&desc, sizeof(desc));
    desc.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;
    desc.ByteWidth = (g_nNumParticles + VOLCUBE1_COUNT + VOLCUBE2_COUNT) * sizeof(PARTICLE);
    desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
    desc.StructureByteStride = sizeof(PARTICLE);
    desc.Usage = D3D11_USAGE_DEFAULT;

    // Desc for Volumetric buffer
    D3D11_BUFFER_DESC vdesc;
    ZeroMemory(&vdesc, sizeof(vdesc));
    vdesc.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;
    vdesc.ByteWidth = VOLCUBE1_COUNT * sizeof(MASSPOINT);
    vdesc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
    vdesc.StructureByteStride = sizeof(MASSPOINT);
    vdesc.Usage = D3D11_USAGE_DEFAULT;

    // Initialize the data in the buffers
    PARTICLE* pData1 = new PARTICLE[(g_nNumParticles + VOLCUBE1_COUNT + VOLCUBE2_COUNT)];
    MASSPOINT* vData1 = new MASSPOINT[VOLCUBE1_COUNT];
    MASSPOINT* vData2 = new MASSPOINT[VOLCUBE2_COUNT];
    if (!pData1 || !vData1 || !vData2)
        return E_OUTOFMEMORY;

    /// Load Particle cube, load Volumetric cubes
    LoadModel(pData1);
    LoadVolumetricCubes(vData1, vData2, pData1);

    // Set initial Particles data
    D3D11_SUBRESOURCE_DATA InitData;
    InitData.pSysMem = pData1;
    V_RETURN(pd3dDevice->CreateBuffer(&desc, &InitData, &g_pParticleArray0));
    V_RETURN(pd3dDevice->CreateBuffer(&desc, &InitData, &g_pParticleArray1));
    DXUT_SetDebugName(g_pParticleArray0, "ParticleArray0");
    DXUT_SetDebugName(g_pParticleArray1, "ParticleArray1");
    SAFE_DELETE_ARRAY(pData1);

    ///Set initial Volumetric data
    D3D11_SUBRESOURCE_DATA v1data;
    v1data.pSysMem = vData1;
    D3D11_SUBRESOURCE_DATA v2data;
    v2data.pSysMem = vData2;
    V_RETURN(pd3dDevice->CreateBuffer(&vdesc, &v1data, &g_pVolCube1));
    V_RETURN(pd3dDevice->CreateBuffer(&vdesc, &v1data, &g_pVolCube1c));
    DXUT_SetDebugName(g_pVolCube1, "VolCube1");
    DXUT_SetDebugName(g_pVolCube1c, "VolCube1c");
    SAFE_DELETE_ARRAY(vData1);
    vdesc.ByteWidth = VOLCUBE2_COUNT * sizeof(MASSPOINT);
    V_RETURN(pd3dDevice->CreateBuffer(&vdesc, &v2data, &g_pVolCube2));
    V_RETURN(pd3dDevice->CreateBuffer(&vdesc, &v2data, &g_pVolCube2c));
    DXUT_SetDebugName(g_pVolCube2, "VolCube2");
    DXUT_SetDebugName(g_pVolCube2c, "VolCube2c");
    SAFE_DELETE_ARRAY(vData2);


    /// Buffer for IndexCube
    D3D11_BUFFER_DESC desc2;
    ZeroMemory(&desc2, sizeof(desc2));
    desc2.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;
    desc2.ByteWidth = g_nNumParticles * sizeof(INDEXER);
    desc2.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
    desc2.StructureByteStride = sizeof(INDEXER);
    desc2.Usage = D3D11_USAGE_DEFAULT;
    D3D11_SUBRESOURCE_DATA indexer_init;
    indexer_init.pSysMem = indexcube;
    V_RETURN(pd3dDevice->CreateBuffer(&desc2, &indexer_init, &g_pIndexCube));
    DXUT_SetDebugName(g_pIndexCube, "IndexCube");
    SAFE_DELETE_ARRAY(indexcube);

    /// RV for Particle data
    D3D11_SHADER_RESOURCE_VIEW_DESC DescRV;
    ZeroMemory(&DescRV, sizeof(DescRV));
    DescRV.Format = DXGI_FORMAT_UNKNOWN;
    DescRV.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
    DescRV.Buffer.FirstElement = 0;
    DescRV.Buffer.NumElements = desc.ByteWidth / desc.StructureByteStride;
    V_RETURN(pd3dDevice->CreateShaderResourceView(g_pParticleArray0, &DescRV, &g_pParticleArrayRV0));
    V_RETURN(pd3dDevice->CreateShaderResourceView(g_pParticleArray1, &DescRV, &g_pParticleArrayRV1));
    DXUT_SetDebugName(g_pParticleArrayRV0, "ParticleArray0 SRV");
    DXUT_SetDebugName(g_pParticleArrayRV1, "ParticleArray1 SRV");

    /// SRV for indexcube
    D3D11_SHADER_RESOURCE_VIEW_DESC DescRV2;
    ZeroMemory(&DescRV2, sizeof(DescRV2));
    DescRV2.Format = DXGI_FORMAT_UNKNOWN;
    DescRV2.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
    DescRV2.Buffer.FirstElement = 0;
    DescRV2.Buffer.NumElements = desc2.ByteWidth / desc2.StructureByteStride;
    V_RETURN(pd3dDevice->CreateShaderResourceView(g_pIndexCube, &DescRV2, &g_pIndexCubeRV));
    DXUT_SetDebugName(g_pIndexCubeRV, "IndexCube SRV");

    /// SRV for VolCubes
    D3D11_SHADER_RESOURCE_VIEW_DESC DescRVV;
    ZeroMemory(&DescRVV, sizeof(DescRVV));
    DescRVV.Format = DXGI_FORMAT_UNKNOWN;
    DescRVV.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
    DescRVV.Buffer.FirstElement = 0;
    DescRVV.Buffer.NumElements = VOLCUBE1_COUNT;
    V_RETURN(pd3dDevice->CreateShaderResourceView(g_pVolCube1, &DescRVV, &g_pVolCube1RV));
    V_RETURN(pd3dDevice->CreateShaderResourceView(g_pVolCube1c, &DescRVV, &g_pVolCube1cRV));
    DXUT_SetDebugName(g_pVolCube1RV, "VolCube1 RV");
    DXUT_SetDebugName(g_pVolCube1cRV, "VolCube1c RV");
    DescRVV.Buffer.NumElements = VOLCUBE2_COUNT;
    V_RETURN(pd3dDevice->CreateShaderResourceView(g_pVolCube2, &DescRVV, &g_pVolCube2RV));
    V_RETURN(pd3dDevice->CreateShaderResourceView(g_pVolCube2c, &DescRVV, &g_pVolCube2cRV));
    DXUT_SetDebugName(g_pVolCube2RV, "VolCube2 RV");
    DXUT_SetDebugName(g_pVolCube2cRV, "VolCube2c RV");

    /// UAV for Particle data
    D3D11_UNORDERED_ACCESS_VIEW_DESC DescUAV;
    ZeroMemory(&DescUAV, sizeof(D3D11_UNORDERED_ACCESS_VIEW_DESC));
    DescUAV.Format = DXGI_FORMAT_UNKNOWN;
    DescUAV.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
    DescUAV.Buffer.FirstElement = 0;
    DescUAV.Buffer.NumElements = desc.ByteWidth / desc.StructureByteStride;
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(g_pParticleArray0, &DescUAV, &g_pParticleArrayUAV0));
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(g_pParticleArray1, &DescUAV, &g_pParticleArrayUAV1));
    DXUT_SetDebugName(g_pParticleArrayUAV0, "ParticleArray0 UAV");
    DXUT_SetDebugName(g_pParticleArrayUAV1, "ParticleArray1 UAV");

    /// UAVs for Volumetric data
    D3D11_UNORDERED_ACCESS_VIEW_DESC vDescUAV;
    ZeroMemory(&vDescUAV, sizeof(D3D11_UNORDERED_ACCESS_VIEW_DESC));
    vDescUAV.Format = DXGI_FORMAT_UNKNOWN;
    vDescUAV.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
    vDescUAV.Buffer.FirstElement = 0;
    vDescUAV.Buffer.NumElements = (VOLCUBE1_COUNT * sizeof(MASSPOINT)) / vdesc.StructureByteStride;
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(g_pVolCube1, &vDescUAV, &g_pVolCube1UAV));
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(g_pVolCube1c, &vDescUAV, &g_pVolCube1cUAV));
    DXUT_SetDebugName(g_pVolCube1UAV, "VolCube1 UAV");
    DXUT_SetDebugName(g_pVolCube1cUAV, "VolCube1c UAV");
    vDescUAV.Buffer.NumElements = (VOLCUBE2_COUNT * sizeof(MASSPOINT)) / vdesc.StructureByteStride;
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(g_pVolCube2, &vDescUAV, &g_pVolCube2UAV));
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(g_pVolCube2c, &vDescUAV, &g_pVolCube2cUAV));
    DXUT_SetDebugName(g_pVolCube2UAV, "VolCube2 UAV");
    DXUT_SetDebugName(g_pVolCube2cUAV, "VolCube2c UAV");

    return hr;
}

bool CALLBACK ModifyDeviceSettings(DXUTDeviceSettings* pDeviceSettings, void* pUserContext)
{
    //assert( pDeviceSettings->ver == DXUT_D3D11_DEVICE );
    //// For the first device created if it is a REF device, optionally display a warning dialog box
    //static bool s_bFirstTime = true;
    //if( s_bFirstTime )
    //{
    //    s_bFirstTime = false;
    //    if( ( DXUT_D3D9_DEVICE == pDeviceSettings->ver && pDeviceSettings->d3d9.DeviceType == D3DDEVTYPE_REF ) ||
    //        ( DXUT_D3D11_DEVICE == pDeviceSettings->ver &&
    //        pDeviceSettings->d3d11.DriverType == D3D_DRIVER_TYPE_REFERENCE ) )
    //    {
    //        DXUTDisplaySwitchingToREFWarning( pDeviceSettings->ver );
    //    }
    //}
    return true;
}

template <class T>
void SWAP(T* &x, T* &y)
{
    T* temp = x;
    x = y;
    y = temp;
}

//--------------------------------------------------------------------------------------
// This callback function will be called once at the beginning of every frame. This is the
// best location for your application to handle updates to the scene, but is not 
// intended to contain actual rendering calls, which should instead be placed in the 
// OnFrameRender callback.  
//--------------------------------------------------------------------------------------
void CALLBACK OnFrameMove(double fTime, float fElapsedTime, void* pUserContext)
{
    HRESULT hr;

    auto pd3dImmediateContext = DXUTGetD3D11DeviceContext();

    //--------------------------------------------------------------------------------------
    // EXECUTE FIRST COMPUTE SHADER: UPDATE VOLUMETRIC MODELS
    pd3dImmediateContext->CSSetShader(g_pCompute1CS, nullptr, 0);

    ID3D11ShaderResourceView* srvs[4] = { g_pVolCube1cRV, g_pVolCube2cRV, g_pModelSRV[0], g_pModelSRV[1] };
    pd3dImmediateContext->CSSetShaderResources(0, 4, srvs);

    ID3D11UnorderedAccessView* aUAViews[2] = { g_pVolCube1UAV, g_pVolCube2UAV };
    pd3dImmediateContext->CSSetUnorderedAccessViews(0, 2, aUAViews, (UINT*)(&aUAViews));

    // For CS constant buffer
    D3D11_MAPPED_SUBRESOURCE MappedResource;
    V(pd3dImmediateContext->Map(g_pcbCS, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource));
    CB_CS* pcbCS = (CB_CS*)MappedResource.pData;

    // Update CS constant buffer
    pcbCS->vcwidth = VCUBEWIDTH;
    pcbCS->vccell = g_nVolCubeCell * 1.2f;
    pcbCS->numparticles = g_nNumParticles;
    pcbCS->stiffness = g_fStiffness;
    pcbCS->damping = g_fDamping;
    pcbCS->dt = fElapsedTime;
    pcbCS->im = g_fInvMass;

    // Send picking data to GPU
    if (g_bPicking)
        pcbCS->is_picking = 1;
    else
        pcbCS->is_picking = 0;

    XMMATRIX view = g_Camera.GetViewMatrix();			// V matrix
    XMMATRIX proj = g_Camera.GetProjMatrix();			// P matrix
    XMVECTOR eye = g_Camera.GetEyePt();					// eye pos

    XMMATRIX viewproj = XMMatrixMultiply(view, proj);	// VP matrix
    XMMATRIX trans = XMMatrixTranslation(XMVectorGetX(eye), XMVectorGetY(eye), XMVectorGetZ(eye));	// E matrix

    XMVECTOR pickdir = XMVectorSet((float)((2.0f*g_nCurrentMouseX) / g_nWindowWidth) - 1.0f,
        (-1)*((float)((2.0f*g_nCurrentMouseY) / g_nWindowHeight) - 1.0f), 0, 1);		// current mouse ndc
    trans = XMMatrixInverse(nullptr, XMMatrixMultiply(trans, viewproj));							// (E*VP)^-1
    pickdir = XMVector3Normalize(XMVector4Transform(pickdir, trans));								// pick direction = cmouse_ndc * (E*VP)^-1  --> normalized
    XMVectorSetW(pickdir, 1.0f);

    pcbCS->pickOriginX = g_nPickOriginX;
    pcbCS->pickOriginY = g_nPickOriginY;
    XMStoreFloat4(&pcbCS->pickDir, pickdir);
    XMStoreFloat4(&pcbCS->eyePos, eye);

    pd3dImmediateContext->Unmap(g_pcbCS, 0);
    ID3D11Buffer* ppCB[1] = { g_pcbCS };
    pd3dImmediateContext->CSSetConstantBuffers(0, 1, ppCB);

    // Run first CS (first volcube)
    pd3dImmediateContext->Dispatch(VCUBEWIDTH, VCUBEWIDTH, VCUBEWIDTH);

    // Run second CS (second volcube)
    pd3dImmediateContext->CSSetShader(g_pCompute2CS, nullptr, 0);
    pd3dImmediateContext->Dispatch(VCUBEWIDTH + 1, VCUBEWIDTH + 1, VCUBEWIDTH + 1);

    // Unbind resources for CS
    ID3D11ShaderResourceView* srvnull[4] = { nullptr, nullptr, nullptr, nullptr };
    pd3dImmediateContext->CSSetShaderResources(0, 4, srvnull);

    ID3D11UnorderedAccessView* ppUAViewNULL[2] = { nullptr, nullptr };
    pd3dImmediateContext->CSSetUnorderedAccessViews(0, 2, ppUAViewNULL, (UINT*)(&aUAViews));

    // SWAP resources
    SWAP(g_pVolCube1, g_pVolCube1c);
    SWAP(g_pVolCube1RV, g_pVolCube1cRV);
    SWAP(g_pVolCube1UAV, g_pVolCube1cUAV);
    SWAP(g_pVolCube2, g_pVolCube2c);
    SWAP(g_pVolCube2RV, g_pVolCube2cRV);
    SWAP(g_pVolCube2UAV, g_pVolCube2cUAV);

    //--------------------------------------------------------------------------------------
    // EXECUTE SECOND COMPUTE SHADER: UPDATE POSITIONS
    pd3dImmediateContext->CSSetShader(g_pUdateCS, nullptr, 0);

    ID3D11ShaderResourceView* uaRViews[1] = { g_pIndexCubeRV };
    pd3dImmediateContext->CSSetShaderResources(0, 1, uaRViews);
    ID3D11UnorderedAccessView* uaUAViews[3] = { g_pParticleArrayUAV1, g_pVolCube1UAV, g_pVolCube2UAV };
    pd3dImmediateContext->CSSetUnorderedAccessViews(0, 3, uaUAViews, (UINT*)(&uaUAViews));

    //pd3dImmediateContext->Dispatch(g_nNumParticles, 1, 1);
    pd3dImmediateContext->Dispatch((g_nNumParticles + VOLCUBE1_COUNT + VOLCUBE2_COUNT), 1, 1);

    ID3D11UnorderedAccessView* uppUAViewNULL[3] = { nullptr, nullptr, nullptr };
    pd3dImmediateContext->CSSetUnorderedAccessViews(0, 3, uppUAViewNULL, (UINT*)(&uaUAViews));
    ID3D11ShaderResourceView* uppSRVNULL[1] = { nullptr };
    pd3dImmediateContext->CSSetShaderResources(0, 1, uppSRVNULL);

    // SWAP RESOURCES
    SWAP(g_pParticleArray0, g_pParticleArray1);
    SWAP(g_pParticleArrayRV0, g_pParticleArrayRV1);
    SWAP(g_pParticleArrayUAV0, g_pParticleArrayUAV1);

    // Update the camera's position based on user input 
    g_Camera.FrameMove(fElapsedTime);
}


//--------------------------------------------------------------------------------------
// Poke rendered object 
//--------------------------------------------------------------------------------------
void PokeModel()
{
    print_debug("model poked");
}

//--------------------------------------------------------------------------------------
// Print debug information to file
//--------------------------------------------------------------------------------------
void print_debug(const char* string)
{
    time_t cti = time(nullptr);
    struct tm* ctime = localtime(&cti);
    debug.open("debug.txt", std::ios::out | std::ios::app);
    debug << "[" << 1900 + ctime->tm_year << "." << std::setw(2) << std::setfill('0') << ctime->tm_mon << "." << ctime->tm_mday << ". " << ctime->tm_hour << ":" << ctime->tm_min << ":" << ctime->tm_sec << "]\t" << string << std::endl;
    debug.close();
}

//--------------------------------------------------------------------------------------
// Before handling window messages, DXUT passes incoming windows 
// messages to the application through this callback function. If the application sets 
// *pbNoFurtherProcessing to TRUE, then DXUT will not process this message.
//--------------------------------------------------------------------------------------
LRESULT CALLBACK MsgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam, bool* pbNoFurtherProcessing,
    void* pUserContext)
{
    // Pass messages to dialog resource manager calls so GUI state is updated correctly
    *pbNoFurtherProcessing = g_DialogResourceManager.MsgProc(hWnd, uMsg, wParam, lParam);
    if (*pbNoFurtherProcessing)
        return 0;

    // Pass messages to settings dialog if its active
    if (g_D3DSettingsDlg.IsActive())
    {
        g_D3DSettingsDlg.MsgProc(hWnd, uMsg, wParam, lParam);
        return 0;
    }

    // Give the dialogs a chance to handle the message first
    *pbNoFurtherProcessing = g_HUD.MsgProc(hWnd, uMsg, wParam, lParam);
    if (*pbNoFurtherProcessing)
        return 0;
    *pbNoFurtherProcessing = g_SampleUI.MsgProc(hWnd, uMsg, wParam, lParam);
    if (*pbNoFurtherProcessing)
        return 0;

    if (uMsg == WM_RBUTTONDOWN){
        g_bPicking = true;
        g_bRM2Texture = true;
        g_nPickOriginX = (short)LOWORD(lParam);
        g_nPickOriginY = (short)HIWORD(lParam);
        g_nCurrentMouseX = (short)LOWORD(lParam);
        g_nCurrentMouseY = (short)HIWORD(lParam);
    }
    else if (uMsg == WM_MOUSEMOVE){
        g_nCurrentMouseX = (short)LOWORD(lParam);
        g_nCurrentMouseY = (short)HIWORD(lParam);
    }
    else if (uMsg == WM_RBUTTONUP){
        g_bPicking = false;
    }

    // Pass all windows messages to camera so it can respond to user input
    g_Camera.HandleMessages(hWnd, uMsg, wParam, lParam);

    return 0;
}


//------------------------------------------------------
// Handle key presses
//------------------------------------------------------
void CALLBACK OnKeyboard(UINT nChar, bool bKeyDown, bool bAltDown, void* pUserContext)
{
    XMVECTOR lookat = g_Camera.GetLookAtPt();
    XMVECTOR eye = XMVectorSet(0, 0, 0, 0);
    XMVECTOR x = XMVectorSet(200, 0, 0, 0);
    XMVECTOR y = XMVectorSet(0, 200, 0, 0);
    XMVECTOR v;
    switch (nChar){
    case VK_LEFT:	v = XMVectorSubtract(g_Camera.GetEyePt(), x);
        XMStoreFloat4(reinterpret_cast<XMFLOAT4*>(&eye), v);
        g_Camera.SetViewParams(eye, lookat);
        break;
    case VK_RIGHT:	v = XMVectorAdd(g_Camera.GetEyePt(), x);
        XMStoreFloat4(reinterpret_cast<XMFLOAT4*>(&eye), v);
        g_Camera.SetViewParams(eye, lookat);
        break;
    case VK_UP:		v = XMVectorAdd(g_Camera.GetEyePt(), y);
        XMStoreFloat4(reinterpret_cast<XMFLOAT4*>(&eye), v);
        g_Camera.SetViewParams(eye, lookat);
        break;
    case VK_DOWN:	v = XMVectorSubtract(g_Camera.GetEyePt(), y);
        XMStoreFloat4(reinterpret_cast<XMFLOAT4*>(&eye), v);
        g_Camera.SetViewParams(eye, lookat);
        break;
    case 0x58:
    {
                 SAFE_RELEASE(g_pParticleArray0);
                 SAFE_RELEASE(g_pParticleArray1);
                 SAFE_RELEASE(g_pParticleArrayRV0);
                 SAFE_RELEASE(g_pParticleArrayRV1);
                 SAFE_RELEASE(g_pParticleArrayUAV0);
                 SAFE_RELEASE(g_pParticleArrayUAV1);
                 SAFE_RELEASE(g_pIndexCube);
                 SAFE_RELEASE(g_pIndexCubeRV);
                 SAFE_RELEASE(g_pVolCube1);
                 SAFE_RELEASE(g_pVolCube1c);
                 SAFE_RELEASE(g_pVolCube1RV);
                 SAFE_RELEASE(g_pVolCube1cRV);
                 SAFE_RELEASE(g_pVolCube1UAV);
                 SAFE_RELEASE(g_pVolCube1cUAV);
                 SAFE_RELEASE(g_pVolCube2);
                 SAFE_RELEASE(g_pVolCube2c);
                 SAFE_RELEASE(g_pVolCube2RV);
                 SAFE_RELEASE(g_pVolCube2cRV);
                 SAFE_RELEASE(g_pVolCube2UAV);
                 SAFE_RELEASE(g_pVolCube2cUAV);
                 CreateParticlePosVeloBuffers(DXUTGetD3D11Device());
                 break;
    }
    default: break;
    }
}


//--------------------------------------------------------------------------------------
// Handles the GUI events
//--------------------------------------------------------------------------------------
void CALLBACK OnGUIEvent(UINT nEvent, int nControlID, CDXUTControl* pControl, void* pUserContext)
{
    switch (nControlID)
    {
    case IDC_TOGGLEFULLSCREEN:
        DXUTToggleFullScreen(); break;
    case IDC_TOGGLEREF:
        DXUTToggleREF(); break;
    case IDC_CHANGEDEVICE:
        g_D3DSettingsDlg.SetActive(!g_D3DSettingsDlg.IsActive()); break;

    case IDC_RESETPARTICLES:
    {
                               SAFE_RELEASE(g_pParticleArray0);
                               SAFE_RELEASE(g_pParticleArray1);
                               SAFE_RELEASE(g_pParticleArrayRV0);
                               SAFE_RELEASE(g_pParticleArrayRV1);
                               SAFE_RELEASE(g_pParticleArrayUAV0);
                               SAFE_RELEASE(g_pParticleArrayUAV1);
                               SAFE_RELEASE(g_pIndexCube);
                               SAFE_RELEASE(g_pIndexCubeRV);
                               SAFE_RELEASE(g_pVolCube1);
                               SAFE_RELEASE(g_pVolCube1c);
                               SAFE_RELEASE(g_pVolCube1RV);
                               SAFE_RELEASE(g_pVolCube1cRV);
                               SAFE_RELEASE(g_pVolCube1UAV);
                               SAFE_RELEASE(g_pVolCube1cUAV);
                               SAFE_RELEASE(g_pVolCube2);
                               SAFE_RELEASE(g_pVolCube2c);
                               SAFE_RELEASE(g_pVolCube2RV);
                               SAFE_RELEASE(g_pVolCube2cRV);
                               SAFE_RELEASE(g_pVolCube2UAV);
                               SAFE_RELEASE(g_pVolCube2cUAV);
                               CreateParticlePosVeloBuffers(DXUTGetD3D11Device());
                               break;
    }
    case IDC_POKEMODEL:
        PokeModel(); break;
    }
}

bool CALLBACK IsD3D11DeviceAcceptable(const CD3D11EnumAdapterInfo *AdapterInfo, UINT Output, const CD3D11EnumDeviceInfo *DeviceInfo,
    DXGI_FORMAT BackBufferFormat, bool bWindowed, void* pUserContext)
{
    // reject any device which doesn't support CS4x
    if (DeviceInfo->ComputeShaders_Plus_RawAndStructuredBuffers_Via_Shader_4_x == FALSE)
        return false;

    return true;
}

//--------------------------------------------------------------------------------------
// Create any D3D11 resources that aren't dependant on the back buffer
//--------------------------------------------------------------------------------------
HRESULT CALLBACK OnD3D11CreateDevice(ID3D11Device* pd3dDevice, const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc,
    void* pUserContext)
{

    HRESULT hr;

    static bool bFirstOnCreateDevice = true;

    // Warn the user that in order to support CS4x, a non-hardware device has been created, continue or quit?
    if (DXUTGetDeviceSettings().d3d11.DriverType != D3D_DRIVER_TYPE_HARDWARE && bFirstOnCreateDevice)
    {
        if (MessageBox(0, L"CS4x capability is missing. "\
            L"In order to continue, a non-hardware device has been created, "\
            L"it will be very slow, continue?", L"Warning", MB_ICONEXCLAMATION | MB_YESNO) != IDYES)
            return E_FAIL;
    }

    CWaitDlg CompilingShadersDlg;
    CompilingShadersDlg.ShowDialog(L"Compiling Shaders...");

    bFirstOnCreateDevice = false;

    D3D11_FEATURE_DATA_D3D10_X_HARDWARE_OPTIONS ho;
    V_RETURN(pd3dDevice->CheckFeatureSupport(D3D11_FEATURE_D3D10_X_HARDWARE_OPTIONS, &ho, sizeof(ho)));

    auto pd3dImmediateContext = DXUTGetD3D11DeviceContext();
    V_RETURN(g_DialogResourceManager.OnD3D11CreateDevice(pd3dDevice, pd3dImmediateContext));
    V_RETURN(g_D3DSettingsDlg.OnD3D11CreateDevice(pd3dDevice));
    g_pTxtHelper = new CDXUTTextHelper(pd3dDevice, pd3dImmediateContext, &g_DialogResourceManager, 15);

    ID3DBlob* pBlobRenderParticlesVS = nullptr;
    ID3DBlob* pBlobRenderParticlesGS = nullptr;
    ID3DBlob* pBlobRenderParticlesPS = nullptr;
    ID3DBlob* pBlobModelPS1 = nullptr;
    ID3DBlob* pBlobModelPS2 = nullptr;
    ID3DBlob* pBlobCalc1CS = nullptr;
    ID3DBlob* pBlobCalc2CS = nullptr;
    ID3DBlob* pBlobUpdateCS = nullptr;

    // Create the shaders
    V_RETURN(DXUTCompileFromFile(L"ParticleDraw.hlsl", nullptr, "VSParticleDraw", "vs_5_0", D3DCOMPILE_ENABLE_STRICTNESS, 0, &pBlobRenderParticlesVS));
    V_RETURN(DXUTCompileFromFile(L"ParticleDraw.hlsl", nullptr, "GSParticleDraw", "gs_5_0", D3DCOMPILE_ENABLE_STRICTNESS, 0, &pBlobRenderParticlesGS));
    V_RETURN(DXUTCompileFromFile(L"ParticleDraw.hlsl", nullptr, "PSParticleDraw", "ps_5_0", D3DCOMPILE_ENABLE_STRICTNESS, 0, &pBlobRenderParticlesPS));
    V_RETURN(DXUTCompileFromFile(L"ParticleDraw.hlsl", nullptr, "PSModelDraw1", "ps_5_0", D3DCOMPILE_ENABLE_STRICTNESS, 0, &pBlobModelPS1));
    V_RETURN(DXUTCompileFromFile(L"ParticleDraw.hlsl", nullptr, "PSModelDraw2", "ps_5_0", D3DCOMPILE_ENABLE_STRICTNESS, 0, &pBlobModelPS2));
    V_RETURN(DXUTCompileFromFile(L"Deformation.hlsl", nullptr, "CSMain1", "cs_5_0", D3DCOMPILE_ENABLE_STRICTNESS, 0, &pBlobCalc1CS));
    V_RETURN(DXUTCompileFromFile(L"Deformation.hlsl", nullptr, "CSMain2", "cs_5_0", D3DCOMPILE_ENABLE_STRICTNESS, 0, &pBlobCalc2CS));
    V_RETURN(DXUTCompileFromFile(L"PosUpdate.hlsl", nullptr, "PosUpdate", "cs_5_0", D3DCOMPILE_ENABLE_STRICTNESS, 0, &pBlobUpdateCS));


    V_RETURN(pd3dDevice->CreateVertexShader(pBlobRenderParticlesVS->GetBufferPointer(), pBlobRenderParticlesVS->GetBufferSize(), nullptr, &g_pRenderParticlesVS));
    DXUT_SetDebugName(g_pRenderParticlesVS, "VSParticleDraw");

    V_RETURN(pd3dDevice->CreateGeometryShader(pBlobRenderParticlesGS->GetBufferPointer(), pBlobRenderParticlesGS->GetBufferSize(), nullptr, &g_pRenderParticlesGS));
    DXUT_SetDebugName(g_pRenderParticlesGS, "GSParticleDraw");

    V_RETURN(pd3dDevice->CreatePixelShader(pBlobRenderParticlesPS->GetBufferPointer(), pBlobRenderParticlesPS->GetBufferSize(), nullptr, &g_pRenderParticlesPS));
    DXUT_SetDebugName(g_pRenderParticlesPS, "PSParticleDraw");

    V_RETURN(pd3dDevice->CreatePixelShader(pBlobModelPS1->GetBufferPointer(), pBlobModelPS1->GetBufferSize(), nullptr, &g_pModelPS1));
    DXUT_SetDebugName(g_pModelPS1, "PSModelDraw1");

    V_RETURN(pd3dDevice->CreatePixelShader(pBlobModelPS2->GetBufferPointer(), pBlobModelPS2->GetBufferSize(), nullptr, &g_pModelPS2));
    DXUT_SetDebugName(g_pModelPS2, "PSModelDraw2");

    V_RETURN(pd3dDevice->CreateComputeShader(pBlobCalc1CS->GetBufferPointer(), pBlobCalc1CS->GetBufferSize(), nullptr, &g_pCompute1CS));
    DXUT_SetDebugName(g_pCompute1CS, "CSMain1");

    V_RETURN(pd3dDevice->CreateComputeShader(pBlobCalc2CS->GetBufferPointer(), pBlobCalc2CS->GetBufferSize(), nullptr, &g_pCompute2CS));
    DXUT_SetDebugName(g_pCompute2CS, "CSMain2");

    V_RETURN(pd3dDevice->CreateComputeShader(pBlobUpdateCS->GetBufferPointer(), pBlobUpdateCS->GetBufferSize(), nullptr, &g_pUdateCS));
    DXUT_SetDebugName(g_pUdateCS, "PosUpdate");

    // Create our vertex input layout
    const D3D11_INPUT_ELEMENT_DESC layout[] =
    {
        { "COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
    };
    V_RETURN(pd3dDevice->CreateInputLayout(layout, sizeof(layout) / sizeof(layout[0]),
        pBlobRenderParticlesVS->GetBufferPointer(), pBlobRenderParticlesVS->GetBufferSize(), &g_pParticleVertexLayout));
    DXUT_SetDebugName(g_pParticleVertexLayout, "Particles' layout");


    SAFE_RELEASE(pBlobRenderParticlesVS);
    SAFE_RELEASE(pBlobRenderParticlesGS);
    SAFE_RELEASE(pBlobRenderParticlesPS);
    SAFE_RELEASE(pBlobModelPS1);
    SAFE_RELEASE(pBlobModelPS2);
    SAFE_RELEASE(pBlobCalc1CS);
    SAFE_RELEASE(pBlobCalc2CS);
    SAFE_RELEASE(pBlobUpdateCS);

    // Read model file, set global volcube parameters
    ReadModel();
    ModelRenderBuffers(pd3dDevice, 800, 600);
    V_RETURN(CreateParticleBuffer(pd3dDevice));
    V_RETURN(CreateParticlePosVeloBuffers(pd3dDevice));

    // Setup constant buffer
    D3D11_BUFFER_DESC Desc;
    Desc.Usage = D3D11_USAGE_DYNAMIC;
    Desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
    Desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    Desc.MiscFlags = 0;
    Desc.ByteWidth = sizeof(CB_GS);
    V_RETURN(pd3dDevice->CreateBuffer(&Desc, nullptr, &g_pcbGS));
    DXUT_SetDebugName(g_pcbGS, "CB_GS");

    Desc.ByteWidth = sizeof(CB_CS);
    V_RETURN(pd3dDevice->CreateBuffer(&Desc, nullptr, &g_pcbCS));
    DXUT_SetDebugName(g_pcbCS, "CB_CS");

    // Load the Particle Texture
    V_RETURN(DXUTCreateShaderResourceViewFromFile(pd3dDevice, L"misc\\Particle.dds", &g_pParticleTexRV));
    DXUT_SetDebugName(g_pParticleTexRV, "Particle.dds");

    D3D11_SAMPLER_DESC SamplerDesc;
    ZeroMemory(&SamplerDesc, sizeof(SamplerDesc));
    SamplerDesc.AddressU = D3D11_TEXTURE_ADDRESS_CLAMP;
    SamplerDesc.AddressV = D3D11_TEXTURE_ADDRESS_CLAMP;
    SamplerDesc.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
    SamplerDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
    V_RETURN(pd3dDevice->CreateSamplerState(&SamplerDesc, &g_pSampleStateLinear));
    DXUT_SetDebugName(g_pSampleStateLinear, "Linear");

    D3D11_BLEND_DESC BlendStateDesc;
    ZeroMemory(&BlendStateDesc, sizeof(BlendStateDesc));
    BlendStateDesc.RenderTarget[0].BlendEnable = TRUE;
    BlendStateDesc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
    BlendStateDesc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
    BlendStateDesc.RenderTarget[0].DestBlend = D3D11_BLEND_ONE;
    BlendStateDesc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
    BlendStateDesc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ZERO;
    BlendStateDesc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ZERO;
    BlendStateDesc.RenderTarget[0].RenderTargetWriteMask = 0x0F;
    V_RETURN(pd3dDevice->CreateBlendState(&BlendStateDesc, &g_pBlendingStateParticle));
    DXUT_SetDebugName(g_pBlendingStateParticle, "Blending");

    D3D11_DEPTH_STENCIL_DESC DepthStencilDesc;
    ZeroMemory(&DepthStencilDesc, sizeof(DepthStencilDesc));
    DepthStencilDesc.DepthEnable = FALSE;
    DepthStencilDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
    pd3dDevice->CreateDepthStencilState(&DepthStencilDesc, &g_pDepthStencilState);
    DXUT_SetDebugName(g_pDepthStencilState, "DepthOff");

    // Setup the camera's view parameters
    XMVECTOR vecEye = XMVectorSet(-g_fSpread * 0, g_fSpread * 0, -g_fSpread * 27, 0.0f);
    XMVECTOR vecAt = XMVectorSet(0.0f, 0.0f, 0.0f, 0.0f);
    g_Camera.SetViewParams(vecEye, vecAt);

    CompilingShadersDlg.DestroyDialog();

    return S_OK;
}

HRESULT CALLBACK OnD3D11ResizedSwapChain(ID3D11Device* pd3dDevice, IDXGISwapChain* pSwapChain,
    const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc, void* pUserContext)
{

    HRESULT hr = S_OK;

    V_RETURN(g_DialogResourceManager.OnD3D11ResizedSwapChain(pd3dDevice, pBackBufferSurfaceDesc));
    V_RETURN(g_D3DSettingsDlg.OnD3D11ResizedSwapChain(pd3dDevice, pBackBufferSurfaceDesc));

    // Setup the camera's projection parameters
    g_nWindowWidth = pBackBufferSurfaceDesc->Width;
    g_nWindowHeight = pBackBufferSurfaceDesc->Height;
    float fAspectRatio = pBackBufferSurfaceDesc->Width / (FLOAT)pBackBufferSurfaceDesc->Height;
    g_Camera.SetProjParams(XM_PI / 4, fAspectRatio, 10.0f, 500000.0f);
    g_Camera.SetWindow(pBackBufferSurfaceDesc->Width, pBackBufferSurfaceDesc->Height);
    g_Camera.SetButtonMasks(0, MOUSE_WHEEL, MOUSE_LEFT_BUTTON | MOUSE_MIDDLE_BUTTON);

    g_HUD.SetLocation(pBackBufferSurfaceDesc->Width - 170, 0);
    g_HUD.SetSize(170, 170);
    g_SampleUI.SetLocation(pBackBufferSurfaceDesc->Width - 170, pBackBufferSurfaceDesc->Height - 300);
    g_SampleUI.SetSize(170, 300);

    ModelRenderBuffers(pd3dDevice, pBackBufferSurfaceDesc->Width, pBackBufferSurfaceDesc->Height);

    return hr;
}

void CALLBACK OnD3D11ReleasingSwapChain(void* pUserContext)
{
    g_DialogResourceManager.OnD3D11ReleasingSwapChain();
}

void RenderText()
{
    g_pTxtHelper->Begin();
    g_pTxtHelper->SetInsertionPos(2, 0);
    g_pTxtHelper->SetForegroundColor(Colors::DarkTurquoise);
    g_pTxtHelper->DrawTextLine(DXUTGetFrameStats(DXUTIsVsyncEnabled()));
    g_pTxtHelper->DrawTextLine(DXUTGetDeviceStats());
    g_pTxtHelper->DrawTextLine(L"[Project Deformation v0.9]");
    std::wstring s = L"#particle: " + std::to_wstring(g_nNumParticles) + L", #masspoint: " + std::to_wstring(VOLCUBE1_COUNT + VOLCUBE2_COUNT) + L", stiff: " + std::to_wstring(g_fStiffness) + L", damp: " + std::to_wstring(g_fDamping);
    g_pTxtHelper->DrawTextLine(s.c_str());
    g_pTxtHelper->End();
}


//--------------------------------------------------------------------------------------
// (Re)Create texture and buffers for model render target (for picking)
//--------------------------------------------------------------------------------------
void ModelRenderBuffers(ID3D11Device* pd3dDevice, int width, int height)
{

    SAFE_RELEASE(g_pModelTex[0]);
    SAFE_RELEASE(g_pModelTex[1]);
    SAFE_RELEASE(g_pModelRTV[0]);
    SAFE_RELEASE(g_pModelRTV[1]);
    SAFE_RELEASE(g_pModelSRV[0]);
    SAFE_RELEASE(g_pModelSRV[1]);

    D3D11_TEXTURE2D_DESC tdesc;
    D3D11_RENDER_TARGET_VIEW_DESC rtvdesc;
    D3D11_SHADER_RESOURCE_VIEW_DESC srvdesc;

    ZeroMemory(&tdesc, sizeof(tdesc));
    tdesc.Width = width;
    tdesc.Height = height;
    tdesc.MipLevels = 1;
    tdesc.ArraySize = 2;
    tdesc.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
    tdesc.SampleDesc.Count = 1;
    tdesc.Usage = D3D11_USAGE_DEFAULT;
    tdesc.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;
    tdesc.CPUAccessFlags = 0;
    tdesc.MiscFlags = 0;

    pd3dDevice->CreateTexture2D(&tdesc, nullptr, &g_pModelTex[0]);
    DXUT_SetDebugName(g_pModelTex[0], "Model TEX 1");
    pd3dDevice->CreateTexture2D(&tdesc, nullptr, &g_pModelTex[1]);
    DXUT_SetDebugName(g_pModelTex[1], "Model TEX 2");

    rtvdesc.Format = tdesc.Format;
    rtvdesc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2D;
    rtvdesc.Texture2D.MipSlice = 0;
    pd3dDevice->CreateRenderTargetView(g_pModelTex[0], &rtvdesc, &g_pModelRTV[0]);
    DXUT_SetDebugName(g_pModelRTV[0], "Model RTV 1");
    pd3dDevice->CreateRenderTargetView(g_pModelTex[1], &rtvdesc, &g_pModelRTV[1]);
    DXUT_SetDebugName(g_pModelRTV[1], "Model RTV 2");

    srvdesc.Format = tdesc.Format;
    srvdesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
    srvdesc.Texture2D.MostDetailedMip = 0;
    srvdesc.Texture2D.MipLevels = 1;
    pd3dDevice->CreateShaderResourceView(g_pModelTex[0], &srvdesc, &g_pModelSRV[0]);
    DXUT_SetDebugName(g_pModelSRV[0], "Model SRV 1");
    pd3dDevice->CreateShaderResourceView(g_pModelTex[1], &srvdesc, &g_pModelSRV[1]);
    DXUT_SetDebugName(g_pModelSRV[1], "Model SRV 2");

}


//--------------------------------------------------------------------------------------
// Render model to texture, for picking / dragging
//--------------------------------------------------------------------------------------
bool RenderModel(ID3D11DeviceContext* pd3dImmediateContext, CXMMATRIX mView, CXMMATRIX mProj)
{

    ID3D11RenderTargetView* pRTV = DXUTGetD3D11RenderTargetView();
    ID3D11DepthStencilView* pDSV = DXUTGetD3D11DepthStencilView();
    pd3dImmediateContext->ClearDepthStencilView(pDSV, D3D11_CLEAR_DEPTH, 1.0, 0);
    pd3dImmediateContext->OMSetRenderTargets(1, &g_pModelRTV[0], pDSV);
    pd3dImmediateContext->ClearRenderTargetView(g_pModelRTV[0], Colors::Black);

    ID3D11BlendState *pBlendState0 = nullptr;
    ID3D11DepthStencilState *pDepthStencilState0 = nullptr;
    UINT SampleMask0, StencilRef0;
    XMFLOAT4 BlendFactor0;
    pd3dImmediateContext->OMGetBlendState(&pBlendState0, &BlendFactor0.x, &SampleMask0);
    pd3dImmediateContext->OMGetDepthStencilState(&pDepthStencilState0, &StencilRef0);

    pd3dImmediateContext->VSSetShader(g_pRenderParticlesVS, nullptr, 0);
    pd3dImmediateContext->GSSetShader(g_pRenderParticlesGS, nullptr, 0);
    pd3dImmediateContext->PSSetShader(g_pModelPS1, nullptr, 0);

    pd3dImmediateContext->IASetInputLayout(g_pParticleVertexLayout);

    // Set IA parameters
    ID3D11Buffer* pBuffers[1] = { g_pParticleBuffer };
    UINT stride[1] = { sizeof(PARTICLE_VERTEX) };
    UINT offset[1] = { 0 };
    pd3dImmediateContext->IASetVertexBuffers(0, 1, pBuffers, stride, offset);
    pd3dImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST);

    ID3D11ShaderResourceView* aRViews[1] = { g_pParticleArrayRV0 };
    pd3dImmediateContext->VSSetShaderResources(0, 1, aRViews);

    D3D11_MAPPED_SUBRESOURCE MappedResource;
    pd3dImmediateContext->Map(g_pcbGS, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource);
    auto pCBGS = reinterpret_cast<CB_GS*>(MappedResource.pData);
    XMStoreFloat4x4(&pCBGS->m_WorldViewProj, XMMatrixMultiply(mView, mProj));
    XMStoreFloat4x4(&pCBGS->m_InvView, XMMatrixInverse(nullptr, mView));
    pd3dImmediateContext->Unmap(g_pcbGS, 0);
    pd3dImmediateContext->GSSetConstantBuffers(0, 1, &g_pcbGS);

    pd3dImmediateContext->PSSetShaderResources(0, 1, &g_pParticleTexRV);
    pd3dImmediateContext->PSSetSamplers(0, 1, &g_pSampleStateLinear);

    pd3dImmediateContext->OMSetBlendState(nullptr, nullptr, 0xFFFFFFFF);
    pd3dImmediateContext->OMSetDepthStencilState(g_pDepthStencilState, 0);


    // Render first "layer" of information: vertex IDs in first volcube
    pd3dImmediateContext->Draw(g_nNumParticles, 0);

    // Render second layer of information: vertex IDs in second volcube
    pd3dImmediateContext->ClearDepthStencilView(pDSV, D3D11_CLEAR_DEPTH, 1.0, 0);
    pd3dImmediateContext->OMSetRenderTargets(1, &g_pModelRTV[1], pDSV);
    pd3dImmediateContext->ClearRenderTargetView(g_pModelRTV[1], Colors::Black);
    pd3dImmediateContext->PSSetShader(g_pModelPS2, nullptr, 0);
    pd3dImmediateContext->Draw(g_nNumParticles, 0);


    ID3D11ShaderResourceView* ppSRVNULL[1] = { nullptr };
    pd3dImmediateContext->VSSetShaderResources(0, 1, ppSRVNULL);
    pd3dImmediateContext->PSSetShaderResources(0, 1, ppSRVNULL);

    pd3dImmediateContext->GSSetShader(nullptr, nullptr, 0);
    pd3dImmediateContext->OMSetBlendState(pBlendState0, &BlendFactor0.x, SampleMask0); SAFE_RELEASE(pBlendState0);
    pd3dImmediateContext->OMSetDepthStencilState(pDepthStencilState0, StencilRef0); SAFE_RELEASE(pDepthStencilState0);

    pd3dImmediateContext->OMSetRenderTargets(1, &pRTV, pDSV);
    g_bRM2Texture = false;

    return true;
}

//--------------------------------------------------------------------------------------
// Render model to display
//--------------------------------------------------------------------------------------
bool RenderParticles(ID3D11DeviceContext* pd3dImmediateContext, CXMMATRIX mView, CXMMATRIX mProj)
{
    ID3D11BlendState *pBlendState0 = nullptr;
    ID3D11DepthStencilState *pDepthStencilState0 = nullptr;
    UINT SampleMask0, StencilRef0;
    XMFLOAT4 BlendFactor0;
    pd3dImmediateContext->OMGetBlendState(&pBlendState0, &BlendFactor0.x, &SampleMask0);
    pd3dImmediateContext->OMGetDepthStencilState(&pDepthStencilState0, &StencilRef0);

    pd3dImmediateContext->VSSetShader(g_pRenderParticlesVS, nullptr, 0);
    pd3dImmediateContext->GSSetShader(g_pRenderParticlesGS, nullptr, 0);
    pd3dImmediateContext->PSSetShader(g_pRenderParticlesPS, nullptr, 0);

    pd3dImmediateContext->IASetInputLayout(g_pParticleVertexLayout);

    // Set IA parameters
    ID3D11Buffer* pBuffers[1] = { g_pParticleBuffer };
    UINT stride[1] = { sizeof(PARTICLE_VERTEX) };
    UINT offset[1] = { 0 };
    pd3dImmediateContext->IASetVertexBuffers(0, 1, pBuffers, stride, offset);
    pd3dImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST);

    ID3D11ShaderResourceView* aRViews[1] = { g_pParticleArrayRV0 };
    pd3dImmediateContext->VSSetShaderResources(0, 1, aRViews);

    D3D11_MAPPED_SUBRESOURCE MappedResource;
    pd3dImmediateContext->Map(g_pcbGS, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource);
    auto pCBGS = reinterpret_cast<CB_GS*>(MappedResource.pData);
    XMStoreFloat4x4(&pCBGS->m_WorldViewProj, XMMatrixMultiply(mView, mProj));
    XMStoreFloat4x4(&pCBGS->m_InvView, XMMatrixInverse(nullptr, mView));
    pd3dImmediateContext->Unmap(g_pcbGS, 0);
    pd3dImmediateContext->GSSetConstantBuffers(0, 1, &g_pcbGS);

    pd3dImmediateContext->PSSetShaderResources(0, 1, &g_pParticleTexRV);
    pd3dImmediateContext->PSSetSamplers(0, 1, &g_pSampleStateLinear);

    float bf[] = { 0.f, 0.f, 0.f, 0.f };
    pd3dImmediateContext->OMSetBlendState(g_pBlendingStateParticle, bf, 0xFFFFFFFF);
    pd3dImmediateContext->OMSetDepthStencilState(g_pDepthStencilState, 0);

    pd3dImmediateContext->Draw((g_nNumParticles + VOLCUBE1_COUNT + VOLCUBE2_COUNT), 0);

    ID3D11ShaderResourceView* ppSRVNULL[1] = { nullptr };
    pd3dImmediateContext->VSSetShaderResources(0, 1, ppSRVNULL);
    pd3dImmediateContext->PSSetShaderResources(0, 1, ppSRVNULL);

    pd3dImmediateContext->GSSetShader(nullptr, nullptr, 0);
    pd3dImmediateContext->OMSetBlendState(pBlendState0, &BlendFactor0.x, SampleMask0); SAFE_RELEASE(pBlendState0);
    pd3dImmediateContext->OMSetDepthStencilState(pDepthStencilState0, StencilRef0); SAFE_RELEASE(pDepthStencilState0);

    return true;
}

void CALLBACK OnD3D11FrameRender(ID3D11Device* pd3dDevice, ID3D11DeviceContext* pd3dImmediateContext, double fTime,
    float fElapsedTime, void* pUserContext)
{

    // If the settings dialog is being shown, then render it instead of rendering the app's scene
    if (g_D3DSettingsDlg.IsActive())
    {
        g_D3DSettingsDlg.OnRender(fElapsedTime);
        return;
    }

    ID3D11RenderTargetView* pRTV = DXUTGetD3D11RenderTargetView();
    pd3dImmediateContext->ClearRenderTargetView(pRTV, Colors::Black);
    ID3D11DepthStencilView* pDSV = DXUTGetD3D11DepthStencilView();
    pd3dImmediateContext->ClearDepthStencilView(pDSV, D3D11_CLEAR_DEPTH, 1.0, 0);

    XMMATRIX mView = g_Camera.GetViewMatrix();
    XMMATRIX mProj = g_Camera.GetProjMatrix();

    // Render the model first for picking, only if RMBUTTONDOWN happened
    if (g_bRM2Texture){
        //ModelRenderBuffers(pd3dDevice,g_nWindowWidth,g_nWindowHeight);
        RenderModel(pd3dImmediateContext, mView, mProj);
    }

    // Render the particles
    RenderParticles(pd3dImmediateContext, mView, mProj);

    ////DXUT_BeginPerfEvent( DXUT_PERFEVENTCOLOR, L"HUD / Stats" );
    //g_HUD.OnRender(fElapsedTime);
    //g_SampleUI.OnRender(fElapsedTime);
    //RenderText();
    ////DXUT_EndPerfEvent();

    // The following could be used to output fps stats into debug output window,
    // which is useful because you can then turn off all UI rendering as they cloud performance
    static DWORD dwTimefirst = GetTickCount();
    if (GetTickCount() - dwTimefirst > 3000)
    {
        OutputDebugString(DXUTGetFrameStats(DXUTIsVsyncEnabled()));
        OutputDebugString(L"\n");
        dwTimefirst = GetTickCount();
    }
}

//--------------------------------------------------------------------------------------
// Release D3D11 resources created in OnD3D11CreateDevice 
//--------------------------------------------------------------------------------------
void CALLBACK OnD3D11DestroyDevice(void* pUserContext)
{
    g_DialogResourceManager.OnD3D11DestroyDevice();
    g_D3DSettingsDlg.OnD3D11DestroyDevice();
    DXUTGetGlobalResourceCache().OnDestroyDevice();
    SAFE_DELETE(g_pTxtHelper);
    SAFE_RELEASE(g_pParticleBuffer);
    SAFE_RELEASE(g_pParticleVertexLayout);
    SAFE_RELEASE(g_pParticleArray0);
    SAFE_RELEASE(g_pParticleArray1);
    SAFE_RELEASE(g_pIndexCube);
    SAFE_RELEASE(g_pIndexCubeRV);
    SAFE_RELEASE(g_pVolCube1);
    SAFE_RELEASE(g_pVolCube1c);
    SAFE_RELEASE(g_pVolCube2);
    SAFE_RELEASE(g_pVolCube2c);
    SAFE_RELEASE(g_pVolCube1RV);
    SAFE_RELEASE(g_pVolCube1cRV);
    SAFE_RELEASE(g_pVolCube2RV);
    SAFE_RELEASE(g_pVolCube2cRV);
    SAFE_RELEASE(g_pVolCube1UAV);
    SAFE_RELEASE(g_pVolCube1cUAV);
    SAFE_RELEASE(g_pVolCube2UAV);
    SAFE_RELEASE(g_pVolCube2cUAV);
    SAFE_RELEASE(g_pParticleArrayRV0);
    SAFE_RELEASE(g_pParticleArrayRV1);
    SAFE_RELEASE(g_pParticleArrayUAV0);
    SAFE_RELEASE(g_pParticleArrayUAV1);
    SAFE_RELEASE(g_pcbGS);
    SAFE_RELEASE(g_pcbCS);
    SAFE_RELEASE(g_pParticleTexRV);
    SAFE_RELEASE(g_pRenderParticlesVS);
    SAFE_RELEASE(g_pRenderParticlesGS);
    SAFE_RELEASE(g_pRenderParticlesPS);
    SAFE_RELEASE(g_pModelPS1);
    SAFE_RELEASE(g_pModelPS2);
    SAFE_RELEASE(g_pCompute1CS);
    SAFE_RELEASE(g_pCompute2CS);
    SAFE_RELEASE(g_pUdateCS);
    SAFE_RELEASE(g_pSampleStateLinear);
    SAFE_RELEASE(g_pBlendingStateParticle);
    SAFE_RELEASE(g_pDepthStencilState);
    SAFE_RELEASE(g_pModelTex[0]);
    SAFE_RELEASE(g_pModelTex[1]);
    SAFE_RELEASE(g_pModelRTV[0]);
    SAFE_RELEASE(g_pModelRTV[1]);
    SAFE_RELEASE(g_pModelSRV[0]);
    SAFE_RELEASE(g_pModelSRV[1]);
}
