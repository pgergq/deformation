//--------------------------------------------------------------------------------------
// File: Deformation.cpp
//
// Project Deformation
// Object deformation with mass-spring systems
//
// (Object size order in world space = O(100)
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
#include "Deformable.h"
#include "Constants.h"


using namespace DirectX;

//--------------------------------------------------------------------------------------
// Global variables
//--------------------------------------------------------------------------------------

// render variables
CModelViewerCamera                  g_Camera;
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
ID3D11Texture2D*					g_pModelTex[2] = { nullptr, nullptr };
ID3D11RenderTargetView*				g_pModelRTV[2] = { nullptr, nullptr };
ID3D11ShaderResourceView*			g_pModelSRV[2] = { nullptr, nullptr };
ID3D11Buffer*                       g_pcbGS = nullptr;
ID3D11ShaderResourceView*           g_pParticleTexRV = nullptr;

// Scene variables
std::vector<Deformable>				deformableObjects;				// scene objects
uint                                objectCount;
uint                                particleCount;
uint                                mass1Count;
uint                                mass2Count;
uint                                cubeCellSize;

// Window & picking variables
int									g_nWindowWidth = 800;
int									g_nWindowHeight = 600;
int									g_nCurrentMouseX;
int									g_nCurrentMouseY;
int									g_nPickOriginX;
int									g_nPickOriginY;
bool								g_bPicking = false;				// RBUTTON is pressed
bool								g_bRM2Texture = false;			// render model to texture for picking
XMFLOAT4							g_vLightPosition(0, 0, -10000, 1);// light position

// Debug file
std::ofstream						debug;

struct PARTICLE_VERTEX
{
    XMFLOAT4 Color;
};

struct CB_GS
{
    XMFLOAT4X4 m_WorldViewProj;
    XMFLOAT4X4 m_InvView;
    XMFLOAT4 eyepos;
    XMFLOAT4 lightpos;
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


//--------------------------------------------------------------------------------------
// Forward declarations 
//--------------------------------------------------------------------------------------
bool CALLBACK ModifyDeviceSettings(DXUTDeviceSettings* pDeviceSettings, void* pUserContext);
void CALLBACK OnKeyboard(UINT nChar, bool bKeyDown, bool bAltDown, void* pUserContext);
void CALLBACK OnFrameMove(double fTime, float fElapsedTime, void* pUserContext);
LRESULT CALLBACK MsgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam, bool* pbNoFurtherProcessing, void* pUserContext);
bool CALLBACK IsD3D11DeviceAcceptable(const CD3D11EnumAdapterInfo *AdapterInfo, UINT Output, const CD3D11EnumDeviceInfo *DeviceInfo, DXGI_FORMAT BackBufferFormat, bool bWindowed, void* pUserContext);
HRESULT CALLBACK OnD3D11CreateDevice(ID3D11Device* pd3dDevice, const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc, void* pUserContext);
HRESULT CALLBACK OnD3D11ResizedSwapChain(ID3D11Device* pd3dDevice, IDXGISwapChain* pSwapChain, const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc, void* pUserContext);
void CALLBACK OnD3D11DestroyDevice(void* pUserContext);
void CALLBACK OnD3D11FrameRender(ID3D11Device* pd3dDevice, ID3D11DeviceContext* pd3dImmediateContext, double fTime, float fElapsedTime, void* pUserContext);
HRESULT initBuffers(ID3D11Device* pd3dDevice);
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
    DXUTSetCallbackD3D11DeviceDestroyed(OnD3D11DestroyDevice);

    // Use this line instead to try to create a hardware device
    DXUTInit(true, true);

    // Show the cursor and clip it when in full screen
    DXUTSetCursorSettings(true, true);
    DXUTCreateWindow(L"Project Deformation");
    DXUTCreateDevice(D3D_FEATURE_LEVEL_11_0, true, 800, 600);

    // Enter into the DXUT render loop
    DXUTMainLoop();

    return DXUTGetExitCode();
}

//--------------------------------------------------------------------------------------
// On device settings modifications 
//--------------------------------------------------------------------------------------
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

//--------------------------------------------------------------------------------------
// Update data at the beginning of every frame (no rendering call, only data updates)  
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
    pcbCS->vccell = cubeCellSize;
    pcbCS->numparticles = particleCount;
    pcbCS->stiffness = g_fStiffness;
    pcbCS->damping = g_fDamping;
    pcbCS->dt = fElapsedTime;
    pcbCS->im = g_fInvMass;

    // Send picking data to GPU
    if (g_bPicking)
        pcbCS->is_picking = 1;
    else
        pcbCS->is_picking = 0;

    XMMATRIX view = g_Camera.GetViewMatrix();    // V matrix
    XMMATRIX proj = g_Camera.GetProjMatrix();    // P matrix
    XMVECTOR eye = g_Camera.GetEyePt();          // eye pos

    XMMATRIX viewproj = XMMatrixMultiply(view, proj);    // VP matrix
    XMMATRIX trans = XMMatrixTranslation(XMVectorGetX(eye), XMVectorGetY(eye), XMVectorGetZ(eye));    // E matrix

    XMVECTOR pickdir = XMVectorSet((float)((2.0f*g_nCurrentMouseX) / g_nWindowWidth) - 1.0f,
        (-1)*((float)((2.0f*g_nCurrentMouseY) / g_nWindowHeight) - 1.0f), 0, 1);             // current mouse ndc
    trans = XMMatrixInverse(nullptr, XMMatrixMultiply(trans, viewproj));                     // (E*VP)^-1
    pickdir = XMVector3Normalize(XMVector4Transform(pickdir, trans));                        // pick direction = cmouse_ndc * (E*VP)^-1  --> normalized
    XMVectorSetW(pickdir, 1.0f);

    pcbCS->pickOriginX = g_nPickOriginX;
    pcbCS->pickOriginY = g_nPickOriginY;
    XMStoreFloat4(&pcbCS->pickDir, pickdir);
    XMStoreFloat4(&pcbCS->eyePos, eye);

    pd3dImmediateContext->Unmap(g_pcbCS, 0);
    ID3D11Buffer* ppCB[1] = { g_pcbCS };
    pd3dImmediateContext->CSSetConstantBuffers(0, 1, ppCB);

    // Run first CS (first volcube)
    pd3dImmediateContext->Dispatch(VCUBEWIDTH, VCUBEWIDTH, VCUBEWIDTH * objectCount);

    // Run second CS (second volcube)
    pd3dImmediateContext->CSSetShader(g_pCompute2CS, nullptr, 0);
    pd3dImmediateContext->Dispatch((VCUBEWIDTH + 1), (VCUBEWIDTH + 1), (VCUBEWIDTH + 1) * objectCount);

    // Unbind resources for CS
    ID3D11ShaderResourceView* srvnull[4] = { nullptr, nullptr, nullptr, nullptr };
    pd3dImmediateContext->CSSetShaderResources(0, 4, srvnull);

    ID3D11UnorderedAccessView* ppUAViewNULL[2] = { nullptr, nullptr };
    pd3dImmediateContext->CSSetUnorderedAccessViews(0, 2, ppUAViewNULL, (UINT*)(&aUAViews));

    // SWAP resources
    std::swap(g_pVolCube1, g_pVolCube1c);
    std::swap(g_pVolCube1RV, g_pVolCube1cRV);
    std::swap(g_pVolCube1UAV, g_pVolCube1cUAV);
    std::swap(g_pVolCube2, g_pVolCube2c);
    std::swap(g_pVolCube2RV, g_pVolCube2cRV);
    std::swap(g_pVolCube2UAV, g_pVolCube2cUAV);

    //--------------------------------------------------------------------------------------
    // EXECUTE SECOND COMPUTE SHADER: UPDATE POSITIONS
    pd3dImmediateContext->CSSetShader(g_pUdateCS, nullptr, 0);

    ID3D11ShaderResourceView* uaRViews[1] = { g_pIndexCubeRV };
    pd3dImmediateContext->CSSetShaderResources(0, 1, uaRViews);
    ID3D11UnorderedAccessView* uaUAViews[3] = { g_pParticleArrayUAV1, g_pVolCube1UAV, g_pVolCube2UAV };
    pd3dImmediateContext->CSSetUnorderedAccessViews(0, 3, uaUAViews, (UINT*)(&uaUAViews));

    //pd3dImmediateContext->Dispatch(g_nNumParticles, 1, 1);
    pd3dImmediateContext->Dispatch(particleCount, 1, 1);

    ID3D11UnorderedAccessView* uppUAViewNULL[3] = { nullptr, nullptr, nullptr };
    pd3dImmediateContext->CSSetUnorderedAccessViews(0, 3, uppUAViewNULL, (UINT*)(&uaUAViews));
    ID3D11ShaderResourceView* uppSRVNULL[1] = { nullptr };
    pd3dImmediateContext->CSSetShaderResources(0, 1, uppSRVNULL);

    // SWAP RESOURCES
    std::swap(g_pParticleArray0, g_pParticleArray1);
    std::swap(g_pParticleArrayRV0, g_pParticleArrayRV1);
    std::swap(g_pParticleArrayUAV0, g_pParticleArrayUAV1);

    // Update the camera's position based on user input 
    g_Camera.FrameMove(fElapsedTime);
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
// Print debug information to console
//--------------------------------------------------------------------------------------
void print_debug_float(float out)
{
    std::wstringstream a; a << "[" << out << "]" << std::endl;
    std::wstring b = a.str();
    OutputDebugString(b.c_str());
}


//--------------------------------------------------------------------------------------
// Handle messages (first: button overrides, second: DXUT camera)
//--------------------------------------------------------------------------------------
LRESULT CALLBACK MsgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam, bool* pbNoFurtherProcessing, void* pUserContext){

    // Picking settings
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
// Handle keypresses
//------------------------------------------------------
void CALLBACK OnKeyboard(UINT nChar, bool bKeyDown, bool bAltDown, void* pUserContext)
{
    XMVECTOR lookat = g_Camera.GetLookAtPt();
    XMVECTOR eye = XMVectorSet(0, 0, 0, 0);
    XMVECTOR x = XMVectorSet(200, 0, 0, 0);
    XMVECTOR y = XMVectorSet(0, 200, 0, 0);
    XMVECTOR v;
    switch (nChar){
        case VK_LEFT:
            v = XMVectorSubtract(g_Camera.GetEyePt(), x);
            XMStoreFloat4(reinterpret_cast<XMFLOAT4*>(&eye), v);
            g_Camera.SetViewParams(eye, lookat);
            break;
        case VK_RIGHT:
            v = XMVectorAdd(g_Camera.GetEyePt(), x);
            XMStoreFloat4(reinterpret_cast<XMFLOAT4*>(&eye), v);
            g_Camera.SetViewParams(eye, lookat);
            break;
        case VK_UP:
            v = XMVectorAdd(g_Camera.GetEyePt(), y);
            XMStoreFloat4(reinterpret_cast<XMFLOAT4*>(&eye), v);
            g_Camera.SetViewParams(eye, lookat);
            break;
        case VK_DOWN:
            v = XMVectorSubtract(g_Camera.GetEyePt(), y);
            XMStoreFloat4(reinterpret_cast<XMFLOAT4*>(&eye), v);
            g_Camera.SetViewParams(eye, lookat);
            break;
        case 0x58:    // 'X' key
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
            initBuffers(DXUTGetD3D11Device());
            break;
        }
        default: break;
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
// Init shaders, set vertex buffer
//--------------------------------------------------------------------------------------
HRESULT importFiles(){

    // set up first bunny: create Deformable, build representation, add to central container
    Deformable body1("bunny_res3_scaled.obj", 0);
    body1.build();
    deformableObjects.push_back(body1);

    //set up second bunny
    Deformable body2("bunny_res3_scaled.obj", 1);
    body2.build();
    body2.translate(1000, 0, 0);
    deformableObjects.push_back(body2);

    // Set variables
    objectCount = deformableObjects.size();
    particleCount = mass1Count = mass2Count = 0;
    for (unsigned int i = 0; i < deformableObjects.size(); i++){
        particleCount += deformableObjects[i].particles.size();
        mass1Count += deformableObjects[i].masscube1.size();
        mass2Count += deformableObjects[i].masscube2.size();
    }
    cubeCellSize = deformableObjects[0].cubeCellSize;

    print_debug_float(mass1Count);
    print_debug_float(mass2Count);
    print_debug_float(objectCount);

    return S_OK;
}

//--------------------------------------------------------------------------------------
// Init shaders, set vertex buffer
//--------------------------------------------------------------------------------------
HRESULT initShaders(ID3D11Device* pd3dDevice, ID3D11DeviceContext* pd3dImmediateContext){

    HRESULT hr;
    ID3DBlob* pBlobRenderParticlesVS = nullptr;
    ID3DBlob* pBlobRenderParticlesGS = nullptr;
    ID3DBlob* pBlobRenderParticlesPS = nullptr;
    ID3DBlob* pBlobModelPS1 = nullptr;
    ID3DBlob* pBlobModelPS2 = nullptr;
    ID3DBlob* pBlobCalc1CS = nullptr;
    ID3DBlob* pBlobCalc2CS = nullptr;
    ID3DBlob* pBlobUpdateCS = nullptr;

    // Compile shaders
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

    // No vertex buffer necessary, particle data is read from an SRV
    pd3dImmediateContext->IASetInputLayout(nullptr);
    ID3D11Buffer* pBuffers[1] = { nullptr };
    UINT tmp[1] = { 0 };
    pd3dImmediateContext->IASetVertexBuffers(0, 1, pBuffers, tmp, tmp);
    pd3dImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST);

    // Release blobs
    SAFE_RELEASE(pBlobRenderParticlesVS);
    SAFE_RELEASE(pBlobRenderParticlesGS);
    SAFE_RELEASE(pBlobRenderParticlesPS);
    SAFE_RELEASE(pBlobModelPS1);
    SAFE_RELEASE(pBlobModelPS2);
    SAFE_RELEASE(pBlobCalc1CS);
    SAFE_RELEASE(pBlobCalc2CS);
    SAFE_RELEASE(pBlobUpdateCS);

    return S_OK;
}

//--------------------------------------------------------------------------------------
// Create buffer structures with initial data
//--------------------------------------------------------------------------------------
HRESULT initBuffers(ID3D11Device* pd3dDevice)
{

    HRESULT hr = S_OK;

    // Load particle and masscube data in temporal arrays
    PARTICLE* pData1 = new PARTICLE[particleCount];
    INDEXER* iData1 = new INDEXER[particleCount];
    MASSPOINT* vData1 = new MASSPOINT[mass1Count];
    MASSPOINT* vData2 = new MASSPOINT[mass2Count];
    if (!pData1 || !iData1 || !vData1 || !vData2)
        return E_OUTOFMEMORY;

    uint x = 0;
    for (uint i = 0; i < objectCount; i++){
        for (uint k = 0; k < deformableObjects[i].particles.size(); k++){
            pData1[x] = deformableObjects[i].particles[k];
            x++;
        }
    }
    x = 0;
    uint size = sizeof(float[8]);
    for (uint i = 0; i < objectCount; i++){
        for (uint k = 0; k < deformableObjects[i].indexcube.size(); k++){
            iData1[x].vc1index = deformableObjects[i].indexcube[k].vc1index;
            iData1[x].vc2index = deformableObjects[i].indexcube[k].vc2index;
            memcpy(iData1[x].w1, deformableObjects[i].indexcube[k].w1, size);
            memcpy(iData1[x].nw1, deformableObjects[i].indexcube[k].nw1, size);
            memcpy(iData1[x].w2, deformableObjects[i].indexcube[k].w2, size);
            memcpy(iData1[x].nw2, deformableObjects[i].indexcube[k].nw2, size);
            x++;
        }
    }
    x = 0;
    for (uint i = 0; i < objectCount; i++){
        for (uint k = 0; k < deformableObjects[i].masscube1.size(); k++){
            vData1[x] = deformableObjects[i].masscube1[k];
            x++;
        }
    }
    x = 0;
    for (uint i = 0; i < objectCount; i++){
        for (uint k = 0; k < deformableObjects[i].masscube2.size(); k++){
            vData2[x] = deformableObjects[i].masscube2[k];
            x++;
        }
    }

    // Desc for Particle buffers
    D3D11_BUFFER_DESC desc;
    ZeroMemory(&desc, sizeof(desc));
    desc.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;
    desc.ByteWidth = particleCount * sizeof(PARTICLE);
    desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
    desc.StructureByteStride = sizeof(PARTICLE);
    desc.Usage = D3D11_USAGE_DEFAULT;

    // Desc for Volumetric buffer
    D3D11_BUFFER_DESC vdesc;
    ZeroMemory(&vdesc, sizeof(vdesc));
    vdesc.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;
    vdesc.ByteWidth = mass1Count * sizeof(MASSPOINT);
    vdesc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
    vdesc.StructureByteStride = sizeof(MASSPOINT);
    vdesc.Usage = D3D11_USAGE_DEFAULT;

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
    vdesc.ByteWidth = mass2Count * sizeof(MASSPOINT);
    V_RETURN(pd3dDevice->CreateBuffer(&vdesc, &v2data, &g_pVolCube2));
    V_RETURN(pd3dDevice->CreateBuffer(&vdesc, &v2data, &g_pVolCube2c));
    DXUT_SetDebugName(g_pVolCube2, "VolCube2");
    DXUT_SetDebugName(g_pVolCube2c, "VolCube2c");
    SAFE_DELETE_ARRAY(vData2);


    /// Buffer for IndexCube
    D3D11_BUFFER_DESC desc2;
    ZeroMemory(&desc2, sizeof(desc2));
    desc2.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;
    desc2.ByteWidth = particleCount * sizeof(INDEXER);
    desc2.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
    desc2.StructureByteStride = sizeof(INDEXER);
    desc2.Usage = D3D11_USAGE_DEFAULT;
    D3D11_SUBRESOURCE_DATA indexer_init;
    indexer_init.pSysMem = iData1;
    V_RETURN(pd3dDevice->CreateBuffer(&desc2, &indexer_init, &g_pIndexCube));
    DXUT_SetDebugName(g_pIndexCube, "IndexCube");
    SAFE_DELETE_ARRAY(iData1);

    /// RV for Particle data
    D3D11_SHADER_RESOURCE_VIEW_DESC DescRV;
    ZeroMemory(&DescRV, sizeof(DescRV));
    DescRV.Format = DXGI_FORMAT_UNKNOWN;
    DescRV.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
    DescRV.Buffer.FirstElement = 0;
    DescRV.Buffer.NumElements = particleCount;
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
    DescRV2.Buffer.NumElements = particleCount;
    V_RETURN(pd3dDevice->CreateShaderResourceView(g_pIndexCube, &DescRV2, &g_pIndexCubeRV));
    DXUT_SetDebugName(g_pIndexCubeRV, "IndexCube SRV");

    /// SRV for VolCubes
    D3D11_SHADER_RESOURCE_VIEW_DESC DescRVV;
    ZeroMemory(&DescRVV, sizeof(DescRVV));
    DescRVV.Format = DXGI_FORMAT_UNKNOWN;
    DescRVV.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
    DescRVV.Buffer.FirstElement = 0;
    DescRVV.Buffer.NumElements = mass1Count;
    V_RETURN(pd3dDevice->CreateShaderResourceView(g_pVolCube1, &DescRVV, &g_pVolCube1RV));
    V_RETURN(pd3dDevice->CreateShaderResourceView(g_pVolCube1c, &DescRVV, &g_pVolCube1cRV));
    DXUT_SetDebugName(g_pVolCube1RV, "VolCube1 RV");
    DXUT_SetDebugName(g_pVolCube1cRV, "VolCube1c RV");
    DescRVV.Buffer.NumElements = mass2Count;
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
    DescUAV.Buffer.NumElements = particleCount;
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
    vDescUAV.Buffer.NumElements = mass1Count;
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(g_pVolCube1, &vDescUAV, &g_pVolCube1UAV));
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(g_pVolCube1c, &vDescUAV, &g_pVolCube1cUAV));
    DXUT_SetDebugName(g_pVolCube1UAV, "VolCube1 UAV");
    DXUT_SetDebugName(g_pVolCube1cUAV, "VolCube1c UAV");
    vDescUAV.Buffer.NumElements = mass2Count;
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(g_pVolCube2, &vDescUAV, &g_pVolCube2UAV));
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(g_pVolCube2c, &vDescUAV, &g_pVolCube2cUAV));
    DXUT_SetDebugName(g_pVolCube2UAV, "VolCube2 UAV");
    DXUT_SetDebugName(g_pVolCube2cUAV, "VolCube2c UAV");

    return hr;
}

//--------------------------------------------------------------------------------------
// Init depthstencil, blend, constant buffers, texture
//--------------------------------------------------------------------------------------
HRESULT initRender(ID3D11Device* pd3dDevice){

    HRESULT hr;

    // Setup constant buffers
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

    // Load Particle Texture
    V_RETURN(DXUTCreateShaderResourceViewFromFile(pd3dDevice, L"misc\\Particle.dds", &g_pParticleTexRV));
    DXUT_SetDebugName(g_pParticleTexRV, "Particle.dds");

    // Create sampler
    D3D11_SAMPLER_DESC SamplerDesc;
    ZeroMemory(&SamplerDesc, sizeof(SamplerDesc));
    SamplerDesc.AddressU = D3D11_TEXTURE_ADDRESS_CLAMP;
    SamplerDesc.AddressV = D3D11_TEXTURE_ADDRESS_CLAMP;
    SamplerDesc.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
    SamplerDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
    V_RETURN(pd3dDevice->CreateSamplerState(&SamplerDesc, &g_pSampleStateLinear));
    DXUT_SetDebugName(g_pSampleStateLinear, "Linear");

    // Create blend
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

    // Create depth stencil
    D3D11_DEPTH_STENCIL_DESC DepthStencilDesc;
    ZeroMemory(&DepthStencilDesc, sizeof(DepthStencilDesc));
    DepthStencilDesc.DepthEnable = FALSE;
    DepthStencilDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
    pd3dDevice->CreateDepthStencilState(&DepthStencilDesc, &g_pDepthStencilState);
    DXUT_SetDebugName(g_pDepthStencilState, "DepthOff");

    return S_OK;
}

//--------------------------------------------------------------------------------------
// (Re)Create texture and buffers for model render target (for picking)
//--------------------------------------------------------------------------------------
HRESULT initPicking(ID3D11Device* pd3dDevice, int width, int height)
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

    return S_OK;
}

//--------------------------------------------------------------------------------------
// Create any D3D11 resources that aren't dependant on the back buffer
//--------------------------------------------------------------------------------------
HRESULT CALLBACK OnD3D11CreateDevice(ID3D11Device* pd3dDevice, const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc, void* pUserContext){

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

    CWaitDlg setUpDialog;
    setUpDialog.ShowDialog(L"Setting up environment...");

    bFirstOnCreateDevice = false;

    D3D11_FEATURE_DATA_D3D10_X_HARDWARE_OPTIONS ho;
    V_RETURN(pd3dDevice->CheckFeatureSupport(D3D11_FEATURE_D3D10_X_HARDWARE_OPTIONS, &ho, sizeof(ho)));

    auto pd3dImmediateContext = DXUTGetD3D11DeviceContext();

    // Create and compile shaders, init vertex buffer
    V_RETURN(initShaders(pd3dDevice, pd3dImmediateContext));

    // Import objects from files, set up deformable objects
    V_RETURN(importFiles());

    // Initialize buffers with data
    V_RETURN(initBuffers(pd3dDevice));

    // Create textures&buffers for picking
    V_RETURN(initPicking(pd3dDevice, 800, 600));

    // Init depth stencil, blend, texture, constant buffers
    V_RETURN(initRender(pd3dDevice));

    // Setup the camera's view parameters
    XMVECTOR vecEye = XMVectorSet(-g_fSpread * 0, g_fSpread * 0, -g_fSpread * 27, 0.0f);
    XMVECTOR vecAt = XMVectorSet(0.0f, 0.0f, 0.0f, 0.0f);
    g_Camera.SetViewParams(vecEye, vecAt);

    setUpDialog.DestroyDialog();

    return S_OK;
}

//--------------------------------------------------------------------------------------
// On screen resize
//--------------------------------------------------------------------------------------
HRESULT CALLBACK OnD3D11ResizedSwapChain(ID3D11Device* pd3dDevice, IDXGISwapChain* pSwapChain, const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc, void* pUserContext){

    HRESULT hr = S_OK;

    // Setup the camera's projection parameters
    g_nWindowWidth = pBackBufferSurfaceDesc->Width;
    g_nWindowHeight = pBackBufferSurfaceDesc->Height;
    float fAspectRatio = pBackBufferSurfaceDesc->Width / (FLOAT)pBackBufferSurfaceDesc->Height;
    g_Camera.SetProjParams(XM_PI / 4, fAspectRatio, 10.0f, 500000.0f);
    g_Camera.SetWindow(pBackBufferSurfaceDesc->Width, pBackBufferSurfaceDesc->Height);
    g_Camera.SetButtonMasks(0, MOUSE_WHEEL, MOUSE_LEFT_BUTTON | MOUSE_MIDDLE_BUTTON);

    initPicking(pd3dDevice, pBackBufferSurfaceDesc->Width, pBackBufferSurfaceDesc->Height);

    return hr;
}

//--------------------------------------------------------------------------------------
// Render model to texture, for picking / dragging
//--------------------------------------------------------------------------------------
bool drawPicking(ID3D11DeviceContext* pd3dImmediateContext, CXMMATRIX mView, CXMMATRIX mProj)
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

    ID3D11ShaderResourceView* aRViews[1] = { g_pParticleArrayRV0 };
    pd3dImmediateContext->VSSetShaderResources(0, 1, aRViews);

    D3D11_MAPPED_SUBRESOURCE MappedResource;
    pd3dImmediateContext->Map(g_pcbGS, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource);
    auto pCBGS = reinterpret_cast<CB_GS*>(MappedResource.pData);
    XMStoreFloat4x4(&pCBGS->m_WorldViewProj, XMMatrixMultiply(mView, mProj));
    XMStoreFloat4x4(&pCBGS->m_InvView, XMMatrixInverse(nullptr, mView));
    XMStoreFloat4(&pCBGS->eyepos, g_Camera.GetEyePt());
    pCBGS->lightpos = g_vLightPosition;
    pd3dImmediateContext->Unmap(g_pcbGS, 0);
    pd3dImmediateContext->GSSetConstantBuffers(0, 1, &g_pcbGS);

    pd3dImmediateContext->PSSetShaderResources(0, 1, &g_pParticleTexRV);
    pd3dImmediateContext->PSSetSamplers(0, 1, &g_pSampleStateLinear);

    pd3dImmediateContext->OMSetBlendState(nullptr, nullptr, 0xFFFFFFFF);
    pd3dImmediateContext->OMSetDepthStencilState(g_pDepthStencilState, 0);


    // Render first "layer" of information: vertex IDs in first volcube
    pd3dImmediateContext->Draw(particleCount, 0);

    // Render second layer of information: vertex IDs in second volcube
    pd3dImmediateContext->ClearDepthStencilView(pDSV, D3D11_CLEAR_DEPTH, 1.0, 0);
    pd3dImmediateContext->OMSetRenderTargets(1, &g_pModelRTV[1], pDSV);
    pd3dImmediateContext->ClearRenderTargetView(g_pModelRTV[1], Colors::Black);
    pd3dImmediateContext->PSSetShader(g_pModelPS2, nullptr, 0);
    pd3dImmediateContext->Draw(particleCount, 0);


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
// Render objects to display
//--------------------------------------------------------------------------------------
bool drawObjects(ID3D11DeviceContext* pd3dImmediateContext, CXMMATRIX mView, CXMMATRIX mProj)
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

    ID3D11ShaderResourceView* aRViews[1] = { g_pParticleArrayRV0 };
    pd3dImmediateContext->VSSetShaderResources(0, 1, aRViews);

    D3D11_MAPPED_SUBRESOURCE MappedResource;
    pd3dImmediateContext->Map(g_pcbGS, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource);
    auto pCBGS = reinterpret_cast<CB_GS*>(MappedResource.pData);
    XMStoreFloat4x4(&pCBGS->m_WorldViewProj, XMMatrixMultiply(mView, mProj));
    XMStoreFloat4x4(&pCBGS->m_InvView, XMMatrixInverse(nullptr, mView));
    XMStoreFloat4(&pCBGS->eyepos, g_Camera.GetEyePt());
    pCBGS->lightpos = g_vLightPosition;
    pd3dImmediateContext->Unmap(g_pcbGS, 0);
    pd3dImmediateContext->GSSetConstantBuffers(0, 1, &g_pcbGS);

    pd3dImmediateContext->PSSetShaderResources(0, 1, &g_pParticleTexRV);
    pd3dImmediateContext->PSSetSamplers(0, 1, &g_pSampleStateLinear);

    float bf[] = { 0.f, 0.f, 0.f, 0.f };
    pd3dImmediateContext->OMSetBlendState(g_pBlendingStateParticle, bf, 0xFFFFFFFF);
    pd3dImmediateContext->OMSetDepthStencilState(g_pDepthStencilState, 0);

    pd3dImmediateContext->Draw(particleCount, 0);

    ID3D11ShaderResourceView* ppSRVNULL[1] = { nullptr };
    pd3dImmediateContext->VSSetShaderResources(0, 1, ppSRVNULL);
    pd3dImmediateContext->PSSetShaderResources(0, 1, ppSRVNULL);

    pd3dImmediateContext->GSSetShader(nullptr, nullptr, 0);
    pd3dImmediateContext->OMSetBlendState(pBlendState0, &BlendFactor0.x, SampleMask0); SAFE_RELEASE(pBlendState0);
    pd3dImmediateContext->OMSetDepthStencilState(pDepthStencilState0, StencilRef0); SAFE_RELEASE(pDepthStencilState0);

    return true;
}

//--------------------------------------------------------------------------------------
// Render next frame
//--------------------------------------------------------------------------------------
void CALLBACK OnD3D11FrameRender(ID3D11Device* pd3dDevice, ID3D11DeviceContext* pd3dImmediateContext, double fTime, float fElapsedTime, void* pUserContext){

    ID3D11RenderTargetView* pRTV = DXUTGetD3D11RenderTargetView();
    pd3dImmediateContext->ClearRenderTargetView(pRTV, Colors::Black);
    ID3D11DepthStencilView* pDSV = DXUTGetD3D11DepthStencilView();
    pd3dImmediateContext->ClearDepthStencilView(pDSV, D3D11_CLEAR_DEPTH, 1.0, 0);

    XMMATRIX mView = g_Camera.GetViewMatrix();
    XMMATRIX mProj = g_Camera.GetProjMatrix();

    // Render the model first for picking, only if RMBUTTONDOWN happened
    if (g_bRM2Texture){
        drawPicking(pd3dImmediateContext, mView, mProj);
    }

    // Render the particles
    drawObjects(pd3dImmediateContext, mView, mProj);

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
    DXUTGetGlobalResourceCache().OnDestroyDevice();
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