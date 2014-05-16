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
#include <array>
#include <time.h>
#include "Deformable.h"
#include "Constants.h"
#include "Collision.h"


using namespace DirectX;

//--------------------------------------------------------------------------------------
// Global variables
//--------------------------------------------------------------------------------------

// rendering
CModelViewerCamera                  camera;
ID3D11BlendState*                   blendState = nullptr;
ID3D11DepthStencilState*            depthStencilState = nullptr;
ID3D11SamplerState*                 samplerState = nullptr;
ID3D11ShaderResourceView*           particleTextureSRV = nullptr;

// storage & access
ID3D11Buffer*                       bvhCatalogueBuffer1 = nullptr;
ID3D11Buffer*                       bvhCatalogueBuffer2 = nullptr;
ID3D11Buffer*                       bvhDataBuffer1 = nullptr;
ID3D11Buffer*                       bvhDataBuffer2 = nullptr;
ID3D11Buffer*                       csConstantBuffer = nullptr;
ID3D11Buffer*                       gsConstantBuffer = nullptr;
ID3D11Buffer*                       indexerBuffer = nullptr;
ID3D11Buffer*                       masscube1Buffer1 = nullptr;
ID3D11Buffer*                       masscube1Buffer2 = nullptr;
ID3D11Buffer*                       masscube2Buffer1 = nullptr;
ID3D11Buffer*                       masscube2Buffer2 = nullptr;
ID3D11Buffer*                       particleBuffer1 = nullptr;
ID3D11Buffer*                       particleBuffer2 = nullptr;
ID3D11RenderTargetView*             pickingRTV1 = nullptr;
ID3D11RenderTargetView*             pickingRTV2 = nullptr;
ID3D11ShaderResourceView*           bvhCatalogueSRV1 = nullptr;
ID3D11ShaderResourceView*           bvhCatalogueSRV2 = nullptr;
ID3D11ShaderResourceView*           bvhDataSRV1 = nullptr;
ID3D11ShaderResourceView*           bvhDataSRV2 = nullptr;
ID3D11ShaderResourceView*           indexerSRV = nullptr;
ID3D11ShaderResourceView*           masscube1SRV1 = nullptr;
ID3D11ShaderResourceView*           masscube1SRV2 = nullptr;
ID3D11ShaderResourceView*           masscube2SRV1 = nullptr;
ID3D11ShaderResourceView*           masscube2SRV2 = nullptr;
ID3D11ShaderResourceView*           particleSRV1 = nullptr;
ID3D11ShaderResourceView*           particleSRV2 = nullptr;
ID3D11ShaderResourceView*           pickingSRV1 = nullptr;
ID3D11ShaderResourceView*           pickingSRV2 = nullptr;
ID3D11Texture2D*                    pickingTexture1 = nullptr;
ID3D11Texture2D*                    pickingTexture2 = nullptr;
ID3D11UnorderedAccessView*          bvhCatalogueUAV1 = nullptr;
ID3D11UnorderedAccessView*          bvhCatalogueUAV2 = nullptr;
ID3D11UnorderedAccessView*          bvhDataUAV1 = nullptr;
ID3D11UnorderedAccessView*          bvhDataUAV2 = nullptr;
ID3D11UnorderedAccessView*          masscube1UAV1 = nullptr;
ID3D11UnorderedAccessView*          masscube1UAV2 = nullptr;
ID3D11UnorderedAccessView*          masscube2UAV1 = nullptr;
ID3D11UnorderedAccessView*          masscube2UAV2 = nullptr;
ID3D11UnorderedAccessView*          particleUAV1 = nullptr;
ID3D11UnorderedAccessView*          particleUAV2 = nullptr;

// shaders
ID3D11ComputeShader*                physicsCS1 = nullptr;
ID3D11ComputeShader*                physicsCS2 = nullptr;
ID3D11ComputeShader*                updateCS = nullptr;
ID3D11GeometryShader*               renderGS = nullptr;
ID3D11PixelShader*                  renderPS = nullptr;
ID3D11PixelShader*                  pickingPS1 = nullptr;
ID3D11PixelShader*                  pickingPS2 = nullptr;
ID3D11VertexShader*                 renderVS = nullptr;

// Scene variables
std::vector<Deformable>             deformableObjects;				// scene objects
uint                                objectCount;					// # of deformable bodies
uint                                particleCount;					// # of total vertex count
uint                                mass1Count;						// #s of total masspoints
uint                                mass2Count;
uint                                cubeCellSize;					// cell size in masscubes

// Window & picking variables
int                                 windowWidth = 800;
int                                 windowHeight = 600;
int                                 mouseClickX;
int                                 mouseClickY;
int                                 pickOriginX;
int                                 pickOriginY;
bool                                isPicking = false;				// RBUTTON is pressed
bool                                renderPicking = false;			// render model to texture for picking
XMFLOAT4                            lightPos(0, 0, -10000, 1);		// light position

// Debug file
std::ofstream                       debug;


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
void print_debug_file(const char*);



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
    pd3dImmediateContext->CSSetShader(physicsCS1, nullptr, 0);

    ID3D11ShaderResourceView* srvs[4] = { masscube1SRV2, masscube2SRV2, pickingSRV1, pickingSRV2 };
    pd3dImmediateContext->CSSetShaderResources(0, 4, srvs);

    ID3D11UnorderedAccessView* aUAViews[2] = { masscube1UAV1, masscube2UAV1 };
    pd3dImmediateContext->CSSetUnorderedAccessViews(0, 2, aUAViews, (UINT*)(&aUAViews));

    // For CS constant buffer
    D3D11_MAPPED_SUBRESOURCE MappedResource;
    V(pd3dImmediateContext->Map(csConstantBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource));
    auto pcbCS = reinterpret_cast<CB_CS*>(MappedResource.pData);

    // Update CS constant buffer
    pcbCS->cubeWidth = VCUBEWIDTH;
    pcbCS->cubeCellSize = cubeCellSize;
    pcbCS->objectCount = objectCount;
    pcbCS->stiffness = stiffnessConstant;
    pcbCS->damping = dampingConstant;
    pcbCS->dt = fElapsedTime;
    pcbCS->im = invMassConstant;
    pcbCS->gravity = gravityConstant;
    pcbCS->tablePos = tablePositionConstant;

    // Send picking data to GPU
    if (isPicking)
        pcbCS->isPicking = 1;
    else
        pcbCS->isPicking = 0;

    XMMATRIX view = camera.GetViewMatrix();    // V matrix
    XMMATRIX proj = camera.GetProjMatrix();    // P matrix
    XMVECTOR eye = camera.GetEyePt();          // eye pos

    XMMATRIX viewproj = XMMatrixMultiply(view, proj);    // VP matrix
    XMMATRIX trans = XMMatrixTranslation(XMVectorGetX(eye), XMVectorGetY(eye), XMVectorGetZ(eye));    // E matrix

    XMVECTOR pickdir = XMVectorSet((float)((2.0f*mouseClickX) / windowWidth) - 1.0f,
        (-1)*((float)((2.0f*mouseClickY) / windowHeight) - 1.0f), 0, 1);             // current mouse ndc
    trans = XMMatrixInverse(nullptr, XMMatrixMultiply(trans, viewproj));                     // (E*VP)^-1
    pickdir = XMVector3Normalize(XMVector4Transform(pickdir, trans));                        // pick direction = cmouse_ndc * (E*VP)^-1  --> normalized
    XMVectorSetW(pickdir, 1.0f);

    pcbCS->pickOriginX = pickOriginX;
    pcbCS->pickOriginY = pickOriginY;
    XMStoreFloat4(&pcbCS->pickDir, pickdir);
    XMStoreFloat4(&pcbCS->eyePos, eye);

    pd3dImmediateContext->Unmap(csConstantBuffer, 0);
    ID3D11Buffer* ppCB[1] = { csConstantBuffer };
    pd3dImmediateContext->CSSetConstantBuffers(0, 1, ppCB);

    // Run first CS (first volcube)
    pd3dImmediateContext->Dispatch(VCUBEWIDTH, VCUBEWIDTH, VCUBEWIDTH * objectCount);

    // Run second CS (second volcube)
    pd3dImmediateContext->CSSetShader(physicsCS2, nullptr, 0);
    pd3dImmediateContext->Dispatch((VCUBEWIDTH + 1), (VCUBEWIDTH + 1), (VCUBEWIDTH + 1) * objectCount);

    // Unbind resources for CS
    ID3D11ShaderResourceView* srvnull[4] = { nullptr, nullptr, nullptr, nullptr };
    pd3dImmediateContext->CSSetShaderResources(0, 4, srvnull);

    ID3D11UnorderedAccessView* ppUAViewNULL[2] = { nullptr, nullptr };
    pd3dImmediateContext->CSSetUnorderedAccessViews(0, 2, ppUAViewNULL, (UINT*)(&aUAViews));

    // SWAP resources
    std::swap(masscube1Buffer1, masscube1Buffer2);
    std::swap(masscube1SRV1, masscube1SRV2);
    std::swap(masscube1UAV1, masscube1UAV2);
    std::swap(masscube2Buffer1, masscube2Buffer2);
    std::swap(masscube2SRV1, masscube2SRV2);
    std::swap(masscube2UAV1, masscube2UAV2);
    //--------------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------------
    // EXECUTE SECOND COMPUTE SHADER: UPDATE POSITIONS
    pd3dImmediateContext->CSSetShader(updateCS, nullptr, 0);

    ID3D11ShaderResourceView* uaRViews[1] = { indexerSRV };
    pd3dImmediateContext->CSSetShaderResources(0, 1, uaRViews);
    ID3D11UnorderedAccessView* uaUAViews[3] = { particleUAV2, masscube1UAV1, masscube2UAV1 };
    pd3dImmediateContext->CSSetUnorderedAccessViews(0, 3, uaUAViews, (UINT*)(&uaUAViews));

    //pd3dImmediateContext->Dispatch(g_nNumParticles, 1, 1);
    pd3dImmediateContext->Dispatch(particleCount, 1, 1);

    ID3D11UnorderedAccessView* uppUAViewNULL[3] = { nullptr, nullptr, nullptr };
    pd3dImmediateContext->CSSetUnorderedAccessViews(0, 3, uppUAViewNULL, (UINT*)(&uaUAViews));
    ID3D11ShaderResourceView* uppSRVNULL[1] = { nullptr };
    pd3dImmediateContext->CSSetShaderResources(0, 1, uppSRVNULL);

    // SWAP RESOURCES
    std::swap(particleBuffer1, particleBuffer2);
    std::swap(particleSRV1, particleSRV2);
    std::swap(particleUAV1, particleUAV2);

    // Update the camera's position based on user input 
    camera.FrameMove(fElapsedTime);
}

//--------------------------------------------------------------------------------------
// Print debug information to file
//--------------------------------------------------------------------------------------
void print_debug_file(const char* string)
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
// Print debug information to console
//--------------------------------------------------------------------------------------
void print_debug_string(std::string out)
{
    std::wstringstream a; a << "[" << out.c_str() << "]" << std::endl;
    std::wstring b = a.str();
    OutputDebugString(b.c_str());
}

//--------------------------------------------------------------------------------------
// Handle messages (first: button overrides, second: DXUT camera)
//--------------------------------------------------------------------------------------
LRESULT CALLBACK MsgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam, bool* pbNoFurtherProcessing, void* pUserContext){

    // Picking settings
    if (uMsg == WM_RBUTTONDOWN){
        isPicking = true;
        renderPicking = true;
        pickOriginX = (short)LOWORD(lParam);
        pickOriginY = (short)HIWORD(lParam);
        mouseClickX = (short)LOWORD(lParam);
        mouseClickY = (short)HIWORD(lParam);
    }
    else if (uMsg == WM_MOUSEMOVE){
        mouseClickX = (short)LOWORD(lParam);
        mouseClickY = (short)HIWORD(lParam);
    }
    else if (uMsg == WM_RBUTTONUP){
        isPicking = false;
    }

    // Pass all windows messages to camera so it can respond to user input
    camera.HandleMessages(hWnd, uMsg, wParam, lParam);

    return 0;
}

//------------------------------------------------------
// Handle keypresses
//------------------------------------------------------
void CALLBACK OnKeyboard(UINT nChar, bool bKeyDown, bool bAltDown, void* pUserContext)
{
    XMVECTOR lookat = camera.GetLookAtPt();
    XMVECTOR eye = XMVectorSet(0, 0, 0, 0);
    XMVECTOR x = XMVectorSet(200, 0, 0, 0);
    XMVECTOR y = XMVectorSet(0, 200, 0, 0);
    XMVECTOR v;
    switch (nChar){
        case VK_LEFT:
            v = XMVectorSubtract(camera.GetEyePt(), x);
            XMStoreFloat4(reinterpret_cast<XMFLOAT4*>(&eye), v);
            camera.SetViewParams(eye, lookat);
            break;
        case VK_RIGHT:
            v = XMVectorAdd(camera.GetEyePt(), x);
            XMStoreFloat4(reinterpret_cast<XMFLOAT4*>(&eye), v);
            camera.SetViewParams(eye, lookat);
            break;
        case VK_UP:
            v = XMVectorAdd(camera.GetEyePt(), y);
            XMStoreFloat4(reinterpret_cast<XMFLOAT4*>(&eye), v);
            camera.SetViewParams(eye, lookat);
            break;
        case VK_DOWN:
            v = XMVectorSubtract(camera.GetEyePt(), y);
            XMStoreFloat4(reinterpret_cast<XMFLOAT4*>(&eye), v);
            camera.SetViewParams(eye, lookat);
            break;
        case 0x58:    // 'X' key
        {
            SAFE_RELEASE(bvhCatalogueBuffer1);
            SAFE_RELEASE(bvhCatalogueBuffer2);
            SAFE_RELEASE(bvhDataBuffer1);
            SAFE_RELEASE(bvhDataBuffer2);
            SAFE_RELEASE(indexerBuffer);
            SAFE_RELEASE(masscube1Buffer1);
            SAFE_RELEASE(masscube1Buffer2);
            SAFE_RELEASE(masscube2Buffer1);
            SAFE_RELEASE(masscube2Buffer2);
            SAFE_RELEASE(particleBuffer1);
            SAFE_RELEASE(particleBuffer2);
            SAFE_RELEASE(bvhCatalogueSRV1);
            SAFE_RELEASE(bvhCatalogueSRV2);
            SAFE_RELEASE(bvhDataSRV1);
            SAFE_RELEASE(bvhDataSRV2);
            SAFE_RELEASE(indexerSRV);
            SAFE_RELEASE(masscube1SRV1);
            SAFE_RELEASE(masscube1SRV2);
            SAFE_RELEASE(masscube2SRV1);
            SAFE_RELEASE(masscube2SRV2);
            SAFE_RELEASE(particleSRV1);
            SAFE_RELEASE(particleSRV2);
            SAFE_RELEASE(bvhCatalogueUAV1);
            SAFE_RELEASE(bvhCatalogueUAV2);
            SAFE_RELEASE(bvhDataUAV1);
            SAFE_RELEASE(bvhDataUAV2);
            SAFE_RELEASE(masscube1UAV1);
            SAFE_RELEASE(masscube1UAV2);
            SAFE_RELEASE(masscube2UAV1);
            SAFE_RELEASE(masscube2UAV2);
            SAFE_RELEASE(particleUAV1);
            SAFE_RELEASE(particleUAV2);
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
// Loading a model: 1.) Deformable instance("file");
//                  2.) instance.build();
//                 [3.) instance.translate(...);]
//                  4.) instance.initCollisionDetection();
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

    //set up third bunny
    //Deformable body3("bunny_res3_scaled.obj", 2);
    //body3.build();
    //body3.translate(0, 2000, 0);
    //deformableObjects.push_back(body3);

    // Set variables
    objectCount = deformableObjects.size();
    particleCount = mass1Count = mass2Count = 0;

    for (uint i = 0; i < deformableObjects.size(); i++){
        particleCount += deformableObjects[i].particles.size();
        mass1Count += deformableObjects[i].masscube1.size();
        mass2Count += deformableObjects[i].masscube2.size();
    }
    cubeCellSize = deformableObjects[0].cubeCellSize;

    // Initialize collision detection structures -- AFTER TRANSLATING THE MODELS
    for (uint i = 0; i < objectCount; i++){
        deformableObjects[i].initCollisionDetection();
    }

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


    V_RETURN(pd3dDevice->CreateVertexShader(pBlobRenderParticlesVS->GetBufferPointer(), pBlobRenderParticlesVS->GetBufferSize(), nullptr, &renderVS));
    DXUT_SetDebugName(renderVS, "VSParticleDraw");

    V_RETURN(pd3dDevice->CreateGeometryShader(pBlobRenderParticlesGS->GetBufferPointer(), pBlobRenderParticlesGS->GetBufferSize(), nullptr, &renderGS));
    DXUT_SetDebugName(renderGS, "GSParticleDraw");

    V_RETURN(pd3dDevice->CreatePixelShader(pBlobRenderParticlesPS->GetBufferPointer(), pBlobRenderParticlesPS->GetBufferSize(), nullptr, &renderPS));
    DXUT_SetDebugName(renderPS, "PSParticleDraw");

    V_RETURN(pd3dDevice->CreatePixelShader(pBlobModelPS1->GetBufferPointer(), pBlobModelPS1->GetBufferSize(), nullptr, &pickingPS1));
    DXUT_SetDebugName(pickingPS1, "PSModelDraw1");

    V_RETURN(pd3dDevice->CreatePixelShader(pBlobModelPS2->GetBufferPointer(), pBlobModelPS2->GetBufferSize(), nullptr, &pickingPS2));
    DXUT_SetDebugName(pickingPS2, "PSModelDraw2");

    V_RETURN(pd3dDevice->CreateComputeShader(pBlobCalc1CS->GetBufferPointer(), pBlobCalc1CS->GetBufferSize(), nullptr, &physicsCS1));
    DXUT_SetDebugName(physicsCS1, "CSMain1");

    V_RETURN(pd3dDevice->CreateComputeShader(pBlobCalc2CS->GetBufferPointer(), pBlobCalc2CS->GetBufferSize(), nullptr, &physicsCS2));
    DXUT_SetDebugName(physicsCS2, "CSMain2");

    V_RETURN(pd3dDevice->CreateComputeShader(pBlobUpdateCS->GetBufferPointer(), pBlobUpdateCS->GetBufferSize(), nullptr, &updateCS));
    DXUT_SetDebugName(updateCS, "PosUpdate");

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


    /// Load Collision Detection structures in one place

    // Array for BVH-decriptor structures
    uint offset = 0;
    // first masscube collision trees
    BVHDESC* bdData = new BVHDESC[objectCount];
    if (!bdData) return E_OUTOFMEMORY;

    // create tree descriptors
    for (uint i = 0; i < objectCount; i++){

        // create entry for the first ctree
        BVHDESC tmp;
        tmp.arrayOffset = offset;
        tmp.masspointCount = deformableObjects[i].ctree.size();
        tmp.minX = deformableObjects[i].ctree[0].minX;
        tmp.maxX = deformableObjects[i].ctree[0].maxX;
        tmp.minY = deformableObjects[i].ctree[0].minY;
        tmp.maxY = deformableObjects[i].ctree[0].maxY;
        tmp.minZ = deformableObjects[i].ctree[0].minZ;
        tmp.maxZ = deformableObjects[i].ctree[0].maxZ;
        bdData[i] = tmp;
        offset += tmp.masspointCount;

    }

    // Array for BVHierarchies' data
    BVBOX* btData = new BVBOX[offset];          // offset = total collision masspoint count
    if (!btData) return E_OUTOFMEMORY;

    uint ix = 0;
    // store bvbox data in one place
    for (uint i = 0; i < objectCount; i++){
        for (uint j = 0; j < deformableObjects[i].ctree.size(); j++){
            btData[ix] = deformableObjects[i].ctree[j];
            ix++;
        }
    }


    /// Create buffers

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

    // Desc for collision detection catalogue
    D3D11_BUFFER_DESC bcdesc;
    ZeroMemory(&bcdesc, sizeof(bcdesc));
    bcdesc.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;
    bcdesc.ByteWidth = objectCount * sizeof(BVHDESC);
    bcdesc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
    bcdesc.StructureByteStride = sizeof(BVHDESC);
    bcdesc.Usage = D3D11_USAGE_DEFAULT;

    // Desc for collision detection data
    D3D11_BUFFER_DESC bddesc;
    ZeroMemory(&bddesc, sizeof(bddesc));
    bddesc.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;
    bddesc.ByteWidth = offset * sizeof(BVBOX);
    bddesc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
    bddesc.StructureByteStride = sizeof(BVBOX);
    bddesc.Usage = D3D11_USAGE_DEFAULT;

    // Set initial Particles data
    D3D11_SUBRESOURCE_DATA InitData;
    InitData.pSysMem = pData1;
    V_RETURN(pd3dDevice->CreateBuffer(&desc, &InitData, &particleBuffer1));
    V_RETURN(pd3dDevice->CreateBuffer(&desc, &InitData, &particleBuffer2));
    DXUT_SetDebugName(particleBuffer1, "ParticleBuffer1");
    DXUT_SetDebugName(particleBuffer2, "ParticleBuffer2");
    SAFE_DELETE_ARRAY(pData1);

    // Set initial Volumetric data
    D3D11_SUBRESOURCE_DATA v1data;
    v1data.pSysMem = vData1;
    D3D11_SUBRESOURCE_DATA v2data;
    v2data.pSysMem = vData2;
    V_RETURN(pd3dDevice->CreateBuffer(&vdesc, &v1data, &masscube1Buffer1));
    V_RETURN(pd3dDevice->CreateBuffer(&vdesc, &v1data, &masscube1Buffer2));
    DXUT_SetDebugName(masscube1Buffer1, "VolCube1");
    DXUT_SetDebugName(masscube1Buffer2, "VolCube1c");
    SAFE_DELETE_ARRAY(vData1);
    vdesc.ByteWidth = mass2Count * sizeof(MASSPOINT);
    V_RETURN(pd3dDevice->CreateBuffer(&vdesc, &v2data, &masscube2Buffer1));
    V_RETURN(pd3dDevice->CreateBuffer(&vdesc, &v2data, &masscube2Buffer2));
    DXUT_SetDebugName(masscube2Buffer1, "VolCube2");
    DXUT_SetDebugName(masscube2Buffer2, "VolCube2c");
    SAFE_DELETE_ARRAY(vData2);

    // Buffer for IndexCube
    D3D11_BUFFER_DESC desc2;
    ZeroMemory(&desc2, sizeof(desc2));
    desc2.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;
    desc2.ByteWidth = particleCount * sizeof(INDEXER);
    desc2.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
    desc2.StructureByteStride = sizeof(INDEXER);
    desc2.Usage = D3D11_USAGE_DEFAULT;
    D3D11_SUBRESOURCE_DATA indexer_init;
    indexer_init.pSysMem = iData1;
    V_RETURN(pd3dDevice->CreateBuffer(&desc2, &indexer_init, &indexerBuffer));
    DXUT_SetDebugName(indexerBuffer, "IndexCube");
    SAFE_DELETE_ARRAY(iData1);

    // Buffer for collision detection catalogue and data
    D3D11_SUBRESOURCE_DATA bc_init;
    bc_init.pSysMem = bdData;
    V_RETURN(pd3dDevice->CreateBuffer(&bcdesc, &bc_init, &bvhCatalogueBuffer1));
    V_RETURN(pd3dDevice->CreateBuffer(&bcdesc, &bc_init, &bvhCatalogueBuffer2));
    DXUT_SetDebugName(bvhCatalogueBuffer1, "BVHCatalogue buffer1");
    DXUT_SetDebugName(bvhCatalogueBuffer2, "BVHCatalogue buffer2");
    SAFE_DELETE_ARRAY(bdData);

    D3D11_SUBRESOURCE_DATA bd_init;
    bd_init.pSysMem = btData;
    V_RETURN(pd3dDevice->CreateBuffer(&bddesc, &bd_init, &bvhDataBuffer1));
    V_RETURN(pd3dDevice->CreateBuffer(&bddesc, &bd_init, &bvhDataBuffer2));
    DXUT_SetDebugName(bvhDataBuffer1, "BVHData buffer1");
    DXUT_SetDebugName(bvhDataBuffer2, "BVHData buffer2");
    SAFE_DELETE_ARRAY(btData);

    // SRV for Particle data
    D3D11_SHADER_RESOURCE_VIEW_DESC DescRV;
    ZeroMemory(&DescRV, sizeof(DescRV));
    DescRV.Format = DXGI_FORMAT_UNKNOWN;
    DescRV.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
    DescRV.Buffer.FirstElement = 0;
    DescRV.Buffer.NumElements = particleCount;
    V_RETURN(pd3dDevice->CreateShaderResourceView(particleBuffer1, &DescRV, &particleSRV1));
    V_RETURN(pd3dDevice->CreateShaderResourceView(particleBuffer2, &DescRV, &particleSRV2));
    DXUT_SetDebugName(particleSRV1, "ParticleArray0 SRV");
    DXUT_SetDebugName(particleSRV2, "ParticleArray1 SRV");

    // SRV for indexcube
    D3D11_SHADER_RESOURCE_VIEW_DESC DescRV2;
    ZeroMemory(&DescRV2, sizeof(DescRV2));
    DescRV2.Format = DXGI_FORMAT_UNKNOWN;
    DescRV2.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
    DescRV2.Buffer.FirstElement = 0;
    DescRV2.Buffer.NumElements = particleCount;
    V_RETURN(pd3dDevice->CreateShaderResourceView(indexerBuffer, &DescRV2, &indexerSRV));
    DXUT_SetDebugName(indexerSRV, "IndexCube SRV");

    // SRV for VolCubes
    D3D11_SHADER_RESOURCE_VIEW_DESC DescRVV;
    ZeroMemory(&DescRVV, sizeof(DescRVV));
    DescRVV.Format = DXGI_FORMAT_UNKNOWN;
    DescRVV.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
    DescRVV.Buffer.FirstElement = 0;
    DescRVV.Buffer.NumElements = mass1Count;
    V_RETURN(pd3dDevice->CreateShaderResourceView(masscube1Buffer1, &DescRVV, &masscube1SRV1));
    V_RETURN(pd3dDevice->CreateShaderResourceView(masscube1Buffer2, &DescRVV, &masscube1SRV2));
    DXUT_SetDebugName(masscube1SRV1, "VolCube1 RV");
    DXUT_SetDebugName(masscube1SRV2, "VolCube1c RV");
    DescRVV.Buffer.NumElements = mass2Count;
    V_RETURN(pd3dDevice->CreateShaderResourceView(masscube2Buffer1, &DescRVV, &masscube2SRV1));
    V_RETURN(pd3dDevice->CreateShaderResourceView(masscube2Buffer2, &DescRVV, &masscube2SRV2));
    DXUT_SetDebugName(masscube2SRV1, "VolCube2 RV");
    DXUT_SetDebugName(masscube2SRV2, "VolCube2c RV");

    // SRV for collision detection catalogue and data
    D3D11_SHADER_RESOURCE_VIEW_DESC bcdescRV;
    ZeroMemory(&bcdescRV, sizeof(bcdescRV));
    bcdescRV.Format = DXGI_FORMAT_UNKNOWN;
    bcdescRV.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
    bcdescRV.Buffer.FirstElement = 0;
    bcdescRV.Buffer.NumElements = objectCount;
    V_RETURN(pd3dDevice->CreateShaderResourceView(bvhCatalogueBuffer1, &bcdescRV, &bvhCatalogueSRV1));
    V_RETURN(pd3dDevice->CreateShaderResourceView(bvhCatalogueBuffer2, &bcdescRV, &bvhCatalogueSRV2));
    DXUT_SetDebugName(bvhCatalogueSRV1, "BVHCatalogue SRV1");
    DXUT_SetDebugName(bvhCatalogueSRV2, "BVHCatalogue SRV2");

    D3D11_SHADER_RESOURCE_VIEW_DESC bddescRV;
    ZeroMemory(&bddescRV, sizeof(bddescRV));
    bddescRV.Format = DXGI_FORMAT_UNKNOWN;
    bddescRV.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
    bddescRV.Buffer.FirstElement = 0;
    bddescRV.Buffer.NumElements = offset;
    V_RETURN(pd3dDevice->CreateShaderResourceView(bvhDataBuffer1, &bddescRV, &bvhDataSRV1));
    V_RETURN(pd3dDevice->CreateShaderResourceView(bvhDataBuffer2, &bddescRV, &bvhDataSRV2));
    DXUT_SetDebugName(bvhDataSRV1, "BVHData SRV1");
    DXUT_SetDebugName(bvhDataSRV2, "BVHData SRV2");

    // UAV for Particle data
    D3D11_UNORDERED_ACCESS_VIEW_DESC DescUAV;
    ZeroMemory(&DescUAV, sizeof(D3D11_UNORDERED_ACCESS_VIEW_DESC));
    DescUAV.Format = DXGI_FORMAT_UNKNOWN;
    DescUAV.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
    DescUAV.Buffer.FirstElement = 0;
    DescUAV.Buffer.NumElements = particleCount;
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(particleBuffer1, &DescUAV, &particleUAV1));
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(particleBuffer2, &DescUAV, &particleUAV2));
    DXUT_SetDebugName(particleUAV1, "ParticleArray0 UAV");
    DXUT_SetDebugName(particleUAV2, "ParticleArray1 UAV");

    // UAVs for Volumetric data
    D3D11_UNORDERED_ACCESS_VIEW_DESC vDescUAV;
    ZeroMemory(&vDescUAV, sizeof(D3D11_UNORDERED_ACCESS_VIEW_DESC));
    vDescUAV.Format = DXGI_FORMAT_UNKNOWN;
    vDescUAV.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
    vDescUAV.Buffer.FirstElement = 0;
    vDescUAV.Buffer.NumElements = mass1Count;
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(masscube1Buffer1, &vDescUAV, &masscube1UAV1));
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(masscube1Buffer2, &vDescUAV, &masscube1UAV2));
    DXUT_SetDebugName(masscube1UAV1, "VolCube1 UAV");
    DXUT_SetDebugName(masscube1UAV2, "VolCube1c UAV");
    vDescUAV.Buffer.NumElements = mass2Count;
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(masscube2Buffer1, &vDescUAV, &masscube2UAV1));
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(masscube2Buffer2, &vDescUAV, &masscube2UAV2));
    DXUT_SetDebugName(masscube2UAV1, "VolCube2 UAV");
    DXUT_SetDebugName(masscube2UAV2, "VolCube2c UAV");

    // UAV for collision detection catalogue and data
    D3D11_UNORDERED_ACCESS_VIEW_DESC bcdescUAV;
    ZeroMemory(&bcdescUAV, sizeof(D3D11_UNORDERED_ACCESS_VIEW_DESC));
    bcdescUAV.Format = DXGI_FORMAT_UNKNOWN;
    bcdescUAV.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
    bcdescUAV.Buffer.FirstElement = 0;
    bcdescUAV.Buffer.NumElements = objectCount;
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(bvhCatalogueBuffer1, &bcdescUAV, &bvhCatalogueUAV1));
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(bvhCatalogueBuffer2, &bcdescUAV, &bvhCatalogueUAV2));
    DXUT_SetDebugName(bvhCatalogueUAV1, "BVHCatalogue UAV1");
    DXUT_SetDebugName(bvhCatalogueUAV2, "BVHCatalogue UAV2");

    D3D11_UNORDERED_ACCESS_VIEW_DESC bddescUAV;
    ZeroMemory(&bddescUAV, sizeof(D3D11_UNORDERED_ACCESS_VIEW_DESC));
    bddescUAV.Format = DXGI_FORMAT_UNKNOWN;
    bddescUAV.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
    bddescUAV.Buffer.FirstElement = 0;
    bddescUAV.Buffer.NumElements = offset;
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(bvhDataBuffer1, &bddescUAV, &bvhDataUAV1));
    V_RETURN(pd3dDevice->CreateUnorderedAccessView(bvhDataBuffer2, &bddescUAV, &bvhDataUAV2));
    DXUT_SetDebugName(bvhDataUAV1, "BVHData UAV1");
    DXUT_SetDebugName(bvhDataUAV2, "BVHData UAV2");


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
    V_RETURN(pd3dDevice->CreateBuffer(&Desc, nullptr, &gsConstantBuffer));
    DXUT_SetDebugName(gsConstantBuffer, "CB_GS");

    Desc.ByteWidth = sizeof(CB_CS);
    V_RETURN(pd3dDevice->CreateBuffer(&Desc, nullptr, &csConstantBuffer));
    DXUT_SetDebugName(csConstantBuffer, "CB_CS");

    // Load Particle Texture
    V_RETURN(DXUTCreateShaderResourceViewFromFile(pd3dDevice, L"misc\\Particle.dds", &particleTextureSRV));
    DXUT_SetDebugName(particleTextureSRV, "Particle.dds");

    // Create sampler
    D3D11_SAMPLER_DESC SamplerDesc;
    ZeroMemory(&SamplerDesc, sizeof(SamplerDesc));
    SamplerDesc.AddressU = D3D11_TEXTURE_ADDRESS_CLAMP;
    SamplerDesc.AddressV = D3D11_TEXTURE_ADDRESS_CLAMP;
    SamplerDesc.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
    SamplerDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
    V_RETURN(pd3dDevice->CreateSamplerState(&SamplerDesc, &samplerState));
    DXUT_SetDebugName(samplerState, "Linear");

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
    V_RETURN(pd3dDevice->CreateBlendState(&BlendStateDesc, &blendState));
    DXUT_SetDebugName(blendState, "Blending");

    // Create depth stencil
    D3D11_DEPTH_STENCIL_DESC DepthStencilDesc;
    ZeroMemory(&DepthStencilDesc, sizeof(DepthStencilDesc));
    DepthStencilDesc.DepthEnable = FALSE;
    DepthStencilDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
    pd3dDevice->CreateDepthStencilState(&DepthStencilDesc, &depthStencilState);
    DXUT_SetDebugName(depthStencilState, "DepthOff");

    return S_OK;
}

//--------------------------------------------------------------------------------------
// (Re)Create texture and buffers for model render target (for picking)
//--------------------------------------------------------------------------------------
HRESULT initPicking(ID3D11Device* pd3dDevice, int width, int height)
{

    SAFE_RELEASE(pickingTexture1);
    SAFE_RELEASE(pickingTexture2);
    SAFE_RELEASE(pickingRTV1);
    SAFE_RELEASE(pickingRTV2);
    SAFE_RELEASE(pickingSRV1);
    SAFE_RELEASE(pickingSRV2);

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

    pd3dDevice->CreateTexture2D(&tdesc, nullptr, &pickingTexture1);
    DXUT_SetDebugName(pickingTexture1, "Picking TEX 1");
    pd3dDevice->CreateTexture2D(&tdesc, nullptr, &pickingTexture2);
    DXUT_SetDebugName(pickingTexture2, "Picking TEX 2");

    rtvdesc.Format = tdesc.Format;
    rtvdesc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2D;
    rtvdesc.Texture2D.MipSlice = 0;
    pd3dDevice->CreateRenderTargetView(pickingTexture1, &rtvdesc, &pickingRTV1);
    DXUT_SetDebugName(pickingRTV1, "Picking RTV 1");
    pd3dDevice->CreateRenderTargetView(pickingTexture2, &rtvdesc, &pickingRTV2);
    DXUT_SetDebugName(pickingRTV2, "Picking RTV 2");

    srvdesc.Format = tdesc.Format;
    srvdesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
    srvdesc.Texture2D.MostDetailedMip = 0;
    srvdesc.Texture2D.MipLevels = 1;
    pd3dDevice->CreateShaderResourceView(pickingTexture1, &srvdesc, &pickingSRV1);
    DXUT_SetDebugName(pickingSRV1, "Picking SRV 1");
    pd3dDevice->CreateShaderResourceView(pickingTexture2, &srvdesc, &pickingSRV2);
    DXUT_SetDebugName(pickingSRV2, "Picking SRV 2");

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
        if (MessageBox(0, L"CS5x capability is missing. "\
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
    XMVECTOR vecEye = XMVectorSet(-spreadConstant * 0, spreadConstant * 0, -spreadConstant * 27, 0.0f);
    XMVECTOR vecAt = XMVectorSet(0.0f, 0.0f, 0.0f, 0.0f);
    camera.SetViewParams(vecEye, vecAt);

    setUpDialog.DestroyDialog();

    return S_OK;
}

//--------------------------------------------------------------------------------------
// On screen resize
//--------------------------------------------------------------------------------------
HRESULT CALLBACK OnD3D11ResizedSwapChain(ID3D11Device* pd3dDevice, IDXGISwapChain* pSwapChain, const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc, void* pUserContext){

    HRESULT hr = S_OK;

    // Setup the camera's projection parameters
    windowWidth = pBackBufferSurfaceDesc->Width;
    windowHeight = pBackBufferSurfaceDesc->Height;
    float fAspectRatio = pBackBufferSurfaceDesc->Width / (FLOAT)pBackBufferSurfaceDesc->Height;
    camera.SetProjParams(XM_PI / 4, fAspectRatio, 10.0f, 500000.0f);
    camera.SetWindow(pBackBufferSurfaceDesc->Width, pBackBufferSurfaceDesc->Height);
    camera.SetButtonMasks(0, MOUSE_WHEEL, MOUSE_LEFT_BUTTON | MOUSE_MIDDLE_BUTTON);

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
    pd3dImmediateContext->OMSetRenderTargets(1, &pickingRTV1, pDSV);
    pd3dImmediateContext->ClearRenderTargetView(pickingRTV1, Colors::Black);

    ID3D11BlendState *pBlendState0 = nullptr;
    ID3D11DepthStencilState *pDepthStencilState0 = nullptr;
    UINT SampleMask0, StencilRef0;
    XMFLOAT4 BlendFactor0;
    pd3dImmediateContext->OMGetBlendState(&pBlendState0, &BlendFactor0.x, &SampleMask0);
    pd3dImmediateContext->OMGetDepthStencilState(&pDepthStencilState0, &StencilRef0);

    pd3dImmediateContext->VSSetShader(renderVS, nullptr, 0);
    pd3dImmediateContext->GSSetShader(renderGS, nullptr, 0);
    pd3dImmediateContext->PSSetShader(pickingPS1, nullptr, 0);

    ID3D11ShaderResourceView* aRViews[1] = { particleSRV1 };
    pd3dImmediateContext->VSSetShaderResources(0, 1, aRViews);

    D3D11_MAPPED_SUBRESOURCE MappedResource;
    pd3dImmediateContext->Map(gsConstantBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource);
    auto pCBGS = reinterpret_cast<CB_GS*>(MappedResource.pData);
    XMStoreFloat4x4(&pCBGS->worldViewProjection, XMMatrixMultiply(mView, mProj));
    XMStoreFloat4x4(&pCBGS->inverseView, XMMatrixInverse(nullptr, mView));
    XMStoreFloat4(&pCBGS->eyePos, camera.GetEyePt());
    pCBGS->lightPos = lightPos;
    pd3dImmediateContext->Unmap(gsConstantBuffer, 0);
    pd3dImmediateContext->GSSetConstantBuffers(0, 1, &gsConstantBuffer);

    pd3dImmediateContext->PSSetShaderResources(0, 1, &particleTextureSRV);
    pd3dImmediateContext->PSSetSamplers(0, 1, &samplerState);

    pd3dImmediateContext->OMSetBlendState(nullptr, nullptr, 0xFFFFFFFF);
    pd3dImmediateContext->OMSetDepthStencilState(depthStencilState, 0);


    // Render first "layer" of information: vertex IDs in first volcube
    pd3dImmediateContext->Draw(particleCount, 0);

    // Render second layer of information: vertex IDs in second volcube
    pd3dImmediateContext->ClearDepthStencilView(pDSV, D3D11_CLEAR_DEPTH, 1.0, 0);
    pd3dImmediateContext->OMSetRenderTargets(1, &pickingRTV2, pDSV);
    pd3dImmediateContext->ClearRenderTargetView(pickingRTV2, Colors::Black);
    pd3dImmediateContext->PSSetShader(pickingPS2, nullptr, 0);
    pd3dImmediateContext->Draw(particleCount, 0);


    ID3D11ShaderResourceView* ppSRVNULL[1] = { nullptr };
    pd3dImmediateContext->VSSetShaderResources(0, 1, ppSRVNULL);
    pd3dImmediateContext->PSSetShaderResources(0, 1, ppSRVNULL);

    pd3dImmediateContext->GSSetShader(nullptr, nullptr, 0);
    pd3dImmediateContext->OMSetBlendState(pBlendState0, &BlendFactor0.x, SampleMask0); SAFE_RELEASE(pBlendState0);
    pd3dImmediateContext->OMSetDepthStencilState(pDepthStencilState0, StencilRef0); SAFE_RELEASE(pDepthStencilState0);

    pd3dImmediateContext->OMSetRenderTargets(1, &pRTV, pDSV);
    renderPicking = false;

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

    pd3dImmediateContext->VSSetShader(renderVS, nullptr, 0);
    pd3dImmediateContext->GSSetShader(renderGS, nullptr, 0);
    pd3dImmediateContext->PSSetShader(renderPS, nullptr, 0);

    ID3D11ShaderResourceView* aRViews[1] = { particleSRV1 };
    pd3dImmediateContext->VSSetShaderResources(0, 1, aRViews);

    D3D11_MAPPED_SUBRESOURCE MappedResource;
    pd3dImmediateContext->Map(gsConstantBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource);
    auto pCBGS = reinterpret_cast<CB_GS*>(MappedResource.pData);
    XMStoreFloat4x4(&pCBGS->worldViewProjection, XMMatrixMultiply(mView, mProj));
    XMStoreFloat4x4(&pCBGS->inverseView, XMMatrixInverse(nullptr, mView));
    XMStoreFloat4(&pCBGS->eyePos, camera.GetEyePt());
    pCBGS->lightPos = lightPos;
    pd3dImmediateContext->Unmap(gsConstantBuffer, 0);
    pd3dImmediateContext->GSSetConstantBuffers(0, 1, &gsConstantBuffer);

    pd3dImmediateContext->PSSetShaderResources(0, 1, &particleTextureSRV);
    pd3dImmediateContext->PSSetSamplers(0, 1, &samplerState);

    float bf[] = { 0.f, 0.f, 0.f, 0.f };
    pd3dImmediateContext->OMSetBlendState(blendState, bf, 0xFFFFFFFF);
    pd3dImmediateContext->OMSetDepthStencilState(depthStencilState, 0);

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

    XMMATRIX mView = camera.GetViewMatrix();
    XMMATRIX mProj = camera.GetProjMatrix();

    // Render the model first for picking, only if RMBUTTONDOWN happened
    if (renderPicking){
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
    SAFE_RELEASE(blendState);
    SAFE_RELEASE(depthStencilState);
    SAFE_RELEASE(samplerState);
    SAFE_RELEASE(particleTextureSRV);
    SAFE_RELEASE(bvhCatalogueBuffer1);
    SAFE_RELEASE(bvhCatalogueBuffer2);
    SAFE_RELEASE(bvhDataBuffer1);
    SAFE_RELEASE(bvhDataBuffer2);
    SAFE_RELEASE(csConstantBuffer);
    SAFE_RELEASE(gsConstantBuffer);
    SAFE_RELEASE(indexerBuffer);
    SAFE_RELEASE(masscube1Buffer1);
    SAFE_RELEASE(masscube1Buffer2);
    SAFE_RELEASE(masscube2Buffer1);
    SAFE_RELEASE(masscube2Buffer2);
    SAFE_RELEASE(particleBuffer1);
    SAFE_RELEASE(particleBuffer2);
    SAFE_RELEASE(pickingRTV1);
    SAFE_RELEASE(pickingRTV2);
    SAFE_RELEASE(bvhCatalogueSRV1);
    SAFE_RELEASE(bvhCatalogueSRV2);
    SAFE_RELEASE(bvhDataSRV1);
    SAFE_RELEASE(bvhDataSRV2);
    SAFE_RELEASE(indexerSRV);
    SAFE_RELEASE(masscube1SRV1);
    SAFE_RELEASE(masscube1SRV2);
    SAFE_RELEASE(masscube2SRV1);
    SAFE_RELEASE(masscube2SRV2);
    SAFE_RELEASE(particleSRV1);
    SAFE_RELEASE(particleSRV2);
    SAFE_RELEASE(pickingSRV1);
    SAFE_RELEASE(pickingSRV2);
    SAFE_RELEASE(pickingTexture1);
    SAFE_RELEASE(pickingTexture2);
    SAFE_RELEASE(bvhCatalogueUAV1);
    SAFE_RELEASE(bvhCatalogueUAV2);
    SAFE_RELEASE(bvhDataUAV1);
    SAFE_RELEASE(bvhDataUAV2);
    SAFE_RELEASE(masscube1UAV1);
    SAFE_RELEASE(masscube1UAV2);
    SAFE_RELEASE(masscube2UAV1);
    SAFE_RELEASE(masscube2UAV2);
    SAFE_RELEASE(particleUAV1);
    SAFE_RELEASE(particleUAV2);
    SAFE_RELEASE(physicsCS1);
    SAFE_RELEASE(physicsCS2);
    SAFE_RELEASE(updateCS);
    SAFE_RELEASE(renderGS);
    SAFE_RELEASE(renderPS);
    SAFE_RELEASE(pickingPS1);
    SAFE_RELEASE(pickingPS2);
    SAFE_RELEASE(renderVS);
    
}