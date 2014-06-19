//--------------------------------------------------------------------------------------
// File: IPCServer.cpp
//
// Project Deformation
// Object deformation with mass-spring systems
//
// Inter-process communication server implementation
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#include "IPCServer.h"


HANDLE hPipe = INVALID_HANDLE_VALUE;

//--------------------------------------------------------------------------------------
// StartServer: Host NamedPipe server and accept commands
//--------------------------------------------------------------------------------------
HRESULT StartServer(){

    bool fConnected = false;
    const wchar_t* pipeName = L"\\\\.\\pipe\\deformablepipe";

    /// create named pipe
    std::cout << "\n[.] PipeServer: initialized at [" << pipeName << "]" << std::endl;
    hPipe = CreateNamedPipe(pipeName, PIPE_ACCESS_DUPLEX, PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT, PIPE_UNLIMITED_INSTANCES, BUFSIZE, BUFSIZE, 0, nullptr);

    if (hPipe == INVALID_HANDLE_VALUE)
    {
        std::cout << "[!] CreateNamedPipe failed, GLE=" << GetLastError() << std::endl;
        return E_FAIL;
    }

    /// connect client
    fConnected = ConnectNamedPipe(hPipe, nullptr) ? true : (GetLastError() == ERROR_PIPE_CONNECTED);
    if (fConnected)
    {
        std::cout << "[.] PipeServer: client connected." << std::endl;

    }
    else
        CloseHandle(hPipe);

    std::cout << "[.] PipeServer: processing messages..." << std::endl;
    return S_OK;
}


//--------------------------------------------------------------------------------------
// ProcessCommand: Read commands from input buffer
//--------------------------------------------------------------------------------------
std::tuple<std::wstring, std::wstring> ProcessCommand(byte* buf, int validBytes){

    /// Get command from buffer command
    std::wstring request = L"";                         // incoming command
    std::wstring reply = L"";
    for (int i = 0; i < validBytes - 2; i++){
        request += (char)buf[i];
    }

    /// Process command...
    std::string type, param;
    float valueX = 0.0f, valueY = 0.0f, valueZ = 0.0f;
    int num = 0;
    std::stringstream x(std::string(request.begin(), request.end()));

    x >> type;                                          // get command type
    if (type == "set"){                                 // SET commands
        x >> param;
        if (param == "stiffness")
        {
            x >> valueX;
            stiffnessConstant = valueX;
            reply = L"ok";
        }
        else if (param == "damping")
        {
            x >> valueX;
            dampingConstant = valueX;
            reply = L"ok";
        }
        else if (param == "gravity")
        {
            x >> valueX;
            gravityConstant = valueX;
            reply = L"ok";
        }
        else if (param == "lightpos")
        {
            x >> valueX >> valueY >> valueZ;
            lightPos.store(VECTOR4(valueX, valueY, valueZ, 1.0f));
            reply = L"ok";
        }
        else if (param == "lightcol")
        {
            x >> valueX >> valueY >> valueZ;
            lightCol.store(VECTOR4(valueX, valueY, valueZ, 1.0f));
            reply = L"ok";
        }
        else
        {
            reply = L"unrecognized set command";
        }
    }

    else if (type == "add"){                            // ADD command
        x >> param >> num;
        if (param == "bunny"){
            uint bc = deformableObjects.size();
            for (uint i = 0; i < num; i++)
            {
                Deformable tmp("bunny_res3_scaled.obj", bc + i);
                tmp.build();
                deformableObjects.push_back(tmp);
            }
            reply = L"added " + std::to_wstring(num) + L" bunnies";
        }
    }

    else if (type == "get"){                            // GET commands
        x >> param;
        if (param == "stiffness"){
            reply = std::to_wstring(stiffnessConstant);
        }
        else if (param == "damping")
        {
            //#TODO
            reply = std::to_wstring(dampingConstant);
        }
        else if (param == "gravity")
        {
            //#TODO
            reply = std::to_wstring(gravityConstant);
        }
        else if (param == "lightpos")
        {
            VECTOR4 tmp = lightPos.load();
            reply = L"(" + std::to_wstring(tmp.x) + L"," + std::to_wstring(tmp.y) + L"," + std::to_wstring(tmp.z) + L"," + std::to_wstring(tmp.w) + L")";
        }
        else if (param == "lightcol")
        {
            VECTOR4 tmp = lightCol.load();
            reply = L"(" + std::to_wstring(tmp.x) + L"," + std::to_wstring(tmp.y) + L"," + std::to_wstring(tmp.z) + L"," + std::to_wstring(tmp.w) + L")";
        }
        else if (param == "bunny")
        {
            reply = std::to_wstring(deformableObjects.size());
        }
        else
        {
            reply = L"unrecognized get command";
        }
    }

    else                                                // invalid command
    {
        reply = L"unrecognized command";
    }

    /// Reply
    reply = L"[" + request + L"] -> [" + reply + L"]";
    return std::tuple<std::wstring, std::wstring>(request, reply);
}


//--------------------------------------------------------------------------------------
// InstanceThread: Message processing thread
//--------------------------------------------------------------------------------------
void InstanceThread(LPVOID lpvParam)
{
    HANDLE hHeap = GetProcessHeap();
    wchar_t* pchRequest = (wchar_t*)HeapAlloc(hHeap, 0, BUFSIZE * sizeof(wchar_t));
    wchar_t* pchReply = (wchar_t*)HeapAlloc(hHeap, 0, BUFSIZE * sizeof(wchar_t));

    DWORD cbBytesRead = 0, cbReplyBytes = 0, cbWritten = 0;
    bool fSuccess = false;
    HANDLE hlPipe = nullptr;

    /// error checking
    if (pchRequest == nullptr || pchReply == nullptr || lpvParam == nullptr)
    {
        std::cout << "[!] PipeServer: memory allocation error." << std::endl;
        if (pchReply != nullptr) HeapFree(hHeap, 0, pchReply);
        if (pchRequest != nullptr) HeapFree(hHeap, 0, pchRequest);
        return;
    }

    /// instanceThread receiving and processing messages

    // the thread's parameter is a handle to a pipe object instance. 
    hlPipe = (HANDLE)lpvParam;

    /// buffer reading cycle
    while (true)
    {
        fSuccess = ReadFile(hlPipe, pchRequest, BUFSIZE * sizeof(wchar_t), &cbBytesRead, nullptr);
        if (!fSuccess || cbBytesRead == 0)
        {
            if (GetLastError() == ERROR_BROKEN_PIPE)
                std::cout << "[.] PipeServer: client disconnected." << std::endl;
            else
                std::cout << "[!] PipeServer: reading error" << std::endl;
            break;
        }

        // process the incoming message
        std::tuple<std::wstring, std::wstring> tmp = ProcessCommand((byte*)pchRequest, cbBytesRead);
        std::wstring req = std::get<0>(tmp);
        std::wstring rep = std::get<1>(tmp);
        const wchar_t* reply = rep.c_str();
        int replyBytes = rep.size()*sizeof(wchar_t);

        // write the reply to the pipe
        fSuccess = WriteFile(hlPipe, reply, replyBytes, &cbWritten, nullptr);
        if (!fSuccess || replyBytes != cbWritten)
        {
            std::cout << "[!] PipeServer: writing error" << std::endl;
            break;
        }

        std::cout << "[.] Command [" << std::string(req.begin(), req.end()) << "] processed: " << std::string(rep.begin(), rep.end()) << std::endl;
    }

    FlushFileBuffers(hlPipe);
    DisconnectNamedPipe(hlPipe);
    CloseHandle(hlPipe);

    HeapFree(hHeap, 0, pchRequest);
    HeapFree(hHeap, 0, pchReply);

    std::cout << "[.] InstanceThread exiting." << std::endl;
    return;
}