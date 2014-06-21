//--------------------------------------------------------------------------------------
// File: IPCClient.cpp
//
// Project Deformation
// Object deformation with mass-spring systems
//
// Inter-process communication client implementation
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#include "IPCClient.h"

HANDLE hPipe = INVALID_HANDLE_VALUE;
wchar_t chBuf[BUFSIZE];
bool isIPC = true;


//--------------------------------------------------------------------------------------
// IPCOpenPipe: Connect to named pipe
//--------------------------------------------------------------------------------------
void IPCPipeClient(){

    /// variables
    bool fSuccess = false;
    DWORD cbRead, cbToWrite, cbWritten;
    const wchar_t* pipeName = L"\\\\.\\pipe\\deformablepipe";
    std::wstring out = L"";

    /// open named pipe
    while (true)
    {
        // check if still running
        if (!isIPC) return;

        hPipe = CreateFile(pipeName, GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
        // pipe opened
        if (hPipe != INVALID_HANDLE_VALUE)
            break;
    }

    /// pipe connected; change to byte-read mode
    if (!SetNamedPipeHandleState(hPipe, PIPE_READMODE_BYTE, nullptr, nullptr))
    {
        out = L"\n[.] PipeClient: SetNamedPipeHandleState failed\n";
        OutputDebugString(out.c_str());
        return;
    }

    IPCProcessMessages();

    out = L"[.] PipeClient exiting\n";
    OutputDebugString(out.c_str());

    return;
}

//--------------------------------------------------------------------------------------
// InstanceThread: Message processing thread
//--------------------------------------------------------------------------------------
void IPCProcessMessages()
{
    HANDLE hHeap = GetProcessHeap();
    wchar_t* pchRequest = (wchar_t*)HeapAlloc(hHeap, 0, BUFSIZE * sizeof(wchar_t));
    wchar_t* pchReply = (wchar_t*)HeapAlloc(hHeap, 0, BUFSIZE * sizeof(wchar_t));
    std::wstring out = L"";

    DWORD cbBytesRead = 0, cbReplyBytes = 0, cbWritten = 0;
    bool fSuccess = false;
    HANDLE hlPipe = nullptr;

    /// error checking
    if (pchRequest == nullptr || pchReply == nullptr || hPipe == nullptr)
    {
        out = L"[!] PipeClient: memory allocation error\n";
        OutputDebugString(out.c_str());
        if (pchReply != nullptr) HeapFree(hHeap, 0, pchReply);
        if (pchRequest != nullptr) HeapFree(hHeap, 0, pchRequest);
        return;
    }

    /// client receiving and processing messages
    // buffer reading cycle
    while (isIPC)
    {
        fSuccess = ReadFile(hlPipe, pchRequest, BUFSIZE * sizeof(wchar_t), &cbBytesRead, nullptr);
        if (!fSuccess || cbBytesRead == 0)
        {
            if (GetLastError() == ERROR_BROKEN_PIPE){
                out = L"[.] PipeClient: client disconnected\n";
                OutputDebugString(out.c_str());
            }
            else{
                out = L"[!] PipeClient: reading error\n";
                OutputDebugString(out.c_str());
            }
            break;
        }

        // process the incoming message
        wstuple tmp = ProcessCommand((byte*)pchRequest, cbBytesRead);
        std::wstring req = std::get<0>(tmp);
        std::wstring rep = std::get<1>(tmp);
        const wchar_t* reply = rep.c_str();
        int replyBytes = rep.size()*sizeof(wchar_t);

        // write the reply to the pipe
        fSuccess = WriteFile(hlPipe, reply, replyBytes, &cbWritten, nullptr);
        if (!fSuccess || replyBytes != cbWritten)
        {
            out = L"[!] PipeClient: writing error\n";
            OutputDebugString(out.c_str());
            break;
        }

        out = L"[.] Command [" + req + L"] processed: " + rep + L"\n";
        OutputDebugString(out.c_str());
    }

    FlushFileBuffers(hlPipe);
    DisconnectNamedPipe(hlPipe);
    CloseHandle(hlPipe);

    HeapFree(hHeap, 0, pchRequest);
    HeapFree(hHeap, 0, pchReply);

    return;
}


//--------------------------------------------------------------------------------------
// ProcessCommand: Read commands from input buffer
//--------------------------------------------------------------------------------------
wstuple ProcessCommand(byte* buf, int validBytes){

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
            reply = std::to_wstring(dampingConstant);
        }
        else if (param == "gravity")
        {
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
    return wstuple(request, reply);
}