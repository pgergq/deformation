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

#include "../Headers/IPCClient.h"

HANDLE hPipe_comm = INVALID_HANDLE_VALUE;
wchar_t chBuf[BUFSIZE];
bool isIPC = true;


//--------------------------------------------------------------------------------------
// IPCOpenPipe: Connect to named pipe
//--------------------------------------------------------------------------------------
void IPCPipeClient(const wchar_t* pipeName){

    // variables
    std::wstring out = L"";

    // open named pipe
    while (true)
    {
        // check if still running
        if (!isIPC) return;

        hPipe_comm = CreateFile(pipeName, GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);

        // pipe open
        if (hPipe_comm != INVALID_HANDLE_VALUE){
            out = L"[.] PipeClient: connected\n";
            OutputDebugString(out.c_str());
            break;
        }
    }

    // pipe connected; change to byte-read mode
    if (!SetNamedPipeHandleState(hPipe_comm, PIPE_READMODE_BYTE, nullptr, nullptr))
    {
        out = L"\n[.] PipeClient: SetNamedPipeHandleState failed\n";
        OutputDebugString(out.c_str());
        return;
    }

    IPCProcessMessages();

    out = L"[.] PipeClient: exiting\n";
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

    DWORD cbBytesRead = 0, cbWritten = 0;
    bool fSuccess = false;

    // error checking
    if (pchRequest == nullptr || pchReply == nullptr || hPipe_comm == nullptr)
    {
        out = L"[!] PipeClient: memory allocation error\n";
        OutputDebugString(out.c_str());
        if (pchReply != nullptr) HeapFree(hHeap, 0, pchReply);
        if (pchRequest != nullptr) HeapFree(hHeap, 0, pchRequest);
        return;
    }

    // client receiving and processing messages
    // buffer reading cycle
    while (isIPC)
    {
        fSuccess = ReadFile(hPipe_comm, pchRequest, BUFSIZE * sizeof(wchar_t), &cbBytesRead, nullptr) != 0;
        if (!fSuccess || cbBytesRead == 0)
        {
            if (GetLastError() == ERROR_BROKEN_PIPE){
                out = L"[.] PipeClient: server disconnected\n";
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
        DWORD replyBytes = rep.size() * sizeof(wchar_t);

        // write the reply to the pipe
        fSuccess = WriteFile(hPipe_comm, reply, replyBytes, &cbWritten, nullptr) != 0;
        if (!fSuccess || replyBytes != cbWritten)
        {
            out = L"[!] PipeClient: writing error\n";
            OutputDebugString(out.c_str());
            break;
        }

        out = L"[.] Command [" + req + L"] processed: " + rep + L"\n";
        OutputDebugString(out.c_str());
    }

    FlushFileBuffers(hPipe_comm);
    CloseHandle(hPipe_comm);

    HeapFree(hHeap, 0, pchRequest);
    HeapFree(hHeap, 0, pchReply);

    return;
}


//--------------------------------------------------------------------------------------
// ProcessCommand: Read commands from input buffer
//--------------------------------------------------------------------------------------
wstuple ProcessCommand(byte* buf, int validBytes){

    // Get command from buffer command
    // incoming command
    std::wstring request = L"";
    std::wstring reply = L"";
    for (int i = 0; i < validBytes - 2; i++){
        request += (char)buf[i];
    }

    /// Process command...
    std::string type, param;
    float valueX = 0.0f, valueY = 0.0f, valueZ = 0.0f;
    unsigned int num = 0;
    std::stringstream x(std::string(request.begin(), request.end()));

    // get command type
    x >> type;
    // SET commands
    if (type == "set"){
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

    // ADD command
    else if (type == "add"){
        x >> param >> num;
        if (param == "bunny"){
            uint bc = sceneObjects.size();
            for (uint i = 0; i < num; i++)
            {
                DeformableOBJ tmp("bunny_res3_scaled.obj", bc + i);
                tmp.build();
                sceneObjects.push_back(std::make_unique<DeformableOBJ>(tmp));
            }
            reply = L"added " + std::to_wstring(num) + L" bunnies";
        }
    }

    // GET commands
    else if (type == "get"){
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
            reply = std::to_wstring(sceneObjects.size());
        }
        else
        {
            reply = L"unrecognized get command";
        }
    }

    // invalid command
    else
    {
        reply = L"unrecognized command";
    }

    // reply
    reply = L"[" + request + L"] -> [" + reply + L"]";
    return wstuple(request, reply);
}
