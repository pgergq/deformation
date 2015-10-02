//--------------------------------------------------------------------------------------
// File: IPCClient.h
//
// Project Deformation
// Object deformation with mass-spring systems
//
// Inter-process communication client
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#ifndef _IPCCLIENT_H_
#define _IPCCLIENT_H_

#include <windows.h>
#include <iostream>
#include <sstream>
#include <string>
#include <memory>
#include <tuple>
#include "Constants.h"
#include "DeformableOBJ.h"

#define BUFSIZE 512

// pipe handle
extern HANDLE hPipe_comm;
// pipe handle
extern HANDLE hPipe_quit;
// IPC status
extern bool isIPC;
extern std::vector<std::unique_ptr<DeformableBase>> sceneObjects;

// open named pipe
void IPCPipeClient(const wchar_t*);
// IPC-client message processor function
void IPCProcessMessages();
// message processor
wstuple ProcessCommand(byte*, int);

#endif