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
#include <tuple>
#include "Constants.h"
#include "Deformable.h"

#define BUFSIZE 512

extern HANDLE hPipe_comm;           // pipe handle
extern HANDLE hPipe_quit;           // pipe handle
extern bool isIPC;                  // IPC status
extern std::vector<Deformable> deformableObjects;

void IPCPipeClient(const wchar_t*); // open named pipe
void IPCProcessMessages();          // IPC-client message processor function
wstuple ProcessCommand(byte*, int); // message processor

#endif