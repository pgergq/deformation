//--------------------------------------------------------------------------------------
// File: IPCServer.h
//
// Project Deformation
// Object deformation with mass-spring systems
//
// Inter-process communication server
//
// @Copyright (c) pgq
//--------------------------------------------------------------------------------------

#ifndef _IPCSERVER_H_
#define _IPCSERVER_H_

#include <windows.h>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <atomic>
#include <tuple>
#include "Constants.h"
#include "Deformable.h"

#define BUFSIZE 512

extern HANDLE hPipe;
extern std::vector<Deformable> deformableObjects;
extern bool isReceiving;

HRESULT StartServer();
void InstanceThread(LPVOID);
wstuple ProcessCommand(byte*, int);


#endif