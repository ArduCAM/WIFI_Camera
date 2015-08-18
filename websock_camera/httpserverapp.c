//*****************************************************************************
// httpserver_app.c
//
// camera application macro & APIs
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************
//*****************************************************************************
//
//! \addtogroup Httpserverapp
//! @{
//
//*****************************************************************************

#include <string.h>
#include <stdlib.h>

// Driverlib Includes
#include "rom_map.h"
#include "hw_types.h"
#include "prcm.h"
#include "utils.h"

// SimpleLink include
#include "simplelink.h"

// Free-RTOS/TI-RTOS include
#include "osi.h"

// HTTP lib includes
#include "HttpCore.h"
#include "HttpRequest.h"

// Common-interface includes
#include "network_if.h"
#include "uart_if.h"
#include "common.h"

#include "websocketapp.h"
#include "httpserverapp.h"
#include "camera_app.h"
#include "i2cconfig.h"
#include "mt9d111.h"


/****************************************************************************/
/*				MACROS										*/
/****************************************************************************/
#define CAMERA_SERVICE_PRIORITY     1

/****************************************************************************
                              Global variables
****************************************************************************/
char *g_Buffer;
UINT8 g_success = 0;
int g_close = 0;
UINT16 g_uConnection;
OsiTaskHandle g_iCameraTaskHdl = 0;


void WebSocketCloseSessionHandler(void)
{
	g_close = 1;
}

void CameraAppTask(void *param)
{
	UINT8 Opcode = 0x02;
	struct HttpBlob Write;

	InitCameraComponents(640, 480);

	while(1)
	{
		if(g_close == 0)
		{
			Write.uLength = StartCamera((char **)&Write.pData);

			if(!sl_WebSocketSend(g_uConnection, Write, Opcode))
			{
				while(1);
			}
		}
	}

}


/*!
 * 	\brief 					This websocket Event is called when WebSocket Server receives data
 * 							from client.
 *
 *
 * 	\param[in]  uConnection	Websocket Client Id
 * 	\param[in] *ReadBuffer		Pointer to the buffer that holds the payload.
 *
 * 	\return					none.
 *     					
 */
void WebSocketRecvEventHandler(UINT16 uConnection, char *ReadBuffer)
{
	char *camera = "capture";

	/*
	 * UINT8 Opcode;
	 * struct HttpBlob Write;
	*/

	g_uConnection = uConnection;

	g_Buffer = ReadBuffer;
	g_close = 0;
	if (!strcmp(ReadBuffer,camera))
	{
		if(!g_iCameraTaskHdl)
		{
			osi_TaskCreate(CameraAppTask,
								   "CameraApp",
									1024,
									NULL,
									CAMERA_SERVICE_PRIORITY,
									&g_iCameraTaskHdl);
		}

	}
	//Free memory as we are not using anywhere later
	free(g_Buffer);
	g_Buffer = NULL;
	/* Enter websocket application code here */
	return;
}


/*!
 * 	\brief 						This websocket Event indicates successful handshake with client
 * 								Once this is called the server can start sending data packets over websocket using
 * 								the sl_WebSocketSend API.
 *
 *
 * 	\param[in] uConnection			Websocket Client Id
 *
 * 	\return						none
 */
void WebSocketHandshakeEventHandler(UINT16 uConnection)
{
	g_success = 1;
	g_uConnection = uConnection;
}


//****************************************************************************
//
//! Task function start the device and crete a TCP server showcasing the smart
//! plug
//!
//****************************************************************************


void HttpServerAppTask(void * param)
{
	long lRetVal = -1;
	
	//Start SimpleLink in AP Mode
	lRetVal = Network_IF_InitDriver(ROLE_AP);
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }	

	//Stop Internal HTTP Server
	lRetVal = sl_NetAppStop(SL_NET_APP_HTTP_SERVER_ID);
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }	

	//Run APPS HTTP Server
	HttpServerInitAndRun(NULL);

	LOOP_FOREVER();

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
