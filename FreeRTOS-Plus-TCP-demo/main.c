/*
 * FreeRTOS+TCP V2.3.2
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/**
 * @file main.c
 * @brief FreeRTOS-Plus-TCP demo.
 */

#include <stdio.h>

#include "RTE_Components.h"
#include CMSIS_device_header

#include <FreeRTOS.h>
#include "task.h"

#include <FreeRTOSConfig.h>

#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

/* For demo tasks*/
#include "SimpleTCPEchoServer.h"
#include "SimpleUDPEchoServer.h"

/* Echo server task parameters. */
#define mainECHO_SERVER_TASK_STACK_SIZE               ( configMINIMAL_STACK_SIZE )
#define mainECHO_SERVER_TASK_PRIORITY                 ( tskIDLE_PRIORITY + 1 )

#define mainUDP_ECHO_SERVER_PORT                      7
/** @brief Used by the pseudo random number generator. */
static UBaseType_t ulNextRand;

/** @brief Local MAC Address */
const uint8_t ucMACAddress[ 6 ] =
{
	configMAC_ADDR0,
    configMAC_ADDR1,
    configMAC_ADDR2,
    configMAC_ADDR3,
    configMAC_ADDR4,
    configMAC_ADDR5
};
/** @brief Local IP address */
static const uint8_t ucIPAddress[ 4 ] =
{
    configIP_ADDR0,
    configIP_ADDR1,
    configIP_ADDR2,
    configIP_ADDR3
};

/** @brief Net mask */
static const uint8_t ucNetMask[ 4 ] =
{
    configNET_MASK0,
    configNET_MASK1,
    configNET_MASK2,
    configNET_MASK3
};

/** @brief Gateway address */
static const uint8_t ucGatewayAddress[ 4 ] =
{
    configGATEWAY_ADDR0,
    configGATEWAY_ADDR1,
    configGATEWAY_ADDR2,
    configGATEWAY_ADDR3
};

/** @brief DNS Server Address */
static const uint8_t ucDNSServerAddress[ 4 ] =
{
    configDNS_SERVER_ADDR0,
    configDNS_SERVER_ADDR1,
    configDNS_SERVER_ADDR2,
    configDNS_SERVER_ADDR3
};

/** @brief Utility function to generate a pseudo random number */
UBaseType_t uxRand( void )
{
    const uint32_t ulMultiplier = 0x015a4e35UL, ulIncrement = 1UL;

    ulNextRand = ( ulMultiplier * ulNextRand ) + ulIncrement;

    return( ( int ) ( ulNextRand ) & 0x7fffUL );
}

/**
 * @brief Get a pseuo random number
 *
 * @param[out] pulNumber: Address to store the random number in
 */
BaseType_t xApplicationGetRandomNumber( uint32_t * pulNumber )
{
    *pulNumber = uxRand();

    return pdTRUE;
}

/**
 * @brief Callback to provide a TCP initial sequence number
 *
 * Callback that provides the inputs necessary to generate a randomized TCP
 * Initial Sequence Number per RFC 6528.  THIS IS ONLY A DUMMY IMPLEMENTATION
 * THAT RETURNS A PSEUDO RANDOM NUMBER SO IS NOT INTENDED FOR USE IN PRODUCTION
 * SYSTEMS.
 *
 */
extern uint32_t ulApplicationGetNextSequenceNumber( uint32_t ulSourceAddress,
                                                    uint16_t usSourcePort,
                                                    uint32_t ulDestinationAddress,
                                                    uint16_t usDestinationPort )
{
	( void ) ulSourceAddress;
	( void ) usSourcePort;
	( void ) ulDestinationAddress;
    ( void ) usDestinationPort;

    return uxRand();
}

/**
 * @brief Function called by FreeRTOS+TCP when network connects/disconnects
 *
 * Defined by the application code, but called by FreeRTOS+TCP when the network
 * connects/disconnects (if ipconfigUSE_NETWORK_EVENT_HOOK is set to 1 in
 * FreeRTOSIPConfig.h).
 *
 * For the demo of the FreeRTOS+TCP, when the network is up, this function creates
 * one TCP echo server and one UDP echo server tasks. It also prints the network
 * address information (IP address, netmask, gateway address etc.).
 */
void vApplicationIPNetworkEventHook( eIPCallbackEvent_t eNetworkEvent )
{
uint32_t ulIPAddress, ulNetMask, ulGatewayAddress, ulDNSServerAddress;
static BaseType_t xTasksAlreadyCreated = pdFALSE;
int8_t cBuffer[ 16 ];

    /* Check this was a network up event, as opposed to a network down event. */
    if( eNetworkEvent == eNetworkUp )
    {
        /* Create the tasks that use the TCP/IP stack if they have not already been
        created. */
        if( xTasksAlreadyCreated == pdFALSE )
        {
            /*
             * Create Simple echo server tasks.
             */
	        vStartSimpleTCPServerTasks( mainECHO_SERVER_TASK_STACK_SIZE, mainECHO_SERVER_TASK_PRIORITY );
	        vStartSimpleUDPServerTask( mainECHO_SERVER_TASK_STACK_SIZE, mainUDP_ECHO_SERVER_PORT, mainECHO_SERVER_TASK_PRIORITY );

            xTasksAlreadyCreated = pdTRUE;
        }

        /* The network is up and configured.  Print out the configuration,
        which may have been obtained from a DHCP server. */
        FreeRTOS_GetAddressConfiguration( &ulIPAddress,
                                          &ulNetMask,
                                          &ulGatewayAddress,
                                          &ulDNSServerAddress );

        /* Convert the IP address to a string then print it out. */
        FreeRTOS_inet_ntoa( ulIPAddress, ( char * ) cBuffer );
        FreeRTOS_printf(( "IP Address: %s\n", cBuffer ));

        /* Convert the net mask to a string then print it out. */
        FreeRTOS_inet_ntoa( ulNetMask, ( char * ) cBuffer );
        FreeRTOS_printf(( "Subnet Mask: %s\n", cBuffer ));

        /* Convert the IP address of the gateway to a string then print it out. */
        FreeRTOS_inet_ntoa( ulGatewayAddress, ( char * ) cBuffer );
        FreeRTOS_printf(( "Gateway IP Address: %s\n", cBuffer ));

        /* Convert the IP address of the DNS server to a string then print it out. */
        FreeRTOS_inet_ntoa( ulDNSServerAddress, ( char * ) cBuffer );
        FreeRTOS_printf(( "DNS server IP Address: %s\n", cBuffer ));
    }
}

/* FreeRTOS stack overflow callback function */
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
   (void) pxTask;

   for (;;);
}

/**
 * @brief The main function
 *
 * Apart from the system/OS initialization, this function calls FreeRTOS_IPInit()
 * to create the FreeRTOS IP task and to initialize the stack. This must be the
 * the first freeRTOS-Plus-TCP function called.
 */
int main (void) {

    // System Initialization
    SystemCoreClockUpdate();

    /* Start the FreeRTOS IP task */
    FreeRTOS_IPInit(
        ucIPAddress,
        ucNetMask,
        ucGatewayAddress,
        ucDNSServerAddress,
        ucMACAddress );

    /* Start the FreeRTOS Scheduler */
    vTaskStartScheduler();

    /* Should never get here! */
    return 0;
}
