/*
 * Bluetooth emergency-stop prototype modifications:
 * Copyright (c) 2020 Pengyu Li.
 *
 * The original Texas Instruments SimpleLink SDK license below still applies.
 */

/******************************************************************************

 @file  simple_central.c

 @brief This file contains the Simple Central sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2013-2019, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/drivers/SPI.h>

#include <ti/display/Display.h>
#include <driverlib/aon_batmon.h>
#include "bcomdef.h"
#include <gap_initiator.h>
#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

//#include "board_key.h"
#include "board.h"

#include "ble_user_config.h"

#include "simple_gatt_profile.h"


#include "simple_central.h"

#include "gatt.h"
#include "GUA_Led.h"

#include "GUA_Key.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Application events
#define SC_EVT_KEY_CHANGE          0x01
#define SC_EVT_SCAN_ENABLED        0x02
#define SC_EVT_SCAN_DISABLED       0x03
#define SC_EVT_ADV_REPORT          0x04
#define SC_EVT_SVC_DISC            0x05
#define SC_EVT_READ_RSSI           0x06
#define SC_EVT_PAIR_STATE          0x07
#define SC_EVT_PASSCODE_NEEDED     0x08
#define SC_EVT_READ_RPA            0x09
#define SC_EVT_INSUFFICIENT_MEM    0x0A

//GUA
#define GATT_CLIENT_CFG_NOTIFY                  0x0001 //打开notify开关的数值
#define GATT_CLIENT_CFG_INDICATE                0x0002 //打开indicate开关的数值
//
#define GUA_CHAR4_Hdl                           0x29 //char4的句柄
#define GUA_CHAR4_CCC_Hdl                       0x27 //char4的CCC的句柄
//GUA



// Simple Central Task Events
#define SC_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define SC_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

#define SC_ALL_EVENTS                        (SC_ICALL_EVT           | \
                                              SC_QUEUE_EVT)

#define SBP_GUA_PERIODIC_EVT       Event_Id_00
#define MY_EVT_DISC_SVC            Event_Id_01
#define MY_EVT_OPEN_NOTIFY         Event_Id_02
#define MY_EVT_SEND_STATE          Event_Id_03
#define MY_EVT_DEBOND              Event_Id_04
#define MY_EVT_PHYUPDATE           Event_Id_05
#define MY_EVT_VOLTAGE             Event_Id_06
#define SBP_GUA_ALL_EVENTS         (SBP_GUA_PERIODIC_EVT |\
                                    MY_EVT_DISC_SVC |\
                                    MY_EVT_OPEN_NOTIFY |\
                                    MY_EVT_SEND_STATE|\
                                    MY_EVT_DEBOND|\
                                    MY_EVT_PHYUPDATE|\
                                    MY_EVT_VOLTAGE)




// Address mode of the local device
// Note: When using the DEFAULT_ADDRESS_MODE as ADDRMODE_RANDOM or 
// ADDRMODE_RP_WITH_RANDOM_ID, GAP_DeviceInit() should be called with 
// it's last parameter set to a static random address
#define DEFAULT_ADDRESS_MODE                 ADDRMODE_RP_WITH_PUBLIC_ID

// Default PHY for scanning and initiating
#define DEFAULT_SCAN_PHY                     SCAN_PRIM_PHY_CODED //SCAN_PRIM_PHY_1M
#define DEFAULT_INIT_PHY                     INIT_PHY_CODED      //INIT_PHY_1M      //

// Default scan duration in 10 ms
#define DEFAULT_SCAN_DURATION                100 // 1 sec

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                  3000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Supervision timeout conversion rate to miliseconds
#define CONN_TIMEOUT_MS_CONVERSION            10

// How often to read current current RPA (in ms)
#define SC_READ_RPA_PERIOD                    3000

 //Period
#define SBP_GUA_PERIODIC_EVT_PERIOD           500
#define MY_EVT_DISC_SVC_PERIOD                1000
#define MY_EVT_OPEN_NOTIFY_PERIOD             500
#define MY_EVT_SEND_STATE_PERIOD              500
#define MY_EVT_DEBOND_PERIOD                  1000
#define MY_EVT_PHYUPDATE_PERIOD               1000
#define MY_EVT_VOLTAGE_PERIOD                 2000



//uint8_t PeripheralMac1[6] = {0x98, 0x8D, 0x68, 0xCF, 0x61, 0x0C}; // 58 93 D8 48 26 F7
 //uint8_t PeripheralMac1[6] = {0xF7, 0x26, 0x48, 0xD8, 0x93, 0x58};
// uint8_t PeripheralMac1[6] = {0x1F, 0x41, 0x69, 0xED, 0x04, 0x18}; //guichuan
 uint8_t PeripheralMac1[6] = {0x58, 0x43, 0x69, 0xED, 0x04, 0x18}; //No.1
 //uint8_t PeripheralMac1[6] = {0x3F, 0x40, 0x69, 0xED, 0x04, 0x18}; //No.2

// Task configuration
#define SC_TASK_PRIORITY                     1

#ifndef SC_TASK_STACK_SIZE
#define SC_TASK_STACK_SIZE                   1024
#endif

// Advertising report fields to keep in the list
// Interested in only peer address type and peer address
#define SC_ADV_RPT_FIELDS   (SCAN_ADVRPT_FLD_ADDRTYPE | SCAN_ADVRPT_FLD_ADDRESS)

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define SC_ADDR_STR_SIZE     15

// Row numbers for two-button menu
#define SC_ROW_SEPARATOR     (TBM_ROW_APP + 0)
#define SC_ROW_CUR_CONN      (TBM_ROW_APP + 1)
#define SC_ROW_ANY_CONN      (TBM_ROW_APP + 2)
#define SC_ROW_NON_CONN      (TBM_ROW_APP + 3)
#define SC_ROW_NUM_CONN      (TBM_ROW_APP + 4)
#define SC_ROW_IDA           (TBM_ROW_APP + 5)
#define SC_ROW_RPA           (TBM_ROW_APP + 6)

//States
unsigned char Connected=0;
unsigned char Svc_Found=0;
unsigned char Scan=0;
unsigned char NOTIFY_OPEN_SUCCESS=0;
unsigned char State_Sent=0;
unsigned char Res_perip=0;
unsigned char PHYUPDATED=0;


// Spin if the expression is not true
#define SIMPLECENTRAL_ASSERT(expr) if (!(expr)) SimpleCentral_spin();

/*********************************************************************
 * TYPEDEFS
 */

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR,                 // Characteristic discovery
  BLE_DISC_STATE_CHAR1
};

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data
} scEvt_t;

// Scanned device information record
typedef struct
{
  uint8_t addrType;         // Peer Device's Address Type
  uint8_t addr[B_ADDR_LEN]; // Peer Device Address
} scanRec_t;

// Connected device information
typedef struct
{
  uint16_t connHandle;        // Connection Handle
  uint8_t  addr[B_ADDR_LEN];  // Peer Device Address
  uint8_t  charHandle;        // Characteristic Handle
  uint8_t  charHandle1;

  Clock_Struct *pRssiClock;   // pointer to clock struct
} connRec_t;

// Container to store paring state info when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint16_t connHandle;
  uint8_t  status;
} scPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t deviceAddr[B_ADDR_LEN];
  uint16_t connHandle;
  uint8_t uiInputs;
  uint8_t uiOutputs;
  uint32_t numComparison;
} scPasscodeData_t;
/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct scTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(scTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t scTaskStack[SC_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple Central";

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
// Number of scan results filtered by Service UUID
static uint8_t numScanRes = 0;

// Scan results filtered by Service UUID
static scanRec_t scanList[DEFAULT_MAX_SCAN_RES];
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

// Number of connected devices
static uint8_t numConn = 0;

// List of connections
static connRec_t connList[MAX_NUM_BLE_CONNS];

// Connection handle of current connection
static uint16_t scConnHandle = CONNHANDLE_INVALID;

// Accept or reject L2CAP connection parameter update request
static bool acceptParamUpdateReq = true;

// Discovery state
static uint8_t discState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Value to write
static uint8_t charVal = 0;

// Maximum PDU size (default = 27 octets)
static uint16_t scMaxPduSize;

// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;


// Build clock
static Clock_Struct GUA_periodicClock;
static Clock_Struct My_Disc_Svc_Clock;
static Clock_Struct My_Open_Notify_Clock;
static Clock_Struct My_Send_State_Clock;
static Clock_Struct My_Evt_Debond_Clock;
static Clock_Struct My_Evt_Phyupdate_Clock;
static Clock_Struct My_Evt_Voltage_Clock;
//Util_constructClock(&startDiscClock, SimpleBLECentral_startDiscHandler,
//                    DEFAULT_SVC_DISCOVERY_DELAY, 0, false, 0);
// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = {0};
#endif // PRIVACY_1_2_CFG

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void SimpleCentral_init(void);
static void SimpleCentral_taskFxn(uintptr_t a0, uintptr_t a1);

static void SimpleCentral_handleKeys(uint8_t keys);
static uint8_t SimpleCentral_processStackMsg(ICall_Hdr *pMsg);
static void SimpleCentral_processGapMsg(gapEventHdr_t *pMsg);
static void SimpleCentral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleCentral_processAppMsg(scEvt_t *pMsg);
static void SimpleCentral_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void SimpleCentral_startSvcDiscovery(void);
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
static bool SimpleCentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                      uint16_t dataLen);
static void SimpleCentral_addScanInfo(uint8_t *pAddr, uint8_t addrType);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID
static uint8_t SimpleCentral_addConnInfo(uint16_t connHandle, uint8_t *pAddr);
static uint8_t SimpleCentral_removeConnInfo(uint16_t connHandle);
static uint8_t SimpleCentral_getConnIndex(uint16_t connHandle);
#ifndef Display_DISABLE_ALL
static char* SimpleCentral_getConnAddrStr(uint16_t connHandle);
#endif
static void SimpleCentral_processPairState(uint8_t state,
                                           scPairStateData_t* pPairStateData);
static void SimpleCentral_processPasscode(scPasscodeData_t *pData);

static void SimpleCentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static status_t SimpleCentral_StartRssi();
static status_t SimpleCentral_CancelRssi(uint16_t connHandle);

static void SimpleCentral_passcodeCb(uint8_t *deviceAddr, uint16_t connHandle,
                                     uint8_t uiInputs, uint8_t uiOutputs,
                                     uint32_t numComparison);
static void SimpleCentral_pairStateCb(uint16_t connHandle, uint8_t state,
                                      uint8_t status);

static void SimpleCentral_keyChangeHandler(uint8 keys);
static void SimpleCentral_clockHandler(UArg arg);

static status_t SimpleCentral_enqueueMsg(uint8_t event, uint8_t status,
                                         uint8_t *pData);

static void SimpleCentral_scanCb(uint32_t evt, void* msg, uintptr_t arg);


// Period call back
static void GUA_performPeriodicTask(void);
static void SimpleBLECentral_GUAHandler(UArg a0);
static void GUA_HandleKeys(uint8 GUA_Keys);
//Discover service callback
static void My_Disc_Svc_Task(void);
static void My_Disc_Svc_Handler(UArg a0);

//Open notify call back
static void My_Open_Notify_Task(void);
static void My_Open_Notify_Handler(UArg a0);

// Send state
static void My_Send_State_Task(void);
static void My_Send_State_Handler(UArg a0);

static void My_Debond_Task(void);
static void My_Debond_Handler(UArg a0);

static void My_Phyupdate_Task(void);
static void My_Phyupdate_Handler(UArg a0);

static void My_Voltage_Task(void);
static void My_Voltage_Handler(UArg a0);

char numconnect=0;
unsigned char dataCount=0;
unsigned char OpenNoti=0;



uint8_t OpenNotify();
uint8_t Send_Connect_State();

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Bond Manager Callbacks
static gapBondCBs_t bondMgrCBs =
{
  SimpleCentral_passcodeCb, // Passcode callback
  SimpleCentral_pairStateCb // Pairing/Bonding state Callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleCentral_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void SimpleCentral_spin(void)
{
  volatile uint8_t x;

  while(1)
  {
    x++;
  }
}

/*********************************************************************
 * @fn      SimpleCentral_createTask
 *
 * @brief   Task creation function for the Simple Central.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleCentral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = scTaskStack;
  taskParams.stackSize = SC_TASK_STACK_SIZE;
  taskParams.priority = SC_TASK_PRIORITY;

  Task_construct(&scTask, SimpleCentral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleCentral_Init
 *
 * @brief   Initialization function for the Simple Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleCentral_init(void)
{
  uint8_t i;

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);
  IOCPortConfigureSet(IOID_6, IOC_PORT_RFC_GPO0, IOC_IOMODE_NORMAL);
  IOCPortConfigureSet(IOID_5, IOC_PORT_RFC_GPO1, IOC_IOMODE_NORMAL);
  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Initialize internal data
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    connList[i].connHandle = CONNHANDLE_INVALID;
    connList[i].pRssiClock = NULL;
  }

  //Board_initKeys(SimpleCentral_keyChangeHandler);

  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                   (void *)attDeviceName);

  //Set default values for Data Length Extension
  //Extended Data Length Feature is already enabled by default
  //in build_config.opt in stack project.
  {
    //Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

    //This API is documented in hci.h
    //See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
    //http://software-dl.ti.com/lprf/ble5stack-latest/
    //HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // Set Bond Manager parameters
  {
    // Don't send a pairing request after connecting; the device waits for the
    // application to start pairing
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    // Do not use authenticated pairing
    uint8_t mitm = FALSE;
    // This is a display only device
    uint8_t ioCap = GAPBOND_IO_CAP_KEYBOARD_ONLY;
    // Create a bond during the pairing process
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  // Start Bond Manager and register callback
  // This must be done before initialing the GAP layer
  VOID GAPBondMgr_Register(&bondMgrCBs);

  // Accept all parameter update requests
  GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, GAP_UPDATE_REQ_ACCEPT_ALL);

  // Register with GAP for HCI/Host messages (for RSSI)
  GAP_RegisterForMsgs(selfEntity);

  // Initialize GAP layer for Central role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_CENTRAL, selfEntity, addrMode, NULL);

  dispHandle = Display_open(Display_Type_ANY, NULL);

  // Disable all items in the main menu



// Init scan clock
  Util_constructClock(&GUA_periodicClock, SimpleBLECentral_GUAHandler,
   SBP_GUA_PERIODIC_EVT_PERIOD, 0, false,
  SBP_GUA_PERIODIC_EVT);
// Init Found service clock
  Util_constructClock(&My_Disc_Svc_Clock, My_Disc_Svc_Handler,
                      MY_EVT_DISC_SVC_PERIOD, 0, false, MY_EVT_DISC_SVC);

//Init Open Notify Clock
  Util_constructClock(&My_Open_Notify_Clock, My_Open_Notify_Handler,
                        MY_EVT_OPEN_NOTIFY_PERIOD, 0, false, MY_EVT_OPEN_NOTIFY);
  //
  Util_constructClock(&My_Send_State_Clock, My_Send_State_Handler,
                         MY_EVT_SEND_STATE_PERIOD, 0, false, MY_EVT_SEND_STATE);

  Util_constructClock(&My_Evt_Debond_Clock, My_Debond_Handler,
                         MY_EVT_DEBOND_PERIOD, 0, false, MY_EVT_DEBOND);
  Util_constructClock(&My_Evt_Phyupdate_Clock, My_Phyupdate_Handler,
                          MY_EVT_PHYUPDATE_PERIOD, 0, false, MY_EVT_PHYUPDATE);
  Util_constructClock(&My_Evt_Voltage_Clock, My_Voltage_Handler,
                            MY_EVT_VOLTAGE_PERIOD, 0, false, MY_EVT_VOLTAGE);

  HCI_LE_SetDefaultPhyCmd(LL_PHY_USE_ANY_PHY, LL_PHY_1_MBPS | LL_PHY_2_MBPS| HCI_PHY_CODED, LL_PHY_1_MBPS | LL_PHY_2_MBPS| HCI_PHY_CODED);
  HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_HIGH);
  HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);
  SimpleCentral_doSetScanPhy(1);
 // HCI_LE_SetPhyCmd(connList[0].connHandle,LL_PHY_USE_PHY_PARAM,HCI_PHY_CODED,HCI_PHY_CODED, LL_PHY_OPT_S8);

  Util_startClock(&GUA_periodicClock);
  GUA_initKeys(GUA_HandleKeys);
  //GUA_Led_Set(GUA_LED_NO_ALL, GUA_LED_MODE_OFF);
  GUA_Led_Set(GUA_LED_NO_1, GUA_LED_MODE_OFF);
  GUA_Led_Set(GUA_LED_NO_2, GUA_LED_MODE_OFF);
  GUA_Led_Set(GUA_LED_NO_3, GUA_LED_MODE_OFF);

  GUA_Led_Set(GUA_LED_NO_4, GUA_LED_MODE_OFF); //RELAY:PIN LOW

  GUA_Led_Set(GUA_LED_NO_1, GUA_LED_MODE_ON);

  AONBatMonEnable();
 // Util_startClock(&My_Evt_Voltage_Clock);
Scan=1;

}

/*********************************************************************
 * @fn      SimpleCentral_taskFxn
 *
 * @brief   Application task entry point for the Simple Central.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void SimpleCentral_taskFxn(uintptr_t a0, uintptr_t a1)
{
  // Initialize application
  SimpleCentral_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    events = Event_pend(syncEvent, Event_Id_NONE, SC_ALL_EVENTS | SBP_GUA_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = SimpleCentral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message
      if (events & SC_QUEUE_EVT)
      {
        scEvt_t *pMsg;
        while (pMsg = (scEvt_t *)Util_dequeueMsg(appMsgQueue))
        {
          // Process message
          SimpleCentral_processAppMsg(pMsg);

          // Free the space from the message
          ICall_free(pMsg);
        }
      }

   //MY EVENTS

      if (events & SBP_GUA_PERIODIC_EVT)
      {
          // Util_startClock(&GUA_periodicClock);
          GUA_performPeriodicTask();
      }

      if (events & MY_EVT_DISC_SVC)
      {
          My_Disc_Svc_Task();
      }
      if(events & MY_EVT_OPEN_NOTIFY)
      {
          My_Open_Notify_Task();
      }
      if(events & MY_EVT_SEND_STATE)
      {
          My_Send_State_Task();
      }

      if(events & MY_EVT_DEBOND)
            {
                My_Debond_Task();

            }
      if(events & MY_EVT_PHYUPDATE)
                  {
                      My_Phyupdate_Task();

                  }
      if(events & MY_EVT_VOLTAGE)
      {
          My_Voltage_Task();
      }
    }
  }
    }

/*********************************************************************
 * @fn      SimpleCentral_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleCentral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimpleCentral_processGapMsg((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      SimpleCentral_processGATTMsg((gattMsgEvent_t *)pMsg);

      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch (pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
          SimpleCentral_processCmdCompleteEvt((hciEvt_CmdComplete_t *) pMsg);
          break;

        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
          AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
          break;

        // HCI Commands Events
        case HCI_COMMAND_STATUS_EVENT_CODE:
          {
            hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;
            switch ( pMyMsg->cmdOpcode )
            {
              case HCI_LE_SET_PHY:
                {
                  if (pMyMsg->cmdStatus ==
                      HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
                  {

                  }
                  else
                  {

                  }
                }
                break;
              case HCI_DISCONNECT:
                break;

              default:
                {

                }
              break;
            }
          }
          break;

        // LE Events
        case HCI_LE_EVENT_CODE:
        {
          hciEvt_BLEPhyUpdateComplete_t *pPUC
            = (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

          if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)     //pyt update
          {
            if (pPUC->status != SUCCESS)
            {

            }
            else
            {

                PHYUPDATED=1;

            }
          }

          break;
        }

        default:
          break;
      }

      break;
    }

    case L2CAP_SIGNAL_EVENT:
      // place holder for L2CAP Connection Parameter Reply
      break;

    default:
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimpleCentral_processAppMsg
 *
 * @brief   Scanner application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void SimpleCentral_processAppMsg(scEvt_t *pMsg)
{
  bool safeToDealloc = TRUE;

  switch (pMsg->hdr.event)
  {
    case SC_EVT_KEY_CHANGE:
     // SimpleCentral_handleKeys(pMsg->hdr.state);
      break;

    case SC_EVT_ADV_REPORT:
    {
      GapScan_Evt_AdvRpt_t* pAdvRpt = (GapScan_Evt_AdvRpt_t*) (pMsg->pData);

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
      if (SimpleCentral_findSvcUuid(SIMPLEPROFILE_SERV_UUID,
                                    pAdvRpt->pData, pAdvRpt->dataLen))
      {
        SimpleCentral_addScanInfo(pAdvRpt->addr, pAdvRpt->addrType);

      }
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
      Display_printf(dispHandle, SC_ROW_NON_CONN, 0, "Discovered: %s",
                     Util_convertBdAddr2Str(pAdvRpt->addr));
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

      // Free report payload data
      if (pAdvRpt->pData != NULL)
      {
        ICall_free(pAdvRpt->pData);
      }
      break;
    }

    case SC_EVT_SCAN_ENABLED:
      // Disable everything but "Stop Discovering" on the menu
     // tbm_setItemStatus(&scMenuMain, SC_ITEM_STOPDISC,
                 //       (SC_ITEM_ALL & ~SC_ITEM_STOPDISC));

      break;

    case SC_EVT_SCAN_DISABLED:
    {
      uint8_t numReport;
      uint8_t i;
      static uint8_t* pAddrs = NULL;
      uint8_t* pAddrTemp;

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
      numReport = numScanRes;
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
      GapScan_Evt_AdvRpt_t advRpt;

      numReport = ((GapScan_Evt_End_t*) (pMsg->pData))->numReport;
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID



      if (numReport > 0)
      {
        // Also enable "Connect to"

      }

      if (numConn > 0)
      {
        // Also enable "Work with"

      }

      // Enable "Discover Devices", "Set Scanning PHY", and possibly
      // "Connect to" and/or "Work with".
      // Disable "Stop Discovering".
      //tbm_setItemStatus(&scMenuMain, itemsToEnable, SC_ITEM_STOPDISC);

      // Allocate buffer to display addresses
      if (pAddrs != NULL)
      {
        // A scan has been done previously, release the previously allocated buffer
        ICall_free(pAddrs);
      }
      pAddrs = ICall_malloc(numReport * SC_ADDR_STR_SIZE);
      if (pAddrs == NULL)
      {
        numReport = 0;
      }



      if (pAddrs != NULL)
      {
        pAddrTemp = pAddrs;
        for (i = 0; i < numReport; i++, pAddrTemp += SC_ADDR_STR_SIZE)
        {
  #if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
          // Get the address from the list, convert it to string, and
          // copy the string to the address buffer
          memcpy(pAddrTemp, Util_convertBdAddr2Str(scanList[i].addr),
                 SC_ADDR_STR_SIZE);
  #else // !DEFAULT_DEV_DISC_BY_SVC_UUID
          // Get the address from the report, convert it to string, and
          // copy the string to the address buffer
          GapScan_getAdvReport(i, &advRpt);
          memcpy(pAddrTemp, Util_convertBdAddr2Str(advRpt.addr),
                 SC_ADDR_STR_SIZE);
  #endif // DEFAULT_DEV_DISC_BY_SVC_UUID

          // Assign the string to the corresponding action description of the menu

        }

        // Disable any non-active scan results
        for (; i < DEFAULT_MAX_SCAN_RES; i++)
        {

        }

        // Note: pAddrs is not freed since it will be used by the two button menu
        // to display the discovered address.
        // This implies that at least the last discovered addresses
        // will be maintained until a new scan is done.
      }
      break;
    }

    case SC_EVT_SVC_DISC:
      SimpleCentral_startSvcDiscovery();

      break;

    case SC_EVT_READ_RSSI:
    {
      uint8_t connIndex = pMsg->hdr.state;
      uint16_t connHandle = connList[connIndex].connHandle;

      // If link is still valid
      if (connHandle != CONNHANDLE_INVALID)
      {
        // Restart timer
        Util_startClock(connList[connIndex].pRssiClock);

        // Read RSSI
        VOID HCI_ReadRssiCmd(connHandle);
      }

      break;
    }

    // Pairing event
    case SC_EVT_PAIR_STATE:
    {
      SimpleCentral_processPairState(pMsg->hdr.state,
                                     (scPairStateData_t*) (pMsg->pData));
      break;
    }

    // Passcode event
    case SC_EVT_PASSCODE_NEEDED:
    {
      SimpleCentral_processPasscode((scPasscodeData_t *)(pMsg->pData));
      break;
    }

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
    case SC_EVT_READ_RPA:
    {
      uint8_t* pRpaNew;

      // Read the current RPA.
      pRpaNew = GAP_GetDevAddress(FALSE);

      if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
      {
        // If the RPA has changed, update the display

        memcpy(rpa, pRpaNew, B_ADDR_LEN);
      }
      break;
    }
#endif // PRIVACY_1_2_CFG

    // Insufficient memory
    case SC_EVT_INSUFFICIENT_MEM:
    {
      // We are running out of memory.


      // We might be in the middle of scanning, try stopping it.
      GapScan_disable();
      break;
    }

    default:
      // Do nothing.
      break;
  }

  if ((safeToDealloc == TRUE) && (pMsg->pData != NULL))
  {
    ICall_free(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      SimpleCentral_processGapMsg
 *
 * @brief   GAP message processing function.
 *
 * @param   pMsg - pointer to event message structure
 *
 * @return  none
 */
static void SimpleCentral_processGapMsg(gapEventHdr_t *pMsg)
{
  switch (pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      uint8_t temp8;
      uint16_t temp16;
      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

      // Setup scanning
      // For more information, see the GAP section in the User's Guide:
      // http://software-dl.ti.com/lprf/ble5stack-latest/

      // Register callback to process Scanner events
      GapScan_registerCb(SimpleCentral_scanCb, NULL);

      // Set Scanner Event Mask
      GapScan_setEventMask(GAP_EVT_SCAN_ENABLED | GAP_EVT_SCAN_DISABLED |
                           GAP_EVT_ADV_REPORT);

      // Set Scan PHY parameters
      GapScan_setPhyParams(DEFAULT_SCAN_PHY, SCAN_TYPE_PASSIVE,
                           SCAN_PARAM_DFLT_INTERVAL, SCAN_PARAM_DFLT_INTERVAL);

      // Set Advertising report fields to keep
      temp16 = SC_ADV_RPT_FIELDS;
      GapScan_setParam(SCAN_PARAM_RPT_FIELDS, &temp16);
      // Set Scanning Primary PHY
      temp8 = DEFAULT_SCAN_PHY;
      GapScan_setParam(SCAN_PARAM_PRIM_PHYS, &temp8);
      // Set LL Duplicate Filter
      temp8 = SCAN_FLT_DUP_ENABLE;
      GapScan_setParam(SCAN_PARAM_FLT_DUP, &temp8);

      // Set PDU type filter -
      // Only 'Connectable' and 'Complete' packets are desired.
      // It doesn't matter if received packets are
      // whether Scannable or Non-Scannable, whether Directed or Undirected,
      // whether Scan_Rsp's or Advertisements, and whether Legacy or Extended.
      temp16 = SCAN_FLT_PDU_CONNECTABLE_ONLY | SCAN_FLT_PDU_COMPLETE_ONLY;
      GapScan_setParam(SCAN_PARAM_FLT_PDU_TYPE, &temp16);

      scMaxPduSize = pPkt->dataPktLen;

      // Enable "Discover Devices", "Set Scanning PHY", and "Set Address Type"
      // in the main menu



      // Display device address

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
      if (addrMode > ADDRMODE_RANDOM)
      {
        // Update the current RPA.
        memcpy(rpa, GAP_GetDevAddress(FALSE), B_ADDR_LEN);


        // Create one-shot clock for RPA check event.
        Util_constructClock(&clkRpaRead, SimpleCentral_clockHandler,
                            SC_READ_RPA_PERIOD, 0, true, SC_EVT_READ_RPA);
      }
#endif // PRIVACY_1_2_CFG
      break;
    }

    case GAP_CONNECTING_CANCELLED_EVENT:
    {


      if (numConn > 0)
      {

      }



      // Enable "Discover Devices", "Connect To", and "Set Scanning PHY"
      // and disable everything else.


      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      uint16_t connHandle = ((gapEstLinkReqEvent_t*) pMsg)->connectionHandle;
      uint8_t* pAddr = ((gapEstLinkReqEvent_t*) pMsg)->devAddr;
      uint8_t  connIndex;

      uint8_t* pStrAddr;
      uint8_t i;
      uint8_t numConnectable = 0;

      // Add this connection info to the list
      connIndex = SimpleCentral_addConnInfo(connHandle, pAddr);

      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      SIMPLECENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      connList[connIndex].charHandle = 0;


      pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);

      //SimpleCentral_enqueueMsg(SC_EVT_SVC_DISC, 0, NULL);


Connected=1;
Util_stopClock(&GUA_periodicClock);

Util_startClock(&My_Disc_Svc_Clock);        //start to discover service


      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      uint16_t connHandle = ((gapTerminateLinkEvent_t*) pMsg)->connectionHandle;
      uint8_t connIndex;

      uint8_t* pStrAddr;
      uint8_t i;
      uint8_t numConnectable = 0;
Connected=0;
Scan=1;
Svc_Found=0;
NOTIFY_OPEN_SUCCESS=0;
State_Sent=0;

GUA_Led_Set(GUA_LED_NO_2, GUA_LED_MODE_OFF);
//restart scan
Util_startClock(&GUA_periodicClock);



      // Cancel timers
      SimpleCentral_CancelRssi(connHandle);

      // Mark this connection deleted in the connected device list.
      connIndex = SimpleCentral_removeConnInfo(connHandle);

      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      SIMPLECENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);






      break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
      gapUpdateLinkParamReqReply_t rsp;
      gapUpdateLinkParamReq_t *pReq;

      pReq = &((gapUpdateLinkParamReqEvent_t *)pMsg)->req;

      rsp.connectionHandle = pReq->connectionHandle;
      rsp.signalIdentifier = pReq->signalIdentifier;

      if (acceptParamUpdateReq)
      {
        rsp.intervalMin = pReq->intervalMin;
        rsp.intervalMax = pReq->intervalMax;
        rsp.connLatency = pReq->connLatency;
        rsp.connTimeout = pReq->connTimeout;
        rsp.accepted = TRUE;
      }
      else
      {
        // Reject the request.
        rsp.accepted = FALSE;
      }

      // Send Reply
      VOID GAP_UpdateLinkParamReqReply(&rsp);

      break;
    }

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;
      // Get the address from the connection handle
      linkDBInfo_t linkInfo;

      if (linkDB_GetInfo(pPkt->connectionHandle, &linkInfo) ==  SUCCESS)
      {
        if(pPkt->status == SUCCESS)
        {

        }
        else
        {
          // Display the address of the connection update failure

        }
      }

      break;
    }

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleCentral_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_LEFT
 *                 KEY_RIGHT
 *
 * @return  none
 */

/*********************************************************************
 * @fn      SimpleCentral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  none
 */
static void SimpleCentral_processGATTMsg(gattMsgEvent_t *pMsg)
{

  if (linkDB_Up(pMsg->connHandle))
  {

    // See if GATT server was unable to transmit an ATT response
    if (pMsg->hdr.status == blePending)
    {
      // No HCI buffer was available. App can try to retransmit the response
      // on the next connection event. Drop it for now.

    }
    else if ((pMsg->method == ATT_READ_RSP)   ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {

      }
      else
      {
        // After a successful read, display the read value

      }
    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {

      }
      else
      {
        // After a successful write, display the value that was written and
        // increment value

      }


    }
    else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
      // ATT request-response or indication-confirmation flow control is
      // violated. All subsequent ATT requests or indications will be dropped.
      // The app is informed in case it wants to drop the connection.

      // Display the opcode of the message that caused the violation.

    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
      // MTU size updated

    }
    else if (discState != BLE_DISC_STATE_IDLE)
    {
      SimpleCentral_processGATTDiscEvent(pMsg);

    }



    else if ( ( pMsg->method == ATT_HANDLE_VALUE_NOTI ) )   //通知
        {

        uint8_t connIndex = SimpleCentral_getConnIndex(scConnHandle);
          if( pMsg->msg.handleValueNoti.handle == connList[connIndex].charHandle)     //CHAR6的通知  串口打印
          {

              char str[32] = {0};
              memcpy(str, pMsg->msg.handleValueNoti.pValue, pMsg->msg.handleValueNoti.len );
            if(str[0]=='s' && str[1]=='p')
            {
                GUA_Led_Set(GUA_LED_NO_3, GUA_LED_MODE_ON);
                GUA_Led_Set(GUA_LED_NO_4, GUA_LED_MODE_ON); //PIN HIGH
            }
            else if(str[0]=='r' && str[1]=='s')
            {
                GUA_Led_Set(GUA_LED_NO_3, GUA_LED_MODE_OFF);
                GUA_Led_Set(GUA_LED_NO_4, GUA_LED_MODE_OFF); // PIN LOW
            }
            else if(str[0]==0x05)
            {
                GUA_Led_Set(GUA_LED_NO_2, GUA_LED_MODE_ON);
                Res_perip=1;
                Util_startClock(&My_Evt_Phyupdate_Clock);
                //Util_startClock(&GUA_periodicClock);
            }
          }
        }


  } // else - in case a GATT message came after a connection has dropped, ignore it.


  // Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      SimpleCentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleCentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
    {
#ifndef Display_DISABLE_ALL
      uint16_t connHandle = BUILD_UINT16(pMsg->pReturnParam[1],
                                         pMsg->pReturnParam[2]);
      int8 rssi = (int8)pMsg->pReturnParam[3];
      


#endif
      break;
    }

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleCentral_StartRssi
 *
 * @brief   Start periodic RSSI reads on the current link.
 *
 * @return  SUCCESS: RSSI Read timer started
 *          bleIncorrectMode: Aready started
 *          bleNoResources: No resources
 */


/*********************************************************************
 * @fn      SimpleCentral_CancelRssi
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connection handle
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: Has not started
 */
static status_t SimpleCentral_CancelRssi(uint16_t connHandle)
{
  uint8_t connIndex = SimpleCentral_getConnIndex(connHandle);

  // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
  SIMPLECENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // If already running
  if (connList[connIndex].pRssiClock == NULL)
  {
    return bleIncorrectMode;
  }

  // Stop timer
  Util_stopClock(connList[connIndex].pRssiClock);

  // Destroy the clock object
  Clock_destruct(connList[connIndex].pRssiClock);

  // Free clock struct
  ICall_free(connList[connIndex].pRssiClock);
  connList[connIndex].pRssiClock = NULL;


  return SUCCESS;
}

/*********************************************************************
 * @fn      SimpleCentral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimpleCentral_processPairState(uint8_t state,
                                           scPairStateData_t* pPairData)
{
  uint8_t status = pPairData->status;

  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {

  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      linkDBInfo_t linkInfo;




#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
      if (linkDB_GetInfo(pPairData->connHandle, &linkInfo) == SUCCESS)
      {
        // If the peer was using private address, update with ID address
        if ((linkInfo.addrType == ADDRTYPE_PUBLIC_ID ||
             linkInfo.addrType == ADDRTYPE_RANDOM_ID) &&
             !Util_isBufSet(linkInfo.addrPriv, 0, B_ADDR_LEN))
        {
          // Update the address of the peer to the ID address

          // Update the connection list with the ID address
          uint8_t i = SimpleCentral_getConnIndex(pPairData->connHandle);

          SIMPLECENTRAL_ASSERT(i < MAX_NUM_BLE_CONNS);
          memcpy(connList[i].addr, linkInfo.addr, B_ADDR_LEN);
        }
      }
#endif // PRIVACY_1_2_CFG
    }
    else
    {

    }
  }
  else if (state == GAPBOND_PAIRING_STATE_ENCRYPTED)
  {
    if (status == SUCCESS)
    {

    }
    else
    {

    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (status == SUCCESS)
    {


    }
    else
    {

    }
  }
}

/*********************************************************************
 * @fn      SimpleCentral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimpleCentral_processPasscode(scPasscodeData_t *pData)
{

  // Display passcode to user
  if (pData->uiOutputs != 0)
  {

  }

  // Send passcode response
    GAPBondMgr_PasscodeRsp(pData->connHandle, SUCCESS, B_APP_DEFAULT_PASSCODE);
    //GAPBondMgr_PasscodeRsp(pData->connHandle, SUCCESS,test_code);
}

/*********************************************************************
 * @fn      SimpleCentral_startSvcDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void SimpleCentral_startSvcDiscovery(void)
{
  attExchangeMTUReq_t req;

  // Initialize cached handles
  svcStartHdl = svcEndHdl = 0;

  discState = BLE_DISC_STATE_MTU;

  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = scMaxPduSize - L2CAP_HDR_SIZE;

  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(scConnHandle, &req, selfEntity);

}

/*********************************************************************
 * @fn      SimpleCentral_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void SimpleCentral_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
  if (discState == BLE_DISC_STATE_MTU)
  {


    // MTU size response received, discover simple service
    if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
    {

      uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                         HI_UINT16(SIMPLEPROFILE_SERV_UUID) };

      discState = BLE_DISC_STATE_SVC;

      // Discovery simple service
      VOID GATT_DiscPrimaryServiceByUUID(pMsg->connHandle, uuid,
                                         ATT_BT_UUID_SIZE, selfEntity);
    }
  }
  else if (discState == BLE_DISC_STATE_SVC)
  {

    // Service found, store handles
    if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
        pMsg->msg.findByTypeValueRsp.numInfo > 0)
    {
      svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
    }

    // If procedure complete
    if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) &&
         (pMsg->hdr.status == bleProcedureComplete))  ||
        (pMsg->method == ATT_ERROR_RSP))
    {
      if (svcStartHdl != 0)
      {
        attReadByTypeReq_t req;

        // Discover characteristic
        discState = BLE_DISC_STATE_CHAR;

        req.startHandle = svcStartHdl;
        req.endHandle = svcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
            req.type.uuid[0]= LO_UINT16(SIMPLEPROFILE_CHAR4_UUID);
             req.type.uuid[1]=HI_UINT16(SIMPLEPROFILE_CHAR4_UUID);
        //req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
      //  req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

        VOID GATT_DiscCharsByUUID(pMsg->connHandle, &req, selfEntity);


      }
    }
  }
  else if (discState == BLE_DISC_STATE_CHAR)
  {
    // Characteristic found, store handle
    if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
        (pMsg->msg.readByTypeRsp.numPairs > 0))
    {
      uint8_t connIndex = SimpleCentral_getConnIndex(scConnHandle);

      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      SIMPLECENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      // Store the handle of the simpleprofile characteristic 1 value
      connList[connIndex].charHandle
        = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[3],
                       pMsg->msg.readByTypeRsp.pDataList[4]);
    }
else
    {
    attReadByTypeReq_t req;
        discState = BLE_DISC_STATE_CHAR1;
        req.startHandle = svcStartHdl;
              req.endHandle = svcEndHdl;
              req.type.len = ATT_BT_UUID_SIZE;
                  req.type.uuid[0]= LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
                   req.type.uuid[1]=HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);
                   VOID GATT_DiscCharsByUUID(pMsg->connHandle, &req, selfEntity);
    }
  }

  else if (discState == BLE_DISC_STATE_CHAR1)
  {

      if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
             (pMsg->msg.readByTypeRsp.numPairs > 0))
         {
           uint8_t connIndex = SimpleCentral_getConnIndex(scConnHandle);

           // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
           SIMPLECENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

           // Store the handle of the simpleprofile characteristic 1 value
           connList[connIndex].charHandle1
             = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[3],
                            pMsg->msg.readByTypeRsp.pDataList[4]);
         }


Svc_Found=1;
Util_stopClock(&My_Disc_Svc_Clock);
Util_startClock(&My_Open_Notify_Clock);

discState = BLE_DISC_STATE_IDLE;
  }
// Now we can use GATT Read/Write


}



#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
/*********************************************************************
 * @fn      SimpleCentral_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool SimpleCentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                      uint16_t dataLen)
{
  uint8_t adLen;
  uint8_t adType;
  uint8_t *pEnd;

  if (dataLen > 0)
  {
    pEnd = pData + dataLen - 1;

    // While end of data not reached
    while (pData < pEnd)
    {
      // Get length of next AD item
      adLen = *pData++;
      if (adLen > 0)
      {
        adType = *pData;

        // If AD type is for 16-bit service UUID
        if ((adType == GAP_ADTYPE_16BIT_MORE) ||
            (adType == GAP_ADTYPE_16BIT_COMPLETE))
        {
          pData++;
          adLen--;

          // For each UUID in list
          while (adLen >= 2 && pData < pEnd)
          {
            // Check for match
            if ((pData[0] == LO_UINT16(uuid)) && (pData[1] == HI_UINT16(uuid)))
            {
              // Match found
              return TRUE;
            }

            // Go to next
            pData += 2;
            adLen -= 2;
          }

          // Handle possible erroneous extra byte in UUID list
          if (adLen == 1)
          {
            pData++;
          }
        }
        else
        {
          // Go to next item
          pData += adLen;
        }
      }
    }
  }

  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      SimpleCentral_addScanInfo
 *
 * @brief   Add a device to the scanned device list
 *
 * @return  none
 */
static void SimpleCentral_addScanInfo(uint8_t *pAddr, uint8_t addrType)
{
  uint8_t i;

  // If result count not at max
  if (numScanRes < DEFAULT_MAX_SCAN_RES)
  {
    // Check if device is already in scan results
    for (i = 0; i < numScanRes; i++)
    {
      if (memcmp(pAddr, scanList[i].addr , B_ADDR_LEN) == 0)
      {
        return;
      }
    }

    // Add addr to scan result list
    memcpy(scanList[numScanRes].addr, pAddr, B_ADDR_LEN);
    scanList[numScanRes].addrType = addrType;

    // Increment scan result count
    numScanRes++;
  }
}
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

/*********************************************************************
 * @fn      SimpleCentral_addConnInfo
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimpleCentral_addConnInfo(uint16_t connHandle, uint8_t *pAddr)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      connList[i].connHandle = connHandle;
      memcpy(connList[i].addr, pAddr, B_ADDR_LEN);
      numConn++;

      break;
    }
  }

  return i;
}

/*********************************************************************
 * @fn      SimpleCentral_removeConnInfo
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimpleCentral_removeConnInfo(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      // Found the entry to mark as deleted
      connList[i].connHandle = CONNHANDLE_INVALID;
      numConn--;

      break;
    }
  }

  return i;
}

/*********************************************************************
 * @fn      SimpleCentral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimpleCentral_getConnIndex(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      break;
    }
  }

  return i;
}

#ifndef Display_DISABLE_ALL
/*********************************************************************
 * @fn      SimpleCentral_getConnAddrStr
 *
 * @brief   Return, in string form, the address of the peer associated with
 *          the connHandle.
 *
 * @return  A null-terminated string of the address.
 *          if there is no match, NULL will be returned.
 */
static char* SimpleCentral_getConnAddrStr(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      return Util_convertBdAddr2Str(connList[i].addr);
    }
  }

  return NULL;
}
#endif

/*********************************************************************
 * @fn      SimpleCentral_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimpleCentral_pairStateCb(uint16_t connHandle, uint8_t state,
                                      uint8_t status)
{
  scPairStateData_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(scPairStateData_t))))
  {
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    if(SimpleCentral_enqueueMsg(SC_EVT_PAIR_STATE, state, (uint8_t*) pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
* @fn      SimpleCentral_passcodeCb
*
* @brief   Passcode callback.
*
* @param   deviceAddr - pointer to device address
*
* @param   connHandle - the connection handle
*
* @param   uiInputs - pairing User Interface Inputs
*
* @param   uiOutputs - pairing User Interface Outputs
*
* @param   numComparison - numeric Comparison 20 bits
*
* @return  none
*/
static void SimpleCentral_passcodeCb(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs,
                                  uint32_t numComparison)
{
  scPasscodeData_t *pData = ICall_malloc(sizeof(scPasscodeData_t));

  // Allocate space for the passcode event.
  if (pData)
  {
    pData->connHandle = connHandle;
    memcpy(pData->deviceAddr, deviceAddr, B_ADDR_LEN);
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    if (SimpleCentral_enqueueMsg(SC_EVT_PASSCODE_NEEDED, 0,(uint8_t *) pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimpleCentral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */


/*********************************************************************
 * @fn      SimpleCentral_clockHandler
 *
 * @brief   clock handler function
 *
 * @param   arg - argument from the clock initiator
 *
 * @return  none
 */
void SimpleCentral_clockHandler(UArg arg)
{
  uint8_t evtId = (uint8_t) (arg & 0xFF);

  switch (evtId)
  {
    case SC_EVT_READ_RSSI:
      SimpleCentral_enqueueMsg(SC_EVT_READ_RSSI, (uint8_t) (arg >> 8) , NULL);
      break;

    case SC_EVT_READ_RPA:
      // Restart timer
      Util_startClock(&clkRpaRead);
      // Let the application handle the event
      SimpleCentral_enqueueMsg(SC_EVT_READ_RPA, 0, NULL);
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleCentral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static status_t SimpleCentral_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  uint8_t success;
  scEvt_t *pMsg = ICall_malloc(sizeof(scEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    success = Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
    return (success) ? SUCCESS : FAILURE;
  }

  return(bleMemAllocError);
}

/*********************************************************************
 * @fn      SimpleCentral_scanCb
 *
 * @brief   Callback called by GapScan module
 *
 * @param   evt - event
 * @param   msg - message coming with the event
 * @param   arg - user argument
 *
 * @return  none
 */
void SimpleCentral_scanCb(uint32_t evt, void* pMsg, uintptr_t arg)
{
  uint8_t event;

  if (evt & GAP_EVT_ADV_REPORT)
  {
    event = SC_EVT_ADV_REPORT;
  }
  else if (evt & GAP_EVT_SCAN_ENABLED)
  {
    event = SC_EVT_SCAN_ENABLED;
  }
  else if (evt & GAP_EVT_SCAN_DISABLED)
  {
    event = SC_EVT_SCAN_DISABLED;
  }
  else if (evt & GAP_EVT_INSUFFICIENT_MEMORY)
  {
    event = SC_EVT_INSUFFICIENT_MEM;
  }
  else
  {
    return;
  }

  if(SimpleCentral_enqueueMsg(event, SUCCESS, pMsg) != SUCCESS)
  {
    ICall_free(pMsg);
  }
}

/*********************************************************************
 * @fn      SimpleCentral_doSetScanPhy
 *
 * @brief   Set PHYs for scanning.
 *
 * @param   index - 0: 1M PHY
 *                  1: CODED PHY (Long range)
 *
 * @return  always true
 */
bool SimpleCentral_doSetScanPhy(uint8_t index)
{
  uint8_t temp8;

  if (index == 0)
  {
    temp8 = SCAN_PRIM_PHY_1M;
  }
  else
  {
    temp8 = SCAN_PRIM_PHY_CODED;
  }

  // Set scanning primary PHY
  GapScan_setParam(SCAN_PARAM_PRIM_PHYS, &temp8);



  return (true);
}

/*********************************************************************
 * @fn      SimpleCentral_doDiscoverDevices
 *
 * @brief   Enables scanning
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool SimpleCentral_doDiscoverDevices(uint8_t index)
{
  (void) index;

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
  // Scanning for DEFAULT_SCAN_DURATION x 10 ms.
  // The stack does not need to record advertising reports
  // since the application will filter them by Service UUID and save.

  // Reset number of scan results to 0 before starting scan
  numScanRes = 0;
  GapScan_enable(0, DEFAULT_SCAN_DURATION, 0);
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
  // Scanning for DEFAULT_SCAN_DURATION x 10 ms.
  // Let the stack record the advertising reports as many as up to DEFAULT_MAX_SCAN_RES.
  GapScan_enable(0, DEFAULT_SCAN_DURATION, DEFAULT_MAX_SCAN_RES);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

  // Enable only "Stop Discovering" and disable all others in the main menu


  return (true);
}

/*********************************************************************
 * @fn      SimpleCentral_doStopDiscovering
 *
 * @brief   Stop on-going scanning
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool SimpleCentral_doStopDiscovering(uint8_t index)
{
  (void) index;

  GapScan_disable();

  return (true);
}

/*********************************************************************
 * @fn      SimpleCentral_doEstablishLink
 *
 * @brief   Establish a link to a peer device
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */



bool SimpleCentral_doConnect(uint8_t index)
{
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)

    //GapInit_InitPhy_t initPhy = INIT_PHY_CODED;
  if(GapInit_connect(scanList[index].addrType & MASK_ADDRTYPE_ID,
                  scanList[index].addr, DEFAULT_INIT_PHY, 0)==SUCCESS)
  {


  }
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
  GapScan_Evt_AdvRpt_t advRpt;

  GapScan_getAdvReport(index, &advRpt);k

  GapInit_connect(advRpt.addrType & MASK_ADDRTYPE_ID,
                  advRpt.addr, DEFAULT_INIT_PHY, 0);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

  // Enable only "Cancel Connecting" and disable all others in the main menu
  //tbm_setItemStatus(&scMenuMain, SC_ITEM_CANCELCONN,
    //                (SC_ITEM_ALL & ~SC_ITEM_CANCELCONN));



 // tbm_goTo(&scMenuMain);

  return (true);
}

/*********************************************************************
 * @fn      SimpleCentral_doCancelConnecting
 *
 * @brief   Cancel on-going connection attempt
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool SimpleCentral_doCancelConnecting(uint8_t index)
{
  (void) index;

  GapInit_cancelConnect();

  return (true);
}

/*********************************************************************
 * @fn      SimpleCentral_doSelectConn
 *
 * @brief   Select a connection to communicate with
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool SimpleCentral_doSelectConn(uint8_t index)
{


  // index cannot be equal to or greater than MAX_NUM_BLE_CONNS
  SIMPLECENTRAL_ASSERT(index < MAX_NUM_BLE_CONNS);

  scConnHandle = connList[index].connHandle;

  if (connList[index].charHandle == 0)
  {
    // Initiate service discovery
    SimpleCentral_enqueueMsg(SC_EVT_SVC_DISC, 0, NULL);

    // Diable GATT Read/Write until simple service is found

  }

  // Set the menu title and go to this connection's context


  // Set RSSI items properly depending on current state
  if (connList[index].pRssiClock == NULL)
  {

  }
  else
  {

  }

  // Clear non-connection-related message


  return (true);
}

/*********************************************************************
 * @fn      SimpleCentral_doGattRead
 *
 * @brief   GATT Read
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool SimpleCentral_doGattRead(uint8_t index)
{
  attReadReq_t req;
  uint8_t connIndex = SimpleCentral_getConnIndex(scConnHandle);

  // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
  SIMPLECENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  req.handle = connList[connIndex].charHandle;
  GATT_ReadCharValue(scConnHandle, &req, selfEntity);

  return (true);
}

/*********************************************************************
 * @fn      SimpleCentral_doGattWrite
 *
 * @brief   GATT Write
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool SimpleCentral_doGattWrite(uint8_t index)
{
  status_t status;
  uint8_t charVals[4] = { 0x00, 0x55, 0xAA, 0xFF }; // Should be consistent with
                                                    // those in scMenuGattWrite

  attWriteReq_t req;

  req.pValue = GATT_bm_alloc(scConnHandle, ATT_WRITE_REQ, 1, NULL);

  if ( req.pValue != NULL )
  {
    uint8_t connIndex = SimpleCentral_getConnIndex(scConnHandle);

    // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
    SIMPLECENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    req.handle = connList[connIndex].charHandle;
    req.len = 1;
    charVal = charVals[index];
    req.pValue[0] = charVal;
    req.sig = 0;
    req.cmd = 0;

    status = GATT_WriteCharValue(scConnHandle, &req, selfEntity);
    if ( status != SUCCESS )
    {
      GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
    }
  }

  return (true);
}


//bool SimpleCentral_opennoti(uint8_t index)
//{
//  status_t status;
//  uint8_t charVals[2] = { LO_UINT16(GATT_CLIENT_CFG_NOTIFY), HI_UINT16(GATT_CLIENT_CFG_NOTIFY)}; // Should be consistent with
//                                                    // those in scMenuGattWrite
//
//  attWriteReq_t req;
//
//  req.pValue = GATT_bm_alloc(scConnHandle, ATT_WRITE_REQ, 1, NULL);
//
//  if ( req.pValue != NULL )
//  {
//    uint8_t connIndex = SimpleCentral_getConnIndex(scConnHandle);
//
//    // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
//    SIMPLECENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);
//
//    req.handle = GUA_CHAR4_CCC_Hdl;
//    req.len = 2;
//    charVal = charVals[index];
//    req.pValue[0] = charVal;
//    req.sig = 0;
//    req.cmd = 0;
//
//    status = GATT_WriteCharValue(scConnHandle, &req, selfEntity);
//    if ( status != SUCCESS )
//    {
//      GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
//    }
//  }
//
//  return (true);
//}

/*********************************************************************
 * @fn      SimpleCentral_doRssiRead
 *
 * @brief   Toggle RSSI Read
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */


/*********************************************************************
 * @fn      SimpleCentral_doConnUpdate
 *
 * @brief   Initiate Connection Update procedure
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool SimpleCentral_doConnUpdate(uint8_t index)
{
  gapUpdateLinkParamReq_t params;

  (void) index;

  params.connectionHandle = scConnHandle;
  params.intervalMin = DEFAULT_UPDATE_MIN_CONN_INTERVAL;
  params.intervalMax = DEFAULT_UPDATE_MAX_CONN_INTERVAL;
  params.connLatency = DEFAULT_UPDATE_SLAVE_LATENCY;

  linkDBInfo_t linkInfo;
  if (linkDB_GetInfo(scConnHandle, &linkInfo) == SUCCESS)
  {
    if (linkInfo.connTimeout == DEFAULT_UPDATE_CONN_TIMEOUT)
    {
      params.connTimeout = DEFAULT_UPDATE_CONN_TIMEOUT + 200;
    }
    else
    {
      params.connTimeout = DEFAULT_UPDATE_CONN_TIMEOUT;
    }
  }
  else
  {

  }
  GAP_UpdateLinkParamReq(&params);



  return (true);
}

/*********************************************************************
 * @fn      SimpleCentral_doSetConnPhy
 *
 * @brief   Set Connection PHY preference.
 *
 * @param   index - 0: 1M PHY
 *                  1: 2M PHY
 *                  2: 1M + 2M PHY
 *                  3: CODED PHY (Long range)
 *                  4: 1M + 2M + CODED PHY
 *
 * @return  always true
 */
bool SimpleCentral_doSetConnPhy(uint8_t index)
{
  static uint8_t phy[] = {
    HCI_PHY_1_MBPS, HCI_PHY_2_MBPS, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS,
    HCI_PHY_CODED, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS | HCI_PHY_CODED,
  };

  // Set Phy Preference on the current connection. Apply the same value
  // for RX and TX. For more information, see the LE 2M PHY section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  // Note PHYs are already enabled by default in build_config.opt in stack project.
  HCI_LE_SetPhyCmd(scConnHandle, 0, phy[index], phy[index], 0);



  return (true);
}

/*********************************************************************
 * @fn      SimpleCentral_doDisconnect
 *
 * @brief   Disconnect the specified link
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool SimpleCentral_doDisconnect(uint8_t index)
{
  (void) index;

  GAP_TerminateLinkReq(scConnHandle, HCI_DISCONNECT_REMOTE_USER_TERM);

  return (true);
}

/*********************************************************************
 * @fn      SimpleCentral_menuSwitchCb
 *
 * @brief   Detect menu context switching
 *
 * @param   pMenuObjCurr - the current menu object
 * @param   pMenuObjNext - the menu object the context is about to switch to
 *
 * @return  none
 */


// Scan & Connect
static void GUA_performPeriodicTask(void)
{
    //Display_printf(dispHandle, 0, 0,
                              //  "PHY Change failure, peer does not support this");
if(Connected==0)
{
    if(numScanRes>0)
      {
        SimpleCentral_doStopDiscovering(1);
        uint8_t addrType;
        uint8_t *peerAddr;
        for (uint8_t i = 0; i < numScanRes; i++)
        {
            if (memcmp(PeripheralMac1, scanList[i].addr, 6) == 0)
            {
                peerAddr = scanList[i].addr;
                addrType = scanList[i].addrType;
                SimpleCentral_doConnect(i);
                Scan=0;
            }
        }
      }

    if(Scan==1)
    {
    SimpleCentral_doStopDiscovering(0);
        SimpleCentral_doDiscoverDevices(0);
    }
    Util_startClock(&GUA_periodicClock);
}
}


static void GUA_HandleKeys(uint8 GUA_Keys)
{

  if(GUA_Keys & GUA_KEY_UP_VALUE)
  {
      Util_startClock(&My_Evt_Debond_Clock);
  }
}




static void SimpleBLECentral_GUAHandler(UArg a0)
{
 Event_post(syncEvent, a0);
}


// Discover Service
static void My_Disc_Svc_Task(void)
{
   SimpleCentral_doSelectConn(0);


   if(Svc_Found==0)
   {
        Util_startClock(&My_Disc_Svc_Clock);
   }


}


static void My_Disc_Svc_Handler(UArg a0)
{
  Event_post(syncEvent, a0);
}

//Open Notify Task
static void My_Open_Notify_Task(void)
{
    OpenNotify();
    if(NOTIFY_OPEN_SUCCESS==0)
    {
        //Util_startClock(&My_Open_Notify_Clock);
    }
    else if(NOTIFY_OPEN_SUCCESS==1)
    {
       // Util_stopClock(&My_Open_Notify_Clock);
       // Util_startClock(&My_Send_State_Clock);
    }
}

static void My_Open_Notify_Handler(UArg a0)
{
    Event_post(syncEvent, a0 );
}


static void My_Send_State_Task(void)
{
  if(State_Sent==0)
  {
    Send_Connect_State();
   // Util_startClock(&My_Send_State_Clock);
  }
//  else if(State_Sent==1)
//  {
//      Util_stopClock(&My_Send_State_Clock);
//  }
  if(Res_perip==1)
  {
      Util_stopClock(&My_Send_State_Clock);
  }

}

static void My_Send_State_Handler(UArg a0)
{
    Event_post(syncEvent, a0 );
}
//Open Notify
uint8_t OpenNotify()
{
    attWriteReq_t req;
    uint8_t status;
    req.pValue = GATT_bm_alloc(scConnHandle, ATT_WRITE_REQ, 2, NULL);
    if ( req.pValue != NULL )
    {
        uint8_t connIndex = SimpleCentral_getConnIndex(scConnHandle);

            // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
            SIMPLECENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      req.handle = connList[connIndex].charHandle+1;
      req.len = 2;
      req.pValue[0] = LO_UINT16(GATT_CLIENT_CFG_NOTIFY);
      req.pValue[1] = HI_UINT16(GATT_CLIENT_CFG_NOTIFY);
      req.sig = 0;
      req.cmd = 0;

      status = GATT_WriteCharValue(scConnHandle, &req, selfEntity);
      if ( status != SUCCESS )
      {
          //UartSendStringAndint32("FailedReason=  ",status);
        GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
        Util_startClock(&My_Open_Notify_Clock);
      }
      else
      {
          NOTIFY_OPEN_SUCCESS=1;
          Util_stopClock(&My_Open_Notify_Clock);
          Util_startClock(&My_Send_State_Clock);
          return TRUE;
      }
    }
    return FALSE;
}

//Indicate connection
uint8_t Send_Connect_State()
{
    attWriteReq_t req;
    uint8_t status;
    unsigned char State=0x05;
    req.pValue = GATT_bm_alloc(scConnHandle, ATT_WRITE_REQ, 1, NULL);
    if ( req.pValue != NULL )
    {
        uint8_t connIndex = SimpleCentral_getConnIndex(scConnHandle);

            // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
            SIMPLECENTRAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      req.handle = connList[connIndex].charHandle1;
      req.len = 1;
      req.pValue[0] = State;
      //req.pValue[1] = HI_UINT16(GATT_CLIENT_CFG_NOTIFY);
      req.sig = 0;
      req.cmd = 0;

      status = GATT_WriteCharValue(scConnHandle, &req, selfEntity);
     // Display_printf(dispHandle, SC_ROW_NON_CONN, 0, "Connecting...");
      if ( status != SUCCESS )
      {
        GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
        Util_startClock(&My_Send_State_Clock);
      }

      else if(status==SUCCESS)
      {
          State_Sent=1;
          Util_stopClock(&My_Send_State_Clock);

      }
    }
    return FALSE;
}


static void My_Debond_Task(void)
{

    GAPBondMgr_SetParameter(GAPBOND_ERASE_ALLBONDS, 0, NULL);

//Util_startClock(&My_Evt_Debond_Clock);

}


static void My_Debond_Handler(UArg a0)
{
  Event_post(syncEvent, a0);
}

static void My_Phyupdate_Task(void)
{


   // HCI_LE_ReadPhyCmd(0);
       SimpleCentral_doSetConnPhy(3);
       if(PHYUPDATED==0)
       {
       Util_startClock(&My_Evt_Phyupdate_Clock);
       }
       else if(PHYUPDATED==1)
       {
           Util_stopClock(&My_Evt_Phyupdate_Clock);

       }

}


static void My_Phyupdate_Handler(UArg a0)
{
  Event_post(syncEvent, a0);
}



static void My_Voltage_Task(void)
{
uint32_t batval, temp;

batval = AONBatMonBatteryVoltageGet();

temp= ((batval&0xFF)*1000)>>8;
temp += ((batval>>8) & 0xFF)*1000;


   Display_printf(dispHandle, 0, 0, "Voltage: %d",temp);

 if(temp>3000)
{

}
  Util_startClock(&My_Evt_Voltage_Clock);
}


static void My_Voltage_Handler(UArg a0)
{
  Event_post(syncEvent, a0);
}



/*********************************************************************
*********************************************************************/




