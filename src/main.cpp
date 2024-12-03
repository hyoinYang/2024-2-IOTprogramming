#include "mbed.h"
#include "main.h"
#include "sx1272-hal.h"
#include "debug.h"
#include "stdio.h"
#include "hcsr04.hpp"

/* Set this flag to '1' to display debug messages on the console */
#define DEBUG_MESSAGE 0

/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA 1
#define USE_MODEM_FSK !USE_MODEM_LORA

#define RF_FREQUENCY 915000000  // Hz
#define TX_OUTPUT_POWER 14      // 14 dBm
#define LORA_BANDWIDTH 0        // [0: 125 kHz,
                                //  1: 250 kHz,
                                //  2: 500 kHz,
                                //  3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1       // [1: 4/5,
                                //  2: 4/6,
                                //  3: 4/7,
                                //  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 5   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_FHSS_ENABLED false
#define LORA_NB_SYMB_HOP 4
#define LORA_IQ_INVERSION_ON false
#define LORA_CRC_ENABLED true

#define RX_TIMEOUT_VALUE 1000 // in ms
#define BUFFER_SIZE 32        // Define the payload size here

#define Rx_ID 14

/*
 *  Global variables declarations
 */
typedef enum
{
    LOWPOWER = 0,
    IDLE,

    RX,
    RX_TIMEOUT,
    RX_ERROR,

    TX,
    TX_TIMEOUT,

    CAD,
    CAD_DONE
} AppStates_t;

volatile AppStates_t State = LOWPOWER;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*
 *  Global variables declarations
 */
SX1272MB2xAS Radio(NULL);

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

int16_t RssiValue = 0.0;
int8_t SnrValue = 0.0;

int main(void)
{
    uint8_t i;

    debug("\n\n\r     SX1272 Ping Pong Demo Application \n\n\r");

    // Initialize Radio driver
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxError = OnRxError;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    Radio.Init(&RadioEvents);

    // verify the connection with the board
    while (Radio.Read(REG_VERSION) == 0x00)
    {
        debug("Radio could not be detected!\n\r", NULL);
        wait(1);
    }

    debug_if((DEBUG_MESSAGE & (Radio.DetectBoardType() == SX1272MB2XAS)), "\n\r > Board Type: SX1272MB2xAS < \n\r");

    Radio.SetChannel(RF_FREQUENCY);
    phymac_id = 1;
    debug("[PHYMAC] ID : %i\n", phymac_id);

    debug_if(LORA_FHSS_ENABLED, "\n\n\r             > LORA FHSS Mode < \n\n\r");
    debug_if(!LORA_FHSS_ENABLED, "\n\n\r             > LORA Mode < \n\n\r");

    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                      LORA_IQ_INVERSION_ON, 2000);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
                      LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                      LORA_IQ_INVERSION_ON, true);

    debug_if(DEBUG_MESSAGE, "Starting Ping-Pong loop\r\n");

    State = IDLE;
    Buffer[0] = Rx_ID;
    while (1)
    {
        switch (State)
        {
        case IDLE:
            sprintf((char *)Buffer + 1, "Sonic Left : %f \r\n", hcsr04_l.read_cm());
            Radio.Send(Buffer, BufferSize);
            debug((char *)Buffer);

            sprintf((char *)Buffer + 1, "Sonic Right : %f \r\n", hcsr04_r.read_cm());
            Radio.Send(Buffer, BufferSize);
            debug((char *)Buffer);
            State = TX;
            break;
        case TX:
            break;
        case RX:
            break;

        default:
            //     State = TX;
            break;
        }
    }
}

void OnTxDone(void)
{
    Radio.Sleep();
    State = IDLE;
    debug_if(DEBUG_MESSAGE, "> OnTxDone\n\r");
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    uint8_t dataIndFlag = 0;
    Radio.Sleep();
    State = IDLE;

    if (Buffer[PHYMAC_PDUOFFSET_TYPE] == PHYMAC_PDUTYPE_DATA &&
        Buffer[PHYMAC_PDUOFFSET_DSTID] == phymac_id)
    {
        dataIndFlag = 1;
    }
}

void OnTxTimeout(void)
{
    Radio.Sleep();
    // State = TX_TIMEOUT;
    State = IDLE;
    debug_if(DEBUG_MESSAGE, "> OnTxTimeout\n\r");
}

void OnRxTimeout(void)
{
    Radio.Sleep();
    Buffer[BufferSize] = 0;
    // State = RX_TIMEOUT;
    State = RX;
    debug_if(DEBUG_MESSAGE, "> OnRxTimeout\n\r");
}

void OnRxError(void)
{
    Radio.Sleep();
    // State = RX_ERROR;
    State = RX;
    debug_if(DEBUG_MESSAGE, "> OnRxError\n\r");
}

void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
}
