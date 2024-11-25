#include "mbed.h"
#include "main.h"
//#include "system_stm32f4xx.h"
//#include "stm32f4xx_hal_tim.h"
#include "sx1272-hal.h"
#include "debug.h"
#include "stdio.h"
#include "dotmat.h"

/* Set this flag to '1' to display debug messages on the console */
#define DEBUG_MESSAGE   1

/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   !USE_MODEM_LORA

#define RF_FREQUENCY                                    915000000 // Hz
#define TX_OUTPUT_POWER                                 14        // 14 dBm
#define LORA_BANDWIDTH                                  0         // [0: 125 kHz,
                                                                //  1: 250 kHz,
                                                                //  2: 500 kHz,
                                                                //  3: Reserved]
#define LORA_SPREADING_FACTOR                           7         // [SF7..SF12]
#define LORA_CODINGRATE                                 1         // [1: 4/5,
                                                                //  2: 4/6,
                                                                //  3: 4/7,
                                                                //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                            8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                             5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                      false
#define LORA_FHSS_ENABLED                               false  
#define LORA_NB_SYMB_HOP                                4     
#define LORA_IQ_INVERSION_ON                            false
#define LORA_CRC_ENABLED                                true


#define RX_TIMEOUT_VALUE                                1000      // in ms
#define BUFFER_SIZE                                     32        // Define the payload size here

#define Rx_ID                                           16

void GPIO_Init(void);

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
}AppStates_t;

volatile AppStates_t State = LOWPOWER;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*
 *  Global variables declarations
 */
SX1272MB2xAS Radio( NULL );

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

uint8_t HelloMsg[] = "Hello";
char Msg[10];

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

int16_t RssiValue = 0.0;
int8_t SnrValue = 0.0;

#define FOWARD 0
#define UTURN 1

int8_t FowardState = FOWARD;
int8_t Flag = 0;
int RightMotor = 150;
int LeftMotor = 150;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

void Head_Right();
void Head_Left();

int main( void ) 
{
    HAL_Init();
    uint8_t i;

    debug( "\n\n\r     SX1272 Ping Pong Demo Application \n\n\r" );

    // Initialize Radio driver
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxError = OnRxError;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    Radio.Init( &RadioEvents );

    // verify the connection with the board
    while( Radio.Read( REG_VERSION ) == 0x00  )
    {
        debug( "Radio could not be detected!\n\r", NULL );
        wait( 1 );
    }
    
    debug_if( ( DEBUG_MESSAGE & ( Radio.DetectBoardType( ) == SX1272MB2XAS ) ), "\n\r > Board Type: SX1272MB2xAS < \n\r" );

    Radio.SetChannel( RF_FREQUENCY ); 
    phymac_id = 1;
    debug("[PHYMAC] ID : %i\n", phymac_id);

    debug_if( LORA_FHSS_ENABLED, "\n\n\r             > LORA FHSS Mode < \n\n\r" );
    debug_if( !LORA_FHSS_ENABLED, "\n\n\r             > LORA Mode < \n\n\r" );

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                         LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                         LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                         LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                         LORA_IQ_INVERSION_ON, 2000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                         LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                         LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
                         LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                         LORA_IQ_INVERSION_ON, true );

    debug_if( DEBUG_MESSAGE, "Starting Ping-Pong loop\r\n" );
    
    int sensorValue = 0;
    int RightValue = 0;
    int LeftValue = 0;
    int Direction = 0;

    MX_TIM3_Init();
    MX_TIM4_Init();
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    
    State = IDLE;
    Buffer[0] = Rx_ID;


    GPIO_Init();

    while( 1 )
    {

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, LeftMotor);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, RightMotor);
        
        RightValue = HAL_GPIO_ReadPin(RightIR_GPIO_Port, RightIR_Pin);
        LeftValue = HAL_GPIO_ReadPin(LeftIR_GPIO_Port, LeftIR_Pin);

        printf("Right : %d    Left : %d\n", RightValue, LeftValue);
        //sensorValue = HAL_GPIO_ReadPin(Prox_GPIO_Port, Prox_Pin);
        if (RightValue + LeftValue == 2) {
            FowardState = UTURN;
        }
        else {
            FowardState = FOWARD;
            Direction = LeftValue - RightValue;
        }

        switch (Direction) {
            case -1:
                Head_Left();
                break;
            case 1:
                Head_Right();
                break;
        }

        updateDisplay(SCR_DEFAULT);

        switch( State )
        {
        case IDLE:
            wait_ms( 1000 );
            sprintf(Msg, "Prox : %d\n", sensorValue);
            strcpy( ( char* )Buffer+1, ( char* )Msg );

            Radio.Send( Buffer, BufferSize );
            debug( "...Ping\r\n" );
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
        HAL_Delay(100);
    }
}

void Head_Right() {
    printf("Head Right!\n");
    // LeftMotor = 80;
    // RightMotor = 200;
}

void Head_Left() {
    printf("Head Left!\n");
    // LeftMotor = 100;
    // RightMotor = 210;
}

void GO_U_turn() {
    printf("GO UTURN\n");
    // LeftMotor = 100;
    // RightMotor = 100;
}

static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 900-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

static void MX_TIM4_Init(void)
{
  /* USER CODE BEGIN TIM4_Init 0 */
  __HAL_RCC_TIM4_CLK_ENABLE();
  /* USER CODE END TIM4_Init 0 */
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 900-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PA7     ------> TIM3_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }
  else if(htim->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspPostInit 0 */

  /* USER CODE END TIM4_MspPostInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM4_MspPostInit 1 */

  /* USER CODE END TIM4_MspPostInit 1 */
  }
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
      printf("error error\n");
  }
  /* USER CODE END Error_Handler_Debug */
}


void OnTxDone( void )
{
    Radio.Sleep( );
    State = IDLE;
    debug_if( DEBUG_MESSAGE, "> OnTxDone\n\r" );
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    uint8_t dataIndFlag = 0;
    Radio.Sleep( );
    State = IDLE;
    
    if (Buffer[PHYMAC_PDUOFFSET_TYPE] == PHYMAC_PDUTYPE_DATA &&
        Buffer[PHYMAC_PDUOFFSET_DSTID] == phymac_id)
    {
        dataIndFlag = 1;
    }

}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    // State = TX_TIMEOUT;
    State = IDLE;
    debug_if( DEBUG_MESSAGE, "> OnTxTimeout\n\r" );
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    Buffer[BufferSize] = 0;
    // State = RX_TIMEOUT;
    State = RX;
    debug_if( DEBUG_MESSAGE, "> OnRxTimeout\n\r" );
}

void OnRxError( void )
{
    Radio.Sleep( );
    // State = RX_ERROR;
    State = RX;
    debug_if( DEBUG_MESSAGE, "> OnRxError\n\r" );
}

void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SER_PIN | RCK_PIN | SRCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}