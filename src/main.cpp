#include "mbed.h"
#include "main.h"
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
int Railout = 0;
int Lastuse = 0;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

//static void MX_GPIO_Init(void);

static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
void GPIO_Init(void);

void Head_Right();
void Head_Left();
void Do_Uturn();
void Foward();

int main( void ) 
{
    HAL_Init();
    uint8_t i;
    GPIO_Init();

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

    // PWM timer init and Initialization motor
    MX_TIM1_Init();
    MX_TIM8_Init();
    // Start PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

    State = IDLE;
    Buffer[0] = Rx_ID;

    while( 1 )
    {
        RightValue = HAL_GPIO_ReadPin(RightIR_GPIO_Port, RightIR_Pin);
        LeftValue = HAL_GPIO_ReadPin(LeftIR_GPIO_Port, LeftIR_Pin);

        updateDisplay(SCR_DEFAULT);

        // decide uturn or not
        if (RightValue + LeftValue == 2) {
            Direction = 2;
        }
        else {
            // compare two tracer sensor
            Direction = LeftValue - RightValue;
        }

        if (Railout == 10) {
            Direction = 1;
        }

        // change motor value
        switch (Direction) {
            case -1:
                Head_Left();
                break;
            case 0:
                Foward();
                break;
            case 1:
                Head_Right();
                break;
            case 2:
                Do_Uturn();
                break;
        }
        // wait 0.5s for motor
        HAL_Delay(500);

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 300);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 300);
        HAL_Delay(500);

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
    }
}

void Head_Right() {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 140);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 380);
    Railout = 0;
    Lastuse = 0;
}


void Head_Left() {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 220);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 400);
    Lastuse = 1;
}

void Foward() {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 205);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 400);
    Railout += Lastuse*1;
}

void Do_Uturn() {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 100);
    // extra delay
    HAL_Delay(640);
}

static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  __HAL_RCC_TIM1_CLK_ENABLE();
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 900-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  //htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */
  __HAL_RCC_TIM8_CLK_ENABLE();
  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 900-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 2000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  //htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PA8     ------> TIM1_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }
  else if(htim->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspPostInit 0 */

  /* USER CODE END TIM8_MspPostInit 0 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**TIM8 GPIO Configuration
    PC7     ------> TIM8_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM8_MspPostInit 1 */

  /* USER CODE END TIM8_MspPostInit 1 */
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