/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "defs.h"
#include "funcs.h"
#include "usbd_cdc_if.h"
#include "MS5607.h"
#include "GPS.h"
#include "ISRegisters.h"
#include "HR4000.h"
#include "sdfs.h"
#include "TMP117.h"
#include "AT30TS74.h"
#include "PID.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LOGENVIRO (true)
#define LOGCOORDS (true)
#define USEHR4000 (true)
#define LOGTMP117 (false)
#define LOGAT30TS74 (true)
#define USESD (true)
#define USETMP117 (false)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// The main user code runs off of timer which decides when to run various routines
volatile bool DoPressureSensor = false;
volatile bool DoGPSTick = false;
// DoHR4000 runs a state machine which configures the HR4000 and takes spectra
volatile bool DoHR4000 = false;
// this is a relic from an earlier system that didn't return to state 0 between exposures
volatile bool DoGetSpectra = false;
volatile bool HRDataReady = false;
volatile bool DoSampleTMP117 = false;
volatile bool DoSampleAT30TS74 = false;
volatile bool DoUserUpdate = false;

volatile uint8_t Ticks_TMP117 = 0;
volatile uint8_t Ticks_AT30TS74 = 0;

volatile uint8_t PressureSensorState = 0;
volatile uint16_t ClockTick = 0;
volatile uint16_t HeaterTick = 0;
volatile uint64_t ElapsedSeconds = 0;
struct sUARTBuffer GPSUART = {0, 0, {0}, {0}};

volatile uint8_t hrbuf[8200] = {0};

static uint8_t I2C1_Input_Buffer[10] = {0};
static uint8_t I2C1_Output_Buffer[10] = {0};
static uint8_t I2C1_Command_Buffer[10] = {0};

// the code which uses these is broken
bool RegistersBusy = false;
uint32_t Registers[255];
uint32_t BackupRegisters[255];

volatile struct sHeaters Heaters;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM13_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  struct sState State;
  struct sGPSFrame GPSFrame;
  struct sMS5607 MS5607;

  struct sHR4000 HR4000;
  struct sSpectra Spectra;
  struct sTMP117 TMP117_1, TMP117_2;
  struct sAT30TS74 AT30TS74_1, AT30TS74_2;
  struct sPID PID_1, PID_2, PID_3, PID_4, PID_5, PID_6;

  int i, j;

  uint8_t buffer[250];
  uint64_t t;

  bool Booting = true;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  InitState(&State);
  InitGPSFrame(&GPSFrame);
  InitMS5607(&MS5607);
  InitRegisters(Registers, BackupRegisters);
  PID_InitStruct(&PID_1);
  PID_InitStruct(&PID_2);
  PID_InitStruct(&PID_3);
  PID_InitStruct(&PID_4);
  PID_InitStruct(&PID_5);
  PID_InitStruct(&PID_6);
  if (USETMP117)
  {
    TMP117_InitStruct(&TMP117_1, &hi2c2, 0);
    TMP117_InitStruct(&TMP117_2, &hi2c2, 1);
  }

  AT30TS74_InitStruct(&AT30TS74_1, &hi2c2, 3);
  AT30TS74_InitStruct(&AT30TS74_2, &hi2c2, 7);
//  AT30TS74_InitStruct(&AT30TS74_3, &hi2c2, 6);
//  AT30TS74_InitStruct(&AT30TS74_4, &hi2c2, 1);

  HR_InitStruct(&HR4000);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SDIO_SD_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_TIM13_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  printf("-- REBOOT --\n");
  InitDWTTimer(); // system clock timer for creating us delays for onewire

  j = sizeof Spectra.RawData / sizeof Spectra.RawData[0];
  for (i=0;i<j;i++)
    Spectra.RawData[i]=0;  // zero the spectra data memory

// we keep track of the heater dwell, instead of the duty cycle, because it speeds up math in the heater PWM interrupt
// these default to a duty cycle of 0

  Heaters.HeaterDwell[0]=(100-00)/2;
  Heaters.HeaterDwell[1]=(100-00)/2;
  Heaters.HeaterDwell[2]=(100-00)/2;
  Heaters.HeaterDwell[3]=(100-00)/2;
  Heaters.HeaterDwell[4]=(100-00)/2;
  Heaters.HeaterDwell[5]=(100-00)/2;

// I don't remember the numbers that led to this being 2/2, probably something to do with the clock frequency
// this tells the PID code how long between updates
  PID_5.DeltaT = 2.0f/2.0f;
  PID_6.DeltaT = 2.0f/2.0f;

  // these are the target temperatures for heater controllers 5 and 6.
  PID_5.TargetP = 27.0f;
  PID_6.TargetP = 27.0f;

  // heaters are configured with a gain of three. With a gain of three, a temperature difference between
  // the set point and the measured temperature of 0.1 C will result in 30% duty cycle.

  PID_5.Kp = 3.0f;
  PID_6.Kp = 3.0f;

  // default configuration values for the HR4000

  HR4000.NextIntegrationTime_ms = 100;
  HR4000.Smoothing = 0;
  HR4000.Summing = 1;
  HR4000.Checksum = false;

  // we want to force configuring the integration time, so we set the NextIntegrationTime_ms to be different
  // from the current integration time. This is left over from before we forced a reset to state 0 in the
  // HR4000 state machine.

  HR4000.IntegrationTime_ms = HR4000.NextIntegrationTime_ms + 1;

  if (USESD)
  {
    while (SDFS_Mount())
    {
      printf("Could not mount SDFS. Sleeping for 3s and trying again.\n");
      HAL_Delay(3000);
    }
  }

  HAL_UART_Receive_IT(&huart2, &GPSUART.ByteBuffer,1);
  HAL_I2C_Slave_Receive_IT(&hi2c1, I2C1_Input_Buffer, 5);
  // we need the coefficients programmed into the MS5607 to interpret the data
  MS5607_GetCoefficients(&hi2c2, &MS5607);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_Delay(100); // give the devices a moment to compose themselves

  // Timers must be started after all structs are initialized!
  HAL_TIM_Base_Start_IT(&htim13); // Heater Timer
  HAL_TIM_Base_Start_IT(&htim14); // start our timer last

  while (1)
  {
    if (DoUserUpdate)
    {
      DoUserUpdate = false;
      printf("Booting %d, GPS fix %d\n",Booting, GPSFrame.Fixed);
    }

    if (Booting)
    {
      if (USESD)
        SDFS_SetupFS(&State, &GPSFrame);
      Booting = false;
    }

    // keep a single directory from becoming too full. FATFS does not handle large directories well.
    if (USESD & (State.CreatedFiles >= 250))
      SDFS_IncrementDirectory(&State);

    // This is a semaphore which is only set in the interrupt and only cleared in the main program
    // to notify us when data is ready to be written.

    if (HRDataReady)
    {
      HRDataReady = false;
      HR4000.DataReady = true;
    }

    if (DoSampleAT30TS74 && (!Booting))
    {
      DoSampleAT30TS74 = false;
      if (AT30TS74_1.Configured)
      {
        AT30TS74_GetTemperature(&AT30TS74_1);
        AT30TS74_GetTemperature(&AT30TS74_2);
        t = GetMilliseconds();
        printf("%ld.%03d - %.2f %.2f    %.0f %.0f \n",
            (uint32_t) (t/1000), (uint16_t) (t%1000),
            AT30TS74_1.Temperature,
            AT30TS74_2.Temperature,
            (float)(100 - 2 * Heaters.HeaterDwell[4]),
            (float)(100 - 2 * Heaters.HeaterDwell[5]));

        if (USESD & LOGAT30TS74)
        {
          t = GetMilliseconds();
          snprintf((char*) buffer, 200, "%ld.%03d - %.2f %.2f    %.0f %.0f \n",
            (uint32_t) (t/1000), (uint16_t) (t%1000),
            AT30TS74_1.Temperature,
            AT30TS74_2.Temperature,
            (float)(100 - 2 * Heaters.HeaterDwell[4]),
            (float)(100 - 2 * Heaters.HeaterDwell[5]));
          SDFS_WriteString(&State, buffer, "AT30TS74.txt");

          // we should have PID_Effort handle PID_SavePoint but I'm afraid to change anything now
          // so we leave it for flight

          // this code runs the PID calculations and sets the heater dwell length
          PID_SavePoint(&PID_5, AT30TS74_1.Temperature);
          Heaters.HeaterDwell[4] = 100 * (1-PID_Effort(&PID_5, AT30TS74_1.Temperature))/2;
          PID_SavePoint(&PID_6, AT30TS74_2.Temperature);
          Heaters.HeaterDwell[5] = 100 * (1-PID_Effort(&PID_6, AT30TS74_2.Temperature))/2;
        }
      }
      else
      {
        AT30TS74_Configure(&AT30TS74_1);
        AT30TS74_Configure(&AT30TS74_2);
      }
    }

    if (USETMP117 &&DoSampleTMP117 && (!Booting))
    {
      DoSampleTMP117 = false;
      if (TMP117_1.Configured)
      {
        TMP117_GetTemperature(&TMP117_1);
        TMP117_GetTemperature(&TMP117_2);

        t = GetMilliseconds();
        if (((TMP117_1.Index) % 2) == 0)
        {
          printf("%ld.%03d - ", (uint32_t) (t/1000), (uint16_t) (t%1000));
          printf("TMP %.3f %.3f\n", TMP117_1.Temperature, TMP117_2.Temperature);

          PID_SavePoint(&PID_1, TMP117_1.Temperature);
          Heaters.HeaterDwell[0] = 100 * (1-PID_Effort(&PID_1, TMP117_1.Temperature))/2;
        }

        if (USESD & LOGTMP117)
        {
          snprintf((char*) buffer, 200, "%ld.%03d - %.3f %.3f\n", (uint32_t) (t/1000), (uint16_t) (t%1000), TMP117_1.Temperature, TMP117_2.Temperature);
          SDFS_WriteString(&State, buffer, "TMP117.txt");
        }
      }
      else
      {
        TMP117_Configure(&TMP117_1);
        TMP117_Configure(&TMP117_2);
      }
    }

    // This is left over from before we reset the state back to 0 after getting a spectra
    if (DoGetSpectra && USEHR4000 && (!Booting))
    {
      DoGetSpectra = false;
      HR4000.GetSpectra = true;
    }

    if (DoHR4000 && USEHR4000 && (!Booting))
    {
      DoHR4000 = false;
      HR_Execute(&State, &HR4000, &Spectra, &GPSFrame);
      if (USESD & Spectra.ReadyToSave)
      {
        Spectra.ReadyToSave = false;
        SDFS_WriteSpectraBinary(&State, &Spectra, &GPSFrame);
      }
    }

    // we want this before processing a GPS sentence
    if (DoGPSTick)
    {
      DoGPSTick = false;
      GPSFrame.Ticks += 1;
    }

    if (DoPressureSensor && (!Booting))
    {
      DoPressureSensor = false;
      switch (PressureSensorState)
      {
        case 0:
          MS5607_StartConversion(&hi2c2, false);
          break;
        case 1:
          MS5607.D[1] = MS5607_ReadADC(&hi2c2);
          MS5607_StartConversion(&hi2c2, true);
          break;
        case 2:
          MS5607.D[2] = MS5607_ReadADC(&hi2c2);
          MS5607_CalculatePressure(&MS5607, &(State.MSTemperature), &(State.MSPressure));
//          printf("Temperature: %.2f, Pressure: %.2f\n", (double) State.MSTemperature/100, (double) State.MSPressure/100);
          if (USESD & LOGENVIRO)
            SDFS_WriteEnvironmental(&State, &GPSFrame);
          break;
      }
    }

    if (GPSUART.ReadySentence[0] != 0)
    {
      if ((strncmp((char*) GPSUART.ReadySentence, "$GNRMC", 5) == 0) ||
          (strncmp((char*) GPSUART.ReadySentence, "$GNGGA", 5) == 0))  // if header matches
      {
        if (strncmp((char*) GPSUART.ReadySentence, "$GNRMC", 5) == 0)  // if header matches
          strncpy(GPSFrame.RMCSentence, (char*) GPSUART.ReadySentence, 90);
        else if (strncmp((char*) GPSUART.ReadySentence, "$GNGGA", 5) == 0)  // if header matches
          strncpy(GPSFrame.GGASentence, (char*) GPSUART.ReadySentence, 90);
        if (!ProcessGPSFrame(&GPSFrame))
        {
//          printf("Time: %ld   Date: %ld   Lat: %f   Long: %f \n", GPSFrame.Time, GPSFrame.Date, GPSFrame.Latitude, GPSFrame.Longitude);
//          printf("GPS Time: %ld\n", GPSFrame.Time);
          if (USESD & LOGCOORDS)
            SDFS_WriteCoords(&State, &GPSFrame);
        }
      }
      GPSUART.ReadySentence[0] = 0; // flag that we've processed this sentence.
    }


    if (I2C1_Command_Buffer[0] != 0)
      ProcessI2CCommand(I2C1_Command_Buffer, &State);

    //This is untested and likely broken.
    //LoadRegisters(Registers, BackupRegisters, &RegistersBusy, &GPSFrame, &MS5607, &State);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 70;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 3000;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 6;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 720;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE8 PE9 PE10 PE11
                           PE12 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len)
{
  int i, res;
  for (i=0;i<2050; i++)
  {
    res = CDC_Transmit_FS((uint8_t*) ptr, len);  // USB out
    if (res == USBD_OK)
      break;
  }
  for (int i = 0; i < len; i++)
    ITM_SendChar((*ptr++));              // debugger out
  return len;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint8_t i;
  if (htim->Instance == htim13.Instance)  // deal with the software PWM
  {
    HeaterTick = (HeaterTick + 1) % 100;
    for (i=0;i<6;i++)
    {
      if ((HeaterTick > Heaters.HeaterDwell[i])
        && (HeaterTick < (100 - Heaters.HeaterDwell[i])))
        SetHeater(i, true);
      else
        SetHeater(i, false);
    }
  }

  if (htim->Instance == htim14.Instance)  // this is our 10 ms processing tick
  {
    ClockTick = (ClockTick + 1) % 100;
    // this should be after the ClockTick increment
    if (ClockTick == 0)
    {
      ElapsedSeconds++;
      if (ElapsedSeconds%10 == 0)
        DoUserUpdate = true;
    }

    if ((ElapsedSeconds > 20) && ((ElapsedSeconds%5)==0) && (ClockTick == 0))
      DoGetSpectra = true;

    DoGPSTick = true;
    DoHR4000 = true;

    Ticks_TMP117++;
    Ticks_AT30TS74++;

    if (Ticks_TMP117 >= 13)
    {
      Ticks_TMP117 = 0;
      DoSampleTMP117 = true;
    }

    if (Ticks_AT30TS74 >=100)
    {
      Ticks_AT30TS74 = 0;
      DoSampleAT30TS74 = true;
    }

    if (ClockTick == 20)
    {
      DoPressureSensor = true;
      PressureSensorState = 0;
    }
    if (ClockTick == 22)
    {
      DoPressureSensor = true;
      PressureSensorState = 1;
    }
    if (ClockTick == 24)
    {
      DoPressureSensor = true;
      PressureSensorState = 2;
    }
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *i2c)
{
  int i;
  uint32_t retval = 0;
  if (i2c->Instance == I2C1)
  {
    if (I2C1_Input_Buffer[0] < 0x80) // read
    {
      if (RegistersBusy)
        retval = BackupRegisters[I2C1_Input_Buffer[0]];
      else
        retval = Registers[I2C1_Input_Buffer[0]];
      I2C1_Output_Buffer[0] = I2C1_Input_Buffer[0];
      I2C1_Output_Buffer[1] = (retval >> 24) & 0xff;
      I2C1_Output_Buffer[2] = (retval >> 16) & 0xff;
      I2C1_Output_Buffer[3] = (retval >> 8) & 0xff;
      I2C1_Output_Buffer[4] = retval & 0xff;
      HAL_I2C_Slave_Transmit_IT(i2c,I2C1_Output_Buffer, 5);
    }
    else // write
    {
      for (i=0;i<5;i++)
        I2C1_Command_Buffer[i] = I2C1_Input_Buffer[i];
    }
  }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) // GPS
  {
    if ((GPSUART.ByteBuffer != 0) && (GPSUART.ByteBuffer != '\r')) // ignore carriage returns
    {
      if (GPSUART.ByteBuffer == '\n')
      {
        GPSUART.ReceiveBuffer[GPSUART.Index++] = 0;
        memcpy(GPSUART.ReadySentence, GPSUART.ReceiveBuffer, GPSUART.Index);
        GPSUART.Index = 0;
      }
      else
      {
        GPSUART.ReceiveBuffer[GPSUART.Index++] = GPSUART.ByteBuffer;
        if (GPSUART.Index >= 90) // we have junk in our buffer
          GPSUART.Index = 0;
      }
    }
    HAL_UART_Receive_IT(&huart2, &GPSUART.ByteBuffer,1);
  }
  if (huart->Instance == USART1) // HR4000
  {
    HRDataReady = true;
  }


}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart)
{

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

