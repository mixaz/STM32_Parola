/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>

#include "MD_MAX72xx.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DELAYTIME 50
#define HARDWARE_TYPE FC16_HW
#define MAX_DEVICES 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
LL_SPI_InitTypeDef SPI_InitStruct = {0};
static LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void sendData(uint8_t reg, uint8_t data);
void InitSPI(void);
void scrollText(const char *p);
void zeroPointSet(void);
void rows(void);
void checkboard(void);
void columns(void);
void cross(void);
void bullseye(void);
void stripe(void);
void spiral(void);
void bounce(void);
void intensity(void);
void blinking(void);
void scanLimit(void);
void transformation1(void);
void transformation2(void);
void showCharset(void);
void wrapText(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

#define cs_set() LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4)
#define cs_reset() LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4)

static MD_MAX72XX_t max7219;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* System interrupt init*/

    /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
    */
    LL_GPIO_AF_Remap_SWJ_NOJTAG();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_SPI1_Init();
    /* USER CODE BEGIN 2 */
    MD_MAX72XX_constructor2(&max7219,FC16_HW,(uint8_t)CS_Pin, MAX_DEVICES);
    MD_MAX72XX_begin(&max7219);
//    LL_mDelay(100);
//    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
//    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
//    LL_SPI_Enable(SPI1);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
#if 1
        scrollText("Graphics");
        zeroPointSet();
        rows();
        columns();
        cross();
        stripe();
        checkboard();
        bullseye();
        bounce();
        spiral();
#endif

#if 1
        scrollText("Control");
        intensity();
        scanLimit();
        blinking();
#endif

#if 1
        scrollText("Transform");
        transformation1();
        transformation2();
#endif

#if 1
        scrollText("Charset");
        wrapText();
        showCharset();
#endif
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

   if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
    Error_Handler();
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(72000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(72000000);
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

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    /**SPI1 GPIO Configuration
    PA5   ------> SPI1_SCK
    PA7   ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly = 10;
    LL_SPI_Init(SPI1, &SPI_InitStruct);
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
    LL_SPI_Enable(SPI1);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void scrollText(const char *p)
{
  uint8_t charWidth;
  uint8_t cBuf[8];  // this should be ok for all built-in fonts

  MD_MAX72XX_clear(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1);

  while (*p != '\0')
  {
    charWidth = MD_MAX72XX_getChar(&max7219,*p++, sizeof(cBuf) / sizeof(cBuf[0]), cBuf);

    for (uint8_t i=0; i<=charWidth; i++)	// allow space between characters
    {
      MD_MAX72XX_transform1(&max7219,TSL);
      if (i < charWidth)
    	  MD_MAX72XX_setColumn2(&max7219,0, cBuf[i]);
      LL_mDelay(DELAYTIME);
    }
  }
}

void zeroPointSet()
// Demonstrates the use of setPoint and
// show where the zero point is in the display
{

  MD_MAX72XX_clear(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1);

  if (MAX_DEVICES > 1)
  	MD_MAX72XX_setChar(&max7219,(2*COL_SIZE)-1, '0');

  for (uint8_t i=0; i<ROW_SIZE; i++)
  {
    MD_MAX72XX_setPoint(&max7219,i, i, true);
    MD_MAX72XX_setPoint(&max7219,0, i, true);
    MD_MAX72XX_setPoint(&max7219,i, 0, true);
    LL_mDelay(DELAYTIME);

  }

  LL_mDelay(DELAYTIME*3);
}

void rows()
// Demonstrates the use of setRow()
{
  MD_MAX72XX_clear(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1);

  for (uint8_t row=0; row<ROW_SIZE; row++)
  {
    MD_MAX72XX_setRow2(&max7219,row, 0xff);
    LL_mDelay(2*DELAYTIME);
    MD_MAX72XX_setRow2(&max7219,row, 0x00);
  }
}

void checkboard()
// nested rectangles spanning the entire display
{
  uint8_t chkCols[][2] = { { 0x55, 0xaa }, { 0x33, 0xcc }, { 0x0f, 0xf0 }, { 0xff, 0x00 } };

  MD_MAX72XX_clear(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1);

  for (uint8_t pattern = 0; pattern < sizeof(chkCols)/sizeof(chkCols[0]); pattern++)
  {
    uint8_t col = 0;
    uint8_t idx = 0;
    uint8_t rep = 1 << pattern;

    while (col < MD_MAX72XX_getColumnCount(&max7219))
    {
      for (uint8_t r = 0; r < rep; r++)
        MD_MAX72XX_setColumn2(&max7219,col++, chkCols[pattern][idx]);   // use odd/even column masks
      idx++;
      if (idx > 1) idx = 0;
    }

    LL_mDelay(10 * DELAYTIME);
  }
}

void columns()
// Demonstrates the use of setColumn2()
{
  MD_MAX72XX_clear(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1);

  for (uint8_t col=0; col<MD_MAX72XX_getColumnCount(&max7219); col++)
  {
    MD_MAX72XX_setColumn2(&max7219,col, 0xff);
    LL_mDelay(DELAYTIME/MAX_DEVICES);
    MD_MAX72XX_setColumn2(&max7219,col, 0x00);
  }
}

void cross()
// Combination of setRow() and setColumn2() with user controlled
// display updates to ensure concurrent changes.
{
  MD_MAX72XX_clear(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1);
  MD_MAX72XX_control2(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1, UPDATE, OFF);

  // diagonally down the display R to L
  for (uint8_t i=0; i<ROW_SIZE; i++)
  {
    for (uint8_t j=0; j<MAX_DEVICES; j++)
    {
      MD_MAX72XX_setColumn(&max7219,j, i, 0xff);
      MD_MAX72XX_setRow1(&max7219,j, i, 0xff);
    }
    MD_MAX72XX_flushBufferAll(&max7219);
    LL_mDelay(DELAYTIME);
    for (uint8_t j=0; j<MAX_DEVICES; j++)
    {
      MD_MAX72XX_setColumn(&max7219,j, i, 0x00);
      MD_MAX72XX_setRow1(&max7219,j, i, 0x00);
    }
  }

  // moving up the display on the R
  for (int8_t i=ROW_SIZE-1; i>=0; i--)
  {
    for (uint8_t j=0; j<MAX_DEVICES; j++)
    {
      MD_MAX72XX_setColumn(&max7219,j, i, 0xff);
      MD_MAX72XX_setRow1(&max7219,j, ROW_SIZE-1, 0xff);
    }
    MD_MAX72XX_flushBufferAll(&max7219);
    LL_mDelay(DELAYTIME);
    for (uint8_t j=0; j<MAX_DEVICES; j++)
    {
      MD_MAX72XX_setColumn(&max7219,j, i, 0x00);
      MD_MAX72XX_setRow1(&max7219,j, ROW_SIZE-1, 0x00);
    }
  }

  // diagonally up the display L to R
  for (uint8_t i=0; i<ROW_SIZE; i++)
  {
    for (uint8_t j=0; j<MAX_DEVICES; j++)
    {
      MD_MAX72XX_setColumn(&max7219,j, i, 0xff);
      MD_MAX72XX_setRow1(&max7219,j, ROW_SIZE-1-i, 0xff);
    }
    MD_MAX72XX_flushBufferAll(&max7219);
    LL_mDelay(DELAYTIME);
    for (uint8_t j=0; j<MAX_DEVICES; j++)
    {
      MD_MAX72XX_setColumn(&max7219,j, i, 0x00);
      MD_MAX72XX_setRow1(&max7219,j, ROW_SIZE-1-i, 0x00);
    }
  }
  MD_MAX72XX_control2(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1, UPDATE, ON);
}

void bullseye()
// Demonstrate the use of buffer based repeated patterns
// across all devices.
{
  MD_MAX72XX_clear(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1);
  MD_MAX72XX_control2(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1, UPDATE, OFF);

  for (uint8_t n=0; n<3; n++)
  {
    uint8_t  b = 0xff;
    int   i = 0;

    while (b != 0x00)
    {
      for (uint8_t j=0; j<MAX_DEVICES+1; j++)
      {
        MD_MAX72XX_setRow1(&max7219,j, i, b);
        MD_MAX72XX_setColumn(&max7219,j, i, b);
        MD_MAX72XX_setRow1(&max7219,j, ROW_SIZE-1-i, b);
        MD_MAX72XX_setColumn(&max7219,j, COL_SIZE-1-i, b);
      }
      MD_MAX72XX_flushBufferAll(&max7219);
      LL_mDelay(3*DELAYTIME);
      for (uint8_t j=0; j<MAX_DEVICES+1; j++)
      {
        MD_MAX72XX_setRow1(&max7219,j, i, 0);
        MD_MAX72XX_setColumn(&max7219,j, i, 0);
        MD_MAX72XX_setRow1(&max7219,j, ROW_SIZE-1-i, 0);
        MD_MAX72XX_setColumn(&max7219,j, COL_SIZE-1-i, 0);
      }

      bitClear(b, i);
      bitClear(b, 7-i);
      i++;
    }

    while (b != 0xff)
    {
      for (uint8_t j=0; j<MAX_DEVICES+1; j++)
      {
        MD_MAX72XX_setRow1(&max7219,j, i, b);
        MD_MAX72XX_setColumn(&max7219,j, i, b);
        MD_MAX72XX_setRow1(&max7219,j, ROW_SIZE-1-i, b);
        MD_MAX72XX_setColumn(&max7219,j, COL_SIZE-1-i, b);
      }
      MD_MAX72XX_flushBufferAll(&max7219);
      LL_mDelay(3*DELAYTIME);
      for (uint8_t j=0; j<MAX_DEVICES+1; j++)
      {
        MD_MAX72XX_setRow1(&max7219,j, i, 0);
        MD_MAX72XX_setColumn(&max7219,j, i, 0);
        MD_MAX72XX_setRow1(&max7219,j, ROW_SIZE-1-i, 0);
        MD_MAX72XX_setColumn(&max7219,j, COL_SIZE-1-i, 0);
      }

      i--;
      bitSet(b, i);
      bitSet(b, 7-i);
    }
  }

  MD_MAX72XX_control2(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1, UPDATE, ON);
}

void stripe()
// Demonstrates animation of a diagonal stripe moving across the display
// with points plotted outside the display region ignored.
{
  const uint16_t maxCol = MAX_DEVICES*ROW_SIZE;
  const uint8_t	stripeWidth = 10;

  MD_MAX72XX_clear(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1);

  for (uint16_t col=0; col<maxCol + ROW_SIZE + stripeWidth; col++)
  {
    for (uint8_t row=0; row < ROW_SIZE; row++)
    {
      MD_MAX72XX_setPoint(&max7219,row, col-row, true);
        MD_MAX72XX_setPoint(&max7219,row, col-row - stripeWidth, false);
    }
    LL_mDelay(DELAYTIME);
  }
}

void spiral()
// setPoint() used to draw a spiral across the whole display
{
  int  rmin = 0, rmax = ROW_SIZE-1;
  int  cmin = 0, cmax = (COL_SIZE*MAX_DEVICES)-1;

  MD_MAX72XX_clear(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1);
  while ((rmax > rmin) && (cmax > cmin))
  {
    // do row
    for (int i=cmin; i<=cmax; i++)
    {
      MD_MAX72XX_setPoint(&max7219,rmin, i, true);
      LL_mDelay(DELAYTIME/MAX_DEVICES);
    }
    rmin++;

    // do column
    for (uint8_t i=rmin; i<=rmax; i++)
    {
      MD_MAX72XX_setPoint(&max7219,i, cmax, true);
      LL_mDelay(DELAYTIME/MAX_DEVICES);
    }
    cmax--;

    // do row
    for (int i=cmax; i>=cmin; i--)
    {
      MD_MAX72XX_setPoint(&max7219,rmax, i, true);
      LL_mDelay(DELAYTIME/MAX_DEVICES);
    }
    rmax--;

    // do column
    for (uint8_t i=rmax; i>=rmin; i--)
    {
      MD_MAX72XX_setPoint(&max7219,i, cmin, true);
      LL_mDelay(DELAYTIME/MAX_DEVICES);
    }
    cmin++;
  }
}

void bounce()
// Animation of a bouncing ball
{
  const int minC = 0;
  const int maxC = MD_MAX72XX_getColumnCount(&max7219)-1;
  const int minR = 0;
  const int maxR = ROW_SIZE-1;

  int  nCounter = 0;

  int  r = 0, c = 2;
  int8_t dR = 1, dC = 1;	// delta row and column

  MD_MAX72XX_clear(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1);

  while (nCounter++ < 200)
  {
    MD_MAX72XX_setPoint(&max7219,r, c, false);
    r += dR;
    c += dC;
    MD_MAX72XX_setPoint(&max7219,r, c, true);
    LL_mDelay(DELAYTIME/2);

    if ((r == minR) || (r == maxR))
      dR = -dR;
    if ((c == minC) || (c == maxC))
      dC = -dC;
  }
}

void intensity()
// Demonstrates the control of display intensity (brightness) across
// the full range.
{
  uint8_t row;

  MD_MAX72XX_clear(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1);

  // Grow and get brighter
  row = 0;
  for (int8_t i=0; i<=MAX_INTENSITY; i++)
  {
    MD_MAX72XX_control2(&max7219,0,MD_MAX72XX_getDeviceCount(&max7219)-1,INTENSITY, i);
    if (i%2 == 0)
        MD_MAX72XX_setRow2(&max7219,row++, 0xff);
    LL_mDelay(DELAYTIME*3);
  }

  MD_MAX72XX_control2(&max7219,0,MD_MAX72XX_getDeviceCount(&max7219)-1,INTENSITY, 8);
}

void blinking()
// Uses the test function of the MAX72xx to blink the display on and off.
{
  int  nDelay = 1000;

  MD_MAX72XX_clear(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1);

  while (nDelay > 0)
  {
   MD_MAX72XX_control2(&max7219,0,MD_MAX72XX_getDeviceCount(&max7219)-1,TEST, ON);
   LL_mDelay(nDelay);
   MD_MAX72XX_control2(&max7219,0,MD_MAX72XX_getDeviceCount(&max7219)-1,TEST, OFF);
   LL_mDelay(nDelay);

   nDelay -= DELAYTIME;
  }
}

void scanLimit(void)
// Uses scan limit function to restrict the number of rows displayed.
{
  MD_MAX72XX_clear(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1);

  MD_MAX72XX_control2(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1, UPDATE, OFF);
  for (uint8_t row=0; row<ROW_SIZE; row++)
      MD_MAX72XX_setRow2(&max7219,row, 0xff);
  MD_MAX72XX_control2(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1, UPDATE, ON);

  for (int8_t s=MAX_SCANLIMIT; s>=0; s--)
  {
    MD_MAX72XX_control2(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1, SCANLIMIT, s);
    LL_mDelay(DELAYTIME*5);
  }
  MD_MAX72XX_control2(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1, SCANLIMIT, MAX_SCANLIMIT);
}

void transformation1()
// Demonstrates the use of transform() to move bitmaps on the display
// In this case a user defined bitmap is created and animated.
{
  uint8_t arrow[COL_SIZE] =
  {	0x08, 0x1C, 0X3E, 0x7F, 0x1C, 0x1C, 0X3E};

  enum transformType_t  t[] =
    {
      TSL, TSL, TSL, TSL,
      TSL, TSL, TSL, TSL,
      TSL, TSL, TSL, TSL,
      TSL, TSL, TSL, TSL,
      TFLR,
      TSR, TSR, TSR, TSR,
      TSR, TSR, TSR, TSR,
      TSR, TSR, TSR, TSR,
      TSR, TSR, TSR, TSR,
      TRC,
      TSD, TSD, TSD, TSD,
      TSD, TSD, TSD, TSD,
      TFUD,
      TSU, TSU, TSU, TSU,
      TSU, TSU, TSU, TSU,
      TINV,
      TRC, TRC, TRC, TRC,
      TINV
    };

  MD_MAX72XX_clear(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1);

  // use the arrow bitmap
  MD_MAX72XX_control2(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1, UPDATE, OFF);
  for (uint8_t j=0; j<MD_MAX72XX_getDeviceCount(&max7219); j++)
      MD_MAX72XX_setBuffer(&max7219,((j+1)*COL_SIZE)-1, COL_SIZE, arrow);

  MD_MAX72XX_control2(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1, UPDATE, ON);
  LL_mDelay(DELAYTIME);

  // run through the transformations
  MD_MAX72XX_control2(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1, WRAPAROUND, ON);
  for (uint8_t i=0; i<(sizeof(t)/sizeof(t[0])); i++)
  {
    MD_MAX72XX_transform1(&max7219,t[i]);
    LL_mDelay(DELAYTIME*4);
  }
  MD_MAX72XX_control2(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1, WRAPAROUND, OFF);
}

void transformation2()
// Demonstrates the use of transform() to move bitmaps on the display
// In this case font characters are loaded into the display for animation.
{
  enum transformType_t  t[] =
  {
    TINV,
    TRC, TRC, TRC, TRC,
    TINV,
    TSL, TSL, TSL, TSL, TSL,
    TSR, TSR, TSR, TSR, TSR,
    TSR, TSR, TSR, TSR, TSR, TSR, TSR, TSR,
    TSL, TSL, TSL, TSL, TSL, TSL, TSL, TSL,
    TSR, TSR, TSR,
    TSD, TSU, TSD, TSU,
    TFLR, TFLR, TFUD, TFUD
  };

  MD_MAX72XX_clear(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1);

  // use the arrow bitmap
  MD_MAX72XX_control2(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1, WRAPAROUND, OFF);

  // draw something that will show changes
  for (uint8_t j=0; j<MD_MAX72XX_getDeviceCount(&max7219); j++)
  {
      MD_MAX72XX_setChar(&max7219,((j+1)*COL_SIZE)-1, '0'+j);
  }
  LL_mDelay(DELAYTIME*5);

  // run thru transformations
  for (uint8_t i=0; i<(sizeof(t)/sizeof(t[0])); i++)
  {
    MD_MAX72XX_transform1(&max7219,t[i]);
    LL_mDelay(DELAYTIME*3);
  }
}

void wrapText()
// Display text and animate scrolling using auto wraparound of the buffer
{
  MD_MAX72XX_clear(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1);
  MD_MAX72XX_control2(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1, WRAPAROUND, OFF);

  // draw something that will show changes
  for (uint16_t j=0; j<MD_MAX72XX_getDeviceCount(&max7219); j++)
  {
      MD_MAX72XX_setChar(&max7219,((j+1)*COL_SIZE)-1, (j&1 ? 'M' : 'W'));
  }
  LL_mDelay(DELAYTIME*5);

  // run thru transformations
  for (uint16_t i=0; i<3*COL_SIZE*MAX_DEVICES; i++)
  {
      MD_MAX72XX_transform1(&max7219,TSL);
      LL_mDelay(DELAYTIME/2);
  }
  for (uint16_t i=0; i<3*COL_SIZE*MAX_DEVICES; i++)
  {
      MD_MAX72XX_transform1(&max7219,TSR);
      LL_mDelay(DELAYTIME/2);
  }
  for (uint8_t i=0; i<ROW_SIZE; i++)
  {
      MD_MAX72XX_transform1(&max7219,TSU);
      LL_mDelay(DELAYTIME*2);
  }
  for (uint8_t i=0; i<ROW_SIZE; i++)
  {
      MD_MAX72XX_transform1(&max7219,TSD);
      LL_mDelay(DELAYTIME*2);
  }

  MD_MAX72XX_control2(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1, WRAPAROUND,OFF);
}

void showCharset(void)
// Run through display of the the entire font characters set
{
  MD_MAX72XX_clear(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1);
  MD_MAX72XX_control2(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1, UPDATE, OFF);

  for (uint16_t i=0; i<256; i++)
  {
    MD_MAX72XX_clear2(&max7219,0);
    MD_MAX72XX_setChar(&max7219,COL_SIZE-1, i);

    if (MAX_DEVICES >= 3)
    {
      char hex[3];

//      sprintf(hex, "%02X", i);
      if(i <= 0xF) {
          hex[0] = '0';
          itoa(i, hex+1, 16);
      }
      else {
          itoa(i, hex, 16);
      }

      MD_MAX72XX_clear2(&max7219,1);
      MD_MAX72XX_setChar(&max7219,(2*COL_SIZE)-1,hex[1]);
      MD_MAX72XX_clear2(&max7219,2);
      MD_MAX72XX_setChar(&max7219,(3*COL_SIZE)-1,hex[0]);
    }

    MD_MAX72XX_flushBufferAll(&max7219);
    LL_mDelay(DELAYTIME*2);
  }
  MD_MAX72XX_control2(&max7219,0, MD_MAX72XX_getDeviceCount(&max7219)-1, UPDATE,ON);
}
	
void HAL_SYSTICK_Callback(void)
{
  
  static uint16_t cnt_led = 0;
	
  if(cnt_led == 100)
  {
        LL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		cnt_led = 0;
  }
	
cnt_led++;
  
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
