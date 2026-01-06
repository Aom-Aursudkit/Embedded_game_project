/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAW_UP     (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_SET)
#define RAW_DOWN   (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == GPIO_PIN_SET)
#define RAW_LEFT   (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_SET)
#define RAW_RIGHT  (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)
#define RAW_A      (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_SET)
#define RAW_B      (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_SET)

#define DEBOUNCE_MS 20

#define BLOCK_SIZE   12
#define BOARD_COLS   10
#define BOARD_ROWS   20
#define BORDER 2

#define BOARD_WIDTH  (BLOCK_SIZE * BOARD_COLS)   // 120
#define BOARD_HEIGHT (BLOCK_SIZE * BOARD_ROWS)   // 240

#define BOARD_X      20    // left margin
#define BOARD_Y      40    // top margin

//TETRIS MENU TEXT
#define LOGO_BLOCK 6
#define LETTER_W 5
#define LETTER_H 5

#define COLOR_TEXT   0xFFFF   // white
#define COLOR_BG     0x0000   // black
#define COLOR_GRID   0x7BEF   // gray
#define COLOR_BORDER 0xFFFF   // white

#define GRAVITY_MS     800  // normal fall
#define SOFTDROP_MS    50   // soft drop speed
#define DAS_DELAY_MS  170   // initial delay
#define ARR_MS         40   // repeat speed
#define LOCK_DELAY_MS 500
#define LOCK_RESET_LIMIT   15

#define PREVIEW_X   (BOARD_X + BOARD_WIDTH + 12)
#define PREVIEW_Y   (BOARD_Y + 12)
#define PREVIEW_BLOCK  12
#define PREVIEW_BOARD_ROW   1
#define PREVIEW_BOARD_COL   (BOARD_COLS + 1)


#define UI_X   (BOARD_X + BOARD_WIDTH + 10)
#define UI_Y   (BOARD_Y + 80)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef struct {
    uint8_t state;        // debounced state (0/1)
    uint8_t pressed;       // 0->1
    uint8_t released;      // 1->0
    uint32_t last_change_ms;
} Button_t;
Button_t btn_up, btn_down, btn_left, btn_right, btn_a, btn_b;
volatile uint8_t btn_changed;

typedef enum {
    STATE_MENU,
    STATE_PLAY,
    STATE_GAMEOVER
} GameState;

GameState gameState = STATE_MENU;

uint8_t menuLevel = 0;
uint8_t uiDrawn = 0;

typedef enum {
    PIECE_I = 0,
    PIECE_O,
    PIECE_T,
    PIECE_S,
    PIECE_Z,
    PIECE_J,
    PIECE_L,
    PIECE_COUNT
} TetrisPiece;

const uint16_t PIECE_COLOR[PIECE_COUNT] = {
	0x04FF, // I - soft cyan
	0xFD20, // O - warm yellow
	0xC81F, // T - purple
	0x05E0, // S - green
	0xF280, // Z - orange-red
	0x0016, // J - deep blue
	0xD800  // L - red
};

const uint8_t TETROMINO[PIECE_COUNT][4][4][4] =
{
/* ================= I ================= */
{
    {
        {0,0,0,0},
        {1,1,1,1},
        {0,0,0,0},
        {0,0,0,0}
    },
    {
        {0,0,1,0},
        {0,0,1,0},
        {0,0,1,0},
        {0,0,1,0}
    },
    {
        {0,0,0,0},
        {0,0,0,0},
        {1,1,1,1},
        {0,0,0,0}
    },
    {
        {0,1,0,0},
        {0,1,0,0},
        {0,1,0,0},
        {0,1,0,0}
    }
},

/* ================= O ================= */
{
    {
        {0,1,1,0},
        {0,1,1,0},
        {0,0,0,0},
        {0,0,0,0}
    },
    {
        {0,1,1,0},
        {0,1,1,0},
        {0,0,0,0},
        {0,0,0,0}
    },
    {
        {0,1,1,0},
        {0,1,1,0},
        {0,0,0,0},
        {0,0,0,0}
    },
    {
        {0,1,1,0},
        {0,1,1,0},
        {0,0,0,0},
        {0,0,0,0}
    }
},

/* ================= T ================= */
{
    {
        {0,1,0,0},
        {1,1,1,0},
        {0,0,0,0},
        {0,0,0,0}
    },
    {
        {0,1,0,0},
        {0,1,1,0},
        {0,1,0,0},
        {0,0,0,0}
    },
    {
        {0,0,0,0},
        {1,1,1,0},
        {0,1,0,0},
        {0,0,0,0}
    },
    {
        {0,1,0,0},
        {1,1,0,0},
        {0,1,0,0},
        {0,0,0,0}
    }
},

/* ================= S ================= */
{
    {
        {0,1,1,0},
        {1,1,0,0},
        {0,0,0,0},
        {0,0,0,0}
    },
    {
        {0,1,0,0},
        {0,1,1,0},
        {0,0,1,0},
        {0,0,0,0}
    },
    {
        {0,0,0,0},
        {0,1,1,0},
        {1,1,0,0},
        {0,0,0,0}
    },
    {
        {1,0,0,0},
        {1,1,0,0},
        {0,1,0,0},
        {0,0,0,0}
    }
},

/* ================= Z ================= */
{
    {
        {1,1,0,0},
        {0,1,1,0},
        {0,0,0,0},
        {0,0,0,0}
    },
    {
        {0,0,1,0},
        {0,1,1,0},
        {0,1,0,0},
        {0,0,0,0}
    },
    {
        {0,0,0,0},
        {1,1,0,0},
        {0,1,1,0},
        {0,0,0,0}
    },
    {
        {0,1,0,0},
        {1,1,0,0},
        {1,0,0,0},
        {0,0,0,0}
    }
},

/* ================= J ================= */
{
    {
        {1,0,0,0},
        {1,1,1,0},
        {0,0,0,0},
        {0,0,0,0}
    },
    {
        {0,1,1,0},
        {0,1,0,0},
        {0,1,0,0},
        {0,0,0,0}
    },
    {
        {0,0,0,0},
        {1,1,1,0},
        {0,0,1,0},
        {0,0,0,0}
    },
    {
        {0,1,0,0},
        {0,1,0,0},
        {1,1,0,0},
        {0,0,0,0}
    }
},

/* ================= L ================= */
{
    {
        {0,0,1,0},
        {1,1,1,0},
        {0,0,0,0},
        {0,0,0,0}
    },
    {
        {0,1,0,0},
        {0,1,0,0},
        {0,1,1,0},
        {0,0,0,0}
    },
    {
        {0,0,0,0},
        {1,1,1,0},
        {1,0,0,0},
        {0,0,0,0}
    },
    {
        {1,1,0,0},
        {0,1,0,0},
        {0,1,0,0},
        {0,0,0,0}
    }
}
};

const uint8_t TETRIS_FONT[][LETTER_H] = {
    // T
    {
        0b11111,
        0b00100,
        0b00100,
        0b00100,
        0b00100
    },
    // E
    {
        0b11111,
        0b10000,
        0b11110,
        0b10000,
        0b11111
    },
    // T
    {
        0b11111,
        0b00100,
        0b00100,
        0b00100,
        0b00100
    },
    // R
    {
        0b11110,
        0b10001,
        0b11110,
        0b10100,
        0b10010
    },
    // I
    {
        0b11111,
        0b00100,
        0b00100,
        0b00100,
        0b11111
    },
    // S
    {
        0b01111,
        0b10000,
        0b01110,
        0b00001,
        0b11110
    }
};

const uint8_t SPAWN_ROT[PIECE_COUNT] = {
    0, // I
    0, // O
    2, // T
    0, // S
    0, // Z
    2, // J
    2  // L
};

volatile uint32_t gameTick = 0;

typedef struct {
    int8_t row;
    int8_t col;
    uint8_t type;       // 0..PIECE_COUNT-1
    uint8_t rotation;   // 0..3
    uint16_t color;
} ActivePiece_t;
ActivePiece_t active;

uint16_t board[BOARD_ROWS][BOARD_COLS];

static uint32_t lockStart = 0;
static uint8_t  lockResets = 0;

uint8_t pieceBag[PIECE_COUNT];
uint8_t bagIndex = 0;
uint8_t nextPiece;

const uint16_t LINE_SCORE[5] = {
    0,      // 0
    100,    // 1
    300,    // 2
    500,    // 3
    800     // 4
};

uint32_t score = 0;
uint32_t softdrop_score = 0;
uint16_t level = 0;
uint16_t totalLines = 0;
uint32_t gravity_ms = 800;
uint32_t softdrop_ms = 50;

static uint32_t lastScore = 0xFFFFFFFF;
static uint16_t lastLines = 0xFFFF;
static uint16_t lastLevel = 0xFFFF;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void DrawButtonTestUI(void);
uint8_t DebounceButton(Button_t *btn, uint8_t raw);
void UpdateButtons(void);
void UpdateButtonUI(void);

void StartGame();
void DrawLogoBlock(int x, int y, int size, uint16_t color);
void DrawTetrisLogo(int startX, int startY);
void DrawMenu(void);
void MenuInput(void);
void DrawGameOver(void);

void GameTick_Update(void);
void HandlePieceInput(void);

uint16_t ColorLight(uint16_t c);
uint16_t ColorDark(uint16_t c);
void DrawBlock(uint8_t row, uint8_t col, uint16_t color);
void DrawBlock3D(uint8_t row, uint8_t col, uint16_t color);
void ClearBlock(uint8_t row, uint8_t col);
void DrawBoard(void);
void DrawPiece(TetrisPiece piece, uint8_t rotation, int8_t boardRow,
		int8_t boardCol, uint16_t color);
void TestPieces(void);

int8_t GetSpawnOffsetY(uint8_t type, uint8_t rotation);
uint8_t SpawnPiece(void);
void DrawActivePiece(uint16_t color);
void ClearActivePiece(void);
void MoveLeft(void);
void MoveRight(void);
uint8_t MoveDown(void);
void RotatePieceCW(void);
void RotatePieceCCW(void);

uint8_t CheckCollision(int8_t row, int8_t col, uint8_t rotation);
uint8_t IsGrounded(void);
uint8_t IsGroundedAt(int8_t row, int8_t col, uint8_t rotation);
void LockPiece(void);
void ResetLockDelay(void);
void ClearLockDelay(void);

uint8_t ClearLines(void);
void RemoveLine(int row);
void RedrawBoardFromArray(void);

void ShuffleBag(void);
uint8_t GetNextPiece(void);
void DrawNextPiecePreview(void);

void AddScore(uint8_t lines);
void UpdateLevel(void);
void UpdateGravity(void);

void DrawHUD(void);
void UpdateHUD(void);
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
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
	DWT->CYCCNT = 0;                                // Clear counter
	DWT->CTRL = DWT_CTRL_CYCCNTENA_Msk;             // Enable counter
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
//  HAL_TIM_Base_Start_IT(&htim1);

  LCD_init();

  memset(board, 0, sizeof(board));

  LCD_Fill(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, COLOR_BG);
//  DrawButtonTestUI();
	for (int r = 0; r < BOARD_ROWS; r++) {
		for (int c = 0; c < BOARD_COLS; c++) {
			board[r][c] = 0;
		}
	}

	ResetLockDelay();

	srand(HAL_GetTick());
	ShuffleBag();

	DrawBoard();
	nextPiece = GetNextPiece();
	SpawnPiece();
	DrawActivePiece(active.color);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  LCD_Test();
//	  LCD_Fill(0, 0, LCD_WIDTH-1, LCD_HEIGHT-1, 0xF800); // RED
//	  HAL_Delay(800);
//
//	  LCD_Fill(0, 0, LCD_WIDTH-1, LCD_HEIGHT-1, 0x07E0); // GREEN
//	  HAL_Delay(800);
//
//	  LCD_Fill(0, 0, LCD_WIDTH-1, LCD_HEIGHT-1, 0x001F); // BLUE
//	  HAL_Delay(800);
//	  LCD_Fill(0, 0, 239, 319, COLOR_BG);
//	  TestPieces();
//		if (btn_changed) {
//			btn_changed = 0;
//			UpdateButtons();
//			UpdateButtonUI();
//		}
//	  GameTick_Update();

		switch (gameState) {

		case STATE_MENU:
			if (!uiDrawn) {
				DrawMenu();
				uiDrawn = 1;
			}
			MenuInput();
			UpdateButtons();
			if (btn_a.pressed) {
				StartGame();
				uiDrawn = 0;
				gameState = STATE_PLAY;
			}
			break;

		case STATE_PLAY:
			GameTick_Update();
			UpdateHUD();
			break;

		case STATE_GAMEOVER:
			if (!uiDrawn) {
				DrawGameOver();
				uiDrawn = 1;
			}
			UpdateButtons();
			if (btn_a.pressed) {
				uiDrawn = 0;
				gameState = STATE_MENU;
			}
			break;
		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 9999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_DC_Pin|LCD_RST_Pin|LCD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC10
                           PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DC_Pin LCD_RST_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin|LCD_RST_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin >= GPIO_PIN_10 && GPIO_Pin <= GPIO_PIN_15)
    {
        btn_changed = 1;
    }
}

uint8_t DebounceButton(Button_t *btn, uint8_t raw)
{
    uint32_t now = HAL_GetTick();

    btn->pressed  = 0;
    btn->released = 0;

    if (raw != btn->state)
    {
        if ((now - btn->last_change_ms) >= DEBOUNCE_MS)
        {
            btn->last_change_ms = now;

            if (raw)
                btn->pressed = 1;
            else
                btn->released = 1;

            btn->state = raw;
            return 1;
        }
    }
    else
    {
        btn->last_change_ms = now;
    }

    return 0;
}
void UpdateButtons(void)
{
	DebounceButton(&btn_up, RAW_UP);
	DebounceButton(&btn_down, RAW_DOWN);
	DebounceButton(&btn_left, RAW_LEFT);
	DebounceButton(&btn_right, RAW_RIGHT);
	DebounceButton(&btn_a, RAW_A);
	DebounceButton(&btn_b, RAW_B);
}

void DrawButtonTestUI(void)
{
    LCD_Fill(0, 0, 239, 319, 0x0000); // black background

    // D-pad
    LCD_Fill(80,  40, 160, 100, 0x7BEF); // UP
    LCD_Fill(80, 140, 160, 200, 0x7BEF); // DOWN
    LCD_Fill(20,  90,  80, 150, 0x7BEF); // LEFT
    LCD_Fill(160, 90, 220, 150, 0x7BEF); // RIGHT

    // A / B
    LCD_Fill(40,  240, 100, 300, 0x7BEF); // A
    LCD_Fill(140, 240, 200, 300, 0x7BEF); // B
}
void UpdateButtonUI(void)
{
    static uint8_t u,d,l,r,a,b;

	uint8_t nu = btn_up.state;
	uint8_t nd = btn_down.state;
	uint8_t nl = btn_left.state;
	uint8_t nr = btn_right.state;
	uint8_t na = btn_a.state;
	uint8_t nb = btn_b.state;

    if (nu != u) {
        LCD_Fill(80, 40, 160, 100, nu ? 0x07E0 : 0x7BEF);
        u = nu;
    }

    if (nd != d) {
        LCD_Fill(80, 140, 160, 200, nd ? 0xF800 : 0x7BEF);
        d = nd;
    }

    if (nl != l) {
        LCD_Fill(20, 90, 80, 150, nl ? 0x001F : 0x7BEF);
        l = nl;
    }

    if (nr != r) {
        LCD_Fill(160, 90, 220, 150, nr ? 0xFFE0 : 0x7BEF);
        r = nr;
    }

    if (na != a) {
        LCD_Fill(40, 240, 100, 300, na ? 0xF81F : 0x7BEF);
        a = na;
    }

    if (nb != b) {
        LCD_Fill(140, 240, 200, 300, nb ? 0x07FF : 0x7BEF);
        b = nb;
    }
}

void StartGame(void)
{
	LCD_Fill(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, COLOR_BG);
	for (int r = 0; r < BOARD_ROWS; r++) {
		for (int c = 0; c < BOARD_COLS; c++) {
			board[r][c] = 0;
		}
	}

	gravity_ms = 800;
	softdrop_ms = 50;

	score = 0;
	totalLines = 0;
	level = menuLevel;

    UpdateGravity();

	lastScore = 0xFFFFFFFF;
	lastLines = 0xFFFF;
	lastLevel = 0xFFFF;

	ResetLockDelay();

	srand(HAL_GetTick());
	ShuffleBag();

	DrawBoard();
	DrawHUD();
	UpdateHUD();

	nextPiece = GetNextPiece();
	SpawnPiece();
	DrawActivePiece(active.color);
}
void DrawLogoBlock(int x, int y, int size, uint16_t color)
{
    uint16_t light = ColorLight(color);
    uint16_t dark  = ColorDark(color);
    uint8_t b = size / 5;

    LCD_Fill(x + b, y + b,
             x + size - b - 1,
             y + size - b - 1,
             color);

    LCD_Fill(x, y, x + size - 1, y + b - 1, light);

    LCD_Fill(x, y, x + b - 1, y + size - 1, light);

    LCD_Fill(x, y + size - b,
             x + size - 1, y + size - 1, dark);

    LCD_Fill(x + size - b, y,
             x + size - 1, y + size - 1, dark);
}
void DrawTetrisLogo(int startX, int startY)
{
    const uint16_t logoColors[6] = {
        0xF800, // Red
        0xFC00, // Orange
        0xFFE0, // Yellow
        0x07E0, // Green
        0x001F, // Blue
        0x780F  // Purple
    };

    for (int letter = 0; letter < 6; letter++) {

        uint16_t color = logoColors[letter];

        for (int r = 0; r < LETTER_H; r++) {
            for (int c = 0; c < LETTER_W; c++) {

                if (TETRIS_FONT[letter][r] & (1 << (LETTER_W - 1 - c))) {

                    DrawLogoBlock(
                        startX + (letter * (LETTER_W + 1) + c) * LOGO_BLOCK,
                        startY + r * LOGO_BLOCK,
                        LOGO_BLOCK,
                        color
                    );
                }
            }
        }
    }
}
void DrawMenu(void)
{
    char buf[20];

    LCD_Fill(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, COLOR_BG);

    DrawTetrisLogo(10, 50);

    LCD_PutStr(20, 120, "PRESS A TO START", &FONT_8X12, COLOR_TEXT, COLOR_BG);

    sprintf(buf, "START LEVEL: %d", menuLevel);
    LCD_PutStr(20, 160, buf, &FONT_8X12, COLOR_TEXT, COLOR_BG);

    LCD_PutStr(20, 185, "<  > CHANGE LEVEL", &FONT_8X12, COLOR_TEXT, COLOR_BG);
}
void MenuInput(void)
{
    if (btn_left.pressed) {
        if (menuLevel > 0) {
            menuLevel--;
            DrawMenu();
        }
    }

    if (btn_right.pressed) {
        if (menuLevel < 10) {
            menuLevel++;
            DrawMenu();
        }
    }
}
void DrawGameOver(void)
{
    char buf[32];

    LCD_Fill(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, COLOR_BG);

    LCD_PutStr(20, 60, "GAME OVER", &FONT_16X26, COLOR_TEXT, COLOR_BG);

    sprintf(buf, "SCORE : %lu", score);
    LCD_PutStr(30, 120, buf, &FONT_12X20, COLOR_TEXT, COLOR_BG);

    sprintf(buf, "LINES : %u", totalLines);
    LCD_PutStr(30, 150, buf, &FONT_12X20, COLOR_TEXT, COLOR_BG);

    sprintf(buf, "LEVEL : %u", level);
    LCD_PutStr(30, 180, buf, &FONT_12X20, COLOR_TEXT, COLOR_BG);

    LCD_PutStr(10, 230, "PRESS A TO MENU", &FONT_8X12, COLOR_TEXT, COLOR_BG);
}
uint16_t ColorLight(uint16_t c)
{
    uint16_t r = (c >> 11) & 0x1F;
    uint16_t g = (c >> 5)  & 0x3F;
    uint16_t b = c & 0x1F;

    r = (r * 5 + 31) / 6;
    g = (g * 5 + 63) / 6;
    b = (b * 5 + 31) / 6;

    return (r << 11) | (g << 5) | b;
}
uint16_t ColorDark(uint16_t c)
{
    uint16_t r = (c >> 11) & 0x1F;
    uint16_t g = (c >> 5)  & 0x3F;
    uint16_t b = c & 0x1F;

    r = (r * 2) / 3;
    g = (g * 2) / 3;
    b = (b * 2) / 3;

    return (r << 11) | (g << 5) | b;
}
void DrawBlock(uint8_t row, uint8_t col, uint16_t color)
{
    uint16_t x = BOARD_X + col * BLOCK_SIZE;
    uint16_t y = BOARD_Y + row * BLOCK_SIZE;

    LCD_Fill(
        x,
        y,
        x + BLOCK_SIZE - 1,
        y + BLOCK_SIZE - 1,
        color
    );
}
void DrawBlock3D(uint8_t row, uint8_t col, uint16_t color)
{
    int x = BOARD_X + col * BLOCK_SIZE;
    int y = BOARD_Y + row * BLOCK_SIZE;

    uint16_t light = ColorLight(color);
    uint16_t dark  = ColorDark(color);

    // Center
    LCD_Fill(x + BORDER, y + BORDER,
             x + BLOCK_SIZE - BORDER - 1,
             y + BLOCK_SIZE - BORDER - 1,
             color);

    // Top highlight
    LCD_Fill(x, y,
             x + BLOCK_SIZE - 1, y + BORDER - 1,
             light);

    // Left highlight
    LCD_Fill(x, y,
             x + BORDER - 1, y + BLOCK_SIZE - 1,
             light);

    // Bottom shadow
    LCD_Fill(x, y + BLOCK_SIZE - BORDER,
             x + BLOCK_SIZE - 1, y + BLOCK_SIZE - 1,
             dark);

    // Right shadow
    LCD_Fill(x + BLOCK_SIZE - BORDER, y,
             x + BLOCK_SIZE - 1, y + BLOCK_SIZE - 1,
             dark);
}
void ClearBlock(uint8_t row, uint8_t col)
{
    int x = BOARD_X + col * BLOCK_SIZE;
    int y = BOARD_Y + row * BLOCK_SIZE;

    LCD_Fill(x, y,
             x + BLOCK_SIZE - 1,
             y + BLOCK_SIZE - 1,
             COLOR_BG);
}

void DrawBoard(void)
{
    // BG
    LCD_Fill(
        BOARD_X,
        BOARD_Y,
        BOARD_X + BOARD_WIDTH - 1,
        BOARD_Y + BOARD_HEIGHT - 1,
        COLOR_BG
    );

//    // Vertical grid lines
//    for (int c = 0; c <= BOARD_COLS; c++)
//    {
//        uint16_t x = BOARD_X + c * BLOCK_SIZE;
//        LCD_DrawLine(
//            x,
//            BOARD_Y,
//            x,
//            BOARD_Y + BOARD_HEIGHT,
//            COLOR_GRID
//        );
//    }
//
//    // Horizontal grid lines
//    for (int r = 0; r <= BOARD_ROWS; r++)
//    {
//        uint16_t y = BOARD_Y + r * BLOCK_SIZE;
//        LCD_DrawLine(
//            BOARD_X,
//            y,
//            BOARD_X + BOARD_WIDTH,
//            y,
//            COLOR_GRID
//        );
//    }

    // Outer border
    LCD_DrawLine(BOARD_X - 1, BOARD_Y - 1,
                 BOARD_X + BOARD_WIDTH, BOARD_Y - 1, COLOR_BORDER);
    LCD_DrawLine(BOARD_X - 1, BOARD_Y + BOARD_HEIGHT,
                 BOARD_X + BOARD_WIDTH, BOARD_Y + BOARD_HEIGHT, COLOR_BORDER);
    LCD_DrawLine(BOARD_X - 1, BOARD_Y - 1,
                 BOARD_X - 1, BOARD_Y + BOARD_HEIGHT, COLOR_BORDER);
    LCD_DrawLine(BOARD_X + BOARD_WIDTH, BOARD_Y - 1,
                 BOARD_X + BOARD_WIDTH, BOARD_Y + BOARD_HEIGHT, COLOR_BORDER);
}

void DrawPiece(TetrisPiece piece, uint8_t rotation, int8_t boardRow,
		int8_t boardCol, uint16_t color) {
	rotation = (rotation + 4) & 0x03;
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			if (TETROMINO[piece][rotation][r][c]) {
				DrawBlock3D(boardRow + r, boardCol + c, color);
			}
		}
	}
}
void TestPieces(void) {
	int row = 0;

	for (int p = 0; p < PIECE_COUNT; p++) {
		for (int r = 0; r < 4; r++) {
			DrawPiece3D((TetrisPiece) p, r, row, 0, 0x07E0   // green
					);
			row += 5;
		}
		row = 0;
		HAL_Delay(300);
		DrawBoard();
	}
}

int8_t GetSpawnOffsetY(uint8_t type, uint8_t rotation)
{
    for (int8_t r = 0; r < 4; r++) {
        for (int8_t c = 0; c < 4; c++) {
            if (TETROMINO[type][rotation][r][c]) {
                return -r;
            }
        }
    }
    return 0;
}
uint8_t SpawnPiece(void)
{
    active.type = nextPiece;
    active.rotation = SPAWN_ROT[active.type];
    active.row = GetSpawnOffsetY(active.type, active.rotation);
    active.col = (BOARD_COLS / 2) - 2;
    active.color = PIECE_COLOR[active.type];

    if (CheckCollision(active.row, active.col, active.rotation)) {
            return 0;
    }
    DrawActivePiece(active.color);
    nextPiece = GetNextPiece();
    DrawNextPiecePreview();
    return 1;
}

void DrawActivePiece(uint16_t color)
{
    for (uint8_t r = 0; r < 4; r++) {
        for (uint8_t c = 0; c < 4; c++) {
            if (TETROMINO[active.type][active.rotation][r][c]) {
                DrawBlock3D(active.row + r,
                          active.col + c,
                          color);
            }
        }
    }
}
void ClearActivePiece(void)
{
    for (uint8_t r = 0; r < 4; r++) {
        for (uint8_t c = 0; c < 4; c++) {

            if (!TETROMINO[active.type][active.rotation][r][c])
                continue;

            ClearBlock(active.row + r, active.col + c);
        }
    }
}


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim->Instance == TIM1)
//    {
//        gameTick++;
//    }
//}
void GameTick_Update(void)
{
	static uint32_t lastFall = 0;
	static uint32_t lastMoveLR = 0;
	static uint32_t holdStartLR = 0;
	static int8_t holdDir = 0;

	uint32_t now = HAL_GetTick();

    UpdateButtons();

    if (btn_a.pressed)
		RotatePieceCW();
	if (btn_b.pressed)
		RotatePieceCCW();

	if (btn_left.pressed) {
		MoveLeft();
		holdDir = -1;
		holdStartLR = now;
		lastMoveLR = now;
	} else if (btn_right.pressed) {
		MoveRight();
		holdDir = +1;
		holdStartLR = now;
		lastMoveLR = now;
	} else if (!btn_left.state && !btn_right.state) {
		holdDir = 0;
	}

	// Auto repeat after DAS
	if (holdDir != 0 && (now - holdStartLR) >= DAS_DELAY_MS) {
		if (now - lastMoveLR >= ARR_MS) {
			lastMoveLR = now;
			if (holdDir < 0)
				MoveLeft();
			else
				MoveRight();
		}
	}

	uint32_t fallInterval = btn_down.state ? softdrop_ms : gravity_ms;
	if (now - lastFall >= fallInterval) {
		lastFall = now;
		MoveDown();
	}

	if (IsGrounded() && lockStart == 0) {
	    lockStart = HAL_GetTick();
	    lockResets = 0;
	}
	if (lockStart && (HAL_GetTick() - lockStart) >= LOCK_DELAY_MS) {
	    LockPiece();
	    ClearLockDelay();
	    holdDir = 0;
	}
}


void MoveLeft(void)
{
    if (!CheckCollision(active.row, active.col - 1, active.rotation)) {
    	ClearActivePiece();
        active.col--;
        DrawActivePiece(active.color);
        ResetLockDelay();
    }
}
void MoveRight(void)
{
    if (!CheckCollision(active.row, active.col + 1, active.rotation)) {
    	ClearActivePiece();
        active.col++;
        DrawActivePiece(active.color);
        ResetLockDelay();
    }
}
uint8_t MoveDown(void)
{
    if (!CheckCollision(active.row + 1, active.col, active.rotation)) {
    	ClearActivePiece();
        active.row++;
        DrawActivePiece(active.color);

        if (btn_down.state) {
            softdrop_score += 1;
        }

        ClearLockDelay();
        return 1;
    }

    if (lockStart == 0) {
        lockStart = HAL_GetTick();
        lockResets = 0;
    }

    return 0;
}
void RotatePieceCW(void)
{
    uint8_t next = (active.rotation + 1) & 0x03;

    const int8_t kicks[][2] = {
        { 0,  0},
        { 1,  0},
        {-1,  0},
        { 2,  0},
        {-2,  0},
        { 0,  1},
        { 0, -1},
    };

    for (uint8_t i = 0; i < sizeof(kicks)/sizeof(kicks[0]); i++) {

        int8_t nr = active.row + kicks[i][1];
        int8_t nc = active.col + kicks[i][0];

        if (CheckCollision(nr, nc, next))
            continue;

        ClearActivePiece();
        active.row = nr;
        active.col = nc;
        active.rotation = next;
        DrawActivePiece(active.color);

        ResetLockDelay();
        return;
    }
}
void RotatePieceCCW(void)
{
    uint8_t next = (active.rotation + 3) & 0x03;

    const int8_t kicks[][2] = {
        { 0,  0},
        { 1,  0},
        {-1,  0},
        { 2,  0},
        {-2,  0},
        { 0,  1},
        { 0, -1},
    };

    for (uint8_t i = 0; i < sizeof(kicks)/sizeof(kicks[0]); i++) {

        int8_t nr = active.row + kicks[i][1];
        int8_t nc = active.col + kicks[i][0];

        if (CheckCollision(nr, nc, next))
            continue;

        ClearActivePiece();
        active.row = nr;
        active.col = nc;
        active.rotation = next;
        DrawActivePiece(active.color);

        ResetLockDelay();
        return;
    }
}

uint8_t CheckCollision(int8_t row, int8_t col, uint8_t rotation)
{
    for (uint8_t r = 0; r < 4; r++) {
        for (uint8_t c = 0; c < 4; c++) {

            if (!TETROMINO[active.type][rotation][r][c])
                continue;

            int br = row + r;
            int bc = col + c;

            // Out of bounds
            if (bc < 0 || bc >= BOARD_COLS ||
                br < 0 || br >= BOARD_ROWS)
                return 1;

            // Hit existing block
            if (board[br][bc] != 0)
                return 1;
        }
    }
    return 0;
}
uint8_t IsGrounded(void)
{
    return CheckCollision(active.row + 1, active.col, active.rotation);
}
uint8_t IsGroundedAt(int8_t row, int8_t col, uint8_t rotation)
{
    return CheckCollision(row + 1, col, rotation);
}

void LockPiece(void)
{
    for (uint8_t r = 0; r < 4; r++) {
        for (uint8_t c = 0; c < 4; c++) {

            if (TETROMINO[active.type][active.rotation][r][c]) {

                int br = active.row + r;
                int bc = active.col + c;

                if (br >= 0 && br < BOARD_ROWS &&
                    bc >= 0 && bc < BOARD_COLS) {
                    board[br][bc] = active.color;
                }
            }
        }
    }

    uint8_t cleared = ClearLines();

    if (cleared > 0) {
        AddScore(cleared);
        UpdateLevel();
    }

    ClearLockDelay();

    if (!SpawnPiece()) {
        gameState = STATE_GAMEOVER;
    }
}
void ResetLockDelay(void)
{
	if (IsGrounded())
		return;

	if (lockResets >= LOCK_RESET_LIMIT)
		return;

	lockStart = 0;
	lockResets++;
}
void ClearLockDelay(void)
{
    lockStart = 0;
    lockResets = 0;
}

uint8_t ClearLines(void)
{
    uint8_t linesCleared = 0;

    for (int r = BOARD_ROWS - 1; r >= 0; r--) {

        uint8_t full = 1;
        for (int c = 0; c < BOARD_COLS; c++) {
            if (board[r][c] == 0) {
                full = 0;
                break;
            }
        }

        if (full) {
            RemoveLine(r);
            linesCleared++;
            r++;
        }
    }

    if (linesCleared > 0) {
        RedrawBoardFromArray();
    }

    return linesCleared;
}
void RemoveLine(int row)
{
    for (int r = row; r > 0; r--) {
        for (int c = 0; c < BOARD_COLS; c++) {
            board[r][c] = board[r - 1][c];
        }
    }

    for (int c = 0; c < BOARD_COLS; c++) {
        board[0][c] = 0;
    }
}
void RedrawBoardFromArray(void)
{
    for (int r = 0; r < BOARD_ROWS; r++) {
        for (int c = 0; c < BOARD_COLS; c++) {
			if (board[r][c]) {
				DrawBlock3D(r, c, board[r][c]);
			} else {
				ClearBlock(r, c);
			}
        }
    }
}

void ShuffleBag(void)
{
    for (uint8_t i = 0; i < PIECE_COUNT; i++)
        pieceBag[i] = i;

    // Fisher–Yates
    for (uint8_t i = PIECE_COUNT - 1; i > 0; i--) {
        uint8_t j = rand() % (i + 1);
        uint8_t t = pieceBag[i];
        pieceBag[i] = pieceBag[j];
        pieceBag[j] = t;
    }

    bagIndex = 0;
}
uint8_t GetNextPiece(void)
{
    if (bagIndex >= PIECE_COUNT) {
        ShuffleBag();
    }

    return pieceBag[bagIndex++];
}
void DrawNextPiecePreview(void)
{
    // Clear
    LCD_Fill(PREVIEW_X - 2,
             PREVIEW_Y - 2,
             PREVIEW_X + PREVIEW_BLOCK * 4 + 2,
             PREVIEW_Y + PREVIEW_BLOCK * 4 + 2,
             COLOR_BG);

    // Border
    LCD_DrawLine(PREVIEW_X - 2, PREVIEW_Y - 2,
                 PREVIEW_X + PREVIEW_BLOCK * 4 + 2, PREVIEW_Y - 2, COLOR_BORDER);
    LCD_DrawLine(PREVIEW_X - 2, PREVIEW_Y + PREVIEW_BLOCK * 4 + 2,
                 PREVIEW_X + PREVIEW_BLOCK * 4 + 2, PREVIEW_Y + PREVIEW_BLOCK * 4 + 2, COLOR_BORDER);
    LCD_DrawLine(PREVIEW_X - 2, PREVIEW_Y - 2,
                 PREVIEW_X - 2, PREVIEW_Y + PREVIEW_BLOCK * 4 + 2, COLOR_BORDER);
    LCD_DrawLine(PREVIEW_X + PREVIEW_BLOCK * 4 + 2, PREVIEW_Y - 2,
                 PREVIEW_X + PREVIEW_BLOCK * 4 + 2, PREVIEW_Y + PREVIEW_BLOCK * 4 + 2, COLOR_BORDER);

	DrawPiece(nextPiece, 0, PREVIEW_BOARD_ROW, PREVIEW_BOARD_COL,
			PIECE_COLOR[nextPiece]);
}

void AddScore(uint8_t lines)
{

    if (lines <= 4) {
        score += LINE_SCORE[lines] * level;
        score += softdrop_score;
        softdrop_score = 0;
    }

    totalLines += lines;
}
void UpdateLevel(void)
{
    uint16_t newLevel = menuLevel + (totalLines / 10);

    if (newLevel != level) {
        level = newLevel;
        UpdateGravity();
    }
}
void UpdateGravity(void)
{
    if (level == 0)          gravity_ms = 800;
    else if (level == 1)     gravity_ms = 717;
    else if (level == 2)     gravity_ms = 633;
    else if (level == 3)     gravity_ms = 550;
    else if (level == 4)     gravity_ms = 467;
    else if (level == 5)     gravity_ms = 383;
    else if (level == 6)     gravity_ms = 300;
    else if (level == 7)     gravity_ms = 217;
    else if (level == 8)     gravity_ms = 133;
    else if (level == 9)     gravity_ms = 100;
	else if (level <= 12) {
		gravity_ms = 83;
		softdrop_ms = 42;
	} else if (level <= 15) {
		gravity_ms = 67;
		softdrop_ms = 34;
	} else if (level <= 18) {
		gravity_ms = 50;
		softdrop_ms = 25;
	} else if (level <= 28) {
		gravity_ms = 33;
		softdrop_ms = 17;
	} else {
		gravity_ms = 17;
		softdrop_ms = 9;
	}
}

void DrawHUD(void)
{
    LCD_PutStr(UI_X, UI_Y + 0,   "SCORE", &FONT_12X20, COLOR_TEXT, COLOR_BG);
    LCD_PutStr(UI_X, UI_Y + 60,  "LINES", &FONT_12X20, COLOR_TEXT, COLOR_BG);
    LCD_PutStr(UI_X, UI_Y + 120,  "LEVEL", &FONT_12X20, COLOR_TEXT, COLOR_BG);
}
void UpdateHUD(void)
{
    char buf[16];

    if (score != lastScore) {
        sprintf(buf, "%lu   ", score);
        LCD_PutStr(UI_X, UI_Y + 25, buf, &FONT_8X12, COLOR_TEXT, COLOR_BG);
        lastScore = score;
    }

    if (totalLines != lastLines) {
        sprintf(buf, "%u   ", totalLines);
        LCD_PutStr(UI_X, UI_Y + 85, buf, &FONT_8X12, COLOR_TEXT, COLOR_BG);
        lastLines = totalLines;
    }

    if (level != lastLevel) {
        sprintf(buf, "%u   ", level);
        LCD_PutStr(UI_X, UI_Y + 145, buf, &FONT_8X12, COLOR_TEXT, COLOR_BG);
        lastLevel = level;
    }
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
