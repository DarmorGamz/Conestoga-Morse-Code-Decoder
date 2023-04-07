/********************************************
Author: Darren Morrison
Email: dmorrison8832@conestogac.on.ca
SN: 8258832
*********************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
#include <stdio.h>
#include "stdbool.h"
#include <string.h>

#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
#define DOT_THRESHOLD       1000
#define DASH_THRESHOLD      3000
#define GAP_INTRA_LETTER    1000
#define GAP_LETTER          3000
#define GAP_WORD            7000
#define INPUT_FINISH        10000

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "Task1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
  .name = "Task2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task3 */
osThreadId_t Task3Handle;
const osThreadAttr_t Task3_attributes = {
  .name = "Task3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task4 */
osThreadId_t Task4Handle;
const osThreadAttr_t Task4_attributes = {
  .name = "Task4",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Queue1 */
osMessageQueueId_t Queue1Handle;
const osMessageQueueAttr_t Queue1_attributes = {
  .name = "Queue1"
};
/* Definitions for Queue2_1 */
osMessageQueueId_t Queue2_1Handle;
const osMessageQueueAttr_t Queue2_1_attributes = {
  .name = "Queue2_1"
};
/* Definitions for Queue2_2 */
osMessageQueueId_t Queue2_2Handle;
const osMessageQueueAttr_t Queue2_2_attributes = {
  .name = "Queue2_2"
};
/* Definitions for IndicateToReciever */
osSemaphoreId_t IndicateToRecieverHandle;
const osSemaphoreAttr_t IndicateToReciever_attributes = {
  .name = "IndicateToReciever"
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void Task1_Temp(void *argument);
void Task2_Temp(void *argument);
void Task3_Temp(void *argument);
void Task4_Temp(void *argument);

/* Private user code ---------------------------------------------------------*/
typedef struct {
    const char* morse;
    char character;
} MorseCode;

typedef enum {
    SIGNAL_NONE,
    SIGNAL_DOT,
    SIGNAL_DASH,
	SIGNAL_SPACE,
	SIGNAL_END
} MorseSignal;

static const MorseCode morseLookupTable[] = {
    { ".-", 'A' },
    { "-...", 'B' },
    { "-.-.", 'C' },
    { "-..", 'D' },
    { ".", 'E' },
    { "..-.", 'F' },
    { "--.", 'G' },
    { "....", 'H' },
    { "..", 'I' },
    { ".---", 'J' },
    { "-.-", 'K' },
    { ".-..", 'L' },
    { "--", 'M' },
    { "-.", 'N' },
    { "---", 'O' },
    { ".--.", 'P' },
    { "--.-", 'Q' },
    { ".-.", 'R' },
    { "...", 'S' },
    { "-", 'T' },
    { "..-", 'U' },
    { "...-", 'V' },
    { ".--", 'W' },
    { "-..-", 'X' },
    { "-.--", 'Y' },
    { "--..", 'Z' },
    { "-----", '0' },
    { ".----", '1' },
    { "..---", '2' },
    { "...--", '3' },
    { "....-", '4' },
    { ".....", '5' },
    { "-....", '6' },
    { "--...", '7' },
    { "---..", '8' },
    { "----.", '9' },
};

int main(void) {
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* Init scheduler */
  osKernelInitialize();

  /* Create the semaphores(s) */
  IndicateToRecieverHandle = osSemaphoreNew(1, 1, &IndicateToReciever_attributes);
  osSemaphoreAcquire(IndicateToRecieverHandle, portMAX_DELAY); // Ensure receiver is prevented from displaying morse until message is complete

  /* Create the queue(s) */
  Queue1Handle = osMessageQueueNew (16, sizeof(uint16_t), &Queue1_attributes);
  Queue2_1Handle = osMessageQueueNew (16, sizeof(uint16_t), &Queue2_1_attributes);
  Queue2_2Handle = osMessageQueueNew (16, sizeof(uint16_t), &Queue2_2_attributes);

  /* Create the thread(s) */
  Task1Handle = osThreadNew(Task1_Temp, NULL, &Task1_attributes);
  Task2Handle = osThreadNew(Task2_Temp, NULL, &Task2_attributes);
  Task3Handle = osThreadNew(Task3_Temp, NULL, &Task3_attributes);
  Task4Handle = osThreadNew(Task4_Temp, NULL, &Task4_attributes);

  /* Start scheduler */
  osKernelStart();

  /* Infinite loop */
  while (1) { }

}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON1_Pin */
  GPIO_InitStruct.Pin = BUTTON1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
int _write(int file, char *data, int len) {
    if((file != STDOUT_FILENO) && (file != STDERR_FILENO)) { errno = EBADF; return -1; }

    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t *)data, len, HAL_MAX_DELAY);

    // return # of bytes written or error
    return (status == HAL_OK ? len : EIO);
}

void Task1_Temp(void *argument) {
	// Init vars.
	uint32_t button_press_time = 0; // Rising edge.
	uint32_t button_release_time = 0; // Falling edge.
	bool button_pressed = false; // Used to prevent calculation if there was no rising edge.

	bool sent_space = false;
	bool sent_gap = false;
	bool sent_end = false;
	MorseSignal signal;

	// Init console.
	fflush(stdout);
	printf("\r\n");

	for (;;) {
		// Check if the button is pressed
		if (HAL_GPIO_ReadPin(BUTTON1_GPIO_Port, BUTTON1_Pin) == GPIO_PIN_SET && !button_pressed) {
			button_pressed = true;
			button_press_time = HAL_GetTick();

			sent_space = false;
			sent_gap = false;
			sent_end = false;

		} else if (HAL_GPIO_ReadPin(BUTTON1_GPIO_Port, BUTTON1_Pin) == GPIO_PIN_RESET && button_pressed) {
			// Reset falling edge detect.
			button_pressed = false;

			// Get release time.
			button_release_time = HAL_GetTick();

			// Calculate press duration.
			uint32_t press_duration = button_release_time - button_press_time;

			// Determine signal (dash or dot).
			if (press_duration < DOT_THRESHOLD) { signal = SIGNAL_DOT; }
			else { signal = SIGNAL_DASH; }

			// Enqueue the Morse code signal (dot or dash) to Queue1
			osMessageQueuePut(Queue1Handle, &signal, 0, 0);

		} else {
			if(button_press_time == 0 || button_pressed) { osDelay(100); continue; } // First button press of the signal.

			// Get current tick.
			uint32_t current_time = HAL_GetTick(); // Prevents multiple calls of HAL_GetTick();

			if(button_release_time+GAP_LETTER < current_time && !sent_gap) {
				// Set the signal.
				signal = SIGNAL_NONE;
				// Prevent multiple spaces from being sent.
				sent_gap = true;
				// Enqueue the Morse code signal (space, end, or none) to Queue1
				osMessageQueuePut(Queue1Handle, &signal, 0, 0);

			} else if(button_release_time+GAP_WORD < HAL_GetTick() && !sent_space) {
				// Set the signal.
				signal = SIGNAL_SPACE;
				// Prevent multiple spaces from being sent.
				sent_space = true;
				// Enqueue the Morse code signal (space, end, or none) to Queue1
				osMessageQueuePut(Queue1Handle, &signal, 0, 0);

			} else if(button_release_time+INPUT_FINISH < HAL_GetTick() && !sent_end) {
				// Set the signal.
				signal = SIGNAL_END;
				// Prevent multiple spaces from being sent.
				sent_end = true;
				// Enqueue the Morse code signal (space, end, or none) to Queue1
				osMessageQueuePut(Queue1Handle, &signal, 0, 0);
			}
		}

		// Prevent CPU deadlock.
		osDelay(100);
	}
}

void Task2_Temp(void *argument) {
	// Init vars.
	MorseSignal signal;
	osStatus_t status;
	uint32_t led_on_time = 0;
	bool led_on = false;

	for (;;) {
		// Dequeue a Morse code signal (dot or dash) from Queue1
		status = osMessageQueueGet(Queue1Handle, &signal, NULL, 0);
		if (status == osOK) {
			// Enqueue the signal to Queue2_1 and Queue2_2
			osMessageQueuePut(Queue2_1Handle, &signal, 0, 0);
			osMessageQueuePut(Queue2_2Handle, &signal, 0, 0);

			// Sender Indicator light ignores spaces and letter gaps.
			if(signal == SIGNAL_SPACE || signal == SIGNAL_NONE || signal == SIGNAL_END) {
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				led_on = false;
				continue;
			}

			// Turn on the LED
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
			led_on = true;
			led_on_time = HAL_GetTick();

		} else { // Dash or Dot in Queue.
			if (signal == SIGNAL_DOT && led_on) {
				if(led_on_time+DOT_THRESHOLD <= HAL_GetTick()) {
					// Turn off the LED
					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
					led_on = false;
				}
			} else if (signal == SIGNAL_DASH && led_on) {
				if(led_on_time+DASH_THRESHOLD <= HAL_GetTick()) {
					// Turn off the LED
					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
					led_on = false;
				}
			}
		}
	}
}

void Task3_Temp(void *argument) {
	// Init vars.
	MorseSignal signal;
	osStatus_t status;

	// Buffer to store the Morse code sequence
	char morse_buffer[8] = {0};
	uint8_t buffer_index = 0;

	for (;;) {
		// Dequeue a Morse code signal (dot or dash) from Queue2_1
		status = osMessageQueueGet(Queue2_1Handle, &signal, NULL, 0);
		if (status == osOK) { // Got Dash or Dot
			// Add the signal to the buffer
			if (signal == SIGNAL_DOT) {
				morse_buffer[buffer_index++] = '.';
			} else if (signal == SIGNAL_DASH) {
				morse_buffer[buffer_index++] = '-';
			} else if (signal == SIGNAL_END) {
				osSemaphoreRelease(IndicateToRecieverHandle);
			} else if (signal == SIGNAL_SPACE) {
				printf(" ");
				fflush(stdout);
			} else if (signal == SIGNAL_NONE) {
				// Make sure the buffer is null-terminated
				morse_buffer[buffer_index] = '\0';

				// Decode the Morse code and print the corresponding letter
				for (int i = 0; morseLookupTable[i].morse != NULL; ++i) {
					if (strcmp(morse_buffer, morseLookupTable[i].morse) == 0) {
						printf("%c", morseLookupTable[i].character);
						fflush(stdout);
						break;
					}
				}
				buffer_index = 0;
				memset(morse_buffer, 0, sizeof(morse_buffer));
			}

		} else { }
	}
}

void Task4_Temp(void *argument) {
	// Init vars.
	MorseSignal signal;
	osStatus_t status;

	for(;;) {
		osSemaphoreAcquire(IndicateToRecieverHandle, portMAX_DELAY); // Take semaphore when available, this
		// Dequeue a Morse code signal (dot or dash) from Queue2_1
		if((status = osMessageQueueGet(Queue2_2Handle, &signal, NULL, 0)) == osOK) {
			printf("TASK4: %d\r\n", signal);
			if (signal == SIGNAL_DOT) {
				// Turn on the LED
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
				// Wait for Dot duration.
				HAL_Delay(DOT_THRESHOLD);
				// Turn off the LED
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				// Pause briefly so LED isn't on forever.
				HAL_Delay(GAP_INTRA_LETTER);

			} else if (signal == SIGNAL_DASH) {
				// Turn on the LED
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
				// Wait for Dot duration.
				HAL_Delay(DASH_THRESHOLD);
				// Turn off the LED
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				// Pause briefly so LED isn't on forever.
				HAL_Delay(GAP_INTRA_LETTER);

			}
			else if (signal == SIGNAL_NONE) { HAL_Delay(GAP_LETTER); }
			else if (signal == SIGNAL_SPACE) { HAL_Delay(GAP_WORD); }

			// Continue iterating over the queue.
			if (signal != SIGNAL_END) { osSemaphoreRelease(IndicateToRecieverHandle); }

		} else { }
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) { HAL_IncTick(); }
}

void Error_Handler(void) {
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
	void assert_failed(uint8_t *file, uint32_t line) { }
#endif /* USE_FULL_ASSERT */
