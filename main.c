/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdint.h>
#include <stdlib.h>
#include "stm32f0xx.h"
#include <lcd_stm32f0.h>
#include <lcd_stm32f0.c>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim16;
uint8_t SW0Pressed = 0;
uint8_t SW1Pressed = 0;
uint8_t SW2Pressed = 0;
uint8_t SW3Pressed = 0;
static uint8_t direction = 1; //1 forward

uint32_t cnt = 0;

/* USER CODE BEGIN PV */
GPIO_TypeDef* led_ports[] = {LED0_GPIO_Port, LED1_GPIO_Port, LED2_GPIO_Port, LED3_GPIO_Port, LED4_GPIO_Port, LED5_GPIO_Port, LED6_GPIO_Port, LED7_GPIO_Port};
uint16_t led_pins[] = {LED0_Pin, LED1_Pin, LED2_Pin, LED3_Pin, LED4_Pin, LED5_Pin, LED6_Pin, LED7_Pin};

/* USER CODE END PV */
uint8_t i=0;
uint8_t j=0;
uint8_t k = 0;
uint8_t mode = 0;
// Variables for Mode 3

// Sparkle mode states
typedef enum {
    SPARKLE_ALL_OFF,
    SPARKLE_SET_RANDOM,
    SPARKLE_HOLD_ON,
    SPARKLE_TURNING_OFF
} SparkleState;

 SparkleState sparkle_state = SPARKLE_ALL_OFF;
 uint8_t sparkle_leds = 0;   // current LEDs ON as bitmask
 uint8_t leds_to_turn_off = 0;
 uint32_t sparkle_timer = 0;
 uint32_t sparkle_delay = 0;
 uint16_t check_current_delay=999;

/* USER CODE BEGIN PV */
// TODO: Define input variables


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

int get_random_range(int min, int max);
void turn_on_random_leds(void);
void sw0_delay(uint16_t new_delay);

/* USER CODE BEGIN PFP */
void TIM16_IRQHandler(void);
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
    init_LCD();
    lcd_command(CLEAR);
    lcd_putstring("MLDKHA010");
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
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  // TODO: Start timer TIM16
  HAL_TIM_Base_Start_IT(&htim16);
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (HAL_GPIO_ReadPin(Button0_GPIO_Port, Button0_Pin) == GPIO_PIN_RESET){
      HAL_Delay(10);
      if (HAL_GPIO_ReadPin(Button0_GPIO_Port, Button0_Pin) == GPIO_PIN_RESET){
        if (check_current_delay==999){
          check_current_delay=499;
        }
        else if (check_current_delay==499){
          check_current_delay=999;
        }
        sw0_delay (check_current_delay);
      }
    }
    // TODO: Check pushbuttons to change timer delay
    // Check for Button1 press (assuming pull-up, so it's low when pressed)
    if (HAL_GPIO_ReadPin(Button1_GPIO_Port, Button1_Pin) == GPIO_PIN_RESET) {
        // Debouncing: Wait a short time to make sure the press is stable
        HAL_Delay(10);
        if (HAL_GPIO_ReadPin(Button1_GPIO_Port, Button1_Pin) == GPIO_PIN_RESET) {
          if (k == 1) {
                k = 0;
                //turns off all the leds
                for (int i = 0; i < 8; i++) {
                    HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_RESET);
                }
            }
            // Button is pressed, and we want to toggle the animation state
            j = !j;

            // If the animation is now stopped, turn off all LEDs
            if (j) {
             
                mode = 1; // Set mode to 1
                i = 0;
                direction = 1;
                for (int i = 0; i < 8; i++) {
                    HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_RESET);
                }
                //turns on the LEDs
                HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_SET);
          
            }
            else {
                mode = 0; // Set mode to 0
                for (int i = 0; i < 8; i++) {
                    HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_RESET);
                }
            }
            
            // Wait for the button to be released to prevent multiple toggles
            while (HAL_GPIO_ReadPin(Button1_GPIO_Port, Button1_Pin) == GPIO_PIN_RESET) {}
            HAL_Delay(10); // Debounce on release
        }
    }
    if (HAL_GPIO_ReadPin(Button2_GPIO_Port, Button2_Pin) == GPIO_PIN_RESET) {
        // Debouncing: Wait a short time to make sure the press is stable
        HAL_Delay(10);
        if (HAL_GPIO_ReadPin(Button2_GPIO_Port, Button2_Pin) == GPIO_PIN_RESET) {
          if (j == 1) {
                j = 0;
                for (int i = 0; i < 8; i++) {
                    HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_RESET);
                }
            }
            // Button is pressed, and we want to toggle the animation state
            k = !k;

            // If the animation is now stopped, turn off all LEDs
            if (k) {
              mode = 2; // Set mode to 2
                i = 0;
                direction = 1;
                for (int i = 0; i < 8; i++) {
                    HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_SET);
                }
                HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_RESET);
            }else {
                mode = 0; // Set mode to 0
                for (int i = 0; i < 8; i++) {
                    HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_RESET);
                }
            }
            
            // Wait for the button to be released to prevent multiple toggles
            while (HAL_GPIO_ReadPin(Button2_GPIO_Port, Button2_Pin) == GPIO_PIN_RESET) {}
            HAL_Delay(10); // Debounce on release
        }

    
    // Your main loop can now check other buttons or perform other tasks
    // For example, checking for other buttons (Button2, Button3, etc.)
    // if (HAL_GPIO_ReadPin(Button2_GPIO_Port, Button2_Pin) == GPIO_PIN_RESET) {
    //     // Handle other mode
    // }


  }

  // --- Button 3: Mode 3 (Sparkle) ---
      if (HAL_GPIO_ReadPin(Button3_GPIO_Port, Button3_Pin) == GPIO_PIN_RESET) {
    HAL_Delay(10);  // simple debounce, replace with better method if desired
    if (HAL_GPIO_ReadPin(Button3_GPIO_Port, Button3_Pin) == GPIO_PIN_RESET) {
        mode = 3;
        sparkle_state = SPARKLE_ALL_OFF;
        sparkle_leds = 0;
        leds_to_turn_off = 0;
        sparkle_timer = HAL_GetTick();

        while (HAL_GPIO_ReadPin(Button3_GPIO_Port, Button3_Pin) == GPIO_PIN_RESET) {}
        HAL_Delay(10);
    }
}


      // --- Mode 3 Logic: This section runs the sparkle animation ---
  
  /* USER CODE END 3 */

}
}
uint8_t get_random_led_pattern(void) {
    // Returns random 8-bit pattern (0-255)
    return (uint8_t)(rand() % 256);  // Or your custom RNG
}

uint32_t get_random_delay(uint32_t min_ms, uint32_t max_ms) {
    return min_ms + (rand() % (max_ms - min_ms + 1));
}

int get_random_led_index(uint8_t mask) {
    // Pick random LED ON in mask
    // If no LEDs ON, return -1
    uint8_t count = 0;
    for (int i=0; i<8; i++) if ((mask >> i) & 1) count++;
    if (count == 0) return -1;
    int target = rand() % count;
    for (int i=0; i<8; i++) {
        if ((mask >> i) & 1) {
            if (target == 0) return i;
            target--;
        }
    }
    return -1;
}

void sw0_delay (uint16_t new_delay){
  HAL_TIM_Base_Stop_IT(&htim16);//stop the timer

  __HAL_TIM_SET_AUTORELOAD(&htim16,new_delay);

  HAL_TIM_Base_Start_IT(&htim16);
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 8000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */
  NVIC_EnableIRQ(TIM16_IRQn);
  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);

  /**/
  GPIO_InitStruct.Pin = Button0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED5_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED6_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED6_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED7_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED7_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void TIM16_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim16); // Handle base interrupt logic

  }
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM16) {
        switch (mode) {
            case 1: // Button 1 animation: Single LED ON
                HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_RESET);
                i += direction;
                if (i >= 7) {
                    direction = -1;
                    if (i > 7) i = 7;
                } else if (i <= 0) {
                    direction = 1;
                    if (i < 0) i = 0;
                }
                HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_SET);
                break;

            case 2: // Button 2 animation: Single LED OFF
                HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_SET);
                i += direction;
                if (i >= 7) {
                    direction = -1;
                    if (i > 7) i = 7;
                } else if (i <= 0) {
                    direction = 1;
                    if (i < 0) i = 0;
                }
                HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_RESET);
                break;
            case 3: //button 0 for delay and sped up
            uint32_t now = HAL_GetTick();

        switch (sparkle_state) {
            case SPARKLE_ALL_OFF:
                // Ensure all LEDs off
                for (int i=0; i<8; i++) {
                    HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_RESET);
                }
                sparkle_state = SPARKLE_SET_RANDOM;
                break;

            case SPARKLE_SET_RANDOM:
                // Set a random pattern of LEDs ON
                sparkle_leds = get_random_led_pattern();
                leds_to_turn_off = sparkle_leds;

                // Set LEDs ON according to sparkle_leds bitmask
                for (int i=0; i<8; i++) {
                    if ((sparkle_leds >> i) & 1)
                        HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_SET);
                    else
                        HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_RESET);
                }

                // Hold ON for random delay 100-1500 ms
                sparkle_delay = get_random_delay(100, 1500);
                sparkle_timer = now;
                sparkle_state = SPARKLE_HOLD_ON;
                break;

            case SPARKLE_HOLD_ON:
                if (now - sparkle_timer >= sparkle_delay) {
                    sparkle_state = SPARKLE_TURNING_OFF;
                    sparkle_timer = now;
                    // Next step: start turning off LEDs
                }
                break;

            case SPARKLE_TURNING_OFF:
                // Turn off LEDs one at a time with ~100ms random delay
                if (now - sparkle_timer >= get_random_delay(50, 100)) {
                    int led_index = get_random_led_index(leds_to_turn_off);
                    if (led_index >= 0) {
                        HAL_GPIO_WritePin(led_ports[led_index], led_pins[led_index], GPIO_PIN_RESET);
                        leds_to_turn_off &= ~(1 << led_index);
                        sparkle_timer = now;  // reset timer for next LED off delay
                    } else {
                        // All LEDs off, restart sparkle
                        sparkle_state = SPARKLE_SET_RANDOM;
                    }
                }
                break;
        }
    
    break;
            default:
               
                break;
        }
    }
}
/* USER CODE END 0 */


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
