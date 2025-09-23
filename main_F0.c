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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Task 7: Fixed Point Arithmetic Scaling Factors
#define TASK7_MAX_ITER 100  // Fixed MAX_ITER for Task 7

// Task 7: Different scaling factors to test (10^3, 10^4, 10^6)
const int64_t SCALING_FACTORS[] = {1000LL, 10000LL, 1000000LL};
const uint32_t NUM_SCALING_FACTORS = 3;
const char* SCALING_FACTOR_NAMES[] = {"10^3", "10^4", "10^6"};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//TODO: Define variables you think you might need
// - Performance timing variables (e.g execution time, throughput, pixels per second, clock cycles)

// Task 7: Performance timing variables
uint32_t start_time = 0;       // Start time in ticks
uint32_t end_time = 0;         // End time in ticks
uint32_t execution_time = 0;   // Execution time in ticks (end_time - start_time)
uint64_t checksum = 0;         // 64-bit checksum returned from mandelbrot function

// Task 7: Extended timing variables (STM32F0 limitations)
volatile uint32_t start_cycles, end_cycles, execution_cycles;
volatile float pixels_per_second;
volatile uint32_t estimated_cpu_cycles;  // Estimated from timing and clock freq
volatile uint32_t cpu_clock_hz;          // CPU clock frequency

// Image dimensions for testing - same as Prac 1B (128, 160, 192, 224, 256)
const uint32_t IMAGE_DIMENSIONS[] = {128, 160, 192, 224, 256};
const uint32_t NUM_DIMENSIONS = 5;

// Task 7: Test configuration - change these for different tests
uint32_t current_dimension_index = 1 ;  // Image size: 0-4
uint32_t current_scaling_index = 1;    // Scaling factor: 0-2

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
//TODO: Define any function prototypes you might need such as the calculate Mandelbrot function among others

// Task 7: Mandelbrot function with variable scaling factor
uint64_t calculate_mandelbrot_fixed_point_scaling(int width, int height, int max_iterations, int64_t scale);

// Task 7: Timing functions (STM32F0 alternative to DWT)
void SysTick_Init_Cycles(void);
uint32_t SysTick_Get_Cycles(void);
void SysTick_Start_Cycle_Counter(void);
uint32_t SysTick_Stop_Cycle_Counter(void);
uint32_t Get_CPU_Clock_Hz(void);

// Task 7: Benchmarking function
void benchmark_task7_scaling_test(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief Initialize SysTick for cycle counting (STM32F0 alternative to DWT)
  * @retval None
  */
void SysTick_Init_Cycles(void) {
    // SysTick is already initialized by HAL_Init()
    // We'll use HAL_GetTick() for timing since Cortex-M0 doesn't have DWT
}

/**
  * @brief Get current tick count (STM32F0 alternative to DWT cycles)
  * @retval Current tick count
  */
uint32_t SysTick_Get_Cycles(void) {
    return HAL_GetTick();
}

/**
  * @brief Start cycle counter measurement (STM32F0 alternative)
  * @retval None
  */
void SysTick_Start_Cycle_Counter(void) {
    start_cycles = HAL_GetTick();
}

/**
  * @brief Stop cycle counter and return elapsed ticks (STM32F0 alternative)
  * @retval Elapsed ticks (1ms resolution)
  */
uint32_t SysTick_Stop_Cycle_Counter(void) {
    end_cycles = HAL_GetTick();
    execution_cycles = end_cycles - start_cycles;
    return execution_cycles;
}

/**
  * @brief Get CPU clock frequency in Hz
  * @retval CPU clock frequency
  */
uint32_t Get_CPU_Clock_Hz(void) {
    return HAL_RCC_GetHCLKFreq();
}

/**
  * @brief Calculate Mandelbrot set using fixed-point arithmetic with variable scaling
  * @param width: Image width
  * @param height: Image height
  * @param max_iterations: Maximum iterations
  * @param scale: Fixed-point scaling factor (10^3, 10^4, 10^6)
  * @retval Checksum of all iterations
  */
uint64_t calculate_mandelbrot_fixed_point_scaling(int width, int height, int max_iterations, int64_t scale) {
    uint64_t checksum = 0;
    int64_t escape_radius_squared = 4LL * scale;  // 4.0 in current scale

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Scale coordinates into fixed-point with current scale
            int64_t x0 = ((int64_t)x * (int64_t)(3.5 * scale)) / width - (int64_t)(2.5 * scale);
            int64_t y0 = ((int64_t)y * (int64_t)(2.0 * scale)) / height - scale;

            int64_t xi = 0;
            int64_t yi = 0;
            int iteration = 0;

            while (iteration < max_iterations) {
                // Check for potential overflow before calculation (overflow detection)
                if (xi > scale * 1000 || yi > scale * 1000 || xi < -scale * 1000 || yi < -scale * 1000) {
                    // Potential overflow detected - exit early
                    break;
                }

                int64_t xi2 = (xi * xi) / scale;
                int64_t yi2 = (yi * yi) / scale;

                if (xi2 + yi2 > escape_radius_squared) {
                    break;
                }

                int64_t old_xi = xi;
                xi = xi2 - yi2 + x0;
                yi = (2 * old_xi * yi) / scale + y0;

                iteration++;
            }

            checksum += iteration;
        }
    }

    return checksum;
}

/**
  * @brief Task 7: Benchmark with different fixed-point scaling factors
  * @retval None
  */
void benchmark_task7_scaling_test(void) {
    uint32_t test_dimension = IMAGE_DIMENSIONS[current_dimension_index];
    int64_t current_scale = SCALING_FACTORS[current_scaling_index];
    uint32_t total_pixels = test_dimension * test_dimension;

    // Get CPU clock frequency
    cpu_clock_hz = Get_CPU_Clock_Hz();

    // Turn on LED0 to signify the start of the operation
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    // Record the start time with maximum precision available
    start_time = HAL_GetTick();
    SysTick_Start_Cycle_Counter();

    // Call the Mandelbrot Function (Task 7: Test with current scaling factor)
    checksum = calculate_mandelbrot_fixed_point_scaling(test_dimension, test_dimension, TASK7_MAX_ITER, current_scale);

    // Record the end time
    execution_cycles = SysTick_Stop_Cycle_Counter();
    end_time = HAL_GetTick();

    // Calculate detailed timing metrics for Task 7
    execution_time = end_time - start_time;  // Wall-clock time in ms

    // Estimate CPU cycles (since STM32F0 doesn't have DWT)
    if (execution_time > 0) {
        estimated_cpu_cycles = (execution_time * cpu_clock_hz) / 1000;
        pixels_per_second = (float)total_pixels * 1000.0f / execution_time;
    } else {
        // Handle very fast execution (< 1ms)
        estimated_cpu_cycles = cpu_clock_hz / 1000;  // Assume ~1ms minimum
        pixels_per_second = (float)total_pixels * 1000.0f;  // Theoretical max
    }

    // Turn on LED1 to signify the end of the operation
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

    // Hold the LEDs on for a 1s delay
    HAL_Delay(1000);

    // Turn off the LEDs
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
}

/* USER CODE END 0 */

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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  // Initialize SysTick for cycle counting (STM32F0 doesn't have DWT)
  SysTick_Init_Cycles();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //TODO: Visual indicator: Turn on LED0 to signal processing start
	  //TODO: Benchmark and Profile Performance
	  //TODO: Visual indicator: Turn on LED1 to signal processing start
	  //TODO: Keep the LEDs ON for 2s
	  //TODO: Turn OFF LEDs

	  // TASK 7: Fixed Point Arithmetic Scaling Factor (STM32F0)
	  // Constraint: MAX_ITER = 100, test different scaling factors and image sizes
	  // Change current_dimension_index (0-4) and current_scaling_index (0-2):
	  // Scaling: 0=10^3, 1=10^4, 2=10^6

	  benchmark_task7_scaling_test();

	  // Stop after one test - check debugger variables for Task 7:
	  // execution_time (ms) - Wall-clock time
	  // estimated_cpu_cycles - Estimated CPU cycles
	  // pixels_per_second - Throughput
	  // checksum - Precision comparison between scaling factors
	  // Current test: IMAGE_DIMENSIONS[current_dimension_index] with SCALING_FACTORS[current_scaling_index]
	  while(1) {
	      // Task 7 test complete - record precision, overflow risk, execution speed data
	  }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//TODO: Function signatures you defined previously , implement them here

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
#ifdef USE_FULL_ASSERT
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
