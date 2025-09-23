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
#include <stdint.h>
#include "stm32f4xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Mandelbrot parameters (matching Prac 1B)
#define MAX_ITER 100  // Base value - will be overridden by MAX_ITER_VALUES array
#define FIXED_POINT_SCALE 1000000LL  // 10^6 scale factor
#define ESCAPE_RADIUS_SQUARED (4LL * FIXED_POINT_SCALE)   // scaled 4.0

// DWT Cycle Counter for STM32F4 (Cortex-M4 has DWT!)
#define DWT_CYCCNT_ENA_Msk   (0x1UL << 0)
#define DWT_CTRL_CYCCNTENA_Msk (0x1UL << 0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//TODO: Define variables you think you might need
// - Performance timing variables (e.g execution time, throughput, pixels per second, clock cycles)

// Performance timing variables (matching Prac 1B structure)
uint32_t start_time = 0;       // Start time in ticks
uint32_t end_time = 0;         // End time in ticks
uint32_t execution_time = 0;   // Execution time in ticks (end_time - start_time)
uint64_t checksum = 0;         // 64-bit checksum returned from mandelbrot function

// Task 2: Extended timing variables with real DWT support
volatile uint32_t start_cycles, end_cycles, execution_cycles;
volatile float pixels_per_second;
volatile uint32_t cpu_clock_hz;

// Image dimensions for testing - same as Prac 1B (128, 160, 192, 224, 256)
const uint32_t IMAGE_DIMENSIONS[] = {128, 160, 192, 224, 256};
const uint32_t NUM_DIMENSIONS = 5;

// Task 2: MAX_ITER values to test (100, 250, 500, 750, 1000)
const uint32_t MAX_ITER_VALUES[] = {100, 250, 500, 750, 1000};
const uint32_t NUM_MAX_ITER_VALUES = 5;

// Task 3: Fixed MAX_ITER = 100 for all tests
#define TASK3_MAX_ITER 100

// Task 5: FPU Impact testing parameters
#define TASK5_MAX_ITER 100
typedef enum {
    MANDELBROT_FIXED_POINT,
    MANDELBROT_FLOAT,
    MANDELBROT_DOUBLE
} mandelbrot_algorithm_t;

// Task 7: Fixed Point Arithmetic Scaling Factors
#define TASK7_MAX_ITER 100
const int64_t SCALING_FACTORS[] = {1000LL, 10000LL, 1000000LL};  // 10^3, 10^4, 10^6
const uint32_t NUM_SCALING_FACTORS = 3;
const char* SCALING_FACTOR_NAMES[] = {"10^3", "10^4", "10^6"};

typedef struct {
    int64_t scale;
    int64_t escape_radius_squared;
    const char* name;
} scaling_config_t;

// Test configuration - change these for different tests
uint32_t current_dimension_index = 4;  // Image size: 0-4 (128,160,192,224,256)
uint32_t current_max_iter_index = 0;   // MAX_ITER value: 0-4 (for Task 2 only)
mandelbrot_algorithm_t current_algorithm = MANDELBROT_FIXED_POINT;  // Task 5: Algorithm selection
uint32_t current_scaling_index = 2;    // Task 7: Scaling factor index: 0-2

// Task 3: Only need to change dimension index (MAX_ITER fixed at 100)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
//TODO: Define any function prototypes you might need such as the calculate Mandelbrot function among others

// Mandelbrot functions (same as Prac 1B)
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_float(int width, int height, int max_iterations);  // Task 5: Float version
uint64_t calculate_mandelbrot_fixed_point_scaling(int width, int height, int max_iterations, int64_t scale);  // Task 7: Variable scaling

// DWT cycle counter functions for STM32F4 (real hardware DWT!)
void DWT_Init(void);
uint32_t DWT_Get_Cycles(void);
void DWT_Start_Cycle_Counter(void);
uint32_t DWT_Stop_Cycle_Counter(void);

// Task 2: Benchmarking functions
void benchmark_task2_single_test(void);
void benchmark_task3_single_test(void);  // Task 3: Extended timing measurement
void benchmark_task5_fpu_test(void);     // Task 5: FPU impact testing
void benchmark_task7_scaling_test(void); // Task 7: Fixed-point scaling factor testing
uint32_t Get_CPU_Clock_Hz(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief Initialize DWT cycle counter (STM32F4 has real DWT!)
  */
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

uint32_t DWT_Get_Cycles(void) {
    return DWT->CYCCNT;
}

void DWT_Start_Cycle_Counter(void) {
    DWT->CYCCNT = 0;
    start_cycles = DWT_Get_Cycles();
}

uint32_t DWT_Stop_Cycle_Counter(void) {
    end_cycles = DWT_Get_Cycles();
    execution_cycles = end_cycles - start_cycles;
    return execution_cycles;
}

uint32_t Get_CPU_Clock_Hz(void) {
    return HAL_RCC_GetHCLKFreq();
}

uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations) {
    uint64_t checksum = 0;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int64_t x0 = ((int64_t)x * (int64_t)(3.5 * FIXED_POINT_SCALE)) / width
                         - (int64_t)(2.5 * FIXED_POINT_SCALE);
            int64_t y0 = ((int64_t)y * (int64_t)(2.0 * FIXED_POINT_SCALE)) / height
                         - FIXED_POINT_SCALE;

            int64_t xi = 0;
            int64_t yi = 0;
            int iteration = 0;

            while (iteration < max_iterations) {
                int64_t xi2 = (xi * xi) / FIXED_POINT_SCALE;
                int64_t yi2 = (yi * yi) / FIXED_POINT_SCALE;

                if (xi2 + yi2 > ESCAPE_RADIUS_SQUARED) {
                    break;
                }

                int64_t old_xi = xi;
                xi = xi2 - yi2 + x0;
                yi = (2 * old_xi * yi) / FIXED_POINT_SCALE + y0;

                iteration++;
            }

            checksum += iteration;
        }
    }

    return checksum;
}

uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations) {
    uint64_t checksum = 0;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            double x0 = ((double)x / width) * 3.5 - 2.5;
            double y0 = ((double)y / height) * 2.0 - 1.0;

            double xi = 0.0;
            double yi = 0.0;
            int iteration = 0;

            while (iteration < max_iterations) {
                double xi2 = xi * xi;
                double yi2 = yi * yi;

                if (xi2 + yi2 > 4.0) {
                    break;
                }

                double temp = xi2 - yi2 + x0;
                yi = 2.0 * xi * yi + y0;
                xi = temp;

                iteration++;
            }

            checksum += iteration;
        }
    }

    return checksum;
}

void benchmark_task2_single_test(void) {
    uint32_t test_dimension = IMAGE_DIMENSIONS[current_dimension_index];
    uint32_t test_max_iter = MAX_ITER_VALUES[current_max_iter_index];
    uint32_t total_pixels = test_dimension * test_dimension;

    cpu_clock_hz = Get_CPU_Clock_Hz();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    start_time = HAL_GetTick();
    DWT_Start_Cycle_Counter();

    checksum = calculate_mandelbrot_fixed_point_arithmetic(test_dimension, test_dimension, test_max_iter);

    execution_cycles = DWT_Stop_Cycle_Counter();
    end_time = HAL_GetTick();

    execution_time = end_time - start_time;

    if (execution_time > 0) {
        pixels_per_second = (float)total_pixels * 1000.0f / execution_time;
    } else {
        pixels_per_second = (float)total_pixels * 1000.0f;
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
}

uint64_t calculate_mandelbrot_float(int width, int height, int max_iterations) {
    uint64_t checksum = 0;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float x0 = ((float)x / width) * 3.5f - 2.5f;
            float y0 = ((float)y / height) * 2.0f - 1.0f;

            float xi = 0.0f;
            float yi = 0.0f;
            int iteration = 0;

            while (iteration < max_iterations) {
                float xi2 = xi * xi;
                float yi2 = yi * yi;

                if (xi2 + yi2 > 4.0f) {
                    break;
                }

                float temp = xi2 - yi2 + x0;
                yi = 2.0f * xi * yi + y0;
                xi = temp;

                iteration++;
            }

            checksum += iteration;
        }
    }

    return checksum;
}

void benchmark_task5_fpu_test(void) {
    uint32_t test_dimension = IMAGE_DIMENSIONS[current_dimension_index];
    uint32_t total_pixels = test_dimension * test_dimension;

    cpu_clock_hz = Get_CPU_Clock_Hz();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    start_time = HAL_GetTick();
    DWT_Start_Cycle_Counter();

    // Call appropriate Mandelbrot function based on algorithm selection
    switch(current_algorithm) {
        case MANDELBROT_FIXED_POINT:
            checksum = calculate_mandelbrot_fixed_point_arithmetic(test_dimension, test_dimension, TASK5_MAX_ITER);
            break;
        case MANDELBROT_FLOAT:
            checksum = calculate_mandelbrot_float(test_dimension, test_dimension, TASK5_MAX_ITER);
            break;
        case MANDELBROT_DOUBLE:
            checksum = calculate_mandelbrot_double(test_dimension, test_dimension, TASK5_MAX_ITER);
            break;
    }

    execution_cycles = DWT_Stop_Cycle_Counter();
    end_time = HAL_GetTick();

    execution_time = end_time - start_time;

    if (execution_time > 0) {
        pixels_per_second = (float)total_pixels * 1000.0f / execution_time;
    } else {
        pixels_per_second = (float)total_pixels * 1000.0f;
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
}

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
                // Check for potential overflow before calculation
                if (xi > scale * 1000 || yi > scale * 1000 || xi < -scale * 1000 || yi < -scale * 1000) {
                    // Potential overflow detected
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

void benchmark_task7_scaling_test(void) {
    uint32_t test_dimension = IMAGE_DIMENSIONS[current_dimension_index];
    int64_t current_scale = SCALING_FACTORS[current_scaling_index];
    uint32_t total_pixels = test_dimension * test_dimension;

    cpu_clock_hz = Get_CPU_Clock_Hz();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    start_time = HAL_GetTick();
    DWT_Start_Cycle_Counter();

    // Task 7: Test with current scaling factor
    checksum = calculate_mandelbrot_fixed_point_scaling(test_dimension, test_dimension, TASK7_MAX_ITER, current_scale);

    execution_cycles = DWT_Stop_Cycle_Counter();
    end_time = HAL_GetTick();

    execution_time = end_time - start_time;

    if (execution_time > 0) {
        pixels_per_second = (float)total_pixels * 1000.0f / execution_time;
    } else {
        pixels_per_second = (float)total_pixels * 1000.0f;
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
}

void benchmark_task3_single_test(void) {
    uint32_t test_dimension = IMAGE_DIMENSIONS[current_dimension_index];
    uint32_t total_pixels = test_dimension * test_dimension;

    cpu_clock_hz = Get_CPU_Clock_Hz();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    start_time = HAL_GetTick();
    DWT_Start_Cycle_Counter();

    // Task 3: Fixed MAX_ITER = 100
    checksum = calculate_mandelbrot_fixed_point_arithmetic(test_dimension, test_dimension, TASK3_MAX_ITER);

    execution_cycles = DWT_Stop_Cycle_Counter();
    end_time = HAL_GetTick();

    execution_time = end_time - start_time;

    if (execution_time > 0) {
        pixels_per_second = (float)total_pixels * 1000.0f / execution_time;
    } else {
        pixels_per_second = (float)total_pixels * 1000.0f;
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(1000);
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

  // Initialize DWT cycle counter (STM32F4 has real hardware DWT!)
  DWT_Init();

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

	  // TASK 7: Fixed Point Arithmetic Scaling Factor (STM32F4)
	  // Constraint: MAX_ITER = 100, test different scaling factors
	  // Change current_dimension_index (0-4) and current_scaling_index (0-2):
	  // 0: 10^3 scale,  1: 10^4 scale,  2: 10^6 scale

	  benchmark_task7_scaling_test();

	  // Stop after one test - check debugger variables:
	  // execution_time (ms) - Wall-clock time
	  // execution_cycles - Real CPU cycles
	  // pixels_per_second - Throughput
	  // checksum - Precision comparison between scales
	  // Current test: IMAGE_DIMENSIONS[current_dimension_index] with SCALING_FACTORS[current_scaling_index]
	  while(1) {
	      // Task 7 test complete - record precision, overflow risk, execution speed
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
