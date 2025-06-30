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
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "uart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "camera.h"
#include "lcd.h"

#include <stdio.h>

#include "tensorflow/lite/core/c/common.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/recording_micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_profiler.h"

// TODO : 1. Model File Include
// 모델 파일을 프로젝트의 models 폴더에 준비해주세요.
// 이후 모델 Header 파일 include 해주세요.
#include "models/fm_qat_model.h"



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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

#ifdef TFT96
// QQVGA
#define FrameWidth 160
#define FrameHeight 120
#define CAPTURE_ZONE_SIZE 48
#elif TFT18
// QQVGA2
#define FrameWidth 128
#define FrameHeight 160
#endif
// picture buffer
uint16_t pic[FrameHeight][FrameWidth];
volatile uint32_t DCMI_FrameIsReady;
uint32_t Camera_FPS=0;
volatile uint8_t captured_flag = 0;

// External key debouncing variables
volatile uint8_t external_key_pressed = 0;
volatile uint8_t external_key_debounce = 0;

// TODO : (필요시) tensor_arena_size는 필요한 만큼 조정해주세요.
constexpr int tensor_arena_size = 50 * 1024;
static uint8_t tensor_arena[tensor_arena_size];

int Get_Top_Prediction(const float* predictions, int num_categories);
void Get_Data(void* target);
void Get_Capture(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void MPU_Config(void)
{
	MPU_Region_InitTypeDef MPU_InitStruct = {0};

	/* Disables the MPU */
	HAL_MPU_Disable();

	/* Configure the MPU attributes for the QSPI 256MB without instruction access */
	MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
	MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress      = QSPI_BASE;
	MPU_InitStruct.Size             = MPU_REGION_SIZE_256MB;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
	MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
	MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
	MPU_InitStruct.SubRegionDisable = 0x00;
	HAL_MPU_ConfigRegion(&MPU_InitStruct);

	/* Configure the MPU attributes for the QSPI 8MB (QSPI Flash Size) to Cacheable WT */
	MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
	MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
	MPU_InitStruct.BaseAddress      = QSPI_BASE;
	MPU_InitStruct.Size             = MPU_REGION_SIZE_8MB;
	MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RO;
	MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
	MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
	MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
	MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
	MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
	MPU_InitStruct.SubRegionDisable = 0x00;
	HAL_MPU_ConfigRegion(&MPU_InitStruct);

	/* Setup AXI SRAM in Cacheable WB */
	MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
	MPU_InitStruct.BaseAddress      = D1_AXISRAM_BASE;
	MPU_InitStruct.Size             = MPU_REGION_SIZE_512KB;
	MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
	MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
	MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.Number           = MPU_REGION_NUMBER2;
	MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
	MPU_InitStruct.SubRegionDisable = 0x00;
	MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
	HAL_MPU_ConfigRegion(&MPU_InitStruct);

	/* Enables the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

static void CPU_CACHE_Enable(void)
{
	/* Enable I-Cache */
	SCB_EnableICache();

	/* Enable D-Cache */
	SCB_EnableDCache();
}

void LED_Blink(uint32_t Hdelay, uint32_t Ldelay)
{
	HAL_GPIO_WritePin(PE3_GPIO_Port, PE3_Pin, GPIO_PIN_SET);
	HAL_Delay(Hdelay - 1);
	HAL_GPIO_WritePin(PE3_GPIO_Port, PE3_Pin, GPIO_PIN_RESET);
	HAL_Delay(Ldelay - 1);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

#ifdef W25Qxx
	SCB->VTOR = QSPI_BASE;
#endif
	MPU_Config();
	CPU_CACHE_Enable();

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
	MX_DCMI_Init();
	MX_I2C1_Init();
	MX_SPI4_Init();
	MX_TIM1_Init();
	UART_Init();
	/* USER CODE BEGIN 2 */
	uint8_t text[80];

	LCD_Test();

	sprintf((char *)&text, "Camera Not Found");
	LCD_ShowString(0, 58, ST7735Ctx.Width, 16, 16, text);

	//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	//	HAL_Delay(10);
#ifdef TFT96
	Camera_Init_Device(&hi2c1, FRAMESIZE_QQVGA);
#elif TFT18
	Camera_Init_Device(&hi2c1, FRAMESIZE_QQVGA2);
#endif

	//clean Ypos 58
	ST7735_LCD_Driver.FillRect(&st7735_pObj, 0, 58, ST7735Ctx.Width, 16, BLACK);

	UART_Send_String((char*)"Fashion-MNIST Classification (CNN, Quantization-Aware-Training) \n\r");

	// TODO : 2. Get Model
	// 모델을 GetModel 이용하여 불러오고, model의 schema version을 체크해주세요.
	// library의 version과 다를 시 무한루프
	const tflite::Model* model = ::tflite::GetModel(fm_qat_model);
	if (model->version() != TFLITE_SCHEMA_VERSION) {
		for(;;);
	}

	UART_Send_String((char*)"TFLITE_SCHEMA_VERSION OK!\n\r");

	tflite::MicroMutableOpResolver<10> micro_op_resolver;

	// TODO : 3. micro_op_resolver에 Ops 추가하기
	// 모델 연산에 필요한 Ops를 micro_op_resolver에 추가해주세요.
	micro_op_resolver.AddConv2D();
	micro_op_resolver.AddDequantize();
	micro_op_resolver.AddFullyConnected();
	micro_op_resolver.AddStridedSlice();
	micro_op_resolver.AddShape();
	micro_op_resolver.AddMaxPool2D();
	micro_op_resolver.AddQuantize();
	micro_op_resolver.AddPack();
	micro_op_resolver.AddReshape();
	micro_op_resolver.AddSoftmax();

	// TODO : 4. interpreter 생성 및 allocate tensor 수행
	// 준비된 model, micro_op_resolver, tensor_arena를 이용하여 interpreter 생성하고
	// interpreter의 AllocateTensors()를 실행해주세요.
	// AllocateTensors()실행 실패 시 무한루프로 프로그램을 멈춰주세요.
	tflite::MicroInterpreter interpreter(model, micro_op_resolver, tensor_arena, tensor_arena_size);
	if(interpreter.AllocateTensors() != kTfLiteOk){
		sprintf((char *)&text, "TFLM Allocate Tensor Failed!!");
		UART_Send_String((char*)text);
		while(1);
	}

	sprintf((char *)&text, "Tensor Arena used : %d\n\r", interpreter.arena_used_bytes());
	UART_Send_String((char*)text);

	TfLiteTensor* input = interpreter.input(0);
	sprintf((char *)&text, "Input_Shape : (%d,%d,%d)\n\r",input->dims->data[0], input->dims->data[1], input->dims->data[2]) ;
	UART_Send_String((char*)text);

	while (HAL_GPIO_ReadPin(EXTERNAL_KEY_GPIO_Port, EXTERNAL_KEY_Pin) == GPIO_PIN_SET)
	{
		sprintf((char *)&text, "Camera id:0x%x   ", hcamera.device_id);
		LCD_ShowString(0, 58, ST7735Ctx.Width, 16, 12, text);

		LED_Blink(5, 500);

		sprintf((char *)&text, "LongPress External Key to Run");
		LCD_ShowString(0, 58, ST7735Ctx.Width, 16, 12, text);

		LED_Blink(5, 500);
	}

	HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)&pic, FrameWidth * FrameHeight * 2 / 4);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (DCMI_FrameIsReady)
		{
			HAL_DCMI_Stop(&hdcmi);
			DCMI_FrameIsReady = 0;

			int time_start_lcd = HAL_GetTick();
			ST7735_FillRGBRect(&st7735_pObj,0,0,(uint8_t *)&pic[20][0], ST7735Ctx.Width, 80);
			int time_end_lcd = HAL_GetTick();

			// External key debouncing and detection
			GPIO_PinState current_key_state = HAL_GPIO_ReadPin(EXTERNAL_KEY_GPIO_Port, EXTERNAL_KEY_Pin);
			
			// Key is pressed (LOW) and not already processed
			if (current_key_state == GPIO_PIN_RESET && !external_key_pressed && !captured_flag) {
				external_key_debounce++;
				
				// Debounce delay (wait for stable state)
				if (external_key_debounce > 5) { // 약 50ms (10ms * 5)
					external_key_pressed = 1;
					external_key_debounce = 0;
					captured_flag = 1;
					
					// 디버그: 키 상태 확인
					UART_Send_String((char*)"External Key Pressed - Starting Image Transfer\n");
					
					// 이미지 전송 시작 마커 전송
					UART_Send_String((char*)"IMG_START\n");

					int r_base = FrameHeight/2 - CAPTURE_ZONE_SIZE/2;
					int c_base = FrameWidth/2 - CAPTURE_ZONE_SIZE/2;
					// 이미지 데이터 전송 (전체 프레임)
					for (int y = r_base; y < r_base + CAPTURE_ZONE_SIZE; y++) {
						for (int x = c_base; x < c_base + CAPTURE_ZONE_SIZE; x++) {
							// 픽셀 데이터 추출 (리틀 엔디안 처리)
							uint8_t pixel_low = pic[y][x] & 0xFF;
							uint8_t pixel_high = (pic[y][x] >> 8) & 0xFF;
							// UART로 픽셀 데이터 전송
							UART_Send_Char(pixel_low);
							UART_Send_Char(pixel_high);
				    	}
					}

					// 이미지 전송 종료 마커 전송
					UART_Send_String((char*)"IMG_END\n");

					// 디버그 메시지
					UART_Send_String((char*)"Image sent via UART\n");

					captured_flag = 0;
				}
			}
			// Key is released (HIGH) - reset debounce
			else if (current_key_state == GPIO_PIN_SET) {
				external_key_pressed = 0;
				external_key_debounce = 0;
			}
			
			// 디버그: 키가 눌리지 않았을 때 상태 출력 (주기적으로)
			static uint32_t last_debug_time = 0;
			if (HAL_GetTick() - last_debug_time > 2000) { // 2초마다
				last_debug_time = HAL_GetTick();
				sprintf((char *)&text, "Key State: %s, Debounce: %d\n\r", 
					(current_key_state == GPIO_PIN_SET) ? "HIGH(Not Pressed)" : "LOW(Pressed)", 
					external_key_debounce);
				UART_Send_String((char*)text);
			}
			// Draw Input Data Guide Line
			ST7735_DrawVLine(&st7735_pObj, 160/2-CAPTURE_ZONE_SIZE/2, 80/2-CAPTURE_ZONE_SIZE/2, CAPTURE_ZONE_SIZE, 0xf800);
			ST7735_DrawVLine(&st7735_pObj, 160/2+CAPTURE_ZONE_SIZE/2, 80/2-CAPTURE_ZONE_SIZE/2, CAPTURE_ZONE_SIZE, 0xf800);
			ST7735_DrawHLine(&st7735_pObj, 160/2-CAPTURE_ZONE_SIZE/2, 80/2-CAPTURE_ZONE_SIZE/2, CAPTURE_ZONE_SIZE, 0xf800);
			ST7735_DrawHLine(&st7735_pObj, 160/2-CAPTURE_ZONE_SIZE/2, 80/2+CAPTURE_ZONE_SIZE/2, CAPTURE_ZONE_SIZE, 0xf800);

			int time_start_getdata = HAL_GetTick();
			Get_Data(input->data.f);
			int time_end_getdata = HAL_GetTick();

			int time_start_invoke = HAL_GetTick();
			TfLiteStatus invoke_status = interpreter.Invoke();
			if (invoke_status != kTfLiteOk) {
				UART_Send_String((char*)"Invoke failed.\n\r");
				break;
			}
			int time_end_invoke = HAL_GetTick();

			TfLiteTensor* output = interpreter.output(0);
			int top_ind = Get_Top_Prediction(output->data.f, 10);

			sprintf((char *)&text, "%d %.1f%%\n\r", top_ind, output->data.f[top_ind] * 100.0);
			UART_Send_String((char*)text);

			sprintf((char *)&text, "%d\n\r", top_ind);
			LCD_ShowString(0, 0, ST7735Ctx.Width, 16, 12, text);

			sprintf((char *)&text, "Cam to LCD : %dms\n\r",time_end_lcd - time_start_lcd);
			UART_Send_String((char*)text);

			sprintf((char *)&text, "Get_Data : %dms\n\r",time_end_getdata - time_start_getdata);
			UART_Send_String((char*)text);

			sprintf((char *)&text, "invoke : %dms\n\r",time_end_invoke - time_start_invoke);
			UART_Send_String((char*)text);

			HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)&pic, FrameWidth * FrameHeight * 2 / 4);
		}
		// HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
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

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 5;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
			|RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_4);
}

/* USER CODE BEGIN 4 */
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	static uint32_t count = 0,tick = 0;

	if(HAL_GetTick() - tick >= 1000)
	{
		tick = HAL_GetTick();
		Camera_FPS = count;
		count = 0;
	}
	count ++;

	DCMI_FrameIsReady = 1;
}

void Get_Data(void* target){
	// TODO : 5. Get_Data 구현
	// Camera Image(pic 배열의 내용)의 중심점을 중심으로 둔 width 56, height 56의 이미지를 28*28 로 줄여서
	// target이 가르키는 메모리에 넣어주는 program을 작성해주세요.
	// pixel data는 model의 요구사항에 맞게 변환하여 넣어주세요.
	int r_base = FrameHeight/2 - 28;
	int c_base = FrameWidth/2 - 28;
	float (*a_target)[28] = (float (*)[28])target;
	for(int r_t = 0;r_t<28;r_t++){
		for(int c_t = 0;c_t<28;c_t++){
			uint16_t temp = pic[r_base+(2*r_t)][c_base+(2*c_t)];
			uint8_t pixel_low = temp & 0xFF;
			uint8_t pixel_high = (temp >> 8) & 0xFF;
			uint16_t pixel = uint16_t((pixel_low << 8) | pixel_high);
			uint8_t r = ((pixel >> 11) & 0x1F)<<3;
			uint8_t g = ((pixel >> 5) & 0x3F)<<2;
			uint8_t b = (pixel & 0x1F)<<3;
			float gray = (3*r+6*g+b)/10./255.;

			a_target[r_t][c_t] = gray;
		}
	}
}

int Get_Top_Prediction(const float* predictions, int num_categories) {
	// TODO : 6. Get_Top_Prediction() 구현
	// predictions의 내용을 바탕으로 num_categories 개의 class 중 가장 확률이 높은 class의
	// index를 return하는 함수로 구현해주세요.
	float max_score = predictions[0];
	int guess = 0;

	for (int category_index = 1; category_index < num_categories; category_index++) {
		const float category_score = predictions[category_index];
		if (category_score > max_score) {
			max_score = category_score;
			guess = category_index;
		}
	}
	return guess;
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
	while (1)
	{
		LED_Blink(5, 250);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
