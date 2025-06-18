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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"					//計算速度
#include "usbd_cdc_if.h"			//VCP傳值操作
#include "PID.h"							//導入PID算法控制速度
#include "mpu6050.h"					//計算角速度與加速度
#include "cacul+filter.h"			//濾波
#include "stdio.h"
#include "math.h"
#include "motor.h"						//馬達驅動代碼
#include "balance.h"					//平衡控制代碼
#include "stm32f1xx_hal_gpio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct PID;	
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CPR 16  // 原始編碼器解析度
#define REDUCTION_RATIO 19.0f
#define MIN_RPM_THRESHOLD 50.0f

#define PWM_MAX 2999.0f
#define PWM_MIN -2999.0f
#define Kff    8.0f    // FF增益，視系統慣性再微調
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t last_tick = 0;     
float I_term= 0.0f;

/*VCP指令控制pwm輸出參數*/
uint32_t per,pul = 0;
uint32_t input_duty,input_freq;
uint32_t stopflag = 1;
//float pwm_per ,pwm_pul  = 0;
float pwm_pul = 300 ;	//300
/* M法計算馬達速度 */	
uint32_t sys_tick;
int Encoder_Right;
uint16_t last_capture1 = 0;
uint16_t last_capture2 = 0;
uint16_t capture1 = 0;
uint16_t capture2 = 0;
uint16_t delta_t1 = 0;
uint16_t delta_t2 = 0;
float time_s;
float rpm_left;
float rpm_right;
int pulses_per_rev = 1400;  
uint32_t delta_t_avg;

/*Encoder模式測速*/
short Encoder1Count = 0;//编码器计数器值
/*PWM輸入捕獲參數*/
uint32_t  IC1Value,IC2Value;		
double DutyCycle,Frequency;

/*PID控制參數*/
float pwm_input;
float balance_pwm_input;
/*姿態角解算參數*/
float ord_zacc;					//初始z軸加速度
float filter_zacc;			//濾波後z軸加速度
float filter_xacc;			//濾波後x軸加速度
float PID_zacc;					//PID算法z軸加速度
float ord_ygyro;
float ord_xacc;
float Angle;						//當前姿態角
/*MPU6050狀態*/
float mpu_ready;			//ready旗標
float y_bias = 0;				//y軸角速度的偏移
/*一階指數滑動平均（EMA），但記住這會引入延遲*/
float smooth_ygyro = 0.0f;
float alpha = 0.1f; //根據穩定性調整為0.05~0.2

/*馬達運動控制*/
float motor_state ;		//馬達當前狀態--> 停止:0；正轉:1；反轉:2
float last_state;
//float test_INT;		//測試中斷觸發後While內if是否有執行

/*轉向環參數*/
float offset;
char rx_data;
char rx_buffer[10];
uint8_t idx = 0;
int pid_offset;
int huart_flag;
int last_offset = 0;
int offset_var;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float get_motor_rpm(void);
float balance_PID_main(Balance_PID *pid, float Angle,float ygyro);
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
  MX_TIM2_Init(pwm_pul);
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);		// 在啟動定時器1中斷
	HAL_TIM_Base_Start_IT(&htim3); 		//開啟定時器2 中斷
	//啟用接收中斷
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_data, 1);
	uint8_t RxData;
	HAL_UART_Receive_IT(&huart1, &RxData, 1); // 啟用中斷接收
	// 設定 NVIC
	HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
	/*mpu6050初始化*/
	mpu_begin();
	MPU_Init();
	//MPU6050_EXTI_Init();
	MPU_Set_LPF(20);
	//y_bias = calibrate_ygyro(750);		// 校正偏移（靜止不動時進行）
	HAL_Delay(100);
	
	/* 定時器2通道3,4輸出PWM */
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	//HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	/* 霍爾感測器使能捕獲/比較2中斷請求 */
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);

	/*PID控制時間間隔中斷*/

	/*平衡控制初始化*/
	PID_balance_init();
	PID_speed_init();
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
		/*測試區域*/
		
		//右邊馬達向前
		
		//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_ALL);
		//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
		//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
		//HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
		/*轉向環控制區域*/
		//小車行駛速度pwm_pul = 300(約為全速運轉的10%)
		//usb_printf("Current offset = %.2f\r\n", offset);
		
		/*
		//1.保存計數器值
		Encoder1Count =(short)__HAL_TIM_GET_COUNTER(&htim3);
		//2.清零計數器值
		__HAL_TIM_SET_COUNTER(&htim3,0);
		if(Encoder1Count<0)
		{
		Encoder1Count = -(Encoder1Count);
		rpm = (float)Encoder1Count*60.0f/412.0f*20.0f;
		}
		else
		{
		rpm = (float)Encoder1Count*60.0f/412.0f*20.0f;
		}
		//usb_printf("Encoder1Count:%d\r\n",Encoder1Count);
		
		rpm = get_motor_rpm();
		usb_printf("Current Motor Speed = %.2f\r\n",rpm);
		HAL_Delay(1000);
		*/
		if(huart_flag)
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_SET);
			//左輪向前
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, GPIO_PIN_SET);			
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_RESET);		
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,300);			//3比4低-->左轉
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,300);
			if(pid_offset > 0 )
			{
				offset_var = pid_offset - last_offset;
				if(offset_var > 5 && offset_var < 10 )		//向左偏了，改右轉==>3比4高
				{
					pwm_pul = position_PID(&PositionPID,pid_offset);
					pwm_pul = pwm_pul>700?700:(pwm_pul<(0)?(0):pwm_pul);
					uint32_t compare_value = (uint32_t) (pwm_pul);
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,pwm_pul);
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,400);
				}
				if(offset_var < 0 )		//向右邊了，改左轉==>3比4高
				{
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,300);			//直行
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,300);
				}
			}
			if(pid_offset < 0)
			{
				offset_var = pid_offset - last_offset;
				if(offset_var < -5 && offset_var > -10 )		//向右邊了，改左轉==>3比4低
				{
					pwm_pul = -(position_PID(&PositionPID,pid_offset));
					pwm_pul = pwm_pul>700?700:(pwm_pul<(0)?(0):pwm_pul);
					uint32_t compare_value = (uint32_t) (pwm_pul);
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,400);
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,pwm_pul);
				}
				if(offset_var > 0 )		//逐漸向左偏了，繼續前進
				{
					continue;
				}
			}
			if(pid_offset == 0)
			{
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,400);
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,400);
			}
		}
		
		if(mpu_ready>500)
		{
			motor_state = 1;
			mpu_ready = 0;
			ord_ygyro = (mpu_gy_y()-y_bias)/131.0;										//獲取原始y軸角速度
			smooth_ygyro = alpha * ord_ygyro+(1-alpha)*smooth_ygyro;	//平滑y軸角速度誤差
			ord_zacc = mpu_acc_z()/16384.0;														//獲取原始z軸加速度
			ord_xacc = mpu_acc_x()/16384.0;														//獲取原始x軸加速度
			filter_zacc = update_filtered_z_acc(ord_zacc);						//z軸加速度一階低通濾波
			filter_xacc = update_filtered_x_acc(ord_xacc);						//x軸加速度一階低通濾波
			Angle = update_filtered_acc(filter_xacc,filter_zacc,smooth_ygyro);	//姿態角解算輸出
			if(Angle <-2.0 )
			{
				if(last_state)
				{
					pwm_pul = balance_PID_main(&BalancePID,Angle,smooth_ygyro) *1.0;
					pwm_pul = pwm_pul>2200?2200:(pwm_pul<(0)?(0):pwm_pul);
					//usb_printf("Current PWM = %.2f\r\n",pwm_pul);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_SET);
					//HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
					//HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
					//HAL_Delay(50);
				//	pwm_pul = 600;
					uint32_t compare_value = (uint32_t) (pwm_pul);
					//compare_value = compare_value *1.05				//因為重心偏前面
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,compare_value);
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,compare_value);
					//usb_printf("Current GYCC = %.2f\r\n",smooth_ygyro);	
					//usb_printf("Current PWM = %d\r\n",compare_value);
					last_state = 1;
				}
				else
				{
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);
					pwm_pul = balance_PID_main(&BalancePID,Angle,smooth_ygyro)*1.0;
					pwm_pul = pwm_pul>2200?2200:(pwm_pul<(0)?(0):pwm_pul);
					//usb_printf("Current PWM = %.2f\r\n",pwm_pul);
					//pwm_pul = 600;
					HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
					HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_4);
					HAL_Delay(10);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_SET);
					uint32_t compare_value = (uint32_t) (pwm_pul);
					//compare_value = compare_value *1.05;				//因為重心偏前面					
					HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
					HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,compare_value);
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,compare_value);
					//usb_printf("Current GYCC = %.2f\r\n",smooth_ygyro);	
					//usb_printf("Current Angle = %.2f\r\n",Angle);	
					//usb_printf("Current PWM = %d\r\n",compare_value);
					last_state = 1;
				}
			
			}
			if(Angle > 2.0f)
			{
				if(last_state)
				{
					pwm_pul = -(balance_PID_main(&BalancePID,Angle,smooth_ygyro));
					pwm_pul = pwm_pul>2200?2200:(pwm_pul<(0)?(0):pwm_pul);
					//pwm_pul = 600;
					HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
					HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_4);
					HAL_Delay(10);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_RESET);
					HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
					HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
					uint32_t compare_value = (uint32_t) (pwm_pul);
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,compare_value);
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,compare_value);
					//usb_printf("Current GYCC = %.2f\r\n",smooth_ygyro);	
					//usb_printf("Current PWM = %d\r\n",compare_value);
					//HAL_Delay(2);
					last_state = 0;
				}
				else
				{
					pwm_pul = -(balance_PID_main(&BalancePID,Angle,smooth_ygyro));
					pwm_pul = pwm_pul>2200?2200:(pwm_pul<(0)?(0):pwm_pul);
					//usb_printf("Current PWM = %.2f\r\n",pwm_pul);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_RESET);
					//HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
					//HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
					//HAL_Delay(50);
					//pwm_pul = 600;
					uint32_t compare_value = (uint32_t) (pwm_pul); 
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,compare_value);
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,compare_value);
					//usb_printf("Current GYCC = %.2f\r\n",smooth_ygyro);	
					//usb_printf("Current Angle = %.2f\r\n",Angle);	
					//usb_printf("Current PWM = %d\r\n",compare_value);
					last_state = 0;
					//HAL_Delay(2);
				}
			 
			}
			if (Angle > 2.0f && Angle < -2.0f)
			{
				/*HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_SET);
				pwm_pul = 120;
				//compare_value = compare_value *1.1;				//因為重心偏前面
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,pwm_pul);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,pwm_pul);
				//usb_printf("Current Angle = %.2f\r\n",Angle);	
				//usb_printf("Current PWM = %d\r\n",compare_value);
				last_state = 1;*/
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_RESET);
				pwm_pul = 0;
				uint32_t compare_value = (uint32_t) (pwm_pul); 
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,compare_value);
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,compare_value);
				HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
				HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_4);
				HAL_Delay(10);
				//HAL_Delay(10);
				//usb_printf("Balance !\r\n");
			}
			if(Angle > 30.0f && Angle < -30.0f)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);
				HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
				HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);	
			}
			//usb_printf("Current Angle = %.2f\r\n",Angle);
			float last_encoder_count;
			int32_t encoder_count = __HAL_TIM_GET_COUNTER(&htim2);
			int32_t diff = encoder_count - last_encoder_count;
			// 處理溢位（避免從65535跳0）
			if (diff > 32767) diff -= 65536;
			if (diff < -32767) diff += 65536;

			last_encoder_count = encoder_count;

			float wheel_rps = (float)diff / 39600.0f / 0.01f;  // 假設每10ms更新一次
			float wheel_rpm = wheel_rps * 60.0f;
			usb_printf("Current rpm = %.2f\r\n",wheel_rpm);
		}
		//HAL_Delay(15);
		
		/*當馬達開始轉動時啟用PID控制
		if(rpm>100000)
		{
			pwm_pul = speed_PID(&SpeedPID ,rpm);
			if(pwm_pul<0)
			{
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
			}
			else if (pwm_pul>1500)
			{
			pwm_pul=1500;
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,1500);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,1500);
			}
			uint32_t compare_value = (uint32_t) (pwm_pul); 
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,compare_value);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,compare_value);
			usb_printf("Current Speec = %.2f\r\n",rpm);
		}
		usb_printf("Current Speec = %.2f\r\n",rpm);
		*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//usb_printf("Current speed = %.2f\r\n",rpm_right);
		//usb_printf("Current speed = %.2f\r\n",rpm_left);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

float balance_PID_main(Balance_PID *pid, float Angle,float ygyro)
{			
	uint32_t now = HAL_GetTick();
  float dt = (now - last_tick) * 0.001f;      // 秒
	last_tick = now;
	
	
	
	pid ->Error = pid ->target_val - Angle;		//Õ`²î=Ä¿˜ËœpŒëH
	I_term += pid->Ki * pid ->Error * dt;
	if (pid->Ki > PWM_MAX)  pid->Ki = PWM_MAX;
  if (pid->Ki < PWM_MIN)  pid->Ki = PWM_MIN;
	/* 前饋項 */
  float FF_term = Kff * ygyro;
	
	pid ->output_val = FF_term+(pid->Kp * pid->Error)+I_term - (pid ->Kd * ygyro-0.0f);
	
	if (pid ->output_val > PWM_MAX)  pid ->output_val = PWM_MAX;
  if (pid ->output_val < PWM_MIN)  pid ->output_val = PWM_MIN;
	
	return pid->output_val;
}



float get_motor_rpm()
{
    static float last_rpm = 0;
    static uint32_t last_time = 0;

		
    uint32_t now = HAL_GetTick();
		//float dt = 0.001;
    float dt = (now - last_time) / 1000.0f;
    if (dt < 0.05f) return last_rpm;  // 至少 50ms 間隔
    last_time = now;

    int16_t encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);  // 歸零

    float turns = (float)encoder_count / (4.0f * REDUCTION_RATIO * CPR);
    float rpm = turns / dt * 60.0f;  // 保留正負號（正轉/反轉）

    // 死區處理（保留方向）
    if (fabsf(rpm) < MIN_RPM_THRESHOLD) rpm = 0.0f;

    // 平滑處理
    //rpm = rpm * 0.7f + last_rpm * 0.3f;
    //last_rpm = rpm;

    return rpm;
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
	  /* 獲取輸入捕獲值 */
	  IC1Value = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);
	  IC2Value = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_2);	
	  if (IC1Value != 0)
	  {
		//* 占空比計算 */
		DutyCycle = (float)((IC2Value+1) * 100) / (IC1Value+1);
 
			/* 頻率計算 */
		Frequency = 72000000/72/(float)(IC1Value+1);
		
	  }
	  else
	  {
		DutyCycle = 0;
		Frequency = 0;
	  }
		
  }
	
	if(htim->Instance == TIM3)
{
	// 讀取兩個通道的捕獲值
   capture1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
   capture2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    // 分別計算兩通道的時間差
    delta_t1 = (capture1 >= last_capture1) ? 
               (capture1 - last_capture1) : 
               (0xFFFF - last_capture1 + capture1);

    delta_t2 = (capture2 >= last_capture2) ? 
               (capture2 - last_capture2) : 
               (0xFFFF - last_capture2 + capture2);

    // 更新前次值
    last_capture1 = capture1;
    last_capture2 = capture2;

    // 平均兩個通道的 delta_t，提高穩定性
    delta_t_avg = (delta_t1 + delta_t2) / 2.0f;

    // 避免除以 0
    if (delta_t_avg == 0 || pulses_per_rev == 0)
    {
        rpm_left = 0.0f;
    }
    else
    {
        time_s = delta_t_avg / 72000000.0f;  // 72 MHz 時鐘
        rpm_left = (1.0f / time_s) * (60.0f / pulses_per_rev);
				usb_printf("Current speed = %.2f\r\n",rpm_left);
    }	
}
	if(htim->Instance == TIM4)
{
	// 讀取兩個通道的捕獲值
   capture1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
   capture2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    // 分別計算兩通道的時間差
    delta_t1 = (capture1 >= last_capture1) ? 
               (capture1 - last_capture1) : 
               (0xFFFF - last_capture1 + capture1);

    delta_t2 = (capture2 >= last_capture2) ? 
               (capture2 - last_capture2) : 
               (0xFFFF - last_capture2 + capture2);

    // 更新前次值
    last_capture1 = capture1;
    last_capture2 = capture2;

    // 平均兩個通道的 delta_t，提高穩定性
    delta_t_avg = (delta_t1 + delta_t2) / 2.0f;

    // 避免除以 0
    if (delta_t_avg == 0 || pulses_per_rev == 0)
    {
        rpm_right = 0.0f;
    }
    else
    {
        time_s = delta_t_avg / 72000000.0f;  // 72 MHz 時鐘
        rpm_right = (1.0f / time_s) * (60.0f / pulses_per_rev);
				usb_printf("Current speed = %.2f\r\n",rpm_right);
    }	
}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{
		mpu_ready = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        if (rx_data == '\r' || rx_data == '\n') {
            if (idx > 0) {
                rx_buffer[idx] = '\0';  // 結尾補 0
                int offset = 0;
                if (sscanf(rx_buffer, "%d", &offset) == 1) {
                    usb_printf("📥 接收到 Offset = %d\r\n", offset);
                    // 這裡你可以儲存 offset 給 PID 控制器使用
                    pid_offset = offset;  // 你定義的變數
                } else {
                    usb_printf("⚠️ 無法解析 Offset，收到：[%s]\r\n", rx_buffer);
                }
                idx = 0;  // 重設 index
            }
            // 否則是連續 \r\n 其中一個，忽略
        } else {
            if (idx < sizeof(rx_buffer) - 1) {
                rx_buffer[idx++] = rx_data;
            } else {
                // 太長的字串，自動重置（避免溢出）
                idx = 0;
            }
        }

        HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_data, 1);  // 再次開啟中斷接收
    }
		huart_flag = 1;
}
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
	//if(GPIO_Pin == GPIO_PIN_0)
	//{
		//mpu_ready = 1;		//通知主迴圈：MPU 資料更新了
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // 假設你用 PC13 連LED
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, GPIO_PIN_RESET);
		//usb_printf("INT trigged! \r\n");
		//usb_printf("INT Callback triggered! Pin = %d\r\n", GPIO_Pin);
	//}
//}
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
