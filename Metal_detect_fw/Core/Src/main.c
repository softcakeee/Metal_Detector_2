/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Improved Version)
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h" // HAL 라이브러리에서 생성된 메인 헤더파일. 기본 주변장치 설정 및 선언 포함.

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> // printf 등을 사용하기 위한 표준 입출력 헤더파일
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define US_DELAY_BETWEEN_PULSES    3    // 커패시터 충전용 펄스 사이의 간격 (마이크로초)
#define DISCHARGE_DELAY_US         20   // 커패시터 방전 대기 시간 (마이크로초)
#define PULSE_COUNT                3    // 커패시터 충전을 위한 펄스 발생 횟수

#define RX_BUFFER_SIZE  10 // UART 수신 버퍼 사이즈
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1; // ADC1 핸들 구조체(ADC 설정 및 사용을 위해 필요)
TIM_HandleTypeDef htim1; // us 단위 딜레이용 타이머(TIM1)
TIM_HandleTypeDef htim2; // 부저(PWM 등)로 사용되는 타이머(TIM2)
TIM_HandleTypeDef htim3; // 모터 제어용 PWM 타이머(TIM3)
TIM_HandleTypeDef htim4; // 모터 제어용 PWM 타이머(TIM4)
UART_HandleTypeDef huart1; // UART1 핸들 구조체(UART 설정 및 사용)

uint8_t rx_data; // UART로부터 한 바이트씩 수신할 때 임시 저장하는 변수
uint8_t rx_buffer[RX_BUFFER_SIZE]; // 수신 데이터 버퍼 (최대 10바이트)
uint8_t rx_index = 0; // 수신 버퍼 인덱스 관리용 변수

uint8_t Gearbox = 0;       // 전진/후진 결정하는 변수(0: 후진, 1: 전진)
uint8_t Turn_Direction = 0;// 좌/우회전 결정하는 변수(2: 우회전, 3: 좌회전)
uint16_t PWM_Value = 0;    // 전/후진 모터 속도값
uint16_t Turn_Value = 0;   // 좌/우회전 시 차감/추가되는 속도값
uint32_t ADC_CAP_VAL[3] = {0}; // ADC로 읽은 커패시터 충전값 저장 배열(Left, Middle, Right)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);          // 시스템 클럭 설정 함수
static void MX_GPIO_Init(void);         // GPIO 초기화 함수
static void MX_TIM3_Init(void);         // TIM3 초기화 함수(PWM용)
static void MX_TIM4_Init(void);         // TIM4 초기화 함수(PWM용)
static void MX_USART1_UART_Init(void);  // UART1 초기화 함수
static void MX_ADC1_Init(void);         // ADC1 초기화 함수
static void MX_TIM1_Init(void);         // TIM1 초기화 함수(마이크로초 딜레이용)
static void MX_TIM2_Init(void);         // TIM2 초기화 함수(부저 등 활용 가능)

int __io_putchar(int ch);              // printf 리디렉션 함수
void delay_us(uint16_t us);            // us 단위 딜레이 함수

static void PWM_START(void);           // PWM 채널 시작 함수
static void PWM_RIGHT_FRONT_SPEED(uint16_t speed_value); // 오른쪽 전진용
static void PWM_LEFT_FRONT_SPEED(uint16_t speed_value);  // 왼쪽 전진용
static void PWM_RIGHT_BACK_SPEED(uint16_t speed_value);  // 오른쪽 후진용
static void PWM_LEFT_BACK_SPEED(uint16_t speed_value);   // 왼쪽 후진용

static void applyPulses(GPIO_TypeDef* port, uint16_t pin);        // 펄스 발생 함수(커패시터 충전용)
static void Discharge_Capacitor(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin); // 커패시터 방전 함수
static void ADC_Read_Channel(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t *adc_value); // 지정 채널에서 ADC 읽기 함수

static void Parse_Received_Data(void);   // 수신 버퍼로부터 데이터 파싱 함수
static void Perform_Cap_Measurement(void);// 커패시터 측정 및 결과 송신 함수
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  printf를 UART로 송출하기 위한 함수
  * @param  ch: 전송할 문자
  * @retval int
  */
int __io_putchar(int ch)
{
	if (ch == '\n')
		HAL_UART_Transmit(&huart1, (uint8_t*) "\r", 1, HAL_MAX_DELAY); // '\n' 앞에 '\r'를 추가
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);      // 실제 문자 전송
	return ch;
}

/**
  * @brief  us 단위 딜레이 함수
  * @param  us: 지연시킬 마이크로초
  * @retval None
  */
void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);  // 타이머 카운터값을 0으로 리셋
	while(__HAL_TIM_GET_COUNTER(&htim1) < us); // 지정된 us만큼 대기
}

/**
  * @brief  커패시터 충전용 펄스를 발생시키는 함수
  * @param  port: GPIO 포트
  * @param  pin : GPIO 핀
  * @retval None
  */
static void applyPulses(GPIO_TypeDef* port, uint16_t pin)
{
    for (int i = 0; i < PULSE_COUNT; i++)
    {
      HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);   // 핀 HIGH
      delay_us(US_DELAY_BETWEEN_PULSES);            // 일정 시간 대기
      HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); // 핀 LOW
      delay_us(US_DELAY_BETWEEN_PULSES);            // 일정 시간 대기
    }
}

/**
  * @brief  모든 PWM 채널을 시작하는 함수
  * @retval None
  */
static void PWM_START()
{
	// TIM3의 4개 채널 PWM 시작
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	// TIM4의 4개 채널 PWM 시작
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

/**
  * @brief  오른쪽 모터 전진 속도 제어
  * @param  speed_value: PWM 듀티값
  * @retval None
  */
static void PWM_RIGHT_FRONT_SPEED(uint16_t speed_value)
{
	htim3.Instance->CCR1 = speed_value; // 채널1에 속도값
	htim3.Instance->CCR2 = 0;          // 채널2는 0

	htim4.Instance->CCR1 = speed_value;
	htim4.Instance->CCR2 = 0;
}

/**
  * @brief  왼쪽 모터 전진 속도 제어
  * @param  speed_value: PWM 듀티값
  * @retval None
  */
static void PWM_LEFT_FRONT_SPEED(uint16_t speed_value)
{
	htim3.Instance->CCR3 = speed_value;
	htim3.Instance->CCR4 = 0;

	htim4.Instance->CCR4 = speed_value;
	htim4.Instance->CCR3 = 0;
}

/**
  * @brief  오른쪽 모터 후진 속도 제어
  * @param  speed_value: PWM 듀티값
  * @retval None
  */
static void PWM_RIGHT_BACK_SPEED(uint16_t speed_value)
{
	htim3.Instance->CCR2 = speed_value;
	htim3.Instance->CCR1 = 0;

	htim4.Instance->CCR2 = speed_value;
	htim4.Instance->CCR1 = 0;
}

/**
  * @brief  왼쪽 모터 후진 속도 제어
  * @param  speed_value: PWM 듀티값
  * @retval None
  */
static void PWM_LEFT_BACK_SPEED(uint16_t speed_value)
{
	htim3.Instance->CCR4 = speed_value;
	htim3.Instance->CCR3 = 0;

	htim4.Instance->CCR3 = speed_value;
	htim4.Instance->CCR4 = 0;
}

/**
  * @brief  커패시터를 방전시키는 함수
  * @param  GPIOx: GPIO 포트
  * @param  GPIO_Pin: GPIO 핀
  * @retval None
  */
static void Discharge_Capacitor(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // 방전을 위해 Pin을 Push-Pull Output으로 설정
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

  // LOW로 출력하여 방전
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
  delay_us(DISCHARGE_DELAY_US);

  // 다시 ADC 측정을 위해 Pin을 Analog 모드로 전환
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
  * @brief  특정 ADC 채널에서 한 번 읽어오는 함수
  * @param  hadc: ADC 핸들
  * @param  channel: 읽고자 하는 채널
  * @param  adc_value: 결과를 저장할 변수 포인터
  * @retval None
  */
static void ADC_Read_Channel(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t *adc_value)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = channel;                              // 채널 설정
  sConfig.Rank = ADC_REGULAR_RANK_1;                      // 순서
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;       // 샘플링 타임

  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    Error_Handler(); // 에러 시 사용자 정의 핸들러
  }

  if (HAL_ADC_Start(hadc) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY) != HAL_OK)
  {
    Error_Handler();
  }

  *adc_value = HAL_ADC_GetValue(hadc); // 읽어온 ADC 값을 포인터에 저장

  if (HAL_ADC_Stop(hadc) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  수신 버퍼 데이터를 실제 명령/속도 값으로 파싱하는 함수
  * @retval None
  */
static void Parse_Received_Data(void)
{
	// rx_buffer[] 구조:
	//  [0]        : Gearbox (0 또는 1)
	//  [1~4]      : PWM_Value 4자리
	//  [5]        : Turn_Direction (2 또는 3)
	//  [6~9]      : Turn_Value 4자리

	Gearbox = rx_buffer[0];

	// PWM_Value 복원
	uint16_t temp_pwm = (uint16_t)(rx_buffer[1] * 1000 + rx_buffer[2] * 100 + rx_buffer[3] * 10 + rx_buffer[4]);
	PWM_Value = temp_pwm;

	// 회전 방향
	Turn_Direction = rx_buffer[5];

	// Turn_Value 복원
	uint16_t temp_turn = (uint16_t)(rx_buffer[6] * 1000 + rx_buffer[7] * 1 + rx_buffer[8] * 10 + rx_buffer[9]);
	Turn_Value = temp_turn;
}

/**
  * @brief  커패시터 3개(Left, Middle, Right)의 충전값을 측정하고 UART로 송신
  * @retval None
  */
static void Perform_Cap_Measurement(void)
{
	// Left Cap 측정
	Discharge_Capacitor(GPIOA, GPIO_PIN_1);         // 커패시터 방전
	applyPulses(PULSE_PIN_LEFT_GPIO_Port, PULSE_PIN_LEFT_Pin); // 펄스 발생
	ADC_Read_Channel(&hadc1, ADC_CHANNEL_1, &ADC_CAP_VAL[0]);  // ADC 읽기

	// Middle Cap 측정
	Discharge_Capacitor(GPIOA, GPIO_PIN_2);
	applyPulses(PULSE_PIN_MIDDLE_GPIO_Port, PULSE_PIN_MIDDLE_Pin);
	ADC_Read_Channel(&hadc1, ADC_CHANNEL_2, &ADC_CAP_VAL[1]);

	// Right Cap 측정
	Discharge_Capacitor(GPIOA, GPIO_PIN_3);
	applyPulses(PULSE_PIN_RIGHT_GPIO_Port, PULSE_PIN_RIGHT_Pin);
	ADC_Read_Channel(&hadc1, ADC_CHANNEL_3, &ADC_CAP_VAL[2]);

	// 측정 결과를 UART로 출력(파이썬에서 파싱하기 좋게 CSV 포맷)
	printf("%ld,%ld,%ld\n", ADC_CAP_VAL[0], ADC_CAP_VAL[1], ADC_CAP_VAL[2]);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // 메인 함수 시작 전 초기화할 변수/설정이 있다면 여기서 작성
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init(); // HAL 라이브러리 초기화(인터럽트/타이머 등)

  /* Configure the system clock */
  SystemClock_Config(); // 시스템 클럭 설정

  /* Initialize all configured peripherals */
  MX_GPIO_Init();        // GPIO 초기화
  MX_TIM3_Init();        // 타이머3(PWM) 초기화
  MX_TIM4_Init();        // 타이머4(PWM) 초기화
  MX_USART1_UART_Init(); // UART1 초기화
  MX_ADC1_Init();        // ADC1 초기화
  MX_TIM1_Init();        // 타이머1(마이크로초 딜레이용) 초기화
  MX_TIM2_Init();        // 타이머2(부저 또는 추가 PWM) 초기화

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1); // TIM1 카운터 시작(마이크로초 딜레이에 사용)

  PWM_START(); // 모든 PWM 채널 시작
  HAL_UART_Receive_IT(&huart1, &rx_data, 1); // UART 인터럽트 수신 시작

  // ADC 핀(충전용) 초기 상태를 Low로 설정
  HAL_GPIO_WritePin(PULSE_PIN_LEFT_GPIO_Port, PULSE_PIN_LEFT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PULSE_PIN_MIDDLE_GPIO_Port, PULSE_PIN_MIDDLE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PULSE_PIN_RIGHT_GPIO_Port, PULSE_PIN_RIGHT_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // 기어박스가 전진(Gearbox=1)인지 후진(0)인지, 그리고 좌우 회전값에 따라
	  // 모터의 PWM을 제어하는 로직

	  // 안전장치: 회전 값이 전체 속도보다 크면 속도보다 큰 값을 차감하지 않도록 보정
	  if (PWM_Value < Turn_Value)
	  {
		  Turn_Value = PWM_Value;
	  }

	  if (Gearbox == 1) // 전진
	  {
		  if (Turn_Direction == 2) // 우회전
		  {
			  PWM_RIGHT_FRONT_SPEED(PWM_Value - Turn_Value); // 오른쪽 속도 줄임
			  PWM_LEFT_FRONT_SPEED(PWM_Value);               // 왼쪽은 그대로
		  }
		  else if (Turn_Direction == 3) // 좌회전
		  {
			  PWM_LEFT_FRONT_SPEED(PWM_Value - Turn_Value);  // 왼쪽 속도 줄임
			  PWM_RIGHT_FRONT_SPEED(PWM_Value);              // 오른쪽은 그대로
		  }
	  }
	  else if (Gearbox == 0) // 후진
	  {
		  if (Turn_Direction == 2) // 우회전
		  {
			  PWM_RIGHT_BACK_SPEED(PWM_Value - Turn_Value);
			  PWM_LEFT_BACK_SPEED(PWM_Value);
		  }
		  else if (Turn_Direction == 3) // 좌회전
		  {
			  PWM_LEFT_BACK_SPEED(PWM_Value - Turn_Value);
			  PWM_RIGHT_BACK_SPEED(PWM_Value);
		  }
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 메인 루프에서 계속 반복
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  // 내부 HSI 클럭을 사용하는 설정, PLL 비활성 등 기본 설정
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  // HSI 설정
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler(); // 클럭 설정 실패 시 에러 처리
  }

  // CPU, AHB, APB 클럭 설정
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; // HSI를 메인 클럭 소스로
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  // ADC 클럭 설정(ADC 전용 PCLK2 분주)
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  // ADC1 기본 설정: 단일 채널, SW 스타트, 지속 미사용 등
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;    // 스캔 모드 비활성화
  hadc1.Init.ContinuousConvMode = DISABLE;       // 연속 변환 모드 비활성화
  hadc1.Init.DiscontinuousConvMode = DISABLE;    // 디스컨티뉴어스 변환 비활성화
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START; // 소프트웨어 트리거
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;    // 오른쪽 정렬
  hadc1.Init.NbrOfConversion = 1;                // 변환 개수 1

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler(); // ADC 초기화 실패 시 에러
  }
}

/**
  * @brief TIM1 Initialization Function
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  // TIM1 설정: us 단위 딜레이용(프리스케일러 등 설정)
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8-1;      // 클럭 분주(예: 8MHz -> 1MHz)
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;       // 최대 카운터
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; // 내부 클럭 사용
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // 마스터 설정
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  // TIM2는 부저 PWM 혹은 추가 기능으로 사용 가능
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;           // PWM 주기 설정(8MHz/8 -> 1MHz)
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;            // 주기 1000 -> 1kHz
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // 채널1 PWM 설정
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;                       // 초기 듀티 0%
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2); // PWM 핀 초기화
}

/**
  * @brief TIM3 Initialization Function
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  // 모터 제어용 1kHz PWM 타이머3
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;       // (8MHz / 8) -> 1MHz
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;        // 1kHz (1MHz / 1000)
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // 4채널 공통 설정
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;                       // 초기 듀티 0%
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  // 채널1
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  // 채널2
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  // 채널3
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  // 채널4
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief TIM4 Initialization Function
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  // 모터 제어용 1kHz PWM 타이머4
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // 4채널 공통 설정
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  // 채널1
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  // 채널2
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  // 채널3
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  // 채널4
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);
}

/**
  * @brief USART1 Initialization Function
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  // UART1 기본 설정
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;               // 보드레이트
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
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  // GPIO 핀 초기화
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // GPIOA, GPIOB 클럭 활성화
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // PULSE_PIN_LEFT_Pin, PULSE_PIN_MIDDLE_Pin, PULSE_PIN_RIGHT_Pin을 출력 Low로 초기화
  HAL_GPIO_WritePin(GPIOA, PULSE_PIN_LEFT_Pin|PULSE_PIN_MIDDLE_Pin|PULSE_PIN_RIGHT_Pin, GPIO_PIN_RESET);

  // 펄스 핀 3개를 Push-Pull Output 모드로 설정
  GPIO_InitStruct.Pin = PULSE_PIN_LEFT_Pin|PULSE_PIN_MIDDLE_Pin|PULSE_PIN_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/**
  * @brief UART Rx Complete Callback
  * @param huart: UART 핸들
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) // UART1에 대한 인터럽트 콜백
	{
		if (rx_data >= '0' && rx_data <= '9')
		{
			// 0~9 숫자면 버퍼에 저장
			if (rx_index < RX_BUFFER_SIZE)
			{
				rx_buffer[rx_index++] = rx_data - '0'; // 아스키 -> 숫자 변환
			}
		}

		// 줄바꿈 문자가 들어오면(= 명령 하나가 끝났을 때)
		if (rx_data == '\n' || rx_data == '\r')
		{
			if (rx_index == RX_BUFFER_SIZE)
			{
				Parse_Received_Data();   // 버퍼 파싱
				Perform_Cap_Measurement(); // 커패시터 측정 및 송신
			}
			rx_index = 0; // 인덱스 리셋
		}

		// 다음 바이트 수신 대기
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  // 에러 발생 시 인터럽트를 비활성화하고 무한루프
  __disable_irq();
  while (1)
  {
    // 사용자는 여기서 에러 로깅 등을 수행할 수 있음
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  assert_failed
  * @param  file: 소스 파일 이름
  * @param  line: 오류가 발생한 라인
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  // assert_param 매크로 에러 발생 시 호출됨
  // 필요시 디버깅 정보 출력 가능
}
#endif /* USE_FULL_ASSERT */
