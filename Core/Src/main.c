/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "math.h"
#include "String.h"

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
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

////////////Joystick/////////////////////////////

uint8_t Bt_Data[8];
uint8_t Mode,Speed,Steer_mode,Steer_Angle,Joystick,Sheering=0;
uint8_t Bt_Count;

/////////////////////////////////////////////////

/////////////CAN/////////////////////////////////
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t TxMailBox;
uint16_t Node_Id[20];
uint8_t Received_Command_Id=0;
uint8_t Received_Node_Id=0;
uint16_t Drive_Node_id=0;
uint8_t close_loop[4]={8,0,0,0};


#define  VEL_ID  0x0D;
////////////////////////////////////////////////

//////////////////IMU_PID/////////////////////////
float D_Yaw,D_Pitch,D_Roll=0;
uint8_t I2cReadBuff[11];
uint8_t I2cWriteBuff[11];
int Address_imu  = 0x28<<1;
float Err_Roll,Err_Pitch=0;
float Err_Slope_Roll,Err_Slope_Pitch=0;
float Err_Area_Roll,Err_Area_Pitch=0;
float Home_Roll=-0.35,Home_Pitch=0.78;
float Pitch_boundary=0.5;
float D_Kp_Roll=0.06,D_Kp_Pitch=0.065;
float D_Ki_Roll=0,D_Ki_Pitch=0.15;
float D_Kd_Roll=0,D_Kd_Pitch=0.25;
float Roll_Speed,Pitch_Speed=0;
float Roll_Speed_PID,Pitch_Speed_PID=0;
float Roll_Speed_P,Roll_Speed_I,Roll_Speed_D=0;
float Pitch_Speed_P,Pitch_Speed_I,Pitch_Speed_D=0;
uint16_t Gimbal_id_temp=0;
float Error_Change_Pitch_PID=0;
float Prev_Error_Roll=0;
float Prev_Error_Pitch=0;
double dt=0.01 ;
///////////////////////////////////////////////////////

////////////////////Wheels//////////////////////////////
	uint8_t Wheel_Id[4];
	uint16_t Wheel_Id_temp;
	float Velocity[4]={0,0,0,0};
	float SteerTestVelocity = 1;
	uint8_t wheel_speed_temp=0;
	uint16_t count=0;
	uint8_t Test,Test_temp=0;
///////////////////////////////////////////////////////

////////////////Skid-Turn//////////////////////////////
float Skid_Factor=0;
float W_Velocity=1.5;
float skid_speed=0;
float Steer_Angle_temp=0;
float actual_speed=0;
///////////////////////////////////////////////////////
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C3_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
////////////////// Joystick/////////////////////////////////////////////////////
void Joystick_Data(void){
	
	if((Bt_Data[0]==0xAA && Bt_Data[7]==0xFF ))
		{
			Mode = Bt_Data[1];
			 
			Speed = Bt_Data[2];
			
			Steer_mode = Bt_Data[3];
			
			Steer_Angle =Bt_Data[4] ;
			
			Joystick= Bt_Data[5];
			
			Sheering = Bt_Data[6];
		}
 }

 //////////////////////////////////////////////////////////////////////////////
 
 ///////////////////////////CAN////////////////////////////////////////////////
 
 float float16_to_decimal(uint16_t float16) {
    // Extract the sign, exponent, and mantissa
    uint16_t sign = (float16 >> 15) & 0x1;
    uint16_t exponent = (float16 >> 10) & 0x1F;
    uint16_t mantissa = float16 & 0x3FF;
   
 
    float result;
        // Convert exponent to actual exponent
        int exp = (int)exponent - 15; // Adjust exponent by bias
        // Convert mantissa to fraction
        float frac = (float)mantissa / 1024.0f;
        // Combine sign, exponent, and mantissa
 
         double decimal_value = pow(-1, sign) * (1 + mantissa/ 1024.0) * pow(2, exponent - 15);
        //result = ldexpf(1.0f + frac, exp);
        //result = (sign ? -result : result);
 
 
    return decimal_value;
}
 
	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
	
	Received_Node_Id = RxHeader.StdId >> 5;
	Received_Command_Id = RxHeader.StdId & 0x01F;
	
  if (Received_Command_Id == 0x01)
	{
	   Node_Id[Received_Node_Id]++;
	}	

  if(RxHeader.StdId == 0x013){
		Node_Id[9]++;
		uint16_t Pitch=RxData[0]<<8|RxData[1];
		D_Pitch=float16_to_decimal(Pitch);
		uint16_t Roll=RxData[2]<<8|RxData[3];
		D_Roll=float16_to_decimal(Roll);
	  uint16_t Yaw=RxData[4]<<8|RxData[5];
		D_Yaw=float16_to_decimal(Yaw);
	}	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
			HAL_UART_Receive_IT(&huart4,Bt_Data ,sizeof(Bt_Data));
//			Joystick_Reception();
			
}
	int CAN_Transmit(uint16_t Std_Id,uint8_t dlc,uint8_t rtr,uint8_t *data)
	{
		TxHeader.StdId=Std_Id;
		TxHeader.ExtId=CAN_ID_STD;
		TxHeader.DLC=dlc;
		TxHeader.RTR=rtr;
		
		if(HAL_CAN_AddTxMessage(&hcan1,&TxHeader,data,&TxMailBox)== HAL_OK)
		{
			return 1;
		}
		else{ return 0;}
		
	}
 //////////////////////////////////////////////////////////////////////////////

////////////////////////////////CLOSED_LOOP////////////////////////////////////
void  is_Armed( uint8_t node_id)
{
	
	Drive_Node_id=(node_id << 5 )| 0x007;
	memcpy(TxData,&close_loop,4);
	
	CAN_Transmit(Drive_Node_id,4,CAN_RTR_DATA,TxData);
	

}	
///////////////////////////////////////////////////////////////////////////////

//////////////////////////////Wheels_Speed/////////////////////////////////////

void  Wheels_speed(uint16_t id,uint8_t* data)
{	
		 Wheel_Id_temp= (id << 5 )| VEL_ID;
		 CAN_Transmit(Wheel_Id_temp,4,CAN_RTR_DATA,data);	

}
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////Gimbal_Speed////////////////////////////////////
void Gimbal_Speed(uint16_t id, uint8_t* data)
{
	
	Gimbal_id_temp=(id << 5) | VEL_ID;
	CAN_Transmit(Gimbal_id_temp,4,CAN_RTR_DATA,data);
	HAL_Delay(1);
}
//////////////////////////////////////////////////////////////////////////////

///////////////////////////Wheels_Block//////////////////////////////////////
void Wheels_Block()
{

	
	if(Joystick != Test_temp && Steer_Angle == 90 &&  Steer_mode == 1)
	{
	  switch (Joystick)
	  {		
		case 1 : 
			     
		       for(int i=1;i<7;i++)
           { 
						Velocity[0] = ((i == 4) || (i == 6))? -0.5*Speed :((i == 2) || (i == 5))? 0: 0.5*Speed;
					  memcpy(TxData,&Velocity,4);						 
						Wheels_speed(i,TxData);HAL_Delay(5);
					 }
					 break;
            					 
				   
    case 2 : 
			     	
		       for(int i=1;i<7;i++)
           {
             Velocity[0] = ((i == 4) || (i == 6))? 0.5*Speed :((i == 2) || (i == 5))? 0: -0.5*Speed;
             memcpy(TxData,&Velocity,4);							 
		         Wheels_speed(i,TxData);HAL_Delay(5);
					 }
					 break;	
   
    case 0 : 
			      Velocity[0]=0;						 
						memcpy(TxData,&Velocity,4);
		       for(int i=1;i<7;i++)
           {	
		         Wheels_speed(i,0);HAL_Delay(5);
					 }
					 break;
					 
		default :
             break;			
     }
		Test_temp=Joystick;
  }
}
/////////////////////////////////////////////////////////////////////////////////

///////////////////////////Skid-Turning//////////////////////////////////////////
void Skid_Turn()
{
	if(Steer_Angle!= Steer_Angle_temp && Joystick !=0)
	{
	Skid_Factor= Steer_Angle -90;
	skid_speed= ((W_Velocity) - (fabs(Skid_Factor)/30));
	actual_speed=W_Velocity-skid_speed;
		if(Steer_Angle < 90)
		{
			if(Skid_Factor < 0 && Skid_Factor >=-90)	
			{
			for(int i=0; i<7;i++)
			{
		  Velocity[0]=(i == 4 || i == 5 || i == 6 )?-1:0.25;
		  memcpy(TxData,&Velocity,4);
	  	Wheels_speed(i,TxData);
      }
		  }
//			if(Skid_Factor <= -50 && Skid_Factor >=-90)
//			{
//				for(int i=0; i<7;i++)
//			{
//		  Velocity[0]=(i == 4 || i == 5 || i == 6 )?-W_Velocity:-W_Velocity;
//		  memcpy(TxData,&Velocity,4);
//	  	Wheels_speed(i,TxData);
//			}
//		 }
		}
		if(Steer_Angle > 90)
		{
		if(Skid_Factor > 0 && Skid_Factor <=90)
			{
			for(int i=0; i<7;i++)
			{
		  Velocity[0]=(i == 4 || i == 5 || i == 6 )?-0.25:1;
		  memcpy(TxData,&Velocity,4);
	  	Wheels_speed(i,TxData);
      }
		  }
//			if(Skid_Factor >= 50 && Skid_Factor <=90)
//			{
//				for(int i=0; i<7;i++)
//			{
//		  Velocity[0]=(i == 1 || i == 2 || i == 3 )?skid_speed:-skid_speed;
//		  memcpy(TxData,&Velocity,4);
//	  	Wheels_speed(i,TxData);
//			}
//			}
		}
			Steer_Angle_temp = Steer_Angle;
		}

}
///////////////////////////////////////////////////////////////////////////

////////////////////////////Gimble//////////////////////////////////////
void Gimble()
{
	Err_Roll = D_Roll - Home_Roll;
	Roll_Speed=Roll_PID(Err_Roll);
	
	Err_Pitch = D_Pitch - Home_Pitch;
	Pitch_Speed=Pitch_PID(Err_Pitch);
	//Pitch_Speed=Pitch_Speed*(0.01);
	
	memcpy(TxData,&Roll_Speed,4);
	Gimbal_Speed(8,TxData);
	
	//Pitch_Speed= Pitch_Speed < -1.25 ? -1.25 : Pitch_Speed > 1.25 ? 1.25 : Pitch_Speed;	
	
	Pitch_Speed = ((D_Pitch < (Home_Pitch + Pitch_boundary)) && (D_Pitch > (Home_Pitch - Pitch_boundary))) ? 0:Pitch_Speed;
	Pitch_Speed= Pitch_Speed < 0.05 && Pitch_Speed > -0.05 ? 0 : Pitch_Speed;
	//Pitch_Speed=(D_Pitch <-2.78&& D_Pitch > -2.78) || (D_Pitch >178 && D_Pitch <180) ? 0:Pitch_Speed;
 	memcpy(TxData,&Pitch_Speed,4);
	 Gimbal_Speed(7,TxData);
		
}
////////////////////////////////////////////////////////////////////

///////////////////////////////PID/////////////////////////////////
float Roll_PID(float Error){
	
   float Error_Change_Roll_PID=Error-Prev_Error_Roll;
	 Err_Slope_Roll=Error_Change_Roll_PID/dt;
	 Err_Area_Roll=Err_Area_Roll+(Error_Change_Roll_PID*dt);
	
	 Roll_Speed_P=Error * D_Kp_Roll;
	 Roll_Speed_I=Err_Area_Roll * D_Ki_Roll;
	 Roll_Speed_D=Err_Slope_Roll * D_Kd_Roll;
	
	 Roll_Speed_PID=Roll_Speed_P+Roll_Speed_I+Roll_Speed_D;
	
	 Prev_Error_Roll=Error;
	
	 return Roll_Speed_PID;
}

float Pitch_PID(float Error){
	
   Error_Change_Pitch_PID=Error-Prev_Error_Pitch;
	 Err_Slope_Pitch=Error_Change_Pitch_PID/dt;
	 Err_Area_Pitch=Err_Area_Pitch+(Error_Change_Pitch_PID*dt);
	
	Pitch_Speed_P=Error*D_Kp_Pitch;
  Pitch_Speed_I=Err_Area_Pitch * D_Ki_Pitch;
	Pitch_Speed_D=Err_Slope_Pitch * D_Kd_Pitch;
	
	Pitch_Speed_PID=Pitch_Speed_P+Pitch_Speed_I+Pitch_Speed_D;
	Prev_Error_Pitch=Error;
	
	 return Pitch_Speed_PID;
}
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////Frame_Block///////////////////////////////
void Frame_Block()
{
	
	if(Joystick != Test_temp && Steer_mode == 3 )
	{
		
	  switch (Joystick)
	  {		
		case 1 : 
			     
		       
						Velocity[0] = 0.25;
					  memcpy(TxData,&Velocity,4);						 
						Wheels_speed(8,TxData);HAL_Delay(1);
					 
					 break;
            					 
				   
    case 2 : 
			     	
		       
             Velocity[0] = -0.25;
             memcpy(TxData,&Velocity,4);							 
		         Wheels_speed(8,TxData);HAL_Delay(1);
					 
					 break;	
   
    case 0 : 
			      Velocity[0]=0;						 
						memcpy(TxData,&Velocity,4);
		       
		         Wheels_speed(8,0);HAL_Delay(1);
					 
					 break;
					 
		default :
             break;			
     }
	 }
		
 else if(Joystick != Test_temp && Steer_mode == 4 )
	{
	  switch (Joystick)
	  {		
		case 1 : 
			     
		       
						Velocity[0] = 0.25;
					  memcpy(TxData,&Velocity,4);						 
						Wheels_speed(7,TxData);HAL_Delay(1);
					 
					 break;
            					 
				   
    case 2 : 
			     	
		       
             Velocity[0] = -0.25;
             memcpy(TxData,&Velocity,4);							 
		         Wheels_speed(7,TxData);HAL_Delay(1);
					 
					 break;	
   
    case 0 : 
			      Velocity[0]=0;						 
						memcpy(TxData,&Velocity,4);
		       
		         Wheels_speed(7,0);HAL_Delay(1);
					 
					 break;
					 
		default :
             break;			
     }
	 }
		Test_temp=Joystick;
 }
////////////////////////////////////////////////////////////////////////



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
  MX_UART4_Init();
  MX_I2C3_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart4,Bt_Data,sizeof(Bt_Data));
	
	HAL_CAN_Start(&hcan1);HAL_Delay(100);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_Delay(2000);
	for(int i=1;i<9;i++)
	{
		for(int j=0;j<2;j++){
	is_Armed(i);
	HAL_Delay(200);
  }
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

			Joystick_Data();
//		if (Steer_mode!=3 && Steer_mode!=4){
//		Gimble();

//		}
//		else{
		
	//	Frame_Block();
//		}
		
		Wheels_Block();
		//Read_IMU();
		
		
//		Skid_Turn();
		
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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 0;	// which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0x000<<5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0x000<<5;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 14; //14 // how many filters to assign to the CAN1 (master can)

	HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 12;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 14; //14 // which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	canfilterconfig.FilterIdHigh = 0x000<<5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0x000<<5;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 14;//14	// how many filters to assign to the CAN1 (master can)

	HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */


  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
