  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/

#include <main.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"

#include "../../Drivers/SSD1306/ssd1306.h"

/* Global Enums & #defines -------------------------------------------------------*/
typedef enum drone_modes{STANDBY, BATTLE, RESCUE}drone_mode;

<<<<<<< HEAD
#define RESCUE_TIME_MS          10000
#define MULTI_PRESS_DETECT_MS   400
#define UART_SEND_PERIOD_MS     1000
#define DEBOUNCE_TIME_MS        50

#define UPSIDE_DOWN_ACCEL_MS2   -9

=======
#define RESCUE_TIME_MS             10000
#define MULTI_PRESS_DETECT_MS      500
#define UART_SEND_PERIOD_MS        1000
#define UPSIDE_DOWN_ACCEL_MS2      -8.5
#define SCREEN_UPDATE_PERIOD_MS    500
#define BUZZER_SOUND_TIME_MS       500
>>>>>>> current
#define PI 3.141592654

//#define DEBUG_MODE     //Comment out to use UART, leave to view printf

#ifdef DEBUG_MODE
extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)
#endif /* DEBUG_MODE */

/* Function Prototypes  --------------------------------------------------------*/

//Sensors

static void init_board_sensors();
static void update_accel(float accel_data[3]);
static void update_gyro(float gyro_data[3]);
static void update_mag(float mag_data[3]);
<<<<<<< HEAD
static void handle_HAL_error(int err);
=======
static int check_upside_down(float accel_data[3]);
>>>>>>> current


//Configurations(To call once or not used at all)
static void UART1_Init(void);
static void init_LSM_exti(void);

//LEDs

static void blink_leds(drone_mode mode);


//Push Button

static int check_button_press();


//UART

void print_UART(char msg[200]);
static void print_telemmentry(drone_mode mode,
					float accel_data[3], float gyro_data[3],float mag_data[3],
					float pres_data, float humi_data, float temp_data);

//Drawing

static void draw_tilt(float angle_deg);
static float calculate_tilt_angle(float *accel_data, float *gyro_data);

//Tilt detect
static void init_LSM_tilt_int(void);
static void init_buzzer_pin(void);

/* Global Variables --------------------------------------------------------*/

UART_HandleTypeDef huart1;
<<<<<<< HEAD
static volatile int button_presses;
static volatile int imu_data_ready;
=======
static volatile int exti_button_presses;
static volatile int sound_buzzer;
>>>>>>> current

int main(void)
{
	HAL_Init();

	drone_mode enum_current_mode = STANDBY;

#ifdef DEBUG_MODE
	initialise_monitor_handles();
#else
	UART1_Init();
#endif

	BSP_LED_Init(LED2);
<<<<<<< HEAD
	
	init_push_button();
	init_LSM_exti();
	MX_GPIO_Init();
=======
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
	init_buzzer_pin();
>>>>>>> current

	init_board_sensors();
	ssd1306_Init();
	
	init_LSM_tilt_int();
	print_UART("Board setup OK \r\n");

	/* Params */
	uint32_t last_data_read = 0;
	uint32_t last_screen_update = 0;
	uint32_t time_enter_rescue;
	uint32_t time_now;

	float accel_data[3];
	float gyro_data[3];
	float mag_data[3];
	float temp_data, humi_data, pres_data;

<<<<<<< HEAD
	button_presses = 0;
	imu_data_ready = 1;

=======
	exti_button_presses = 0;
	sound_buzzer = 0;
	int buzzer_on = 0;

	uint32_t buzzer_start_time = 0;
>>>>>>> current
	int laser_ammo = 0;

	int activate_oled = 1;

	char UART_msg[200];
	char drone_mode_text[20];

	while (1)
	{	
		time_now = HAL_GetTick();
		blink_leds(enum_current_mode);
		
		if(imu_data_ready){
			update_accel(accel_data);
			update_gyro(gyro_data);
			imu_data_ready = 0;
		}

		//Check change of mode and send status over UART
		int but_press = check_button_press();
		if(but_press == 2){
			if(enum_current_mode == STANDBY){
				enum_current_mode = BATTLE; 
				sprintf(drone_mode_text,"BATTLE");
			}
			else if(enum_current_mode == BATTLE){
				enum_current_mode = STANDBY;
				sprintf(drone_mode_text,"STANDBY");
			}
			else if(enum_current_mode == RESCUE){
				enum_current_mode = BATTLE;
			}

			sprintf(UART_msg,"Entering %s mode \r\n", drone_mode_text);
			print_UART(UART_msg);
<<<<<<< HEAD

			last_data_read = time_now; //We want UART Data to be sent 1s after
			}
=======
		}
		else if(but_press >= 3){
			if(activate_oled == 1){
				ssd1306_Fill(Black);
				ssd1306_UpdateScreen();
				activate_oled = 0;
			}
			else{
				activate_oled = 1;
			}
		}
>>>>>>> current

		//Sample sensors and send tele via UART based on status
		//IMU (accel and gyro) data is updated through interrupts
		if(time_now - last_data_read > UART_SEND_PERIOD_MS){
			if( (enum_current_mode == STANDBY) || (enum_current_mode == BATTLE)){
<<<<<<< HEAD
				update_mag(mag_data);

=======
				if(!activate_oled){
					update_gyro(gyro_data);
				}
				update_mag(mag_data);
>>>>>>> current
				pres_data = BSP_PSENSOR_ReadPressure();
				humi_data = BSP_HSENSOR_ReadHumidity();
			}

			if(enum_current_mode == BATTLE){
				if(!activate_oled){
					update_accel(accel_data);
				}
				temp_data = BSP_TSENSOR_ReadTemp();
			}
			//else (enum_current_mode == RESUCE) -> dont sample at all

			print_telemmentry(enum_current_mode,
				accel_data,gyro_data,mag_data,pres_data,humi_data,temp_data);

			last_data_read = time_now;
		}


		//Check for resuce and laser gun charging
		if(enum_current_mode == BATTLE){
<<<<<<< HEAD
			if(accel_data[2] < UPSIDE_DOWN_ACCEL_MS2){
				//Upside down check for switching
				enum_current_mode = RESCUE;
				time_enter_rescue = time_now;
				last_data_read = time_now; //We want UART Data to be sent 1s after
=======
			if(check_upside_down(accel_data)){
				//Upside down check for switching
				enum_current_mode = RESCUE;
				time_enter_rescue = time_now;
				print_UART("Entering RESCUE mode \r\n");
				last_data_read = 0; //immediately write that drone is being attacked
>>>>>>> current
			}
			else if(but_press == 1){
				//LASERGUN_CHARGING
				if(laser_ammo <= 7){
					laser_ammo += 3;
					sprintf(UART_msg,"Reloading Laser! , Current Ammo : %i\r\n",laser_ammo);
					print_UART(UART_msg);
				}
			}	

			//LAZERGUN SHOOTING
			if(laser_ammo >= 5){
				laser_ammo -= 5;
				sprintf(UART_msg,"Shooting Laser! , Remaining Ammo : %i\r\n",laser_ammo);
				print_UART(UART_msg);
			}

		}
		else if((enum_current_mode == RESCUE) && (time_now - time_enter_rescue > RESCUE_TIME_MS)){
			print_UART("Drone has been captured \r\n");
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen();
			while(1); //Terminate program
		}


		//Buzzer (sound_buzzer flag) only activates when EXTI triggered in BATTLE mode
		if(sound_buzzer && enum_current_mode == BATTLE){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			buzzer_start_time = time_now;
			sound_buzzer = 0;
			buzzer_on = 1;
		}
		else if(sound_buzzer){
			sound_buzzer = 0;
		}
		else if((buzzer_on) && (time_now - buzzer_start_time > BUZZER_SOUND_TIME_MS)){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			buzzer_on = 0;
		}


		//Update OLED Screen
		if(activate_oled){
			if(time_now - last_screen_update > SCREEN_UPDATE_PERIOD_MS){
				update_gyro(gyro_data);
				update_accel(accel_data); // Update this for checking threshold and upside down
				draw_tilt(calculate_tilt_angle(accel_data, gyro_data));
				last_screen_update = time_now;
			}
		}

	} /* main while(1) loop */

} /*main func*/

/* Sensor Functions -----------------------------------------------------*/

/*Configuration (Init board sensors)
 *
 * Initialise all the sensors on the discovery board via BSP, and handle failed init
 *  - [LSM6DSL]Accel Gyro, [LIS3MDL]Mag, [HTS221]Humidity Temp, [LPS22HB]Pressure
 *  - If failed init, send error over UART, prompt user reset
 */
static void init_board_sensors()
{
	int sensors_ok = 1;

	if(BSP_ACCELERO_Init() != ACCELERO_OK){
		sensors_ok = 0;
		print_UART("ACCELERO not working \r\n");
	}

	uint8_t tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_INT1_CTRL);
	tmp &= ~(0xFE); //Clear the last digit
	tmp |= 0x1;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_INT1_CTRL, tmp);

	if(BSP_GYRO_Init() != GYRO_OK){
		sensors_ok = 0;
		print_UART("GYRO not working \r\n");
	}

	if(BSP_MAGNETO_Init() != MAGNETO_OK){
		sensors_ok = 0;
		print_UART("MAGNETO not working \r\n");
	}
	if(BSP_HSENSOR_Init()!= HSENSOR_OK){
		sensors_ok = 0;
		print_UART("HSENSOR not working \r\n");
	}
	if(BSP_TSENSOR_Init()!= TSENSOR_OK){
		sensors_ok = 0;
		print_UART("TSENSOR not working \r\n");
	}
	if(BSP_TSENSOR_Init()!= TSENSOR_OK){
		sensors_ok = 0;
		print_UART("TSENSOR not working \r\n");
	}
	if(BSP_PSENSOR_Init()!= PSENSOR_OK){
		sensors_ok = 0;
		print_UART("PSENSOR not working \r\n");
	}

	if(!sensors_ok){
		print_UART("One or more sensors not working, freezing board now.");
		while(1);
	}

}

/* Update gyroscope readings in form (x,y,z) 
 *
 * Input  -> pointer to float array of size 3 to store readings.
 *  
 * Output -> None
 *
 */
static void update_gyro(float gyro_data[3])
{
	float gyro_data_float[3] = {0};
	BSP_GYRO_GetXYZ(gyro_data_float);

	//Include gyro offsets
	gyro_data[0] = (float)gyro_data_float[0]/1000.0f - 0.384;	//x
	gyro_data[1] = (float)gyro_data_float[1]/1000.0f - -1.23;	//y
	gyro_data[2] = (float)gyro_data_float[2]/1000.0f - 0.07;	//z
}

/* Update accel readings in form (x,y,z). 
 *
 * Input  -> pointer to float array of size 3 to store readings. 
 * 
 * Output -> None
 *
 */
static void update_accel(float accel_data[3])
{
	int16_t accel_data_i16[3] = {0};
	BSP_ACCELERO_AccGetXYZ(accel_data_i16);
	accel_data[0] = (float)accel_data_i16[0] * (9.8/1000.0f);	//x
	accel_data[1] = (float)accel_data_i16[1] * (9.8/1000.0f);	//y
	accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);	//z

}

/* Update mag readings in form (x,y,z). 
 *
 * Input  -> pointer to float array of size 3 to store readings. 
 * 
 * Output -> None
 *
 */
static void update_mag(float mag_data[3])
{
	int16_t mag_data_i16[3] = {0};
	BSP_MAGNETO_GetXYZ(mag_data_i16);

	//Why do we need to change to float?
	mag_data[0] = (float)mag_data_i16[0]/1000.0f;	//x
	mag_data[1] = (float)mag_data_i16[1]/1000.0f;	//y
	mag_data[2] = (float)mag_data_i16[2]/1000.0f;	//z

}

<<<<<<< HEAD
=======
/* Check if drone is upside by polling accelerometer data
 *
 * Input  -> ptr to processed accel data array
 * Output -> 1 if upside down if lesser than UPSIDE_DOWN_ACCEL_MS2, else 0
 */
static int check_upside_down(float accel_data[3])
{
	return (accel_data[2] < UPSIDE_DOWN_ACCEL_MS2);
}

>>>>>>> current

/* UART Functions--------------------------------------------------------*/

/* Print UART data, max msg length is 200 characters
 *
 * Input  -> string in the form "message to be sent" or a char array of
 * 			 max size 200
 * 
 * Output -> None, but prints data to UART
 */
void print_UART(char msg[200])
{
#ifdef DEBUG_MODE
	printf(msg);
#else
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg),0xFFFF);
#endif
}

/* Print telemtary data based on the mode, and whether a certain threshold has been met
 *
 * Input -> drone_mode, 
 * 			[FLOAT] accel_data[3], gyro_data[3],mag_data[3],
 * 			[FLOAT] pres, humid, temp
 * 
 * Output -> None, but prints data to UART
 */
static void print_telemmentry(drone_mode mode,
					float accel_data[3], float gyro_data[3],float mag_data[3],
					float pres_data, float humi_data, float temp_data)
{
	char msg[200];
	
	if(mode == RESCUE){
		sprintf(msg,"Drone was attacked! \r\n");		
	}
	else if(accel_data[0] > 5.00){
		sprintf(msg, "Ax:%.2f (m/s^2), exceeds threshold of 5.00 (m/s^2). \r\n",accel_data[0]);
	}
	//Check do we need to show all axis? Or isit just one axis for each imu sensor?
	else if(mode == STANDBY){
		sprintf(msg,
				"Gz:%.2f (deg/s), Mz:%.2f (gauss), P:%.2f (hPa), H:%.2f (%%) \r\n",
				gyro_data[2], mag_data[2], pres_data, humi_data);
	}
	else if(mode == BATTLE){
		//Note that %% does not print on printf, not too sure why
		sprintf(msg,
				"T:%.2f (Celcius), Az:%.2f (m/s^2), Gz:%.2f (deg/s), Mz:%.2f (gauss), P:%.2f (hPa), H:%.2f (%%) \r\n",
				temp_data, accel_data[2],gyro_data[2], mag_data[2], pres_data, humi_data);
	}

	
	print_UART(msg);

}

/* Configuration (STM32 UART Output via USB cable)
 * - Config STM32 UART output
 */
static void UART1_Init(void)
{

    /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configuring UART1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
      while(1);
    }

}


/* Display Functions --------------------------------------------------------*/

/* Toggles User LEDs at a certain freq based on mode
 * 
 * Input  -> drone_mode mode
 * 	- STANDBY: always on
 *  - BATTLE: Freq = 1hz
 *  - RESCUE: Freq = 2hz
 * 
 * Output -> NIL
 */
static void blink_leds(drone_mode mode){
	int freq;
	static uint32_t last_time_blinked;

	if(mode == STANDBY){
		BSP_LED_On(LED2);
		return;
	}
	else if(mode == BATTLE){
		freq = 1;
	}
	else if(mode == RESCUE){
		freq = 2;
	}
	
	int period = 500/freq;

	uint32_t time_now = HAL_GetTick();

	if(time_now - last_time_blinked > period){
		BSP_LED_Toggle(LED2);
		last_time_blinked = time_now;
	}
}

<<<<<<< HEAD
/* Print HAL Error Message and stop prog
 *
 * Input  -> HAL_StatusTypeDef err (int)
 *
 * Output -> NIL
 * */
static void handle_HAL_error(int err)
{
	char error_type[100];
	if(err == 0x00){
		//sprintf(error_type,"HAL_OK \r\n");
		return;
	}
	else if(err == 0x01){
		sprintf(error_type,"HAL_ERROR \r\n");
	}
	else if(err == 0x02){
		sprintf(error_type,"HAL_BUSY \r\n");
	}
	else if(err == 0x03){
		sprintf(error_type,"HAL_TIMEOUT \r\n");
	}

	print_UART("Error: ");
	print_UART(error_type);
	print_UART("Blocking program");

	while(1);
}

=======
>>>>>>> current

/* Button Functions --------------------------------------------------------*/

/* Returns the number of button presses using systick & EXTI Interrupt.
 *
 * This function should be called via POLLING - once every *main* loop
 * After MULTI_PRESS_DETECT_MS, the number of button presses is returned and it is reset to zero.
 * Hence, the next poll will indicate that 0 presses are detected
 * 
 * Input  -> NIL
 * 
 * Output -> Number of button presses
 */
static int check_button_press(){
	static uint32_t last_button_change = 0;
	static int last_button_presses = 0;

	uint32_t time_now = HAL_GetTick();
	if(last_button_presses != exti_button_presses){
		//Remember previous PB state and time activated
		last_button_change = time_now;
		last_button_presses = exti_button_presses;
	}

	if( (time_now - last_button_change > MULTI_PRESS_DETECT_MS) && (exti_button_presses > 0) ){
		//No activation after a while -> number of button presses can be confirmed
		int ret_val = exti_button_presses;
		exti_button_presses = 0;

		return ret_val;
	}

	return 0;
}

/* Interrupt function which is automatically called when interrupt detected on any GPIO Pin.
 *
 * For the push button, 50ms debouncing is used before the global variable button_presses is 
 * updated
 * LSM interrupt is activated on 35 deg tilt
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint32_t last_button_press;

	if(GPIO_Pin == BUTTON_EXTI13_Pin){
		uint32_t time_now = HAL_GetTick();
		uint32_t time_diff = time_now - last_button_press;

		if(time_diff >DEBOUNCE_TIME_MS){
			//button_press is the only thing I want to change
			exti_button_presses++;
		}

		last_button_press = time_now;
	}

	else if(GPIO_Pin == LSM6DSL_INT1_EXTI11_Pin){
<<<<<<< HEAD
		imu_data_ready = 1;
	}
}

/* Configuration (Blue User PB)
 * - Initialise the user push button
 */
static void init_push_button()
{
	//Port C, Pin 13
	GPIO_InitTypeDef button_config;

	USER_BUTTON_GPIO_CLK_ENABLE();

	button_config.Pin = GPIO_PIN_13;
	button_config.Mode = GPIO_MODE_INPUT;
	button_config.Pull = GPIO_PULLUP;
	button_config.Speed = GPIO_SPEED_FREQ_HIGH;

	HAL_GPIO_Init(GPIOC,&button_config);

}

static void init_LSM_exti(void)
{
	GPIO_InitTypeDef button_config;

	__HAL_RCC_GPIOD_CLK_ENABLE(); 	// Enable for GPIOD

	button_config.Pin = LSM6DSL_INT1_EXTI11_Pin;
	button_config.Mode = GPIO_MODE_INPUT;
	button_config.Pull = GPIO_NOPULL;
	button_config.Speed = GPIO_SPEED_FREQ_HIGH;

	HAL_GPIO_Init(LSM6DSL_INT1_EXTI11_GPIO_Port,&button_config);

}


/* Configuration (PB Interrupt, and LSM Interrupt)
 * - Config user push button for falling interrupt
 * - Config LSM interrupt pin for rising interrupt (PD11, GPIO_EXTL11)
 */
static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOC_CLK_ENABLE();	// Enable AHB2 Bus for GPIOC
	__HAL_RCC_GPIOD_CLK_ENABLE(); 	// Enable for GPIOD

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// Enable NVIC EXTI line 13 External Line[15:10] Interrupts (both PB and IMU)
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0x0F, 0x00);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	GPIO_InitStruct.Pin = LSM6DSL_INT1_EXTI11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

}
=======
		sound_buzzer = 1;
	}
}

>>>>>>> current

/* OLED Drawing Functions ---------------------------------------------------*/

/* Flip the values in parameters a and b, only for the draw_tilt func
 *
 * Input -> ptr to int a (e.g &a, where a is int)
 * 		 -> ptr to int b
 *
 */
static void flip(int *a, int*b)
{
	int c = *a;
	*a = *b;
	*b = c;
}


/* Draw line on screen given an angle, where the line goes through the centre of the screen
 * and is at angle deg from the horizontal. Note that ssd_UpdateScreen must be called
 *
 * Input  -> angle [float]
 *
 * Output -> NIL
 */
static void draw_tilt(float angle_rad)
{
	//Note that top left corner is (0,0);
	static int x_end, y_end, x_start, y_start;
	ssd1306_Line(x_start,y_start, x_end, y_end, Black); //Clear prev line


	//char msg[200];
	//sprintf(msg,"Angle: %.2f \r\n",angle_rad*180/PI);
	//print_UART(msg);

	//Set a limit on the angle returned
	if(angle_rad > 0.5 * PI) angle_rad = 0.5 * PI;
	else if(angle_rad < -0.5 * PI) angle_rad = -0.5 * PI;

	//Use a a filter to ensure non-rapid changes
	//angle_rad = filter_angles(angle_rad);

	float tan_theta = tan(angle_rad);
	int theta_neg = (tan_theta < 0);
	if(theta_neg){
		tan_theta = tan_theta * -1.0;
	}

	if(64*tan_theta > 32){
		//Use y axis as ref. y_start = 63, y_end = 0

		x_end = 64 + 32/tan_theta;
		x_start = 64 - 32/tan_theta;

		y_end = 0;
		y_start = 63;

	}
	else{
		//Use x axis as ref
		x_end = 127;
		x_start = 0;

		y_end = 32 - 64*tan_theta;
		y_start = 32 + 64*tan_theta;

	}

	if(theta_neg){
		flip(&x_end,&x_start); //Flip the line draw if theta < 0;
	}

	//Middle is 64 - 1, 32 - 1

	//Crosshairs
	ssd1306_Line(55,32,73,32,White);
	ssd1306_Line(63,32,63,25,White);

	//Tilt Line
	ssd1306_Line(x_start,y_start, x_end, y_end, White);

	ssd1306_UpdateScreen();
}

/** Return tilt angle in rad based on processed accel and gyro data
 *	Input -> ptr to float accel_data
 *  Input -> ptr to float gyro data
 */
static float calculate_tilt_angle(float *accel_data, float *gyro_data)
{	
	//Gyro data in dps, Accel data in ms^-2

	//Angular rotation about x axis
	static float gyro_theta_sum = 0;
	gyro_theta_sum = gyro_theta_sum +  gyro_data[0] * PI/180 * SCREEN_UPDATE_PERIOD_MS * 0.001;

	float z_accel = accel_data[2];
	if(z_accel> 10.10){
		z_accel = 10.10;
	}
	else if(z_accel < -10.10){
		z_accel = -10.10;
	}
	float accel_theta =  acosf(z_accel/10.10); //Measured accel is around 10.11 when vertically upright
	if(accel_data[0] < 0){
		accel_theta = accel_theta * -1.0;
	}

	return 0.80 * gyro_theta_sum + 0.20*accel_theta;
	//return accel_theta;
}

void write_to_reg(uint8_t addr, uint8_t reg, uint8_t bitmask)
{
	uint8_t ctrl;

	ctrl = SENSOR_IO_Read(addr, reg);
	ctrl |= bitmask;

	SENSOR_IO_Write(addr, reg, ctrl);
}

/** Enable LSM tilt interrupt by configuring interrupt and the pin it is connected to
*/
static void init_LSM_tilt_int(void)
{	
	HAL_NVIC_SetPriority(LSM6DSL_INT1_EXTI11_EXTI_IRQn, 0x0F , 0x00);
	HAL_NVIC_ClearPendingIRQ(LSM6DSL_INT1_EXTI11_EXTI_IRQn);
	HAL_NVIC_EnableIRQ(LSM6DSL_INT1_EXTI11_EXTI_IRQn);

	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef LSM_int_pin_config;
	LSM_int_pin_config.Pin = LSM6DSL_INT1_EXTI11_Pin;
	LSM_int_pin_config.Mode = GPIO_MODE_IT_RISING;
	LSM_int_pin_config.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(LSM6DSL_INT1_EXTI11_GPIO_Port, &LSM_int_pin_config);

	//enable tilt function and interrupt
	write_to_reg(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL10_C, 0x0C); //Enable Tilt Detect
	write_to_reg(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG,  0x02); //Tilt detector interrupt driven to INT1
}

/* Initialise the Buzzer on PA4 */
static void init_buzzer_pin(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef buzzer_config;
	buzzer_config.Pin = GPIO_PIN_4;
	buzzer_config.Mode  = GPIO_MODE_OUTPUT_PP;
	buzzer_config.Pull  = GPIO_NOPULL;
	buzzer_config.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(GPIOA,&buzzer_config);
}
