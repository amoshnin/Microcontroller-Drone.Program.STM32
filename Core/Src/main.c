/******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * (c) EE2028 Teaching Team
 ******************************************************************************/
#define historySize 10
#define numberOfSensors 6

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
#include "stdbool.h"
#include "stddef.h"

#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h" // Importing library for (Accelerometer)
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h" // Importing library for (Gyroscope)
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h" // Importing library for (Humidity Sensor)
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h" // Importing library for (Magnetometer)
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h" // Importing library for (Pressure Sensor)
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h" // Importing library for (Temperature Sensor)

// Function for (Printing)
static void printer(char message[2622]);
static void printCrazyMessage(const char* sensorName, float value, float min, float max);
void drawGraphs(int dataArrays[][historySize], float max_values[], int numGraphs, char* titles[]);

// Functions for (Modification)
static void incrementBattery(int *batteryLevel);
static void decrementBattery(int *batteryLevel);
static float roundToFourDecimalPlaces(float num);
int min(int a, int b);
int max(int a, int b);

// Functions for (Initialising Systems)
static void initialiseLED(void);
static void initialisePins(void);
static void initialiseSysTick(void);
static void initialiseButton(void);
static void initialiseSensors(void);
static void initialiseUART(void);

// Functions for (Reading Sensors)
static void readGyroscope(float *x, float *y,
		float *z); // for both (STANDBY and BATTLE) modes
static void readMagnetometer(float *x, float *y,
		float *z); // for both (STANDBY and BATTLE) modes
static void
readPressureSensor(float *pressure); // for both (STANDBY and BATTLE) modes
static void
readHumiditySensor(float *humidity); // for both (STANDBY and BATTLE) modes
static void readAccelerometer(float *x, float *y,
		float *z);               // for (BATTLE) mode only
static void readTemperatureSensor(float *temperature); // for (BATTLE) mode only

// Declaring Global Variables
UART_HandleTypeDef huart;

int batteryLevel = 0;
bool isStandby = true;
bool isDanger = false;
bool gotAttacked = false;

int healthLevel = 10;

int rescueStartTime = NULL;
int bpTime1 = NULL;
int bpTime2 = NULL;

HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == BUTTON_EXTI13_Pin) {
		if (bpTime1 == NULL) {
			bpTime1 = HAL_GetTick();
		} else if (bpTime2 == NULL) {
			bpTime2 = HAL_GetTick();
		}
	}
}

// Declaration of Threshold Variables
// - Temperature Thresholds
float minTemperature = 20;
float maxTemperature = 40;

// - Pressure Thresholds
float minPressure = 0.994000;
float maxPressure = 1;

// - Humidity Thresholds
float minHumidity = 70;
float maxHumidity = 101;

// - Accelerometer Thresholds
float minAccelerometer = -1;
float maxAccelerometer = 30;

// - Gyroscope Thresholds
float minGyroscope = -1;
float maxGyroscope = 200;

// - Magnetometer Thresholds
float minMagnetometer = 1000;
float maxMagnetometer = 2200;

int main(void) {
	// Initialisation of (Systems)
	initialisePins();
	initialiseLED();
	initialiseButton();
	initialiseSysTick();
	initialiseSensors();
	initialiseUART();

	// Configuration of SysTick
	int counter = 0;

	uint32_t t1 = HAL_GetTick();
	uint32_t t2 = HAL_GetTick();

	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	printer("Drone Status: ACTIVE");
	printer("Entering STANDBY MODE\r\n");

	int accelerometerHistory [historySize] = {};
	int magnetometerHistory [historySize] = {};
	int gyroscopeHistory [historySize] = {};
	int humidityHistory [historySize] = {};
	int pressureHistory [historySize] = {};
	int temperatureHistory [historySize] = {};

	while (1) {
		t2 = HAL_GetTick();
		if ((t2 - t1) < 250) { continue; }
		t1 = HAL_GetTick();
		counter++;

		if (isDanger && (healthLevel == 0)) {
			printer("Structural Integrity have been Compromised! Goodbye World!\r\n");
			printer("Drone Status: INACTIVE");
			break;
		}

		if (bpTime1 != NULL && bpTime2 != NULL) {
			if (bpTime2 - bpTime1 <= 500) {
				if (isDanger) {
					isDanger = false;
					rescueStartTime = NULL;
					gotAttacked = false;
					healthLevel = max(healthLevel, 5);
					printer("Entering BATTLE MODE\r\n");
				} else {
					if (isStandby) {
						isStandby = false;
						printer("Entering BATTLE MODE\r\n");
					} else {
						isStandby = true;
						printer("Entering STANDBY MODE\r\n");
					}
				}

				bpTime1 = NULL;
				bpTime2 = NULL;
			}
		} else if (bpTime1 != NULL && (t2 - bpTime1 > 500) && bpTime2 == NULL) {
			bpTime1 = NULL;
			if (!isStandby && !isDanger) {
				printer("Increment Battery\r\n");
				incrementBattery(&batteryLevel);

				char msg[20];
				sprintf(msg, "Battery Level: %d\r\n", batteryLevel);
				printer(msg);

				if (batteryLevel >= 5) {
					printer("Laser Gun Fired!\r\n");
					decrementBattery(&batteryLevel);
				}
			}
			if (isStandby) {
				if (healthLevel < 10) {
					int newHealthLevel = healthLevel + 1;

					char increaseHealthText[70];
					sprintf(increaseHealthText, "Drone Structural Integrity Level (Health Level) Increased to %d\r\n", newHealthLevel);
					printer(increaseHealthText);

					healthLevel = newHealthLevel;
				}
			}
		}

		if (isStandby) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		} else {
			if (isDanger && counter % 1 == 0) {
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			}
			if (!isDanger && counter % 2 == 0) {
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			}
		}

		float g_x;
		float g_y;
		float g_z;
		readGyroscope(&g_x, &g_y, &g_z);
		float gyroscope = sqrt(pow(g_x, 2) + pow(g_y, 2) + pow(g_z, 2));
		bool isGyroscopeCrazy = (gyroscope < minGyroscope) || (gyroscope > maxGyroscope);

		float m_x;
		float m_y;
		float m_z;
		readMagnetometer(&m_x, &m_y, &m_z);
		float magnetometer = sqrt(pow(m_z, 2) + pow(m_y, 2) + pow(m_z, 2));
		bool isMagnetometerCrazy = (magnetometer < minMagnetometer || magnetometer > maxMagnetometer);

		float a_x;
		float a_y;
		float a_z;
		readAccelerometer(&a_x, &a_y, &a_z);
		float accelerometer = sqrt(pow(a_x, 2) + pow(a_y, 2) + pow(a_z, 2));
		bool isAccelerometerCrazy = (accelerometer < minAccelerometer) || (accelerometer > maxAccelerometer);

		float pressure;
		readPressureSensor(&pressure);
		bool isPressureCrazy = (pressure < minPressure || pressure > maxPressure);

		float humidity;
		readHumiditySensor(&humidity);
		bool isHumidityCrazy = (humidity < minHumidity || humidity > maxHumidity);

		float temperature;
		readTemperatureSensor(&temperature);
		bool isTemperatureCrazy = (temperature < minTemperature || temperature > maxTemperature);

		bool isMonitoring = isGyroscopeCrazy | isMagnetometerCrazy | isPressureCrazy | isHumidityCrazy | isTemperatureCrazy | isAccelerometerCrazy;

		if (counter % 4 == 0) {

			for (int i = 0; i < historySize - 1; i++) {
				accelerometerHistory[i] = accelerometerHistory[i + 1];
			}
			accelerometerHistory[historySize - 1] = (int)accelerometer;

			for (int i = 0; i < historySize - 1; i++) {
				magnetometerHistory[i] = magnetometerHistory[i + 1];
			}
			magnetometerHistory[historySize - 1] = (int)magnetometer;

			for (int i = 0; i < historySize - 1; i++) {
				gyroscopeHistory[i] = gyroscopeHistory[i + 1];
			}
			gyroscopeHistory[historySize - 1] = (int)gyroscope;

			for (int i = 0; i < historySize - 1; i++) {
				humidityHistory[i] = humidityHistory[i + 1];
			}
			humidityHistory[historySize - 1] = (int)humidity;

			for (int i = 0; i < historySize - 1; i++) {
				pressureHistory[i] = pressureHistory[i + 1];
			}
			pressureHistory[historySize - 1] = (int)(pressure * (101.3 * 1000) / 100);

			for (int i = 0; i < historySize - 1; i++) {
				temperatureHistory[i] = temperatureHistory[i + 1];
			}
			temperatureHistory[historySize - 1] = (int)temperature;

			char* titles[] = {"Accelerometer", "Magnetometer", "Gyroscope", "Humidity", "Pressure", "Temperature"};
			float max_values[] = {maxAccelerometer, maxMagnetometer, maxGyroscope, maxHumidity, maxPressure * (101.3 * 1000) / 100, maxTemperature};

			int dataArrays[6][historySize];
			memcpy(dataArrays[0], accelerometerHistory, sizeof(accelerometerHistory));
			memcpy(dataArrays[1], magnetometerHistory, sizeof(magnetometerHistory));
			memcpy(dataArrays[2], gyroscopeHistory, sizeof(gyroscopeHistory));
			memcpy(dataArrays[3], humidityHistory, sizeof(humidityHistory));
			memcpy(dataArrays[4], pressureHistory, sizeof(pressureHistory));
			memcpy(dataArrays[5], temperatureHistory, sizeof(temperatureHistory));

			// Time Calculation
			int milliseconds = HAL_GetTick();

			int millisecondsPerDay = 24 * 60 * 60 * 1000;
			int millisecondsPerHour = 60 * 60 * 1000;
			int millisecondsPerMinute = 60 * 1000;
			int millisecondsPerSecond = 1000;

			int days = milliseconds / millisecondsPerDay;
			milliseconds %= millisecondsPerDay;
			int hours = milliseconds / millisecondsPerHour;
			milliseconds %= millisecondsPerHour;
			int minutes = milliseconds / millisecondsPerMinute;
			milliseconds %= millisecondsPerMinute;
			int seconds = milliseconds / millisecondsPerSecond;
			milliseconds %= millisecondsPerSecond;

			if (isDanger) {
				healthLevel = max(0, healthLevel - 1);
				char attackText[70];
				sprintf(attackText, "Drone Was Attacked! Structural Integrity Level (Health Level): %d\r\n", healthLevel);
				printer(attackText);
			} else {
				int msg_size = 60;
				printer("\r\n\n");
				if (isMonitoring) {
					printer(">>>>>> >>>>>> >>>>>> >>>>>> >>>>>> >>>>>> >>>>>> >>>>>> >>>>>> DRONE STATISTICS for COMMAND CENTER (WARNING STATE) <<<<<< <<<<<< <<<<<< <<<<<< <<<<<< <<<<<< <<<<<< <<<<<< <<<<<<\r\n\n");

					if (isGyroscopeCrazy) {
						printCrazyMessage("G", gyroscope, minGyroscope, maxGyroscope);
					}
					if (isMagnetometerCrazy) {
						printCrazyMessage("M", magnetometer, minMagnetometer, maxMagnetometer);
					}
					if (isPressureCrazy) {
						printCrazyMessage("P", pressure, minPressure, maxPressure);
					}
					if (isHumidityCrazy) {
						printCrazyMessage("H", humidity, minHumidity, maxHumidity);
					}

					if (!isStandby) {
						if (isTemperatureCrazy) {
							printCrazyMessage("T", temperature, minTemperature, maxTemperature);
						}
						if (isAccelerometerCrazy) {
							printCrazyMessage("A", accelerometer, minAccelerometer, maxAccelerometer);
						}
					}

					drawGraphs(dataArrays, max_values, numberOfSensors, titles);
					printer("\r\n\n");
					printer(">>>>>> >>>>>> >>>>>> >>>>>> >>>>>> >>>>>> >>>>>> >>>>>> >>>>>> >>>>>> >>>>>> >>>>>> >>>>>> <<<<<< <<<<<< <<<<<< <<<<<< <<<<<< <<<<<< <<<<<< <<<<<< <<<<<< <<<<<< <<<<<< <<<<<< <<<<<<\r\n");
				} else {
					char message[2622];
					if (isStandby) {
						sprintf(message, "G:(x=%.4f, y=%.4f, z=%.4f, total=%.4f) (Degrees Per Second), M:(x=%.4f, y=%.4f, z=%.4f, total=%.4f) (milli gauus), P:(%.4f) (atm), H:(%.4f)(Relative Humidity) \r\n",
								g_x, g_y, g_z, gyroscope, m_x, m_y, m_z, magnetometer, pressure, humidity);
					}
					if (!isStandby && !isDanger) {
						sprintf(message, "G:(x=%.4f, y=%.4f, z=%.4f, total=%.4f) (Degrees Per Second), M:(x=%.4f, y=%.4f, z=%.4f, total=%.4f) (milli gauus), P:(%.4f) (atm), H:(%.4f) (Relative Humidity), T:(%.4f) (Degrees Celsius), A:(x=%.4f, y=%.4f, z=%.4f, total=%.4f) (m/s^2) \r\n",
								g_x, g_y, g_z, gyroscope, m_x, m_y, m_z, magnetometer, pressure, humidity, temperature, a_x, a_y, a_z, accelerometer);
					}
					printer("······ ······ ······ ······ ······ ······ ······ ······ ······ DRONE STATISTICS for COMMAND CENTER (HEALTHY STATE) ······ ······ ······ ······ ······ ······ ······ ······ ······\r\n\n");
					char flightTime[270];
					sprintf(flightTime, "Flight Time:");
					if (days > 0) {
						sprintf(flightTime, "%s %d Days |", flightTime, days);
					}
					if (hours > 0) {
						sprintf(flightTime, "%s %d Hours |", flightTime, hours);
					}
					if (minutes > 0) {
						sprintf(flightTime, "%s %d Minutes |", flightTime, minutes);
					}
					sprintf(flightTime, "%s %d Seconds | %d Milliseconds\r\n", flightTime, seconds, milliseconds);
					printer(flightTime);

					char healthInfo[50];
					sprintf(healthInfo, "Structural Integrity Level (Health Level); %d\r\n", healthLevel);
					printer(healthInfo);

					printer(message);
					drawGraphs(dataArrays, max_values, numberOfSensors, titles);
					printer("\r\n\n");
					printer("······ ······ ······ ······ ······ ······ ······ ······ ······ ······ ······ ······ ······ ······ ······ ······ ······ ······ ······ ······ ······ ······ ······ ······ ······\r\n");
				}
			}
		}

		if (!isStandby && !gotAttacked) {
			if (a_z < -8) { gotAttacked = true; }
			if (gotAttacked) {
				isDanger = true;
				rescueStartTime = HAL_GetTick();
				printer("!!!!!! !!!!!! !!!!!! !!!!!! !!!!!! !!!!!! !!!!!! !!!!!! !!!!!! DRONE WARNING for COMMAND CENTER (DANGER STATE) !!!!!! !!!!!! !!!!!! !!!!!! !!!!!! !!!!!! !!!!!! !!!!!! !!!!!!\r\n\n");
			}
		}
	}

	BSP_GYRO_DeInit();     // DeInitialise (Gyroscope)
	BSP_MAGNETO_DeInit();  // DeInitialise (Magnetometer)
	BSP_ACCELERO_DeInit(); // DeInitialise (Accelerometer)
	HAL_UART_DeInit(&huart);  // DeInitialise (UART)
}

// Function for (Printing)
static void printer(char message[2622]) {
	HAL_UART_Transmit(&huart, (uint8_t*)message, strlen(message),0xFFFF);
};

static void printCrazyMessage(const char* sensorName, float value, float min, float max) {
	char text[60];
	sprintf(text, "%s: %f exceeds the limits of %f - %f\r\n", sensorName, value, min, max);
	printer(text);
}

void drawGraphs(int dataArrays[][historySize], float max_values[], int numGraphs, char* titles[]) {
	int max = 13;
	int graphSize = 29;

	printer("\n");
	for (int row = max; row > 0; row--) {
		// Display y-values on the left for each row
		for (int graphIndex = 0; graphIndex < numGraphs; graphIndex++) {
			char yValue[20];
			sprintf(yValue, "%3d%%| ", row * 10);
			printer(yValue);

			for (int col = 0; col < historySize; col++) {
				int newData = (int)((dataArrays[graphIndex][col] / max_values[graphIndex]) * 10.0);

				if (newData >= row) {
					printer("| ");
				} else {
					printer("  ");
				}
			}

			// Separate graphs with some space
			printer("   ");
		}
		printer("\r\n");
	}

	// X-axis (time) for each graph
	printer("     ");
	for (int graphIndex = 0; graphIndex < numGraphs; graphIndex++) {
		for (int i = 0; i < historySize; i++) {
			printer("--");
		}
		printer("         ");  // Separate graphs with some space
	}
	printer("\r\n      ");
	for (int graphIndex = 0; graphIndex < numGraphs; graphIndex++) {
		for (int i = 0; i < historySize; i++) {
			char mai[20];
			sprintf(mai, "%d ", i + 1);
			printer(mai);
		}

		// Separate graphs with some space
		printer("        ");
	}
	printer("\r\n\n");

	for (int graphIndex = 0; graphIndex < numGraphs; graphIndex++) {
		for (int i = 0; i < (int)((graphSize - strlen(titles[graphIndex])) / 2); i++) {
			printer(" ");
		}
		printer(titles[graphIndex]);
		for (int i = 0; i < (int)((graphSize - strlen(titles[graphIndex])) / 2); i++) {
			printer(" ");
		}
	}
}

// Functions for (Modification)
static void incrementBattery(int *batteryLevel) {
	*batteryLevel = min(*batteryLevel + 3, 10);
}

static void decrementBattery(int *batteryLevel) {
	*batteryLevel -= 5;
}

float roundToFourDecimalPlaces(float num) {
	float rounded = roundf(num * 10000.0) / 10000.0;
	return rounded;
}

int max(int a, int b) {
	return (a > b) ? a : b;
}

int min(int a, int b) {
	return (a < b) ? a : b;
}

// Functions for (Initialising Systems)
static void initialiseLED(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE(); /* GPIO Ports Clock Enable */
	HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET); /*Configure GPIO pin Output Level */
	/*Configure GPIO pin LED2_Pin */
	GPIO_InitStruct.Pin = LED2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void initialiseSysTick(void) {
	HAL_Init();
}

static void initialisePins(void) {
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable AHB2 Bus for GPIOC
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	// Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	// Enable NVIC EXTI line 13
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void initialiseUART(void) {
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
	huart.Instance = USART1;
	huart.Init.BaudRate = 115200;
	huart.Init.WordLength = UART_WORDLENGTH_8B;
	huart.Init.StopBits = UART_STOPBITS_1;
	huart.Init.Parity = UART_PARITY_NONE;
	huart.Init.Mode = UART_MODE_TX_RX;
	huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart.Init.OverSampling = UART_OVERSAMPLING_16;
	huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart) != HAL_OK)
	{
		while(1);
	}
}

static void initialiseButton(void) {
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
}

static void initialiseSensors(void) {
	BSP_GYRO_Init();     // Initialise (Gyroscope) (Units: )
	BSP_MAGNETO_Init();  // Initialise (Magnetometer) (Units: )
	BSP_PSENSOR_Init();  // Initialise (Pressure Sensor) (Units: Atmosphere (atm))
	BSP_HSENSOR_Init();  // Initialise (Humidity Sensor) (Units: Relative Humidity (RL))
	BSP_ACCELERO_Init(); // Initialise (Accelerometer) (Units: )
	BSP_TSENSOR_Init();  // Initialise (Temperature Sensor) (Units: Degrees Celsius (Cº))
}

// Functions for (Reading Sensors)
static void readGyroscope(float *x, float *y, float *z) {
	float constant = pow(10, -3);
	float data[3];
	BSP_GYRO_GetXYZ(data);
	*x = roundToFourDecimalPlaces(data[0] * constant);
	*y = roundToFourDecimalPlaces(data[1] * constant);
	*z = roundToFourDecimalPlaces(data[2] * constant);
}

static void readMagnetometer(float *x, float *y, float *z) {
	int16_t data[3] = { 0 };
	BSP_MAGNETO_GetXYZ(data);
	*x = roundToFourDecimalPlaces((float)data[0]);
	*y = roundToFourDecimalPlaces((float)data[1]);
	*z = roundToFourDecimalPlaces((float)data[2]);
}

static void readPressureSensor(float *pressure) {
	*pressure = roundToFourDecimalPlaces((BSP_PSENSOR_ReadPressure() * 100) / (101.3 * 1000));
}

static void readHumiditySensor(float *humidity) {
	*humidity = roundToFourDecimalPlaces(BSP_HSENSOR_ReadHumidity());
}

static void readAccelerometer(float *x, float *y, float *z) {
	float constant = (9.8/1000.0f);
	int16_t data[3] = { 0 };			// array to store the x, y and z readings.
	BSP_ACCELERO_AccGetXYZ(data);		// read accelerometer
	// the function above returns 16 bit integers which are acceleration in mg (9.8/1000 m/s^2).
	// Converting to float to print the actual acceleration.
	*x = roundToFourDecimalPlaces((float)data[0] * constant);
	*y = roundToFourDecimalPlaces((float)data[1] * constant);
	*z = roundToFourDecimalPlaces((float)data[2] * constant);
}

static void readTemperatureSensor(float *temperature) {
	*temperature = roundToFourDecimalPlaces(BSP_TSENSOR_ReadTemp());
}
