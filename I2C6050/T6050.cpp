#include "I2CIO.h"
#include "Serial.h"

#define  DIV 19

uint8_t samplingRateMs = (DIV + 1) / 2;
  // 1KHz clock divider for sample rate (99+1) -> 10Hz for 5Hz bw
  // should be sampling rate -1 

  // low pass filter: 1:184Hz, 2:94Hz, 3:44Hz 4: 20Hz, 5: 10Hz, 6: 5Hz
#define LPF 4



#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_PWR_MGMT_2         0x6C   // R/W
#define MPU6050_FIFO_COUNTH        0x72   // R/W
#define MPU6050_FIFO_COUNTL        0x73   // R/W
#define MPU6050_FIFO_R_W           0x74   // R/W
#define MPU6050_WHO_AM_I           0x75   // R
#define MPU6050_RA_ACCEL_XOUT_H    0x3B
#define MPU6050_RA_ACCEL_YOUT_H    0x3D
#define MPU6050_RA_SIGNAL_PATH_RESET  0x68
#define MPU6050_RA_CONFIG            0x1A
#define MPU6050_RA_SMPLRT_DIV      0x19

#define MPU6050_I2C_ADDRESS 0x68 // depends on pin AD): 0x68 or 0x69

unsigned long timer;


double yPosition = 0.0;
double ySpeed = 0.0;
double yAccel;

uint8_t bFirst = 1;
short totalDelay = 0;

void delayUS_DWT(uint16_t uSec)
{
	volatile uint32_t cycles = (SystemCoreClock / 1000000L)*uSec;
	volatile uint32_t start = DWT->CYCCNT;
	do {
	} while (DWT->CYCCNT - start < cycles);
}


void readId()
{
	uint8_t v=0;
#if 0
// scope sync in case something has to be debugged
		delayUS_DWT(400);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		delayUS_DWT(400);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
#endif
		int status = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)MPU6050_I2C_ADDRESS << 1, 1, 10); 
		Serial.print("ready = ");
	    Serial.print(status,DEC);
		Serial.println(status?"fail":"OK");
	    if (status != 0)
		    return; // failure
		I2c.timeOut(80); // fail fast
		v = 0;
	// request 1 bytes from slave device register WHO_AM_I, returns the I2C address
		I2c.read(MPU6050_I2C_ADDRESS, MPU6050_WHO_AM_I, 1, &v);    
		int sValue;

		Serial.print("Id = ");
		Serial.println(v, HEX);
}



float getYAccel()
{
	uint8_t values[2];
	I2c.read(MPU6050_I2C_ADDRESS, MPU6050_RA_ACCEL_YOUT_H, 2, values);
	short sValue = (values[0] << 8) | values[1];
	return sValue * 9.81 / 16384.0;
}


float accelYOffset = 0.0;

float calibrateYAccel()
{
	for (int i = 0; i < 100; i++) {
		accelYOffset += getYAccel();
		HAL_Delay(100);
	}
	accelYOffset /= 100.0;
}

void init_6050()
{
	int error = 0;
	readId();
  
	// According to the datasheet, the 'sleep' bit
	// should read a '1'.
	// That bit has to be cleared, since the sensor
	// is in sleep mode at power-up. 
	uint8_t value;
	I2c.read(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 1, &value);
	Serial.print(F("PWR_MGMT_1 : "));
	Serial.println(value, HEX);
	value = 0;
	I2c.write(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, &value, 1);
	I2c.read(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 1, &value);
	Serial.print(F("PWR_MGMT_1 : "));
	Serial.println(value, HEX);
	value = 7;
	I2c.write(MPU6050_I2C_ADDRESS, MPU6050_RA_SIGNAL_PATH_RESET, &value, 1);
	// low pass filter: 2:94Hz, 3:44Hz 4: 20Hz, 5: 10Hz, 6: 5Hz
	value = LPF;
	I2c.write(MPU6050_I2C_ADDRESS, MPU6050_RA_CONFIG, &value, 1);
	// 1KHz clock divider for sample rate (99+1) -> 10Hz for 5Hz bw
	value = DIV;
	I2c.write(MPU6050_I2C_ADDRESS, MPU6050_RA_SMPLRT_DIV, &value, 1);
	// setup FIFO
  
  
	HAL_Delay(100);
	calibrateYAccel();
	if (error == 0)
	{
		Serial.print("MPU6050 Init Successful"); 
	}
}

void setup(void) {
  //telemetry on Serial
	Serial.begin(115200);
	I2c.begin(&hi2c1);
	I2c.timeOut(500);
	I2c.pullup(true);
	I2c.setSpeed(0);//slow
	init_6050();
	HAL_Delay(100);
	Serial.println("Starting...");
}



void acquireY()
{
	yAccel = getYAccel() - accelYOffset;
	ySpeed = ySpeed + yAccel * samplingRateMs / 1000;
	yPosition = yPosition + ySpeed * samplingRateMs / 1000;
}
 
void displayYPosition()
{
	Serial.print(yAccel);
	Serial.print((char)9);
	Serial.print(ySpeed);
	Serial.print((char)9);
	Serial.println(yPosition);
}


void loop(void) {
	if (bFirst) {
		acquireY();
		displayYPosition();
		HAL_Delay(samplingRateMs);
		while (totalDelay < 50) {
			acquireY();
			displayYPosition();
			HAL_Delay(samplingRateMs);
			totalDelay += samplingRateMs;
		}
		while (totalDelay < 100) {
			acquireY();
			displayYPosition();
			HAL_Delay(samplingRateMs);
			totalDelay += samplingRateMs;
		}
		bFirst = 1;
	}
	HAL_Delay(1000);
}
 

 
 