/* Библиотека для работы с датчиком BMP 180 */

#include "stm32f1xx_hal.h"
#include "math.h"

extern I2C_HandleTypeDef hi2c1;
#define BMP180_i2c &hi2c1 //импортируем сконфигурированную структуру для работы с портом I2c на котором висит датчик;
#define BMP180_address 0x77 //адрес датчика
/************************************************/


/* Создаём переменные и константы согласно DataSheet BMP180 */
#define write_register 0xF4
#define read_register 0xF6

short AC1 = 0;
short AC2 = 0;
short AC3 = 0;
unsigned short AC4 = 0;
unsigned short AC5 = 0;
unsigned short AC6 = 0;
short B1 = 0;
short B2 = 0;
short MB = 0;
short MC = 0;
short MD = 0;
short oss = 1; //выбран стандартный режим (компромисс между потребляемой мощностью и скоростью)
long UT = 0;
long UP = 0;
long X1 = 0;
long X2 = 0;
long B5  = 0;
long B6 = 0;
long B3 = 0;
unsigned long B4 = 0;
long X3 = 0;
unsigned long B7 = 0;

long Temperature = 0;
long Pressure = 0;
/************************************************/

/* Функция чтения калибровочных данных */

void Read_callibration_data (void)
{
	uint8_t callib_data[22] = {0};
	uint8_t start_address = 0xAA;
	HAL_I2C_Mem_Read(BMP180_i2c, (BMP180_address << 1), start_address, 1, callib_data, 22, 22);

	AC1 = ((callib_data[0] << 8) | callib_data[1]);
	AC2 = ((callib_data[2] << 8) | callib_data[3]);
	AC3 = ((callib_data[4] << 8) | callib_data[5]);
	AC4 = ((callib_data[6] << 8) | callib_data[7]);
	AC5 = ((callib_data[8] << 8) | callib_data[9]);
	AC6 = ((callib_data[10] << 8) | callib_data[11]);
	B1 = ((callib_data[12] << 8) | callib_data[13]);
	B2 = ((callib_data[14] << 8) | callib_data[15]);
	MB = ((callib_data[16] << 8) | callib_data[17]);
	MC = ((callib_data[18] << 8) | callib_data[19]);
	MD = ((callib_data[20] << 8) | callib_data[21]);
}

uint16_t Read_uncompensated_Temperature (void)
{
	uint8_t write_data = 0x2E;
	uint8_t MSB_LSB_Data[2] = {0};

	HAL_I2C_Mem_Write(BMP180_i2c, (BMP180_address << 1), write_register, 1, &write_data, 1, 1);
	HAL_Delay(5);
	HAL_I2C_Mem_Read(BMP180_i2c, (BMP180_address << 1), read_register, 1, MSB_LSB_Data, 2, 1);
	return ((MSB_LSB_Data[0] << 8) + MSB_LSB_Data[1]);
}

float Compensated_Temperature (void)
{
	UT = Read_uncompensated_Temperature();
	X1 = ((UT-AC6) * (AC5/(pow(2, 15))));
	X2 = ((MC*(pow(2,11))) / (X1+MD));
	B5 = (X1+X2);
	Temperature = ((B5+8)/(pow(2,4)));
	return Temperature / 10.0;
}
uint32_t Read_uncompensated_Pressure (void)
{
	uint8_t write_data = 0x34 + (oss << 6);
	uint8_t MSB_LSB_XLSB_Data[3] = {0};

	HAL_I2C_Mem_Write(BMP180_i2c, (BMP180_address << 1), write_register, 1, &write_data, 1, 1);
	HAL_Delay(8);
	HAL_I2C_Mem_Read(BMP180_i2c, (BMP180_address << 1), read_register, 1, MSB_LSB_XLSB_Data, 3, 3);
	return (((MSB_LSB_XLSB_Data[0] << 16) + (MSB_LSB_XLSB_Data[1] << 8) + MSB_LSB_XLSB_Data[2]) >> (8 - oss));
}

float Compensated_Pressure (void)
{
	UP = Read_uncompensated_Pressure();
	B6 = B5 - 4000;
	X1 = (B2 * (B6 * B6 / pow(2, 12))) / (pow(2, 11));
	X2 = AC2 * B6 / pow(2, 11);
	X3 = X1 + X2;
	B3 = (((AC1 * 4 + X3) << oss) + 2) / 4;
	X1 = AC3 * B6 / pow(2, 13);
	X2 = (B1 * (B6 * B6 / pow(2, 12))) / (pow(2, 16));
	X3 = ((X1 + X2) + 2) / pow(2, 2);
	B4 = AC4 * (unsigned long)(X3 + 32768) / (pow(2, 15));
	B7 = ((unsigned long)UP - B3) * (50000 >> oss);
	if (B7 < 0x80000000)
	{
		Pressure = (B7 * 2) / B4;
	}
	else
	{
		Pressure = (B7 / B4) * 2;
	}
	X1 = (Pressure / pow(2, 8)) * (Pressure / pow(2, 8));
	X1 = (X1 * 3038) / pow(2, 16);
	X2 = (-7357 * Pressure) / pow(2, 16);
	Pressure = Pressure + (X1 + X2 + 3791) / pow(2, 4);
	return Pressure;
}

void Inicialization (void)
{
	Read_callibration_data();
}
