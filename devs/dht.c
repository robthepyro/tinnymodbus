/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018
 *
 * Balint Cristian <cristian dot balint at gmail dot com>
 * DHT sensor added by ramb0t
 *
 * TinnyModbus
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

/*

  dht.c (DHT temperature and humidity sensor)

*/

#include "dht.h"

#if (DHT_Type == DHT11)
	#define __DHT_Temperature_Min	0
	#define __DHT_Temperature_Max	50
	#define __DHT_Humidity_Min		20
	#define __DHT_Humidity_Max		90
	#define __DHT_Delay_Read		50
#elif (DHT_Type == DHT22)
	#define __DHT_Temperature_Min	-40
	#define __DHT_Temperature_Max	80
	#define __DHT_Humidity_Min		0
	#define __DHT_Humidity_Max		100
	#define __DHT_Delay_Read		20
#endif

//----- Prototypes ----------------------------//
static float ExtractTemperature(uint8_t Data3, uint8_t Data4);
static float ExtractHumidity(uint8_t Data1, uint8_t Data2);
//---------------------------------------------//

//----- Functions -----------------------------//
//Setup sensor.
void DHT_Setup()
{
	_delay_ms(__DHT_Delay_Setup);
	__DHT_STATUS = DHT_Ok;
}

//Get sensor status.
enum DHT_Status_t DHT_GetStatus()
{
	return (__DHT_STATUS);
}

//Read raw buffer from sensor.
enum DHT_Status_t DHT_ReadRaw(uint8_t Data[4])
{
	uint8_t buffer[5] = {0, 0, 0, 0, 0};
	uint8_t retries, i;
	int8_t j;
	__DHT_STATUS = DHT_Ok;
	retries = i = j = 0;

	//----- Step 1 - Start communication -----
	if (__DHT_STATUS == DHT_Ok)
	{
		//Request data
		DigitalWrite(DHT_Pin, Low);			//DHT_PIN = 0
		PinMode(DHT_Pin, Output);			//DHT_PIN = Output
		_delay_ms(__DHT_Delay_Read);

		//Setup DHT_PIN as input with pull-up resistor so as to read data
		DigitalWrite(DHT_Pin, High);		//DHT_PIN = 1 (Pull-up resistor)
		PinMode(DHT_Pin, Input);			//DHT_PIN = Input

		//Wait for response for 20-40us
		retries = 0;
		while (DigitalRead(DHT_Pin))
		{
			_delay_us(2);
			retries += 2;
			if (retries > 60)
			{
				__DHT_STATUS = DHT_Error_Timeout;	//Timeout error
				break;
			}
		}
	}
	//----------------------------------------

	//----- Step 2 - Wait for response -----
	if (__DHT_STATUS == DHT_Ok)
	{
		//Response sequence began
		//Wait for the first response to finish (low for ~80us)
		retries = 0;
		while (!DigitalRead(DHT_Pin))
		{
			_delay_us(2);
			retries += 2;
			if (retries > 100)
			{
				__DHT_STATUS = DHT_Error_Timeout;	//Timeout error
				break;
			}
		}
		//Wait for the last response to finish (high for ~80us)
		retries = 0;
		while(DigitalRead(DHT_Pin))
		{
			_delay_us(2);
			retries += 2;
			if (retries > 100)
			{
				__DHT_STATUS = DHT_Error_Timeout;	//Timeout error
				break;
			}
		}
	}
	//--------------------------------------

	//----- Step 3 - Data transmission -----
	if (__DHT_STATUS == DHT_Ok)
	{
		//Reading 5 bytes, bit by bit
		for (i = 0 ; i < 5 ; i++)
			for (j = 7 ; j >= 0 ; j--)
			{
				//There is always a leading low level of 50 us
				retries = 0;
				while(!DigitalRead(DHT_Pin))
				{
					_delay_us(2);
					retries += 2;
					if (retries > 70)
					{
						__DHT_STATUS = DHT_Error_Timeout;	//Timeout error
						j = -1;								//Break inner for-loop
						i = 5;								//Break outer for-loop
						break;								//Break while loop
					}
				}

				if (__DHT_STATUS == DHT_Ok)
				{
					//We read data bit || 26-28us means '0' || 70us means '1'
					_delay_us(35);							//Wait for more than 28us
					if (DigitalRead(DHT_Pin))				//If HIGH
						BitSet(buffer[i], j);				//bit = '1'

					retries = 0;
					while(DigitalRead(DHT_Pin))
					{
						_delay_us(2);
						retries += 2;
						if (retries > 100)
						{
							__DHT_STATUS = DHT_Error_Timeout;	//Timeout error
							break;
						}
					}
				}
			}
	}
	//--------------------------------------


	//----- Step 4 - Check checksum and return data -----
	if (__DHT_STATUS == DHT_Ok)
	{
		if (((uint8_t)(buffer[0] + buffer[1] + buffer[2] + buffer[3])) != buffer[4])
		{
			__DHT_STATUS = DHT_Error_Checksum;	//Checksum error
		}
		else
		{
			//Build returning array
			//data[0] = Humidity		(int)
			//data[1] = Humidity		(dec)
			//data[2] = Temperature		(int)
			//data[3] = Temperature		(dec)
			//data[4] = Checksum
			for (i = 0 ; i < 4 ; i++)
				Data[i] = buffer[i];
		}
	}
	//---------------------------------------------------

	return DHT_GetStatus();
}

/*
 * read sensor measurements
 */
int32_t dhtReadValue( uint8_t TYPE )
{
    uint32_t V;

    i2c_init();

    i2c_start(SHT21_ADDR<<1);
    i2c_write(TYPE);

    i2c_rep_start((SHT21_ADDR<<1)|0x1);
    uint16_t value = ((uint16_t)i2c_read(0)) << 8;
    value |= i2c_read(1);

    i2c_stop();

    switch ( TYPE )
    {
        case SHT21_TEMP:
          // temperature
          V = ((21965 * (uint32_t)value) >> 13) - 46850;
          break;
        case SHT21_HUMI:
          // humidity
          V = ((15625 * (uint32_t)value) >> 13) - 6000;
          break;
    } // end switch

  return V;
}

//Read temperature and humidity.
float dhtReadValue( uint8_t TYPE )
{
	float f;
	
	uint8_t data[4] = { 0, 0, 0, 0 };

	//Read data
	enum DHT_Status_t status =  DHT_ReadRaw(data);

	//If read successfully
	if (status == DHT_Ok)
	{
		//Calculate values
		*Temperature = ExtractTemperature(data[2], data[3]);
		*Humidity = ExtractHumidity(data[0], data[1]);

		//Check values
		if ((*Temperature < __DHT_Temperature_Min) || (*Temperature > __DHT_Temperature_Max))
			__DHT_STATUS = DHT_Error_Temperature;
		else if ((*Humidity < __DHT_Humidity_Min) || (*Humidity > __DHT_Humidity_Max))
			__DHT_STATUS = DHT_Error_Humidity;
	}

	return DHT_GetStatus();
}



