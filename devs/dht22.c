/* Fast DHT Lirary
 *
 * Copyright (C) 2015 Sergey Denisov.
 * Written by Sergey Denisov aka LittleBuster (DenisovS21@gmail.com)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public Licence
 * as published by the Free Software Foundation; either version 3
 * of the Licence, or (at your option) any later version.
 *
 * Original library written by Adafruit Industries. MIT license.
 */

#include "dht22.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


#define DHT_COUNT 6
#define DHT_MAXTIMINGS 75

#define _maxcycles	255  //255uS timeout? 


void dht_init(struct dht22 *dht, uint8_t pin)
{
    dht->pin = pin;
    /* Setup the pins! */
    DDR_DHT &= ~(1 << dht->pin);
    PORT_DHT |= (1 << dht->pin);
}

static uint8_t dht_read(struct dht22 *dht)
{
    //uint8_t tmp;
    uint8_t sum = 0;
    //uint8_t j = 0, i;
    //uint8_t last_state = 1;
    //uint16_t counter = 0;
	
	uint8_t cycles[80];
    /*
     * Pull the pin 1 and wait 250 milliseconds
     */
    //PORT_DHT |= (1 << dht->pin);
    //_delay_ms(250);

    dht->data[0] = dht->data[1] = dht->data[2] = dht->data[3] = dht->data[4] = 0;

    /* Now pull it low for ~20 milliseconds */
    DDR_DHT |= (1 << dht->pin);  // pin output
    PORT_DHT &= ~(1 << dht->pin); // low
    _delay_ms(5);
    cli();
    PORT_DHT |= (1 << dht->pin); // high
    _delay_us(30);
    DDR_DHT &= ~(1 << dht->pin); // pininput
	
	// First expect a low signal for ~80 microseconds followed by a high signal
    // for ~80 microseconds again.
    if (expectPulse(0, dht->pin) == 0) {
      return 0;
    }
    if (expectPulse(1, dht->pin) == 0) {
      return 0;
    }

    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
    // microsecond low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed all
    // the pulses are read into a array and then examined in a later step.
    for (int i=0; i<80; i+=2) {
      cycles[i]   = expectPulse(0, dht->pin);
      cycles[i+1] = expectPulse(1, dht->pin);
    }

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  for (int i=0; i<40; ++i) {
    uint8_t lowCycles  = cycles[2*i];
    uint8_t highCycles = cycles[2*i+1];
    if ((lowCycles == 0) || (highCycles == 0)) { // sanity check
      return 0;
    }
		dht->data[i/8] <<= 1;
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (highCycles > lowCycles) {
      // High cycles are greater than 50us low cycle count, must be a 1.
		dht->data[i/8] |= 1;
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }


    sei();
    sum = dht->data[0] + dht->data[1] + dht->data[2] + dht->data[3];

    // check dat checksum! 
    if (dht->data[4] == (sum & 0xFF))
        return 1;
    return 0;
}

uint8_t dht_read_temp(struct dht22 *dht, float *temp)
{
    if (dht_read(dht)) {
        *temp = dht->data[2] & 0x7F;
        *temp *= 256;
        *temp += dht->data[3];
        *temp /= 10;

        if (dht->data[2] & 0x80)
            *temp *= -1;
        return 1;
    }
    return 0;
}

uint8_t dht_read_hum(struct dht22 *dht, float *hum)
{
    if (dht_read(dht)) {
        *hum = dht->data[0];
        *hum *= 256;
        *hum += dht->data[1];
        *hum /= 10;
        if (*hum == 0.0f)
            return 0;
        return 1;
    }
    return 0;
}

uint8_t dht_read_data(struct dht22 *dht, float *temp, float *hum)
{
    if (dht_read(dht)) {
        /* Reading temperature */
        *temp = dht->data[2] & 0x7F;
        *temp *= 256;
        *temp += dht->data[3];
        *temp /= 10;

        if (dht->data[2] & 0x80)
            *temp *= -1;

        /* Reading humidity */
        *hum = dht->data[0];
        *hum *= 256;
        *hum += dht->data[1];
        *hum /= 10;
        if (*hum == 0.0f)
            return 0;
        return 1;
    }
    return 0;
}



// Expect the signal line to be at the specified level for a period of time and
// return a count of loop cycles spent at that level (this cycle count can be
// used to compare the relative time of two pulses).  If more than a millisecond
// ellapses without the level changing then the call fails with a 0 response.
// ellapses without the level changing then the call fails with a 0 response.
// This is adapted from Arduino's pulseInLong function (which is only available
// in the very latest IDE versions):
//   https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/wiring_pulse.c
uint8_t expectPulse(_Bool level, uint8_t _pin) {
  uint8_t count = 0;
  
	uint8_t portState = level ? (1 << _pin) : 0;
	while ((PIN_DHT & (1 << _pin)) == portState) {
        _delay_us(1);
    	if (count++ >= _maxcycles) {
    	   return 0; // Exceeded timeout, fail.
    	}
	}

  return count;
}