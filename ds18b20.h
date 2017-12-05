#ifndef __DS_18B20_H__
#define __DS_18B20_H__

#define DS18B20_NUM			2
#define DS18B20				0x28
#define DS18B20_FAIL		-255

#define DS18B20_CONV09		95
#define DS18B20_CONV10		195
#define DS18B20_CONV11		390
#define DS18B20_CONV12		780

#include "ds2482.h"

typedef enum {
  DS18B20_SUCCESS = 0,
  DS18B20_ERROR,
} ds18b20_error_t;

typedef struct {
  uint16_t convtime;			// convertation time, mS
  int16_t temperature;			// temperature * 10
  ds18b20_error_t error;		// ds18b20 errors
  uint8_t resolution;			// resolution bits
  onewireRomAddress romaddr;	// ds18b20 rom address
} ds18b20_sensor_t;

#if 0
typedef struct {
	onewireDriver* drv;
	ds18b20_sensor_t* sensors;
} ds18b20_read_t;
#endif
extern ds18b20_sensor_t ds18b20[DS18B20_NUM];

#ifdef __cplusplus
extern "C" {
#endif

/* 
   initialise and configure sensors 
 */
bool ds18b20Init(onewireDriver* drv, const onewireRomAddress *address,
		      const uint8_t precBits);

/*
  ask conversion, wait for the conversion to be done, then return value,
  could be the simplest way to acquire data when there is only one sensor
  choose timeout (mS) from
  Tconv conversion time (p.3 Datasheet)
   	   9bit 	- 93.75 mS
   	   10bit 	- 187.5 mS
   	   11bit 	- 375 mS
   	   12bit 	- 750 mS
*/
int16_t ds18b20ReadTemp(onewireDriver* drv, const onewireRomAddress *address, uint16_t timeout);

/*
  separate ask conversion command and get values.
  this is the fastest way to acquire data when there is a lot of sensors :
  1/ ask conversion for all sensors
  2/ wait time accordingly to precision (see datasheet)
  3/ get the temperature for one or all sensors
 */
bool ds18b20AskTemp(onewireDriver* drv, const onewireRomAddress *address);
int16_t ds18b20ReadTempFromRam(onewireDriver* drv, const onewireRomAddress *address);

void ds18b20ThdInit(void);
void ds18b20ThdReadStart(void);

#ifdef __cplusplus
}
#endif

#endif //__DS_18B20_H__
