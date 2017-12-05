#include "ch.h"
#include "hal.h"

#include <string.h>

#include "ds2482.h"
#include "ds18b20.h"

#define SEARCH_ROM		0xF0
#define ALARM_SEARCH	0xEC
#define READ_ROM		0x33
#define MATCH_ROM		0x55
#define SKIP_ROM		0xCC

// DS18B20 specific
#define CONVERT			0x44
#define WRITE_SCRATCH	0x4E
#define READ_SCRATCH	0xBE
#define COPY_SCRATCH	0x48
#define RECALL_E2		0xB8
#define READ_POWER		0xB4

// DS18B20 temperature convertation to int16_t
#define BIT2_1 			(1<<3)
#define BIT2_2 			(1<<2)
#define BIT2_3 			(1<<1)
#define BIT2_4 			(1<<0)

#define POWER2_1 		(5000)
#define POWER2_2 		(2500)
#define POWER2_3 		(1250)
#define POWER2_4 		(0650)

ds18b20_sensor_t ds18b20[DS18B20_NUM];

#define DS18B20_PRIO	(NORMALPRIO)
static THD_WORKING_AREA(waDS18B20Thread, 512);
static binary_semaphore_t ds18b20_read_sem;

// return DS18B20 temperature * 10
static int16_t tconv(int16_t raw) {
	int16_t frac =  ((raw & BIT2_1) ? POWER2_1 : 0) +
					((raw & BIT2_2) ? POWER2_2 : 0) +
					((raw & BIT2_3) ? POWER2_3 : 0) +
					((raw & BIT2_4) ? POWER2_4 : 0);
	frac = (frac - frac / 1000) >= 500 ? frac + 500 : frac - 500;
	frac = (raw & (1<<15)) ? (-1 * frac / 1000) : (frac / 1000);
	return (10 * (raw >> 4) + frac);
}

static bool ds18b20_seq_start(onewireDriver* drv, const onewireRomAddress *address) {
  uint8_t cmd;

  if (!onewireReset(drv))
	  return false;
  if (address == NULL) {
	cmd = SKIP_ROM;
	if (!onewireWrite(drv, &cmd, 1))
		return false;
  } else {
	cmd = MATCH_ROM;
	if (!onewireWrite(drv, &cmd, 1))
		return false;
	if (!onewireWrite(drv, address->addr, sizeof(onewireRomAddress)))
		return false;
  }
  return true;
}

bool ds18b20Init(onewireDriver* drv, const onewireRomAddress *address, const uint8_t precBits) {
  uint8_t cfg[] = {WRITE_SCRATCH, 0x0, 0x0, 0x0};
  uint8_t resolution = 9;
  uint16_t convtime = DS18B20_CONV09;

  // configuration register bit5 - R0, bit6 - R1 setup (p.9 Datasheet), default 9bit
  switch (precBits) {
  case 10:
	  resolution = 10;
	  convtime = DS18B20_CONV10;
	  cfg[3] = 0b00100000;
	  break;
  case 11:
	  resolution = 11;
	  convtime = DS18B20_CONV11;
	  cfg[3] = 0b01000000;
	  break;
  case 12:
	  resolution = 12;
	  convtime = DS18B20_CONV12;
	  cfg[3] = 0b01100000;
	  break;
  default:
	  resolution = 9;
	  convtime = DS18B20_CONV09;
	  cfg[3] = 0b00000000;
	  break;
  }
  
  chMtxLock(&drv->owMtx);
  if (!ds18b20_seq_start(drv, address))
	  goto end;
  if (!onewireWrite(drv, cfg, sizeof(cfg)))
	  goto end;
  chMtxUnlock(&drv->owMtx);

  for (uint8_t i=0; i<DS18B20_NUM; i++) {
	if (ds18b20[i].romaddr.addr[0] == DS18B20) {
	  if (memcmp(ds18b20[i].romaddr.addr, address, sizeof(onewireRomAddress)) == 0) {
		ds18b20[i].error = DS18B20_SUCCESS;
		ds18b20[i].resolution = resolution;
		ds18b20[i].convtime = convtime;
		memcpy((onewireRomAddress*) ds18b20[i].romaddr.addr, (onewireRomAddress*) address, sizeof(onewireRomAddress));
		break;
	  } else {
		continue;
	  }
	}
	ds18b20[i].error = DS18B20_SUCCESS;
	ds18b20[i].resolution = resolution;
	ds18b20[i].convtime = convtime;
	memcpy((onewireRomAddress*) ds18b20[i].romaddr.addr, (onewireRomAddress*) address, sizeof(onewireRomAddress));
	break;
  }
  return true;

end:
  chMtxUnlock(&drv->owMtx);
  return false;
}

// read DS18B20 temperature * 10
//  return (rawTemp * 0.0625f);
int16_t ds18b20ReadTemp(onewireDriver* drv, const onewireRomAddress *address, uint16_t timeout) {
  uint8_t cmd, ram[9];

  chMtxLock(&drv->owMtx);
  if (!ds18b20_seq_start(drv, address))
	goto end;

  cmd = CONVERT;
  if (!onewireWrite(drv, &cmd, 1))
	goto end;

  chThdSleepMilliseconds(timeout);
  
  if (!ds18b20_seq_start(drv, address))
	goto end;

  cmd = READ_SCRATCH;
  if (!onewireWrite(drv, &cmd, 1))
	goto end;

  if (!onewireRead(drv, ram, sizeof(ram)))
	goto end;

  if (onewireCRC(ram, 8) != ram[8])
	goto end;

  chMtxUnlock(&drv->owMtx);
  return tconv((ram[1] << 8) | ram[0]);

end:
  chMtxUnlock(&drv->owMtx);
  return DS18B20_FAIL;
}

bool ds18b20AskTemp(onewireDriver* drv, const onewireRomAddress *address) {
  uint8_t cmd = CONVERT;

  chMtxLock(&drv->owMtx);
  if (!ds18b20_seq_start(drv, address))
	goto end;
  if (!onewireWrite(drv, &cmd, 1))
	goto end;
  chMtxUnlock(&drv->owMtx);

  return true;

end:
  chMtxUnlock(&drv->owMtx);
  return false;
}

int16_t ds18b20ReadTempFromRam(onewireDriver* drv, const onewireRomAddress *address) {
  uint8_t cmd = READ_SCRATCH, ram[9];

  chMtxLock(&drv->owMtx);
  if (!ds18b20_seq_start(drv, address))
	goto end;

  if (!onewireWrite(drv, &cmd, 1))
	goto end;

  if (!onewireRead(drv, ram, sizeof(ram)))
	goto end;

  if  (onewireCRC(ram, 8) != ram[8])
	goto end;

  chMtxUnlock(&drv->owMtx);
  return tconv((ram[1] << 8) | ram[0]);

end:
  chMtxUnlock(&drv->owMtx);
  return DS18B20_FAIL;
}

/*
 * DS18B20 read temperature process
*/
static THD_FUNCTION(DS18B20Thread, arg) {
  (void)arg;

  chRegSetThreadName("DS18B20Thd");

  while (TRUE) {
	chBSemWait(&ds18b20_read_sem);

	uint16_t convtime = ds18b20[0].convtime;
    for (uint8_t i=0; i < DS18B20_NUM; i++) {
	  ds18b20[i].error = DS18B20_SUCCESS;
      if (convtime < ds18b20[i].convtime)
    	convtime = ds18b20[i].convtime;
    }

    for (uint8_t i=0; i < DS18B20_NUM; i++) {
      if (ds18b20[i].romaddr.addr[0] == DS18B20) {
    	if (!ds18b20AskTemp(&OWD1, (onewireRomAddress *) ds18b20[i].romaddr.addr)) {
    		ds18b20[i].error = DS18B20_ERROR;
    		continue;
    	}
      }
    }

    systime_t time = chVTGetSystemTimeX() + MS2ST(convtime);
    chThdSleepUntil(time);

    for (uint8_t i=0; i < DS18B20_NUM; i++) {
      if (ds18b20[i].romaddr.addr[0] == DS18B20) {
  		if (ds18b20[i].error == DS18B20_SUCCESS) {
  			ds18b20[i].temperature = ds18b20ReadTempFromRam(&OWD1, (onewireRomAddress *) ds18b20[i].romaddr.addr);
  			if (ds18b20[i].temperature == DS18B20_FAIL)
  	    		ds18b20[i].error = DS18B20_ERROR;
  		}
      }
    }
  }//while
}

// creates DS1820B temperature polling process
void ds18b20ThdInit(void) {
	chBSemObjectInit(&ds18b20_read_sem, TRUE);
	chThdCreateStatic(waDS18B20Thread, sizeof(waDS18B20Thread), DS18B20_PRIO, DS18B20Thread, NULL);
}

void ds18b20ThdReadStart(void) {
	chBSemSignal(&ds18b20_read_sem);
}
