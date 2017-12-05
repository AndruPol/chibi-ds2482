/*
    ChibiOS/RT - Copyright (C) 2014 Uladzimir Pylinsky aka barthess

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <string.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "usbcfg.h"
#include "ds2482.h"
#include "ds18b20.h"

static const onewireConfig owCfg = {
  .i2cd = &I2CD1,
  .ds2482_cfg = {
	.apu = true,
	.spu = false,
	.ows = false,
  },
};

/*===========================================================================*/
/* Main and generic code.                                                    */
/*===========================================================================*/

/*
 * 1-wire search process
*/
#define SEARCH_PRIO	(NORMALPRIO)
#define SEARCH_INT	250
static THD_WORKING_AREA(waowSearchThread, 512);

static THD_FUNCTION(owSearchThread, arg) {
  (void)arg;

  uint8_t rommax = 2;
  onewireRomAddress romAddr[rommax];
  chRegSetThreadName("owSearchThd");

  while (TRUE) {
    systime_t time = chVTGetSystemTimeX() + MS2ST(SEARCH_INT);

    memset(romAddr, 0, rommax * sizeof(onewireRomAddress));
    onewireSearchRom(&OWD1, romAddr, rommax);
    for (uint8_t i=0; i<rommax; i++) {
      if (romAddr[i].addr[0] == 0x01) {	// DS1990
   		palTogglePad(GPIOA, GPIOA_LED);
      }
    }

    chThdSleepUntil(time);
  }
}


// called on kernel panic
void halt(void){
  port_disable();
  while(TRUE) { }
}


/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  palSetPadMode(GPIOA, GPIOA_LED, PAL_MODE_OUTPUT_PUSHPULL);
  palClearPad(GPIOA, GPIOA_LED);

  onewireObjectInit(&OWD1);
  onewireStart(&OWD1, &owCfg);

  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  uint8_t rommax = 2;
  onewireRomAddress romAddr[rommax];
  memset(romAddr, 0, rommax * sizeof(onewireRomAddress));
  onewireSearchRom(&OWD1, romAddr, rommax);
  for (uint8_t i=0; i<rommax; i++) {
  	if (romAddr[i].addr[0] == DS18B20) {	// DS18B20
		ds18b20Init(&OWD1, &romAddr[i], 10);
  	}
  }
  // ds18b20 polling process
  ds18b20ThdInit();
  chThdCreateStatic(waowSearchThread, sizeof(waowSearchThread), SEARCH_PRIO, owSearchThread, NULL);

  while (true) {
    ds18b20ThdReadStart();

    for (uint8_t i=0; i<DS18B20_NUM; i++) {
      if (ds18b20[i].romaddr.addr[0] == DS18B20) {
    	if (ds18b20[i].error == DS18B20_SUCCESS)
    		chprintf((BaseSequentialStream *) &SDU1, "temp: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x - %d\r\n",
    			ds18b20[i].romaddr.addr[0], ds18b20[i].romaddr.addr[1], ds18b20[i].romaddr.addr[2], ds18b20[i].romaddr.addr[3],
				ds18b20[i].romaddr.addr[4], ds18b20[i].romaddr.addr[5], ds18b20[i].romaddr.addr[6], ds18b20[i].romaddr.addr[7],
				ds18b20[i].temperature
    		);
    	else
    		chprintf((BaseSequentialStream *) &SDU1, "temp: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x - error\r\n");
      }
    }

	chThdSleepMilliseconds(1000);
  }
}
