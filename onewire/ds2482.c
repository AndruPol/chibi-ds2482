/*
 * ds2482.c
 *
 *  Created on: 17.11.2017
 *      Author: andru
 */

#include "hal.h"

#if (HAL_USE_ONEWIRE == TRUE) || defined(__DOXYGEN__)

#include <string.h>

#include "ds2482.h"

/*===========================================================================*/
/* DS2482 registers.                                                         */
/*===========================================================================*/
#define REG_STATUS		0xF0
#define REG_DATA		0xE1
#define REG_CONFIG		0xC3

#define STATUS_1WB		(1<<0)
#define STATUS_PPD		(1<<1)
#define STATUS_SD		(1<<2)
#define STATUS_LL		(1<<3)
#define STATUS_RST		(1<<4)
#define STATUS_SBR		(1<<5)
#define STATUS_TSB		(1<<6)
#define STATUS_DIR		(1<<7)

#define DS2482_APU		(1<<0)
#define DS2482_SPU		(1<<2)
#define DS2482_OWS		(1<<3)

#define CMD_DRST		0xF0	// device reset command
#define CMD_SRP			0xE1	// set read pointer command
#define CMD_WCFG		0xD2	// write configuration command
#define CMD_1WRS		0xB4	// 1-wire reset command
#define CMD_1WSB		0x87	// 1-wire single bit command
#define CMD_1WWB		0xA5	// 1-wire write byte command
#define CMD_1WRB		0x96	// 1-wire read byte command
#define CMD_1WT			0x78	// 1-wire triplet command

#define PWR_APPLY_RSP	1		// apply power response

#define OW_CMD_MS		1		// I2C operation timeout
#define DS2482_POLL_US	200		// DS2482 write polling interval
#define POLL_CMD		5		// 1-wire write polling max
#define POLL_RESET		25		// 1-wire reset polling max

/*===========================================================================*/
/* Board specific.                                                           */
/*===========================================================================*/
#define DS2482_ADDR     0x18
#define DS2482_DRIVER   I2CD1
#define DS2482_PORT     GPIOB
#define DS2482_SCL      GPIOB_SCL	// PB6
#define DS2482_SDA      GPIOB_SDA	// PB7
#define PAL_MODE_ACTIVE	PAL_MODE_STM32_ALTERNATE_OPENDRAIN

static onewire_error_t ds2482_write(onewireDriver *owp, uint8_t *txbuf, uint8_t len);
static onewire_error_t ds2482_read(onewireDriver *owp, uint8_t *rxbuf, uint8_t len);
static bool ds2482_read_config(onewireDriver *owp);
static bool ds2482_write_config(onewireDriver *owp, uint8_t config);
static bool ds2482_write_byte(onewireDriver *owp, uint8_t byte);
static uint8_t ds2482_read_byte(onewireDriver *owp);
static uint8_t ds2482_search_triplet(onewireDriver *owp, uint8_t dir);
static bool ds2482_reset(onewireDriver *owp);
static bool ds2482_speed(onewireDriver *owp, uint8_t speed);
static bool ds2482_level(onewireDriver *owp, uint8_t level);

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/
/**
 * @brief 1-wire driver identifier.
 */
onewireDriver OWD1;

/* I2C config */
static const I2CConfig i2c_cfg = {
    OPMODE_I2C,
    100000,
    STD_DUTY_CYCLE,
};

static const uint8_t onewire_crc_table[256] = {
    0x0,  0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
    0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x1,  0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
    0xe1, 0xbf, 0x5d, 0x3,  0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x2,  0x5c, 0xdf, 0x81, 0x63, 0x3d,
    0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x7,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x6,  0x58,
    0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x4,  0x5a, 0xb8, 0xe6,
    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x5,  0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0xf,
    0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
    0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0xe,  0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0xc,  0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0xd,  0xef, 0xb1,
    0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
    0x8,  0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x9,  0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0xb,  0x88, 0xd6, 0x34, 0x6a,
    0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
    0xb6, 0xe8, 0xa,  0x54, 0xd7, 0x89, 0x6b, 0x35
};

// write len bytes from txbuf to DS2482 & wait while 1WB bit == 0
static onewire_error_t ds2482_write(onewireDriver *owp, uint8_t *txbuf, uint8_t len) {
  msg_t state = MSG_OK;
  // STM32F1 I2C hack
#if defined(STM32F1XX_I2C)
  uint8_t n = 2;
#else
  uint8_t n = 1;
#endif
  uint8_t rcvbuf[n];

  if (len > 0) {
#if I2C_USE_MUTUAL_EXCLUSION
	  i2cAcquireBus(owp->config->i2cd);
#endif
	  state = i2cMasterTransmitTimeout(owp->config->i2cd, DS2482_ADDR, txbuf, len, NULL, 0, MS2ST(OW_CMD_MS));
#if I2C_USE_MUTUAL_EXCLUSION
	  i2cReleaseBus(owp->config->i2cd);
#endif

	  if (state == MSG_TIMEOUT) {
		  return ONEWIRE_I2C_TIMEOUT;
	  }
  }

  uint8_t poll_cnt = 0;
  do {

#if I2C_USE_MUTUAL_EXCLUSION
	  i2cAcquireBus(owp->config->i2cd);
#endif
	  state = i2cMasterReceiveTimeout(owp->config->i2cd, DS2482_ADDR, rcvbuf, n, MS2ST(OW_CMD_MS));
#if I2C_USE_MUTUAL_EXCLUSION
	  i2cReleaseBus(owp->config->i2cd);
#endif

	  if (state == MSG_TIMEOUT) {
		  return ONEWIRE_I2C_TIMEOUT;
	  }

	  owp->reg.status = rcvbuf[0];

	  if ((owp->reg.status & STATUS_1WB) == 0) {
		  return ONEWIRE_SUCCESS;
	  }

	  chThdSleepMicroseconds(DS2482_POLL_US);
  } while (poll_cnt++ < owp->reg.poll_cnt);

  return ONEWIRE_OW_TIMEOUT;
}

// read len bytes from 1-wire to rxbuf
static onewire_error_t ds2482_read(onewireDriver *owp, uint8_t *rxbuf, uint8_t len) {
  msg_t state = MSG_OK;
  uint8_t cmd[2] = { CMD_SRP, REG_DATA };

#if defined(STM32F1XX_I2C)
  osalDbgAssert(len != 1, "STM32F1XX I2C can't read 1 byte");
#endif

#if I2C_USE_MUTUAL_EXCLUSION
  i2cAcquireBus(owp->config->i2cd);
#endif
  state = i2cMasterTransmitTimeout(owp->config->i2cd, DS2482_ADDR, cmd, 2, rxbuf, len, MS2ST(OW_CMD_MS));
#if I2C_USE_MUTUAL_EXCLUSION
  i2cReleaseBus(owp->config->i2cd);
#endif

  if (state == MSG_TIMEOUT) {
    return ONEWIRE_I2C_TIMEOUT;
  }

  return ONEWIRE_SUCCESS;
}

// read DS2482 configuration register to owp->reg.config
static bool ds2482_read_config(onewireDriver *owp) {
  uint8_t cmd[2] = { CMD_SRP, REG_CONFIG };

  // STM32F1 I2C hack
#if defined(STM32F1XX_I2C)
  uint8_t n = 2;
#else
  uint8_t n = 1;
#endif
  uint8_t rcvbuf[n];

  ds2482_write(owp, cmd, 2);

#if I2C_USE_MUTUAL_EXCLUSION
  i2cAcquireBus(owp->config->i2cd);
#endif
  msg_t state = i2cMasterReceiveTimeout(owp->config->i2cd, DS2482_ADDR, rcvbuf, n, MS2ST(OW_CMD_MS));
#if I2C_USE_MUTUAL_EXCLUSION
  i2cReleaseBus(owp->config->i2cd);
#endif

  if (state == MSG_TIMEOUT) {
	owp->error = ONEWIRE_I2C_TIMEOUT;
	return false;
  }

  owp->reg.config = rcvbuf[0];
  return true;
}

// write config byte to DS2482 configuration register
static bool ds2482_write_config(onewireDriver *owp, uint8_t config) {
  uint8_t cmd[2] = { CMD_WCFG, 0 };
  cmd[1] = config | (~config << 4);

  ds2482_write(owp, cmd, 2);

  if (!ds2482_read_config(owp))
	  return false;

  if (owp->reg.config != config) return false;
  return true;
}

// write speed to OWS bit DS2482 configuration register
static bool ds2482_speed(onewireDriver *owp, uint8_t speed) {
	uint8_t config = owp->reg.config;
	if (speed == SPEED_OVERDRIVE)
		config |= DS2482_OWS;
	else
		config = config & ~DS2482_OWS;
	if (!ds2482_write_config(owp, config))
		return false;
	return true;
}

// set pullup level to PULLUP_NORMAL
static bool ds2482_level(onewireDriver *owp, uint8_t level) {
	uint8_t config = owp->reg.config;
	if (level == PULLUP_STRONG)
		return false;

	config = config & ~DS2482_SPU;
	if (!ds2482_write_config(owp, config))
		return false;
	return true;
}

#if USE_DS2482_BIT_OPERATIONS
// write bit to 1-wire bus
static bool ds2482_write_bit(onewireDriver *owp, uint8_t bit) {
  uint8_t cmd[2] = { CMD_1WSB, 0 };

  if (bit)
	cmd[1] = 0x80;

  owp->error = ds2482_write(owp, cmd, 2);
  if (owp->error == ONEWIRE_SUCCESS)
   	return true;

  return false;
}

// read bit from 1-wire bus
static uint8_t ds2482_read_bit(onewireDriver *owp) {
  uint8_t cmd[2] = { CMD_SRP, REG_STATUS }, bit;

  owp->error = ds2482_write(owp, cmd, 2);
  if (owp->error != ONEWIRE_SUCCESS)
   	return 0;

  bit = 0;
  if (owp->reg.status & STATUS_SBR)
   	bit = 1;

  return bit;
}

// read bit from 1-wire bus with strong pullup
static uint8_t ds2482_read_bitpower(onewireDriver *owp) {

  if (owp->config->ds2482_cfg.spu) {	// strong pullup needed
	  uint8_t config = owp->reg.config;
	  config |= DS2482_SPU;
	  if (!ds2482_write_config(owp, config)) {
		  ds2482_level(owp, PULLUP_NORMAL);
		  return false;
	  }
  }
  if (ds2482_read_bit(owp) != PWR_APPLY_RSP) {
	  ds2482_level(owp, PULLUP_NORMAL);
	  return false;
  }
  return true;
}
#endif

// write byte to 1-wire bus
static bool ds2482_write_byte(onewireDriver *owp, uint8_t byte) {
  uint8_t cmd[2] = { CMD_1WWB, 0 };

  if (owp->config->ds2482_cfg.spu) {	// strong pullup needed
	  uint8_t config = owp->reg.config;
	  config |= DS2482_SPU;
	  if (!ds2482_write_config(owp, config)) {
		  ds2482_level(owp, PULLUP_NORMAL);
		  return false;
	  }
  }

  cmd[1] = byte;
  owp->error = ds2482_write(owp, cmd, 2);
  if (owp->error == ONEWIRE_SUCCESS)
   	return true;

  return false;
}

// read byte from 1-wire bus
static uint8_t ds2482_read_byte(onewireDriver *owp) {
  uint8_t cmd = CMD_1WRB;
  // STM32F1 I2C hack
#if defined(STM32F1XX_I2C)
  uint8_t n = 2;
#else
  uint8_t n = 1;
#endif
  uint8_t rcvbuf[n];

  owp->error = ds2482_write(owp, &cmd, 1);
  if (owp->error != ONEWIRE_SUCCESS)
    return 0;

  owp->error = ds2482_read(owp, rcvbuf, n);
  if (owp->error != ONEWIRE_SUCCESS)
	return 0;

  return rcvbuf[0];
}

// search triplet operation on 1-wire bus
static uint8_t ds2482_search_triplet(onewireDriver *owp, uint8_t dir) {
  uint8_t cmd[2] = { CMD_1WT, 0 };
  cmd[1] = dir ? 0x80 : 0x00;

  owp->error = ds2482_write(owp, cmd, 2);
  if (owp->error != ONEWIRE_SUCCESS)
   	return 0;

  return owp->reg.status;
}

// send reset to 1-wire bus and return device presence
static bool ds2482_reset(onewireDriver *owp) {
  uint8_t cmd = CMD_1WRS;

  owp->reg.slave_present = false;

  owp->reg.poll_cnt = POLL_RESET;
  owp->error = ds2482_write(owp, &cmd, 1);
  owp->reg.poll_cnt = POLL_CMD;

  if (owp->error != ONEWIRE_SUCCESS)
	  return false;

  if (owp->reg.status & STATUS_SD)
	  owp->error = ONEWIRE_SHORT_DETECT;

  if (owp->reg.status & STATUS_PPD) {
	  owp->reg.slave_present = true;
	  return true;
  }

  return false;
}

/**
 * @brief   Initializes @p onewireDriver structure.
 *
 * @param[out] owp    pointer to the @p onewireDriver object
 *
 * @init
 */
void onewireObjectInit(onewireDriver *owp) {
  osalDbgCheck(NULL != owp);

  owp->config = NULL;
  owp->error = ONEWIRE_SUCCESS;
  owp->reg.slave_present = false;
  owp->reg.state = ONEWIRE_STOP;
  owp->reg.status = 0;
  owp->reg.poll_cnt = POLL_CMD;
  owp->i2ccfg = i2c_cfg;
  chMtxObjectInit(&owp->owMtx);
}

/**
 * @brief   Configures and activates the 1-wire driver.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 * @param[in] config    pointer to the @p onewireConfig object
 *
 * @api
 */
void onewireStart(onewireDriver *owp, const onewireConfig *config) {
  uint8_t cmd[2];

  osalDbgCheck((NULL != owp) && (NULL != config));
  osalDbgAssert(I2C_STOP == config->i2cd->state,
      "I2C will be started by onewire driver internally");
  osalDbgAssert(ONEWIRE_STOP == owp->reg.state, "Invalid state");

  owp->config = config;
  i2cStart(owp->config->i2cd, &owp->i2ccfg);

  /* tune pins for I2C*/
  palSetPadMode(DS2482_PORT, DS2482_SCL, PAL_MODE_ACTIVE);
  palSetPadMode(DS2482_PORT, DS2482_SDA, PAL_MODE_ACTIVE);

  chMtxLock(&owp->owMtx);
  cmd[0] = CMD_DRST;
  owp->error = ds2482_write(owp, cmd, 1);

  if (owp->error == ONEWIRE_SUCCESS) {
	  uint8_t ds2482_cfg = 0;
	  if (config->ds2482_cfg.apu)
		  ds2482_cfg |= DS2482_APU;

	  ds2482_write_config(&OWD1, ds2482_cfg);
  }
  owp->reg.state = ONEWIRE_READY;
  chMtxUnlock(&owp->owMtx);
}

/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 *
 * @api
 */
void onewireStop(onewireDriver *owp) {
  osalDbgCheck(NULL != owp);

  i2cStop(owp->config->i2cd);
  owp->config = NULL;
  owp->reg.state = ONEWIRE_STOP;
}

/**
 * @brief   read DS2482 configuration register to owp->reg.config.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 *
 * @return              Bool read success.
 *
 * @api
 */
bool ds2482ReadConfig(onewireDriver *owp) {
  osalDbgCheck(NULL != owp);
  osalDbgAssert(owp->reg.state == ONEWIRE_READY, "Invalid state");

  chMtxLock(&owp->owMtx);
  bool ret = ds2482_read_config(owp);
  chMtxUnlock(&owp->owMtx);
  return ret;
}

/**
 * @brief   write config to DS2482 configuration register.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 *
 * @param[in] config    configuration byte
 *
 * @return              Bool write success.
 *
 * @api
 */
bool ds2482WriteConfig(onewireDriver *owp, uint8_t config) {
  osalDbgCheck(NULL != owp);
  osalDbgAssert(owp->reg.state == ONEWIRE_READY, "Invalid state");

  chMtxLock(&owp->owMtx);
  bool ret = ds2482_write_config(owp, config);
  chMtxUnlock(&owp->owMtx);
  return ret;
}

/**
 * @brief   Calculates 1-wire CRC.
 *
 * @param[in] buf     pointer to the data buffer
 * @param[in] len     length of data buffer
 *
 * @init
 */
uint8_t onewireCRC(const uint8_t *buf, size_t len) {
  uint8_t ret = 0;

  for (uint8_t i=0; i<len; i++)
    ret = onewire_crc_table[ret ^ buf[i]];

  return ret;
}

/**
 * @brief     Generate reset pulse on bus.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 *
 * @return              Bool flag denoting device presence.
 * @retval true         There is at least one device on bus.
 */
bool onewireReset(onewireDriver *owp) {
  osalDbgCheck(NULL != owp);
  osalDbgAssert(owp->reg.state == ONEWIRE_READY, "Invalid state");

  bool lock = chMtxTryLock(&owp->owMtx);
  bool ret = ds2482_reset(owp);
  if (lock)
	  chMtxUnlock(&owp->owMtx);

  return ret;
}

/**
 * @brief     Read some bytes from slave device.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 * @param[out] rxbuf    pointer to the buffer for read data
 * @param[in] rxbytes   amount of data to be received
 *
 * @return              Bool read success.
 */
bool onewireRead(onewireDriver *owp, uint8_t *rxbuf, size_t rxbytes) {
  osalDbgCheck((NULL != owp) && (NULL != rxbuf));
  osalDbgCheck((rxbytes > 0) && (rxbytes < 65536));
  osalDbgAssert(owp->reg.state == ONEWIRE_READY, "Invalid state");

  bool lock = chMtxTryLock(&owp->owMtx);
  memset(rxbuf, 0, rxbytes);
  for (size_t i=0; i<rxbytes; i++) {
	  rxbuf[i] = ds2482_read_byte(owp);
	  if (owp->error != ONEWIRE_SUCCESS)
		  break;
  }
  if (lock)
	  chMtxUnlock(&owp->owMtx);
  return (owp->error == ONEWIRE_SUCCESS);
}

/**
 * @brief     Write some bytes to slave device.
 *
 * @param[in] owp           pointer to the @p onewireDriver object
 * @param[in] txbuf         pointer to the buffer with data to be written
 * @param[in] txbytes       amount of data to be written
 *
 * @return              	Bool write success.
 */
bool onewireWrite(onewireDriver *owp, const uint8_t *txbuf, size_t txbytes) {
  osalDbgCheck((NULL != owp) && (NULL != txbuf));
  osalDbgCheck((txbytes > 0) && (txbytes < 65536));
  osalDbgAssert(owp->reg.state == ONEWIRE_READY, "Invalid state");

  bool lock = chMtxTryLock(&owp->owMtx);
  for (size_t i=0; i<txbytes; i++) {
	  ds2482_write_byte(owp, txbuf[i]);
	  if (owp->error != ONEWIRE_SUCCESS)
		  break;
  }
  if (lock)
	  chMtxUnlock(&owp->owMtx);
  return (owp->error == ONEWIRE_SUCCESS);
}

#if ONEWIRE_USE_SEARCH_ROM
/**
 * @brief   Performs tree search on bus.
 * @note    This function does internal 1-wire reset calls every search
 *          iteration.
 *
 * @param[in] owp         pointer to a @p OWDriver object
 * @param[out] result     pointer to buffer for discovered ROMs
 * @param[in] max_rom_cnt buffer size in ROMs count for overflow prevention
 *
 * @return              Count of discovered ROMs.
 * @retval 0            no ROMs found or communication error occurred.
 */
uint8_t onewireSearchRom(onewireDriver *owp, onewireRomAddress *result,
                        size_t max_rom_cnt) {

  osalDbgCheck(NULL != owp);
  osalDbgAssert(ONEWIRE_READY == owp->reg.state, "Invalid state");
  osalDbgCheck((max_rom_cnt <= 256) && (max_rom_cnt > 0));

  uint8_t rom_cnt = 0;
  uint8_t lastDevice = 0;
  uint8_t curDevice = 0;
  uint8_t romBit, lastCollision, currentCollision, currentSelection;

  chMtxLock(&owp->owMtx);

  owp->search_rom.single_device = false;
  owp->search_rom.devices_found = 0;
  lastCollision = 0;
  while (rom_cnt++ < max_rom_cnt) {
	currentCollision = 0;

	// reset the search
	if (!onewireReset(owp)) {
		goto end;
	}

	// issue the search command
	uint8_t cmd[2] = { CMD_1WWB, ONEWIRE_CMD_SEARCH_ROM };
	owp->error = ds2482_write(owp, cmd, 2);

	for (romBit = 1; romBit <= 64; romBit++) {
		if (romBit < lastCollision) {
			if (result[lastDevice].addr[(romBit-1) >> 3] & 1 << ((romBit-1) & 0x07))
				currentSelection = 1;
			else
				currentSelection = 0;
		} else {
			// if equal to last pick 1, if not then pick 0
			if (romBit == lastCollision)
				currentSelection = 1;
			else
				currentSelection = 0;
		}
		// Perform a triple operation on the DS2482 which will perform
		// 2 read bits and 1 write bit
		uint8_t status = ds2482_search_triplet(owp, currentSelection);
		// check bit results in status byte
		uint8_t bit_buf;
		bit_buf = ((status & STATUS_SBR) == STATUS_SBR) ? 2 : 0;
		bit_buf += ((status & STATUS_TSB) == STATUS_TSB) ? 1 : 0;
		currentSelection = ((status & STATUS_DIR) == STATUS_DIR) ? (uint8_t) 1 : (uint8_t) 0;

		if (bit_buf == 0b11) {
			// search error
			goto end;
		} else {
			if (bit_buf == 0b00 && currentSelection == 0) {
				currentCollision = romBit;
			}
			if (currentSelection == 1) {
				result[curDevice].addr[(romBit-1) >> 3] |= 1 << ((romBit-1) & 0x07);
			} else {
				result[curDevice].addr[(romBit-1) >> 3] &= ~(1 << ((romBit-1) & 0x07));
			}
		}
	}
	if (romBit == 65 && onewireCRC(&result[curDevice].addr[0], 7) == result[curDevice].addr[7]) {
		owp->search_rom.devices_found++;
		lastDevice = curDevice;
		curDevice++;
	}
	lastCollision = currentCollision;
	if (currentCollision == 0) {
		// no more collisions
		break;
	}
  } // while

  owp->search_rom.result = ONEWIRE_SEARCH_ROM_SUCCESS;
  if (owp->search_rom.devices_found == 1)
	  owp->search_rom.single_device = true;

  chMtxUnlock(&owp->owMtx);
  return owp->search_rom.devices_found;

end:
  owp->search_rom.result = ONEWIRE_SEARCH_ROM_ERROR;
  chMtxUnlock(&owp->owMtx);
  return 0;
}
#endif /* ONEWIRE_USE_SEARCH_ROM */

#endif
