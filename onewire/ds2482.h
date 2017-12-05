/*
 * ds2482.h
 *
 *  Created on: 17.11.2017
 *      Author: andru
 */

#ifndef DS2482_H_
#define DS2482_H_

#if (HAL_USE_ONEWIRE == TRUE) || defined(__DOXYGEN__)

/**
 * @brief   Aliases for 1-wire protocol.
 */
#define ONEWIRE_CMD_READ_ROM              0x33
#define ONEWIRE_CMD_SEARCH_ROM            0xF0
#define ONEWIRE_CMD_MATCH_ROM             0x55
#define ONEWIRE_CMD_SKIP_ROM              0xCC

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !HAL_USE_I2C
#error "1-wire Driver requires HAL_USE_I2C"
#endif

#if !HAL_USE_PAL
#error "1-wire Driver requires HAL_USE_PAL"
#endif

#define USE_DS2482_BIT_OPERATIONS	FALSE

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/
/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  ONEWIRE_UNINIT = 0,         /**< Not initialized.                         */
  ONEWIRE_STOP = 1,           /**< Stopped.                                 */
  ONEWIRE_READY = 2,          /**< Ready.                                   */
} onewire_state_t;

#if ONEWIRE_USE_SEARCH_ROM
/**
 * @brief   Search ROM procedure possible state.
 */
typedef enum {
  ONEWIRE_SEARCH_ROM_SUCCESS = 0,   /**< ROM successfully discovered.       */
  ONEWIRE_SEARCH_ROM_LAST = 1,      /**< Last ROM successfully discovered.  */
  ONEWIRE_SEARCH_ROM_ERROR = 2      /**< Error happened during search.      */
} search_rom_result_t;
#endif /* ONEWIRE_USE_SEARCH_ROM */

/**
 * @brief   OW devices structure
 */
typedef struct {
  uint8_t	addr[8];	// OW address
} onewireRomAddress;

/**
 * @brief   DS2482 configuration register.
 */
typedef struct {
  bool		apu;	// APU bit, 1 - active pullup on
  bool		spu;	// SPU bit, 1 - strong pullup on
  bool		ows;	// OWS bit, 1 - overdrive on
} ds2482_cfg_t;

/**
 * @brief   Driver configuration structure.
 */
typedef struct {
  /**
   * @brief Pointer to @p I2C driver used for communication.
   */
  I2CDriver         *i2cd;
  /**
   * @brief DS2482 configuration.
   */
  ds2482_cfg_t		ds2482_cfg;
} onewireConfig;

#if ONEWIRE_USE_SEARCH_ROM
/**
 * @brief     Search ROM registry. Contains small variables used
 *            in 'search ROM' procedure.
 */
typedef struct {
  /**
   * @brief Bool flag. If @p true than only bus has only one slave device.
   */
  uint16_t      single_device: 1;
  /**
   * @brief Result of discovery procedure (@p search_rom_result_t enum).
   */
  uint16_t      result: 2;
  /**
   * @brief Total device count discovered on bus.
   * @note  Maximum 256.
   */
  uint16_t      devices_found: 8;
} onewire_search_rom_t;
#endif

/**
 * @brief     Onewire registry. Some small variables combined
 *            in single machine word to save RAM.
 */
typedef struct {
  /**
   * @brief   Bool flag. If @p true than at least one device presence on bus.
   */
  uint32_t      slave_present: 1;
  /**
   * @brief   Driver internal state (@p onewire_state_t enum).
   */
  uint32_t      state: 2;
  /**
   * @brief   DS2482 status register.
   */
  uint8_t       status;
  /**
   * @brief   DS2482 config register.
   */
  uint8_t       config;
  /**
   * @brief   DS2482 max polling count.
   */
  uint16_t      poll_cnt;
} onewire_reg_t;

/**
 * @brief   ONEWIRE & DS2482 communication error.
 */
typedef enum {
  ONEWIRE_SUCCESS = 0,			   /* Onewire success.               */
  ONEWIRE_I2C_TIMEOUT = 1,         /* DS2482 I2C operation timeout.  */
  ONEWIRE_OW_TIMEOUT = 2,          /* Onewire write timeout.       */
  ONEWIRE_SHORT_DETECT =3,		   /* Onewire short detected .       */
} onewire_error_t;

/**
 * @brief   DS2482 communication speed.
 */
typedef enum {
  SPEED_STANDARD = 0,			  /** standart speed.       */
  SPEED_OVERDRIVE = 1,         	  /** overdrive speed.  */
} ds2482_speed_t;

/**
 * @brief   DS2482 pullup mode.
 */
typedef enum {
  PULLUP_NORMAL = 0,			  /** standart pullup.       */
  PULLUP_STRONG = 1,         	  /** strong pullup.  */
} ds2482_pullup_t;

/**
 * @brief     Structure representing an 1-wire driver.
 */
typedef struct {
  /**
   * @brief   Onewire registry.
   */
  onewire_reg_t         reg;
  /**
   * @brief   Onewire error.
   */
  onewire_error_t       error;
  /**
   * @brief   Onewire config.
   */
  const onewireConfig   *config;
  /**
   * @brief   Config for underlying I2C driver.
   */
  I2CConfig             i2ccfg;
  /**
   * @brief   Onewire mutex.
   */
  mutex_t               owMtx;

#if ONEWIRE_USE_SEARCH_ROM
  /**
   * @brief   Search ROM helper structure.
   */
  onewire_search_rom_t  search_rom;
#endif /* ONEWIRE_USE_SEARCH_ROM */
} onewireDriver;


/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern onewireDriver OWD1;

#ifdef __cplusplus
extern "C" {
#endif
  void onewireObjectInit(onewireDriver *owp);
  void onewireStart(onewireDriver *owp, const onewireConfig *config);
  void onewireStop(onewireDriver *owp);
  bool onewireReset(onewireDriver *owp);
  bool onewireRead(onewireDriver *owp, uint8_t *rxbuf, size_t rxbytes);
  uint8_t onewireCRC(const uint8_t *buf, size_t len);
  bool onewireWrite(onewireDriver *owp,
                    const uint8_t *txbuf,
                    size_t txbytes);
  bool ds2482ReadConfig(onewireDriver *owp);
  bool ds2482WriteConfig(onewireDriver *owp, uint8_t config);
#if ONEWIRE_USE_SEARCH_ROM
  uint8_t onewireSearchRom(onewireDriver *owp,
		  	  	  	  	  onewireRomAddress *result,
                          size_t max_rom_cnt);
#endif /* ONEWIRE_USE_SEARCH_ROM */
#ifdef __cplusplus
}
#endif


#endif /* HAL_USE_ONEWIRE */

#endif /* DS2482_H_ */
