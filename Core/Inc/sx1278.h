#ifndef __SX1278_H__
#define __SX1278_H__

#include "stm32f4xx_hal.h"

#define SPI_TIMEOUT                         1000
#define LORA_VERSION                        0x12
#define RX_MAX_SIZE                         100

/* lora register address */
#define RegFifo								0x00
#define RegOpMode							0x01
#define	RegFrMsb							0x06
#define	RegFrMid							0x07
#define	RegFrLsb							0x08
#define	RegPaConfig						    0x09
#define RegPaRamp							0x0A
#define RegOcp								0x0B
#define RegLna								0x0C
#define RegFiFoAddPtr						0x0D
#define RegFiFoTxBaseAddr			    	0x0E
#define RegFiFoRxBaseAddr				    0x0F
#define RegFiFoRxCurrentAddr			    0x10
#define RegIrqFlags							0x12
#define RegRxNbBytes						0x13
#define RegPktRssiValue					    0x1A
#define RegModemConfig1				        0x1D
#define RegModemConfig2				        0x1E
#define RegModemConfig3				        0x26
#define RegSymbTimeoutL				        0x1F
#define RegPreambleMsb					    0x20
#define RegPreambleLsb					    0x21
#define RegPayloadLength				    0x22
#define RegDioMapping1					    0x40
#define RegDioMapping2					    0x41
#define RegSyncWord                    	    0x39
#define RegVersion							0x42

/* IRQ Register */
#define IRQ_CAD_DETECTED						(0x01 << 0)
#define IRQ_FHSS_CHANGE_CHANNEL					(0x01 << 1)
#define IRQ_CAD_DONE							(0x01 << 2)
#define	IRQ_TX_DONE								(0x01 << 3)
#define IRQ_VALID_HEADER						(0x01 << 4)
#define IRQ_PAYLOAD_CRC_ERROR					(0x01 << 5)
#define IRQ_RX_DONE								(0x01 << 6)
#define	IRQ_RX_TIME_OUT							(0x01 << 7)

/* Clear IRQ Flags */
#define lora_clear_irq(sx1278, irq) lora_write_reg(sx1278, RegIrqFlags, irq)

typedef enum {
	SLEEP_MODE,
	STANDBY_MODE,
	FSTX,
	TRANSMIT_MODE,
	FSRX,
	RXCONTINUOUS_MODE,
	RXSINGLE_MODE,
	CAD_MODE
} lora_mode_t; /* lora operation mode  */

/*lora signal bandwidth */
typedef enum {
	BW_7_8_KHZ,
	BW_10_4_KHZ,
	BW_15_6_KHZ,
	BW_20_8_KHZ,
	BW_31_25_KHZ,
	BW_41_7_KHZ,
	BW_62_5_KHZ,
	BW_125_KHZ,
	BW_250_KHZ,
	BW_500_KHZ
} bandwidth_t;

typedef enum {
	CR_4_5 = 1,
	CR_4_6,
	CR_4_7,
	CR_4_8
} codingrate_t; /* lora codingrate  */

typedef enum {
	SF_6 = 6,
	SF_7,
	SF_8,
	SF_9,
	SF_10,
	SF_11,
	SF_12
} SF_t; /* lora spreadingFactor */

typedef enum {
	LORA_OK             = 200,
	LORA_NOT_FOUND      = 404,
	LORA_LARGE_PAYLOAD  = 413,
	LORA_UNAVAILABLE    = 503
} status_t; /* lora status */

typedef enum {
	LORA_SUCCESS,
	LORA_EINVAL,
	LORA_ENOMEM,
	LORA_EFAULT
} lora_err_t;

typedef enum {
	POWER_11_DB = 0xF6,
	POWER_14_DB = 0xF9,
	POWER_17_DB = 0xFC,
	POWER_20_DB = 0xFF
} power_t; /* lora power gain */

typedef union {
	u32 val;
	struct decode {
		u8 byte_0;
		u8 byte_1;
		u8 byte_2;
		u8 byte_3;
	} byte;
} u32_t;

#pragma pack(1)

struct lora_dev {
	/* LoRa Feature */
	u32 frequency;
	SF_t spreadingFactor;
	bandwidth_t bandWidth;
	codingrate_t codingRate;
	u16 preamble;
	power_t power;
	u8 overCurrentProtection;
	lora_mode_t current_mode;

	/* LoRa Hardware defined */
	SPI_HandleTypeDef *spi;
	GPIO_TypeDef *cs_port;
	GPIO_TypeDef *rst_port;
	u16 cs_pin;
	u16 rst_pin;
	u16 dio0_pin;
	IRQn_Type dio0_irq_number;

	/* LoRa buffer and callback */
	u8 *rx_buf;
	u32 rx_buf_size;
	void (*receive_buf)(struct lora_dev *sx1278, u8 *buf, u32 size);
};

#pragma pack()

void lora_reset(struct lora_dev *sx1278);
u8 lora_read_reg(struct lora_dev *sx1278, u8 address);
HAL_StatusTypeDef lora_write_reg(struct lora_dev *sx1278, u8 address, u8 value);
HAL_StatusTypeDef lora_goto_mode(struct lora_dev *sx1278, lora_mode_t mode);
void lora_set_frequency(struct lora_dev *sx1278, u32 f);
void lora_set_spreading_factor(struct lora_dev *sx1278, SF_t SF);
void lora_set_power(struct lora_dev *sx1278, power_t power);
void lora_set_ocp(struct lora_dev *sx1278, u8 cur);
void lora_setTOMsb_setCRCon(struct lora_dev *sx1278);
void lora_setLowDaraRateOptimization(struct lora_dev *sx1278, u8 value);
void lora_setAutoLDO(struct lora_dev *sx1278);
void lora_set_sync_word(struct lora_dev *sx1278, u8 syncword);
void lora_set_bandwidth(struct lora_dev *sx1278, bandwidth_t BW);
void lora_set_coding_rate(struct lora_dev *sx1278, codingrate_t cdRate);
HAL_StatusTypeDef lora_burst_write(struct lora_dev *sx1278, u8 address, u8 *value, u32 len);
status_t lora_is_valid(struct lora_dev *sx1278);
HAL_StatusTypeDef lora_cad_check(struct lora_dev *sx1278, u32 timeout);
HAL_StatusTypeDef lora_transmit(struct lora_dev *sx1278, u8 *data, u8 length, u16 timeout);
HAL_StatusTypeDef lora_start_receiving(struct lora_dev *sx1278);
u8 lora_receive(struct lora_dev *sx1278, u8 *data, u8 length);
u8 lora_get_rssi(struct lora_dev *sx1278);
lora_err_t lora_init(struct lora_dev *sx1278);
u8 lora_rx_callback(struct lora_dev *sx1278);

#endif
