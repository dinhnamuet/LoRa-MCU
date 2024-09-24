#include "sx1278.h"
#include <string.h>
#include <stdlib.h>

static struct lora_dev *lora_head = NULL;
/**
  * @brief  Add a lora module into linker list
  * @param  sx1278 lora device instance
  * @retval -1: failed, 0: success
  */
static inline int lora_register(struct lora_dev *sx1278) {
    struct lora_dev *foo;

    if (!sx1278)
        return -1;

	if (!lora_head) {
        /* Init list head */
        lora_head = sx1278;
        lora_head->previous = NULL;
        lora_head->next = NULL;
    } else {
        foo = lora_head;
        while (foo->next) {
            foo = foo->next;
        }
        foo->next = sx1278;
        sx1278->previous = foo;
        sx1278->next = NULL;
    }
    return 0;
}
/**
  * @brief  Remove a lora module in linker list
  * @param  sx1278 lora device instance
  * @retval -1: failed, 0: success
  */
static inline int lora_delete(struct lora_dev *sx1278) {
    if (!sx1278)
		return -1;
	__disable_irq();
    if (sx1278 == lora_head) {
        lora_head = sx1278->next;
        if (lora_head)
			lora_head->previous = NULL;
    } else {
        if (sx1278->previous) {
            sx1278->previous->next = sx1278->next;
        }
        if (sx1278->next) {
            sx1278->next->previous = sx1278->previous;
        }
    }
	__enable_irq();
    lora_deinit(sx1278);
    return 0;
}

/**
  * @brief  Transmit an amount of data in blocking mode.
  * @param  sx1278 lora device instance
  * @param	buf data to write
  * @param	len data length
  * @retval HAL status
  */
static HAL_StatusTypeDef spi_lora_write(struct lora_dev *sx1278, u8 *buf, u32 len) {
	HAL_StatusTypeDef res;
	HAL_GPIO_WritePin(sx1278->cs_port, sx1278->cs_pin, GPIO_PIN_RESET);
	res =  HAL_SPI_Transmit(sx1278->spi, buf, len, SPI_TIMEOUT);
	HAL_GPIO_WritePin(sx1278->cs_port, sx1278->cs_pin, GPIO_PIN_SET);
	return res;
}
/**
  * @brief  Transmit then Receive an amount of data in blocking mode.
  * @param  sx1278 lora device instance
  * @param	tx_buf data to write
  * @param	tx_len data write size
  * @param	rx_buf buffer to get data
  * @param	rx_len data length
  * @retval HAL status
  */
static HAL_StatusTypeDef spi_lora_write_then_read(struct lora_dev *sx1278, u8 *tx_buf, u32 tx_len, u8 *rx_buf, u32 rx_len) {
	HAL_StatusTypeDef res;
	HAL_GPIO_WritePin(sx1278->cs_port, sx1278->cs_pin, GPIO_PIN_RESET);
	res = HAL_SPI_Transmit(sx1278->spi, tx_buf, tx_len, SPI_TIMEOUT);
	if (res != HAL_OK) {
		goto end_of_transmit;
	}
	res = HAL_SPI_Receive(sx1278->spi, rx_buf, rx_len, SPI_TIMEOUT);
end_of_transmit:
	HAL_GPIO_WritePin(sx1278->cs_port, sx1278->cs_pin, GPIO_PIN_SET);
	return res;
}
/**
  * @brief  Write sx1278 register.
  * @param  sx1278 lora device instance
  * @param	address register address
  * @param	value data to write
  * @retval HAL status
  */
HAL_StatusTypeDef lora_write_reg(struct lora_dev *sx1278, u8 address, u8 value) {
	u8 to_send[2];
	to_send[0] = address | (1 << 7);
	to_send[1] = value;
	return spi_lora_write(sx1278, to_send, 2);
}
/**
  * @brief  Read sx1278 register.
  * @param  sx1278 lora device instance
  * @param	address register address
  * @retval register value
  */
u8 lora_read_reg(struct lora_dev *sx1278, u8 address) {
	u8 readData;
	u8 addr;
	addr = address & ~(1 << 7);
	spi_lora_write_then_read(sx1278, &addr, 1, &readData, 1);
	return readData;
}
/**
  * @brief  Reset sx1278.
  * @param  sx1278 lora device instance
  * @retval none
  */
void lora_reset(struct lora_dev *sx1278) {
	HAL_GPIO_WritePin(sx1278->rst_port, sx1278->rst_pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(sx1278->rst_port, sx1278->rst_pin, GPIO_PIN_SET);
	HAL_Delay(100);
}
/**
  * @brief  Switch to lora mode.
  * @param  sx1278 lora device instance
  * @param	mode target mode
  * @retval HAL status
  */
HAL_StatusTypeDef lora_goto_mode(struct lora_dev *sx1278, lora_mode_t mode) {
	u8 read, data;
	read = lora_read_reg(sx1278, RegOpMode);
	switch (mode) {
		case SLEEP_MODE:
		case STANDBY_MODE:
		case TRANSMIT_MODE:
		case RXCONTINUOUS_MODE:
		case RXSINGLE_MODE:
		case CAD_MODE:
			data = (read & 0xF8) | mode;
			sx1278->current_mode = mode;
			break;
		default:
			return HAL_ERROR;
	}
	return lora_write_reg(sx1278, RegOpMode, data);
}
/**
  * @brief  sx1278 set frequency.
  * @param  sx1278 lora device instance
  * @param	f frequency
  * @retval none
  */
void lora_set_frequency(struct lora_dev *sx1278, u32 f) {
	u32_t Fr;
	Fr.val = ((u32)f * 524288) >> 5;

	lora_write_reg(sx1278, RegFrMsb, Fr.byte.byte_2);
	HAL_Delay(5);

	lora_write_reg(sx1278, RegFrMid, Fr.byte.byte_1);
	HAL_Delay(5);

	lora_write_reg(sx1278, RegFrLsb, Fr.byte.byte_0);
	HAL_Delay(5);
}
/**
  * @brief  sx1278 set spreading factor.
  * @param  sx1278 lora device instance
  * @param	SF spreading factor
  * @retval none
  */
void lora_set_spreading_factor(struct lora_dev *sx1278, SF_t SF) {
	u8 data;
	u8 read;

	if (SF > 12)
		SF = 12;
	if (SF < 7)
		SF = 7;

	read = lora_read_reg(sx1278, RegModemConfig2);
	HAL_Delay(10);

	data = (u8)((SF << 4) | (read & 0x0F));
	lora_write_reg(sx1278, RegModemConfig2, data);
	HAL_Delay(10);
	lora_setAutoLDO(sx1278);
}
/**
  * @brief  sx1278 set power.
  * @param  sx1278 lora device instance
  * @param	power power
  * @retval none
  */
void lora_set_power(struct lora_dev *sx1278, power_t power) {
	lora_write_reg(sx1278, RegPaConfig, power);
	HAL_Delay(10);
}
void lora_set_ocp(struct lora_dev *sx1278, u8 cur) {
	u8 OcpTrim = 0;
	if (cur < 45)
		cur = 45;
	if(cur > 240)
		cur = 240;
	if (cur <= 120)
		OcpTrim = (cur - 45) / 5;
	else if (cur <= 240)
		OcpTrim = (cur + 30) / 10;
	OcpTrim = OcpTrim | (1UL << 5);
	lora_write_reg(sx1278, RegOcp, OcpTrim);
	HAL_Delay(10);
}
void lora_setTOMsb_setCRCon(struct lora_dev *sx1278) {
	u8 read, data;
	read = lora_read_reg(sx1278, RegModemConfig2);
	data = read | 0x07;
	lora_write_reg(sx1278, RegModemConfig2, data);
	HAL_Delay(10);
}

void lora_setLowDaraRateOptimization(struct lora_dev *sx1278, u8 value) {
	u8 read, data;
	read = lora_read_reg(sx1278, RegModemConfig3);
	if (value)
		data = read | 0x08;
	else
		data = read & 0xF7;
	lora_write_reg(sx1278, RegModemConfig3, data);
	HAL_Delay(10);
}
void lora_setAutoLDO(struct lora_dev *sx1278) {
	double BW[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0, 500.0};

	lora_setLowDaraRateOptimization(sx1278, (long)((1 << sx1278->spreadingFactor) / ((double)BW[sx1278->bandWidth])) > 16.0);
}
/**
  * @brief  sx1278 set sync word.
  * @param  sx1278 lora device instance
  * @param	syncword
  * @retval none
  */
void lora_set_sync_word(struct lora_dev *sx1278, u8 syncword) {
	lora_write_reg(sx1278, RegSyncWord, syncword);
	HAL_Delay(10);
}
/**
  * @brief  sx1278 set bandwidth.
  * @param  sx1278 lora device instance
  * @param	BW bandwidth
  * @retval none
  */
void lora_set_bandwidth(struct lora_dev *sx1278, bandwidth_t BW) {
	u8 data;
	data = lora_read_reg(sx1278, RegModemConfig1);
	data &= 0x0F;
	data |= (u8)(BW << 4);
	lora_write_reg(sx1278, RegModemConfig1, data);
	lora_setAutoLDO(sx1278);
}
/**
  * @brief  sx1278 set coding rate.
  * @param  sx1278 lora device instance
  * @param	cdRate coding rate
  * @retval none
  */
void lora_set_coding_rate(struct lora_dev *sx1278, codingrate_t cdRate) {
	u8 data;
	data = lora_read_reg(sx1278, RegModemConfig1);
	data &= 0xF1;
	data |= (u8)(cdRate << 1);
	lora_write_reg(sx1278, RegModemConfig1, data);
}
/**
  * @brief  Write multiple data sx1278 register.
  * @param  sx1278 lora device instance
  * @param	address register address
  * @param	value data to write
  * @param	len data length
  * @retval HAL status
  */
HAL_StatusTypeDef lora_burst_write(struct lora_dev *sx1278, u8 address, u8 *value, u32 len) {
	HAL_StatusTypeDef res;
	u8 *to_send = (u8 *)calloc(len + 1, sizeof(u8));
	if (!to_send)
		return HAL_ERROR;
	to_send[0] = address | (1 << 7);
	memcpy(&to_send[1], value, len);
	res = spi_lora_write(sx1278, to_send, len + 1);
	free(to_send);
	return res;
}
/**
  * @brief  Check lora still working or not.
  * @param  sx1278 lora device instance
  * @retval LORA status
  */
status_t lora_is_valid(struct lora_dev *sx1278) {
	if (lora_read_reg(sx1278, RegVersion) == LORA_VERSION)
		return LORA_OK;
	else
		return LORA_NOT_FOUND;
}
/**
  * @brief  Channel Activity Detection checking.
  * @param  sx1278 lora device instance
  * @param	timeout time slice
  * @retval HAL status
  */
HAL_StatusTypeDef lora_cad_check(struct lora_dev *sx1278, u32 timeout) {
	lora_goto_mode(sx1278, STANDBY_MODE);
	HAL_Delay(5);
	/* Channel Activity Detection */
	lora_goto_mode(sx1278, CAD_MODE);
	while(!(lora_read_reg(sx1278, RegIrqFlags) & IRQ_CAD_DONE)) {
		if (--timeout == 0)
			return HAL_TIMEOUT; /* Cannot go to CAD Mode */
		HAL_Delay(1);
	}
	/* CAD detected */
	if(lora_read_reg(sx1278, RegIrqFlags) & IRQ_CAD_DETECTED) {
		lora_clear_irq(sx1278, IRQ_CAD_DETECTED | IRQ_CAD_DONE);
		return HAL_BUSY; /* Channel is busy */
	} else {
		lora_clear_irq(sx1278, IRQ_CAD_DONE);
		return HAL_OK; /* Channel is idle */
	}
}
/**
  * @brief  Sending lora data packet.
  * @param  sx1278 lora device instance
  * @param	data data to send
  * @param	length data length
  * @param	timeout time slice
  * @retval HAL status
  */
HAL_StatusTypeDef lora_transmit(struct lora_dev *sx1278, u8 *data, u8 length, u16 timeout) {
	u8 read;
	lora_mode_t mode = sx1278->current_mode;
	while (lora_cad_check(sx1278, timeout) != HAL_OK); /* Wait until channel is idle */
	/* send data */
//	lora_goto_mode(sx1278, STANDBY_MODE); /* After CAD, Automatic Mode change Standby Mode */
	read = lora_read_reg(sx1278, RegFiFoTxBaseAddr);
	lora_write_reg(sx1278, RegFiFoAddPtr, read);
	lora_write_reg(sx1278, RegPayloadLength, length);
	lora_burst_write(sx1278, RegFifo, data, length);
	lora_goto_mode(sx1278, TRANSMIT_MODE);
	while (1) {
		if (lora_read_reg(sx1278, RegIrqFlags) & IRQ_TX_DONE) {
			lora_clear_irq(sx1278, IRQ_TX_DONE);
			lora_goto_mode(sx1278, mode);
			return HAL_OK;
		} else {
			if (--timeout == 0) {
				lora_goto_mode(sx1278, mode);
				return HAL_TIMEOUT;
			}
		}
		HAL_Delay(1);
	}
}
/**
  * @brief  Switch to Receiving Mode.
  * @param  sx1278 lora device instance
  * @retval HAL status
  */
HAL_StatusTypeDef lora_start_receiving(struct lora_dev *sx1278) {
	return lora_goto_mode(sx1278, RXCONTINUOUS_MODE);
}
/**
  * @brief  Receive lora data packet.
  * @param  sx1278 lora device instance
  * @param	data buffer to get data
  * @param	length data length
  * @retval data length already received
  */
u8 lora_receive(struct lora_dev *sx1278, u8 *data, u8 length) {
	u8 i = 0;
	u8 read;
	u8 data_len;
	u8 min = 0;
	lora_goto_mode(sx1278, STANDBY_MODE);
	if (lora_read_reg(sx1278, RegIrqFlags) & IRQ_RX_DONE) {
		lora_clear_irq(sx1278, IRQ_RX_DONE);
		data_len = lora_read_reg(sx1278, RegRxNbBytes);
		read = lora_read_reg(sx1278, RegFiFoRxCurrentAddr);
		lora_write_reg(sx1278, RegFiFoAddPtr, read);
		min = (length >= data_len) ? data_len : length;
		for (i = 0; i < min; i++)
			data[i] = lora_read_reg(sx1278, RegFifo);
	}
	lora_start_receiving(sx1278);
	return min;
}
/**
  * @brief  Get LoRa RSSI Value of last packet.
  * @param  sx1278 lora device instance
  * @retval rssi value
  */
int lora_get_rssi(struct lora_dev *sx1278) {
	return lora_read_reg(sx1278, RegPktRssiValue) - 164;
}
/**
  * @brief  Lora deinitialize.
  * @param  sx1278 lora device instance
  * @retval none
  */
void lora_deinit(struct lora_dev *sx1278) {
	lora_goto_mode(sx1278, SLEEP_MODE);
	HAL_SPI_DeInit(sx1278->spi);
	HAL_NVIC_DisableIRQ(sx1278->dio0_irq_number);
	HAL_GPIO_DeInit(sx1278->cs_port, sx1278->cs_pin);
	HAL_GPIO_DeInit(sx1278->rst_port, sx1278->rst_pin);
	free(sx1278->rx_buf);
}
/*
* Brief: LoRa Interrupt Service Rountine
*/
static void lora_irq_handle(void) {
	u8 size;
	struct lora_dev *foo = lora_head;

	while (foo) {
		if (__HAL_GPIO_EXTI_GET_IT(foo->dio0_pin) != RESET) {
			__HAL_GPIO_EXTI_CLEAR_IT(foo->dio0_pin);
			if (foo->rx_buf) {
				memset(foo->rx_buf, 0, foo->rx_buf_size);
				size = lora_receive(foo, foo->rx_buf, foo->rx_buf_size);
				if (size && foo->receive_buf) {
					foo->receive_buf(foo, foo->rx_buf, size);
				}
			}
			return;
		} else {
			foo = foo->next;
		}
	}
}
/**
  * @brief  Lora initialize.
  * @param  sx1278 lora device instance
  * @retval lora error code
  */
lora_err_t lora_init(struct lora_dev *sx1278) {
	u8 data;

	NVIC_DisableIRQ(sx1278->dio0_irq_number);
	NVIC_SetVector(sx1278->dio0_irq_number, lora_irq_handle);

	HAL_GPIO_WritePin(sx1278->rst_port, sx1278->rst_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(sx1278->cs_port, sx1278->cs_pin, GPIO_PIN_SET);

	sx1278->rx_buf = (u8 *)calloc(sx1278->rx_buf_size, sizeof(u8));
	if (!sx1278->rx_buf) {
		return LORA_ENOMEM;
	}

	lora_reset(sx1278);

	if (lora_is_valid(sx1278) == LORA_OK) {
		// goto sleep mode:
		lora_goto_mode(sx1278, SLEEP_MODE);
		HAL_Delay(10);

		// turn on lora mode:
		data = lora_read_reg(sx1278, RegOpMode) | 0x80;
		lora_write_reg(sx1278, RegOpMode, data);
		HAL_Delay(100);

		// set frequency:
		lora_set_frequency(sx1278, sx1278->frequency);

		// set output power gain:
		lora_set_power(sx1278, sx1278->power);

		// set over current protection:
		lora_set_ocp(sx1278, sx1278->overCurrentProtection);

		// set LNA gain:
		lora_write_reg(sx1278, RegLna, 0x23);

		// set spreading factor, CRC on, and Timeout Msb:
		lora_setTOMsb_setCRCon(sx1278);
		lora_set_spreading_factor(sx1278, sx1278->spreadingFactor);

		// set Timeout Lsb:
		lora_write_reg(sx1278, RegSymbTimeoutL, 0xFF);

		// set bandwidth, coding rate and expilicit mode:
		// 8 bit RegModemConfig --> | X | X | X | X | X | X | X | X |
		//       bits represent --> |   bandwidth   |     CR    |I/E|
		data = lora_read_reg(sx1278, RegModemConfig1) & 0x01;
		data |= (u8)((sx1278->bandWidth << 4) | (sx1278->codingRate << 1));
		lora_write_reg(sx1278, RegModemConfig1, data);
		lora_setAutoLDO(sx1278);

		// set preamble:
		lora_write_reg(sx1278, RegPreambleMsb, sx1278->preamble >> 8);
		lora_write_reg(sx1278, RegPreambleLsb, (u8)sx1278->preamble);

		// DIO mapping:   --> DIO: RxDone
		data = lora_read_reg(sx1278, RegDioMapping1) | 0x3F;
		lora_write_reg(sx1278, RegDioMapping1, data);

		// goto standby mode:
		lora_goto_mode(sx1278, STANDBY_MODE);
		HAL_Delay(10);

		if (lora_read_reg(sx1278, RegVersion) == LORA_VERSION) {
			NVIC_EnableIRQ(sx1278->dio0_irq_number);
			lora_register(sx1278);
			return LORA_SUCCESS;
		} else {
			free(sx1278->rx_buf);
			return LORA_EFAULT;
		}
	} else {
		free(sx1278->rx_buf);
		return LORA_EFAULT;
	}
}
