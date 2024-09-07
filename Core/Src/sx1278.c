#include "sx1278.h"
#include <string.h>
#include <stdlib.h>

static HAL_StatusTypeDef spi_lora_write(struct lora_dev *sx1278, u8 *buf, u32 len) {
	HAL_StatusTypeDef res;
	HAL_GPIO_WritePin(sx1278->cs_port, sx1278->cs_pin, GPIO_PIN_RESET);
	res =  HAL_SPI_Transmit(sx1278->spi, buf, len, SPI_TIMEOUT);
	HAL_GPIO_WritePin(sx1278->cs_port, sx1278->cs_pin, GPIO_PIN_SET);
	return res;
}
static HAL_StatusTypeDef spi_lora_write_then_read(struct lora_dev *sx1278, u8 *tx_buf, u32 tx_len, u8 *rx_buf, u32 rx_len) {
	HAL_StatusTypeDef res;
	HAL_GPIO_WritePin(sx1278->cs_port, sx1278->cs_pin, GPIO_PIN_RESET);
	res = HAL_SPI_Transmit(sx1278->spi, tx_buf, tx_len, SPI_TIMEOUT);
	if (res != HAL_OK) {
		goto end_of_transmit;
	}
	while (HAL_SPI_GetState(sx1278->spi) != HAL_SPI_STATE_READY);
	res = HAL_SPI_Receive(sx1278->spi, rx_buf, rx_len, SPI_TIMEOUT);
	while (HAL_SPI_GetState(sx1278->spi) != HAL_SPI_STATE_READY);
end_of_transmit:
	HAL_GPIO_WritePin(sx1278->cs_port, sx1278->cs_pin, GPIO_PIN_SET);
	return res;
}
HAL_StatusTypeDef lora_write_reg(struct lora_dev *sx1278, u8 address, u8 value) {
	u8 to_send[2];
	to_send[0] = address | (1 << 7);
	to_send[1] = value;
	return spi_lora_write(sx1278, to_send, 2);
}
u8 lora_read_reg(struct lora_dev *sx1278, u8 address) {
	u8 readData;
	u8 addr;
	addr = address & ~(1 << 7);
	spi_lora_write_then_read(sx1278, &addr, 1, &readData, 1);
	return readData;
}
void lora_reset(struct lora_dev *sx1278) {
	HAL_GPIO_WritePin(sx1278->rst_port, sx1278->rst_pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(sx1278->rst_port, sx1278->rst_pin, GPIO_PIN_SET);
	HAL_Delay(100);
}
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
void lora_set_frequency(struct lora_dev *sx1278, int f) {
	u8 data;
	uint32_t Fr;
	Fr = ((uint32_t)f * 524288) >> 5;

	data = (u8)(Fr >> 16) & 0xFF;
	lora_write_reg(sx1278, RegFrMsb, data);
	HAL_Delay(5);

	data = (u8)(Fr >> 8) & 0xFF;
	lora_write_reg(sx1278, RegFrMid, data);
	HAL_Delay(5);

	data = (u8)Fr & 0xFF;
	lora_write_reg(sx1278, RegFrLsb, data);
	HAL_Delay(5);
}
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

void lora_set_sync_word(struct lora_dev *sx1278, u8 syncword) {
	lora_write_reg(sx1278, RegSyncWord, syncword);
	HAL_Delay(10);
}
void lora_set_bandwidth(struct lora_dev *sx1278, bandwidth_t BW) {
	u8 data;
	data = lora_read_reg(sx1278, RegModemConfig1);
	data &= 0x0F;
	data |= (u8)(BW << 4);
	lora_write_reg(sx1278, RegModemConfig1, data);
	lora_setAutoLDO(sx1278);
}
void lora_set_coding_rate(struct lora_dev *sx1278, codingrate_t cdRate) {
	u8 data;
	data = lora_read_reg(sx1278, RegModemConfig1);
	data &= 0xF1;
	data |= (u8)(cdRate << 1);
	lora_write_reg(sx1278, RegModemConfig1, data);
}
int lora_burst_write(struct lora_dev *sx1278, u8 address, u8 *value, u32 len) {
	u8 *to_send = (u8 *)calloc(len + 1, sizeof(u8));

	if (!to_send) {
		return -1;
	}

	to_send[0] = address | (1 << 7);
	memcpy(&to_send[1], value, len);
	spi_lora_write(sx1278, to_send, len);
	return 0;
}
status_t lora_is_valid(struct lora_dev *sx1278) {
	if (lora_read_reg(sx1278, RegVersion) == LORA_VERSION)
		return LORA_OK;
	else
		return LORA_NOT_FOUND;
}
u8 lora_transmit(struct lora_dev *sx1278, u8 *data, u8 length, u16 timeout) {
	u8 read;
	int mode = sx1278->current_mode;
	lora_goto_mode(sx1278, STANDBY_MODE);
	HAL_Delay(1);
	/* Channel Activity Detection */
	while(1) {
		lora_goto_mode(sx1278, CAD_MODE);
		while(!(lora_read_reg(sx1278, RegIrqFlags)>>2 & 0x01)) {
			HAL_Delay(1);
		}
		read = lora_read_reg(sx1278, RegIrqFlags);
		/* CAD detected */
		if(read & 0x01) {
			lora_write_reg(sx1278, RegIrqFlags, 0xFF);
		} else {
			lora_write_reg(sx1278, RegIrqFlags, 0xFF);
			break;
		}
		HAL_Delay(1);
	}
	/* send data */
	lora_goto_mode(sx1278, STANDBY_MODE);
	read = lora_read_reg(sx1278, RegFiFoTxBaseAddr);
	lora_write_reg(sx1278, RegFiFoAddPtr, read);
	lora_write_reg(sx1278, RegPayloadLength, length);
	lora_burst_write(sx1278, RegFifo, data, length);
	lora_goto_mode(sx1278, TRANSMIT_MODE);
	while (1) {
		read = lora_read_reg(sx1278, RegIrqFlags);
		if (read & (1UL << 3)) {
			lora_write_reg(sx1278, RegIrqFlags, 0xFF);
			lora_goto_mode(sx1278, mode);
			return 1;
		} else {
			if (--timeout == 0) {
				lora_goto_mode(sx1278, mode);
				return 0;
			}
		}
		HAL_Delay(1);
	}
}
HAL_StatusTypeDef lora_start_receiving(struct lora_dev *sx1278) {
	return lora_goto_mode(sx1278, RXCONTINUOUS_MODE);
}
u8 lora_receive(struct lora_dev *sx1278, u8 *data, u8 length) {
	u8 i = 0;
	u8 read;
	u8 data_len;
	u8 min = 0;
	lora_goto_mode(sx1278, STANDBY_MODE);
	read = lora_read_reg(sx1278, RegIrqFlags);
	if (read & (1UL << 6))
	{
		lora_write_reg(sx1278, RegIrqFlags, 0xFF);
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
u8 lora_get_rssi(struct lora_dev *sx1278) {
	u8 read;
	read = lora_read_reg(sx1278, RegPktRssiValue);
	return read;
}
lora_err_t lora_init(struct lora_dev *sx1278) {
	u8 data;
	u8 read;

	NVIC_DisableIRQ(sx1278->dio0_irq_number);

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
		read = lora_read_reg(sx1278, RegOpMode);
		HAL_Delay(10);

		data = read | 0x80;
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
		data = lora_read_reg(sx1278, RegModemConfig1);
		data &= 0x01;
		data |= (u8)((sx1278->bandWidth << 4) | (sx1278->codingRate << 1));
		lora_write_reg(sx1278, RegModemConfig1, data);
		lora_setAutoLDO(sx1278);

		// set preamble:
		lora_write_reg(sx1278, RegPreambleMsb, sx1278->preamble >> 8);
		lora_write_reg(sx1278, RegPreambleLsb, (u8)sx1278->preamble);

		// DIO mapping:   --> DIO: RxDone
		read = lora_read_reg(sx1278, RegDioMapping1);
		data = read | 0x3F;
		lora_write_reg(sx1278, RegDioMapping1, data);

		// goto standby mode:
		lora_goto_mode(sx1278, STANDBY_MODE);
		HAL_Delay(10);

		read = lora_read_reg(sx1278, RegVersion);
		if (read == LORA_VERSION) {
			NVIC_EnableIRQ(sx1278->dio0_irq_number);
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

u8 lora_rx_callback(struct lora_dev *sx1278) {
	u8 size = 0;
	if(__HAL_GPIO_EXTI_GET_IT(sx1278->dio0_pin) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(sx1278->dio0_pin);
		if (sx1278->rx_buf) {
			memset(sx1278->rx_buf, 0, sx1278->rx_buf_size);
			size = lora_receive(sx1278, sx1278->rx_buf, sx1278->rx_buf_size);
			if (size && sx1278->receive_buf) {
				sx1278->receive_buf(sx1278, sx1278->rx_buf, size);
			}
		}
	}
	return size;
}
