/*
 * MCP2515_CAN.c
 *
 *  Created on: Dec 12, 2024
 *  Author: Huzaifa
 */

#include "MCP2515_CAN.h"
#include "stdio.h"
//#include "gpio.h"

/* SPI related variables */

extern SPI_HandleTypeDef        hspi2;
#define SPI_CAN                 &hspi2
#define SPI_TIMEOUT             10

/* Global variables for CS pin */
static GPIO_TypeDef *CS_PORT;
static uint16_t CS_PIN;



/* Prototypes */
static void SPI_Tx(uint8_t data);
static void SPI_TxBuffer(uint8_t *buffer, uint8_t length);
static uint8_t SPI_Rx(void);
static void SPI_RxBuffer(uint8_t *buffer, uint8_t length);

void MCP2515_CS(GPIO_TypeDef *port, uint16_t pin)
{
	CS_PORT = port;
	CS_PIN = pin;
}

 void MCP2515_CS_LOW(void)
{
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
}

 void MCP2515_CS_HIGH(void)
{
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
}

/* initialize MCP2515 */
bool MCP2515_Initialize(void) {
	MCP2515_CS_HIGH();

	uint8_t loop = 10;

	do {
		/* check SPI Ready */
		if (HAL_SPI_GetState(SPI_CAN) == HAL_SPI_STATE_READY)
			return true;

		loop--;
	} while (loop > 0);

	return false;
}

bool MCP2515_SetConfigMode(void) {
	/* configure CANCTRL Register */
	MCP2515_WriteByte(MCP2515_CANCTRL, 0x80);  // Request config mode

	uint8_t loop = 10;
	uint8_t status;

	do {
		/* confirm mode configuration */
		status = MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0;
		printf("CANSTAT: 0x%02X\n", status);  // Print the current CANSTAT value

		if (status == 0x80)
			return true;

		loop--;
	} while (loop > 0);

	return false;
}

bool MCP2515_SetNormalMode(void) {
	/* configure CANCTRL Register */
	MCP2515_WriteByte(MCP2515_CANCTRL, 0x00);  // Request normal mode

	uint8_t loop = 10;
	uint8_t status;

	do {
		/* confirm mode configuration */
		status = MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0;
		printf("CANSTAT: 0x%02X\n", status);  // Print the current CANSTAT value

		if (status == 0x00)
			return true;

		loop--;
	} while (loop > 0);

	return false;
}

bool MCP2515_SetListenOnlyMode(void) {
	/* configure CANCTRL Register */
	MCP2515_WriteByte(MCP2515_CANCTRL, 0x60); // Request listen-only mode (0x60)

	uint8_t loop = 10;
	uint8_t status;

	do {
		/* confirm mode configuration */
		status = MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0;
		printf("CANSTAT: 0x%02X\n", status);  // Print the current CANSTAT value

		if (status == 0x60)  // Check for Listen-Only Mode
			return true;

		loop--;
	} while (loop > 0);

	return false;
}

/* Entering sleep mode */
bool MCP2515_SetSleepMode(void) {
	/* configure CANCTRL Register */
	MCP2515_WriteByte(MCP2515_CANCTRL, 0x20);

	uint8_t loop = 10;

	do {
		/* confirm mode configuration */
		if ((MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0) == 0x20)
			return true;

		loop--;
	} while (loop > 0);

	return false;
}

/* MCP2515 SPI-Reset */
void MCP2515_Reset(void) {
	MCP2515_CS_LOW();

	SPI_Tx(MCP2515_RESET);

	MCP2515_CS_HIGH();
}

/* read single byte */
uint8_t MCP2515_ReadByte(uint8_t address) {
	uint8_t retVal;

	MCP2515_CS_LOW();

	SPI_Tx(MCP2515_READ);
	SPI_Tx(address);
	retVal = SPI_Rx();

	MCP2515_CS_HIGH();

	return retVal;
}

/* read buffer */
void MCP2515_ReadRxSequence(uint8_t instruction, uint8_t *data, uint8_t length) {
	MCP2515_CS_LOW();

	SPI_Tx(instruction);
	SPI_RxBuffer(data, length);

	MCP2515_CS_HIGH();
}

/* write single byte */
void MCP2515_WriteByte(uint8_t address, uint8_t data) {
	MCP2515_CS_LOW();

	SPI_Tx(MCP2515_WRITE);
	SPI_Tx(address);
	SPI_Tx(data);

	MCP2515_CS_HIGH();
}

/* write buffer */
void MCP2515_WriteByteSequence(uint8_t startAddress, uint8_t endAddress,
		uint8_t *data) {
	MCP2515_CS_LOW();

	SPI_Tx(MCP2515_WRITE);
	SPI_Tx(startAddress);
	SPI_TxBuffer(data, (endAddress - startAddress + 1));

	MCP2515_CS_HIGH();
}

/* write to TxBuffer */
void MCP2515_LoadTxSequence(uint8_t instruction, uint8_t *idReg, uint8_t dlc,
		uint8_t *data) {
	MCP2515_CS_LOW();

	SPI_Tx(instruction);
	SPI_TxBuffer(idReg, 4);
	SPI_Tx(dlc);
	SPI_TxBuffer(data, dlc);

	MCP2515_CS_HIGH();
}

/* write to TxBuffer(1 byte) */
void MCP2515_LoadTxBuffer(uint8_t instruction, uint8_t data) {
	MCP2515_CS_LOW();

	SPI_Tx(instruction);
	SPI_Tx(data);

	MCP2515_CS_HIGH();
}

/* request to send */
void MCP2515_RequestToSend(uint8_t instruction) {
	MCP2515_CS_LOW();

	SPI_Tx(instruction);

	MCP2515_CS_HIGH();
}

/* read status */
uint8_t MCP2515_ReadStatus(void) {
	uint8_t retVal;

	MCP2515_CS_LOW();

	SPI_Tx(MCP2515_READ_STATUS);
	retVal = SPI_Rx();

	MCP2515_CS_HIGH();

	return retVal;
}

/* read RX STATUS register */
uint8_t MCP2515_GetRxStatus(void) {
	uint8_t retVal;

	MCP2515_CS_LOW();

	SPI_Tx(MCP2515_RX_STATUS);
	retVal = SPI_Rx();

	MCP2515_CS_HIGH();

	return retVal;
}

/* Use when changing register value */
void MCP2515_BitModify(uint8_t address, uint8_t mask, uint8_t data) {
	MCP2515_CS_LOW();

	SPI_Tx(MCP2515_BIT_MOD);
	SPI_Tx(address);
	SPI_Tx(mask);
	SPI_Tx(data);

	MCP2515_CS_HIGH();
}

/* SPI Tx wrapper function  */
static void SPI_Tx(uint8_t data) {
	HAL_SPI_Transmit(SPI_CAN, &data, 1, SPI_TIMEOUT);
}

/* SPI Tx wrapper function */
static void SPI_TxBuffer(uint8_t *buffer, uint8_t length) {
	HAL_SPI_Transmit(SPI_CAN, buffer, length, SPI_TIMEOUT);
}

/* SPI Rx wrapper function */
static uint8_t SPI_Rx(void) {
	uint8_t retVal;
	HAL_SPI_Receive(SPI_CAN, &retVal, 1, SPI_TIMEOUT);
	return retVal;
}

/* SPI Rx wrapper function */
static void SPI_RxBuffer(uint8_t *buffer, uint8_t length) {
	HAL_SPI_Receive(SPI_CAN, buffer, length, SPI_TIMEOUT);
}
