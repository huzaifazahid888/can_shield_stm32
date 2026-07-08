#include "CANSPI.h"
#include "MCP2515_CAN.h"

/** Local Function Prototypes */
static uint32_t convertReg2ExtendedCANid(uint8_t tempRXBn_EIDH,
		uint8_t tempRXBn_EIDL, uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL);
static uint32_t convertReg2StandardCANid(uint8_t tempRXBn_SIDH,
		uint8_t tempRXBn_SIDL);
static void convertCANid2Reg(uint32_t tempPassedInID, uint8_t canIdType,
		id_reg_t *passedIdReg);

/** Local Variables */
ctrl_status_t ctrlStatus;
ctrl_error_status_t errorStatus;
id_reg_t idReg;

/** CAN SPI APIs */

/* Entering Sleep Mode */
void CANSPI_Sleep(void) {

	/* Clear CAN bus wakeup interrupt */
	MCP2515_BitModify(MCP2515_CANINTF, 0x40, 0x00);
	MCP2515_BitModify(MCP2515_CANINTE, 0x40, 0x40);
	MCP2515_SetSleepMode();

}

/* Initialize CAN */
bool CANSPI_Initialize(uint32_t CrystalFreq, uint16_t baudrate, uint8_t mode) {

	RXF0 RXF0reg;
	RXF1 RXF1reg;
	RXF2 RXF2reg;
	RXF3 RXF3reg;
	RXF4 RXF4reg;
	RXF5 RXF5reg;
	RXM0 RXM0reg;
	RXM1 RXM1reg;

	/* Intialize Rx Mask values */
	RXM0reg.RXM0SIDH = 0x00;
	RXM0reg.RXM0SIDL = 0x00;
	RXM0reg.RXM0EID8 = 0x00;
	RXM0reg.RXM0EID0 = 0x00;

	RXM1reg.RXM1SIDH = 0x00;
	RXM1reg.RXM1SIDL = 0x00;
	RXM1reg.RXM1EID8 = 0x00;
	RXM1reg.RXM1EID0 = 0x00;

	/* Intialize Rx Filter values */
	RXF0reg.RXF0SIDH = 0x00; // This sets the high bits of the CAN filter (bits 10 to 3).
	RXF0reg.RXF0SIDL = 0x00; // This sets the low bits of the CAN filter (bits 2 to 0).
	RXF0reg.RXF0EID8 = 0x00;
	RXF0reg.RXF0EID0 = 0x00;

	RXF1reg.RXF1SIDH = 0x00;
	RXF1reg.RXF1SIDL = 0x08;    //Starndard Filter
	RXF1reg.RXF1EID8 = 0x00;
	RXF1reg.RXF1EID0 = 0x00;    //Exntended Filter

	RXF2reg.RXF2SIDH = 0x00;
	RXF2reg.RXF2SIDL = 0x00;
	RXF2reg.RXF2EID8 = 0x00;
	RXF2reg.RXF2EID0 = 0x00;

	RXF3reg.RXF3SIDH = 0x00;
	RXF3reg.RXF3SIDL = 0x00;
	RXF3reg.RXF3EID8 = 0x00;
	RXF3reg.RXF3EID0 = 0x00;

	RXF4reg.RXF4SIDH = 0x00;
	RXF4reg.RXF4SIDL = 0x00;
	RXF4reg.RXF4EID8 = 0x00;
	RXF4reg.RXF4EID0 = 0x00;

	RXF5reg.RXF5SIDH = 0x00;
	RXF5reg.RXF5SIDL = 0x08;
	RXF5reg.RXF5EID8 = 0x00;
	RXF5reg.RXF5EID0 = 0x00;

	/* Intialize MCP2515, check SPI */

	if (!MCP2515_Initialize()) {
		return false;
	}

	/* Change mode as configuration mode */

	if (!MCP2515_SetConfigMode()) {
		return false;
	}

	/* Configure filter & mask */
	MCP2515_WriteByteSequence(MCP2515_RXM0SIDH, MCP2515_RXM0EID0,
			&(RXM0reg.RXM0SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXM1SIDH, MCP2515_RXM1EID0,
			&(RXM1reg.RXM1SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF0SIDH, MCP2515_RXF0EID0,
			&(RXF0reg.RXF0SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF1SIDH, MCP2515_RXF1EID0,
			&(RXF1reg.RXF1SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF2SIDH, MCP2515_RXF2EID0,
			&(RXF2reg.RXF2SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF3SIDH, MCP2515_RXF3EID0,
			&(RXF3reg.RXF3SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF4SIDH, MCP2515_RXF4EID0,
			&(RXF4reg.RXF4SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF5SIDH, MCP2515_RXF5EID0,
			&(RXF5reg.RXF5SIDH));

	/* Accept All (Standard + Extended) */
	MCP2515_WriteByte(MCP2515_RXB0CTRL, 0x04);    //Enable BUKT, Accept Filter 0
	MCP2515_WriteByte(MCP2515_RXB1CTRL, 0x01);    //Accept Filter 1

	// Configure Baud Rate based on CrystalFreq
	if (CrystalFreq == MCP_8MHz) {
		switch (baudrate) {
		case 250:
			MCP2515_WriteByte(MCP2515_CNF1, 0x00);
			MCP2515_WriteByte(MCP2515_CNF2, 0xE5);
			MCP2515_WriteByte(MCP2515_CNF3, 0x83);
			break;
		case 500:
			MCP2515_WriteByte(MCP2515_CNF1, 0x00);
			MCP2515_WriteByte(MCP2515_CNF2, 0x83);
			MCP2515_WriteByte(MCP2515_CNF3, 0x01);
			break;
		case 1000:
			MCP2515_WriteByte(MCP2515_CNF1, 0x00);
			MCP2515_WriteByte(MCP2515_CNF2, 0x80);
			MCP2515_WriteByte(MCP2515_CNF3, 0x00);
			break;
		case 800:
			MCP2515_WriteByte(MCP2515_CNF1, 0x00);
			MCP2515_WriteByte(MCP2515_CNF2, 0x80);
			MCP2515_WriteByte(MCP2515_CNF3, 0x01);
			break;
		case 200:
			MCP2515_WriteByte(MCP2515_CNF1, 0x00);
			MCP2515_WriteByte(MCP2515_CNF2, 0xb4);
			MCP2515_WriteByte(MCP2515_CNF3, 0x06);
			break;
		case 100:
			MCP2515_WriteByte(MCP2515_CNF1, 0x00);
			MCP2515_WriteByte(MCP2515_CNF2, 0xb4);
			MCP2515_WriteByte(MCP2515_CNF3, 0x06);
			break;
		default:
			return false; // Unsupported baud rate
		}
	} else if (CrystalFreq == MCP_16MHz) {
		switch (baudrate) {
		case 250:
			MCP2515_WriteByte(MCP2515_CNF1, 0x41);
			MCP2515_WriteByte(MCP2515_CNF2, 0xF1);
			MCP2515_WriteByte(MCP2515_CNF3, 0x85);
			break;
		case 500:
			MCP2515_WriteByte(MCP2515_CNF1, 0x00);
			MCP2515_WriteByte(MCP2515_CNF2, 0xE5);
			MCP2515_WriteByte(MCP2515_CNF3, 0x83);
			break;
		case 1000:
			MCP2515_WriteByte(MCP2515_CNF1, 0x00);
			MCP2515_WriteByte(MCP2515_CNF2, 0xD0);
			MCP2515_WriteByte(MCP2515_CNF3, 0x82);
			break;
		case 800:
			MCP2515_WriteByte(MCP2515_CNF1, 0x40);
			MCP2515_WriteByte(MCP2515_CNF2, 0x92);
			MCP2515_WriteByte(MCP2515_CNF3, 0x02);
			break;
		case 200:
			MCP2515_WriteByte(MCP2515_CNF1, 0x01);
			MCP2515_WriteByte(MCP2515_CNF2, 0xFA);
			MCP2515_WriteByte(MCP2515_CNF3, 0x87);
			break;
		case 100:
			MCP2515_WriteByte(MCP2515_CNF1, 0x03);
			MCP2515_WriteByte(MCP2515_CNF2, 0xFA);
			MCP2515_WriteByte(MCP2515_CNF3, 0x87);
			break;
		default:
			return false; // Unsupported baud rate
		}
	} else {
		return false; // Unsupported Crystal Frequency
	}

	// Configure Mode
	if (mode == MODE_NORMAL) { // Normal Mode
		if (!MCP2515_SetNormalMode()) {
			return false;
		}
	} else if (mode == MODE_LISTENONLY) { // Listen-Only Mode
		if (!MCP2515_SetListenOnlyMode()) {
			return false;
		}
	} else {
		return false; // Unsupported Mode
	}
	return true;
}
/* Transmit CAN message */
uint8_t CANSPI_Transmit(uCAN_MSG *tempCanMsg) {
	uint8_t returnValue = 0;

	idReg.tempSIDH = 0;//standard id high
	idReg.tempSIDL = 0;
	idReg.tempEID8 = 0;//extended id high
	idReg.tempEID0 = 0;

	ctrlStatus.ctrl_status = MCP2515_ReadStatus();

	/* Finding empty buffer */
	if (ctrlStatus.TXB0REQ != 1) {
		/* convert CAN ID for register */
		convertCANid2Reg(tempCanMsg->frame.id, tempCanMsg->frame.idType,
				&idReg);

		/* Load data to Tx Buffer */
		MCP2515_LoadTxSequence(MCP2515_LOAD_TXB0SIDH, &(idReg.tempSIDH),
				tempCanMsg->frame.dlc, &(tempCanMsg->frame.data0));

		/* Request to transmit */
		MCP2515_RequestToSend(MCP2515_RTS_TX0);

		returnValue = 1;
	} else if (ctrlStatus.TXB1REQ != 1) {
		convertCANid2Reg(tempCanMsg->frame.id, tempCanMsg->frame.idType,
				&idReg);

		MCP2515_LoadTxSequence(MCP2515_LOAD_TXB1SIDH, &(idReg.tempSIDH),
				tempCanMsg->frame.dlc, &(tempCanMsg->frame.data0));
		MCP2515_RequestToSend(MCP2515_RTS_TX1);

		returnValue = 1;
	} else if (ctrlStatus.TXB2REQ != 1) {
		convertCANid2Reg(tempCanMsg->frame.id, tempCanMsg->frame.idType,
				&idReg);

		MCP2515_LoadTxSequence(MCP2515_LOAD_TXB2SIDH, &(idReg.tempSIDH),
				tempCanMsg->frame.dlc, &(tempCanMsg->frame.data0));
		MCP2515_RequestToSend(MCP2515_RTS_TX2);

		returnValue = 1;
	}

	return (returnValue);
}

/* Receive CAN message */
uint8_t CANSPI_Receive(uCAN_MSG *tempCanMsg) {
	uint8_t returnValue = 0;
	rx_reg_t rxReg;
	ctrl_rx_status_t rxStatus;

	rxStatus.ctrl_rx_status = MCP2515_GetRxStatus();

	/* Check receive buffer */
	if (rxStatus.rxBuffer != 0) {
		/* finding buffer which has a message */
		if ((rxStatus.rxBuffer == MSG_IN_RXB0)
				| (rxStatus.rxBuffer == MSG_IN_BOTH_BUFFERS)) {
			MCP2515_ReadRxSequence(MCP2515_READ_RXB0SIDH, rxReg.rx_reg_array,
					sizeof(rxReg.rx_reg_array));
		} else if (rxStatus.rxBuffer == MSG_IN_RXB1) {
			MCP2515_ReadRxSequence(MCP2515_READ_RXB1SIDH, rxReg.rx_reg_array,
					sizeof(rxReg.rx_reg_array));
		}

		/* if the message is extended CAN type */
		if (rxStatus.msgType == dEXTENDED_CAN_MSG_ID_2_0B) {
			tempCanMsg->frame.idType = (uint8_t) dEXTENDED_CAN_MSG_ID_2_0B;
			tempCanMsg->frame.id = convertReg2ExtendedCANid(rxReg.RXBnEID8,
					rxReg.RXBnEID0, rxReg.RXBnSIDH, rxReg.RXBnSIDL);
		} else {
			/* Standard type */
			tempCanMsg->frame.idType = (uint8_t) dSTANDARD_CAN_MSG_ID_2_0B;
			tempCanMsg->frame.id = convertReg2StandardCANid(rxReg.RXBnSIDH,
					rxReg.RXBnSIDL);
		}

		tempCanMsg->frame.dlc = rxReg.RXBnDLC;
		tempCanMsg->frame.data0 = rxReg.RXBnD0;
		tempCanMsg->frame.data1 = rxReg.RXBnD1;
		tempCanMsg->frame.data2 = rxReg.RXBnD2;
		tempCanMsg->frame.data3 = rxReg.RXBnD3;
		tempCanMsg->frame.data4 = rxReg.RXBnD4;
		tempCanMsg->frame.data5 = rxReg.RXBnD5;
		tempCanMsg->frame.data6 = rxReg.RXBnD6;
		tempCanMsg->frame.data7 = rxReg.RXBnD7;

		returnValue = 1;
	}

	return (returnValue);
}

/* check message buffer and return count */
uint8_t CANSPI_messagesInBuffer(void) {
	uint8_t messageCount = 0;

	ctrlStatus.ctrl_status = MCP2515_ReadStatus();

	if (ctrlStatus.RX0IF != 0) {
		messageCount++;
	}

	if (ctrlStatus.RX1IF != 0) {
		messageCount++;
	}

	return (messageCount);
}

/* check BUS off */
uint8_t CANSPI_isBussOff(void) {
	uint8_t returnValue = 0;

	errorStatus.error_flag_reg = MCP2515_ReadByte(MCP2515_EFLG);

	if (errorStatus.TXBO == 1) {
		returnValue = 1;
	}

	return (returnValue);
}

/* check Rx Passive Error */
uint8_t CANSPI_isRxErrorPassive(void) {
	uint8_t returnValue = 0;

	errorStatus.error_flag_reg = MCP2515_ReadByte(MCP2515_EFLG);

	if (errorStatus.RXEP == 1) {
		returnValue = 1;
	}

	return (returnValue);
}

/* check Tx Passive Error */
uint8_t CANSPI_isTxErrorPassive(void) {
	uint8_t returnValue = 0;

	errorStatus.error_flag_reg = MCP2515_ReadByte(MCP2515_EFLG);

	if (errorStatus.TXEP == 1) {
		returnValue = 1;
	}

	return (returnValue);
}

/* convert register value to extended CAN ID */
static uint32_t convertReg2ExtendedCANid(uint8_t tempRXBn_EIDH,
		uint8_t tempRXBn_EIDL, uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL) {
	uint32_t returnValue = 0;
	uint32_t ConvertedID = 0;
	uint8_t CAN_standardLo_ID_lo2bits;
	uint8_t CAN_standardLo_ID_hi3bits;

	CAN_standardLo_ID_lo2bits = (tempRXBn_SIDL & 0x03);
	CAN_standardLo_ID_hi3bits = (tempRXBn_SIDL >> 5);
	ConvertedID = (tempRXBn_SIDH << 3);
	ConvertedID = ConvertedID + CAN_standardLo_ID_hi3bits;
	ConvertedID = (ConvertedID << 2);
	ConvertedID = ConvertedID + CAN_standardLo_ID_lo2bits;
	ConvertedID = (ConvertedID << 8);
	ConvertedID = ConvertedID + tempRXBn_EIDH;
	ConvertedID = (ConvertedID << 8);
	ConvertedID = ConvertedID + tempRXBn_EIDL;
	returnValue = ConvertedID;
	return (returnValue);
}

/* convert register value to standard CAN ID */
static uint32_t convertReg2StandardCANid(uint8_t tempRXBn_SIDH,
		uint8_t tempRXBn_SIDL) {
	uint32_t returnValue = 0;
	uint32_t ConvertedID;

	ConvertedID = (tempRXBn_SIDH << 3);
	ConvertedID = ConvertedID + (tempRXBn_SIDL >> 5);
	returnValue = ConvertedID;

	return (returnValue);
}

/* convert CAN ID to register value */
static void convertCANid2Reg(uint32_t tempPassedInID, uint8_t canIdType,
		id_reg_t *passedIdReg) {
	uint8_t wipSIDL = 0;

	if (canIdType == dEXTENDED_CAN_MSG_ID_2_0B) {
		//EID0
		passedIdReg->tempEID0 = 0xFF & tempPassedInID;
		tempPassedInID = tempPassedInID >> 8;

		//EID8
		passedIdReg->tempEID8 = 0xFF & tempPassedInID;
		tempPassedInID = tempPassedInID >> 8;

		//SIDL
		wipSIDL = 0x03 & tempPassedInID;
		tempPassedInID = tempPassedInID << 3;
		wipSIDL = (0xE0 & tempPassedInID) + wipSIDL;
		wipSIDL = wipSIDL + 0x08;
		passedIdReg->tempSIDL = 0xEB & wipSIDL;

		//SIDH
		tempPassedInID = tempPassedInID >> 8;
		passedIdReg->tempSIDH = 0xFF & tempPassedInID;
	} else {
		passedIdReg->tempEID8 = 0;
		passedIdReg->tempEID0 = 0;
		tempPassedInID = tempPassedInID << 5;
		passedIdReg->tempSIDL = 0xFF & tempPassedInID;
		tempPassedInID = tempPassedInID >> 8;
		passedIdReg->tempSIDH = 0xFF & tempPassedInID;
	}
}
