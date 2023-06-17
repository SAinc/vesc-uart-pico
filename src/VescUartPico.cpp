#include "VescUartPico.h"

// no debug port
VescUartPico::VescUartPico(uint32_t timeout_ms, uart_inst_t *serialPort, 
				uint baud, uint pinTX, uint pinRX) : _TIMEOUT(timeout_ms) {
	uart_init(serialPort, baud);	// init serial port
	gpio_set_function(pinTX, GPIO_FUNC_UART);	// init pins
	gpio_set_function(pinRX, GPIO_FUNC_UART);

	uartPort = serialPort;
	debug = false;
}

// debug port
VescUartPico::VescUartPico(uint32_t timeout_ms, uart_inst_t *serialPort, 
				uart_inst_t *debugPort, uint baud, uint pinTX, uint pinRX, uint debugTx, uint debugRx) : _TIMEOUT(timeout_ms) {
	uart_init(serialPort, baud);	// init serial port
	gpio_set_function(pinTX, GPIO_FUNC_UART);	// init pins
	gpio_set_function(pinRX, GPIO_FUNC_UART);

	uartPort = serialPort;
	stdio_uart_init_full(debugPort, 115200, debugTx, debugRx);
	debug = true;
}

bool VescUartPico::getFWversion(void) {
    return getFWversion(0);
}

bool VescUartPico::getFWversion(uint8_t canId) {
    int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_FW_VERSION };

	packSendPayload(payload, payloadSize);

	uint8_t message[256];
	int messageLength = receivePayload(message);
	if (messageLength > 0) { 
		return processPayload(message); 
	}
	return false;
}

bool VescUartPico::getVescTelemetry(void) {
    return getVescTelemetry(0);
}

bool VescUartPico::getVescTelemetry(uint8_t canId) {
	if (debug){
		printf("Command: COMM_GET_VALUES %d", canId);
	}

    int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_GET_VALUES };

	packSendPayload(payload, payloadSize);

	uint8_t message[256];
	int messageLength = receivePayload(message);

	if (messageLength > 55) {
		return processPayload(message); 
	}
	return false;
}

void VescUartPico::setMotorCurrent(float current) {
    setMotorCurrent(current, 0);
}

void VescUartPico::setMotorCurrent(float current, uint8_t canId) {
    int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_SET_CURRENT };
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);
	packSendPayload(payload, payloadSize);
}

void VescUartPico::setBrakingCurrent(float brakingCurrent) {
    setBrakingCurrent(brakingCurrent, 0);
}

void VescUartPico::setBrakingCurrent(float brakingCurrent, uint8_t canId) {
    int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}

	payload[index++] = { COMM_SET_CURRENT_BRAKE };
	buffer_append_int32(payload, (int32_t)(brakingCurrent * 1000), &index);

	packSendPayload(payload, payloadSize);
}

void VescUartPico::set_eRPM(float eRPM) {
    set_eRPM(eRPM, 0);
}

void VescUartPico::set_eRPM(float eRPM, uint8_t canId) {
    int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_SET_RPM };
	buffer_append_int32(payload, (int32_t)(eRPM), &index);
	packSendPayload(payload, payloadSize);
}

void VescUartPico::setDuty(float duty) {
    setDuty(duty, 0);
}

void VescUartPico::setDuty(float duty, uint8_t canId) {
    int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_SET_DUTY };
	buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

	packSendPayload(payload, payloadSize);
}

void VescUartPico::sendKeepalive(void) {
    sendKeepalive(0);
}

void VescUartPico::sendKeepalive(uint8_t canId) {
    int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_ALIVE };
	packSendPayload(payload, payloadSize);
}

int VescUartPico::packSendPayload(uint8_t *payload, int length) {
    uint16_t crcPayload = crc16(payload, length);
	int count = 0;
	uint8_t messageSend[256];
	
	if (length <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = length;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(length >> 8);
		messageSend[count++] = (uint8_t)(length & 0xFF);
	}

	memcpy(messageSend + count, payload, length);
	count += length;

	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	// messageSend[count] = NULL;
	
	if(debug){
		printf("Package to send: "); serialPrint(messageSend, count);
	}

	// Sending package
	if(uart_is_enabled(uartPort))
		uart_write_blocking(uartPort, messageSend, count);

	// Returns number of send bytes
	return count;
}

int VescUartPico::receivePayload(uint8_t *payload) {
    // Messages <= 255 starts with "2", 2nd byte is length
	// Messages > 255 starts with "3" 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

	// Makes no sense to run this function if no serialPort is defined.
	if (uart_is_enabled(uartPort))
		return -1;

	uint16_t counter = 0;
	uint16_t endMessage = 256;
	bool messageRead = false;
	uint8_t messageReceived[256];
	uint16_t lenPayload = 0;
	
	absolute_time_t timeout = make_timeout_time_ms(_TIMEOUT); // Defining the timestamp for timeout (100ms before timeout)

	while ( absolute_time_diff_us(get_absolute_time(), timeout) > 0 && messageRead == false) {

		while (uart_is_readable(uartPort)) {

			counter++;
			uart_read_blocking(uartPort, messageReceived, 1);

			if (counter == 2) {

				switch (messageReceived[0])
				{
					case 2:
						endMessage = messageReceived[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
						lenPayload = messageReceived[1];
					break;

					case 3:
						// ToDo: Add Message Handling > 255 (starting with 3)
						if(debug){
							printf("Message is larger than 256 bytes - not supported\n");
						}
					break;

					default:
						if(debug){
							printf("Invalid start bit\n");
						}
					break;
				}
			}

			if (counter >= sizeof(messageReceived)) {
				break;
			}

			if (counter == endMessage && messageReceived[endMessage - 1] == 3) {
				messageReceived[endMessage] = 0;
				if (debug) {
					printf("End of message reached!\n");
				}
				messageRead = true;
				break; // Exit if end of message is reached, even if there is still more data in the buffer.
			}
		}
	}
	if(messageRead == false && debug) {
		printf("Timeout\n");
	}
	
	bool unpacked = false;

	if (messageRead) {
		unpacked = unpackPayload(messageReceived, endMessage, payload);
	}

	if (unpacked) {
		// Message was read
		return lenPayload; 
	}
	else {
		// No Message Read
		return 0;
	}
}

bool VescUartPico::unpackPayload(uint8_t *unpackedPayload, int length, uint8_t *payload) {
    uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;

	// Rebuild crc:
	crcMessage = unpackedPayload[length - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += unpackedPayload[length - 2];

	if(debug){
		printf("CRC received: %d", crcMessage);
	}

	// Extract payload:
	memcpy(payload, &unpackedPayload[2], unpackedPayload[1]);

	crcPayload = crc16(payload, unpackedPayload[1]);

	if(debug){
		printf("CRC calc: %d", crcPayload);
	}
	
	if (crcPayload == crcMessage) {
		if(debug) {
			printf("Received: "); 
			serialPrint(unpackedPayload, length); printf("\n");

			printf("Payload :      ");
			serialPrint(payload, unpackedPayload[1] - 1); printf("\n");
		}

		return true;
	}else{
		return false;
	}
}

bool VescUartPico::processPayload(uint8_t *payload) {
    COMM_PACKET_ID packetId;
	int32_t index = 0;

	packetId = (COMM_PACKET_ID)payload[0];
	payload++; // Removes the packetId from the actual message (payload)

	switch (packetId){
		case COMM_FW_VERSION: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

			fw_version.major = payload[index++];
			fw_version.minor = payload[index++];
			return true;
		case COMM_GET_VALUES: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

			data.tempMosfet 		= buffer_get_float16(payload, 10.0, &index); 	// 2 bytes - mc_interface_temp_fet_filtered()
			data.tempMotor 			= buffer_get_float16(payload, 10.0, &index); 	// 2 bytes - mc_interface_temp_motor_filtered()
			data.avgMotorCurrent 	= buffer_get_float32(payload, 100.0, &index); // 4 bytes - mc_interface_read_reset_avg_motor_current()
			data.avgInputCurrent 	= buffer_get_float32(payload, 100.0, &index); // 4 bytes - mc_interface_read_reset_avg_input_current()
			index += 4; // Skip 4 bytes - mc_interface_read_reset_avg_id()
			index += 4; // Skip 4 bytes - mc_interface_read_reset_avg_iq()
			data.dutyCycleNow 		= buffer_get_float16(payload, 1000.0, &index); 	// 2 bytes - mc_interface_get_duty_cycle_now()
			data.rpm 				= buffer_get_float32(payload, 1.0, &index);		// 4 bytes - mc_interface_get_rpm()
			data.inpVoltage 		= buffer_get_float16(payload, 10.0, &index);		// 2 bytes - GET_INPUT_VOLTAGE()
			data.ampHours 			= buffer_get_float32(payload, 10000.0, &index);	// 4 bytes - mc_interface_get_amp_hours(false)
			data.ampHoursCharged 	= buffer_get_float32(payload, 10000.0, &index);	// 4 bytes - mc_interface_get_amp_hours_charged(false)
			data.wattHours			= buffer_get_float32(payload, 10000.0, &index);	// 4 bytes - mc_interface_get_watt_hours(false)
			data.wattHoursCharged	= buffer_get_float32(payload, 10000.0, &index);	// 4 bytes - mc_interface_get_watt_hours_charged(false)
			data.tachometer 		= buffer_get_int32(payload, &index);				// 4 bytes - mc_interface_get_tachometer_value(false)
			data.tachometerAbs 		= buffer_get_int32(payload, &index);				// 4 bytes - mc_interface_get_tachometer_abs_value(false)
			data.error 				= (mc_fault_code)payload[index++];								// 1 byte  - mc_interface_get_fault()
			data.pidPos				= buffer_get_float32(payload, 1000000.0, &index);	// 4 bytes - mc_interface_get_pid_pos_now()
			data.id					= payload[index++];								// 1 byte  - app_get_configuration()->controller_id	

			return true;

		break;

		/* case COMM_GET_VALUES_SELECTIVE:

			uint32_t mask = 0xFFFFFFFF; */

		default:
			return false;
		break;
	}
}

void VescUartPico::serialPrint(uint8_t * data, int len) {
	if(debug){
		for (int i = 0; i <= len; i++)
		{
			printf("%d ", data[i]);
		}
		printf("\n");
	}
}