#ifndef _VESC_UART_PICO_H
#define _VESC_UART_PICO_H

#include "datatypes.h"
#include "buffer.h"
#include "crc.h"
#include <stdint.h>
#include <stdio.h>
#include <cstring>
#include "pico/stdlib.h"
#include "pico/time.h"

class VescUartPico {
    /** Struct to store the telemetry data returned by the VESC */
	struct dataPackage {
        float avgMotorCurrent;
        float avgInputCurrent;
        float dutyCycleNow;
        float rpm;
        float inpVoltage;
        float ampHours;
        float ampHoursCharged;
        float wattHours;
        float wattHoursCharged;
        long tachometer;
        long tachometerAbs;
        float tempMosfet;
        float tempMotor;
        float pidPos;
        uint8_t id;
        mc_fault_code error; 
	};

    struct FWversionPackage {
        uint8_t major;
        uint8_t minor;
    };

    const uint32_t _TIMEOUT;    // how long controller will wait for a response

    public:
        // CONSTRUCTOR
        // debugging disabled
        VescUartPico(uint32_t timeout_ms, uart_inst_t *serialPort, uint baud, uint pinTX, uint pinRX);
        // debugging enabled
        VescUartPico(uint32_t timeout_ms, uart_inst_t *serialPort, uart_inst_t *debug, uint baud, uint pinTX, uint pinRX, uint debugTx, uint debugRx);

        // FIELDS
        dataPackage data;   // field to hold returned telemetry
        FWversionPackage fw_version; // field to hold fw version
        
        // METHODS
        // get fw version from connected controller
        bool getFWversion(void);

        // get fw version from connected controller with CAN id
        bool getFWversion(uint8_t canId);

        // get telemetry values from connected controller
        bool getVescTelemetry(void);

        // get telemetry values from connected controller with CAN id
        bool getVescTelemetry(uint8_t canId);

        // set motor current of connected controller
        void setMotorCurrent(float current);

        // set motor current of connected controller with CAN id
        void setMotorCurrent(float current, uint8_t canId);

        // set braking current of connected controller
        void setBrakingCurrent(float brakingCurrent);

        // set braking current of connected controller with CAN id
        void setBrakingCurrent(float brakingCurrent, uint8_t canId);

        // set eRPM of motor of connected controller
        void set_eRPM(float eRPM);

        // set eRPM of motor of connected controller with CAN id
        void set_eRPM(float eRPM, uint8_t canId);

        // set duty cycle of motor of connected controller with CAN id
        void setDuty(float duty);

        // set duty cycle of connected motor of connected controller with CAN id
        void setDuty(float duty, uint8_t canId);

        // send keepalive packet to connected controller
        void sendKeepalive(void);

        // send keepalive packet to connected controller with CAN id
        void sendKeepalive(uint8_t canId);

    private:
        uart_inst_t *uartPort;  // uart instance
        bool debug;

        // packs and sends payload over serial
        int packSendPayload(uint8_t *payload, int length);

        // receive UART payload
        int receivePayload(uint8_t *payload);

        // verify and unpack payload
        bool unpackPayload(uint8_t *unpackedPayload, int length, uint8_t *payload);

        // process payload and extract data
        bool processPayload(uint8_t *payload);

        // helper for printing arrays
        void serialPrint(uint8_t * data, int len);
};

#endif