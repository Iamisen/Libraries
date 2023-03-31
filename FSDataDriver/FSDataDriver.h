#pragma once
#ifndef FS_DATA_DRIVER_H
#define FS_DATA_DRIVER_H

#include <Arduino.h>
#include <ESP32DMASPISlave.h>
#include <HardwareSerial.h>
// THIS FILE NEEDS TO BE PLACED IN THE DIRECTORY OF EVERY SINGLE PROJECT THAT USES IT
// FOR ALL INTENTS AND PURPOSES, THE REAL FILE IS IN THE FSDataDriver DIRECTORY
namespace FSDataDriver
{
    // Documentation can be found here:
    // https://github.com/hideakitai/ESP32DMASPI
    // https://github.com/hideakitai/ESP32SPISlave
    ESP32DMASPI::Slave slave;
    const int UART_CHANNEL = 1;
    HardwareSerial SerialPort(UART_CHANNEL);
    typedef uint8_t byte;

    static constexpr uint32_t BUFFER_SIZE{32};
    uint8_t spi_slave_tx_buf[BUFFER_SIZE];
    uint8_t spi_slave_rx_buf[BUFFER_SIZE];

    // Definitions for the communication protocol of the FS
    static const byte initialPacketPadding[5] = {0xff, 0xff, 0xff, 0xff, 0xff};
    static const byte finalPacketPadding[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    static const byte fireCommandPacket[4] = {142, 185, 73, 37};
    static const byte fillCommandPacket[4] = {29, 19, 36, 76};
    static const byte purgeCommandPacket[4] = {172, 13, 135, 56};
    static const byte abortCommandPacket[4] = {6, 254, 163, 13};
    static const byte standbyCommandPacket[4] = {171, 179, 216, 159};
    static const byte keepCommandPacket[4] = {55, 183, 135, 14};
    static const byte calibrateCommandPacket[4] = {191, 90, 204, 107};
    static const byte scientificPacketIdentifier = 202;
    static const byte mainModulePacketIdentifier = 101;
    static const byte commandPacketIdentifier = 55;
    static const int SPI_PACKET_SIZE = 32;

    struct ScientificDataPacket
    {
        int64_t time;
        int32_t oxTankPressure;
        int32_t ccPressure;
        int32_t oxidizerTankTransducerValue;
        int32_t combustionChamberTransducerValue;
        byte timeSinceLastCalibration;
        byte timeSinceLastStartup;
        void dumpPacket(byte *buffer);
    };

    struct MainModuleDataPacket
    {
        int64_t time;
        byte stateByte;
        byte actuatorStatusByte;
        byte timeSinceLastStartup;
        void dumpPacket(byte *buffer);
    };

    void ScientificDataPacket::dumpPacket(byte *buffer)
    {
        buffer[0] = scientificPacketIdentifier;
        memcpy(buffer + 9, &oxTankPressure, 4);
        memcpy(buffer + 13, &ccPressure, 4);
        memcpy(buffer + 17, &oxidizerTankTransducerValue, 4);
        memcpy(buffer + 21, &combustionChamberTransducerValue, 4);
        memcpy(buffer + 25, &timeSinceLastCalibration, 1);
        memcpy(buffer + 26, &timeSinceLastStartup, 1);
        memcpy(buffer + 1, &time, 8);
        // Padding to get to 32 bytes
        memcpy(buffer + 27, finalPacketPadding, 5);
    }

    void MainModuleDataPacket::dumpPacket(byte *buffer)
    {
        buffer[0] = mainModulePacketIdentifier;
        memcpy(buffer + 1, &stateByte, 1);
        memcpy(buffer + 2, &actuatorStatusByte, 1);
        memcpy(buffer + 3, &timeSinceLastStartup, 1);
        memcpy(buffer + 4, &time, 8);
        // Padding to get to 32 bytes
        memcpy(buffer + 12, finalPacketPadding, 20);
    }

    bool doPacketsMatch(const byte pack1[], const byte pack2[])
    {
        for (int i = 0; i < 4; i++)
        {
            if (pack1[i] != pack2[i])
            {
                return false;
            }
        }
        return true;
    }

    String parseCommandPacket(const byte *packet)
    {
        byte commandBytes[4];
        memcpy(commandBytes, packet + 1, 4);

        if (doPacketsMatch(commandBytes, fireCommandPacket))
        {
            return "fire";
        }
        else if (doPacketsMatch(commandBytes, fillCommandPacket))
        {
            return "fill";
        }
        else if (doPacketsMatch(commandBytes, purgeCommandPacket))
        {
            return "purge";
        }
        else if (doPacketsMatch(commandBytes, abortCommandPacket))
        {
            return "abort";
        }
        else if (doPacketsMatch(commandBytes, standbyCommandPacket))
        {
            return "standby";
        }
        else if (doPacketsMatch(commandBytes, keepCommandPacket))
        {
            return "keep";
        }
        else if (doPacketsMatch(commandBytes, calibrateCommandPacket))
        {
            return "calibrate";
        }
        return "null";
    }

    // FOR USE ONLY ON THE SCIENTIFIC MODULE
    class SPIManager
    {
    public:
        SPIManager()
        {
            // to use DMA buffer, use these methods to allocate buffer
            spi_slave_tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
            spi_slave_rx_buf = slave.allocDMABuffer(BUFFER_SIZE);

            // set buffer data...

            // slave device configuration
            slave.setDataMode(SPI_MODE0);
            slave.setMaxTransferSize(BUFFER_SIZE);

            // begin() after setting
            slave.begin(); // HSPI = CS: 15, CLK: 14, MOSI: 13, MISO: 12 -> default
                           // VSPI (CS:  5, CLK: 18, MOSI: 23, MISO: 19)
        }

        // Function to send data, make sure to call it every loop to make sure you have updated data
        void update(ScientificDataPacket &dataPacket)
        {
            // Write to buffer
            dataPacket.dumpPacket(spi_slave_tx_buf);
            // if there is no transaction in queue, add transaction
            if (slave.remained() == 0)
            {
                slave.queue(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);
            }

            // if transaction has completed from master,
            // available() returns size of results of transaction,
            // and buffer is automatically updated
            while (slave.available())
            {
                // do something with received data: spi_slave_rx_buf
                latestCommand = parseCommandPacket(spi_slave_rx_buf);
                slave.pop();
            }
        }

        String getCommand()
        {
            String command = latestCommand;
            // Turn the variable back to null so we don't get the same command twice
            latestCommand = "null";
            return command;
        }

    private:
        String latestCommand;
        static const uint32_t BUFFER_SIZE = SPI_PACKET_SIZE;
        uint8_t *spi_slave_tx_buf;
        uint8_t *spi_slave_rx_buf;
    };

    // FOR USE ONLY ON THE MAIN MODULE
    class SerialManager
    {
    public:
        SerialManager(int baudRate, int rx_pin, int tx_pin)
        {
            SerialPort.begin(baudRate, SERIAL_8N1, rx_pin, tx_pin);
        }

        void sendData(MainModuleDataPacket &packet)
        {
            byte buffer[SPI_PACKET_SIZE];
            packet.dumpPacket(buffer);
            SerialPort.write(buffer, SPI_PACKET_SIZE);
        }

        // Unclear if this will work as intended but let's see
        // Needs testing
        String getCommand()
        {
            byte buffer[SPI_PACKET_SIZE];
            String command = "null";

            while (SerialPort.available())
            {
                // For each packet
                int i = 0;
                while (i < SPI_PACKET_SIZE)
                {
                    buffer[i] = SerialPort.read();
                    i++;
                }
                if (i == SPI_PACKET_SIZE)
                {
                    command = parseCommandPacket(buffer);
                }
            }
            latestCommand = command;
            return command;
        }

    private:
        String latestCommand;
        static const uint32_t BUFFER_SIZE = SPI_PACKET_SIZE;
    };
}
#endif // FS_DATA_DRIVER_H
