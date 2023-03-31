#include <ESP32SPISlave.h>

ESP32SPISlave slave;
typedef uint8_t byte;

static constexpr uint32_t BUFFER_SIZE{32};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

static byte initialPacketPadding[5] = {0xff, 0xff, 0xff, 0xff, 0xff};
static byte finalPacketPadding[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
byte fireCommandPacket[4] = {142, 185, 73, 37};
byte fillCommandPacket[4] = {29, 19, 36, 76};
byte purgeCommandPacket[4] = {172, 13, 135, 56};
byte abortCommandPacket[4] = {6, 254, 163, 13};
byte standbyCommandPacket[4] = {171, 179, 216, 159};
byte keepCommandPacket[4] = {55, 183, 135, 14};
byte calibrateCommandPacket[4] = {191, 90, 204, 107};
struct ScientificDataPacket
{
    int64_t time;
    int32_t oxTankPressure;
    int32_t ccPressure;
    int32_t oxidizerTankTransducerValue;
    int32_t combustionChamberTransducerValue;
    byte timeSinceLastCalibration;
    byte timeSinceLastStartup;
};

struct MainModuleDataPacket
{
    int64_t time;
    byte stateByte;
    byte actuatorStatusByte;
    byte timeSinceLastStartup;
};

bool doPacketsMatch(byte pack1[], byte pack2[])
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

void parseBuffer(byte *buffer)
{
    size_t pointer = 4;
    String command = "";
    // TODO: Fix this
    while (pointer < BUFFER_SIZE)
    {
        if (buffer[pointer] == finalPacketPadding[4] && buffer[pointer - 1] == finalPacketPadding[3] && buffer[pointer - 2] == finalPacketPadding[2] && buffer[pointer - 3] == finalPacketPadding[1] && buffer[pointer - 4] == finalPacketPadding[0])
        {
            if (pointer < 14)
            {
                // The packet is too small to be valid
                break;
            }
            // We have a packet
            std::vector<byte> packet;
            packet.insert(packet.end(), buffer.begin(), buffer.begin() + pointer + 1);
            readAndSavePacket(packet);

            if (buffer[pointer] == finalPacketPadding[4] && buffer[pointer - 1] == finalPacketPadding[3] && buffer[pointer - 2] == finalPacketPadding[2] && buffer[pointer - 3] == finalPacketPadding[1])
            {
                if (doPacketsMatch(buffer, fireCommandPacket))
                {
                    command = "fire";
                }
                else if (doPacketsMatch(buffer, fillCommandPacket))
                {
                    command = "fill";
                }
                else if (doPacketsMatch(buffer, purgeCommandPacket))
                {
                    command = "purge";
                }
                else if (doPacketsMatch(buffer, abortCommandPacket))
                {
                    command = "abort";
                }
                else if (doPacketsMatch(buffer, standbyCommandPacket))
                {
                    command = "standby";
                }
                else if (doPacketsMatch(buffer, keepCommandPacket))
                {
                    command = "keep";
                }
                else if (doPacketsMatch(buffer, calibrateCommandPacket))
                {
                    command = "calibrate";
                }
            }
        }
        else
        {
            pointer++;
        }
    }
}

void setup()
{
    // HSPI = CS: 15, CLK: 14, MOSI: 13, MISO: 12
    // VSPI = CS:  5, CLK: 18, MOSI: 23, MISO: 19
    slave.setDataMode(SPI_MODE0);
    slave.begin(VSPI);
}

void loop()
{
    // if there is no transaction in queue, add transaction
    if (slave.remained() == 0)
        slave.queue(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);

    // if transaction has completed from master,
    // available() returns size of results of transaction,
    // and `spi_slave_rx_buf` is automatically updated
    while (slave.available())
    {
        // do something with `spi_slave_rx_buf`

        slave.pop();
    }
}