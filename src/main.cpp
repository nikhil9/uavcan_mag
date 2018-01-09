/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <board.hpp>
#include <chip.h>
#include <uavcan_lpc11c24/uavcan_lpc11c24.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include <uavcan/protocol/logger.hpp>
#include <uavcan/equipment/air_data/RawAirData.hpp>
#include <uavcan/equipment/ahrs/MagneticFieldStrength.hpp>
#include <math.h>
/*
 * GCC 4.9 cannot generate a working binary with higher optimization levels, although
 *         rest of the firmware can be compiled with -Os.
 * GCC 4.8 and earlier don't work at all on this firmware.
 */
#if __GNUC__
# pragma GCC optimize 1
#endif


#define DEFAULT_I2C          I2C0
I2C_ID_T i2cDev = DEFAULT_I2C;  /* Currently active I2C device */

#define COMPASS_ADDRESS      0x1E
#define ConfigRegA           0x00
#define ConfigRegB           0x01
#define magGain              0x20
#define PositiveBiasConfig   0x11
#define NegativeBiasConfig   0x12
#define NormalOperation      0x10
#define ModeRegister         0x02
#define ContinuousConversion 0x00
#define SingleConversion     0x01

// ConfigRegA valid sample averaging for 5883L
#define SampleAveraging_1    0x00
#define SampleAveraging_2    0x01
#define SampleAveraging_4    0x02
#define SampleAveraging_8    0x03

// ConfigRegA valid data output rates for 5883L
#define DataOutputRate_0_75HZ 0x00
#define DataOutputRate_1_5HZ  0x01
#define DataOutputRate_3HZ    0x02
#define DataOutputRate_7_5HZ  0x03
#define DataOutputRate_15HZ   0x04
#define DataOutputRate_30HZ   0x05
#define DataOutputRate_75HZ   0x06


/**
 * This function re-defines the standard ::rand(), which is used by the class uavcan::DynamicNodeIDClient.
 * Redefinition is normally not needed, but GCC 4.9 tends to generate broken binaries if it is not redefined.
 */
int rand()
{
    static int x = 1;
    x = x * 48271 % 2147483647;
    return x;
}

float field[3];

namespace
{

static constexpr unsigned NodeMemoryPoolSize = 2800;



uavcan::Node<NodeMemoryPoolSize>& getNode()
{
    static uavcan::Node<NodeMemoryPoolSize> node(uavcan_lpc11c24::CanDriver::instance(),
                                                 uavcan_lpc11c24::SystemClock::instance());
    return node;
}

uavcan::GlobalTimeSyncSlave& getTimeSyncSlave()
{
    static uavcan::GlobalTimeSyncSlave tss(getNode());
    return tss;
}

uavcan::Logger& getLogger()
{
    static uavcan::Logger logger(getNode());
    return logger;
}

void init()
{
    board::resetWatchdog();

    board::setErrorLed(false);
    board::setStatusLed(true);

    /*
     * Configuring the clock - this must be done before the CAN controller is initialized
     */
    uavcan_lpc11c24::clock::init();
    /*
     * Configuring the CAN controller
     */
    std::uint32_t bit_rate = 0;
    while (bit_rate == 0)
    {
        bit_rate = uavcan_lpc11c24::CanDriver::detectBitRate(&board::resetWatchdog);
    }


    if (uavcan_lpc11c24::CanDriver::instance().init(bit_rate) < 0)
    {
        board::die();
    }


    board::resetWatchdog();

    /*
     * Configuring the node
     */
    getNode().setName("org.uavcan.lpc11c24_test");

    uavcan::protocol::SoftwareVersion swver;
    swver.major = FW_VERSION_MAJOR;
    swver.minor = FW_VERSION_MINOR;
    swver.vcs_commit = GIT_HASH;
    swver.optional_field_flags = swver.OPTIONAL_FIELD_FLAG_VCS_COMMIT;
    getNode().setSoftwareVersion(swver);

    uavcan::protocol::HardwareVersion hwver;
    std::uint8_t uid[board::UniqueIDSize] = {};
    board::readUniqueID(uid);
    std::copy(std::begin(uid), std::end(uid), std::begin(hwver.unique_id));
    getNode().setHardwareVersion(hwver);

    board::resetWatchdog();

    /*
     * Starting the node and performing dynamic node ID allocation
     */
    if (getNode().start() < 0)
    {
        board::die();
    }


    getNode().setNodeID(21);

    board::resetWatchdog();

    /*
     * Initializing other libuavcan-related objects
     */
    if (getTimeSyncSlave().start() < 0)
    {
        board::die();
    }

    if (getLogger().init() < 0)
    {
        board::die();
    }

    getLogger().setLevel(uavcan::protocol::debug::LogLevel::DEBUG);

    board::resetWatchdog();
}

void register_write(uint8_t reg, uint8_t val){

	uint8_t addr = COMPASS_ADDRESS;
 	uint8_t txbuf[2];
 	txbuf[0] = reg;
 	txbuf[1] = val;
 	Chip_I2C_MasterSend(i2cDev, addr, txbuf, sizeof(txbuf));
}


const float GaussScale = 0.92e-03;
void get_mag_data(void){

	field[0] = (float)0;
	field[1] = (float)0;
	field[2] = (float)521;


	uint8_t addr = COMPASS_ADDRESS;
	uint8_t txbuf[2];
	txbuf[0] = ModeRegister;
	txbuf[1] = SingleConversion;
	Chip_I2C_MasterSend(i2cDev, addr, txbuf, sizeof(txbuf));

	txbuf[0] = 0x03;
	uint8_t buff[6];

	Chip_I2C_MasterCmdRead(i2cDev, addr, 0x03, buff, sizeof(buff));

	int16_t rx, ry, rz;
	rx = (((int16_t)buff[0]) << 8) | buff[1];
	rz = (((int16_t)buff[2]) << 8) | buff[3];
	ry = (((int16_t)buff[4]) << 8) | buff[5];

	field[0] = rx*GaussScale;
	field[1] = ry*GaussScale;
	field[2] = rz*GaussScale;
}

}


int main()
{
    init();
    getNode().setModeOperational();

    uavcan::MonotonicTime prev_log_at;
    uavcan::Publisher<uavcan::equipment::ahrs::MagneticFieldStrength>_uavcan_pub_raw_mag(getNode());

    _uavcan_pub_raw_mag.setPriority(6);

	uint8_t addr = COMPASS_ADDRESS;
	uint8_t txbuf[2];
	txbuf[0] = ConfigRegA;
	txbuf[1] = SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation;

	Chip_I2C_MasterSend(i2cDev, addr, txbuf, sizeof(txbuf));

    while (true)
    {
        getNode().spin(uavcan::MonotonicDuration::fromMSec(10));

        get_mag_data();

        uavcan::equipment::ahrs::MagneticFieldStrength mag;
        std::copy(field, field + 3, mag.magnetic_field_ga.begin());
        (void)_uavcan_pub_raw_mag.broadcast(mag);

        board::resetWatchdog();
    }
}
