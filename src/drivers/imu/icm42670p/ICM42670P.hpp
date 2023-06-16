/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ICM42688P.hpp
 *
 * Driver for the Invensense ICM42688P connected via SPI.
 *
 */

#pragma once

#include "InvenSense_ICM42670P_registers.hpp"

#include <drivers/device/spi.h>
#include <ecl/geo/geo.h>
#include <drivers/drv_hrt.h>
#include <lib/conversion/rotation.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#define ICM42670_SENSOR_DEFAULT_RATE 1000

using namespace InvenSense_ICM42670P;

//bus options (extendable in the future)
enum ICM42670P_BUS {
    ICM42670P_BUS_ALL = 0,
    ICM42670P_BUS_SPI_INTERNAL,
    ICM42670P_BUS_SPI_EXTERNAL
};


class ICM42670P : public device::SPI, public px4::ScheduledWorkItem
{
public:
    ICM42670P(int bus, uint32_t device, enum Rotation rotation);
    virtual ~ICM42670P();

    virtual int init();

    void print_info();

protected:
    virtual int		probe();

private:
	// Sensor Configuration
	static constexpr float FIFO_SAMPLE_DT{1e6f / 1600.f};     // 1600 Hz accel & gyro ODR configured
	static constexpr float GYRO_RATE{1e6f / FIFO_SAMPLE_DT};
	static constexpr float ACCEL_RATE{1e6f / FIFO_SAMPLE_DT};

	// maximum FIFO samples per transfer is limited to the size of sensor_accel_fifo/sensor_gyro_fifo
    static constexpr int32_t FIFO_MAX_SAMPLES{FIFO::SIZE / sizeof(FIFO::DATA)};

	// Transfer data
	struct FIFOTransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::BANK_0::INT_STATUS) | DIR_READ};
		uint8_t INT_STATUS{0};
		uint8_t INT_STATUS2{0};
		uint8_t INT_STATUS3{0};
		uint8_t FIFO_COUNTH{0};
		uint8_t FIFO_COUNTL{0};
		FIFO::DATA f[FIFO_MAX_SAMPLES] {};
	};
	// ensure no struct padding
	static_assert(sizeof(FIFOTransferBuffer) == (6 + FIFO_MAX_SAMPLES *sizeof(FIFO::DATA)));

	struct register_bank0_config_t {

        //add a constructor here for list initialization
        register_bank0_config_t(Register::BANK_0 reg_input, uint8_t set_bits_input, uint8_t clear_bits_input):
            reg(reg_input), set_bits(set_bits_input), clear_bits(clear_bits_input){}

		Register::BANK_0 reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

    struct register_mreg1_config_t {

        //add a constructor here for list initialization
        register_mreg1_config_t(Register::MREG1 reg_input, uint8_t set_bits_input, uint8_t clear_bits_input):
            reg(reg_input), set_bits(set_bits_input), clear_bits(clear_bits_input){}

		Register::MREG1 reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

    /**
     * Start automatic measurement.
     */
    void			start();

    /**
     * Stop automatic measurement.
     */
    void			stop();


    bool reset();

    void Run() override;

	bool Configure();
	void ConfigureSampleRate(int sample_rate);
	void ConfigureFIFOWatermark(uint8_t samples);

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	uint8_t RegisterRead(Register::BANK_0 reg);
	uint8_t RegisterRead(Register::MREG1 reg);

	void RegisterWrite(Register::BANK_0 reg, uint8_t value);
	void RegisterWrite(Register::MREG1 reg, uint8_t value);

	template <typename T> bool RegisterCheck(const T &reg_cfg);
	template <typename T> void RegisterSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits);
	template <typename T> void RegisterSetBits(T reg, uint8_t setbits) { RegisterSetAndClearBits(reg, setbits, 0); }
	template <typename T> void RegisterClearBits(T reg, uint8_t clearbits) { RegisterSetAndClearBits(reg, 0, clearbits); }

	uint16_t FIFOReadCount();
	bool FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples);
	void FIFOReset();

	void ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples);
	void ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples);
	void UpdateTemperature();

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;
    int16_t			_last_accel_sample[3] {};//last accel sample for fifo filtering
    int16_t			_last_gyro_sample[3] {};//last gyro sample for fifo filtering

    perf_counter_t _bad_register_perf;
    perf_counter_t _bad_transfer_perf;
    perf_counter_t _fifo_empty_perf;
    perf_counter_t _fifo_overflow_perf;
    perf_counter_t _fifo_reset_perf;
    perf_counter_t _drdy_missed_perf{nullptr};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	hrt_abstime _temperature_update_timestamp{0};
	int _failure_count{0};

    px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};
	bool _data_ready_interrupt_enabled{false};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		FIFO_READ,
	} _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{1250}; // default 1250 us / 800 Hz transfer interval
	int32_t _fifo_gyro_samples{static_cast<int32_t>(_fifo_empty_interval_us / (1000000 / GYRO_RATE))};

	uint8_t _checked_register_bank0{0};
	static constexpr uint8_t size_register_bank0_cfg{10};
    register_bank0_config_t _register_bank0_cfg[size_register_bank0_cfg] {
        // Register                              | Set bits, Clear bits
        { Register::BANK_0::INT_CONFIG,           INT_CONFIG_BIT::INT1_MODE | INT_CONFIG_BIT::INT1_DRIVE_CIRCUIT, INT_CONFIG_BIT::INT1_POLARITY },
        { Register::BANK_0::PWR_MGMT0,            PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE | PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE, 0 },
        { Register::BANK_0::GYRO_CONFIG0,         GYRO_CONFIG0_BIT::GYRO_FS_SEL_2000_DPS_SET | GYRO_CONFIG0_BIT::GYRO_ODR_1600HZ_SET, GYRO_CONFIG0_BIT::GYRO_FS_SEL_2000_DPS_CLEAR | GYRO_CONFIG0_BIT::GYRO_ODR_1600HZ_CLEAR },
        { Register::BANK_0::ACCEL_CONFIG0,        ACCEL_CONFIG0_BIT::ACCEL_UI_FS_SEL_16G_SET | ACCEL_CONFIG0_BIT::ACCEL_ODR_1600HZ_SET, ACCEL_CONFIG0_BIT::ACCEL_UI_FS_SEL_16G_SET | ACCEL_CONFIG0_BIT::ACCEL_ODR_1600HZ_CLEAR },
        { Register::BANK_0::GYRO_CONFIG1,         0, GYRO_CONFIG1_BIT::GYRO_UI_FILT_BW_BYPASSED_CLEAR },
        { Register::BANK_0::ACCEL_CONFIG1,        0, ACCEL_CONFIG1_BIT::ACCEL_UI_FILT_BW_BYPASSED_CLEAR },
        { Register::BANK_0::FIFO_CONFIG1,         FIFO_CONFIG1_BIT::FIFO_MODE_STOP_ON_FULL, FIFO_CONFIG1_BIT::FIFO_BYPASS },
        { Register::BANK_0::FIFO_CONFIG2,         0, 0 }, // FIFO_WM[7:0] set at runtime
        { Register::BANK_0::FIFO_CONFIG3,         0, 0 }, // FIFO_WM[11:8] set at runtime
        { Register::BANK_0::INT_SOURCE0,          INT_SOURCE0_BIT::FIFO_THS_INT1_EN, 0 },
    };

	uint8_t _checked_register_mreg1{0};
	static constexpr uint8_t size_register_mreg1_cfg{2};
	register_mreg1_config_t _register_mreg1_cfg[size_register_mreg1_cfg] {
		// Register                              | Set bits, Clear bits
		{ Register::MREG1::FIFO_CONFIG5,          FIFO_CONFIG5_BIT::FIFO_GYRO_EN | FIFO_CONFIG5_BIT::FIFO_ACCEL_EN, 0 },
		{ Register::MREG1::INT_CONFIG0,           INT_CONFIG0_BIT::FIFO_THS_INT_CLEAR, 0 },
	};
};
