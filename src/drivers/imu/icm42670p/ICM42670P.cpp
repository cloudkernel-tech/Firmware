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

#include "ICM42670P.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

static int32_t sum(const int16_t samples[], uint8_t len)
{
    int32_t sum = 0;

    for (int n = 0; n < len; n++) {
        sum += samples[n];
    }

    return sum;
}


ICM42670P::ICM42670P(int bus, uint32_t device, enum Rotation rotation):
    SPI("ICM42670P", nullptr, bus, device, SPIDEV_MODE3, SPI_SPEED),
    ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(this->get_device_id())),
    _px4_accel(get_device_id(), (external() ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1), rotation),
    _px4_gyro(get_device_id(),(external() ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1), rotation),
    _bad_register_perf(perf_alloc(PC_COUNT, "icm42670p_bad_register")),
    _bad_transfer_perf(perf_alloc(PC_COUNT, "icm42670p_bad_transfer")),
    _fifo_empty_perf(perf_alloc(PC_COUNT, "icm42670p_fifo_empty")),
    _fifo_overflow_perf(perf_alloc(PC_COUNT, "icm42670p_fifo_overflow")),
    _fifo_reset_perf(perf_alloc(PC_COUNT, "icm42670p_fifo_reset"))
{

    ConfigureSampleRate(ICM42670_SENSOR_DEFAULT_RATE);

}

ICM42670P::~ICM42670P()
{
    /* make sure we are truly inactive */
    stop();

	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

int ICM42670P::init()
{
    /* do SPI init (and probe) first */
    int ret = SPI::init();

    /* if probe/setup failed, bail now */
    if (ret != OK) {
        DEVICE_DEBUG("SPI setup failed");
        return ret;
    }

    ret = reset() ? OK : PX4_ERROR;

    if (ret != OK) {
        PX4_DEBUG("reset failed");
        return ret;
    }

    start();

    return ret;
}


void ICM42670P::start()
{
    if (!DataReadyInterruptConfigure())
        PX4_DEBUG("started failed");
    else
        PX4_INFO("driver started");
}

void ICM42670P::stop()
{
    DataReadyInterruptDisable();
}

bool ICM42670P::reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}


void ICM42670P::print_info()
{

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);

    _px4_accel.print_status();
    _px4_gyro.print_status();
}

int ICM42670P::probe()
{
	const uint8_t whoami = RegisterRead(Register::BANK_0::WHO_AM_I);

	if (whoami != WHOAMI) {
		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void ICM42670P::Run()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// SIGNAL_PATH_RESET: Software Reset (auto clear bit)
		RegisterWrite(Register::BANK_0::SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_BIT::SOFT_RESET_DEVICE_CONFIG);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(1_ms); // wait 1 ms for soft reset to be effective
		break;

	case STATE::WAIT_FOR_RESET:
		if ((RegisterRead(Register::BANK_0::WHO_AM_I) == WHOAMI)
		    && (RegisterRead(Register::BANK_0::SIGNAL_PATH_RESET) == 0x00)
		    && (RegisterRead(Register::BANK_0::INT_STATUS) & INT_STATUS_BIT::RESET_DONE_INT)) {

			// Wakeup accel and gyro and schedule remaining configuration
			RegisterWrite(Register::BANK_0::PWR_MGMT0, PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE | PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE);
			_state = STATE::CONFIGURE;
			ScheduleDelayed(30_ms); // 30 ms gyro startup time, 10 ms accel from sleep to valid data

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading from FIFO
			_state = STATE::FIFO_READ;

			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(100_ms);

			} else {
				_data_ready_interrupt_enabled = false;
				ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
			}

			FIFOReset();

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::FIFO_READ: {
			hrt_abstime timestamp_sample = now;
			uint8_t samples = 0;

			if (_data_ready_interrupt_enabled) {
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
                const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if ((now - drdy_timestamp_sample) < _fifo_empty_interval_us) {
					timestamp_sample = drdy_timestamp_sample;
					samples = _fifo_gyro_samples;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				ScheduleDelayed(_fifo_empty_interval_us * 2);
			}

			if (samples == 0) {
				// check current FIFO count
				const uint16_t fifo_count = FIFOReadCount();

				if (fifo_count >= FIFO::SIZE) {
					FIFOReset();
					perf_count(_fifo_overflow_perf);

				} else if (fifo_count == 0) {
					perf_count(_fifo_empty_perf);

				} else {
					// FIFO count (size in bytes)
					samples = (fifo_count / sizeof(FIFO::DATA));

					// tolerate minor jitter, leave sample to next iteration if behind by only 1
					if (samples == _fifo_gyro_samples + 1) {
						timestamp_sample -= static_cast<int>(FIFO_SAMPLE_DT);
						samples--;
					}

					if (samples > FIFO_MAX_SAMPLES) {
						// not technically an overflow, but more samples than we expected or can publish
						FIFOReset();
						perf_count(_fifo_overflow_perf);
						samples = 0;
					}
				}
			}

			bool success = false;

			if (samples >= 1) {
				if (FIFORead(timestamp_sample, samples)) {
					success = true;

					if (_failure_count > 0) {
						_failure_count--;
					}
				}
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
                    reset();
					return;
				}
			}

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_bank0_cfg[_checked_register_bank0])
				    && RegisterCheck(_register_mreg1_cfg[_checked_register_mreg1])
				   ) {
					_last_config_check_timestamp = now;
					_checked_register_bank0 = (_checked_register_bank0 + 1) % size_register_bank0_cfg;
					_checked_register_mreg1 = (_checked_register_mreg1 + 1) % size_register_mreg1_cfg;

				} else {
					// register check failed, force reset
					perf_count(_bad_register_perf);
                    reset();
				}

			} else {
				// periodically update temperature (~1 Hz)
				if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
					UpdateTemperature();
					_temperature_update_timestamp = now;
				}
			}
		}

		break;
	}
}

void ICM42670P::ConfigureSampleRate(int sample_rate)
{
	// round down to nearest FIFO sample dt
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = roundf(math::min((float)_fifo_empty_interval_us / (1e6f / GYRO_RATE), (float)FIFO_MAX_SAMPLES));

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / GYRO_RATE);

	ConfigureFIFOWatermark(_fifo_gyro_samples);
}

void ICM42670P::ConfigureFIFOWatermark(uint8_t samples)
{
	// FIFO watermark threshold in number of bytes
	const uint16_t fifo_watermark_threshold = samples * sizeof(FIFO::DATA);

	for (auto &r : _register_bank0_cfg) {
		if (r.reg == Register::BANK_0::FIFO_CONFIG2) {
			// FIFO_WM[7:0]  FIFO_CONFIG2
			r.set_bits = fifo_watermark_threshold & 0xFF;

		} else if (r.reg == Register::BANK_0::FIFO_CONFIG3) {
			// FIFO_WM[11:8] FIFO_CONFIG3
			r.set_bits = (fifo_watermark_threshold >> 8) & 0x0F;
		}
	}
}

bool ICM42670P::Configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_bank0_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	for (const auto &reg_cfg : _register_mreg1_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_bank0_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	for (const auto &reg_cfg : _register_mreg1_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	_px4_accel.set_scale(CONSTANTS_ONE_G / 2048.f);

	_px4_gyro.set_scale(math::radians(2000.f / 32768.f));

	return success;
}

int ICM42670P::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ICM42670P *>(arg)->DataReady();
	return 0;
}

void ICM42670P::DataReady()
{
    _drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool ICM42670P::DataReadyInterruptConfigure()
{
	// Setup data ready on falling edge
#if defined(GPIO_SPI1_EXTI_ICM42670P_DRDY_PC14)
    return px4_arch_gpiosetevent(GPIO_SPI1_EXTI_ICM42670P_DRDY_PC14, false, true, true, &DataReadyInterruptCallback, this) == 0;
#else
    return false;
#endif
}

bool ICM42670P::DataReadyInterruptDisable()
{
#if defined(GPIO_SPI1_EXTI_ICM42670P_DRDY_PC14)
    return px4_arch_gpiosetevent(GPIO_SPI1_EXTI_ICM42670P_DRDY_PC14, false, false, false, nullptr, nullptr) == 0;
#else
    return false;
#endif
}

template <typename T>
bool ICM42670P::RegisterCheck(const T &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

uint8_t ICM42670P::RegisterRead(Register::BANK_0 reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

uint8_t ICM42670P::RegisterRead(Register::MREG1 reg)
{
	// BLK_SEL_R must be set to 0
	RegisterWrite(Register::BANK_0::BLK_SEL_R, 0x00);

	// MADDR_R must be set to 0x14 (address of the MREG1 register being accessed)
	RegisterWrite(Register::BANK_0::MADDR_R, (uint8_t)reg);

	// Wait for 10 µs
    up_udelay(10);

	// Read register M_R to access the value in MREG1 register 0x14
	uint8_t value = RegisterRead(Register::BANK_0::M_R);

	// Wait for 10 µs
	// Host must not access any other register for 10 µs once MREG1, MREG2 or MREG3 access is kicked off.
    up_udelay(10);

	return value;
}

void ICM42670P::RegisterWrite(Register::BANK_0 reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void ICM42670P::RegisterWrite(Register::MREG1 reg, uint8_t value)
{
	// BLK_SEL_W must be set to 0
	RegisterWrite(Register::BANK_0::BLK_SEL_W, 0x00);

	// MADDR_W must be set to address of the MREG1 register being accessed
	RegisterWrite(Register::BANK_0::MADDR_W, (uint8_t)reg);

	// M_W must be set to the desired value
	RegisterWrite(Register::BANK_0::M_W, value);

	// Wait for 10 µs
	// Host must not access any other register for 10 µs once MREG1, MREG2 or MREG3 access is kicked off.
    up_udelay(10);
}


template <typename T>
void ICM42670P::RegisterSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

uint16_t ICM42670P::FIFOReadCount()
{
	// read FIFO count
	uint8_t fifo_count_buf[3] {};
	fifo_count_buf[0] = static_cast<uint8_t>(Register::BANK_0::FIFO_COUNTH) | DIR_READ;

	if (transfer(fifo_count_buf, fifo_count_buf, sizeof(fifo_count_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return 0;
	}

	return combine(fifo_count_buf[1], fifo_count_buf[2]);
}

bool ICM42670P::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	FIFOTransferBuffer buffer{};
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 6, FIFO::SIZE);

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	if (buffer.INT_STATUS & INT_STATUS_BIT::FIFO_FULL_INT) {
		perf_count(_fifo_overflow_perf);
		FIFOReset();
		return false;
	}

	const uint16_t fifo_count_bytes = combine(buffer.FIFO_COUNTH, buffer.FIFO_COUNTL);

	if (fifo_count_bytes >= FIFO::SIZE) {
		perf_count(_fifo_overflow_perf);
		FIFOReset();
		return false;
	}

	const uint8_t fifo_count_samples = fifo_count_bytes / sizeof(FIFO::DATA);

	if (fifo_count_samples == 0) {
		perf_count(_fifo_empty_perf);
		return false;
	}

	// check FIFO header in every sample
	uint8_t valid_samples = 0;

	for (int i = 0; i < math::min(samples, fifo_count_samples); i++) {
		bool valid = true;

		// With FIFO_ACCEL_EN and FIFO_GYRO_EN header should be 8’b_0110_10xx
		const uint8_t FIFO_HEADER = buffer.f[i].FIFO_Header;

		if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_MSG) {
			// FIFO sample empty if HEADER_MSG set
			valid = false;

		} else if (!(FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ACCEL)) {
			// accel bit not set
			valid = false;

		} else if (!(FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_GYRO)) {
			// gyro bit not set
			valid = false;

		} else if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_20) {
			// Packet does not contain a new and valid extended 20-bit data
			valid = false;

		} else if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ODR_ACCEL) {
			// accel ODR changed
			valid = false;

		} else if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ODR_GYRO) {
			// gyro ODR changed
			valid = false;
		}

		if (valid) {
			valid_samples++;

		} else {
			perf_count(_bad_transfer_perf);
			break;
		}
	}

	if (valid_samples > 0) {
		ProcessGyro(timestamp_sample, buffer.f, valid_samples);
		ProcessAccel(timestamp_sample, buffer.f, valid_samples);
		return true;
	}

	return false;
}

void ICM42670P::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// FIFO flush requires the following programming sequence:
	//  Write FIFO_FLUSH = 1
	//  Wait for 1.5 µs
	//  Read FIFO_FLUSH, it should now be 0

	// SIGNAL_PATH_RESET: FIFO flush
	RegisterSetBits(Register::BANK_0::SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_BIT::FIFO_FLUSH);
    up_udelay(2); // Wait for 1.5 µs

	const uint8_t SIGNAL_PATH_RESET = RegisterRead(Register::BANK_0::SIGNAL_PATH_RESET);

	if ((SIGNAL_PATH_RESET & SIGNAL_PATH_RESET_BIT::FIFO_FLUSH) != 0) {
		PX4_DEBUG("SIGNAL_PATH_RESET FIFO_FLUSH failed");
	}

	// reset while FIFO is disabled
    _drdy_timestamp_sample.store(0);
}

void ICM42670P::ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
    int16_t accel_x[samples], accel_y[samples], accel_z[samples];

    for (int i = 0; i < samples; i++) {
        accel_x[i] = combine(fifo[i].ACCEL_DATA_X1, fifo[i].ACCEL_DATA_X0);
        accel_y[i] = combine(fifo[i].ACCEL_DATA_Y1, fifo[i].ACCEL_DATA_Y0);
        accel_z[i] = combine(fifo[i].ACCEL_DATA_Z1, fifo[i].ACCEL_DATA_Z0);
    }

    int16_t acc_report[3];

    acc_report[0] = (0.5f*(accel_x[samples-1] + _last_accel_sample[0]) + sum(accel_x, samples-1))/samples;
    acc_report[1] = (0.5f*(accel_y[samples-1] + _last_accel_sample[1]) + sum(accel_y, samples-1))/samples;
    acc_report[2] = (0.5f*(accel_z[samples-1] + _last_accel_sample[2]) + sum(accel_z, samples-1))/samples;

    _last_accel_sample[0] = acc_report[0];
    _last_accel_sample[1] = acc_report[1];
    _last_accel_sample[2] = acc_report[2];

	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

    _px4_accel.update(timestamp_sample, acc_report[0], acc_report[1], acc_report[2]);

}

void ICM42670P::ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
    int16_t gyro_x[samples], gyro_y[samples], gyro_z[samples];

    for (int i = 0; i < samples; i++) {
        gyro_x[i] = combine(fifo[i].GYRO_DATA_X1, fifo[i].GYRO_DATA_X0);
        gyro_y[i] = combine(fifo[i].GYRO_DATA_Y1, fifo[i].GYRO_DATA_Y0);
        gyro_z[i] = combine(fifo[i].GYRO_DATA_Z1, fifo[i].GYRO_DATA_Z0);
    }

    int16_t gyro_report[3];

    gyro_report[0] = (0.5f*(gyro_x[samples-1] + _last_gyro_sample[0]) + sum(gyro_x, samples-1))/samples;
    gyro_report[1] = (0.5f*(gyro_y[samples-1] + _last_gyro_sample[1]) + sum(gyro_y, samples-1))/samples;
    gyro_report[2] = (0.5f*(gyro_z[samples-1] + _last_gyro_sample[2]) + sum(gyro_z, samples-1))/samples;

    _last_gyro_sample[0] = gyro_report[0];
    _last_gyro_sample[1] = gyro_report[1];
    _last_gyro_sample[2] = gyro_report[2];


	_px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				  perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

    _px4_gyro.update(timestamp_sample, gyro_report[0], gyro_report[1], gyro_report[2]);


}

void ICM42670P::UpdateTemperature()
{
	// read current temperature
	uint8_t temperature_buf[3] {};
	temperature_buf[0] = static_cast<uint8_t>(Register::BANK_0::TEMP_DATA1) | DIR_READ;

	if (transfer(temperature_buf, temperature_buf, sizeof(temperature_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return;
	}

	const int16_t TEMP_DATA = combine(temperature_buf[1], temperature_buf[2]);

	// Temperature in Degrees Centigrade
	const float TEMP_degC = (TEMP_DATA / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;

	if (PX4_ISFINITE(TEMP_degC)) {
		_px4_accel.set_temperature(TEMP_degC);
		_px4_gyro.set_temperature(TEMP_degC);
	}
}
