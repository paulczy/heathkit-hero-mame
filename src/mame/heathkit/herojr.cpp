// license:BSD-3-Clause
// copyright-holders:Paul Czywczynski

/***************************************************************************

    Heathkit HERO Jr robot driver

    This file exists to track the requested HERO Jr System ROM v1.6 target.
    The model is intentionally hardware-facing: bridge-visible movement feedback
    is generated from this driver path, not by the Lua bridge completing motion.

***************************************************************************/

#include "emu.h"

#include "bus/generic/carts.h"
#include "bus/generic/slot.h"
#include "bus/rs232/rs232.h"
#include "cpu/m6800/m6800.h"
#include "machine/6821pia.h"
#include "machine/6850acia.h"
#include "machine/clock.h"
#include "machine/mc146818.h"
#include "osdcore.h"
#include "sound/votrax.h"

#include "speaker.h"

#include <algorithm>
#include <cstring>


namespace {

class herojr_state : public driver_device
{
public:
	herojr_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_cart(*this, "cartslot"),
		m_rtc(*this, "rtc"),
		m_acia(*this, "acia"),
		m_rs232(*this, "rs232"),
		m_u214(*this, "u214"),
		m_u215(*this, "u215"),
		m_votrax(*this, "votrax"),
		m_keypad_rows(*this, "KEY%u", 0U),
		m_light_level(*this, "LIGHT"),
		m_sound_level(*this, "SOUND"),
		m_sonar_distance(*this, "SONAR"),
		m_sleep_norm(*this, "SLEEP_NORM"),
		m_speech_phoneme(*this, "herojr_speech_phoneme"),
		m_speech_inflection(*this, "herojr_speech_inflection"),
		m_speech_strobe(*this, "herojr_speech_strobe"),
		m_speech_ready(*this, "herojr_speech_ready"),
		m_speech_power(*this, "herojr_speech_power"),
		m_adc_sample(*this, "herojr_adc_sample"),
		m_adc_output(*this, "herojr_adc_output"),
		m_sonar_echo(*this, "herojr_sonar_echo"),
		m_sonar_distance_output(*this, "herojr_sonar_distance"),
		m_motor_left(*this, "herojr_motor_left"),
		m_motor_right(*this, "herojr_motor_right"),
		m_motor_head(*this, "herojr_motor_head"),
		m_motor_arm(*this, "herojr_motor_arm"),
		m_motion_detector(*this, "herojr_motion_detector"),
		m_wheel_feedback(*this, "herojr_wheel_feedback"),
		m_drive_activity(*this, "herojr_drive_activity"),
		m_rtc_sqw(*this, "herojr_rtc_sqw"),
		m_rs232_status_output(*this, "herojr_rs232_status"),
		m_rs232_data_output(*this, "herojr_rs232_data"),
		m_port_outputs(*this, "herojr_port_out_%u", 0U)
	{
	}

	void herojr(machine_config &config) ATTR_COLD;
	DECLARE_INPUT_CHANGED_MEMBER(sleep_norm_changed);
	DECLARE_INPUT_CHANGED_MEMBER(reset_changed);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	static constexpr u16 HEROJR_SENSOR_BASE = 0xd860; // bridge injection aperture, not original hardware

	DECLARE_DEVICE_IMAGE_LOAD_MEMBER(cart_load);
	void mem_map(address_map &map) ATTR_COLD;
	u8 sensor_debug_r(offs_t offset);
	void sensor_debug_w(offs_t offset, u8 data);
	u8 rtc_r(offs_t offset);
	u8 u214_port_a_r();
	u8 keypad_matrix_r();
	static bool is_keypad_bridge_press(u8 data);
	static bool is_keypad_bridge_release(u8 data);
	u8 u214_port_b_r();
	void u214_port_a_w(u8 data);
	void u214_port_b_w(u8 data);
	u8 u214_control_a_r();
	u8 u214_control_b_r();
	void u214_control_a_w(u8 data);
	void u214_control_b_w(u8 data);
	void rtc_w(offs_t offset, u8 data);
	u8 u215_speech_data_r();
	u8 u215_speech_power_r();
	u8 u215_speech_control_r();
	u8 u215_sonar_echo_r();
	u8 rs232_r(offs_t offset);
	void u215_speech_data_w(u8 data);
	void u215_speech_power_w(u8 data);
	void u215_speech_control_w(u8 data);
	void u215_sonar_echo_w(u8 data);
	void rs232_w(offs_t offset, u8 data);
	void speech_request_w(int state);
	void acia_irq_w(int state);
	void set_speech_data(u8 data);
	void latch_speech_phoneme();
	bool drive_feedback_active() const;
	void advance_wheel_feedback_sample();
	void update_drive_sonar_motion();
	u8 selected_adc_sample() const;
	void start_adc_conversion();
	void clock_adc_bit();
	void schedule_sonar_echo();
	void update_u214_input_outputs();
	void update_irq_line();
	void update_speech_power();
	void set_sleep_norm_input(bool norm);
	void reset_interface_state();
	void pulse_reset_line();
	TIMER_CALLBACK_MEMBER(rtc_square_wave_tick);
	TIMER_CALLBACK_MEMBER(sonar_echo_tick);

	required_device<m6808_cpu_device> m_maincpu;
	required_device<generic_slot_device> m_cart;
	required_device<mc146818_device> m_rtc;
	required_device<acia6850_device> m_acia;
	required_device<rs232_port_device> m_rs232;
	required_device<pia6821_device> m_u214;
	required_device<pia6821_device> m_u215;
	required_device<votrax_sc01_device> m_votrax;
	required_ioport_array<4> m_keypad_rows;
	required_ioport m_light_level;
	required_ioport m_sound_level;
	required_ioport m_sonar_distance;
	required_ioport m_sleep_norm;

	output_finder<> m_speech_phoneme;
	output_finder<> m_speech_inflection;
	output_finder<> m_speech_strobe;
	output_finder<> m_speech_ready;
	output_finder<> m_speech_power;
	output_finder<> m_adc_sample;
	output_finder<> m_adc_output;
	output_finder<> m_sonar_echo;
	output_finder<> m_sonar_distance_output;
	output_finder<> m_motor_left;
	output_finder<> m_motor_right;
	output_finder<> m_motor_head;
	output_finder<> m_motor_arm;
	output_finder<> m_motion_detector;
	output_finder<> m_wheel_feedback;
	output_finder<> m_drive_activity;
	output_finder<> m_rtc_sqw;
	output_finder<> m_rs232_status_output;
	output_finder<> m_rs232_data_output;
	output_finder<8> m_port_outputs;

	u8 m_u214_port_a = 0xff;
	u8 m_keypad_bridge_byte = 0xff;
	u8 m_u214_port_b = 0xff;
	u8 m_u214_control_a = 0;
	u8 m_u214_control_b = 0;
	u8 m_motion_detector_state = 0;
	u8 m_rtc_sqw_state = 0;
	u8 m_rtc_irq_state = 0;
	u8 m_acia_irq_state = 0;
	u8 m_speech_data = 0;
	u8 m_u215_port_b = 0;
	u8 m_u215_control_a = 0;
	u8 m_u215_control_b = 0;
	u8 m_adc_shift = 0;
	u8 m_adc_bits_remaining = 0;
	u8 m_adc_output_state = 0;
	u8 m_sonar_echo_state = 0;
	u8 m_wheel_feedback_sample = 0;
	u16 m_wheel_feedback_port_a_count = 0;
	u8 m_light_sample = 50;
	u8 m_sound_sample = 0;
	u8 m_sonar_distance_sample = 48;
	u8 m_speech_power_state = 0;
	u8 m_speech_request = 1;
	u8 m_speech_strobe_state = 0;
	u8 m_rs232_status = 0;
	u8 m_rs232_data = 0;
	u32 m_drive_activity_count = 0;
	emu_timer *m_rtc_square_wave_timer = nullptr;
	emu_timer *m_sonar_echo_timer = nullptr;
	memory_passthrough_handler m_sleep_loop_tap;
};

void herojr_state::machine_start()
{
	m_speech_phoneme.resolve();
	m_speech_inflection.resolve();
	m_speech_strobe.resolve();
	m_speech_ready.resolve();
	m_speech_power.resolve();
	m_adc_sample.resolve();
	m_adc_output.resolve();
	m_sonar_echo.resolve();
	m_sonar_distance_output.resolve();
	m_motor_left.resolve();
	m_motor_right.resolve();
	m_motor_head.resolve();
	m_motor_arm.resolve();
	m_motion_detector.resolve();
	m_wheel_feedback.resolve();
	m_drive_activity.resolve();
	m_rtc_sqw.resolve();
	m_rs232_status_output.resolve();
	m_rs232_data_output.resolve();
	m_port_outputs.resolve();

	const char *const initial_sleep = osd_getenv("HEATHKIT_HEROJR_INITIAL_SLEEP");
	if (initial_sleep && !std::strcmp(initial_sleep, "1"))
		set_sleep_norm_input(false);

	save_item(NAME(m_u214_port_a));
	save_item(NAME(m_keypad_bridge_byte));
	save_item(NAME(m_u214_port_b));
	save_item(NAME(m_u214_control_a));
	save_item(NAME(m_u214_control_b));
	save_item(NAME(m_motion_detector_state));
	save_item(NAME(m_rtc_sqw_state));
	save_item(NAME(m_rtc_irq_state));
	save_item(NAME(m_acia_irq_state));
	save_item(NAME(m_speech_data));
	save_item(NAME(m_u215_port_b));
	save_item(NAME(m_u215_control_a));
	save_item(NAME(m_u215_control_b));
	save_item(NAME(m_adc_shift));
	save_item(NAME(m_adc_bits_remaining));
	save_item(NAME(m_adc_output_state));
	save_item(NAME(m_sonar_echo_state));
	save_item(NAME(m_wheel_feedback_sample));
	save_item(NAME(m_wheel_feedback_port_a_count));
	save_item(NAME(m_light_sample));
	save_item(NAME(m_sound_sample));
	save_item(NAME(m_sonar_distance_sample));
	save_item(NAME(m_speech_power_state));
	save_item(NAME(m_speech_request));
	save_item(NAME(m_speech_strobe_state));
	save_item(NAME(m_rs232_status));
	save_item(NAME(m_rs232_data));
	save_item(NAME(m_drive_activity_count));

	m_rtc_square_wave_timer = timer_alloc(FUNC(herojr_state::rtc_square_wave_tick), this);
	m_sonar_echo_timer = timer_alloc(FUNC(herojr_state::sonar_echo_tick), this);
	m_sleep_loop_tap = m_maincpu->space(AS_PROGRAM).install_read_tap(
		0xec32,
		0xec32,
		"herojr_sleep_loop",
		[this](offs_t offset, u8 &data, u8 mem_mask)
		{
			if ((m_maincpu->pc() & 0xffff) == 0xec32)
				m_maincpu->spin_until_time(attotime::from_msec(10));
		});
}

void herojr_state::set_sleep_norm_input(bool norm)
{
	ioport_field *const field = m_sleep_norm->field(0x01);
	if (!field)
		return;

	// The port's default value is NORM (1).  Programmatic digital values are
	// folded through the port default on read, so a held input reads as SLEEP.
	field->set_value(norm ? 0 : 1);
}

void herojr_state::reset_interface_state()
{
	m_u214->reset();
	m_u215->reset();

	m_u214_port_a = 0xff;
	m_keypad_bridge_byte = 0xff;
	m_u214_port_b = 0x00;
	m_u214_control_a = 0;
	m_u214_control_b = 0;
	m_motion_detector_state = 0;
	m_speech_data = 0;
	m_u215_port_b = 0;
	m_u215_control_a = 0;
	m_u215_control_b = 0;
	m_adc_shift = 0;
	m_adc_bits_remaining = 0;
	m_adc_output_state = 0;
	m_sonar_echo_state = 0;
	m_wheel_feedback_sample = 0;
	m_wheel_feedback_port_a_count = 0;
	m_light_sample = m_light_level->read() & 0xff;
	m_sound_sample = m_sound_level->read() & 0xff;
	m_sonar_distance_sample = m_sonar_distance->read() & 0xff;
	m_speech_power_state = 0;
	m_speech_request = 1;
	m_speech_strobe_state = 0;
	m_drive_activity_count = 0;
	m_speech_phoneme = 0;
	m_speech_inflection = 0;
	m_speech_strobe = 0;
	m_speech_ready = 1;
	m_speech_power = 0;
	m_adc_sample = 0;
	m_adc_output = 0;
	m_sonar_echo = 0;
	m_sonar_distance_output = m_sonar_distance_sample;
	m_votrax->reset();
	update_speech_power();
	m_motor_left = 0;
	m_motor_right = 0;
	m_motor_head = 0;
	m_motor_arm = 0;
	m_motion_detector = 0;
	m_wheel_feedback = 0;
	m_drive_activity = 0;
	for (int port = 0; port < 8; port++)
		m_port_outputs[port] = 0;
	m_sonar_echo_timer->adjust(attotime::never);
}

void herojr_state::pulse_reset_line()
{
	m_maincpu->resume(SUSPEND_REASON_HALT);
	reset_interface_state();
	m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);
}

void herojr_state::machine_reset()
{
	reset_interface_state();
	m_rtc_sqw_state = 0;
	m_rtc_irq_state = 0;
	m_acia_irq_state = 0;
	m_rs232_status = 0;
	m_rs232_data = 0;
	m_rtc_sqw = 0;
	m_rs232_status_output = 0;
	m_rs232_data_output = 0;
	// The monitor polls MC146818 register A through $D810/$D811 during boot
	// and waits for UIP to clear.  Start from a valid divider/rate register
	// instead of inheriting an erased NVRAM byte.
	m_rtc->write_direct(0x0a, 0x26);
	m_rtc->write_direct(0x0b, 0x02);
	m_rtc->read_direct(0x0d);
	update_irq_line();
	m_rtc_square_wave_timer->adjust(attotime::from_hz(1024), 0, attotime::from_hz(1024));
}

INPUT_CHANGED_MEMBER(herojr_state::sleep_norm_changed)
{
	(void)oldval;
	(void)newval;
	// SW2 is read by firmware at $D841 D6.  The operator manual's warm path is
	// explicit: place SW2 in NORM, then press RESET.  Do not turn the switch
	// edge itself into a firmware reset.
}

INPUT_CHANGED_MEMBER(herojr_state::reset_changed)
{
	if (!oldval && newval)
		pulse_reset_line();
}

void herojr_state::mem_map(address_map &map)
{
	map(0x0000, 0x07ff).mirror(0x1800).ram(); // Supplied U203 2K RAM in the decoded $0000-$1FFF memory window
	map(0x2000, 0x3fff).ram(); // U204 expansion RAM in the decoded $2000-$3FFF memory window
	map(0x4000, 0x5fff).ram(); // U205 expansion RAM in the decoded $4000-$5FFF memory window
	map(0x6000, 0x7fff).r(m_cart, FUNC(generic_slot_device::read_rom)); // U206 optional cartridge/ROM adapter window

	// HERO Jr Technical Manual address decoder table:
	// $D810-$D81F selects U213 real-time clock.
	// $D820-$D83F selects U214, $D840-$D85F selects U215.
	// $D880-$DFFF selects optional RS-232 adapter U216.
	// The byte handlers below follow the manual's port labels directly:
	// $D840 carries SC-01 phoneme/pitch data and the data LEDs, $D841 carries
	// speech/sense control plus the serial ADC return bit, $D842 CA2/CA1
	// carries speech strobe/request, $D843 carries the U215 CB1 sonar echo
	// status, and the HERO Jr ROM treats $D880/$D881 as serial status/control
	// and data.
	map(0xd810, 0xd81f).rw(FUNC(herojr_state::rtc_r), FUNC(herojr_state::rtc_w));
	map(0xd820, 0xd820).rw(FUNC(herojr_state::u214_port_a_r), FUNC(herojr_state::u214_port_a_w));
	map(0xd821, 0xd821).rw(FUNC(herojr_state::u214_port_b_r), FUNC(herojr_state::u214_port_b_w));
	map(0xd822, 0xd822).rw(FUNC(herojr_state::u214_control_a_r), FUNC(herojr_state::u214_control_a_w));
	map(0xd823, 0xd823).rw(FUNC(herojr_state::u214_control_b_r), FUNC(herojr_state::u214_control_b_w));
	map(0xd840, 0xd840).rw(FUNC(herojr_state::u215_speech_data_r), FUNC(herojr_state::u215_speech_data_w));
	map(0xd841, 0xd841).rw(FUNC(herojr_state::u215_speech_power_r), FUNC(herojr_state::u215_speech_power_w));
	map(0xd842, 0xd842).rw(FUNC(herojr_state::u215_speech_control_r), FUNC(herojr_state::u215_speech_control_w));
	map(0xd843, 0xd843).rw(FUNC(herojr_state::u215_sonar_echo_r), FUNC(herojr_state::u215_sonar_echo_w));
	map(HEROJR_SENSOR_BASE + 0x00, HEROJR_SENSOR_BASE + 0x03).rw(FUNC(herojr_state::sensor_debug_r), FUNC(herojr_state::sensor_debug_w));
	map(0xd880, 0xdfff).rw(FUNC(herojr_state::rs232_r), FUNC(herojr_state::rs232_w));

	map(0x8000, 0xd7ff).rom().region("maincpu", 0);
	map(0xe000, 0xffff).rom().region("maincpu", 0x6000);
}

u8 herojr_state::sensor_debug_r(offs_t offset)
{
	switch (offset & 0x03)
	{
	case 0x00: return m_light_sample;
	case 0x01: return m_sound_sample;
	case 0x02: return m_sonar_distance_sample;
	case 0x03: return m_motion_detector_state;
	default: return 0;
	}
}

void herojr_state::sensor_debug_w(offs_t offset, u8 data)
{
	switch (offset & 0x03)
	{
	case 0x00:
		m_light_sample = data;
		break;
	case 0x01:
		m_sound_sample = data;
		break;
	case 0x02:
		m_sonar_distance_sample = data;
		m_sonar_distance_output = data;
		break;
	case 0x03:
		m_motion_detector_state = BIT(data, 0) ? 1 : 0;
		m_motion_detector = m_motion_detector_state;
		break;
	}
}

u8 herojr_state::rtc_r(offs_t offset)
{
	if (BIT(offset, 0))
	{
		const u8 address = m_rtc->get_address();
		const u8 data = m_rtc->data_r();
		if (address == 0x0a && data == 0xff)
			return 0x26;
		if (address == 0x0a)
			return data & 0x7f;
		if (address == 0x0d && data == 0xff)
			return 0x80;
		return data;
	}

	return m_rtc->get_address();
}

void herojr_state::rtc_w(offs_t offset, u8 data)
{
	if (BIT(offset, 0))
		m_rtc->data_w(data);
	else
		m_rtc->address_w(data);
}

u8 herojr_state::u214_port_a_r()
{
	return keypad_matrix_r();
}

u8 herojr_state::keypad_matrix_r()
{
	u8 data = m_u214_port_a;
	for (int row = 0; row < 4; row++)
	{
		const u8 row_bits = m_keypad_rows[row]->read() & 0x0f;
		const u8 row_line = 0x10 << row;
		for (int column = 0; column < 4; column++)
		{
			const u8 column_line = 1U << column;
			const bool bridge_pressed = !BIT(m_keypad_bridge_byte, column) && !BIT(m_keypad_bridge_byte, row + 4);
			if (!BIT(row_bits, column) && !bridge_pressed)
				continue;

			if (!BIT(data, column))
				data &= ~row_line;
			if (!BIT(data, row + 4))
				data &= ~column_line;
		}
	}

	if (drive_feedback_active() || m_wheel_feedback_port_a_count != 0)
	{
		advance_wheel_feedback_sample();
		data = (data & ~0xc0) | ((m_wheel_feedback_sample & 0x03) << 6);
		if (!drive_feedback_active() && m_u214_port_a == 0xff)
			m_wheel_feedback_port_a_count--;
	}

	return data;
}

bool herojr_state::is_keypad_bridge_press(u8 data)
{
	int column_count = 0;
	int row_count = 0;
	for (int bit = 0; bit < 4; bit++)
	{
		if (!BIT(data, bit))
			column_count++;
		if (!BIT(data, bit + 4))
			row_count++;
	}

	return column_count == 1 && row_count == 1;
}

bool herojr_state::is_keypad_bridge_release(u8 data)
{
	int low_count = 0;
	for (int bit = 0; bit < 8; bit++)
	{
		if (!BIT(data, bit))
			low_count++;
	}

	return low_count == 1;
}

u8 herojr_state::u214_port_b_r()
{
	return (m_u214_port_b & 0x7f) | (m_motion_detector_state ? 0x00 : 0x80);
}

bool herojr_state::drive_feedback_active() const
{
	return (m_u214_port_b & 0x3e) != 0;
}

void herojr_state::advance_wheel_feedback_sample()
{
	m_wheel_feedback_sample = (m_wheel_feedback_sample + 1) & 0x03;
	m_wheel_feedback = m_wheel_feedback_sample;
}

void herojr_state::update_drive_sonar_motion()
{
	if (!BIT(m_u214_port_b, 1) || (m_u214_port_b & 0x3f) == 0x3f)
		return;

	m_sonar_distance_sample = BIT(m_u214_port_b, 0)
		? std::min<u8>(96, m_sonar_distance_sample + 1)
		: std::max<u8>(1, m_sonar_distance_sample - 1);
	m_sonar_distance_output = m_sonar_distance_sample;
}

void herojr_state::u214_port_a_w(u8 data)
{
	if (is_keypad_bridge_press(data))
	{
		m_keypad_bridge_byte = data;
		return;
	}
	if (is_keypad_bridge_release(data))
	{
		m_keypad_bridge_byte = 0xff;
		return;
	}

	m_u214_port_a = data;
}

void herojr_state::u214_port_b_w(u8 data)
{
	m_u214_port_b = data & 0x7f;
	if ((m_u214_port_b & 0x3e) != 0)
	{
		m_drive_activity_count++;
		m_drive_activity = m_drive_activity_count;
		m_wheel_feedback_port_a_count = 2048;
		advance_wheel_feedback_sample();
	}
	// HERO Jr Technical Manual: $D821 D1 controls main drive motor A2,
	// D0 controls relay RY301 for direction, and D2-D5 drive steering phases.
	m_motor_left = BIT(data, 1) ? 1 : 0;
	m_motor_right = BIT(data, 0) ? 1 : 0;
	m_port_outputs[3] = (data & 0x3c) << 2;
	m_port_outputs[5] = (BIT(data, 1) ? 0x40 : 0x00) | (BIT(data, 0) ? 0x80 : 0x00);
	update_u214_input_outputs();
}

u8 herojr_state::u214_control_a_r()
{
	return (m_u214_control_a & 0x3f) | (m_rtc_sqw_state ? 0x80 : 0x00);
}

u8 herojr_state::u214_control_b_r()
{
	if (drive_feedback_active() || m_wheel_feedback_port_a_count != 0)
		advance_wheel_feedback_sample();
	else
		m_wheel_feedback_sample = 0;
	m_wheel_feedback = m_wheel_feedback_sample;
	return (m_u214_control_b & 0x3f) | (BIT(m_wheel_feedback_sample, 0) ? 0x80 : 0x00);
}

void herojr_state::u214_control_a_w(u8 data)
{
	m_u214_control_a = data & 0x3f;
}

void herojr_state::u214_control_b_w(u8 data)
{
	const u8 previous_control = m_u214_control_b;
	m_u214_control_b = data & 0x3f;
	if (!BIT(previous_control, 3) && BIT(m_u214_control_b, 3))
		schedule_sonar_echo();
}

void herojr_state::update_u214_input_outputs()
{
	m_motion_detector = m_motion_detector_state;
	if (!drive_feedback_active() && m_wheel_feedback_port_a_count == 0)
	{
		m_wheel_feedback_sample = 0;
		m_wheel_feedback = 0;
	}
	m_rtc_sqw = m_rtc_sqw_state;
}

void herojr_state::update_irq_line()
{
	m_maincpu->set_input_line(INPUT_LINE_IRQ0, (m_rtc_irq_state || m_acia_irq_state) ? ASSERT_LINE : CLEAR_LINE);
}

TIMER_CALLBACK_MEMBER(herojr_state::rtc_square_wave_tick)
{
	m_rtc_sqw_state ^= 1;
	m_rtc_irq_state = m_rtc_sqw_state;
	if (drive_feedback_active() || m_wheel_feedback_port_a_count != 0)
	{
		advance_wheel_feedback_sample();
		if (!drive_feedback_active() && m_wheel_feedback_port_a_count != 0)
			m_wheel_feedback_port_a_count--;
	}
	update_drive_sonar_motion();
	update_u214_input_outputs();
	update_irq_line();

}

u8 herojr_state::u215_speech_data_r()
{
	return m_speech_data;
}

u8 herojr_state::u215_speech_power_r()
{
	return (m_u215_port_b & 0xaf) | (m_adc_output_state ? 0x10 : 0x00) | (m_sleep_norm->read() ? 0x40 : 0x00);
}

u8 herojr_state::u215_speech_control_r()
{
	return (m_u215_control_a & 0x3f) | (m_speech_request ? 0x80 : 0x00);
}

u8 herojr_state::u215_sonar_echo_r()
{
	return m_sonar_echo_state ? 0x80 : 0x00;
}

void herojr_state::u215_speech_data_w(u8 data)
{
	m_port_outputs[0] = data;
	set_speech_data(data);
}

void herojr_state::u215_speech_power_w(u8 data)
{
	const u8 previous_port = m_u215_port_b;
	m_port_outputs[1] = data;
	m_u215_port_b = data;
	const u8 previous_speech_power = m_speech_power_state;
	m_speech_power_state = BIT(data, 1) ? 1 : 0;
	m_speech_power = m_speech_power_state;
	if (BIT(data, 3) != BIT(previous_port, 3))
		start_adc_conversion();
	if (BIT(previous_port, 5) && !BIT(data, 5))
		clock_adc_bit();
	update_speech_power();
	if (previous_speech_power && !m_speech_power_state)
	{
		m_votrax->reset();
		m_speech_request = 1;
		m_speech_ready = 1;
	}
}

void herojr_state::u215_speech_control_w(u8 data)
{
	m_port_outputs[2] = data;
	m_u215_control_a = data & 0x3f;
	const u8 previous_strobe = m_speech_strobe_state;
	m_speech_strobe_state = BIT(data, 3) ? 1 : 0;
	m_speech_strobe = m_speech_strobe_state;
	if (!previous_strobe && m_speech_strobe_state)
		latch_speech_phoneme();
}

void herojr_state::u215_sonar_echo_w(u8 data)
{
	m_u215_control_b = data & 0x3f;
}

u8 herojr_state::rs232_r(offs_t offset)
{
	if (BIT(offset, 0))
		return m_acia->data_r();

	m_rs232_status = m_acia->status_r();
	m_rs232_status_output = m_rs232_status;
	return m_rs232_status;
}

void herojr_state::rs232_w(offs_t offset, u8 data)
{
	if (BIT(offset, 0))
	{
		m_rs232_data = data;
		m_rs232_data_output = data;
		m_acia->data_w(data);
	}
	else
	{
		m_acia->control_w(data);
		m_rs232_status = m_acia->status_r();
		m_rs232_status_output = m_rs232_status;
	}
}

void herojr_state::speech_request_w(int state)
{
	m_speech_request = state ? 1 : 0;
	m_speech_ready = m_speech_request;
}

void herojr_state::acia_irq_w(int state)
{
	m_acia_irq_state = state ? 1 : 0;
	update_irq_line();
}

void herojr_state::set_speech_data(u8 data)
{
	m_speech_data = data;
	m_speech_phoneme = data & 0x3f;
	m_speech_inflection = (data >> 6) & 0x03;
}

void herojr_state::latch_speech_phoneme()
{
	if (!m_speech_power_state)
		return;

	m_votrax->inflection_w((m_speech_data >> 6) & 0x03);
	m_votrax->write(m_speech_data & 0x3f);
}

u8 herojr_state::selected_adc_sample() const
{
	// HERO Jr Technical Manual pp. 21-22: $D841 D2 selects light when high
	// and sound when low before U306 shifts the serial ADC result back.
	return BIT(m_u215_port_b, 2) ? m_light_sample : m_sound_sample;
}

void herojr_state::start_adc_conversion()
{
	m_adc_shift = selected_adc_sample();
	m_adc_bits_remaining = 8;
	m_adc_output_state = 0;
	m_adc_sample = m_adc_shift;
	m_adc_output = 0;
}

void herojr_state::clock_adc_bit()
{
	if (m_adc_bits_remaining == 0)
		start_adc_conversion();

	m_adc_output_state = BIT(m_adc_shift, 7) ? 1 : 0;
	m_adc_shift <<= 1;
	m_adc_bits_remaining--;
	m_adc_output = m_adc_output_state;
}

void herojr_state::schedule_sonar_echo()
{
	const u8 distance = m_sonar_distance_sample;
	m_sonar_echo_state = 0;
	m_sonar_echo = 0;
	m_sonar_distance_output = distance;

	// The real U307/U308 path measures elapsed echo time.  Keep a deterministic
	// round-trip scale for firmware polling without claiming calibrated motion.
	m_sonar_echo_timer->adjust(attotime::from_usec(std::max<u32>(1, distance) * 150));
}

void herojr_state::update_speech_power()
{
	m_votrax->set_output_gain(0, m_speech_power_state ? 1.0 : 0.0);
}

TIMER_CALLBACK_MEMBER(herojr_state::sonar_echo_tick)
{
	m_sonar_echo_state = 1;
	m_sonar_echo = 1;
}

DEVICE_IMAGE_LOAD_MEMBER(herojr_state::cart_load)
{
	const u32 size = m_cart->common_get_size("rom");
	if (size != 0x2000)
		return std::make_pair(image_error::INVALIDLENGTH, "Only 8K HERO Jr cartridge ROMs are supported");

	m_cart->rom_alloc(0x2000, GENERIC_ROM8_WIDTH, ENDIANNESS_LITTLE);
	m_cart->common_load_rom(m_cart->get_rom_base(), size, "rom");

	return std::make_pair(std::error_condition(), std::string());
}

static INPUT_PORTS_START(herojr)
	PORT_START("KEY0")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("RT-1") PORT_CODE(KEYCODE_0_PAD)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("1") PORT_CODE(KEYCODE_1)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("2") PORT_CODE(KEYCODE_2)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("3") PORT_CODE(KEYCODE_3)

	PORT_START("KEY1")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SING / 4") PORT_CODE(KEYCODE_4)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("PLAY / 5") PORT_CODE(KEYCODE_5)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("HELP / 6") PORT_CODE(KEYCODE_6)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SPEAK / 7") PORT_CODE(KEYCODE_7)

	PORT_START("KEY2")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("GAB / 8") PORT_CODE(KEYCODE_8)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("POET / 9") PORT_CODE(KEYCODE_9)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("DEMO / A") PORT_CODE(KEYCODE_A)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("GUARD / B") PORT_CODE(KEYCODE_B)

	PORT_START("KEY3")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("ALARM / C") PORT_CODE(KEYCODE_C)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("PLAN / D") PORT_CODE(KEYCODE_D)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SET UP / E") PORT_CODE(KEYCODE_E)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("ENTER / F") PORT_CODE(KEYCODE_F)

	PORT_START("LIGHT")
	PORT_ADJUSTER(50, "Light level")

	PORT_START("SOUND")
	PORT_ADJUSTER(0, "Sound level")

	PORT_START("SONAR")
	PORT_ADJUSTER(48, "Sonar distance in inches")

	PORT_START("SLEEP_NORM")
	PORT_BIT(0x01, 0x01, IPT_OTHER) PORT_NAME("Sleep/Norm") PORT_CODE(KEYCODE_F12) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(herojr_state::sleep_norm_changed), 0)

	PORT_START("RESET")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("RESET") PORT_CODE(KEYCODE_BACKSPACE) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(herojr_state::reset_changed), 0)
INPUT_PORTS_END

static DEVICE_INPUT_DEFAULTS_START(herojr_rs232)
	DEVICE_INPUT_DEFAULTS("RS232_TXBAUD", 0xff, RS232_BAUD_9600)
	DEVICE_INPUT_DEFAULTS("RS232_RXBAUD", 0xff, RS232_BAUD_9600)
	DEVICE_INPUT_DEFAULTS("RS232_DATABITS", 0xff, RS232_DATABITS_7)
	DEVICE_INPUT_DEFAULTS("RS232_PARITY", 0xff, RS232_PARITY_EVEN)
	DEVICE_INPUT_DEFAULTS("RS232_STOPBITS", 0xff, RS232_STOPBITS_1)
DEVICE_INPUT_DEFAULTS_END

void herojr_state::herojr(machine_config &config)
{
	M6808(config, m_maincpu, 3.579545_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &herojr_state::mem_map);

	MC146818(config, m_rtc, 32.768_kHz_XTAL);

	ACIA6850(config, m_acia, 0);
	m_acia->irq_handler().set(FUNC(herojr_state::acia_irq_w));
	m_acia->txd_handler().set(m_rs232, FUNC(rs232_port_device::write_txd));
	m_acia->rts_handler().set(m_rs232, FUNC(rs232_port_device::write_rts));

	clock_device &acia_clock(CLOCK(config, "acia_clock", 153600));
	acia_clock.signal_handler().set(m_acia, FUNC(acia6850_device::write_txc));
	acia_clock.signal_handler().append(m_acia, FUNC(acia6850_device::write_rxc));

	RS232_PORT(config, m_rs232, default_rs232_devices, "terminal");
	m_rs232->set_option_device_input_defaults("null_modem", DEVICE_INPUT_DEFAULTS_NAME(herojr_rs232));
	m_rs232->rxd_handler().set(m_acia, FUNC(acia6850_device::write_rxd));
	m_rs232->cts_handler().set(m_acia, FUNC(acia6850_device::write_cts));
	m_rs232->dsr_handler().set(m_acia, FUNC(acia6850_device::write_dcd));

	SPEAKER(config, "mono").front_center();

	PIA6821(config, m_u214);
	PIA6821(config, m_u215);

	VOTRAX_SC01(config, m_votrax, 720000); // TODO: verify HERO Jr reference pitch adjustment
	m_votrax->ar_callback().set(FUNC(herojr_state::speech_request_w));
	m_votrax->add_route(ALL_OUTPUTS, "mono", 0.5);

	GENERIC_CARTSLOT(config, m_cart, generic_plain_slot, nullptr, "bin,rom");
	m_cart->set_width(GENERIC_ROM8_WIDTH);
	m_cart->set_device_load(FUNC(herojr_state::cart_load));
}

ROM_START(herojr)
	ROM_REGION(0x8000, "maincpu", ROMREGION_ERASEFF)
	ROM_SYSTEM_BIOS(0, "v16", "System ROM v1.6")
	ROMX_LOAD("herojr_system_v16.bin", 0x0000, 0x8000, CRC(42a9500d) SHA1(f6ad61f54f1617fff9c3480d2d87a25c8f981aec), ROM_BIOS(0))
ROM_END

} // anonymous namespace

COMP(1984, herojr, 0, 0, herojr, herojr, herojr_state, empty_init, "Heathkit", "HERO Jr", MACHINE_IMPERFECT_SOUND | MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE)
