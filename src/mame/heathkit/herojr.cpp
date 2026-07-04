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
#include "machine/ram.h"
#include "osdcore.h"
#include "sound/votrax.h"

#include "speaker.h"

#include <algorithm>
#include <cstring>
#include <utility>


namespace {

bool driver_trace_enabled(const char *system)
{
	const char *const value = osd_getenv("HEATHKIT_HERO_DRIVER_TRACE");
	if (!value || !*value)
		return false;

	return !std::strcmp(value, "1")
		|| !std::strcmp(value, "true")
		|| !std::strcmp(value, "all")
		|| std::strstr(value, system) != nullptr;
}

class herojr_state : public driver_device
{
public:
	herojr_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_ram(*this, RAM_TAG),
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
		m_speech_request_flag_output(*this, "herojr_speech_request_flag"),
		m_speech_power(*this, "herojr_speech_power"),
		m_adc_sample(*this, "herojr_adc_sample"),
		m_adc_output(*this, "herojr_adc_output"),
		m_sonar_echo(*this, "herojr_sonar_echo"),
		m_sonar_distance_output(*this, "herojr_sonar_distance"),
		m_sonar_init_time_us(*this, "herojr_sonar_init_time_us"),
		m_sonar_echo_time_us(*this, "herojr_sonar_echo_time_us"),
		m_phoneme_seq(*this, "herojr_phoneme_seq"),
		m_phoneme_byte(*this, "herojr_phoneme_byte"),
		m_phoneme_time_us(*this, "herojr_phoneme_time_us"),
		m_phoneme_clips(*this, "herojr_phoneme_clips"),
		m_motor_left(*this, "herojr_motor_left"),
		m_motor_right(*this, "herojr_motor_right"),
		m_motor_head(*this, "herojr_motor_head"),
		m_motor_arm(*this, "herojr_motor_arm"),
		m_motion_detector(*this, "herojr_motion_detector"),
		m_wheel_feedback(*this, "herojr_wheel_feedback"),
		m_drive_activity(*this, "herojr_drive_activity"),
		m_rtc_sqw(*this, "herojr_rtc_sqw"),
		m_power_led(*this, "herojr_power_led"),
		m_power_on_time_us(*this, "herojr_power_on_time_us"),
		m_power_off_time_us(*this, "herojr_power_off_time_us"),
		m_power_cycles(*this, "herojr_power_cycles"),
		m_u214_control_b_output(*this, "herojr_u214_control_b"),
		m_rs232_status_output(*this, "herojr_rs232_status"),
		m_rs232_data_output(*this, "herojr_rs232_data"),
		m_ram_top(*this, "herojr_ram_top"),
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
	static constexpr s16 HEROJR_STEERING_LIMIT_STEPS = 32;

	DECLARE_DEVICE_IMAGE_LOAD_MEMBER(cart_load);
	void mem_map(address_map &map) ATTR_COLD;
	u8 sensor_debug_r(offs_t offset);
	void sensor_debug_w(offs_t offset, u8 data);
	u8 rtc_r(offs_t offset);
	u8 u214_port_a_r();
	u8 keypad_matrix_r();
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
	u8 steering_limit_mask() const;
	void update_steering_position(u8 data);
	void advance_wheel_feedback_sample();
	void update_drive_sonar_motion();
	u8 selected_adc_sample() const;
	void start_adc_conversion();
	void clock_adc_bit();
	void arm_sonar_cycle();
	void update_u214_input_outputs();
	void update_irq_line();
	void update_speech_power();
	void set_sleep_norm_input(bool norm);
	void reset_interface_state();
	void set_reset_line(bool asserted);
	void rtc_sqw_w(int state);
	void rtc_irq_w(int state);
	void update_u214_ca2(u8 control);
	void sleep_power_down();
	void sleep_wake(const char *cause);
	TIMER_CALLBACK_MEMBER(wheel_feedback_tick);
	TIMER_CALLBACK_MEMBER(sonar_echo_tick);

	template <typename... FormatParams>
	void driver_tracef(const char *format, FormatParams &&... args)
	{
		if (!m_driver_trace)
			return;

		osd_printf_info("heathkit_hero_driver[herojr]: ");
		osd_printf_info(format, std::forward<FormatParams>(args)...);
		osd_printf_info("\n");
	}

	required_device<m6808_cpu_device> m_maincpu;
	required_device<ram_device> m_ram;
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
	output_finder<> m_speech_request_flag_output;
	output_finder<> m_speech_power;
	output_finder<> m_adc_sample;
	output_finder<> m_adc_output;
	output_finder<> m_sonar_echo;
	output_finder<> m_sonar_distance_output;
	// Emulated-time telemetry (µs, masked into the s31 output range so the
	// double->s32 cast stays defined; wraps after ~35 emulated minutes,
	// harmless for delta use): G1J-06's 0.9-14.4 ms echo delays are
	// unresolvable from the host's ~34 ms bridge cadence, so the model
	// exposes its own INIT/echo stamps.
	output_finder<> m_sonar_init_time_us;
	output_finder<> m_sonar_echo_time_us;
	// Phoneme telemetry (Phase 2.2d): latch sequence counter, latched byte,
	// emulated-time stamp, and clip counter (latch-while-busy events).
	output_finder<> m_phoneme_seq;
	output_finder<> m_phoneme_byte;
	output_finder<> m_phoneme_time_us;
	output_finder<> m_phoneme_clips;

	static s32 emulated_time_us(const attotime &now)
	{
		return s32(u64(now.as_double() * 1e6) & 0x7fffffff);
	}
	output_finder<> m_motor_left;
	output_finder<> m_motor_right;
	output_finder<> m_motor_head;
	output_finder<> m_motor_arm;
	output_finder<> m_motion_detector;
	output_finder<> m_wheel_feedback;
	output_finder<> m_drive_activity;
	output_finder<> m_rtc_sqw;
	// Modeled Vcc / green POWER LED (G2J-08). The green LED is the +5 Vcc
	// power indicator, not a port-driven output (JR-OG startup step 3 "The
	// green POWER LED will light"; JR-OG troubleshooting "green LED blinks
	// only every five seconds" = sleep). LED state IS power state — the
	// 5-second sleep flash is produced by the FIRMWARE's catnap re-arm chain
	// ($9FE9 → $D26E/$D298 → $EC1E), never scripted here. The µs stamps and
	// wake-cycle counter exist because a catnap wake pulse (boot-to-$EC2F)
	// is shorter than the bridge's ~34 ms snapshot cadence — same telemetry
	// pattern as the sonar INIT/echo stamps above.
	output_finder<> m_power_led;
	output_finder<> m_power_on_time_us;
	output_finder<> m_power_off_time_us;
	output_finder<> m_power_cycles;
	// U214 CRB write latch (low six bits) for the bridge io snapshot: the
	// $D823 read handler advances the wheel-feedback sample as a model side
	// effect, so the observer composes from this output instead of reading
	// the bus (plugin herojr_d823_snapshot).
	output_finder<> m_u214_control_b_output;
	output_finder<> m_rs232_status_output;
	output_finder<> m_rs232_data_output;
	// Top of populated RAM ($07FF stock / $3FFF expanded) for bridge truth.
	output_finder<> m_ram_top;
	output_finder<8> m_port_outputs;

	u8 m_u214_port_a = 0xff;
	u8 m_u214_port_b = 0xff;
	u8 m_u214_control_a = 0;
	u8 m_u214_control_b = 0;
	u8 m_motion_detector_state = 0;
	// U213 SQW pin level, driven by the mc146818 device's own sqw()
	// callback (256 Hz once the ROM programs reg $0A = $28 and SQWE at
	// $ED86 — hero-jr-rtc-spec.md §2.2; no driver rate constant exists).
	u8 m_rtc_sqw_level = 0;
	// U214 IRQA1: the 6821-latched CA1 flag ($D822 bit 7). Set on the CA1
	// active transition (SQW through the U211D inverter, spec §1.4),
	// cleared ONLY by a $D820 Peripheral Register A read (spec §2.3).
	u8 m_u214_ca1_flag = 0;
	// U214 CA2 (pin 39) modeled level: CRA bit 5 = 1, bit 4 = 1 puts CA2 in
	// set/reset output mode with bit 3 as the level (6821 datasheet). The
	// v1.6 ROM's exhaustive $D822 writer census is three STAA instructions:
	// $EB8B (#$3B) and $EB95 (#$3F) — awake init, CA2 high — and $EC2F
	// (#$37) — the sleep entry, the ONLY CA2-low write in the image. An
	// undriven CA2 (reset/input mode) reads as high here: the latch below,
	// not CA2's live level, holds the sleep state, and every operator boot
	// crosses the pre-$EB8B window with CA2 undriven, so undriven ⇒ awake
	// is a behavioral necessity (G2J-08 adjudication 2026-07-04).
	u8 m_u214_ca2 = 1;
	// U222 NAND-A/B sleep latch (wire-walked truth, conformance/
	// AUDIT-2026-06.md commit ec6475d): U214 CA2 pin 39 → U222 NAND-A →
	// NAND-B → U221 inverter → P201-14/P301-14 "+5 SHUTDOWN" → U302
	// (LM3524) pin 10. Three inversions: CA2 LOW ⇒ logic 1 on U302-10 ⇒
	// switching regulator (Vcc) off — JR-TM printed p. 21 / PDF p. 23
	// verbatim: "the CPU normally keeps the shutdown line (U302-10) at
	// logic 0 … the CPU will place a logic 1 on this line. This logic 1
	// turns off Q301 to remove the 5-volt logic supply during the sleep
	// mode." NAND-B out 9 feeds back into NAND-A pins 4/5, so the state
	// LATCHES and survives CA2/CPU power loss; wake pulls a pulled-up
	// (R209/R214) NAND-B input low from the RTC-IRQ*/reset side.
	// true = latched off (Vcc down, CPU + interface ICs dead, Vca domain —
	// RTC, retained RAM — alive).
	bool m_sleep_latched = false;
	// U213 IRQ* (pin 19) level as reported by the mc146818 irq() callback
	// (1 = asserted/low). It feeds the U222 wake input through the R209
	// pull-up (JR-TM printed p. 28 / PDF p. 30: "The IRQ interrupt output
	// on U213-19 is used in the sleep mode to pulse the reset circuitry"),
	// NOT the CPU IRQ net. While asserted it holds the wake-side NAND-B
	// input low, so the latch can neither hold nor enter the sleep state.
	u8 m_rtc_irq_asserted = 0;
	u32 m_power_cycle_count = 0;
	u8 m_acia_irq_state = 0;
	u8 m_speech_data = 0;
	u8 m_u215_ddr_a = 0;
	u8 m_u215_port_b = 0;
	u8 m_u215_control_a = 0;
	u8 m_u215_control_b = 0;
	u8 m_u215_ddr_b = 0;
	u8 m_adc_shift = 0;
	u8 m_adc_bits_remaining = 0;
	u8 m_adc_output_state = 0;
	u8 m_sonar_echo_state = 0;
	u32 m_phoneme_seq_count = 0;
	u32 m_phoneme_clip_count = 0;
	u8 m_wheel_feedback_sample = 0;
	u16 m_wheel_feedback_port_a_count = 0;
	u8 m_light_sample = 50;
	u8 m_sound_sample = 0;
	u8 m_sonar_distance_sample = 48;
	u8 m_speech_power_state = 0;
	u8 m_speech_request = 1;
	u8 m_speech_request_flag = 0;
	u8 m_speech_strobe_state = 0;
	u8 m_rs232_status = 0;
	u8 m_rs232_data = 0;
	u32 m_drive_activity_count = 0;
	s16 m_steering_position = 0;
	u8 m_steering_phase = 0;
	bool m_driver_trace = false;
	emu_timer *m_wheel_feedback_timer = nullptr;
	emu_timer *m_sonar_echo_timer = nullptr;
};

void herojr_state::machine_start()
{
	m_driver_trace = driver_trace_enabled("herojr");

	// Install the documented socket population (see mem_map comment).
	address_space &program = m_maincpu->space(AS_PROGRAM);
	if (m_ram->size() == 0x4000)
	{
		// Expanded BASIC-era unit: 8K at U203 ($0000-$1FFF) + 8K at U204
		// ($2000-$3FFF); U205 absent, $4000-$5FFF open bus.
		program.install_ram(0x0000, 0x3fff, m_ram->pointer());
	}
	else
	{
		// Stock build: one 2K 6116 at U203; A11/A12 unconnected to the part,
		// image mirrors 4x across the decoded $0000-$1FFF window.
		program.install_ram(0x0000, 0x07ff, 0x1800, m_ram->pointer());
	}
	m_ram_top.resolve();
	m_ram_top = (m_ram->size() == 0x4000) ? 0x3fff : 0x07ff;

	m_speech_phoneme.resolve();
	m_speech_inflection.resolve();
	m_speech_strobe.resolve();
	m_speech_ready.resolve();
	m_speech_request_flag_output.resolve();
	m_speech_power.resolve();
	m_adc_sample.resolve();
	m_adc_output.resolve();
	m_sonar_echo.resolve();
	m_sonar_distance_output.resolve();
	m_sonar_init_time_us.resolve();
	m_sonar_echo_time_us.resolve();
	m_phoneme_seq.resolve();
	m_phoneme_byte.resolve();
	m_phoneme_time_us.resolve();
	m_phoneme_clips.resolve();
	m_motor_left.resolve();
	m_motor_right.resolve();
	m_motor_head.resolve();
	m_motor_arm.resolve();
	m_motion_detector.resolve();
	m_wheel_feedback.resolve();
	m_drive_activity.resolve();
	m_rtc_sqw.resolve();
	m_power_led.resolve();
	m_power_on_time_us.resolve();
	m_power_off_time_us.resolve();
	m_power_cycles.resolve();
	m_u214_control_b_output.resolve();
	m_rs232_status_output.resolve();
	m_rs232_data_output.resolve();
	m_port_outputs.resolve();

	const char *const initial_sleep = osd_getenv("HEATHKIT_HEROJR_INITIAL_SLEEP");
	if (initial_sleep && !std::strcmp(initial_sleep, "1"))
		set_sleep_norm_input(false);
	driver_tracef("machine_start trace enabled initial_sleep=%s", initial_sleep ? initial_sleep : "");

	save_item(NAME(m_u214_port_a));
	save_item(NAME(m_u214_port_b));
	save_item(NAME(m_u214_control_a));
	save_item(NAME(m_u214_control_b));
	save_item(NAME(m_motion_detector_state));
	save_item(NAME(m_rtc_sqw_level));
	save_item(NAME(m_u214_ca1_flag));
	save_item(NAME(m_u214_ca2));
	save_item(NAME(m_sleep_latched));
	save_item(NAME(m_rtc_irq_asserted));
	save_item(NAME(m_power_cycle_count));
	save_item(NAME(m_acia_irq_state));
	save_item(NAME(m_speech_data));
	save_item(NAME(m_u215_ddr_a));
	save_item(NAME(m_u215_port_b));
	save_item(NAME(m_u215_control_a));
	save_item(NAME(m_u215_control_b));
	save_item(NAME(m_u215_ddr_b));
	save_item(NAME(m_adc_shift));
	save_item(NAME(m_adc_bits_remaining));
	save_item(NAME(m_adc_output_state));
	save_item(NAME(m_sonar_echo_state));
	save_item(NAME(m_phoneme_seq_count));
	save_item(NAME(m_phoneme_clip_count));
	save_item(NAME(m_wheel_feedback_sample));
	save_item(NAME(m_wheel_feedback_port_a_count));
	save_item(NAME(m_light_sample));
	save_item(NAME(m_sound_sample));
	save_item(NAME(m_sonar_distance_sample));
	save_item(NAME(m_speech_power_state));
	save_item(NAME(m_speech_request));
	save_item(NAME(m_speech_request_flag));
	save_item(NAME(m_speech_strobe_state));
	save_item(NAME(m_rs232_status));
	save_item(NAME(m_rs232_data));
	save_item(NAME(m_drive_activity_count));
	save_item(NAME(m_steering_position));
	save_item(NAME(m_steering_phase));

	m_wheel_feedback_timer = timer_alloc(FUNC(herojr_state::wheel_feedback_tick), this);
	m_sonar_echo_timer = timer_alloc(FUNC(herojr_state::sonar_echo_tick), this);
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
	m_u214_port_b = 0x00;
	m_u214_control_a = 0;
	m_u214_control_b = 0;
	m_u214_ca1_flag = 0; // 6821 reset clears the interrupt flags
	m_u214_ca2 = 1; // reset puts CA2 in input mode: undriven ⇒ awake side
	m_motion_detector_state = 0;
	m_speech_data = 0;
	m_u215_ddr_a = 0;
	m_u215_port_b = 0;
	m_u215_control_a = 0;
	m_u215_control_b = 0;
	m_u215_ddr_b = 0;
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
	m_speech_request_flag = 0;
	m_speech_request_flag_output = 0;
	m_speech_strobe_state = 0;
	m_drive_activity_count = 0;
	m_steering_position = 0;
	m_steering_phase = 0;
	m_speech_phoneme = 0;
	m_speech_inflection = 0;
	m_speech_strobe = 0;
	m_speech_ready = 1;
	m_speech_power = 0;
	m_adc_sample = 0;
	m_adc_output = 0;
	m_sonar_echo = 0;
	m_sonar_distance_output = m_sonar_distance_sample;
	m_sonar_init_time_us = 0;
	m_sonar_echo_time_us = 0;
	m_phoneme_seq_count = 0;
	m_phoneme_clip_count = 0;
	m_phoneme_seq = 0;
	m_phoneme_byte = 0;
	m_phoneme_time_us = 0;
	m_phoneme_clips = 0;
	m_votrax->reset();
	update_speech_power();
	m_motor_left = 0;
	m_motor_right = 0;
	m_motor_head = 0;
	m_motor_arm = 0;
	m_motion_detector = 0;
	m_wheel_feedback = 0;
	m_drive_activity = 0;
	m_u214_control_b_output = 0;
	for (int port = 0; port < 8; port++)
		m_port_outputs[port] = 0;
	m_sonar_echo_timer->adjust(attotime::never);
}

// Latched-off consequence (G2J-08, blessed 2026-07-04): Vcc drops, so the
// CPU and interface peripherals lose state and sit dead until wake. The Vca
// domain survives untouched: the RTC keeps time with the armed alarm (JR-TM
// printed p. 17 / PDF p. 19 — SW1 keeps U209/Vca alive "even with the Robot
// in the sleep mode … power being applied to the real-time clock, reset
// circuits, and address decoder"), and RAM contents persist, modeling the
// JR-MAP "Retained during sleep" supply jumpering (stock J202 D-E, expanded
// J202/J204 A-B + D-E) — the firmware's own sleep markers $0085/$0086 are
// read back at $A0C1 on wake, so Heath's sleep feature presumes retention.
// The CPU is held via INPUT_LINE_RESET (suspend-in-reset), which is also
// the physical wake shape: "After the CPU comes out of the sleep mode, it
// goes to the high end of memory (ROM)" — no resumed execution context.
// The I/O window stays mapped: the held CPU cannot execute, so the only
// possible accessors are the debugger/bridge apertures (sanctioned test
// stimulus against the parked machine, G1J-06/G1J-07 re-shape blessing).
void herojr_state::sleep_power_down()
{
	m_sleep_latched = true;
	m_power_led = 0;
	m_power_off_time_us = emulated_time_us(machine().time());
	driver_tracef("sleep_power_down time_us=%d rtc_irq=%u", s32(m_power_off_time_us), m_rtc_irq_asserted);
	// Vcc-domain state loss: PIAs, SC-01, ADC/sonar models, ACIA.
	reset_interface_state();
	m_acia->reset();
	m_acia_irq_state = 0;
	m_rs232_status = 0;
	m_rs232_data = 0;
	m_rs232_status_output = 0;
	m_rs232_data_output = 0;
	update_irq_line();
	m_maincpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
}

// Wake = latch clear → regulator on → RESET-vector boot (rtc-spec §2.8
// chain). The wake reset pulse reaches "the CPU and interface ICs" (JR-TM
// printed p. 28 / PDF p. 30, verbatim) — NOT the RTC: the chip-level AF
// flag must survive to the boot's single $A0C1 read (spec §2.7 CONFIRMED
// wake adjudication), and the spec's own chain has IRQ* released by the
// boot re-init $ED86 clearing AIE, not by RESET*. Only the operator RESET
// key asserts RTC RESET* (set_reset_line below — JR-OG printed pp. 23/38:
// pressing RESET also cancels the Alarm). Interface state was already
// destroyed at power-down; releasing the held reset line boots the CPU
// through $FFFE = $9FBE. The firmware then re-arms alarm = (now + 5) s and
// re-sleeps if SW2 still reads SLEEP ($9FE9 → $D26E/$D298 → $EC1E) — THE
// FIRMWARE produces the 5-second green-LED flash train; this model only
// provides the power/latch/reset physics.
void herojr_state::sleep_wake(const char *cause)
{
	m_sleep_latched = false;
	m_power_cycle_count++;
	m_power_led = 1;
	m_power_on_time_us = emulated_time_us(machine().time());
	m_power_cycles = s32(m_power_cycle_count & 0x7fffffff);
	driver_tracef("sleep_wake cause=%s cycle=%u time_us=%d", cause, m_power_cycle_count, s32(m_power_on_time_us));
	m_maincpu->set_input_line(INPUT_LINE_RESET, CLEAR_LINE);
}

// U213 IRQ* (pin 19) → R209 pull-up → U222 wake input (JR-TM printed p. 28
// / PDF p. 30; rtc-spec §1.4: a power/wake control, never a CPU interrupt).
// Asleep, an alarm match (AF ∧ AIE) pulls the wake input low, the U222
// latch clears, and the regulator restarts — the 5-second catnap pulse.
// Awake, the line is inert (the latch is already in the awake state and
// the ROM never arms an interrupt outside the sleep path, spec §2.8).
void herojr_state::rtc_irq_w(int state)
{
	const u8 previous = m_rtc_irq_asserted;
	m_rtc_irq_asserted = state ? 1 : 0;
	if (m_rtc_irq_asserted && m_sleep_latched)
		sleep_wake("rtc_irq");
	else if (previous && !m_rtc_irq_asserted && !m_sleep_latched && !m_u214_ca2)
		// IRQ* release with CA2 still driven low: the held-high wake input
		// drops away and the latch falls into the sleep state (physical
		// completeness — the v1.6 ROM never reaches this ordering: $ED86
		// clears AIE at boot, long before the $EC2F CA2-low write).
		sleep_power_down();
}

// U214 CA2 follows CRA per the 6821 datasheet: bits 5/4 = 1 select
// set/reset output mode with bit 3 as the driven level. A falling edge
// while powered sets the U222 sleep latch (the ROM's $EC2F CRA = $37 sleep
// entry); the latch cannot set while the RTC IRQ* wake input is asserted
// (it holds NAND-B's output — net V — high regardless, AUDIT ec6475d
// polarity walk). Undriven CA2 never sets the latch (see m_u214_ca2 note).
void herojr_state::update_u214_ca2(u8 control)
{
	const bool driven = BIT(control, 5) && BIT(control, 4);
	const u8 level = driven ? (BIT(control, 3) ? 1 : 0) : 1;
	const u8 previous = m_u214_ca2;
	m_u214_ca2 = level;
	if (previous && !level && !m_sleep_latched && !m_rtc_irq_asserted)
		sleep_power_down();
}

void herojr_state::set_reset_line(bool asserted)
{
	if (asserted)
	{
		m_maincpu->resume(SUSPEND_REASON_HALT);
		reset_interface_state();
		// U213 RESET* (pin 18) sits on the same system reset net that
		// "resets the CPU and interface ICs" (JR-TM printed p. 28 / PDF
		// p. 30; hero-jr-rtc-spec.md §1.4), so the panel RESET press — and
		// the bridge's reset_machine, which presses this same modeled
		// input — must reach the chip. MAME's device_reset() implements
		// the datasheet RESET* rule: PIE/AIE/UIE/SQWE clear, time/calendar
		// /NVRAM survive. Operator-visible consequence: pressing RESET
		// cancels a pending wake alarm (JR-OG printed pp. 23, 38 via
		// JR-PWR; spec §3.2 honesty gap closed 2026-07-03).
		m_rtc->reset();
		update_irq_line();
	}
	m_maincpu->set_input_line(INPUT_LINE_RESET, asserted ? ASSERT_LINE : CLEAR_LINE);
}

void herojr_state::machine_reset()
{
	// Power-up: the U222 latch settles in the awake state — its wake-side
	// inputs are R209/R214 10K pull-ups and CA2 is undriven until the ROM's
	// $EB8B init (undriven ⇒ on: every operator boot crosses that window;
	// G2J-08 adjudication 2026-07-04). Green LED = modeled Vcc, on.
	m_sleep_latched = false;
	m_rtc_irq_asserted = 0;
	m_power_cycle_count = 0;
	m_power_led = 1;
	m_power_on_time_us = 0;
	m_power_off_time_us = 0;
	m_power_cycles = 0;
	// A machine reset is a power cycle: release a sleep-held CPU reset line
	// so the core boots normally (the panel RESET input path manages the
	// line itself through set_reset_line).
	m_maincpu->set_input_line(INPUT_LINE_RESET, CLEAR_LINE);
	reset_interface_state();
	m_acia_irq_state = 0;
	m_rs232_status = 0;
	m_rs232_data = 0;
	m_rtc_sqw = m_rtc_sqw_level;
	m_rs232_status_output = 0;
	m_rs232_data_output = 0;
	// No RTC register scaffolding here: on real hardware registers $0A/$0B
	// are Vca-retained and RESET* only clears PIE/AIE/UIE/SQWE (the
	// mc146818 device_reset already models that). The ROM re-programs both
	// registers unconditionally on every reset-vector pass ($EB51 → $ED86:
	// $0A = $28, $0B = $0C|DSE — hero-jr-rtc-spec.md §2.2/§3.2; the old
	// $26/$02 seed contradicted those ROM values and was dead within
	// milliseconds).
	update_irq_line();
	// Wheel-feedback/world-model pacing timer. 1024 Hz is this driver's
	// pre-existing mechanical-model constant (2048-tick coast tail ≈ 2 s,
	// G1J-04/G1J-05 measured basis) — it is NOT the RTC tick: the real
	// SQW interrupt path runs at 256 Hz through the mc146818 sqw()
	// callback (rtc_sqw_w), and tying the drive model to it would silently
	// re-time drive pacing 4x (spec §3.1 co-move note).
	m_wheel_feedback_timer->adjust(attotime::from_hz(1024), 0, attotime::from_hz(1024));
	driver_tracef("machine_reset sleep_norm=%u speech_request=%u rtc_sqw=%u", m_sleep_norm->read(), m_speech_request, m_rtc_sqw_level);
}

INPUT_CHANGED_MEMBER(herojr_state::sleep_norm_changed)
{
	driver_tracef("sleep_norm_changed old=%d new=%d", oldval, newval);
	// SW2 is read by firmware at $D841 D6 and has NO direct wire into the
	// U222 sleep latch (G2J-08 switch-edge adjudication, 2026-07-04):
	// JR-OG printed p. 38 documents sleep with SW2 in NORM ("It will also
	// go to sleep at times even while it is performing many of the other
	// operations"), so a NORM-level latch clear is physically impossible,
	// no edge-coupling component exists on the wire-walked wake nets
	// (AUDIT ec6475d), and JR-TM printed p. 21 has the SLEEP direction
	// CPU-mediated too ("the CPU will place a logic 1 on this line").
	// Sliding to NORM wakes the robot (JR-OG p. 38) through the honest
	// chain: the next ≤5 s catnap alarm pulse boots the CPU, its $9FE9
	// check reads NORM, and the boot continues instead of re-sleeping —
	// "usually, it will say something upon awakening" is that full boot's
	// utterance. Do not turn the switch edge into a reset or latch clear.
}

INPUT_CHANGED_MEMBER(herojr_state::reset_changed)
{
	driver_tracef("reset_changed old=%d new=%d", oldval, newval);
	if (oldval == newval)
		return;
	// The RESET key reaches the U222 wake side directly ("When the reset
	// line is pulled low, U222A-9 goes high … turn on the 5-volt switching
	// regulator", JR-TM printed p. 28 / PDF p. 30), so a press clears the
	// sleep latch; the ensuing set_reset_line(true) also asserts RTC
	// RESET* — the operator RESET is the one wake path that cancels a
	// pending alarm at the chip (JR-OG printed pp. 23/38).
	if (newval != 0 && m_sleep_latched)
		sleep_wake("reset_key");
	set_reset_line(newval != 0);
}

void herojr_state::mem_map(address_map &map)
{
	// U203/U204/U205 socket population is per documented build option and is
	// installed in machine_start() from the RAM device size (-ramsize):
	//   stock    (2K, default): one 6116 at U203 only; U204/U205 ship empty
	//            (JR-TM appendix printed pp. 43-44, parts list p. 39;
	//            JR-ILL Pictorial 3-4 "NOT USED").
	//   expanded (16K): 8K parts at U203+U204 for BASIC-era units; U205 empty
	//            (RTC-1-8 printed p. 9 pins BASIC's work area above $0800).
	// The U202 decoder PROM (444-295) selects each socket for its full 8K
	// window with no per-size qualification — JR-SCH sheet 3: A11/A12 enter
	// U202 only so U208's select can exclude the $D800-$DFFF I/O hole, and
	// J201/J203/J205 merely reroute socket pin 23 between A11 (8K part) and
	// WE* (2K part) — so a 2K part's image repeats 4x across its window and
	// an absent socket's select drives no device (open bus; modeled as
	// unmapped, see hardware-notes.md).
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
		// Erased-NVRAM guard for the ROM's VRT cold/warm decision only.
		// Register A reads pass through honestly — machine_reset's direct
		// writes keep it valid, and the device models the real update
		// cycle, so UIP (bit 7) asserts for 244+1984 us before each 1 Hz
		// update exactly as the firmware's documented wait expects (G1J-09;
		// the old unconditional UIP mask is gone).
		if (address == 0x0d && data == 0xff)
			return 0x00;
		return data;
	}

	return m_rtc->get_address();
}

void herojr_state::rtc_w(offs_t offset, u8 data)
{
	driver_tracef("rtc_w offset=$%X data=$%02X register_select=%u", unsigned(offset & 0x0f), data, BIT(offset, 0));
	if (BIT(offset, 0))
		m_rtc->data_w(data);
	else
		m_rtc->address_w(data);
}

u8 herojr_state::u214_port_a_r()
{
	const u8 data = keypad_matrix_r();
	// 6821 datasheet: a Peripheral Register A read clears the latched
	// IRQA1 flag (and releases IRQA*). This is the ONLY clear on real
	// silicon, and the v1.6 ROM leans on it: the $D17F tick service's
	// keyboard scan ($D1C5) reads $D820, consuming exactly one RTC SQW
	// tick per service pass — the 256/s cadence of hero-jr-rtc-spec.md
	// §2.3. (U214 has no DDR routing model — tech-debt item 7 — so every
	// read here is a peripheral-register read.) Debugger/UI reads are
	// guarded, matching the U215 request-flag and sonar-echo clears.
	if (!machine().side_effects_disabled() && m_u214_ca1_flag)
	{
		m_u214_ca1_flag = 0;
		update_irq_line();
	}
	driver_tracef("u214_port_a_r data=$%02X port_a=$%02X wheel_feedback=%u feedback_count=%u", data, m_u214_port_a, m_wheel_feedback_sample, m_wheel_feedback_port_a_count);
	return data;
}

// JR-TM p. 24: 4x4 active-low matrix on $D820 D0-D7 (columns D0-D3, rows
// D4-D7). The ROM scans by writing drive patterns and reading back ($D1C5
// in v1.6: $F0, $0F, then $FF with XOR detection); a held key — sourced
// ONLY from the ioport fields — reflects each driven-low line onto its
// crossing line. Bridge key injection sets the same ioport fields a human
// keypress sets; there is no other injection channel.
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
			if (!BIT(row_bits, column))
				continue;

			if (!BIT(data, column))
				data &= ~row_line;
			if (!BIT(data, row + 4))
				data &= ~column_line;
		}
	}

	data &= ~steering_limit_mask();
	return data;
}

u8 herojr_state::u214_port_b_r()
{
	return (m_u214_port_b & 0x7f) | (m_motion_detector_state ? 0x00 : 0x80);
}

bool herojr_state::drive_feedback_active() const
{
	return BIT(m_u214_port_b, 1);
}

u8 herojr_state::steering_limit_mask() const
{
	if (m_steering_phase == 0)
		return 0x00;
	if (m_steering_position >= HEROJR_STEERING_LIMIT_STEPS)
		return 0x80;
	if (m_steering_position <= -HEROJR_STEERING_LIMIT_STEPS)
		return 0x40;
	return 0x00;
}

void herojr_state::update_steering_position(u8 data)
{
	const u8 phase = data & 0x3c;
	const auto phase_index = [](u8 value) -> int
	{
		switch (value)
		{
		case 0x0c: return 0;
		case 0x18: return 1;
		case 0x30: return 2;
		case 0x24: return 3;
		default: return -1;
		}
	};

	const int previous_index = phase_index(m_steering_phase);
	const int current_index = phase_index(phase);
	if (current_index < 0)
	{
		if (phase == 0)
			m_steering_phase = 0;
		return;
	}

	if (previous_index >= 0 && previous_index != current_index)
	{
		const int delta = (current_index - previous_index + 4) & 0x03;
		if (delta == 1)
			m_steering_position = std::min<s16>(HEROJR_STEERING_LIMIT_STEPS, m_steering_position + 1);
		else if (delta == 3)
			m_steering_position = std::max<s16>(-HEROJR_STEERING_LIMIT_STEPS, m_steering_position - 1);
	}

	m_steering_phase = phase;
}

void herojr_state::advance_wheel_feedback_sample()
{
	m_wheel_feedback_sample = (m_wheel_feedback_sample + 1) & 0x03;
	m_wheel_feedback = m_wheel_feedback_sample;
	driver_tracef("advance_wheel_feedback_sample sample=%u active=%u feedback_count=%u port_b=$%02X", m_wheel_feedback_sample, drive_feedback_active() ? 1 : 0, m_wheel_feedback_port_a_count, m_u214_port_b);
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

// Pure output-latch write: every byte updates the scan latch, exactly as
// the ROM's $D1C5 scan routine expects. (The old write-sniffing heuristic
// that interpreted certain bit patterns as bridge key injections is gone —
// keys come only from ioport fields.)
void herojr_state::u214_port_a_w(u8 data)
{
	m_u214_port_a = data;
}

void herojr_state::u214_port_b_w(u8 data)
{
	m_u214_port_b = data & 0x7f;
	update_steering_position(data);
	if ((m_u214_port_b & 0x3e) != 0)
	{
		m_drive_activity_count++;
		m_drive_activity = m_drive_activity_count;
	}
	if (drive_feedback_active())
	{
		m_wheel_feedback_port_a_count = 2048;
		advance_wheel_feedback_sample();
	}
	else
	{
		m_wheel_feedback_port_a_count = 0;
		m_wheel_feedback_sample = 0;
		m_wheel_feedback = 0;
	}
	// HERO Jr Technical Manual: $D821 D1 controls main drive motor A2,
	// D0 controls relay RY301 for direction, and D2-D5 drive steering phases.
	m_motor_left = BIT(data, 1) ? 1 : 0;
	m_motor_right = BIT(data, 0) ? 1 : 0;
	m_port_outputs[3] = (data & 0x3c) << 2;
	m_port_outputs[5] = (BIT(data, 1) ? 0x40 : 0x00) | (BIT(data, 0) ? 0x80 : 0x00);
	driver_tracef("u214_port_b_w data=$%02X latched=$%02X drive_activity=%u wheel_feedback=%u feedback_count=%u steering_position=%d steering_limit=$%02X", data, m_u214_port_b, m_drive_activity_count, m_wheel_feedback_sample, m_wheel_feedback_port_a_count, m_steering_position, steering_limit_mask());
	update_u214_input_outputs();
}

// $D822 = U214 CRA readback, real 6821 semantics (mirrors U215's $D842):
// bits 5-0 return the written control byte; bit 6 (IRQA2) reads 0 because
// every CRA value the v1.6 ROM writes has bit 5 = 1 (CA2 in output mode,
// which holds IRQA2 clear per the datasheet); bit 7 is the LATCHED IRQA1
// flag — the RTC SQW tick through U211D (spec §1.4) — visible regardless
// of CRA bit 0 (that bit only gates the IRQA* line), so the firmware's
// polled reads see pending ticks even with interrupts disabled. A CRA
// read never clears the flag; only the $D820 port A data read does.
// (Previously this returned the raw SQW toggle LEVEL, which — with the
// level-held IRQ — produced the measured 1927.7/s burst service instead
// of the hardware's one-per-period 256/s; spec §3.1.)
u8 herojr_state::u214_control_a_r()
{
	return (m_u214_control_a & 0x3f) | (m_u214_ca1_flag ? 0x80 : 0x00);
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
	// CA2 (pin 39) drives the U222 sleep latch — the ROM's $EC2F CRA = $37
	// write is the physical sleep entry (G2J-08). If it fires, the machine
	// is latched off before the write below matters (state loss).
	update_u214_ca2(data);
	// CRA bit 0 gates IRQA* onto the CPU IRQ net (both U214 interrupt
	// outputs sit on it, JR-SCH sheet 3 / spec §1.4), so a CRA rewrite can
	// mask or expose a pending latched flag.
	update_irq_line();
}

void herojr_state::u214_control_b_w(u8 data)
{
	const u8 previous_control = m_u214_control_b;
	m_u214_control_b = data & 0x3f;
	m_u214_control_b_output = m_u214_control_b;
	if (!BIT(previous_control, 3) && BIT(m_u214_control_b, 3))
		arm_sonar_cycle();
}

void herojr_state::update_u214_input_outputs()
{
	m_motion_detector = m_motion_detector_state;
	if (!drive_feedback_active() && m_wheel_feedback_port_a_count == 0)
	{
		m_wheel_feedback_sample = 0;
		m_wheel_feedback = 0;
	}
}

// CPU IRQ* net: U214 IRQA*/IRQB* and the ACIA interrupt share it (JR-SCH
// sheet 3; spec §1.4). U214's contribution is the 6821 rule — latched flag
// AND its CRA enable bit. The RTC's own IRQ* pin (U213-19) is NOT on this
// net: it feeds the sleep-mode wake circuit (U222A) only, which is G2J-08
// scope. (U215's IRQA wiring to the CPU awaits JR-TM adjudication,
// tech-debt §2 — unchanged here.)
void herojr_state::update_irq_line()
{
	const bool u214_irq_a = m_u214_ca1_flag && BIT(m_u214_control_a, 0);
	m_maincpu->set_input_line(INPUT_LINE_IRQ0, (u214_irq_a || m_acia_irq_state) ? ASSERT_LINE : CLEAR_LINE);
}

// U213 SQW (pin 23) → U211D inverter → U214 CA1 (pin 40): "The output of
// one of the internal dividers (SQW) of U213-23 is routed through U211D to
// interrupt input on U214-40" (JR-TM printed p. 28 / PDF p. 30, verbatim).
// The rate is chip-derived — the ROM's own $ED86 boot init programs
// reg $0A = $28 (DV=010, RS=1000 → 256 Hz) and sets SQWE (spec §2.2), so
// this callback carries no driver frequency constant, and SQWE honestly
// gates it (RESET* clears SQWE, silencing ticks until the ROM re-inits).
// The 6821 IRQA1 flag latches on the CA1 transition selected by CRA
// bit 1 (every CRA value the v1.6 ROM writes has bit 1 = 1: active RISING
// CA1 = falling SQW through the inverter) — one latched tick per SQW
// period, cleared only by the $D820 port A data read (u214_port_a_r).
// This replaces the deleted hand 1024 Hz toggle timer whose level-held
// IRQ measured 1927.7 serviced ticks/s (spec §3.1, probe 2026-07-03).
void herojr_state::rtc_sqw_w(int state)
{
	const u8 level = state ? 1 : 0;
	if (level == m_rtc_sqw_level)
		return;
	m_rtc_sqw_level = level;
	m_rtc_sqw = m_rtc_sqw_level;
	// The SQW pin itself is Vca-domain (U213 keeps running asleep), but
	// U211D/U214 are Vcc-powered: while the U222 latch holds the regulator
	// off no CA1 flag can set (G2J-08 power model).
	if (m_sleep_latched)
		return;
	const u8 ca1_level = m_rtc_sqw_level ? 0 : 1; // U211D inversion
	const bool ca1_active_rising = BIT(m_u214_control_a, 1);
	if ((ca1_active_rising && ca1_level) || (!ca1_active_rising && !ca1_level))
	{
		m_u214_ca1_flag = 1;
		update_irq_line();
	}
}

// Mechanical world-model pacing (wheel feedback + drive-coupled sonar
// distance). 1024 Hz and the 2048-tick coast tail are this driver's
// pre-existing model constants (G1J-04/G1J-05 measured basis); the timer
// deliberately does NOT touch the CPU IRQ line or the RTC tick path —
// those belong to rtc_sqw_w above (spec §3.1 co-move).
TIMER_CALLBACK_MEMBER(herojr_state::wheel_feedback_tick)
{
	if (drive_feedback_active() || m_wheel_feedback_port_a_count != 0)
	{
		advance_wheel_feedback_sample();
		if (!drive_feedback_active() && m_wheel_feedback_port_a_count != 0)
			m_wheel_feedback_port_a_count--;
	}
	update_drive_sonar_motion();
	update_u214_input_outputs();
}

u8 herojr_state::u215_speech_data_r()
{
	if (!BIT(m_u215_control_a, 2))
		return m_u215_ddr_a;

	if (!machine().side_effects_disabled())
	{
		m_speech_request_flag = 0;
		m_speech_request_flag_output = 0;
	}
	return m_speech_data;
}

u8 herojr_state::u215_speech_power_r()
{
	if (!BIT(m_u215_control_b, 2))
		return m_u215_ddr_b;

	// 6821 datasheet: a Peripheral Register B read (CRB bit 2 = 1, this
	// branch) clears the latched IRQB1 flag — the $D843 bit-7 sonar echo,
	// set by the CB1 edge. That read is the ONLY clear on real silicon
	// (never CRB reads, never INIT edges), and the v1.6 ROM leans on it
	// twice per measurement: the $EFB6 stale-flag discard before INIT and
	// the $EFD4 blanking-window discard (hero-jr-sonar-spec.md §3.2 item 3,
	// adjudicated 2026-07-02, implemented 2026-07-03 together with the
	// plugin io-snapshot compose that stops observer bus reads). A DDRB
	// read (branch above) does not clear. Debugger/UI reads are guarded.
	if (!machine().side_effects_disabled() && m_sonar_echo_state)
	{
		m_sonar_echo_state = 0;
		m_sonar_echo = 0;
		driver_tracef("u215_speech_power_r cleared sonar echo flag (6821 IRQB1 clear-on-read)");
	}

	// PB0 is a CPU-driven port-B bit; the firmware uses it as a software
	// speech-busy flag (set before a phoneme, cleared by the CA1 ISR) and reads
	// back its own driven value.  Per the Technical Manual the SC-01 request
	// line is wired to CA1 (U215-40, $D842 bit 7), NOT to any $D841 bit, so do
	// not mirror m_speech_request onto PB0 - that fabricates a connection the
	// hardware does not have and races the firmware's $F067 busy-flag poll.
	return (m_u215_port_b & 0xaf) | (m_adc_output_state ? 0x10 : 0x00) | (m_sleep_norm->read() ? 0x40 : 0x00);
}

u8 herojr_state::u215_speech_control_r()
{
	// 6821 datasheet: the IRQA1 flag (CRA bit 7) is set by an active CA1
	// transition regardless of CRA bit 0 — that bit only enables the IRQ
	// line. The firmware's POLLED reads must therefore see the request
	// flag even with interrupts disabled (G1J-07). (Full delegation of
	// U214/U215 to the instantiated pia6821 devices was evaluated and
	// deferred: the hand model now matches the datasheet flag semantics,
	// and wholesale delegation would re-time every port path mid-phase.)
	const u8 visible_request = m_speech_request_flag ? 0x80 : 0x00;
	return (m_u215_control_a & 0x3f) | visible_request;
}

// $D843 = U215 CRB readback, real 6821 semantics (sonar spec §1.4):
// bits 5-0 return the written control byte (the write path masks to $3F —
// bits 7/6 are not writable on a 6821); bit 6 (IRQB2) reads 0 because every
// CRB value the v1.6 ROM writes has bit 5 = 1, CB2 in output (set/reset)
// mode, which holds the IRQB2 flag clear per the datasheet. The ROM's three
// CRB writers: $EB7C ($30 boot) and $EB86 ($34 run) store immediates, and
// $EF64 is an ORAA #$38 read-modify-write (sole caller $8646) whose
// $34|$38 = $3C keeps the bit-5 invariant — under the old hard-$00
// readback that RMW would have stored $38 (bit 2 = 0), silently selecting
// DDRB for the next $D841 access: a latent corruption the honest readback
// repairs. Bit 7 (IRQB1) is the latched CB1 sonar-echo edge. Idle
// read = $34, echo-latched read = $B4 (previously hard $00/$80).
// The ROM's range arithmetic is provably invariant to the low six bits
// (spec §2.2: the $EFDB pre-read and $EFEB post-subtract bracket them out
// of the count) — measured before/after identical at five distances,
// 2026-07-02.
u8 herojr_state::u215_sonar_echo_r()
{
	return (m_u215_control_b & 0x3f) | (m_sonar_echo_state ? 0x80 : 0x00);
}

void herojr_state::u215_speech_data_w(u8 data)
{
	const bool data_register_selected = BIT(m_u215_control_a, 2);
	driver_tracef("u215_speech_data_w data=$%02X data_register=%u control_a=$%02X ddr_a=$%02X", data, data_register_selected ? 1 : 0, m_u215_control_a, m_u215_ddr_a);
	if (!BIT(m_u215_control_a, 2))
	{
		m_u215_ddr_a = data;
		return;
	}

	m_port_outputs[0] = data;
	set_speech_data(data);
}

void herojr_state::u215_speech_power_w(u8 data)
{
	if (!BIT(m_u215_control_b, 2))
	{
		// CRB bit 2 selects DDRB; the ROM's only DDR write is the boot
		// direction byte $EB81 = $2F (PB0-3,5 outputs; PB4,6,7 inputs).
		// Real init therefore leaves the port B OUTPUT register at $00 —
		// PB0 clear, speech power off — and the firmware powers the
		// SC-01 per utterance through its $EF6F helper.
		m_u215_ddr_b = data;
		return;
	}

	const u8 previous_port = m_u215_port_b;
	m_port_outputs[1] = data;
	m_u215_port_b = data;
	const u8 previous_speech_power = m_speech_power_state;
	m_speech_power_state = BIT(data, 1) ? 1 : 0;
	m_speech_power = m_speech_power_state;
	if (BIT(data, 3) != BIT(previous_port, 3))
		start_adc_conversion();
	// Shared-line truth (JR-TM printed p. 23 / PDF p. 25): PB5 is both the
	// A/D clock and the sonar blanking gate ("the same line that is used for
	// the clock of the A/D converter"). Every falling edge clocks the ADC.
	// Sonar-side, PB5 is consumed as a LEVEL at echo-arrival time
	// (sonar_echo_tick): nothing is scheduled on this edge. The echo is
	// anchored at INIT (arm_sonar_cycle) and an echo arriving while PB5 is
	// still high is absorbed by the blanking mask — sonar spec §3.3(b)/§4
	// as re-adjudicated 2026-07-02.
	if (BIT(previous_port, 5) && !BIT(data, 5))
		clock_adc_bit();
	// $D841 D1 switches Q204/Q205 — the AUDIO rail only. R226 maintains
	// power to U223 itself when they are off and D203 isolates that
	// maintenance voltage from the amplifier (JR-TM p27), so toggling D1
	// mutes/unmutes the speaker (update_speech_power gain) and never
	// resets or stalls the synthesizer's handshake.
	update_speech_power();
}

void herojr_state::u215_speech_control_w(u8 data)
{
	m_port_outputs[2] = data;
	const u8 previous_control = m_u215_control_a;
	m_u215_control_a = data & 0x3f;
	const u8 previous_strobe = m_speech_strobe_state;
	const u8 requested_strobe = BIT(data, 3) ? 1 : 0;
	m_speech_strobe_state = requested_strobe;
	m_speech_strobe = m_speech_strobe_state;
	driver_tracef("u215_speech_control_w data=$%02X previous_control=$%02X control_a=$%02X strobe=%u previous_strobe=%u request_flag=%u", data, previous_control, m_u215_control_a, requested_strobe, previous_strobe, m_speech_request_flag);
	// CA2 follows CRA bit 3 (set/reset mode — every CRA value the ROM
	// writes has bit 4 = 1); the 74LS-side latch fires on the rising edge
	// only. No auto-clear: the ROM lowers CA2 itself before every phrase
	// ($F035 CRA=$36) and between phrase phonemes ($D39B restores the
	// pre-strobe CRA), and the end-of-phrase CRA=$3E deliberately parks
	// CA2 high so the idle $D31B heartbeat pass produces no further edge.
	if (requested_strobe && !previous_strobe)
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
	driver_tracef("rs232_w offset=$%X data=$%02X target=%s", unsigned(offset & 0x01), data, BIT(offset, 0) ? "data" : "control");
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
	// U223's A/R line drives CA1 directly and operates regardless of the
	// $D841 D1 rail switch: R226 maintains power to the CMOS SC-01 when
	// Q204/Q205 are off (JR-TM p27), so the handshake runs even while the
	// audio path is unpowered — the boot's first phrase depends on it.
	// The 6821 IRQA1 flag sets on the RISING edge only (CRA bit 1 = 1 in
	// every CRA value the ROM uses) and persists until a port A data
	// read clears it.
	const u8 previous_request = m_speech_request;
	m_speech_request = state ? 1 : 0;
	m_speech_ready = m_speech_request;
	driver_tracef("speech_request_w state=%d previous=%u request=%u flag=%u control_a=$%02X", state, previous_request, m_speech_request, m_speech_request_flag, m_u215_control_a);
	if (!previous_request && m_speech_request)
	{
		m_speech_request_flag = 1;
		m_speech_request_flag_output = m_speech_request_flag;
	}
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
	// Passive phoneme telemetry (Phase 2.2d): every latched SC-01 byte is
	// published with a sequence counter and its emulated-time stamp so the
	// bridge can emit byte-exact phoneme events (io_changed's dedup and
	// rate limit cannot be byte-exact). A latch while the previous phoneme
	// is still sounding (request low) is a clip on real hardware; count it.
	if (!m_speech_request)
	{
		m_phoneme_clip_count++;
		m_phoneme_clips = s32(m_phoneme_clip_count & 0x7fffffff);
	}
	m_phoneme_seq_count++;
	m_phoneme_seq = s32(m_phoneme_seq_count & 0x7fffffff);
	m_phoneme_byte = m_speech_data;
	m_phoneme_time_us = emulated_time_us(machine().time());

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
	// U306 is an ADC0831 (parts list, Heath 443-1189). A /CS edge arms a
	// fresh conversion: one leading null bit, then eight data bits.
	m_adc_shift = selected_adc_sample();
	m_adc_bits_remaining = 9;
	m_adc_output_state = 0;
	m_adc_sample = m_adc_shift;
	m_adc_output = 0;
}

void herojr_state::clock_adc_bit()
{
	// ADC0831 serial protocol (JR-TM pp. 21-22: successive approximation,
	// serial, MSB first; ROM reader $EBD9-$EC1D): the first falling clock
	// edge after /CS emits a leading null (zero), the next eight emit the
	// sample MSB-first, and past the data the output stays low until the
	// next /CS — the converter never restarts itself mid-conversion. The
	// ROM's two discard pulses consume the null and position the MSB for
	// its read-before-pulse loop.
	if (m_adc_bits_remaining == 0)
	{
		m_adc_output_state = 0;
		m_adc_output = 0;
		return;
	}

	if (m_adc_bits_remaining == 9)
	{
		m_adc_output_state = 0; // leading null bit
	}
	else
	{
		m_adc_output_state = BIT(m_adc_shift, 7) ? 1 : 0;
		m_adc_shift <<= 1;
	}
	m_adc_bits_remaining--;
	m_adc_output = m_adc_output_state;
}

// U214 CB2 (INIT, $D823 bit 3 in the ROM's set/reset mode) rising edge
// starts a sonar cycle: U307 transmits its 16-pulse burst and the receive
// chain arms (JR-TM printed p. 23 / PDF p. 25; sonar spec §1.3 steps 2-3).
// The echo is anchored at INIT for EVERY path (sonar spec §3.3(b)/§4 as
// re-adjudicated 2026-07-02): JR-TM printed p. 23 has the CPU time "the
// moment the INIT line was taken high" to the echo, and Heath's 2-ft worked
// example (~3.65 ms ⇒ 24 in, printed p. 22) reproduces 24 only under this
// anchor. Calibration constants, each with its basis:
//   - Speed of sound used by Heath: 1086 ft/s = 13032 in/s (JR-TM printed
//     p. 22 / PDF p. 24). Sound travels 2 in of path per inch of target
//     distance, so the round trip is 2/13032 s/in = 153.47 µs/in —
//     attotime::from_ticks(2·D, 13032) holds that ratio exactly.
//   - The v1.6 ROM's poll loop is 12 CPU cycles = 12 µs at E = 1 MHz
//     (JR-TM printed p. 24 / PDF p. 26; G1J-02); its counting loop
//     ($EFE2-$EFE9) opens 637 cycles after the INIT store, and the +$32
//     (50) the ROM folds into the count ($EFDE/$EFEE) COMPENSATES for the
//     blanking window the sound already traveled (spec §2.2) — which is
//     why the driver must not add any blanking offset of its own.
//   - Composition: count ≈ 12.79·D − 3, and the BASIC cart's count×10÷127
//     ($744D/$7456) makes SONAR ≈ D (spec §4): D = 5/30/90/150/200 in ⇒
//     4/30/90/150/201, cycle-exact closed form, 2026-07-02.
// The superseded window-anchored model (516b517) scheduled the echo from
// the PB5 blanking release instead and measured SONAR ≈ 1.007·D + 3.9 —
// it double-counted the ~627 µs blanking interval the ROM's +$32 already
// compensates; REFUTED by adversarial verify and re-adjudicated to the
// INIT anchor 2026-07-02. A raw INIT pulse with PB5 already low
// (bridge/test stimulus, G1J-06 shape) anchored at INIT before and still
// does — its 153.3-153.5 µs/in telemetry is unchanged. One echo per INIT
// cycle (the one-shot timer re-arms only here). The latched echo flag
// SURVIVES this edge: per the 6821 datasheet IRQB1 clears only on a $D841
// Peripheral Register B read (u215_speech_power_r; sonar spec §3.2
// item 3, implemented 2026-07-03) — a stale flag is discarded by the
// reader's own $D841 read, exactly the ROM's $EFB6 pre-INIT protocol.
// There is no 1-inch scheduling floor any more: its purpose — keeping an
// aperture value of 0 from latching before the ROM's $EFDB pre-read — is
// superseded by the blanking mask (in the ROM path PB5 is still high that
// early, so the echo is missed; in the raw-INIT path an immediate latch
// is the physical answer for a contact-range target).
void herojr_state::arm_sonar_cycle()
{
	m_sonar_distance_output = m_sonar_distance_sample;
	m_sonar_init_time_us = emulated_time_us(machine().time());
	driver_tracef("arm_sonar_cycle distance=%u pb5=%u control_b=$%02X port_b=$%02X", m_sonar_distance_sample, BIT(m_u215_port_b, 5), m_u214_control_b, m_u214_port_b);
	m_sonar_echo_timer->adjust(attotime::from_ticks(2 * u64(m_sonar_distance_sample), 13032));
}

void herojr_state::update_speech_power()
{
	m_votrax->set_output_gain(0, m_speech_power_state ? 1.0 : 0.0);
}

// Echo arrival. The blanking mask is the physical rule, not a special
// case: U307's internal blanking disables its receive input (pin 8) while
// the CPU holds U215 PB5 ($D841 bit 5) high — "The CPU controls the
// blanking with the same line that is used for the clock of the A/D
// converter" (JR-TM printed p. 23 / PDF p. 25) — so an echo whose round
// trip completes before the receive window opens is never latched at CB1
// at all. The v1.6 ROM holds PB5 high from before INIT ($EFB6-$EFB9)
// until 627 cycles after the INIT store ($EFD2-$EFD7), so targets closer
// than ~4.1 in (627 µs / 153.47 µs-per-in) go unheard and the ROM's
// measurement runs to its 4000-count timeout ⇒ BASIC SONAR 314: the REAL
// sub-≈4-inch blind zone (JR-TM printed p. 22: "about 4 inches to
// 25 feet"; spec §3.3(b)/§4 re-adjudication, 2026-07-02).
TIMER_CALLBACK_MEMBER(herojr_state::sonar_echo_tick)
{
	if (BIT(m_u215_port_b, 5))
	{
		driver_tracef("sonar_echo_tick blanked distance=%u port_b=$%02X", m_sonar_distance_sample, m_u215_port_b);
		return;
	}
	m_sonar_echo_state = 1;
	m_sonar_echo = 1;
	m_sonar_echo_time_us = emulated_time_us(machine().time());
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
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("RT-1") PORT_CODE(KEYCODE_0_PAD)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("1") PORT_CODE(KEYCODE_1)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("2") PORT_CODE(KEYCODE_2)
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("3") PORT_CODE(KEYCODE_3)

	PORT_START("KEY1")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SING / 4") PORT_CODE(KEYCODE_4)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("PLAY / 5") PORT_CODE(KEYCODE_5)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("HELP / 6") PORT_CODE(KEYCODE_6)
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SPEAK / 7") PORT_CODE(KEYCODE_7)

	PORT_START("KEY2")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("GAB / 8") PORT_CODE(KEYCODE_8)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("POET / 9") PORT_CODE(KEYCODE_9)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("DEMO / A") PORT_CODE(KEYCODE_A)
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("GUARD / B") PORT_CODE(KEYCODE_B)

	PORT_START("KEY3")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("ALARM / C") PORT_CODE(KEYCODE_C)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("PLAN / D") PORT_CODE(KEYCODE_D)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SET UP / E") PORT_CODE(KEYCODE_E)
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("ENTER / F") PORT_CODE(KEYCODE_F)

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
	// JR-TM printed p. 24: "The 4 MHz frequency of crystal Y201 is divided
	// by four inside U201" — E = 1 MHz (the MAME M6808 device performs that
	// divide-by-four on its input clock). Corroborated by the printed-p. 6
	// specifications, parts list Y201 = 404-536 (4 MHz crystal), and the
	// JR-SCH CPU board sheet 3 "4MEGHZ" label at Y201.
	M6808(config, m_maincpu, 4_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &herojr_state::mem_map);

	// Documented socket populations only (JR-TM pp. 43-44): "2K" = stock
	// 6116 at U203; "16K" = BASIC-era expanded unit, 8K at U203+U204.
	// The manual-implied maximum (8K in U203/U204/U205 = "24K") is reserved
	// for an explicit user decision before it becomes an option here.
	RAM(config, m_ram).set_default_size("2K").set_extra_options("16K");

	MC146818(config, m_rtc, 32.768_kHz_XTAL);
	// SQW pin → U211D → U214 CA1 (rtc_sqw_w). IRQ* pin (U213-19) →
	// R209 pull-up → U222 sleep-latch wake input (rtc_irq_w): on this
	// board the RTC interrupt is a power/wake control, never a CPU
	// interrupt (JR-TM printed p. 28 / PDF p. 30; rtc-spec §1.4;
	// G2J-08 model, blessed 2026-07-04).
	m_rtc->sqw().set(FUNC(herojr_state::rtc_sqw_w));
	m_rtc->irq().set(FUNC(herojr_state::rtc_irq_w));

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
