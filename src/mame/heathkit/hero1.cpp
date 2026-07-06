// license:BSD-3-Clause
// copyright-holders:Paul Czywczynski

/***************************************************************************

    Heathkit HERO 1 (ET-18) robot driver scaffold

    Hardware summary:
    - Motorola 6808 CPU
    - Base RAM plus ET-18-6/6264 memory expansion at 4000-5FFF
    - System ROM decoded at E000-FFFF on expanded systems
    - BASIC expansion ROM image decoded at A000-BFFF when present
    - Decoded I/O ports at C200-C300, plus internal 6821 PIA-style scaffolds
    - 6850 ACIA cassette/serial-style interface
    - 6 seven-segment LED digits
    - 17-key keypad
    - Sonar/light/sound/motion sensors
    - Motion model per hero-1-motion-spec.md (blessed 2026-07-06): seven
      stepper axes as phase-followers with position-derived limit
      switches, plus the DC drive wheel with a real encoder-pulse path
      into the $40 WHEEL DET interrupt

    TODO:
    - Replace remaining scaffold PIA aliases with exact board behavior.
    - Verify ROM provenance before submitting upstream.
    - Wire exact PIA bit assignments for remaining scaffold aliases.
    - Replace debug sensor registers with board-accurate sensor paths where known.

***************************************************************************/

#include "emu.h"

#include "cpu/m6800/m6800.h"
#include "machine/6821pia.h"
#include "machine/6850acia.h"
#include "machine/clock.h"
#include "machine/msm5832.h"
#include "bus/rs232/rs232.h"
#include "sound/votrax.h"

#include "osdcore.h"
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

class hero1_state : public driver_device
{
public:
	hero1_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_display_pia(*this, "display_pia"),
		m_keypad_pia(*this, "keypad_pia"),
		m_io_pia(*this, "io_pia"),
		m_speech_pia(*this, "speech_pia"),
		m_acia(*this, "acia"),
		m_rs232(*this, "rs232"),
		m_rtc(*this, "rtc"),
		m_votrax(*this, "votrax"),
		m_key_columns(*this, "KEYCOL%u", 0U),
		m_basic_baud(*this, "BASIC_BAUD"),
		m_digits(*this, "hero1_led_digit_%u", 0U),
		m_axis_positions(*this, "hero1_axis_position_%u", 0U),
		m_wheel_pulses_out(*this, "hero1_wheel_pulses"),
		m_speech_phoneme(*this, "hero1_speech_phoneme"),
		m_speech_inflection(*this, "hero1_speech_inflection"),
		m_speech_strobe(*this, "hero1_speech_strobe"),
		m_speech_ready(*this, "hero1_speech_ready"),
		m_motion(*this, "hero1_motion_detected"),
		m_port_outputs(*this, "hero1_port_out_%u", 0U)
	{
	}

	void hero1(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	static constexpr u16 HERO1_BASE_RAM_END = 0x1fff;
	static constexpr u16 HERO1_EXPANSION_RAM_BASE = 0x4000;
	static constexpr u16 HERO1_EXPANSION_RAM_END = 0x5fff;
	static constexpr u16 HERO1_BASIC_ROM_BASE = 0xa000;
	static constexpr u16 HERO1_DEBUG_IO_BASE = 0xd000;
	static constexpr u16 HERO1_SENSOR_BASE = 0xd100;   // debug bridge aperture, not original hardware
	static constexpr u16 HERO1_DEBUG_SPEECH_BASE = 0xd020;
	static constexpr u16 HERO1_ROM_BASE = 0xe000;

	// ------------------------------------------------------------------
	// Motion model — hero-1-motion-spec.md (BLESSED 2026-07-06), §4
	// PROPOSAL P1/P2: the seven stepper axes are phase-followers over the
	// pattern-select nibbles, positions are modeled mechanical microsteps,
	// and the six limit switches derive from position, never from
	// activity.
	// ------------------------------------------------------------------

	// Axis order matches the ROM position bytes $0000-$0006 and the
	// per-motor parameter table at $E29F (spec §2.2/§2.6).
	enum : int
	{
		AXIS_EXTEND = 0,
		AXIS_SHOULDER,
		AXIS_ROTATE,
		AXIS_PIVOT,
		AXIS_GRIPPER,
		AXIS_HEAD,
		AXIS_STEERING,
		AXIS_COUNT
	};

	// One entry per 4-line pattern-select circuit (spec §1.2/§1.4/§1.5):
	// steering ($C260 D7-D4, active-low couplers), pattern 1 = wrist
	// rotate/pivot ($C260 D3-D0), pattern 3 = extend/head ($C280 D7-D4),
	// pattern 2 = gripper/shoulder ($C280 D3-D0) — arm patterns active-high.
	enum : int
	{
		PGROUP_STEERING = 0,
		PGROUP_PIVOT_ROTATE,
		PGROUP_EXTEND_HEAD,
		PGROUP_GRIP_SHOULDER,
		PGROUP_COUNT
	};

	struct axis_config
	{
		const char *name;
		int group;
		u8 supply_mask;   // $C2C0 bit feeding this motor's +12 V common (spec §1.5); 0 = hard-fed (steering, §1.4)
		s32 span;         // mechanical travel in microsteps = ROM range x microstep divider (spec §2.6 table; H1-TM p. 32 ranges)
		s32 initial;      // initialized pose the ROM's cold seeds assume ($FEE7: rotate $4D, head $62, steering $49; spec §2.6)
	};

	static constexpr axis_config AXES[AXIS_COUNT] = {
		{ "extend",   PGROUP_EXTEND_HEAD,   0x40, 0x98 * 32, 0 },
		{ "shoulder", PGROUP_GRIP_SHOULDER, 0x80, 0x86 * 32, 0 },
		{ "rotate",   PGROUP_PIVOT_ROTATE,  0x80, 0x93 * 8,  0x4d * 8 },
		{ "pivot",    PGROUP_PIVOT_ROTATE,  0x40, 0xa5 * 8,  0 },
		{ "gripper",  PGROUP_GRIP_SHOULDER, 0x40, 0x75 * 4,  0 },
		{ "head",     PGROUP_EXTEND_HEAD,   0x80, 0xc2 * 32, 0x62 * 32 },
		{ "steering", PGROUP_STEERING,      0x00, 0x93 * 8,  0x49 * 8 },
	};

	// Position in the two-phase-on full-step cycle from the ROM's $F19B
	// table (66 55 99 AA; spec §2.1/§3.1), in position-INCREASING order.
	// -1 = not a two-phase-on pattern.
	static int phase_index(u8 nibble)
	{
		switch (nibble & 0x0f)
		{
		case 0x6: return 0;
		case 0x5: return 1;
		case 0x9: return 2;
		case 0xa: return 3;
		default: return -1;
		}
	}

	// ------------------------------------------------------------------
	// Drive-wheel model (spec §4 P3): DC PM motor decoded from the $C2A0
	// latch (D7 direction relay K801, D6 ON/OFF, D5-D0 speed DAC), wheel
	// travel integrated as encoder pulses through the modeled 74221
	// one-shot into the $40 WHEEL DET interrupt (spec §1.3/§1.6/§2.5).
	// ------------------------------------------------------------------

	// CONFIRMED cart arithmetic (spec §5 Q1 resolution 2026-07-06): the
	// HERO-1 BASIC cart converts FWD/BWD inches to encoder pulses as
	// pulses = inches x [$42]/64 with the seeded default $6D = 109
	// (hero1_basic.bin $B2C5 conversion, $B1DA seed).
	static constexpr double WHEEL_PULSES_PER_INCH = 109.0 / 64.0;

	// APPROXIMATION A1 (spec §5 Q2, adopted 2026-07-06): no manual states
	// a ground speed, so the DAC-to-speed map is a labeled nominal — 10
	// in/s at DAC 63 (the Jr 10 in/s precedent), 2.5 in/s at DAC 0 (the
	// ROM's $40 "slow" is ON + DAC 0 and must move), affine between.
	// Resulting pulse rates (~4.3-17 pulses/s) sit far below the 74221
	// one-shot's ~140 pulses/s ceiling (spec §3.2).
	static constexpr double WHEEL_SPEED_DAC0_IN_PER_S = 2.5;
	static constexpr double WHEEL_SPEED_DAC63_IN_PER_S = 10.0;

	void mem_map(address_map &map) ATTR_COLD;

	u8 display_memory_r(offs_t offset);
	void display_memory_w(offs_t offset, u8 data);
	u8 keypad_column0_r();
	u8 keypad_column1_r();
	u8 keypad_column2_r();
	u8 keypad_column_r(int column);
	u8 display_pia_a_r();
	u8 keypad_pia_a_r();
	u8 sensor_r(offs_t offset);
	u8 port_c200_interrupt_r();
	u8 port_c220_sonar_r();
	u8 port_c240_sense_r();
	u8 port_c260_limits_speech_r();
	u8 port_c280_motion_r();
	u8 port_c2a0_experimental_r();
	u8 port_c300_clock_r();
	void sensor_w(offs_t offset, u8 data);
	void port_c200_control_w(u8 data);
	void port_c220_experimental_w(u8 data);
	void port_c240_speech_w(u8 data);
	void port_c260_motion_w(u8 data);
	void port_c280_arm_base_w(u8 data);
	void port_c2a0_main_drive_w(u8 data);
	void port_c2c0_select_strobe_w(u8 data);
	void port_c2e0_system_select_w(u8 data);
	void port_c300_clock_w(u8 data);
	void display_pia_a_w(u8 data);
	void display_pia_b_w(u8 data);
	void keypad_pia_b_w(u8 data);
	void io_pia_a_w(u8 data);
	void io_pia_b_w(u8 data);
	u8 speech_pia_b_r();
	void speech_pia_a_w(u8 data);
	void speech_pia_b_w(u8 data);
	void speech_request_w(int state);
	void set_speech_data(u8 data);
	void latch_speech_phoneme(bool ignore_power = false);
	void latch_interrupt(u8 mask);
	void group_pattern_w(int group, u8 pattern);
	bool axis_supplied(int axis) const;
	void axis_apply_pattern(int axis, u8 pattern);
	void axis_energize(int axis);
	void axis_step(int axis, int direction);
	void update_drive_model();
	attotime wheel_pulse_period() const;
	void update_irq_line();
	void update_pendant_port(u8 data);
	void set_pendant_function(bool arm);
	void set_pendant_joint_code(u8 code);
	void set_pendant_motion(u8 motion);
	void set_pendant_trigger(bool pressed);
	void experimental_serial_rxd_w(int state);
	void update_cpu_clock();
	void update_speech_power();
	TIMER_CALLBACK_MEMBER(executive_timer_tick);
	TIMER_CALLBACK_MEMBER(sonar_echo_tick);
	TIMER_CALLBACK_MEMBER(wheel_pulse_tick);

	template <typename... FormatParams>
	void driver_tracef(const char *format, FormatParams &&... args)
	{
		if (!m_driver_trace)
			return;

		osd_printf_info("heathkit_hero_driver[hero1]: ");
		osd_printf_info(format, std::forward<FormatParams>(args)...);
		osd_printf_info("\n");
	}

	required_device<m6808_cpu_device> m_maincpu;
	required_device<pia6821_device> m_display_pia;
	required_device<pia6821_device> m_keypad_pia;
	required_device<pia6821_device> m_io_pia;
	required_device<pia6821_device> m_speech_pia;
	required_device<acia6850_device> m_acia;
	required_device<rs232_port_device> m_rs232;
	required_device<msm5832_device> m_rtc;
	required_device<votrax_sc01_device> m_votrax;
	required_ioport_array<3> m_key_columns;
	required_ioport m_basic_baud;

	output_finder<6> m_digits;
	output_finder<AXIS_COUNT> m_axis_positions;
	output_finder<> m_wheel_pulses_out;
	output_finder<> m_speech_phoneme;
	output_finder<> m_speech_inflection;
	output_finder<> m_speech_strobe;
	output_finder<> m_speech_ready;
	output_finder<> m_motion;
	output_finder<8> m_port_outputs;

	u8 m_display_select = 0;
	u8 m_display_segments = 0;
	u8 m_display_memory[6]{};
	u8 m_key_column_select = 0;
	u8 m_sonar_count = 134;
	u8 m_light_level = 50;
	u8 m_sound_level = 0;
	u8 m_motion_detected = 0;
	u8 m_speech_data = 0;
	u8 m_speech_control = 0;
	u8 m_speech_power = 0;
	u8 m_speech_request = 1;
	u8 m_speech_strobe_state = 0;
	u8 m_debug_speech_data = 0;
	u8 m_debug_speech_control = 0;
	u8 m_tape_in = 1;
	u8 m_key_debug_columns[3]{};
	u8 m_manual_port_out[8]{};
	u8 m_debug_io_port_a = 0;
	u8 m_debug_io_port_b = 0;
	u8 m_interrupt_status = 0x00;
	u8 m_interrupt_clear_latch = 0xff;
	s32 m_axis_pos[AXIS_COUNT]{};      // modeled mechanical microsteps (0..span); board state, survives reset
	u8 m_axis_pattern[AXIS_COUNT]{};   // detent pattern the rotor last followed (0 = no memory)
	u8 m_group_pattern[PGROUP_COUNT]{}; // currently energized nibble per pattern circuit (0 = off)
	u32 m_wheel_pulses = 0;            // total encoder pulses ever (the disc never resets); board state
	bool m_wheel_pulse_pending = false;
	u8 m_clock_address = 0;
	u8 m_clock_control = 0;
	u8 m_pendant_port = 0x8e;  // ET-18 teaching pendant read byte at $C280; idle ARM/N/released
	u8 m_experimental_serial_rxd = 1;
	bool m_driver_trace = false;
	emu_timer *m_executive_timer = nullptr;
	emu_timer *m_sonar_timer = nullptr;
	emu_timer *m_wheel_timer = nullptr;
};

void hero1_state::machine_start()
{
	m_driver_trace = driver_trace_enabled("hero1");

	// Crystal selection is a board property, not a reset behavior: a v10
	// board has the base-board Y401 (3.579545 MHz) soldered from power-on.
	// Select it here — the earliest point where the -bios choice is
	// resolved (romload assigns system_bios() before devices start) — so
	// no machine phase ever runs or reports the config-time 4 MHz
	// placeholder. When this lived in machine_reset(), the Lua bridge
	// could serve get_capabilities from the startup-screen frame_update
	// pump (running_machine::run → ui_initialize → video frame_update →
	// on_periodic) BEFORE the first soft reset applied the v10 crystal:
	// the G1H-02 1-in-N cpuClockHz = 4,000,000 flake (vsix plan §2.4c).
	update_cpu_clock();

	m_digits.resolve();
	m_axis_positions.resolve();
	m_wheel_pulses_out.resolve();
	m_speech_phoneme.resolve();
	m_speech_inflection.resolve();
	m_speech_strobe.resolve();
	m_speech_ready.resolve();
	m_motion.resolve();
	m_port_outputs.resolve();

	save_item(NAME(m_display_select));
	save_item(NAME(m_display_segments));
	save_item(NAME(m_display_memory));
	save_item(NAME(m_key_column_select));
	save_item(NAME(m_sonar_count));
	save_item(NAME(m_light_level));
	save_item(NAME(m_sound_level));
	save_item(NAME(m_motion_detected));
	save_item(NAME(m_speech_data));
	save_item(NAME(m_speech_control));
	save_item(NAME(m_speech_power));
	save_item(NAME(m_speech_request));
	save_item(NAME(m_speech_strobe_state));
	save_item(NAME(m_debug_speech_data));
	save_item(NAME(m_debug_speech_control));
	save_item(NAME(m_tape_in));
	save_item(NAME(m_key_debug_columns));
	save_item(NAME(m_manual_port_out));
	save_item(NAME(m_debug_io_port_a));
	save_item(NAME(m_debug_io_port_b));
	save_item(NAME(m_interrupt_status));
	save_item(NAME(m_interrupt_clear_latch));
	save_item(NAME(m_axis_pos));
	save_item(NAME(m_axis_pattern));
	save_item(NAME(m_group_pattern));
	save_item(NAME(m_wheel_pulses));
	save_item(NAME(m_wheel_pulse_pending));
	save_item(NAME(m_clock_address));
	save_item(NAME(m_clock_control));
	save_item(NAME(m_pendant_port));
	save_item(NAME(m_experimental_serial_rxd));

	// Mechanical pose is board state, not reset state (Jr steering
	// precedent): seed the initialized pose the ROM's cold-boot path
	// assumes — $FEE7 seeds rotate $4D, head $62, steering $49, others 0
	// (spec §2.6 "Cold-boot seeds") — so the position bytes the ROM
	// writes agree with the modeled mechanics until the user runs Zero.
	for (int axis = 0; axis < AXIS_COUNT; axis++)
	{
		m_axis_pos[axis] = AXES[axis].initial;
		m_axis_pattern[axis] = 0;
		m_axis_positions[axis] = u32(m_axis_pos[axis]);
	}
	for (u8 &pattern : m_group_pattern)
		pattern = 0;
	m_wheel_pulses = 0;
	m_wheel_pulses_out = 0;

	m_executive_timer = timer_alloc(FUNC(hero1_state::executive_timer_tick), this);
	m_sonar_timer = timer_alloc(FUNC(hero1_state::sonar_echo_tick), this);
	m_wheel_timer = timer_alloc(FUNC(hero1_state::wheel_pulse_tick), this);
	driver_tracef("machine_start trace enabled");
}

void hero1_state::machine_reset()
{
	m_display_select = 0;
	m_display_segments = 0;
	for (u8 &digit : m_display_memory)
		digit = 0;
	m_key_column_select = 0;
	m_sonar_count = 134;
	m_light_level = 50;
	m_sound_level = 0;
	m_motion_detected = 0;
	m_speech_data = 0;
	m_speech_control = 0;
	m_speech_power = 0;
	m_speech_request = 1;
	m_speech_strobe_state = 0;
	m_debug_speech_data = 0;
	m_debug_speech_control = 0;
	m_tape_in = 1;
	// Interrupt status register (H1-SCH2, hero-1-motion-spec.md §1.8): the
	// LS73 flip-flops and the U411 74LS374 clear latch have no reset pins,
	// so their power-on state is electrically undefined. Deterministic
	// model choice: no source pending, all clears released — the ROM's own
	// reset path parks the clear latch at $FF within its first milliseconds
	// ($F3D2: C6 FF F7 C2 00 F7 0F 04) and dispatches nothing before that.
	m_interrupt_status = 0x00;
	m_interrupt_clear_latch = 0xff;
	m_clock_address = 0;
	m_clock_control = 0;
	m_pendant_port = 0x8e;
	m_experimental_serial_rxd = 1;
	m_rs232->write_txd(1);
	m_speech_phoneme = 0;
	driver_tracef("machine_reset pendant=$%02X speech_request=%u speech_power=%u", m_pendant_port, m_speech_request, m_speech_power);
	m_speech_inflection = 0;
	m_speech_strobe = 0;
	m_speech_ready = 1;
	update_speech_power();
	for (u8 &port : m_manual_port_out)
		port = 0;
	// The output latches were just cleared: with $C260 = $00 the
	// active-low steering couplers all conduct (an invalid non-two-phase
	// pattern — indeterminate torque, no modeled motion) and the
	// active-high arm patterns are all off. Positions and detent memory
	// are mechanical state and deliberately NOT reset; the ROM's own
	// quiesce ($F3AB) parks the latches within milliseconds.
	group_pattern_w(PGROUP_STEERING, 0x0f);
	group_pattern_w(PGROUP_PIVOT_ROTATE, 0x00);
	group_pattern_w(PGROUP_EXTEND_HEAD, 0x00);
	group_pattern_w(PGROUP_GRIP_SHOULDER, 0x00);
	m_debug_io_port_a = 0;
	m_debug_io_port_b = 0;
	for (int port = 0; port < 8; port++)
		m_port_outputs[port] = 0;
	for (u8 &column : m_key_debug_columns)
		column = 0x3f;
	for (int digit = 0; digit < 6; digit++)
		m_digits[digit] = 0;
	update_irq_line();
	m_executive_timer->adjust(attotime::from_hz(1024), 0, attotime::from_hz(1024));
	m_sonar_timer->adjust(attotime::never);
	// The drive latch was just cleared (D6 = 0, motor off): no pulses.
	// The odometer count itself is mechanical state and survives.
	m_wheel_timer->adjust(attotime::never);
	m_wheel_pulse_pending = false;
}

void hero1_state::mem_map(address_map &map)
{
	map(0x0000, HERO1_BASE_RAM_END).ram();
	map(HERO1_EXPANSION_RAM_BASE, HERO1_EXPANSION_RAM_END).ram();

	map(0xc003, 0xc003).r(FUNC(hero1_state::keypad_column2_r));
	map(0xc005, 0xc005).r(FUNC(hero1_state::keypad_column1_r));
	map(0xc006, 0xc006).r(FUNC(hero1_state::keypad_column0_r));
	map(0xc10f, 0xc16f).rw(FUNC(hero1_state::display_memory_r), FUNC(hero1_state::display_memory_w));

	// ET-18 Technical Manual port map, fold-out from page 119.
	map(0xc200, 0xc200).rw(FUNC(hero1_state::port_c200_interrupt_r), FUNC(hero1_state::port_c200_control_w));
	map(0xc220, 0xc220).rw(FUNC(hero1_state::port_c220_sonar_r), FUNC(hero1_state::port_c220_experimental_w));
	map(0xc240, 0xc240).rw(FUNC(hero1_state::port_c240_sense_r), FUNC(hero1_state::port_c240_speech_w));
	map(0xc260, 0xc260).rw(FUNC(hero1_state::port_c260_limits_speech_r), FUNC(hero1_state::port_c260_motion_w));
	map(0xc280, 0xc280).rw(FUNC(hero1_state::port_c280_motion_r), FUNC(hero1_state::port_c280_arm_base_w));
	map(0xc2a0, 0xc2a0).rw(FUNC(hero1_state::port_c2a0_experimental_r), FUNC(hero1_state::port_c2a0_main_drive_w));
	map(0xc2c0, 0xc2c0).w(FUNC(hero1_state::port_c2c0_select_strobe_w));
	map(0xc2e0, 0xc2e0).w(FUNC(hero1_state::port_c2e0_system_select_w));
	map(0xc300, 0xc300).rw(FUNC(hero1_state::port_c300_clock_r), FUNC(hero1_state::port_c300_clock_w));

	// Debug aliases retained for deterministic extension smoke programs while board-level decode is completed.
	map(HERO1_DEBUG_IO_BASE + 0x00, HERO1_DEBUG_IO_BASE + 0x03).rw(m_display_pia, FUNC(pia6821_device::read), FUNC(pia6821_device::write));
	map(HERO1_DEBUG_IO_BASE + 0x04, HERO1_DEBUG_IO_BASE + 0x07).rw(m_keypad_pia, FUNC(pia6821_device::read), FUNC(pia6821_device::write));
	map(HERO1_DEBUG_IO_BASE + 0x08, HERO1_DEBUG_IO_BASE + 0x0b).rw(m_io_pia, FUNC(pia6821_device::read), FUNC(pia6821_device::write));
	map(HERO1_DEBUG_IO_BASE + 0x10, HERO1_DEBUG_IO_BASE + 0x11).rw(m_acia, FUNC(acia6850_device::read), FUNC(acia6850_device::write));
	map(HERO1_DEBUG_SPEECH_BASE + 0x00, HERO1_DEBUG_SPEECH_BASE + 0x03).rw(m_speech_pia, FUNC(pia6821_device::read), FUNC(pia6821_device::write));

	map(HERO1_SENSOR_BASE + 0x00, HERO1_SENSOR_BASE + 0x0f).rw(FUNC(hero1_state::sensor_r), FUNC(hero1_state::sensor_w));

	map(HERO1_BASIC_ROM_BASE, 0xbfff).rom().region("basic", 0);
	map(HERO1_ROM_BASE, 0xffff).rom().region("maincpu", 0);
}

u8 hero1_state::display_memory_r(offs_t offset)
{
	// H1-SCH sheet 4: the digit latches U1201-U1206 are 74LS259 addressable
	// latches (443-804) — write-only devices with no output path onto the
	// CPU data bus. Reads of the display window are open bus.
	return 0xff;
}

void hero1_state::display_memory_w(offs_t offset, u8 data)
{
	const u16 address = 0xc10f + offset;
	const int latch = (address >> 4) & 0x07;
	const int digit = 6 - latch;
	if (digit < 0 || digit >= 6)
		return;

	// H1-SCH sheet 4, pure 74LS259 semantics (U1201-U1206 via the U1208
	// 74LS42 digit decode): A0-A2 select one of the eight latch outputs,
	// D0 is the latched state, and a high output lights the segment. The
	// monitor's OUTCH ($F7C8) writes sixteen descending addresses per
	// glyph through a 9-bit rotate; the second pass lands glyph bit j on
	// segment j exactly, so the accumulated latch state equals the glyph
	// byte — no whole-byte write port exists on the board.
	const u8 segment_bit = 1U << (address & 0x07);
	if (BIT(data, 0))
		m_display_memory[digit] |= segment_bit;
	else
		m_display_memory[digit] &= ~segment_bit;
	m_digits[digit] = m_display_memory[digit];
}

u8 hero1_state::display_pia_a_r()
{
	return m_display_segments;
}

void hero1_state::display_pia_a_w(u8 data)
{
	// Debug aperture state only.  Real display latches are decoded in the
	// C10F-C16F window; this alias must not overwrite ROM-owned display output.
	m_display_segments = data;
}

void hero1_state::display_pia_b_w(u8 data)
{
	m_display_select = data & 0x3f;
}

// BIOS-appropriate crystal (vsix hardware-notes.md §"HERO 1"): v1.0 = the
// base-board Y401 3.579545 MHz (NTSC colorburst, H1-Tech p79/p103);
// v1.3/v1.U = 4 MHz, the ET-18-6 Memory Expansion crystal Y101 (pp. 13-14)
// — those ROMs require the expansion that carries it. Called once from
// machine_start(): the machine config constructs the CPU before the BIOS
// selection is knowable, and start time precedes every emulated cycle and
// every bridge-servable instant, so the selected crystal is in force "from
// power-on" exactly as on the real board.
void hero1_state::update_cpu_clock()
{
	const bool system_v10 = system_bios() == 1;
	m_maincpu->set_unscaled_clock(system_v10 ? 3.579545_MHz_XTAL : 4_MHz_XTAL);
}

u8 hero1_state::keypad_column0_r()
{
	return keypad_column_r(0);
}

u8 hero1_state::keypad_column1_r()
{
	return keypad_column_r(1);
}

u8 hero1_state::keypad_column2_r()
{
	return keypad_column_r(2);
}

u8 hero1_state::keypad_column_r(int column)
{
	// The monitor's INCH routine polls the display-board keyboard decode at
	// C003/C005/C006.  The ROM decode sees the physical columns in reverse
	// address order: C006 is D/A/7/4/1/0, C005 is E/B/8/5/2, and C003 is
	// F/C/9/6/3.  Expose the injected active-low key columns here so
	// headless tests and the panel exercise the same ROM path a user would.
	if (column < 0 || column >= 3)
		return 0xff;

	return (m_key_columns[column]->read() & m_key_debug_columns[column]) | 0xc0;
}

u8 hero1_state::keypad_pia_a_r()
{
	u8 result = 0x3f;

	for (int column = 0; column < 3; column++)
	{
		if (!BIT(m_key_column_select, column))
			result &= m_key_columns[column]->read() & m_key_debug_columns[column];
	}

	return result | 0xc0;
}

void hero1_state::keypad_pia_b_w(u8 data)
{
	// The ET-18 display-board description says address lines A0-A2 are driven
	// low one at a time, then row data D0-D5 is read for the key matrix.
	m_key_column_select = data & 0x07;
}

u8 hero1_state::sensor_r(offs_t offset)
{
	switch (offset & 0x0f)
	{
	case 0x00: return m_sonar_count;
	case 0x01: return m_light_level;
	case 0x02: return m_sound_level;
	case 0x03: return m_motion_detected ? 1 : 0;
	case 0x04: return m_key_debug_columns[0];
	case 0x05: return m_key_debug_columns[1];
	case 0x06: return m_key_debug_columns[2];
	case 0x07: return BIT(m_pendant_port, 7) ? 0 : 1; // legacy debug readback: 0 ARM, 1 BODY
	case 0x08: return (m_pendant_port >> 4) & 0x07;
	case 0x09: return !BIT(m_pendant_port, 2) ? 1 : !BIT(m_pendant_port, 3) ? 2 : 0;
	case 0x0a: return BIT(m_pendant_port, 0) ? 1 : 0;
	case 0x0b: return m_tape_in;
	case 0x0c: return m_pendant_port;
	default: return 0;
	}
}

u8 hero1_state::port_c200_interrupt_r()
{
	// Interrupt status register (H1-SCH2; hero-1-motion-spec.md §1.8):
	// eight 74LS73 J-K flip-flops latch their sources on a negative-going
	// clock edge; the Q outputs read back through buffer U412. Reads are
	// non-destructive and report only genuinely latched events — no source
	// is fabricated here (the old stub's $02 motion-detect toggle during
	// drive activity was firmware-visible fakery; spec §4 items 2/3).
	return m_interrupt_status;
}

u8 hero1_state::port_c220_sonar_r()
{
	// ET-18 sheet 3 reads U313 counter bits Q2-Q9 through U312 onto D0-D7.
	// The bridge converts the panel's inch value into this readable count byte.
	return m_sonar_count;
}

u8 hero1_state::port_c240_sense_r()
{
	// ET-18 sense board: C240 reads the 8-bit ADC output. The sound/light
	// select line chooses which analog source reaches the ADC; the port map
	// labels C2E0 bit 7 as EYE/EAR select, and the sense-board text says high
	// selects light while low selects sound.
	return BIT(m_manual_port_out[7], 7) ? m_light_level : m_sound_level;
}

u8 hero1_state::port_c260_limits_speech_r()
{
	// Input port U302 (74LS244, non-inverting): D7 TAPE IN, D0 SPEECH
	// REQUEST, and the six limit-switch lines — ACTIVE LOW through 10 k
	// pullups; a switch closes to ground exactly at its axis's travel end
	// (hero-1-motion-spec.md §1.7). The adjudicated bit map (H1-IC block
	// labels, corroborated by ROM homing, TM p. 32, and H1-UG): D6/D5/D4 =
	// the position-0 ends (steering left, shoulder down, head CCW),
	// D3/D2/D1 = the maxima (right, up, CW). Limits derive from modeled
	// POSITION only — never asserted in pairs, never mid-travel, and motor
	// latch activity does not touch them (the stub's $48 "feedback" fake
	// is retired; spec §4 item 1: no phantom limits).
	u8 value = (m_tape_in ? 0x80 : 0x00) | (m_speech_request ? 0x01 : 0x00);
	if (m_axis_pos[AXIS_STEERING] > 0)
		value |= 0x40;
	if (m_axis_pos[AXIS_SHOULDER] > 0)
		value |= 0x20;
	if (m_axis_pos[AXIS_HEAD] > 0)
		value |= 0x10;
	if (m_axis_pos[AXIS_STEERING] < AXES[AXIS_STEERING].span)
		value |= 0x08;
	if (m_axis_pos[AXIS_SHOULDER] < AXES[AXIS_SHOULDER].span)
		value |= 0x04;
	if (m_axis_pos[AXIS_HEAD] < AXES[AXIS_HEAD].span)
		value |= 0x02;
	return value;
}

u8 hero1_state::port_c280_motion_r()
{
	return m_pendant_port;
}

u8 hero1_state::port_c2a0_experimental_r()
{
	return 0x78 | (m_basic_baud->read() & 0x07) | (m_experimental_serial_rxd ? 0x80 : 0x00);
}

u8 hero1_state::port_c300_clock_r()
{
	return 0xf0 | (m_rtc->data_r() & 0x0f);
}

void hero1_state::sensor_w(offs_t offset, u8 data)
{
	switch (offset & 0x0f)
	{
	case 0x00:
		// Debug-aperture echo event: a new counter byte models "an echo
		// arrived with this reading", which is exactly what clocks the
		// SONAR flip-flop ($01) on hardware (U323B -> U408A, spec §1.8).
		m_sonar_count = data;
		latch_interrupt(0x01);
		break;
	case 0x01:
		m_light_level = data;
		break;
	case 0x02:
		m_sound_level = data;
		break;
	case 0x03:
	{
		// MOTION DET. (U408B) is edge-clocked like every other source:
		// one latched event per detector off->on transition. Deasserting
		// the aperture never clears the flip-flop — only U411 does.
		const u8 detected = data ? 1 : 0;
		if (detected && !m_motion_detected)
			latch_interrupt(0x02);
		m_motion_detected = detected;
		m_motion = m_motion_detected;
		break;
	}
	case 0x04:
	case 0x05:
	case 0x06:
		m_key_debug_columns[(offset & 0x0f) - 0x04] = data & 0x3f;
		break;
	case 0x07:
		set_pendant_function(data == 0);
		break;
	case 0x08:
	{
		set_pendant_joint_code(data & 0x07);
		break;
	}
	case 0x09:
		set_pendant_motion((data <= 2) ? data : 0);
		break;
	case 0x0a:
		set_pendant_trigger(data != 0);
		break;
	case 0x0b:
		m_tape_in = data ? 1 : 0;
		break;
	case 0x0c:
		update_pendant_port(data);
		break;
	default:
		break;
	}
}

void hero1_state::port_c200_control_w(u8 data)
{
	m_manual_port_out[0] = data;
	m_port_outputs[0] = data;
	driver_tracef("port_c200_control_w data=$%02X interrupt_before=$%02X", data, m_interrupt_status);
	// U411 is a 74LS374 LEVEL latch wired to the LS73 active-low clear
	// pins: a bit written 0 clears that source's flip-flop and HOLDS it
	// cleared (events on a held-low line are dropped); a bit written 1
	// releases the clear so the next source edge can latch again. The
	// ROM's own idiom is a pulse-clear — write (~status & $0F04), then
	// $0F04 (v1.3 IRQ epilogue) — and its reset path parks $FF. H1-TM
	// p. 80's "write a '1' ... can selectively clear" prose is a recorded
	// manual erratum (hero-1-motion-spec.md §1.8, blessing 2026-07-06).
	m_interrupt_clear_latch = data;
	m_interrupt_status &= data;
	update_irq_line();
}

void hero1_state::port_c220_experimental_w(u8 data)
{
	m_manual_port_out[1] = data;
	m_port_outputs[1] = data;
	m_rs232->write_txd(BIT(data, 0));
}

void hero1_state::port_c240_speech_w(u8 data)
{
	m_manual_port_out[2] = data;
	m_port_outputs[2] = data;
	driver_tracef("port_c240_speech_w data=$%02X phoneme=$%02X inflection=%u", data, data & 0x3f, (data >> 6) & 0x03);

	// ET-18 Technical Manual: C240 carries the speech synthesizer's six-bit
	// phoneme address plus two pitch/inflection control bits.
	set_speech_data(data);
}

void hero1_state::port_c260_motion_w(u8 data)
{
	m_manual_port_out[3] = data;
	m_port_outputs[3] = data;
	// High nibble drives the steering couplers ACTIVE-LOW (U806-U809 /
	// Q804-Q807; the ROM's port writer $F171 complements exactly this
	// nibble — spec §1.4/§2.4/§3.1), so undo the inversion to recover the
	// energized-winding pattern. Low nibble is arm pattern circuit 1
	// (wrist rotate + wrist pivot), ACTIVE-HIGH (spec §1.5).
	group_pattern_w(PGROUP_STEERING, (~data >> 4) & 0x0f);
	group_pattern_w(PGROUP_PIVOT_ROTATE, data & 0x0f);
}

void hero1_state::port_c280_arm_base_w(u8 data)
{
	m_manual_port_out[4] = data;
	m_port_outputs[4] = data;
	// D7-D4 = extend/head pattern (arm pattern circuit 3), D3-D0 =
	// gripper/shoulder pattern (circuit 2), both ACTIVE-HIGH; which motor
	// of each pair moves is decided by the $C2C0 supply selects (spec
	// §1.2/§1.5 — the old "head = bits 7-6, arm = bits 5-0" split was a
	// misdecode, spec §4 item 5).
	group_pattern_w(PGROUP_EXTEND_HEAD, (data >> 4) & 0x0f);
	group_pattern_w(PGROUP_GRIP_SHOULDER, data & 0x0f);
}

void hero1_state::port_c2a0_main_drive_w(u8 data)
{
	// U307 main-drive latch: D7 direction relay K801, D6 motor ON/OFF,
	// D5-D0 speed DAC (spec §1.2/§1.3). The machine has ONE drive motor —
	// the old left/right decode was a misdecode (spec §4 item 4).
	m_manual_port_out[5] = data;
	m_port_outputs[5] = data;
	driver_tracef("port_c2a0_main_drive_w data=$%02X direction=%u on=%u dac=%u", data, BIT(data, 7), BIT(data, 6), data & 0x3f);
	update_drive_model();
}

void hero1_state::port_c2c0_select_strobe_w(u8 data)
{
	const u8 previous_supplies = m_manual_port_out[6] & 0xc0;
	m_manual_port_out[6] = data;
	m_port_outputs[6] = data;
	// D7 = arm supply select 1 (+12 V to wrist rotate, shoulder, head),
	// D6 = supply select 2 (wrist pivot, gripper, extend); active HIGH
	// (spec §1.5). Energizing a supply re-detents the supplied motors
	// onto whatever pattern their circuit currently holds — a snap, not
	// a modeled step. De-energizing frees them (position/detent kept).
	const u8 rising_supplies = data & ~previous_supplies & 0xc0;
	if (rising_supplies != 0)
	{
		for (int axis = 0; axis < AXIS_COUNT; axis++)
		{
			if ((AXES[axis].supply_mask & rising_supplies) != 0)
				axis_energize(axis);
		}
	}
	m_clock_address = data & 0x0f;
	m_rtc->address_w(m_clock_address);
	m_speech_control = data;
	const u8 previous_strobe = m_speech_strobe_state;
	m_speech_strobe_state = BIT(data, 5) ? 1 : 0;
	m_speech_strobe = m_speech_strobe_state;
	if (m_speech_strobe_state != previous_strobe)
		driver_tracef("port_c2c0_select_strobe_w data=$%02X clock_address=$%01X strobe=%u previous_strobe=%u pc=$%04X", data, m_clock_address, m_speech_strobe_state, previous_strobe, m_maincpu->pc() & 0xffff);
	if (!previous_strobe && m_speech_strobe_state)
		latch_speech_phoneme();
}

void hero1_state::port_c2e0_system_select_w(u8 data)
{
	const u8 previous_sonar_power = BIT(m_manual_port_out[7], 1) ? 1 : 0;
	m_manual_port_out[7] = data;
	m_port_outputs[7] = data;
	const u8 previous_speech_power = m_speech_power;
	m_speech_power = BIT(data, 3) ? 1 : 0;
	update_speech_power();
	driver_tracef("port_c2e0_system_select_w data=$%02X sonar_power=%u previous_sonar_power=%u speech_power=%u previous_speech_power=%u", data, BIT(data, 1), previous_sonar_power, m_speech_power, previous_speech_power);
	if (previous_speech_power && !m_speech_power)
	{
		m_votrax->reset();
		m_speech_request = 1;
		m_speech_ready = 1;
	}
	if (!previous_sonar_power && BIT(data, 1))
	{
		// Powering the sonar starts a ping; the echo event latches $01
		// when the modeled round trip elapses. Power changes never touch
		// the interrupt flip-flops themselves (spec §1.8; the old stub's
		// "&= ~$10" here belonged to its inverted $01/$10 pairing).
		m_sonar_timer->adjust(attotime::from_ticks(u64(m_sonar_count) * 13500 + 1, 32768 * 45000));
	}
	else if (previous_sonar_power && !BIT(data, 1))
	{
		m_sonar_timer->adjust(attotime::never);
	}
}

void hero1_state::port_c300_clock_w(u8 data)
{
	m_clock_control = data;
	driver_tracef("port_c300_clock_w data=$%02X address=$%01X hold=%u read=%u write=%u cs=%u", data, m_clock_address, BIT(data, 4), BIT(data, 5), BIT(data, 6), BIT(data, 7));
	m_rtc->data_w(data & 0x0f);
	m_rtc->hold_w(BIT(data, 4));
	m_rtc->read_w(BIT(data, 5));
	m_rtc->write_w(BIT(data, 6));
	m_rtc->cs_w(BIT(data, 7));
}

void hero1_state::io_pia_a_w(u8 data)
{
	// Debug scaffold state only.  The real HERO 1 motor outputs are owned by
	// the decoded C2xx handlers above; letting this alias drive the same output
	// finders makes ROM and scaffold writes fight each other.
	m_debug_io_port_a = data;
}

void hero1_state::io_pia_b_w(u8 data)
{
	m_debug_io_port_b = data;
}

u8 hero1_state::speech_pia_b_r()
{
	return m_speech_request ? 0x80 : 0x00;
}

void hero1_state::speech_pia_a_w(u8 data)
{
	// Debug aperture state only.  The real ET-18 speech path is C240 data plus
	// the C2C0 bit-5 strobe, gated by C2E0 speech-board power.
	m_debug_speech_data = data;
}

void hero1_state::speech_pia_b_w(u8 data)
{
	m_debug_speech_control = data;
}

void hero1_state::speech_request_w(int state)
{
	m_speech_request = state ? 1 : 0;
	m_speech_ready = m_speech_request;
	driver_tracef("speech_request_w state=%d ready=%u pc=$%04X", state, m_speech_request, m_maincpu->pc() & 0xffff);
}

void hero1_state::set_speech_data(u8 data)
{
	m_speech_data = data;
	m_speech_phoneme = data & 0x3f;
	m_speech_inflection = (data >> 6) & 0x03;
}

void hero1_state::latch_speech_phoneme(bool ignore_power)
{
	if (!ignore_power && !m_speech_power)
		return;

	m_votrax->inflection_w(m_speech_inflection);
	m_votrax->write(m_speech_phoneme);
}

void hero1_state::latch_interrupt(u8 mask)
{
	// A source event clocks its LS73 flip-flop; it can only latch while
	// its U411 clear line is released (bit = 1). Sources held cleared by
	// a 0 bit in the clear latch drop their events — this is what the
	// ROM's low-battery self-disable idiom ($0F04 = $F7/$FB) relies on
	// (hero-1-motion-spec.md §1.8, §2.3).
	m_interrupt_status |= mask & m_interrupt_clear_latch;
	update_irq_line();
}

void hero1_state::group_pattern_w(int group, u8 pattern)
{
	pattern &= 0x0f;
	if (pattern == m_group_pattern[group])
		return;
	m_group_pattern[group] = pattern;
	for (int axis = 0; axis < AXIS_COUNT; axis++)
	{
		if (AXES[axis].group != group)
			continue;
		// A motor moves only when its pattern lines cycle AND its +12 V
		// supply is energized (spec §1.5); steering is hard-fed (§1.4).
		if (!axis_supplied(axis))
			continue;
		axis_apply_pattern(axis, pattern);
	}
}

bool hero1_state::axis_supplied(int axis) const
{
	const u8 mask = AXES[axis].supply_mask;
	return mask == 0 || (m_manual_port_out[6] & mask) != 0;
}

void hero1_state::axis_apply_pattern(int axis, u8 pattern)
{
	// De-energized at rest (the ROM writes pattern 0 when a move
	// completes, spec §2.4 step 1): the rotor holds its last detent, so
	// keep the pattern memory and model no motion.
	if (pattern == 0)
		return;

	const int to = phase_index(pattern);
	if (to < 0)
	{
		// Not a two-phase-on pattern: torque is indeterminate, model no
		// motion and forget the detent memory (approximation ledger A4).
		m_axis_pattern[axis] = 0;
		return;
	}

	const int from = phase_index(m_axis_pattern[axis]);
	if (from >= 0)
	{
		// Adjacent pattern transitions move exactly one full step in the
		// $F19B cycle's direction (spec §2.1/§3.1). The same pattern, an
		// opposite (distance-2) pattern, or re-energizing after rest
		// moves the model at most re-detent-distance zero — the ROM's
		// move-start phase reset can jump up to two full steps and the
		// physical outcome is load-dependent; the model absorbs it as no
		// motion (spec §2.4 note + approximation A4).
		const int delta = (to - from) & 3;
		if (delta == 1)
			axis_step(axis, 1);
		else if (delta == 3)
			axis_step(axis, -1);
	}
	m_axis_pattern[axis] = pattern;
}

void hero1_state::axis_energize(int axis)
{
	const u8 pattern = m_group_pattern[AXES[axis].group];
	m_axis_pattern[axis] = (phase_index(pattern) >= 0) ? pattern : 0;
}

attotime hero1_state::wheel_pulse_period() const
{
	// Speed follows the latch, so the ROM's own ramp ($F074 writes every
	// intermediate byte to $C2A0) produces a ramping pulse rate; each
	// pulse boundary picks up the current DAC value. Speed map is the
	// labeled A1 approximation (constants above); pulses-per-inch is the
	// CONFIRMED cart calibration.
	const u8 dac = m_manual_port_out[5] & 0x3f;
	const double inches_per_second = WHEEL_SPEED_DAC0_IN_PER_S
			+ (WHEEL_SPEED_DAC63_IN_PER_S - WHEEL_SPEED_DAC0_IN_PER_S) * double(dac) / 63.0;
	return attotime::from_double(1.0 / (inches_per_second * WHEEL_PULSES_PER_INCH));
}

void hero1_state::update_drive_model()
{
	// D6 = motor ON/OFF ("Q801 is used as a switch to turn off the motor
	// when the data at pin 27 is low", H1-TM p. 74 / spec §1.3). While ON
	// the wheel turns and the encoder disc produces one pulse per slot;
	// the first slot after starting from rest arrives one period out.
	// When OFF the model stops emitting immediately (no coast tail —
	// approximation ledger A5, documented model choice).
	const bool motor_on = BIT(m_manual_port_out[5], 6);
	if (!motor_on)
	{
		m_wheel_timer->adjust(attotime::never);
		m_wheel_pulse_pending = false;
	}
	else if (!m_wheel_pulse_pending)
	{
		m_wheel_timer->adjust(wheel_pulse_period());
		m_wheel_pulse_pending = true;
	}
}

TIMER_CALLBACK_MEMBER(hero1_state::wheel_pulse_tick)
{
	// One disc slot: optical pickup -> Schmitt U324A -> 74221 one-shot
	// (~7 ms) -> negative edge latches WHEEL DET $40 (spec §1.6/§1.8).
	// The ROM's $F21C handler counts these down to terminate drive moves
	// and advances both odometers ($0007/$0009) — completion is entirely
	// pulse-driven (spec §2.5/§2.7).
	m_wheel_pulses++;
	m_wheel_pulses_out = m_wheel_pulses;
	m_wheel_pulse_pending = false;
	latch_interrupt(0x40);
	update_drive_model();
}

void hero1_state::axis_step(int axis, int direction)
{
	// Clamp at the mechanical ends: unswitched axes stall harmlessly
	// against their stops while the ROM's RAM count keeps running —
	// exactly the Heath open-loop home (spec §2.6 Zero, approximation
	// A3); switched axes close their limit switches at these same ends
	// (§1.7), which the $C260 read derives from this position.
	const s32 stepped = m_axis_pos[axis] + direction;
	const s32 clamped = std::clamp(stepped, s32(0), AXES[axis].span);
	if (clamped != m_axis_pos[axis])
	{
		m_axis_pos[axis] = clamped;
		m_axis_positions[axis] = u32(clamped);
	}
}

void hero1_state::update_irq_line()
{
	m_maincpu->set_input_line(INPUT_LINE_IRQ0, m_interrupt_status ? ASSERT_LINE : CLEAR_LINE);
}

void hero1_state::update_speech_power()
{
	m_votrax->set_output_gain(0, m_speech_power ? 1.0 : 0.0);
}

TIMER_CALLBACK_MEMBER(hero1_state::executive_timer_tick)
{
	// $10 TIME CLOCK is the unconditional 1024 Hz tick derived from the
	// U315 RTC divider chain (H1-TM printed p. 32: RAM $0EFC "counts at
	// 1024 Hz"; hero-1-motion-spec.md §1.8/§2.3). It drives the ROM's
	// motor/speech engine at $F016 and is never gated by sonar power —
	// the sonar-ready source is $01 only (the old stub had the pairing
	// inverted; spec §4 item 3).
	latch_interrupt(0x10);
}

TIMER_CALLBACK_MEMBER(hero1_state::sonar_echo_tick)
{
	// SONAR (U408A): the range-counter echo latch fires once per ping.
	latch_interrupt(0x01);
}

void hero1_state::update_pendant_port(u8 data)
{
	// TRIGGER (U409B) clocks on the pendant trigger press edge; releasing
	// the trigger does not clear the flip-flop — only the U411 clear
	// latch does (hero-1-motion-spec.md §1.8).
	const bool was_pressed = BIT(m_pendant_port, 0);
	m_pendant_port = data;
	const bool trigger_pressed = BIT(m_pendant_port, 0);
	if (trigger_pressed && !was_pressed)
		latch_interrupt(0x20);
}

void hero1_state::set_pendant_function(bool arm)
{
	update_pendant_port((m_pendant_port & 0x7f) | (arm ? 0x80 : 0x00));
}

void hero1_state::set_pendant_joint_code(u8 code)
{
	update_pendant_port((m_pendant_port & 0x8f) | ((code & 0x07) << 4));
}

void hero1_state::set_pendant_motion(u8 motion)
{
	u8 data = m_pendant_port | 0x0c;
	if (motion == 1)
		data &= ~0x04; // LEFT pressed, active-low D2
	else if (motion == 2)
		data &= ~0x08; // RIGHT pressed, active-low D3
	update_pendant_port(data);
}

void hero1_state::set_pendant_trigger(bool pressed)
{
	update_pendant_port(pressed ? (m_pendant_port | 0x01) : (m_pendant_port & ~0x01));
}

void hero1_state::experimental_serial_rxd_w(int state)
{
	m_experimental_serial_rxd = state ? 1 : 0;
}

static DEVICE_INPUT_DEFAULTS_START(hero1_basic_serial)
	DEVICE_INPUT_DEFAULTS("RS232_TXBAUD", 0xff, RS232_BAUD_9600)
	DEVICE_INPUT_DEFAULTS("RS232_RXBAUD", 0xff, RS232_BAUD_9600)
	DEVICE_INPUT_DEFAULTS("RS232_DATABITS", 0xff, RS232_DATABITS_7)
	DEVICE_INPUT_DEFAULTS("RS232_PARITY", 0xff, RS232_PARITY_NONE)
	DEVICE_INPUT_DEFAULTS("RS232_STOPBITS", 0xff, RS232_STOPBITS_1)
DEVICE_INPUT_DEFAULTS_END

static INPUT_PORTS_START(hero1)
	PORT_START("KEYCOL0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("D") PORT_CODE(KEYCODE_D)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("A") PORT_CODE(KEYCODE_A)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("7") PORT_CODE(KEYCODE_7)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("4") PORT_CODE(KEYCODE_4)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("1") PORT_CODE(KEYCODE_1)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("0") PORT_CODE(KEYCODE_0)
	PORT_BIT(0xc0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("KEYCOL1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("E") PORT_CODE(KEYCODE_E)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("B") PORT_CODE(KEYCODE_B)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("8") PORT_CODE(KEYCODE_8)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("5") PORT_CODE(KEYCODE_5)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("2") PORT_CODE(KEYCODE_2)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0xc0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("KEYCOL2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("F") PORT_CODE(KEYCODE_F)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("C") PORT_CODE(KEYCODE_C)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("9") PORT_CODE(KEYCODE_9)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("6") PORT_CODE(KEYCODE_6)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("3") PORT_CODE(KEYCODE_3)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0xc0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("BASIC_BAUD")
	PORT_CONFNAME(0x07, 0x00, "ETW-18-10 baud rate")
	PORT_CONFSETTING(0x00, "9600")
	PORT_CONFSETTING(0x01, "4800")
	PORT_CONFSETTING(0x02, "2400")
	PORT_CONFSETTING(0x03, "1200")
	PORT_CONFSETTING(0x04, "600")
	PORT_CONFSETTING(0x07, "300")
INPUT_PORTS_END

void hero1_state::hero1(machine_config &config)
{
	M6808(config, m_maincpu, 4_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &hero1_state::mem_map);

	SPEAKER(config, "mono").front_center();

	PIA6821(config, m_display_pia);
	m_display_pia->readpa_handler().set(FUNC(hero1_state::display_pia_a_r));
	m_display_pia->writepa_handler().set(FUNC(hero1_state::display_pia_a_w));
	m_display_pia->writepb_handler().set(FUNC(hero1_state::display_pia_b_w));

	PIA6821(config, m_keypad_pia);
	m_keypad_pia->readpa_handler().set(FUNC(hero1_state::keypad_pia_a_r));
	m_keypad_pia->writepb_handler().set(FUNC(hero1_state::keypad_pia_b_w));

	PIA6821(config, m_io_pia);
	m_io_pia->writepa_handler().set(FUNC(hero1_state::io_pia_a_w));
	m_io_pia->writepb_handler().set(FUNC(hero1_state::io_pia_b_w));

	PIA6821(config, m_speech_pia);
	m_speech_pia->writepa_handler().set(FUNC(hero1_state::speech_pia_a_w));
	m_speech_pia->readpb_handler().set(FUNC(hero1_state::speech_pia_b_r));
	m_speech_pia->writepb_handler().set(FUNC(hero1_state::speech_pia_b_w));

	ACIA6850(config, m_acia, 0);
	m_acia->irq_handler().set_inputline(m_maincpu, INPUT_LINE_IRQ0);
	m_acia->txd_handler().set(m_rs232, FUNC(rs232_port_device::write_txd));
	m_acia->rts_handler().set(m_rs232, FUNC(rs232_port_device::write_rts));

	// Provisional cassette/serial ACIA clock: enough to exercise the MC6850
	// transmit/receive state machines while the HERO 1 tape/serial clock source is verified.
	clock_device &acia_clock(CLOCK(config, "acia_clock", 153600));
	acia_clock.signal_handler().set(m_acia, FUNC(acia6850_device::write_txc));
	acia_clock.signal_handler().append(m_acia, FUNC(acia6850_device::write_rxc));

	RS232_PORT(config, m_rs232, default_rs232_devices, "terminal");
	m_rs232->set_option_device_input_defaults("null_modem", DEVICE_INPUT_DEFAULTS_NAME(hero1_basic_serial));
	m_rs232->rxd_handler().set(m_acia, FUNC(acia6850_device::write_rxd));
	m_rs232->rxd_handler().append(FUNC(hero1_state::experimental_serial_rxd_w));
	m_rs232->cts_handler().set(m_acia, FUNC(acia6850_device::write_cts));
	m_rs232->dsr_handler().set(m_acia, FUNC(acia6850_device::write_dcd));

	MSM5832(config, m_rtc, 32.768_kHz_XTAL);

	VOTRAX_SC01(config, m_votrax, 720000);
	m_votrax->ar_callback().set(FUNC(hero1_state::speech_request_w));
	m_votrax->add_route(ALL_OUTPUTS, "mono", 0.5);
}

ROM_START(hero1)
	ROM_REGION(0x2000, "maincpu", ROMREGION_ERASEFF)
	ROM_SYSTEM_BIOS(0, "v10", "System ROM v1.0")
	ROMX_LOAD("hero1_system_v10.bin", 0x0000, 0x2000, CRC(bc1e0064) SHA1(705136651c4c1674588a9c3f1e29f0c2a053c9e7), ROM_BIOS(0))
	ROM_SYSTEM_BIOS(1, "v13", "System ROM v1.3")
	ROMX_LOAD("hero1_system_v13.bin", 0x0000, 0x2000, CRC(4972949e) SHA1(a8eb3186172c66b7fe0a6f4630a8fc32491a795a), ROM_BIOS(1))
	ROM_SYSTEM_BIOS(2, "v1u", "System ROM v1.U")
	ROMX_LOAD("hero1_system_v1u.bin", 0x0000, 0x2000, CRC(7f1df589) SHA1(aacea2281d224c800377ebde01fdd3f52d61ff6e), ROM_BIOS(2))

	ROM_REGION(0x2000, "basic", ROMREGION_ERASEFF)
	ROM_LOAD("hero1_basic.bin", 0x0000, 0x1744, CRC(b10ed312) SHA1(0b85da5bd3fbcf0938ee77cfac615418d8ad8b9a))
ROM_END

} // anonymous namespace

COMP(1982, hero1, 0, 0, hero1, hero1, hero1_state, empty_init, "Heathkit", "HERO 1 (ET-18)", MACHINE_IMPERFECT_SOUND | MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE)
