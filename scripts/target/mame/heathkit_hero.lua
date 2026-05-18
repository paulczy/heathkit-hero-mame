-- license:BSD-3-Clause
-- copyright-holders:Paul Czywczynski

---------------------------------------------------------------------------
--
--   heathkit_hero.lua
--
--   Slim MAME subtarget for Heathkit HERO 1 and HERO Jr VS Code runtimes.
--   Use make SUBTARGET=heathkit_hero to build.
--
---------------------------------------------------------------------------

CPUS["IE15"] = true
CPUS["M6800"] = true
CPUS["Z80"] = true

SOUNDS["VOTRAX_SC01"] = true
SOUNDS["AY8910"] = true
SOUNDS["BEEP"] = true

VIDEOS["MC6845"] = true

MACHINES["6821PIA"] = true
MACHINES["ACIA6850"] = true
MACHINES["IE15"] = true
MACHINES["INPUT_MERGER"] = true
MACHINES["INS8250"] = true
MACHINES["MC146818"] = true
MACHINES["MM5740"] = true
MACHINES["MSM5832"] = true
MACHINES["PCF8573"] = true
MACHINES["SWTPC8212"] = true
MACHINES["VOTRAXTNT"] = true
MACHINES["Z80DAISY"] = true

BUSES["GENERIC"] = true
BUSES["HEATHZENITH_H19"] = true
BUSES["RS232"] = true
BUSES["SUNKBD"] = true

function createProjects_mame_heathkit_hero(_target, _subtarget)
	project ("mame_heathkit_hero")
	targetsubdir(_target .."_" .. _subtarget)
	kind (LIBTYPE)
	uuid (os.uuid("drv-mame-heathkit-hero"))
	addprojectflags()
	precompiledheaders_novs()

	includedirs {
		MAME_DIR .. "src/osd",
		MAME_DIR .. "src/emu",
		MAME_DIR .. "src/devices",
		MAME_DIR .. "src/mame/shared",
		MAME_DIR .. "src/lib",
		MAME_DIR .. "src/lib/util",
		MAME_DIR .. "3rdparty",
		GEN_DIR  .. "mame/layout",
	}

	files {
		MAME_DIR .. "src/mame/heathkit/hero1.cpp",
		MAME_DIR .. "src/mame/heathkit/herojr.cpp",
	}
end

function linkProjects_mame_heathkit_hero(_target, _subtarget)
	links {
		"mame_heathkit_hero",
	}
end
