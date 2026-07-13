-- license:BSD-3-Clause
-- copyright-holders:Paul Czywczynski

local bridge = {}

-- Transport: the vendored LuaSocket core, preloaded by the fork's
-- luaengine.cpp as "socket.core". It is REQUIRED — a missing module means
-- the plugin is running against a non-fork or stale engine build, and the
-- bridge must refuse loudly instead of degrading (the old emu.file fallback
-- transport was behind the historical contention/race problems and has been
-- deleted; there is exactly one transport).
local socket_core_ok, socket_core = pcall(require, "socket.core")
if not socket_core_ok then
  local message = "heathkit_hero1jr_debug: FATAL: required LuaSocket transport (socket.core) is unavailable: "
    .. tostring(socket_core)
    .. " — the bundled MAME fork preloads it in luaengine.cpp; rebuild/re-vendor the engine (docs/building-mame-and-vsix.md)"
  emu.print_error(message)
  error(message, 0)
end

-- socket.core exposes the raw TCP master object; bind/listen explicitly
-- (the pure-Lua socket.bind convenience wrapper is intentionally not
-- vendored). tcp4() is required, not tcp(): the AF_UNSPEC master defers fd
-- creation to bind time, so setoption before bind would be a silent no-op
-- on an invalid descriptor. reuseaddr is needed for immediate rebind after
-- a MAME-side active close leaves the port in TIME_WAIT (POSIX; G0-06);
-- the win32 tradeoff (SO_REUSEADDR permits a same-user double bind) is
-- accepted because the product serializes sessions and the conformance
-- runner preflights the port.
local function bind_tcp_server(bind_host, bind_port)
  local server, create_err = socket_core.tcp4()
  if not server then
    error("heathkit_hero1jr_debug: failed to create bridge TCP socket: " .. tostring(create_err), 0)
  end

  local option_ok, option_err = server:setoption("reuseaddr", true)
  if not option_ok then
    server:close()
    error("heathkit_hero1jr_debug: failed to set bridge socket reuseaddr: " .. tostring(option_err), 0)
  end

  local bind_ok, bind_err = server:bind(bind_host, bind_port)
  if not bind_ok then
    server:close()
    error("heathkit_hero1jr_debug: failed to bind bridge socket on "
      .. bind_host .. ":" .. tostring(bind_port) .. ": " .. tostring(bind_err), 0)
  end

  local listen_ok, listen_err = server:listen(1)
  if not listen_ok then
    server:close()
    error("heathkit_hero1jr_debug: failed to listen on bridge socket: " .. tostring(listen_err), 0)
  end

  return server
end

local function parse_trace_categories(value)
  local categories = {}
  local raw = tostring(value or ""):lower()
  if raw == "1" or raw == "true" or raw == "all" then
    categories["all"] = true
    return categories
  end

  for category in raw:gmatch("[^,%s]+") do
    categories[category] = true
  end

  return categories
end

local host = os.getenv("HEATHKIT_HERO_BRIDGE_HOST") or "127.0.0.1"
local port = tonumber(os.getenv("HEATHKIT_HERO_BRIDGE_PORT") or "6808")
local autostart_monitor = os.getenv("HEATHKIT_HERO_BRIDGE_AUTOSTART_MONITOR") == "1"
local bridge_trace_categories = parse_trace_categories(os.getenv("HEATHKIT_HERO_BRIDGE_TRACE"))
local session_id = os.getenv("HEATHKIT_HERO_SESSION_ID") or ""
local herojr_initial_sleep = os.getenv("HEATHKIT_HEROJR_INITIAL_SLEEP") == "1"
local autostart_monitor_pending = autostart_monitor
local herojr_initial_sleep_pending = herojr_initial_sleep
local sensor_base = 0xd100
local herojr_sensor_base = 0xd860
local keypad_base = 0xd104
local keypad_end = 0xd106
local server = nil
local clients = {}
local receive_buffers = {}
local breakpoints_by_addr = {}
local breakpoints_by_index = {}
local read_watchpoints_by_addr = {}
local read_watchpoints_by_index = {}
local read_watchpoint_one_shot_by_index = {}
local temp_step_breakpoints = {}
local last_stop_pc = nil
local last_io_state_encoded = nil
local last_io_broadcast_time = 0
local sensor_state = {
  sonarDistanceInches = 48,
  lightLevel = 50,
  soundLevel = 0,
  motionDetected = false,
  tapeIn = true,
  pendant = {
    ["function"] = "ARM",
    joint = "N",
    rotary = "N",
    motion = "CLEAR",
    trigger = false,
    leftPressed = false,
    rightPressed = false,
    triggerPressed = false,
    port = 0x8e
  }
}
local keypad_state = {}
local keypad_columns = { 0x3f, 0x3f, 0x3f }
local function log_prefix()
  if session_id ~= "" then
    return "heathkit_hero1jr_debug[" .. session_id .. "]"
  end

  return "heathkit_hero1jr_debug"
end

local function trace_enabled(category)
  return bridge_trace_categories["all"] or bridge_trace_categories[category]
end

local function trace(category, message)
  if message == nil then
    message = category
    category = "rpc"
  end

  if trace_enabled(category) then
    emu.print_info(log_prefix() .. ": trace:" .. category .. ": " .. message)
  end
end

local function trace_address(addr)
  return "$" .. string.format("%04X", addr & 0xffff)
end

local function trace_address_list(addresses)
  local values = {}
  for addr, _ in pairs(addresses) do
    values[#values + 1] = trace_address(addr)
  end
  table.sort(values)
  return table.concat(values, ", ")
end

local key_map = {
  D = { column = 1, bit = 0x01 },
  A = { column = 1, bit = 0x02 },
  ["7"] = { column = 1, bit = 0x04 },
  ["4"] = { column = 1, bit = 0x08 },
  ["1"] = { column = 1, bit = 0x10 },
  ["0"] = { column = 1, bit = 0x20 },
  E = { column = 2, bit = 0x01 },
  B = { column = 2, bit = 0x02 },
  ["8"] = { column = 2, bit = 0x04 },
  ["5"] = { column = 2, bit = 0x08 },
  ["2"] = { column = 2, bit = 0x10 },
  F = { column = 3, bit = 0x01 },
  C = { column = 3, bit = 0x02 },
  ["9"] = { column = 3, bit = 0x04 },
  ["6"] = { column = 3, bit = 0x08 },
  ["3"] = { column = 3, bit = 0x10 }
}
local write_u8

local pendant_joint_codes = {
  N = 0,
  PIVOT = 1,
  PIVOT_F = 1,
  ROTATE = 2,
  GRIP = 3,
  SHOULDER = 5,
  PIVOT_R = 5,
  EXTEND = 6,
  HEAD = 7,
  WRIST_PIVOT = 1,
  WRIST_ROTATE = 2,
  ARM_PIVOT = 5,
  ARM_EXTEND = 6
}

local function pendant_joint_name_from_code(code)
  if code == 0 then
    return "N"
  elseif code == 1 then
    return "PIVOT"
  elseif code == 2 then
    return "ROTATE"
  elseif code == 3 then
    return "GRIP"
  elseif code == 5 then
    return "SHOULDER"
  elseif code == 6 then
    return "EXTEND"
  elseif code == 7 then
    return "HEAD"
  end
  return "N"
end

local function update_pendant_state_from_port(port)
  local byte = (tonumber(port) or 0x8e) & 0xff
  local left_pressed = (byte & 0x04) == 0
  local right_pressed = (byte & 0x08) == 0
  local trigger_pressed = (byte & 0x01) ~= 0
  local joint = pendant_joint_name_from_code((byte >> 4) & 0x07)
  sensor_state.pendant.port = byte
  sensor_state.pendant["function"] = (byte & 0x80) ~= 0 and "ARM" or "BODY"
  sensor_state.pendant.joint = joint
  sensor_state.pendant.rotary = joint
  sensor_state.pendant.leftPressed = left_pressed
  sensor_state.pendant.rightPressed = right_pressed
  sensor_state.pendant.triggerPressed = trigger_pressed
  sensor_state.pendant.motion = left_pressed and "LEFT" or right_pressed and "RIGHT" or "CLEAR"
  sensor_state.pendant.trigger = trigger_pressed
end

local function pendant_port_from_state()
  local fn = sensor_state.pendant["function"] == "ARM" and 0x80 or 0x00
  local joint = (pendant_joint_codes[sensor_state.pendant.joint] or pendant_joint_codes[sensor_state.pendant.rotary] or pendant_joint_codes.N) << 4
  local right = sensor_state.pendant.rightPressed and 0x00 or 0x08
  local left = sensor_state.pendant.leftPressed and 0x00 or 0x04
  local fixed = 0x02
  local trigger = sensor_state.pendant.triggerPressed and 0x01 or 0x00
  return (fn | joint | right | left | fixed | trigger) & 0xff
end

local function write_pendant_port(port)
  update_pendant_state_from_port(port)
  write_u8(sensor_base + 0x0c, sensor_state.pendant.port)
end
local function herojr_keypad_byte()
  local value = 0xff
  for key, is_pressed in pairs(keypad_state) do
    if is_pressed then
      local nibble = tonumber(key, 16)
      if nibble then
        local column_mask = 1 << (3 - (nibble & 0x03))
        local row_mask = 0x10 << ((nibble >> 2) & 0x03)
        value = value & (~(column_mask | row_mask) & 0xff)
      end
    end
  end
  return value
end

local function herojr_keypad_nibble(key)
  local aliases = {
    ["RT-1"] = 0x0,
    ENTER = 0xf,
    PLAN = 0xd,
    SETUP = 0xe,
    ["SET UP"] = 0xe,
    DEMO = 0xa,
    GUARD = 0xb,
    ALARM = 0xc,
    SPEAK = 0x7,
    GAB = 0x8,
    POET = 0x9,
    SING = 0x4,
    PLAY = 0x5,
    HELP = 0x6
  }
  return tonumber(key, 16) or aliases[key]
end

local function set_herojr_keypad_field(key, pressed)
  local system = manager.machine.system
  if not system or system.name ~= "herojr" then
    return
  end

  local nibble = herojr_keypad_nibble(key)
  if not nibble then
    return
  end

  local row = (nibble >> 2) & 0x03
  local column = 3 - (nibble & 0x03)
  local port = manager.machine.ioport.ports[":KEY" .. tostring(row)] or manager.machine.ioport.ports["KEY" .. tostring(row)]
  if not port then
    return
  end

  local field = port:field(1 << column)
  if field then
    field:set_value(pressed and 1 or 0)
  end
end

local function set_herojr_sleep_norm_field(norm)
  if not manager.machine then
    return false
  end

  local system = manager.machine.system
  if not system or system.name ~= "herojr" then
    return false
  end

  local port = manager.machine.ioport.ports[":SLEEP_NORM"] or manager.machine.ioport.ports["SLEEP_NORM"]
  if not port then
    return false
  end

  local field = port:field(0x01)
  if not field then
    return false
  end

  -- SLEEP_NORM defaults to NORM in the driver. MAME folds programmatic
  -- digital values through that default on read, so the held state reads Sleep.
  field:set_value(norm and 0 or 1)
  return true
end

local function set_herojr_reset_field(pressed)
  if not manager.machine then
    return false
  end

  local system = manager.machine.system
  if not system or system.name ~= "herojr" then
    return false
  end

  local port = manager.machine.ioport.ports[":RESET"] or manager.machine.ioport.ports["RESET"]
  if not port then
    return false
  end

  local field = port:field(0x01)
  if not field then
    return false
  end

  field:set_value(pressed and 1 or 0)
  return true
end
local subscriptions = {}
local pending_step_reason = nil
local pending_step_start_pc = nil
local pending_step_seen_run = false
-- reset_machine engage window (HERO Jr panel RESET pulse). Non-nil while
-- the modeled panel button is held: the EMULATED-time instant at which the
-- pulse is released. MAME samples ioport fields only at emulated frame
-- boundaries (ioport_manager::frame_update), so the hold must be measured
-- in emulated time — a pump-frame countdown kept running while the machine
-- was PAUSED (video repaints pump without emulated progress), letting the
-- 0->1->0 pulse complete unsampled and silently losing the reset
-- (deterministic: conformance/tools/reproHerojrResetLoss.js, and the
-- G2J-05 $AA gate red "Missing $9FDD ... PCs $9FE7",
-- conformance-2026-07-12T13-34-48Z.log).
local herojr_reset_release_at = nil
-- Deferred reset_machine acknowledgement (R2 remediation, 2026-07-13).
-- machine:soft_reset() only SCHEDULES a zero-time scheduler timer
-- (running_machine::schedule_soft_reset, machine.cpp), and the herojr panel
-- pulse only completes after emulated frames elapse — so answering the RPC
-- immediately let the client pause the machine BEFORE the reset had
-- executed. The pending soft reset then fired on the client's eventual
-- continue, re-vectoring the CPU into the boot ROM and discarding the
-- client's post-"reset" setup (PC-to-entry): G3-01's "verified but never
-- firing" read watchpoint (tech-debt #13's ~30%/launch dead install) was a
-- perfectly-installed watchpoint whose fixture program never ran
-- (conformance/tools/reproR2WatchpointDeadInstall.ts, boot-ROM PC samples
-- on every dead launch). The response is therefore withheld until the
-- machine has OBSERVABLY reset:
--   hero1:  the machine-reset notifier fired (the scheduled soft reset ran
--           — a zero-delay timer, so no instruction can execute first);
--   herojr: the pulse release write happened AND either 40 ms of emulated
--           time passed (>= two 60 Hz ioport samples, so the deasserted
--           line was sampled and the CPU is out of reset) or a genuine
--           debugger stop was broadcast (instructions executed, so the CPU
--           is out of reset; without this escape a client breakpoint on
--           the boot path would pause emulated time and deadlock the
--           acknowledgement — tier-1 reset-entry does exactly that).
local pending_reset_ack = nil       -- { client, id }; single-client protocol
local machine_reset_count = 0       -- bumped by the machine-reset notifier
local machine_reset_baseline = 0    -- count captured when the reset was requested
local herojr_reset_respond_at = nil -- emulated instant the release is provably sampled
local reset_ack_stop_seen = false   -- genuine stop broadcast since reset_machine
local DEFER_RESPONSE = {}           -- handler sentinel: the pump owns the response
local supported_commands = {
  "get_capabilities",
  "get_registers",
  "set_registers",
  "read_mem",
  "write_mem",
  "set_breakpoint",
  "clear_breakpoint",
  "set_read_watchpoint",
  "clear_read_watchpoint",
  "continue",
  "pause",
  "pause_for_setup",
  "step_in",
  "step_over",
  "step_out",
  "run_for_ms",
  "get_io_state",
  "set_sensor",
  "press_key",
  "release_key",
  "set_sleep_norm",
  "initialize_herojr_warm_context",
  "reset_machine"
}

local register_names = {
  pc = { "PC", "rPC", "CURPC" },
  sp = { "SP", "S", "rSP" },
  x = { "X", "IX", "rX" },
  a = { "A", "ACCA", "rA" },
  b = { "B", "ACCB", "rB" },
  cc = { "CC", "CCR", "rCC", "CURFLAGS" },
  wai = { "WAI" }
}

local function cpu()
  return manager.machine.devices[":maincpu"]
end

local function system_name()
  local system = manager.machine.system
  if system and system.name then
    return system.name
  end

  return "hero1"
end

local function system_description()
  local system = manager.machine.system
  if system and system.description then
    return system.description
  end

  return system_name()
end

local function output_prefix()
  if system_name() == "herojr" then
    return "herojr"
  end

  return "hero1"
end

local function profile_name()
  if system_name() == "herojr" then
    return "herojr"
  end

  return "hero1"
end

local function program_space()
  local maincpu = cpu()
  if not maincpu or not maincpu.spaces then
    error("main CPU program space is unavailable")
  end

  return maincpu.spaces["program"]
end

local function program_space_available()
  local maincpu = cpu()
  return maincpu and maincpu.spaces and maincpu.spaces["program"]
end

local function cpu_debug()
  local maincpu = cpu()
  if not maincpu or not maincpu.debug then
    error("MAME debugger is unavailable; start MAME with -debug")
  end

  return maincpu.debug
end

local function cpu_debug_available()
  local maincpu = cpu()
  return maincpu and maincpu.debug and manager.machine and manager.machine.debugger
end

local function ensure_debugger_stopped_for_watchpoint(action)
  local state = manager.machine.debugger and manager.machine.debugger.execution_state or "unknown"
  if state ~= "stop" then
    error(action .. " requires debugger state stop; call pause_for_setup before changing read watchpoints")
  end
end

local function debugger_manager()
  if not manager.machine.debugger then
    error("MAME debugger is unavailable; start MAME with -debug")
  end

  return manager.machine.debugger
end

local function focus_maincpu_debugger()
  debugger_manager():command("focus :maincpu")
end

local function find_state_entry(name)
  local maincpu = cpu()
  if not maincpu or not maincpu.state then
    return nil
  end

  for _, candidate in ipairs(register_names[name] or {}) do
    local entry = maincpu.state[candidate]
    if entry then
      return entry
    end
  end

  return nil
end

local function get_register(name)
  local entry = find_state_entry(name)
  if not entry then
    return 0
  end

  return entry.value
end

local function set_register(name, value)
  local entry = find_state_entry(name)
  if entry then
    entry.value = value
  end
end

local function hex_pc()
  return string.format("%04X", get_register("pc") & 0xffff)
end

-- Explicit JSON null for table fields (a plain nil value would simply drop
-- the key). Used for protocol-level errors that correlate to no request id:
-- real request ids are numbers (the product client starts at 1), so id:null
-- can never collide with a pending request.
local json_null = setmetatable({}, { __tostring = function() return "null" end })

local function encode_json(value)
  if value == json_null then
    return "null"
  end
  local value_type = type(value)
  if value_type == "nil" then
    return "null"
  elseif value_type == "boolean" then
    return value and "true" or "false"
  elseif value_type == "number" then
    return tostring(value)
  elseif value_type == "string" then
    return '"' .. value:gsub('[%z\1-\31\\"]', function(char)
      local replacements = {
        ['"'] = '\\"',
        ['\\'] = '\\\\',
        ['\b'] = '\\b',
        ['\f'] = '\\f',
        ['\n'] = '\\n',
        ['\r'] = '\\r',
        ['\t'] = '\\t'
      }
      return replacements[char] or string.format("\\u%04x", char:byte())
    end) .. '"'
  elseif value_type == "table" then
    local is_array = true
    local max = 0
    for key, _ in pairs(value) do
      if type(key) ~= "number" then
        is_array = false
        break
      end
      if key > max then
        max = key
      end
    end

    local parts = {}
    if is_array then
      for i = 1, max do
        parts[#parts + 1] = encode_json(value[i])
      end
      return "[" .. table.concat(parts, ",") .. "]"
    end

    for key, item in pairs(value) do
      parts[#parts + 1] = encode_json(tostring(key)) .. ":" .. encode_json(item)
    end
    return "{" .. table.concat(parts, ",") .. "}"
  end

  error("cannot encode JSON value of type " .. value_type)
end

local function diagnostic_event(scope, message, details)
  if not trace_enabled("diagnostic") then
    return
  end

  emu.print_info("HERO_DIAGNOSTIC " .. encode_json({
    scope = scope,
    message = message,
    details = details or {}
  }))
end

local function decode_json(text)
  local pos = 1

  local function skip_ws()
    while text:sub(pos, pos):match("%s") do
      pos = pos + 1
    end
  end

  local function parse_string()
    pos = pos + 1
    local result = {}
    while pos <= #text do
      local char = text:sub(pos, pos)
      if char == '"' then
        pos = pos + 1
        return table.concat(result)
      elseif char == "\\" then
        local escape = text:sub(pos + 1, pos + 1)
        local replacements = { ['"'] = '"', ["\\"] = "\\", ["/"] = "/", b = "\b", f = "\f", n = "\n", r = "\r", t = "\t" }
        if escape == "u" then
          result[#result + 1] = "?"
          pos = pos + 6
        else
          result[#result + 1] = replacements[escape] or escape
          pos = pos + 2
        end
      else
        result[#result + 1] = char
        pos = pos + 1
      end
    end

    error("unterminated JSON string")
  end

  local parse_value

  local function parse_array()
    pos = pos + 1
    local result = {}
    skip_ws()
    if text:sub(pos, pos) == "]" then
      pos = pos + 1
      return result
    end

    while true do
      result[#result + 1] = parse_value()
      skip_ws()
      local char = text:sub(pos, pos)
      if char == "]" then
        pos = pos + 1
        return result
      elseif char ~= "," then
        error("expected ',' or ']' in JSON array")
      end
      pos = pos + 1
    end
  end

  local function parse_object()
    pos = pos + 1
    local result = {}
    skip_ws()
    if text:sub(pos, pos) == "}" then
      pos = pos + 1
      return result
    end

    while true do
      skip_ws()
      if text:sub(pos, pos) ~= '"' then
        error("expected JSON object key")
      end
      local key = parse_string()
      skip_ws()
      if text:sub(pos, pos) ~= ":" then
        error("expected ':' after JSON object key")
      end
      pos = pos + 1
      result[key] = parse_value()
      skip_ws()
      local char = text:sub(pos, pos)
      if char == "}" then
        pos = pos + 1
        return result
      elseif char ~= "," then
        error("expected ',' or '}' in JSON object")
      end
      pos = pos + 1
    end
  end

  function parse_value()
    skip_ws()
    local char = text:sub(pos, pos)
    if char == '"' then
      return parse_string()
    elseif char == "{" then
      return parse_object()
    elseif char == "[" then
      return parse_array()
    elseif text:sub(pos, pos + 3) == "true" then
      pos = pos + 4
      return true
    elseif text:sub(pos, pos + 4) == "false" then
      pos = pos + 5
      return false
    elseif text:sub(pos, pos + 3) == "null" then
      pos = pos + 4
      return nil
    end

    local start = pos
    while text:sub(pos, pos):match("[%d%+%-%.eE]") do
      pos = pos + 1
    end
    local number = tonumber(text:sub(start, pos - 1))
    if number == nil then
      error("invalid JSON value at byte " .. tostring(start))
    end
    return number
  end

  local result = parse_value()
  skip_ws()
  if pos <= #text then
    error("trailing data after JSON value")
  end
  return result
end

-- Nonblocking LuaSocket send can deliver a prefix and return "timeout";
-- dropping the remainder would corrupt the NDJSON stream mid-line. Drain
-- with a short bound instead (os.clock: CPU time, which this hot retry
-- loop accrues continuously, so the bound terminates): a healthy peer
-- empties a loopback buffer immediately, and a peer wedged past the bound
-- is dead — error out so the caller's pcall closes the client rather than
-- ever sending a torn line.
local SEND_DRAIN_DEADLINE_SECONDS = 2.0

local function send_all(client, payload)
  local first = 1
  local deadline = os.clock() + SEND_DRAIN_DEADLINE_SECONDS
  while true do
    local sent, err, last_sent = client:send(payload, first)
    if sent then
      return
    end
    if err ~= "timeout" then
      error("bridge client send failed: " .. tostring(err))
    end
    first = (last_sent or (first - 1)) + 1
    if os.clock() > deadline then
      error("bridge client send stalled (peer not draining); dropping client")
    end
  end
end

local function send_line(client, value)
  send_all(client, encode_json(value) .. "\n")
end

-- Single removal path: close, drop per-client framing state, and remove by
-- identity (an index captured before a nested removal can go stale).
local drop_framing_state

local function drop_client(client)
  if pending_reset_ack and pending_reset_ack.client == client then
    pending_reset_ack = nil
  end
  pcall(function() client:close() end)
  receive_buffers[client] = nil
  if drop_framing_state then
    drop_framing_state(client)
  end
  for index, candidate in ipairs(clients) do
    if candidate == client then
      table.remove(clients, index)
      return
    end
  end
end

-- Observations need machine-time correlation: wall clock cannot resolve
-- sub-bridge-latency effects (G1J-06 echo delays, G2J-09 phoneme pacing).
local function emulated_time_seconds()
  local ok, now = pcall(emu.time)
  if ok and type(now) == "number" then
    return now
  end
  return 0
end

local function broadcast_event(name, payload)
  local stamped = { event = name, payload = payload or {}, emulatedTimeSeconds = emulated_time_seconds() }
  for _, client in ipairs({ table.unpack(clients) }) do
    local ok = pcall(send_line, client, stamped)
    if not ok then
      drop_client(client)
    end
  end
end

local function get_registers()
  return {
    pc = get_register("pc"),
    sp = get_register("sp"),
    x = get_register("x"),
    a = get_register("a"),
    b = get_register("b"),
    cc = get_register("cc")
  }
end

local function hero1_hardware_capabilities()
  return {
    debugAddresses = {
      displayPortA = 0xd000,
      displayPortB = 0xd002,
      ioPortA = 0xd008,
      ioPortB = 0xd00a,
      speechPortA = 0xd020,
      speechPortB = 0xd022,
      sensorBase = sensor_base,
      keypadBase = keypad_base,
      keypadEnd = keypad_end
    },
    display = {
      digits = 6
    },
    optional = {
      teachingPendant = true,
      experimentalBoard = true
    }
  }
end

local function herojr_hardware_capabilities()
  -- Socket population follows the launched configuration (-ramsize): the
  -- driver publishes top-of-populated-RAM as herojr_ram_top ($07FF stock 2K
  -- 6116 at U203, image mirrored x4 across the decoded window; $3FFF
  -- expanded 16K = 8K at U203+U204). Absent sockets are open bus.
  local ram_top = 0x07ff
  local ok, value = pcall(function() return manager.machine.output:get_value("herojr_ram_top") end)
  if ok and type(value) == "number" and value > 0 then
    ram_top = value
  end
  local expanded = ram_top >= 0x3fff
  return {
    memory = {
      ramTop = ram_top,
      u203 = expanded
        and { startAddress = 0x0000, endAddress = 0x1fff, kind = "ram", device = "8Kx8" }
        or { startAddress = 0x0000, endAddress = 0x1fff, kind = "ram", device = "2Kx8 mirrored x4" },
      u204 = expanded
        and { startAddress = 0x2000, endAddress = 0x3fff, kind = "ram", device = "8Kx8" }
        or { startAddress = 0x2000, endAddress = 0x3fff, kind = "openBus" },
      u205 = { startAddress = 0x4000, endAddress = 0x5fff, kind = "openBus" },
      u206 = { startAddress = 0x6000, endAddress = 0x7fff, kind = "cartridgeRom", slot = "cartslot" },
      monitorRom = { startAddress = 0x8000, endAddress = 0xffff, ioHoleStart = 0xd800, ioHoleEnd = 0xdfff }
    },
    io = { rtc = 0xd810, u214 = 0xd820, u215 = 0xd840, rs232 = 0xd880 },
    cartridge = { startAddress = 0x6000, endAddress = 0x7fff, size = 0x2000, slot = "cartslot", banking = "unsupported" },
    debugAddresses = {
      keypad = 0xd820,
      driveControl = 0xd821,
      motionDetector = 0xd821,
      rtcSqwStatus = 0xd822,
      wheelFeedback = 0xd823,
      rtcStart = 0xd81e,
      rtcEnd = 0xd81f,
      dataLeds = 0xd840,
      control = 0xd841,
      speechStatus = 0xd842,
      sonarEchoStatus = 0xd843,
      sensorBase = herojr_sensor_base,
      rs232Data = 0xd880
    },
    display = {
      digits = 6
    },
    optional = {
      teachingPendant = false,
      cartridge = true
    }
  }
end

local function hardware_capabilities()
  return {
    active = profile_name(),
    hero1 = hero1_hardware_capabilities(),
    herojr = herojr_hardware_capabilities()
  }
end

local function cpu_clock_hz()
  local maincpu = cpu()
  if not maincpu then
    return 0
  end
  -- device_t::clock(): the configured input clock (the crystal feeding the
  -- CPU; the M6808 divides by four internally for its E clock).
  return maincpu.clock or 0
end

local function get_capabilities()
  return {
    system = system_name(),
    profile = profile_name(),
    systemDescription = system_description(),
    protocolVersion = 1,
    transport = "tcp",
    cpuClockHz = cpu_clock_hz(),
    commands = supported_commands,
    hardware = hardware_capabilities()
  }
end

local function read_memory(params)
  local addr = assert(params.addr, "read_mem requires addr")
  local len = assert(params.len, "read_mem requires len")
  local space = program_space()
  local bytes = {}

  for i = 0, len - 1 do
    bytes[#bytes + 1] = space:read_u8((addr + i) & 0xffff)
  end

  return { bytes = bytes }
end

local function write_memory(params)
  local addr = assert(params.addr, "write_mem requires addr")
  local bytes = assert(params.bytes, "write_mem requires bytes")
  local space = program_space()

  for i, byte in ipairs(bytes) do
    space:write_u8((addr + i - 1) & 0xffff, byte & 0xff)
  end

  return {}
end

function write_u8(addr, value)
  program_space():write_u8(addr & 0xffff, value & 0xff)
end

local function read_u8(addr)
  return program_space():read_u8(addr & 0xffff)
end

local function herojr_d842_snapshot(port_prefix, speech_prefix)
  -- 6821 datasheet: the IRQA1 flag (CRA bit 7) is set by an active CA1
  -- transition regardless of CRA bit 0 (that bit only gates the IRQ line),
  -- so the snapshot must not gate visibility on the interrupt-enable bit.
  -- Mirrors the driver's u215_speech_control_r.
  local output = manager.machine.output
  local control = output:get_indexed_value(port_prefix, 2) or 0
  local request = output:get_value("herojr_speech_request_flag") or 0
  return (control & 0x3f) | (request ~= 0 and 0x80 or 0)
end

local function herojr_d841_snapshot(port_prefix)
  -- $D841 (U215 port B) composed from driver outputs, never a bus read:
  -- on a 6821 a Peripheral Register B READ is the ONLY thing that clears
  -- the latched IRQB1 sonar-echo flag ($D843 bit 7) — the ROM leans on
  -- exactly that twice per measurement ($EFB6 stale discard, $EFD4
  -- blanking discard) — so an every-frame observer bus read would consume
  -- echoes the firmware is entitled to see (hero-jr-sonar-spec.md §3.2
  -- item 3, implemented 2026-07-03). Mirrors the driver's
  -- u215_speech_power_r peripheral-register composition:
  --   bits 0-3, 5, 7 = the CPU-driven port B output latch (port_out_1),
  --   bit 4 = serial ADC output bit (herojr_adc_output),
  --   bit 6 = Sleep/Norm switch (ioport read — side-effect-free, the same
  --           m_sleep_norm->read() the driver performs).
  local output = manager.machine.output
  local port_b = output:get_indexed_value(port_prefix, 1) or 0
  local adc_output = output:get_value("herojr_adc_output") or 0
  local sleep_norm = 1
  local port = manager.machine.ioport.ports[":SLEEP_NORM"] or manager.machine.ioport.ports["SLEEP_NORM"]
  if port then
    sleep_norm = port:read() & 0x01
  end
  return (port_b & 0xaf) | (adc_output ~= 0 and 0x10 or 0x00) | (sleep_norm ~= 0 and 0x40 or 0x00)
end

local function herojr_d821_snapshot()
  -- $D821 (U214 port B) composed from driver outputs, never a bus read:
  -- since the adc-encoder spec §7.1 remediation (2026-07-05) a Peripheral
  -- Register B READ is the ONLY thing that clears the latched IRQB1
  -- wheel-encoder flag ($D823 bit 7) — the ROM's own $D19E odometer
  -- service ends in exactly that read — so an every-frame observer bus
  -- read would consume pulses the firmware is entitled to count (the
  -- recompose the 2026-07-03 side-effect audit reserved for the day U214
  -- grew latched flags). Mirrors the driver's u214_port_b_r composition:
  --   bits 0-6 = the CPU-driven port B output latch (herojr_u214_port_b),
  --   bit 7   = motion detector level, active-low (herojr_motion_detector).
  local output = manager.machine.output
  local port_b = output:get_value("herojr_u214_port_b") or 0
  local motion = output:get_value("herojr_motion_detector") or 0
  return (port_b & 0x7f) | (motion ~= 0 and 0x00 or 0x80)
end

local function herojr_d823_snapshot()
  -- $D823 (U214 CRB) composed from driver outputs: low six bits = the
  -- written CRB latch (herojr_u214_control_b), bit 7 = the latched IRQB1
  -- wheel-encoder pulse flag (herojr_wheel_feedback, 0/1) — the same
  -- composition the driver's read handler returns. The read handler is
  -- side-effect-free since the §7.1 remediation (the old model advanced a
  -- wheel-feedback sample on every read); the composition is kept so the
  -- observer stays output-driven rather than re-auditing bus reads.
  local output = manager.machine.output
  local control_b = output:get_value("herojr_u214_control_b") or 0
  local feedback = output:get_value("herojr_wheel_feedback") or 0
  return (control_b & 0x3f) | ((feedback & 0x01) ~= 0 and 0x80 or 0x00)
end

local function reset_vector()
  return ((read_u8(0xfffe) << 8) | read_u8(0xffff)) & 0xffff
end

local function read_u16(addr)
  return ((read_u8(addr) << 8) | read_u8(addr + 1)) & 0xffff
end

local function stack_return_address()
  local sp = get_register("sp") & 0xffff
  return read_u16(sp + 1)
end

local function instruction_length(pc)
  local opcode = read_u8(pc)
  if (opcode >= 0x20 and opcode <= 0x2f) or opcode == 0x8d then
    return 2
  elseif opcode >= 0x60 and opcode <= 0x6f then
    return 2
  elseif opcode >= 0x70 and opcode <= 0x7f then
    return 3
  elseif opcode >= 0x80 and opcode <= 0x8f then
    return (opcode == 0x8c or opcode == 0x8e) and 3 or 2
  elseif opcode >= 0x90 and opcode <= 0xaf then
    return 2
  elseif opcode >= 0xb0 and opcode <= 0xbf then
    return 3
  elseif opcode >= 0xc0 and opcode <= 0xcf then
    return opcode == 0xce and 3 or 2
  elseif opcode >= 0xd0 and opcode <= 0xef then
    return 2
  elseif opcode >= 0xf0 and opcode <= 0xff then
    return 3
  end

  return 1
end

local function step_in_targets()
  local pc = get_register("pc") & 0xffff
  return { pc }
end

local function step_over_targets()
  local pc = get_register("pc") & 0xffff
  local opcode = read_u8(pc)
  if opcode == 0x8d or opcode == 0xad or opcode == 0xbd then
    return { (pc + instruction_length(pc)) & 0xffff }
  end

  return step_in_targets()
end

local function step_out_targets()
  return { stack_return_address() }
end

local function write_keypad_column(column)
  local addr = keypad_base + column - 1
  if addr < keypad_base or addr > keypad_end then
    error("keypad column " .. tostring(column) .. " is outside debug aperture")
  end

  write_u8(addr, keypad_columns[column])
end

local function sonar_count_byte_from_inches(value)
  local inches = tonumber(value) or 0
  if inches < 0 then
    inches = 0
  end

  -- HERO-1 BASIC converts the sampled $C220 byte back to inches at roughly
  -- (count - 14) / 2.5.  Keep the bridge value in the BASIC-visible scale.
  local count_byte = math.floor((inches * 2.5) + 14 + 0.5)
  if count_byte > 255 then
    return 255
  end
  return count_byte
end

local function get_io_state()
  local output = manager.machine.output
  local function output_value(name)
    local value = output:get_value(name)
    return value or 0
  end
  local function indexed_output_value(prefix, index)
    local value = output:get_indexed_value(prefix, index)
    return value or 0
  end
  local prefix = output_prefix()
  local led_prefix = prefix .. "_led_digit_"
  local port_prefix = prefix .. "_port_out_"
  local speech_prefix = prefix .. "_speech_"
  local motor_prefix = prefix .. "_motor_"
  local rs232_data = output_value(prefix .. "_rs232_data")
  local herojr_rs232_status = prefix == "herojr" and output_value(prefix .. "_rs232_status") or 0
  local digits = {}
  for i = 0, 5 do
    digits[#digits + 1] = indexed_output_value(led_prefix, i)
  end

  local steering_pivot_rotate = indexed_output_value(port_prefix, 3)
  local extend_head_gripper_shoulder = indexed_output_value(port_prefix, 4)
  local main_drive = indexed_output_value(port_prefix, 5)
  local arm_select_speech_clock = indexed_output_value(port_prefix, 6)
  local system_select = indexed_output_value(port_prefix, 7)
  local herojr_control = indexed_output_value(port_prefix, 1)
  -- Side-effect adjudication for the herojr io snapshot, which runs every
  -- frame while a client is connected (hero-jr-rtc-spec.md §3.3). Each
  -- address is bus-read ONLY if the driver's read handler is provably free
  -- of model side effects; otherwise it is composed from driver outputs:
  --   $D821 u214_port_b_r    — SIDE-EFFECTFUL since the adc-encoder spec
  --         §7.1 remediation (2026-07-05): a Peripheral Register B read
  --         clears the latched IRQB1 wheel-encoder flag, per 6821 rule —
  --         composed, herojr_d821_snapshot (the recompose the earlier
  --         audit reserved for the day U214 grew latched flags).
  --   $D822 u214_control_a_r — pure (CRA | latched IRQA1); 6821 control-
  --         register reads never clear flags — safe bus read.
  --   $D823 u214_control_b_r — pure since the §7.1 remediation (CRB |
  --         latched IRQB1; the old read-advancing sample counter is
  --         retired): still composed, herojr_d823_snapshot, to keep the
  --         observer output-driven.
  --   $D841 u215_speech_power_r — SIDE-EFFECTFUL (clears the latched
  --         sonar-echo flag per 6821 IRQB1 clear-on-read, sonar spec §3.2
  --         item 3): composed, herojr_d841_snapshot.
  --   $D843 u215_sonar_echo_r — pure (CRB latch | echo flag); a CRB read
  --         does NOT clear IRQB1 (6821 datasheet, sonar spec §3.2 item 3
  --         adjudication) — safe bus read.
  local herojr_d821 = prefix == "herojr" and herojr_d821_snapshot() or 0
  local herojr_d822 = prefix == "herojr" and read_u8(0xd822) or 0
  local herojr_d823 = prefix == "herojr" and herojr_d823_snapshot() or 0
  local herojr_d841 = prefix == "herojr" and herojr_d841_snapshot(port_prefix) or 0
  local herojr_d842 = prefix == "herojr" and herojr_d842_snapshot(port_prefix, speech_prefix) or 0
  local herojr_d843 = prefix == "herojr" and read_u8(0xd843) or 0
  local herojr_motion_detector = output_value(prefix .. "_motion_detector")
  local herojr_wheel_feedback = output_value(prefix .. "_wheel_feedback")
  local herojr_drive_activity = output_value(prefix .. "_drive_activity")
  local herojr_rtc_sqw = output_value(prefix .. "_rtc_sqw")
  local herojr_adc_sample = output_value(prefix .. "_adc_sample")
  local herojr_adc_output = output_value(prefix .. "_adc_output")
  local herojr_sonar_echo = output_value(prefix .. "_sonar_echo")
  local herojr_sonar_distance = output_value(prefix .. "_sonar_distance")
  local herojr_sonar_init_time_us = output_value(prefix .. "_sonar_init_time_us")
  local herojr_sonar_echo_time_us = output_value(prefix .. "_sonar_echo_time_us")
  -- G2J-08 power model outputs: green LED = modeled Vcc (the U222 sleep
  -- latch drives +5 SHUTDOWN; LED off = latched asleep). The µs stamps and
  -- wake-cycle counter resolve catnap pulses shorter than the snapshot
  -- cadence — same pattern as the sonar INIT/echo stamps.
  local herojr_power_led = output_value(prefix .. "_power_led")
  local herojr_power_on_time_us = output_value(prefix .. "_power_on_time_us")
  local herojr_power_off_time_us = output_value(prefix .. "_power_off_time_us")
  local herojr_power_cycles = output_value(prefix .. "_power_cycles")
  local experimental_output = prefix == "hero1" and indexed_output_value(port_prefix, 1) or 0
  local experimental_input = prefix == "hero1" and read_u8(0xc2a0) or 0
  local experimental_interrupt_status = prefix == "hero1" and read_u8(0xc200) or 0
  local experimental_irq_vector = prefix == "hero1" and { read_u8(0x002d), read_u8(0x002e), read_u8(0x002f) } or { 0, 0, 0 }
  local speech_power = output_value(speech_prefix .. "power")
  local eye_ear_select = (system_select >> 7) & 0x01
  local main_power = (system_select >> 6) & 0x01
  local sense_power = (system_select >> 5) & 0x01
  local display_power = (system_select >> 4) & 0x01
  local motion_power = (system_select >> 2) & 0x01
  local sonar_power = (system_select >> 1) & 0x01
  local tape_out = system_select & 0x01
  if prefix == "hero1" then
    speech_power = (system_select >> 3) & 0x01
  elseif prefix == "herojr" then
    system_select = herojr_control
    eye_ear_select = 0
    main_power = 0
    sense_power = (herojr_control >> 5) & 0x01
    display_power = 0
    motion_power = 0
    sonar_power = (herojr_control >> 5) & 0x01
    tape_out = 0
  end

  local pressed = {}
  for key, is_pressed in pairs(keypad_state) do
    if is_pressed then
      pressed[#pressed + 1] = key
    end
  end

  local dataLeds = indexed_output_value(port_prefix, 0)

  local function build_common_io_state()
    return {
      display = {
        digits = digits
      },
      sensors = {
        sonarDistanceInches = sensor_state.sonarDistanceInches,
        lightLevel = sensor_state.lightLevel,
        soundLevel = sensor_state.soundLevel,
        motionDetected = prefix == "herojr" and herojr_motion_detector ~= 0 or sensor_state.motionDetected
      },
      speech = {
        phoneme = output_value(speech_prefix .. "phoneme"),
        inflection = output_value(speech_prefix .. "inflection"),
        strobe = output_value(speech_prefix .. "strobe"),
        power = speech_power,
        ready = output_value(speech_prefix .. "ready")
      }
    }
  end

  local function build_hero1_io_state()
    return {
      keypad = pressed,
      pendant = sensor_state.pendant,
      motors = {
        -- Modeled mechanical positions in microsteps (driver outputs
        -- hero1_axis_position_0..6; order = ROM position bytes
        -- $0000-$0006: extend, shoulder, rotate, pivot, gripper, head,
        -- steering — hero-1-motion-spec.md §2.2/§4 P5).
        axisPositions = {
          indexed_output_value(prefix .. "_axis_position_", 0),
          indexed_output_value(prefix .. "_axis_position_", 1),
          indexed_output_value(prefix .. "_axis_position_", 2),
          indexed_output_value(prefix .. "_axis_position_", 3),
          indexed_output_value(prefix .. "_axis_position_", 4),
          indexed_output_value(prefix .. "_axis_position_", 5),
          indexed_output_value(prefix .. "_axis_position_", 6)
        },
        wheelPulses = output_value(prefix .. "_wheel_pulses"),
        steeringPivotRotate = steering_pivot_rotate,
        steeringPattern = (steering_pivot_rotate >> 4) & 0x0f,
        pivotRotatePattern = steering_pivot_rotate & 0x0f,
        extendHeadGripperShoulder = extend_head_gripper_shoulder,
        extendHeadPattern = (extend_head_gripper_shoulder >> 4) & 0x0f,
        gripperShoulderPattern = extend_head_gripper_shoulder & 0x0f,
        mainDrive = main_drive,
        mainDriveSpeed = main_drive & 0x3f,
        mainDriveEnabled = (main_drive >> 6) & 0x01,
        mainDriveDirection = (main_drive >> 7) & 0x01,
        armSelectSpeechClock = arm_select_speech_clock,
        armSelectRotateShoulderHead = (arm_select_speech_clock >> 7) & 0x01,
        armSelectPivotGripperExtend = (arm_select_speech_clock >> 6) & 0x01,
        clockAddress = arm_select_speech_clock & 0x0f
      },
      ports = {
        systemSelect = system_select,
        eyeEarSelect = eye_ear_select,
        mainPower = main_power,
        sensePower = sense_power,
        displayPower = display_power,
        speechPower = speech_power,
        motionPower = motion_power,
        sonarPower = sonar_power,
        tapeIn = sensor_state.tapeIn and 1 or 0,
        tapeOut = tape_out
      },
      experimental = {
        outputPort = experimental_output,
        outputAddress = 0xc220,
        inputPort = experimental_input,
        inputAddress = 0xc2a0,
        inputMode = "floating/unmodeled",
        interruptStatus = experimental_interrupt_status,
        irqMask = 0x80,
        irqPending = (experimental_interrupt_status >> 7) & 0x01,
        irqVectorAddress = 0x002d,
        irqVector = experimental_irq_vector
      }
    }
  end

  local function build_herojr_io_state()
    return {
      keypad = pressed,
      ports = {
        d820 = herojr_keypad_byte(),
        d821 = herojr_d821,
        d822 = herojr_d822,
        d823 = herojr_d823,
        d840 = dataLeds,
        d841 = herojr_d841,
        d842 = herojr_d842,
        d843 = herojr_d843,
        d880 = herojr_rs232_status,
        d881 = rs232_data
      },
      drive = {
        left = output_value(motor_prefix .. "left"),
        right = output_value(motor_prefix .. "right"),
        steeringPivotRotate = steering_pivot_rotate,
        steeringPattern = (steering_pivot_rotate >> 4) & 0x0f,
        pivotRotatePattern = steering_pivot_rotate & 0x0f,
        mainDrive = main_drive,
        mainDriveSpeed = main_drive & 0x3f,
        mainDriveEnabled = (main_drive >> 6) & 0x01,
        mainDriveDirection = (main_drive >> 7) & 0x01,
        activityCount = herojr_drive_activity,
        wheelFeedback = herojr_wheel_feedback,
        wheelFeedbackStatus = (herojr_d823 >> 7) & 0x01
      },
      leds = {
        data = dataLeds
      },
      rtc = {
        startAddress = 0xd81e,
        endAddress = 0xd81f,
        sqw = herojr_rtc_sqw,
        sqwStatus = (herojr_d822 >> 7) & 0x01
      },
      -- G2J-08 sleep/wake power model (hero-jr-rtc-spec.md §2.7/§2.8;
      -- AUDIT-2026-06.md U222 wire-walk). greenLed is the modeled Vcc: the
      -- green POWER LED is a power indicator, not a port (JR-OG p. 38 —
      -- "The green LED will flash once every five seconds while it is
      -- sleeping" — that flash is firmware catnap physics, never scripted).
      power = {
        greenLed = herojr_power_led,
        vcc = herojr_power_led,
        onTimeUs = herojr_power_on_time_us,
        offTimeUs = herojr_power_off_time_us,
        wakeCycles = herojr_power_cycles
      },
      rs232 = {
        status = herojr_rs232_status,
        txData = rs232_data,
        txReady = (herojr_rs232_status >> 1) & 0x01,
        rxReady = herojr_rs232_status & 0x01
      },
      cartridge = {
        present = false,
        startAddress = 0x6000,
        endAddress = 0x7fff,
        size = 0x2000,
        slot = "cartslot",
        banking = "unsupported"
      },
      sense = {
        control = herojr_control,
        sensePower = sense_power,
        sleepNorm = (herojr_d841 >> 6) & 0x01,
        soundLightSelect = (herojr_control >> 2) & 0x01,
        adcChipSelect = (herojr_control >> 3) & 0x01,
        adcOutput = herojr_adc_output,
        adcOutputStatus = (herojr_control >> 4) & 0x01,
        adcSample = herojr_adc_sample,
        adcClock = (herojr_control >> 5) & 0x01,
        sonarPower = sonar_power,
        sonarDistanceInches = herojr_sonar_distance,
        sonarEcho = herojr_sonar_echo,
        sonarEchoStatus = (herojr_d843 >> 7) & 0x01,
        sonarInit = (herojr_d823 >> 3) & 0x01,
        -- Driver-model emulated-time telemetry (µs): when INIT scheduled the
        -- echo and when the echo asserted. Observability only — G1J-06's
        -- sub-bridge-latency delays cannot be resolved from the host side.
        sonarInitTimeUs = herojr_sonar_init_time_us,
        sonarEchoTimeUs = herojr_sonar_echo_time_us,
        motionDetected = herojr_motion_detector,
        motionStatus = (herojr_d821 >> 7) & 0x01
      }
    }
  end

  local state = {
    protocolVersion = 1,
    system = system_name(),
    profile = profile_name(),
    common = build_common_io_state()
  }

  if prefix == "hero1" then
    state.hero1 = build_hero1_io_state()
  end
  if prefix == "herojr" then
    state.herojr = build_herojr_io_state()
  end

  return state
end

local function broadcast_io_changed(force)
  if #clients == 0 then
    last_io_state_encoded = nil
    last_io_broadcast_time = 0
    return
  end
  if not program_space_available() then
    return
  end

  local state = get_io_state()
  local encoded = encode_json(state)
  if encoded ~= last_io_state_encoded then
    local now = emu.time()
    if not force and now < last_io_broadcast_time + 0.05 then
      return
    end
    last_io_state_encoded = encoded
    last_io_broadcast_time = now
    broadcast_event("io_changed", state)
  end
end

-- Dedicated byte-exact phoneme stream (Phase 2.2d, G1J-07/G2J-09/G2J-01;
-- the hero1 driver publishes the same `<prefix>_phoneme_*` contract since
-- Phase 4.3): the driver publishes a latch sequence counter, the latched
-- byte, its emulated-time stamp, and a cumulative clip counter
-- (latch-while-busy).
-- io_changed's whole-state dedup and 50 ms rate limit cannot be byte-exact,
-- so every sequence advance is emitted as its own `phoneme` event. The
-- shortest SC-01 phoneme (47 ms per the Programmer's Guide chart) outlasts
-- an emulated frame, so the per-frame poll observes every latch; if latches
-- ever outpace the poll the gap is reported loudly in `missed`, never
-- silently absorbed. The counter tracks even with no client attached so a
-- mid-utterance connect starts clean instead of replaying a backlog burst.
local last_phoneme_seq = nil

local function broadcast_phoneme_events()
  if not program_space_available() then
    return
  end
  local output = manager.machine.output
  local prefix = output_prefix()
  local seq = output:get_value(prefix .. "_phoneme_seq") or 0
  if last_phoneme_seq == nil then
    last_phoneme_seq = seq
    return
  end
  if seq < last_phoneme_seq then
    -- Counter restarted (machine reset zeroes the telemetry): latches
    -- 1..seq happened after the restart.
    last_phoneme_seq = 0
  end
  if seq == last_phoneme_seq then
    return
  end
  local missed = seq - last_phoneme_seq - 1
  last_phoneme_seq = seq
  if #clients == 0 or seq == 0 then
    return
  end
  local byte = (output:get_value(prefix .. "_phoneme_byte") or 0) & 0xff
  broadcast_event("phoneme", {
    seq = seq,
    byte = byte,
    phoneme = byte & 0x3f,
    inflection = (byte >> 6) & 0x03,
    timeUs = output:get_value(prefix .. "_phoneme_time_us") or 0,
    clips = output:get_value(prefix .. "_phoneme_clips") or 0,
    missed = missed
  })
end

local function set_registers(params)
  for name, _ in pairs(register_names) do
    if type(params[name]) == "number" then
      set_register(name, params[name])
    end
  end
  if type(params.pc) == "number" then
    set_register("wai", 0)
  end
  return {}
end

local function set_breakpoint(params)
  local addr = assert(params.addr, "set_breakpoint requires addr")
  addr = addr & 0xffff
  local state = manager.machine.debugger and manager.machine.debugger.execution_state or "unknown"
  trace("set_breakpoint request addr=" .. trace_address(addr) .. " pc=" .. trace_address(get_register("pc")) .. " state=" .. tostring(state))
  if not breakpoints_by_addr[addr] then
    local index = cpu_debug():bpset(addr, "", "")
    breakpoints_by_addr[addr] = index
    breakpoints_by_index[index] = addr
    trace("set_breakpoint applied addr=" .. trace_address(addr) .. " index=" .. tostring(index))
  else
    trace("set_breakpoint reused addr=" .. trace_address(addr) .. " index=" .. tostring(breakpoints_by_addr[addr]))
  end
  return { breakpoint = breakpoints_by_addr[addr] }
end

local function clear_breakpoint(params)
  local addr = assert(params.addr, "clear_breakpoint requires addr")
  addr = addr & 0xffff
  local index = breakpoints_by_addr[addr]
  trace("clear_breakpoint request addr=" .. trace_address(addr) .. " index=" .. tostring(index))
  if index then
    cpu_debug():bpclear(index)
    breakpoints_by_addr[addr] = nil
    breakpoints_by_index[index] = nil
    trace("clear_breakpoint applied addr=" .. trace_address(addr) .. " index=" .. tostring(index))
  end
  return {}
end

local function set_read_watchpoint(params)
  local addr = assert(params.addr, "set_read_watchpoint requires addr")
  addr = addr & 0xffff
  local one_shot = params.oneShot ~= false
  local state = manager.machine.debugger and manager.machine.debugger.execution_state or "unknown"
  trace("set_read_watchpoint request addr=" .. trace_address(addr) .. " one_shot=" .. tostring(one_shot) .. " pc=" .. trace_address(get_register("pc")) .. " state=" .. tostring(state))
  ensure_debugger_stopped_for_watchpoint("set_read_watchpoint")
  if not read_watchpoints_by_addr[addr] then
    local index = cpu_debug():wpset(program_space(), "r", addr, 1, "", "")
    read_watchpoints_by_addr[addr] = index
    read_watchpoints_by_index[index] = addr
    trace("set_read_watchpoint applied addr=" .. trace_address(addr) .. " index=" .. tostring(index))
  else
    trace("set_read_watchpoint reused addr=" .. trace_address(addr) .. " index=" .. tostring(read_watchpoints_by_addr[addr]))
  end

  local index = read_watchpoints_by_addr[addr]
  read_watchpoint_one_shot_by_index[index] = one_shot
  return { watchpoint = index }
end

local function clear_read_watchpoint_by_index(index)
  local addr = read_watchpoints_by_index[index]
  if addr then
    pcall(function()
      cpu_debug():wpclear(index)
    end)
    read_watchpoints_by_index[index] = nil
    read_watchpoints_by_addr[addr] = nil
    read_watchpoint_one_shot_by_index[index] = nil
    trace("clear_read_watchpoint applied addr=" .. trace_address(addr) .. " index=" .. tostring(index))
  end
end

local function clear_read_watchpoint(params)
  local addr = assert(params.addr, "clear_read_watchpoint requires addr")
  addr = addr & 0xffff
  local index = read_watchpoints_by_addr[addr]
  trace("clear_read_watchpoint request addr=" .. trace_address(addr) .. " index=" .. tostring(index))
  if index then
    ensure_debugger_stopped_for_watchpoint("clear_read_watchpoint")
    clear_read_watchpoint_by_index(index)
  end
  return {}
end

local function clear_temp_step_breakpoints()
  for index, _ in pairs(temp_step_breakpoints) do
    pcall(function()
      cpu_debug():bpclear(index)
    end)
  end
  temp_step_breakpoints = {}
end

local function run_to_step_targets(targets)
  clear_temp_step_breakpoints()
  pending_step_reason = "step"
  pending_step_start_pc = get_register("pc")
  pending_step_seen_run = false

  local unique = {}
  for _, target in ipairs(targets) do
    local addr = target & 0xffff
    if not unique[addr] then
      unique[addr] = true
    end
  end

  for addr, _ in pairs(unique) do
    local index = cpu_debug():bpset(addr, "", "")
    temp_step_breakpoints[index] = addr
  end
  trace("step targets reason=" .. tostring(pending_step_reason) .. " start_pc=" .. trace_address(pending_step_start_pc or 0) .. " targets=" .. trace_address_list(unique))
  emu.unpause()
  cpu_debug():go()
end

local function set_sensor(params)
  local name = assert(params.name, "set_sensor requires name")
  sensor_state[name] = params.value
  if name == "sonarDistanceInches" then
    if system_name() == "hero1" then
      -- Aperture only. The $D100 write latches the counter byte and raises
      -- the ROM-visible sonar interrupt; the firmware ISR alone produces
      -- the readings RAM state ($0010 hit count, $0011 reading) — the
      -- bridge never writes those addresses (G1H-06).
      write_u8(sensor_base + 0x00, sonar_count_byte_from_inches(params.value))
    elseif system_name() == "herojr" then
      write_u8(herojr_sensor_base + 0x02, tonumber(params.value) or 0)
    end
  elseif name == "lightLevel" then
    if system_name() == "herojr" then
      write_u8(herojr_sensor_base + 0x00, tonumber(params.value) or 0)
    else
      write_u8(sensor_base + 0x01, tonumber(params.value) or 0)
    end
  elseif name == "soundLevel" then
    if system_name() == "herojr" then
      write_u8(herojr_sensor_base + 0x01, tonumber(params.value) or 0)
    else
      write_u8(sensor_base + 0x02, tonumber(params.value) or 0)
    end
  elseif name == "motionDetected" then
    if system_name() == "herojr" then
      if params.value then
        write_u8(herojr_sensor_base + 0x03, 1)
      else
        write_u8(herojr_sensor_base + 0x03, 0)
      end
    else
      write_u8(sensor_base + 0x03, params.value and 1 or 0)
    end
  elseif name == "pendantPort" then
    write_pendant_port(params.value)
  elseif name == "pendantFunction" then
    sensor_state.pendant["function"] = tostring(params.value)
    write_pendant_port(pendant_port_from_state())
  elseif name == "pendantJoint" then
    sensor_state.pendant.joint = tostring(params.value)
    sensor_state.pendant.rotary = sensor_state.pendant.joint
    write_pendant_port(pendant_port_from_state())
  elseif name == "pendantRotary" then
    sensor_state.pendant.rotary = tostring(params.value)
    sensor_state.pendant.joint = sensor_state.pendant.rotary
    write_pendant_port(pendant_port_from_state())
  elseif name == "pendantMotion" then
    sensor_state.pendant.motion = tostring(params.value)
    sensor_state.pendant.leftPressed = sensor_state.pendant.motion == "LEFT"
    sensor_state.pendant.rightPressed = sensor_state.pendant.motion == "RIGHT"
    write_pendant_port(pendant_port_from_state())
  elseif name == "pendantLeftPressed" then
    sensor_state.pendant.leftPressed = params.value and true or false
    write_pendant_port(pendant_port_from_state())
  elseif name == "pendantRightPressed" then
    sensor_state.pendant.rightPressed = params.value and true or false
    write_pendant_port(pendant_port_from_state())
  elseif name == "pendantTrigger" then
    sensor_state.pendant.triggerPressed = params.value and true or false
    sensor_state.pendant.trigger = sensor_state.pendant.triggerPressed
    write_pendant_port(pendant_port_from_state())
  elseif name == "tapeIn" then
    sensor_state.tapeIn = params.value and true or false
    write_u8(sensor_base + 0x0b, sensor_state.tapeIn and 1 or 0)
  end
  broadcast_io_changed(true)
  return {}
end

local function press_key(params)
  local key = assert(params.key, "press_key requires key")
  keypad_state[key] = true
  local mapping = system_name() == "hero1" and key_map[key] or nil
  if mapping then
    keypad_columns[mapping.column] = keypad_columns[mapping.column] & (~mapping.bit & 0x3f)
    write_keypad_column(mapping.column)
  end
  if system_name() == "herojr" then
    -- Ioport fields only (G1J-03): the same input path a human keypress
    -- takes. The driver's matrix read derives held keys from these fields;
    -- the old $D820 write-sniffing side channel is deleted.
    if key == "RESET" then
      set_herojr_reset_field(true)
    else
      set_herojr_keypad_field(key, true)
    end
  end
  broadcast_io_changed(true)
  return {}
end

local function release_key(params)
  local key = assert(params.key, "release_key requires key")
  keypad_state[key] = nil
  local mapping = system_name() == "hero1" and key_map[key] or nil
  if mapping then
    keypad_columns[mapping.column] = (keypad_columns[mapping.column] | mapping.bit) & 0x3f
    write_keypad_column(mapping.column)
  end
  if system_name() == "herojr" then
    if key == "RESET" then
      set_herojr_reset_field(false)
    else
      set_herojr_keypad_field(key, false)
    end
  end
  broadcast_io_changed(true)
  return {}
end

local function set_sleep_norm(params)
  local norm = true
  if params and params.norm ~= nil then
    norm = params.norm and true or false
  elseif params and params.sleep ~= nil then
    norm = not params.sleep
  end

  if not set_herojr_sleep_norm_field(norm) then
    error("set_sleep_norm requires HERO Jr SLEEP_NORM input")
  end

  broadcast_io_changed(true)
  return { sleepNorm = norm and 1 or 0 }
end

local function snapshot_address_map(addresses)
  local values = {}
  for addr, _ in pairs(addresses) do
    values[#values + 1] = trace_address(addr)
  end
  table.sort(values)
  return values
end

local function bridge_debug_snapshot()
  local snapshot = {
    system = system_name(),
    profile = profile_name(),
    pc = trace_address(get_register("pc")),
    debugger_state = manager.machine.debugger and manager.machine.debugger.execution_state or "unknown",
    active_breakpoints = snapshot_address_map(breakpoints_by_addr),
    active_read_watchpoints = snapshot_address_map(read_watchpoints_by_addr),
    last_stop_pc = last_stop_pc and trace_address(last_stop_pc) or nil
  }

  local registers_ok, registers = pcall(get_registers)
  if registers_ok then
    snapshot.registers = registers
  else
    snapshot.registers = { error = tostring(registers) }
  end

  local io_ok, io = pcall(get_io_state)
  if io_ok then
    snapshot.io = io
  else
    snapshot.io = { error = tostring(io) }
  end

  return snapshot
end

-- RTC value encoding (hero-jr-rtc-spec.md §2.2, byte-verified ROM truth):
-- the v1.6 ROM programs register $0B with DM = 1 (BINARY, not BCD) and
-- 24/12 = 0 (12-hour mode: hours hold 1-12 with bit 7 as PM). Every
-- time/calendar byte on the chip — and every ROM RAM copy of one — uses
-- that encoding (spec §2.6; G2J-07's committed readbacks corroborate it).
local function rtc_hours_12h_binary(hour24)
  local hour = (tonumber(hour24) or 0) % 24
  local pm = 0x00
  if hour >= 12 then
    pm = 0x80
    hour = hour - 12
  end
  if hour == 0 then
    hour = 12
  end
  return (pm | hour) & 0xff
end

local function write_rtc_register(register, value)
  write_u8(0xd810, register & 0xff)
  write_u8(0xd811, value & 0xff)
end

local function read_rtc_register(register)
  write_u8(0xd810, register & 0xff)
  return read_u8(0xd811)
end

local function parse_warm_context_time(params)
  if params and params.timestamp then
    local y, mo, d, h, mi, s = string.match(
      params.timestamp,
      "^(%d%d%d%d)%-(%d%d)%-(%d%d)T(%d%d):(%d%d):(%d%d)"
    )
    if not y then
      error("initialize_herojr_warm_context timestamp must be YYYY-MM-DDTHH:MM:SS")
    end
    return {
      year = tonumber(y),
      month = tonumber(mo),
      day = tonumber(d),
      hour = tonumber(h),
      minute = tonumber(mi),
      second = tonumber(s)
    }
  end

  return os.date("*t")
end

local function initialize_herojr_retained_ram_context(t)
  -- ROM $800F initializes the personality/scheduler workspace on the cold
  -- path. A real Sleep/Norm warm start preserves this RAM while taking $A070.
  local retained_defaults = {
    { 0x02fa, 0x02 },
    { 0x02fb, 0x04 },
    { 0x02fc, 0x02 },
    { 0x02fd, 0x02 },
    { 0x02fe, 0x02 },
    { 0x02ff, 0x00 },
    { 0x0300, 0x08 },
    { 0x0301, 0x06 },
    { 0x0302, 0x01 }
  }
  for _, entry in ipairs(retained_defaults) do
    write_u8(entry[1], entry[2])
  end

  write_u8(0x02f4, 0x00)
  write_u8(0x02f5, 0x00)
  for i = 0, 15 do
    write_u8(0x00c4 + (i * 5), 0x00)
  end

  -- Raw RTC register bytes in the chip's own encoding — binary, 12-hour
  -- (hero-jr-rtc-spec.md §2.2/§2.6). Order matches the ROM's 7-byte time
  -- buffers: sec, min, hr, dow, date, month, year (spec §2.1).
  local rtc_snapshot = {
    (t.second or 0) % 60,
    (t.minute or 0) % 60,
    rtc_hours_12h_binary(t.hour or 0),
    (t.wday or 1) & 0xff,
    (t.day or 1) & 0xff,
    (t.month or 1) & 0xff,
    (t.year or 1984) % 100
  }

  -- ROM RTC shadow $067F-$068A (hero-jr-rtc-spec.md §2.1, the $EEC6/$EF0F
  -- layout, index = register number): clock and alarm registers interleave
  -- through day-of-week, then date/month/year follow at STRIDE 1. The old
  -- stride-2-throughout seeding landed date/month/year at $0687/$0689/$068B
  -- instead of $0686/$0687/$0688 (spec §3.5 defect, corrected 2026-07-03).
  -- Alarm slots mirror the chip's alarm registers (no armed wake, $00 —
  -- see initialize_herojr_warm_context).
  local rtc_shadow = {
    { 0x067f, rtc_snapshot[1] }, -- sec        (reg $00)
    { 0x0680, 0x00 },            -- sec-alarm  (reg $01)
    { 0x0681, rtc_snapshot[2] }, -- min        (reg $02)
    { 0x0682, 0x00 },            -- min-alarm  (reg $03)
    { 0x0683, rtc_snapshot[3] }, -- hr         (reg $04)
    { 0x0684, 0x00 },            -- hr-alarm   (reg $05)
    { 0x0685, rtc_snapshot[4] }, -- dow        (reg $06)
    { 0x0686, rtc_snapshot[5] }, -- date       (reg $07)
    { 0x0687, rtc_snapshot[6] }, -- month      (reg $08)
    { 0x0688, rtc_snapshot[7] }  -- year       (reg $09)
  }
  for _, entry in ipairs(rtc_shadow) do
    write_u8(entry[1], entry[2])
  end

  for i, value in ipairs(rtc_snapshot) do
    write_u8(0x0092 + i, value)
  end

  write_u8(0x009b, rtc_snapshot[3])
  write_u8(0x009c, rtc_snapshot[2])
  write_u8(0x009d, rtc_snapshot[5])
  write_u8(0x009e, rtc_snapshot[6])
  write_u8(0x009f, rtc_snapshot[7])

  write_u8(0x03be, 0xa7)
  write_u8(0x03bf, 0x1c)
  write_u8(0x03c0, 0xa7)
  write_u8(0x03c1, 0x2b)
  write_u8(0x0080, 0x39)

  -- ROM $A05F-$A06D clears startup flags and seeds the retained plan root.
  write_u8(0x0085, 0x00)
  write_u8(0x0086, 0x00)
  write_u8(0x0088, 0x00)
  write_u8(0x0677, 0x00)
  write_u8(0x0678, 0x01)
end

local function initialize_herojr_warm_context(params)
  if system_name() ~= "herojr" then
    error("initialize_herojr_warm_context is HERO Jr only")
  end

  local t = parse_warm_context_time(params or {})
  local dst = params and params.dst
  if dst == nil then
    dst = t.isdst == true
  end

  -- Real MC146818 register map (hero-jr-rtc-spec.md §2.1 — the ROM's own
  -- $EE07/$EF0F set-time path writes the full interleaved file $00-$09:
  -- $00 sec, $01 sec-alarm, $02 min, $03 min-alarm, $04 hr, $05 hr-alarm,
  -- $06 dow, $07 date, $08 month, $09 year). Values are BINARY with
  -- 12-hour PM-bit hours per the ROM's DM = 1 / 24-12 = 0 programming
  -- (spec §2.2). The old fixture wrote a compacted BCD $00-$06 map that
  -- skewed the alarm registers and never wrote date/month/year — the spec
  -- §3.5 defect, corrected 2026-07-03.
  --
  -- Alarm registers hold $00 = no armed wake: the warm contract boots with
  -- AF clear and AIE off, the awake ROM never consumes the alarm (spec
  -- §2.8), and in 12-hour mode the hour register never reads $00 (1-12
  -- with PM bit), so a $00 hr-alarm can never spuriously match and set AF.
  write_rtc_register(0x00, (t.second or 0) % 60)
  write_rtc_register(0x01, 0x00)
  write_rtc_register(0x02, (t.minute or 0) % 60)
  write_rtc_register(0x03, 0x00)
  write_rtc_register(0x04, rtc_hours_12h_binary(t.hour or 0))
  write_rtc_register(0x05, 0x00)
  write_rtc_register(0x06, (t.wday or 1) & 0xff)
  write_rtc_register(0x07, (t.day or 1) & 0xff)
  write_rtc_register(0x08, (t.month or 1) & 0xff)
  write_rtc_register(0x09, (t.year or 1984) % 100)

  -- Register $0B: the Vca-retained control byte exactly as RESET leaves it
  -- on a real warm boot — RESET clears SQWE and the interrupt enables but
  -- keeps DM/24-12 (spec §1.4): DM = 1 binary, 12-hour, SET/PIE/AIE/UIE/
  -- SQWE = 0, DSE per request. Writing the mode bits keeps the chip's
  -- update arithmetic consistent with the binary payload above; the ROM's
  -- boot re-init preserves only DSE anyway ($FFA6, spec §2.2).
  write_rtc_register(0x0b, 0x04 | (dst and 0x01 or 0x00))

  -- Register $0C is read-only (spec §3.4/§3.5 — device writes to REG_C are
  -- ignored); the READ is what clears stale AF/PF, the same stale-flag
  -- consumption the ROM's own alarm-set path performs (spec §2.1 $EE7D).
  -- The warm contract requires AF clear (G2J-04 M1), so read and discard.
  read_rtc_register(0x0c)

  -- Register $0D is read-only too; READING it sets VRT — the ROM's own
  -- cookie-arm protocol does exactly this read before writing $0E
  -- (spec §2.5, $ED9B write-mode). The old $0D write-back was a no-op.
  read_rtc_register(0x0d)
  write_rtc_register(0x0e, 0xff)
  initialize_herojr_retained_ram_context(t)

  return {
    rtcWarmFlag = 0xff,
    retainedRam = {
      defaultStateInitializer = "0x800F",
      startupFlagInitializer = "0xA05F-0xA06D",
      functionReturnStub = 0x39,
      robotNamePointer = 0xa71c,
      masterNamePointer = 0xa72b,
      planRoot = 0x0001
    },
    registers = {
      ["0x0bBit0"] = dst and 1 or 0,
      ["0x0cAf"] = 0,
      ["0x0dVrt"] = 1,
      ["0x0eCookie"] = 0xff
    }
  }
end

local function reset_machine(params)
  local preserve_herojr_keys = system_name() == "herojr" and params and params.preserveKeys == true
  -- Arm the deferred-acknowledgement state BEFORE the reset is initiated so
  -- no completion signal can be missed (see the pending_reset_ack note).
  machine_reset_baseline = machine_reset_count
  herojr_reset_respond_at = nil
  reset_ack_stop_seen = false
  clear_temp_step_breakpoints()
  if not preserve_herojr_keys then
    keypad_state = {}
    keypad_columns = { 0x3f, 0x3f, 0x3f }
    if system_name() == "hero1" then
      write_keypad_column(1)
      write_keypad_column(2)
      write_keypad_column(3)
    end
    if system_name() == "herojr" then
      -- Ioport fields only; held fields naturally survive a machine reset,
      -- which is exactly what preserveKeys relies on.
      for key, _ in pairs(key_map) do
        set_herojr_keypad_field(key, false)
      end
      for key = 0, 15 do
        set_herojr_keypad_field(string.format("%X", key), false)
      end
    end
  end
  if system_name() == "herojr" then
    sensor_state.motionDetected = false
    set_herojr_reset_field(false)
    if not set_herojr_reset_field(true) then
      error("reset_machine requires HERO Jr RESET input")
    end
    write_u8(herojr_sensor_base + 0x03, 0)
    -- Hold the modeled panel button for 40 ms of EMULATED time (>= two
    -- ioport frame_update samples at 60 Hz; a human press holds far
    -- longer), then release in the pump. Until the release, the machine
    -- may still execute up to a frame of the DYING boot before the
    -- asserted line is sampled; debugger points are disabled across that
    -- window so no stale-boot stop leaks into the post-reset event stream
    -- (and a tight stale-boot breakpoint loop cannot stall the engage).
    -- They are re-enabled at the release write, which the suspended CPU
    -- cannot outrun: boot code only executes after the release edge is
    -- sampled one frame later. Deterministic both ways in
    -- conformance/tools/reproHerojrResetLoss.js (stopped-reset clause).
    cpu_debug():bpdisable()
    cpu_debug():wpdisable()
    herojr_reset_release_at = emulated_time_seconds() + 0.040
  else
    manager.machine:soft_reset()
  end
  emu.unpause()
  cpu_debug():go()
  broadcast_io_changed(true)
  -- The pump answers this request once the reset has observably completed;
  -- an immediate response would let the client pause the machine with the
  -- reset still pending (the R2 dead-install mechanism).
  return DEFER_RESPONSE
end

local handlers = {
  get_capabilities = function() return get_capabilities() end,
  get_registers = function() return get_registers() end,
  set_registers = set_registers,
  read_mem = read_memory,
  write_mem = write_memory,
  set_breakpoint = set_breakpoint,
  clear_breakpoint = clear_breakpoint,
  set_read_watchpoint = set_read_watchpoint,
  clear_read_watchpoint = clear_read_watchpoint,
  continue = function()
    trace("continue request pc=" .. trace_address(get_register("pc")))
    clear_temp_step_breakpoints()
    pending_step_reason = nil
    pending_step_start_pc = nil
    pending_step_seen_run = false
    emu.unpause()
    cpu_debug():go()
    return {}
  end,
  pause = function()
    trace("pause request pc=" .. trace_address(get_register("pc")))
    clear_temp_step_breakpoints()
    pending_step_reason = nil
    pending_step_start_pc = nil
    pending_step_seen_run = false
    debugger_manager().execution_state = "stop"
    emu.pause()
    broadcast_event("stopped", { reason = "pause", addr = get_register("pc") })
    return {}
  end,
  pause_for_setup = function()
    trace("pause_for_setup request pc=" .. trace_address(get_register("pc")))
    clear_temp_step_breakpoints()
    pending_step_reason = nil
    pending_step_start_pc = nil
    pending_step_seen_run = false
    debugger_manager().execution_state = "stop"
    emu.pause()
    return {}
  end,
  step_in = function()
    focus_maincpu_debugger()
    run_to_step_targets(step_in_targets())
    return {}
  end,
  step_over = function()
    focus_maincpu_debugger()
    run_to_step_targets(step_over_targets())
    return {}
  end,
  step_out = function()
    focus_maincpu_debugger()
    run_to_step_targets(step_out_targets())
    return {}
  end,
  -- Resume for an exact emulated duration, then stop with a `stopped`
  -- event (reason "runFor"). The MAME debugger's gtime command provides
  -- the deterministic emulated-time landing that sub-bridge-latency
  -- observations (e.g. the MC146818's 2.228 ms UIP window) require; this
  -- rides the same pending-step plumbing the step commands use.
  run_for_ms = function(params)
    local ms = assert(params and params.ms, "run_for_ms requires ms")
    ms = math.floor(tonumber(ms) or 0)
    if ms < 1 then
      error("run_for_ms requires ms >= 1")
    end
    trace("run_for_ms request ms=" .. tostring(ms) .. " pc=" .. trace_address(get_register("pc")))
    focus_maincpu_debugger()
    clear_temp_step_breakpoints()
    pending_step_reason = "runFor"
    pending_step_start_pc = get_register("pc")
    pending_step_seen_run = false
    emu.unpause()
    debugger_manager():command("gtime " .. tostring(ms))
    return {}
  end,
  get_io_state = function() return get_io_state() end,
  set_sensor = set_sensor,
  press_key = press_key,
  release_key = release_key,
  set_sleep_norm = set_sleep_norm,
  initialize_herojr_warm_context = initialize_herojr_warm_context,
  reset_machine = reset_machine
}

-- Every response carries the emulated-time stamp of the moment it was
-- built, so hosts can correlate command effects with machine time.
local function send_response(client, response)
  response.emulatedTimeSeconds = emulated_time_seconds()
  return pcall(send_line, client, response)
end

-- Returns false when the client's socket is no longer writable (the caller
-- closes and drops it); a torn response line must never stay half-sent.
local function handle_request(client, line)
  local ok, request = pcall(decode_json, line)
  if not ok or type(request) ~= "table" then
    -- No trustworthy request id exists in a line that failed to parse;
    -- id:null marks the error as request-uncorrelated (never id 0, which a
    -- client could legitimately use).
    local reason = ok and "request must be a JSON object" or tostring(request)
    return send_response(client, { id = json_null, ok = false, error = reason })
  end

  local handler = handlers[request.cmd]
  if not handler then
    return send_response(client, { id = request.id or json_null, ok = false, error = "unsupported command: " .. tostring(request.cmd) })
  end

  trace("request #" .. tostring(request.id) .. " " .. tostring(request.cmd) .. " pc=" .. trace_address(get_register("pc")))
  local success, result = xpcall(function() return handler(request.params or {}) end, debug.traceback)
  if success then
    if result == DEFER_RESPONSE then
      if pending_reset_ack then
        -- Unreachable for a request/await client; answer a superseded
        -- reset honestly rather than orphaning its id.
        send_response(pending_reset_ack.client, {
          id = pending_reset_ack.id,
          ok = false,
          error = "superseded by a newer reset_machine"
        })
      end
      pending_reset_ack = { client = client, id = request.id or json_null }
      trace("response #" .. tostring(request.id) .. " " .. tostring(request.cmd) .. " deferred until the reset completes")
      return true
    end
    trace("response #" .. tostring(request.id) .. " " .. tostring(request.cmd) .. " ok")
    return send_response(client, { id = request.id or json_null, ok = true, result = result or {} })
  end

  trace("response #" .. tostring(request.id) .. " " .. tostring(request.cmd) .. " failed: " .. tostring(result))
  diagnostic_event("mame:lua:rpc-error", "Lua bridge request failed.", {
    id = request.id,
    cmd = request.cmd,
    error = tostring(result),
    snapshot = bridge_debug_snapshot()
  })
  return send_response(client, { id = request.id or json_null, ok = false, error = tostring(result) })
end

-- A single NDJSON line is bounded: the largest legitimate request is a
-- write_mem of a few KB. Past this cap the line is discarded to its
-- terminating newline and answered with one id:null error instead of
-- growing the buffer without bound.
local MAX_REQUEST_LINE_BYTES = 256 * 1024
local oversized_discard = {}

drop_framing_state = function(client)
  oversized_discard[client] = nil
end

local function poll_client(client)
  while true do
    local line, err, partial = client:receive("*l")
    if line then
      if oversized_discard[client] then
        -- This line is the tail of a discarded oversized line; drop it and
        -- report once, then resume normal framing.
        oversized_discard[client] = nil
        local sent = send_response(client, {
          id = json_null,
          ok = false,
          error = "request line exceeded " .. tostring(MAX_REQUEST_LINE_BYTES) .. " bytes and was discarded"
        })
        if not sent then
          return false
        end
      else
        -- G0-05 split-line framing: a request that arrived across TCP
        -- segments accumulated its prefix in receive_buffers; the newline
        -- completes it here.
        local prefix = receive_buffers[client]
        if prefix then
          line = prefix .. line
          receive_buffers[client] = nil
        end
        if not handle_request(client, line) then
          return false
        end
      end
    elseif err == "timeout" then
      if partial and #partial > 0 and not oversized_discard[client] then
        local buffered = (receive_buffers[client] or "") .. partial
        if #buffered > MAX_REQUEST_LINE_BYTES then
          receive_buffers[client] = nil
          oversized_discard[client] = true
        else
          receive_buffers[client] = buffered
        end
      end
      return true
    elseif err == "closed" then
      return false
    else
      return true
    end
  end
end

-- The TCP listener binds at plugin start, long before the machine finishes
-- initializing. Serving a request in that window hits nil devices/debugger,
-- or worse: writes that machine_reset() then silently wipes (measured
-- 2026-06-10: pre-reset set_sensor injections reverted to defaults). Serve
-- only once the machine is genuinely ready — the same predicate the
-- autostart path waits on — and print the "bridge listening" gate line at
-- that moment, since that line is the readiness contract the extension and
-- harness gate on. Earlier connects sit unanswered in the listen backlog.
local bridge_serving = false
local bridge_no_debugger_polls = 0

local function bridge_ready_to_serve()
  if not server then
    -- The machine-stop notifier closed the listener (in-process machine
    -- restart). Never re-assert the gate line with nothing listening.
    return false
  end
  if not manager.machine then
    return false
  end
  if not program_space_available() then
    return false
  end
  if not cpu_debug_available() then
    -- The program space exists but the debugger does not: either a transient
    -- init window (clears within a few frames) or a launch without -debug.
    -- Report the latter once, loudly, instead of leaving the gate line absent
    -- with no explanation.
    bridge_no_debugger_polls = bridge_no_debugger_polls + 1
    if bridge_no_debugger_polls == 240 then
      emu.print_error(log_prefix() .. ": bridge requires the MAME debugger and will not serve; relaunch with -debug")
    end
    return false
  end
  if machine_reset_count < 1 then
    -- The machine's INITIAL reset has not run yet. running_machine::run()
    -- performs its unconditional startup soft reset AFTER Lua periodic
    -- callbacks already pump, so without this gate the "bridge listening"
    -- readiness line printed in the pre-reset window on every launch and a
    -- fast client could be served there — its writes then silently wiped
    -- when the initial reset restored driver defaults. That is the same
    -- wipe class bridge_ready_to_serve was built against (2026-06-10), with
    -- the residual window measured once as G1H-06's $D100 = 134 (the hero1
    -- machine_reset default) in gate conformance-2026-07-13T22-04-04Z (R4
    -- closure). The machine-reset notifier is registered in bridge.start(),
    -- before the machine runs, so it always counts the initial reset;
    -- reproR4ResetGateOrder pins the contract ordering in both directions.
    return false
  end
  return true
end

local function poll()
  if herojr_initial_sleep_pending and set_herojr_sleep_norm_field(false) then
    herojr_initial_sleep_pending = false
    emu.print_info(log_prefix() .. ": initialized HERO Jr Sleep/Norm switch to Sleep")
  end

  if not bridge_serving then
    if not bridge_ready_to_serve() then
      return
    end
    bridge_serving = true
    emu.print_info(log_prefix() .. ": bridge listening on " .. host .. ":" .. tostring(port) .. " via LuaSocket")
  end

  if herojr_reset_release_at and emulated_time_seconds() >= herojr_reset_release_at then
    -- Re-enable the debugger points BEFORE releasing the button, and only
    -- release once that succeeded: the CPU is suspended in reset until the
    -- release edge is sampled at the next emulated frame boundary, so this
    -- order can never miss a fresh-boot stop — and should the debugger
    -- ever be unavailable here, the fail-safe is a still-held RESET, never
    -- a session with silently disabled breakpoints.
    if cpu_debug_available() then
      cpu_debug():bpenable()
      cpu_debug():wpenable()
      herojr_reset_release_at = nil
      set_herojr_reset_field(false)
      -- The deasserted line is only SAMPLED at emulated frame boundaries;
      -- 40 ms (>= two 60 Hz samples, the engage window's own constant)
      -- past the release write guarantees the CPU is out of reset before
      -- a deferred reset_machine acknowledgement is flushed.
      herojr_reset_respond_at = emulated_time_seconds() + 0.040
    end
  end

  if autostart_monitor_pending and cpu_debug_available() and program_space_available() then
    local vector_ok, vector = pcall(reset_vector)
    if not vector_ok then
      return
    end
    autostart_monitor_pending = false
    manager.machine:soft_reset()
    emu.unpause()
    cpu_debug():go()
    emu.print_info(log_prefix() .. ": autostarted HERO ROM monitor at reset vector $" .. string.format("%04X", vector))
  end

  -- Stop detection MUST run before client requests are serviced (G2J-05 P0,
  -- 2026-07-10, gate conformance-2026-07-09T23-50-08Z). With `-debugger none`
  -- a stop auto-resumes (none.cpp wait_for_debugger() issues go()), so each
  -- stop's ONLY detection chance is the single periodic_check pump inside
  -- debugger_cpu::wait_for_debugger()'s stopped loop. When this function
  -- serviced requests first, a `continue` already queued on the socket —
  -- the client answering the PREVIOUS stop, with the two breakpoints only a
  -- few instructions apart ($9FDD/$9FE7) — flipped execution_state to "run"
  -- before the scan, and the stop was lost permanently. Deterministic both
  -- ways in conformance/tools/reproG2j05.ts (queued-continue scenario):
  -- request-first eats a stop 8/8, scan-first reports 8/8. Scanning first is
  -- safe: within one pump no instructions execute, so the scan sees the
  -- authoritative pre-request state, and a continue serviced afterwards
  -- simply resumes the just-broadcast stop.
  --
  -- Stop IDENTITY comes from the debugger's own triggered-point state
  -- (device_debug triggered_breakpoint()/triggered_watchpoint() — the same
  -- read-and-clear accessors the gdbstub OSD consumes), never from console
  -- text. The retired console scan (R3 remediation, 2026-07-12) matched
  -- "Stopped at breakpoint ([0-9]+)" against MAME's "%X"-formatted HEX
  -- index: the tenth breakpoint index of a session prints as "A", matches
  -- nothing, and every later stop was silently dropped for the rest of the
  -- session (conformance/tools/reproR3StopDetection.ts clause C wedged at
  -- round 7 = index $A pre-fix); its last_stop_pc dedup could also eat a
  -- legitimate immediate same-address re-stop. Each genuine halt sets the
  -- triggered pointer fresh, so a stop→continue→same-address-stop
  -- broadcasts every time, and the read-and-clear semantics make a
  -- double-broadcast impossible.
  if manager.machine.debugger and manager.machine.debugger.execution_state == "stop" then
    if pending_step_reason then
      local addr = get_register("pc")
      if pending_step_seen_run or addr ~= pending_step_start_pc then
        last_stop_pc = addr
        debugger_manager().execution_state = "stop"
        emu.pause()
        clear_temp_step_breakpoints()
        reset_ack_stop_seen = true
        broadcast_event("stopped", { reason = pending_step_reason, addr = addr })
        pending_step_reason = nil
        pending_step_start_pc = nil
        pending_step_seen_run = false
      end
    end
    if cpu_debug_available() then
      -- Drain BOTH pointers every stopped pump so nothing stale can ever
      -- be attributed to a later stop (a clear_temp_step_breakpoints above
      -- already nulls a triggered temp breakpoint fork-side on bpclear).
      local triggered_bp = cpu_debug():triggered_breakpoint()
      local triggered_wp = cpu_debug():triggered_watchpoint()
      if herojr_reset_release_at then
        -- Belt-and-suspenders: points are disabled across the reset engage
        -- window, so no stop should reach here; if one does (e.g. a stop
        -- already in flight when reset_machine was serviced), it belongs
        -- to the dying boot — never broadcast it as a protocol event.
        if triggered_bp or triggered_wp then
          trace("suppressed stale-boot stop during reset engage pc=" .. trace_address(get_register("pc")))
        end
        triggered_bp = nil
        triggered_wp = nil
      end
      if triggered_bp then
        local breakpoint_index = triggered_bp.index
        local temp_addr = temp_step_breakpoints[breakpoint_index]
        local addr = breakpoints_by_index[breakpoint_index]
        if temp_addr then
          local current_pc = get_register("pc")
          debugger_manager().execution_state = "stop"
          emu.pause()
          clear_temp_step_breakpoints()
          last_stop_pc = current_pc
          reset_ack_stop_seen = true
          trace("broadcast stopped reason=step addr=" .. trace_address(current_pc))
          broadcast_event("stopped", { reason = "step", addr = current_pc })
          pending_step_reason = nil
          pending_step_start_pc = nil
          pending_step_seen_run = false
        elseif addr then
          debugger_manager().execution_state = "stop"
          emu.pause()
          last_stop_pc = addr
          reset_ack_stop_seen = true
          trace("broadcast stopped reason=breakpoint addr=" .. trace_address(addr) .. " index=" .. tostring(breakpoint_index))
          broadcast_event("stopped", { reason = "breakpoint", addr = addr, breakpoint = breakpoint_index })
        end
      end
      if triggered_wp then
        local watchpoint_index = triggered_wp.index
        local watch_addr = read_watchpoints_by_index[watchpoint_index]
        if watch_addr then
          local current_pc = get_register("pc")
          local value = read_u8(watch_addr)
          debugger_manager().execution_state = "stop"
          emu.pause()
          last_stop_pc = current_pc
          reset_ack_stop_seen = true
          trace("broadcast stopped reason=robotLanguageRead addr=" .. trace_address(watch_addr) .. " value=$" .. string.format("%02X", value) .. " pc=" .. trace_address(current_pc) .. " index=" .. tostring(watchpoint_index))
          broadcast_event("stopped", {
            reason = "robotLanguageRead",
            addr = watch_addr,
            pc = current_pc,
            value = value,
            watchpoint = watchpoint_index
          })
          if read_watchpoint_one_shot_by_index[watchpoint_index] then
            clear_read_watchpoint_by_index(watchpoint_index)
          end
        end
      end
    end
  elseif manager.machine.debugger and manager.machine.debugger.execution_state == "run" then
    if pending_step_reason then
      pending_step_seen_run = true
    end
    last_stop_pc = nil
  end

  -- Flush a deferred reset_machine acknowledgement once the reset has
  -- observably completed (see the pending_reset_ack note). Runs after the
  -- stop scan so a genuine stop broadcast THIS pump already counts, and
  -- before request servicing so the client's next command can only ever be
  -- seen after its reset completed.
  if pending_reset_ack then
    local reset_done
    if system_name() == "herojr" then
      reset_done = herojr_reset_release_at == nil and herojr_reset_respond_at ~= nil
        and (emulated_time_seconds() >= herojr_reset_respond_at or reset_ack_stop_seen)
    else
      reset_done = machine_reset_count > machine_reset_baseline
    end
    if reset_done then
      local ack = pending_reset_ack
      pending_reset_ack = nil
      herojr_reset_respond_at = nil
      trace("response #" .. tostring(ack.id) .. " reset_machine ok (deferred until the reset completed)")
      if not send_response(ack.client, { id = ack.id, ok = true, result = {} }) then
        drop_client(ack.client)
      end
    end
  end

  if server then
    while true do
      local client = server:accept()
      if not client then
        break
      end
      client:settimeout(0)
      if #clients >= 1 then
        -- G0-04 single-client bridge: the second concurrent client is
        -- actively refused — an explicit error line, then a deliberate
        -- close — while the first client stays served.
        send_response(client, {
          id = json_null,
          ok = false,
          error = "bridge already has an active client; the HERO bridge is single-client"
        })
        pcall(function() client:close() end)
      else
        -- No greeting event: the protocol's readiness contract is the
        -- client's own request/response round trip (the old `ready` event
        -- proved nothing a client could rely on and was deleted).
        clients[#clients + 1] = client
      end
    end
  end

  -- Reap by identity, never by captured index: a mid-handler broadcast can
  -- remove entries from `clients` while this loop runs, so a stale index
  -- would close an innocent client.
  for _, client in ipairs({ table.unpack(clients) }) do
    if not poll_client(client) then
      drop_client(client)
    end
  end

  -- Post-request replication of the scan's "run" bookkeeping. Requests above
  -- (continue, step_*, run_for_ms, reset_machine) flip the debugger to run
  -- AFTER this pump's scan already happened; when requests were serviced
  -- first, the same pump's run branch did this. Without it, a sub-frame
  -- gtime window (run_for_ms shorter than the next pump — G1J-09's 5 ms UIP
  -- landings) expires with pending_step_seen_run still false and a landing
  -- PC equal to the start PC (tight RTC poll loop), so the runFor stop is
  -- never broadcast and the auto-resume runs away (gate
  -- conformance-2026-07-10T16-06-09Z, both G1J-09 subtests).
  if manager.machine.debugger and manager.machine.debugger.execution_state == "run" then
    if pending_step_reason then
      pending_step_seen_run = true
    end
    last_stop_pc = nil
  end

  broadcast_phoneme_events()
  broadcast_io_changed()
end

function bridge.start()
  server = bind_tcp_server(host, port)
  server:settimeout(0)
  emu.register_periodic(poll)
  emu.register_frame_done(poll)
  -- Completion signal for the deferred reset_machine acknowledgement:
  -- running_machine::soft_reset() calls the machine-reset notifiers, so a
  -- count increment proves the scheduled reset actually executed.
  subscriptions[#subscriptions + 1] = emu.add_machine_reset_notifier(function()
    machine_reset_count = machine_reset_count + 1
  end)
  subscriptions[#subscriptions + 1] = emu.add_machine_stop_notifier(function()
    bridge_serving = false
    bridge_no_debugger_polls = 0
    for _, client in ipairs(clients) do
      client:close()
    end
    clients = {}
    receive_buffers = {}
    oversized_discard = {}
    if server then
      server:close()
      server = nil
    end
  end)
  -- The "bridge listening" gate line is printed by poll() once the machine
  -- is ready to serve, not here: at this point requests would still fail or
  -- be wiped by machine_reset.
end

return bridge
