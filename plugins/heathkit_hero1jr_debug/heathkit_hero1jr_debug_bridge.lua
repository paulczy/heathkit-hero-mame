-- license:BSD-3-Clause
-- copyright-holders:Paul Czywczynski

local bridge = {}

local socket_ok, socket = pcall(require, "socket")

local function make_emu_file_socket_module()
  local module = {}

  function module.bind(bind_host, bind_port)
    local function create_client()
      local file = emu.file("", 7)
      file:open("socket." .. bind_host .. ":" .. tostring(bind_port))

      local client = {
        buffer = "",
        closed = false
      }

      function client:settimeout(_timeout)
      end

      function client:send(data)
        file:write(data)
        return #data
      end

      function client:receive(pattern)
        if self.closed then
          return nil, "closed"
        end

        if pattern ~= "*l" then
          error("emu.file socket fallback only supports line reads")
        end

        local newline = self.buffer:find("\n", 1, true)
        if newline then
          local line = self.buffer:sub(1, newline - 1):gsub("\r$", "")
          self.buffer = self.buffer:sub(newline + 1)
          return line
        end

        local chunk = file:read(4096)
        if chunk and #chunk > 0 then
          self.buffer = self.buffer .. chunk
          newline = self.buffer:find("\n", 1, true)
          if newline then
            local line = self.buffer:sub(1, newline - 1):gsub("\r$", "")
            self.buffer = self.buffer:sub(newline + 1)
            return line
          end
        end

        return nil, "timeout"
      end

      function client:close()
        self.closed = true
        file:close()
      end

      return client
    end

    local server = {
      client = nil
    }

    function server:settimeout(_timeout)
    end

    function server:accept()
      if self.client and not self.client.closed then
        return nil
      end

      self.client = create_client()
      return self.client
    end

    function server:close()
      if self.client then
        self.client:close()
        self.client = nil
      end
    end

    return server
  end

  return module
end

if not socket_ok then
  socket = make_emu_file_socket_module()
end

local host = os.getenv("HEATHKIT_HERO_BRIDGE_HOST") or "127.0.0.1"
local port = tonumber(os.getenv("HEATHKIT_HERO_BRIDGE_PORT") or "6808")
local autostart_monitor = os.getenv("HEATHKIT_HERO_BRIDGE_AUTOSTART_MONITOR") == "1"
local bridge_trace = os.getenv("HEATHKIT_HERO_BRIDGE_TRACE") == "1"
local autostart_monitor_pending = autostart_monitor
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
local console_log_last = 0
local last_io_state_encoded = nil
local sensor_state = {
  sonarDistanceInches = 48,
  sonarHits = 0,
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
local function trace(message)
  if bridge_trace then
    emu.print_info("heathkit_hero1jr_debug: trace: " .. message)
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
        local column_mask = 1 << (nibble & 0x03)
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
  local column = nibble & 0x03
  local port = manager.machine.ioport.ports[":KEY" .. tostring(row)] or manager.machine.ioport.ports["KEY" .. tostring(row)]
  if not port then
    return
  end

  local field = port:field(1 << column)
  if field then
    field:set_value(pressed and 1 or 0)
  end
end
local subscriptions = {}
local pending_step_reason = nil
local pending_step_start_pc = nil
local pending_step_seen_run = false
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
  "get_io_state",
  "set_sensor",
  "press_key",
  "release_key",
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

local function encode_json(value)
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

local function send_line(client, value)
  client:send(encode_json(value) .. "\n")
end

local function broadcast_event(name, payload)
  for i = #clients, 1, -1 do
    local ok = pcall(send_line, clients[i], { event = name, payload = payload or {} })
    if not ok then
      clients[i]:close()
      table.remove(clients, i)
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
  return {
    memory = {
      u203 = { startAddress = 0x0000, endAddress = 0x1fff, kind = "ram" },
      u204 = { startAddress = 0x2000, endAddress = 0x3fff, kind = "ram" },
      u205 = { startAddress = 0x4000, endAddress = 0x5fff, kind = "ram" },
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

local function get_capabilities()
  return {
    system = system_name(),
    profile = profile_name(),
    systemDescription = system_description(),
    protocolVersion = 1,
    transport = "tcp",
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
  local herojr_d821 = prefix == "herojr" and read_u8(0xd821) or 0
  local herojr_d822 = prefix == "herojr" and read_u8(0xd822) or 0
  local herojr_d823 = prefix == "herojr" and read_u8(0xd823) or 0
  local herojr_d841 = prefix == "herojr" and read_u8(0xd841) or 0
  local herojr_d842 = prefix == "herojr" and read_u8(0xd842) or 0
  local herojr_d843 = prefix == "herojr" and read_u8(0xd843) or 0
  local herojr_motion_detector = output_value(prefix .. "_motion_detector")
  local herojr_wheel_feedback = output_value(prefix .. "_wheel_feedback")
  local herojr_drive_activity = output_value(prefix .. "_drive_activity")
  local herojr_rtc_sqw = output_value(prefix .. "_rtc_sqw")
  local herojr_adc_sample = output_value(prefix .. "_adc_sample")
  local herojr_adc_output = output_value(prefix .. "_adc_output")
  local herojr_sonar_echo = output_value(prefix .. "_sonar_echo")
  local herojr_sonar_distance = output_value(prefix .. "_sonar_distance")
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
        motionDetected = sensor_state.motionDetected
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
        left = output_value(motor_prefix .. "left"),
        right = output_value(motor_prefix .. "right"),
        head = output_value(motor_prefix .. "head"),
        arm = output_value(motor_prefix .. "arm"),
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

local function broadcast_io_changed()
  if #clients == 0 then
    last_io_state_encoded = nil
    return
  end
  if not program_space_available() then
    return
  end

  local state = get_io_state()
  local encoded = encode_json(state)
  if encoded ~= last_io_state_encoded then
    last_io_state_encoded = encoded
    broadcast_event("io_changed", state)
  end
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
    sensor_state.sonarHits = (sensor_state.sonarHits + 1) & 0xff
    if system_name() == "hero1" then
      write_u8(sensor_base + 0x00, sonar_count_byte_from_inches(params.value))
      write_u8(0x0010, sensor_state.sonarHits)
      write_u8(0x0011, sonar_count_byte_from_inches(params.value))
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
      local port_b = read_u8(0xd821)
      if params.value then
        write_u8(0xd821, port_b | 0x80)
      else
        write_u8(0xd821, port_b & 0x7f)
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
  broadcast_io_changed()
  return {}
end

local function press_key(params)
  local key = assert(params.key, "press_key requires key")
  keypad_state[key] = true
  local mapping = key_map[key]
  if mapping then
    keypad_columns[mapping.column] = keypad_columns[mapping.column] & (~mapping.bit & 0x3f)
    write_keypad_column(mapping.column)
  end
  if system_name() == "herojr" then
    set_herojr_keypad_field(key, true)
    write_u8(0xd820, herojr_keypad_byte())
  end
  broadcast_io_changed()
  return {}
end

local function release_key(params)
  local key = assert(params.key, "release_key requires key")
  keypad_state[key] = nil
  local mapping = key_map[key]
  if mapping then
    keypad_columns[mapping.column] = (keypad_columns[mapping.column] | mapping.bit) & 0x3f
    write_keypad_column(mapping.column)
  end
  if system_name() == "herojr" then
    set_herojr_keypad_field(key, false)
    write_u8(0xd820, 0xfe)
  end
  broadcast_io_changed()
  return {}
end

local function reset_machine(params)
  local preserve_herojr_keys = system_name() == "herojr" and params and params.preserveKeys == true
  clear_temp_step_breakpoints()
  if not preserve_herojr_keys then
    keypad_state = {}
    keypad_columns = { 0x3f, 0x3f, 0x3f }
    write_keypad_column(1)
    write_keypad_column(2)
    write_keypad_column(3)
    if system_name() == "herojr" then
      for key, _ in pairs(key_map) do
        set_herojr_keypad_field(key, false)
      end
      for key = 0, 15 do
        set_herojr_keypad_field(string.format("%X", key), false)
      end
      write_u8(0xd820, 0xfe)
    end
  end
  manager.machine:soft_reset()
  if preserve_herojr_keys then
    write_u8(0xd820, herojr_keypad_byte())
  end
  emu.unpause()
  cpu_debug():go()
  broadcast_io_changed()
  return {}
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
  get_io_state = function() return get_io_state() end,
  set_sensor = set_sensor,
  press_key = press_key,
  release_key = release_key,
  reset_machine = reset_machine
}

local function handle_request(client, line)
  local ok, request = pcall(decode_json, line)
  if not ok then
    send_line(client, { id = 0, ok = false, error = tostring(request) })
    return
  end

  local handler = handlers[request.cmd]
  if not handler then
    send_line(client, { id = request.id, ok = false, error = "unsupported command: " .. tostring(request.cmd) })
    return
  end

  trace("request #" .. tostring(request.id) .. " " .. tostring(request.cmd) .. " pc=" .. trace_address(get_register("pc")))
  local success, result = pcall(handler, request.params or {})
  if success then
    trace("response #" .. tostring(request.id) .. " " .. tostring(request.cmd) .. " ok")
    send_line(client, { id = request.id, ok = true, result = result or {} })
  else
    trace("response #" .. tostring(request.id) .. " " .. tostring(request.cmd) .. " failed: " .. tostring(result))
    send_line(client, { id = request.id, ok = false, error = tostring(result) })
  end
end

local function poll_client(client)
  while true do
    local line, err, partial = client:receive("*l")
    if line then
      handle_request(client, line)
    elseif err == "timeout" then
      if partial and #partial > 0 then
        receive_buffers[client] = (receive_buffers[client] or "") .. partial
      end
      return true
    elseif err == "closed" then
      return false
    else
      return true
    end
  end
end

local function poll()
  if autostart_monitor_pending and cpu_debug_available() and program_space_available() then
    local vector_ok, vector = pcall(reset_vector)
    if not vector_ok then
      return
    end
    autostart_monitor_pending = false
    manager.machine:soft_reset()
    emu.unpause()
    cpu_debug():go()
    emu.print_info("heathkit_hero1jr_debug: autostarted HERO ROM monitor at reset vector $" .. string.format("%04X", vector))
  end

  if server then
    while true do
      local client = server:accept()
      if not client then
        break
      end
      client:settimeout(0)
      clients[#clients + 1] = client
      send_line(client, { event = "ready", payload = { system = system_name(), profile = profile_name(), systemDescription = system_description(), pc = hex_pc() } })
    end
  end

  for i = #clients, 1, -1 do
    if not poll_client(clients[i]) then
      clients[i]:close()
      table.remove(clients, i)
    end
  end

  if manager.machine.debugger and manager.machine.debugger.execution_state == "stop" then
    if pending_step_reason then
      local addr = get_register("pc")
      if pending_step_seen_run or addr ~= pending_step_start_pc then
        last_stop_pc = addr
        debugger_manager().execution_state = "stop"
        emu.pause()
        clear_temp_step_breakpoints()
        broadcast_event("stopped", { reason = pending_step_reason, addr = addr })
        pending_step_reason = nil
        pending_step_start_pc = nil
        pending_step_seen_run = false
      end
    end
    local consolelog = manager.machine.debugger.consolelog
    local count = #consolelog
    for index = console_log_last + 1, count do
      local message = consolelog[index]
      local breakpoint_index = message and tonumber(message:match("Stopped at breakpoint ([0-9]+)"))
      if breakpoint_index then
        trace("debugger console stopped breakpoint index=" .. tostring(breakpoint_index) .. " message=" .. tostring(message))
      end
      local watchpoint_index = message and tonumber(message:match("Stopped at watchpoint ([0-9]+)"))
      if watchpoint_index then
        trace("debugger console stopped watchpoint index=" .. tostring(watchpoint_index) .. " message=" .. tostring(message))
      end
      local temp_addr = breakpoint_index and temp_step_breakpoints[breakpoint_index]
      local addr = breakpoint_index and breakpoints_by_index[breakpoint_index]
      local watch_addr = watchpoint_index and read_watchpoints_by_index[watchpoint_index]
      if temp_addr then
        local current_pc = get_register("pc")
        debugger_manager().execution_state = "stop"
        emu.pause()
        clear_temp_step_breakpoints()
        last_stop_pc = current_pc
        trace("broadcast stopped reason=step addr=" .. trace_address(current_pc))
        broadcast_event("stopped", { reason = "step", addr = current_pc })
        pending_step_reason = nil
        pending_step_start_pc = nil
        pending_step_seen_run = false
      elseif addr and last_stop_pc ~= addr then
        debugger_manager().execution_state = "stop"
        emu.pause()
        last_stop_pc = addr
        trace("broadcast stopped reason=breakpoint addr=" .. trace_address(addr) .. " index=" .. tostring(breakpoint_index))
        broadcast_event("stopped", { reason = "breakpoint", addr = addr, breakpoint = breakpoint_index })
      elseif watch_addr then
        local current_pc = get_register("pc")
        local value = read_u8(watch_addr)
        debugger_manager().execution_state = "stop"
        emu.pause()
        last_stop_pc = current_pc
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
    console_log_last = count
  elseif manager.machine.debugger and manager.machine.debugger.execution_state == "run" then
    if pending_step_reason then
      pending_step_seen_run = true
    end
    last_stop_pc = nil
    console_log_last = #manager.machine.debugger.consolelog
  end

  broadcast_io_changed()
end

function bridge.start()
  server = assert(socket.bind(host, port))
  server:settimeout(0)
  emu.register_periodic(poll)
  subscriptions[#subscriptions + 1] = emu.add_machine_stop_notifier(function()
    for _, client in ipairs(clients) do
      client:close()
    end
    clients = {}
    if server then
      server:close()
      server = nil
    end
  end)
  local transport = socket_ok and "LuaSocket" or "emu.file"
  emu.print_info("heathkit_hero1jr_debug: bridge listening on " .. host .. ":" .. tostring(port) .. " via " .. transport)
end

return bridge
