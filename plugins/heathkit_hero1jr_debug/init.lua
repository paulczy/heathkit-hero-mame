-- license:BSD-3-Clause
-- copyright-holders:Paul Czywczynski

local exports = {}

exports.name = "heathkit_hero1jr_debug"
exports.version = "0.0.1"
exports.description = "Heathkit HERO 1/Jr VS Code debug bridge"
exports.license = "BSD-3-Clause"
exports.author = { name = "Paul Czywczynski" }

function exports.startplugin()
  local plugin = manager.plugins[exports.name]
  if plugin and plugin.directory then
    package.path = plugin.directory .. "/?.lua;" .. package.path
  end

  local ok, bridge = pcall(require, "heathkit_hero1jr_debug_bridge")
  if not ok then
    emu.print_error("heathkit_hero1jr_debug: unable to load bridge: " .. tostring(bridge))
    return
  end

  bridge.start()
end

return exports
