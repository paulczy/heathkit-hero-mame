# LuaSocket (vendored C core)

Source: https://github.com/lunarmodules/luasocket tag `v3.1.0`
(commit `95b7efa9da506ef968c1347edf3fc56370f0deed`), MIT license (see LICENSE).

Only the C core needed by `luaopen_socket_core` is vendored (TCP/UDP/select
plus their support modules). The pure-Lua wrappers (`socket.lua`, `http.lua`,
…), MIME, serial, and unix-domain sources are intentionally omitted: the
HERO bridge plugin uses `require("socket.core")` directly.

Local changes vs upstream (kept minimal, each marked `vendored change`):
- `src/inet.c` `inet_trycreate`: explicit `(const char *)` cast in the
  `setsockopt` call — the sources build as C++ here (see below), where the
  implicit `void *` conversion is an error.

Build wiring:
- `scripts/src/3rdparty.lua` — `project "luasocket"` (built **as C++** via
  `ForceCPP`/`-x c++`, matching `lualibs`: MAME compiles its bundled Lua as
  C++, so the Lua API symbols are C++-mangled; `wsocket.c` on Windows,
  `usocket.c` elsewhere; ws2_32 is already in the global Windows link set).
- `scripts/src/main.lua` — linked next to `lualibs`.
- `src/frontend/mame/luaengine.cpp` — preloaded as `package.preload["socket.core"]`.

The HERO bridge transport REQUIRES this module: the plugin treats a missing
`socket.core` as a fatal startup error (no fallback transport exists).
