# OctoLink MCP Skill

Practical operating guide for Claude / AI agents using OctoLink MCP tools to debug embedded targets via GDB, OpenOCD, and serial.

---

## When to Use

Use OctoLink MCP when you need to:

- Inspect target memory, variables, registers, or RTOS state
- Read sensor data or peripheral registers from an embedded target
- Set/delete breakpoints and control execution (step, continue, reset)
- Monitor multiple variables in a single safe batch read
- Communicate with the target via serial port (text or binary)
- Decode OctoLite / legacy binary telemetry frames over serial

**Do not use** when the target is not connected or GDB is not running. Always check status first.

---

## Startup Checklist

Before issuing any MCP tool call, verify the environment:

- [ ] **OctoLink GUI is running**
- [ ] **Agent Bridge MCP server is green** (open Agent Bridge modal -> Start MCP Server -> status dot green)
- [ ] **Project `.mcp.json` is configured** (or `--mcp-config` passed to Claude Code)
- [ ] **Target is connected** - call `get_openocd_status` (running, target detected)
- [ ] **GDB is running** - call `get_gdb_status` (running, ELF loaded, connected)
- [ ] **Serial port status** - call `get_serial_status` if serial work is planned

### Quick health snapshot

```
Tool: get_debug_context
```

Returns a single snapshot of all subsystems: backend, OpenOCD, GDB, serial, offline symbols, with timestamp. Use this as your first call to assess readiness.

### Recommended first calls

| Call | What it tells you |
|------|-------------------|
| `get_debug_context` | Full subsystem snapshot - fastest way to see what's ready |
| `get_mcp_session_health` | Agent readiness summary: safe read/write flags, recent GDB timeouts, recovery suggestion |
| `telemetry_find_series` | First discovery call for tuning/trend/recent-data tasks; use before guessing raw stream keys |
| `get_mcp_permission_policy` | Tool permission levels and `confirm:true` requirements |
| `get_gdb_status` | GDB running, PID, ELF path, connection state |
| `get_openocd_status` | OpenOCD running, PID, port, target detection |
| `get_serial_status` | Serial open/closed, baud, RX/TX counters |

---

## Confirmation Convention

State-changing operations require `"confirm": true` in the arguments. Without it the tool returns an error.

| Tool | Requires `confirm` |
|------|:------------------:|
| `get_debug_context`, `get_gdb_status`, `get_openocd_status`, `get_serial_status`, `list_serial_ports`, `get_offline_symbol_status` | No |
| `gdb_health_check` | No |
| `gdb_recover_session` | **Yes** |
| `gdb_evaluate_expression`, `gdb_evaluate_expressions`, `gdb_read_expressions_safe` | No |
| `read_memory_u32`, `read_memory_bytes` | No |
| `gdb_list_breakpoints` | No |
| `target_interrupt` | No (halting is safe) |
| `write_gdb_variable` | **Yes** |
| `gdb_set_breakpoint`, `gdb_delete_breakpoint` | **Yes** |
| `target_continue` | **Yes** |
| `target_step`, `target_next`, `target_finish` | **Yes** |
| `target_reset_halt`, `target_reset_run` | **Yes** |
| `serial_open`, `serial_close`, `serial_write`, `serial_clear_counters` | **Yes** |
| `debug_get_session_state`, `debug_diagnose_session`, `debug_get_source_context`, `debug_open_source_location`, `debug_clear_session_timeline`, `debug_export_session_report`, `debug_get_session_timeline`, `debug_get_breakpoint_hits`, `debug_get_variable_history` | No |
| `telemetry_suggest_series`, `telemetry_find_series`, `telemetry_list_series`, `telemetry_get_history`, `telemetry_start_capture`, `telemetry_get_capture`, `telemetry_list_captures`, `telemetry_stop_capture` | No |
| `tuning_write_and_observe`, `tuning_step_response` | **Yes when writing** |
| `tuning_compare_before_after` | No |
| `debug_capture_fault_context` | No (read-only) |
| `debug_write_and_verify` | **Yes** |
| `debug_run_to_breakpoint` | **Yes** |
| `debug_step_until` | **Yes** |
| `debug_watch_expression_until_changed` | **Yes** |
| `debug_recover_session` | **Yes** |

Always inform the user what will happen before calling a confirm-required tool.

---

## Safe GDB Variable Read/Write Workflow

### Reading variables (safe, read-only)

**Single expression:**

```
Tool: gdb_evaluate_expression
Args: { "expression": "myVariable" }
```

**Batch read (up to 256 expressions, server auto-chunks at 32):**

```
Tool: gdb_evaluate_expressions
Args: { "expressions": ["var1", "var2", "sensor.temp"], "timeoutMs": 2000 }
```

**Safe batch read with auto halt/resume (preferred for agents):**

```
Tool: gdb_read_expressions_safe
Args: {
  "expressions": ["uwTick", "sensorValue", "motorPWM"],
  "haltIfRunning": true,
  "resumeAfterRead": true,
  "allowReadWhileRunning": false
}
```

Response fields to check:

| Field | Meaning |
|-------|---------|
| `readAttempted` | `false` if halt failed and `allowReadWhileRunning=false` - no data read |
| `haltSucceeded` | Whether target was halted for the read |
| `target_state_before` / `target_state_after` | State transitions |
| `recommendedNextTool` / `nextTool` | Usually `telemetry_find_series` when halt failed early |
| `nextArgs` | Ready-to-call arguments for the recommended next tool |
| `workflowHint` | Text such as `telemetry_first`; use it to avoid GDB timeout loops |
| `warnings` | Non-fatal halt/resume problems |

**If `readAttempted: false`:** Do NOT retry single-expression reads or repeatedly call `target_interrupt`. First use `telemetry_find_series` / `telemetry_get_history` for live data. If exact GDB reads are mandatory, ask the user before `gdb_recover_session { "confirm": true }`; reset is a last-resort user-confirmed option.

---

## Telemetry History And Triggered Capture

For tuning and real-time analysis, prefer cached telemetry datasets over repeated one-shot GDB reads.

OctoLink records two telemetry sources in memory:

- `gdb` - samples from OctoLink GDB watch variables while they are refreshing.
- `stream` - samples decoded from serial stream variables.

### Telemetry-first rule for live tuning

When `get_debug_context` or `get_mcp_session_health` reports `gdb.target_state: "running"` and the user asks for trend, tuning, recent data, real-time behavior, a time window, a dataset, or tuning:

1. **Check for stale state first.** Call `get_mcp_session_health` or `debug_get_session_state` — these automatically probe the target. If `stateProbe.stateChanged` is `true`, the target is actually halted and synchronous GDB reads are safe. Skip the telemetry-first path.
2. Prefer `telemetry_find_series` / `telemetry_suggest_series` (ranked, with reasons) or `telemetry_list_series` first when the exact `source` + `key` is unknown. **Always discover before capturing.**
3. Prefer `telemetry_get_history` once a likely series is known.
4. Do not call `gdb_evaluate_expression`, `gdb_evaluate_expressions`, `gdb_read_expressions_safe`, `gdb_list_breakpoints`, or other synchronous GDB reads in a retry loop while the target is running.
5. If no telemetry is cached, ask the user to enable the matching GDB watch or stream variable in OctoLink, or ask for permission to halt/recover the target.
6. Use `includeSamples:false` for quick decisions and `includeSamples:true` only when raw points are needed for plotting or numerical fitting.

This keeps the MCU running and avoids GDB timeout cascades.

### Discover series (suggest or list)

When the agent does not know the exact `source` + `key`, **must** call one of these before `telemetry_start_capture`:

**`telemetry_suggest_series`** 鈥?ranked candidates with human-readable reasons (preferred):

```
Tool: telemetry_suggest_series
Args: {
  "query": "yaw",
  "source": "stream",
  "timeWindowMs": 10000,
  "maxSeries": 16
}
```

Each candidate includes `score`, `reason` (e.g. "exact key match; 120 samples 鈥?high density; ~50 Hz 鈥?high rate; latest value: 1.2345"), and `summary` stats. Pick the highest-scored candidate.

**`telemetry_list_series`** 鈥?flat list with compact summaries (alternative):

```
Tool: telemetry_list_series
Args: {
  "source": "stream",
  "query": "yaw",
  "timeWindowMs": 10000,
  "maxSeries": 32
}
```

Use the returned `series[].source` and `series[].key` exactly. For example, a label may describe yaw, but the actual key might be `stream:214`.

### Semantic names and tuning tools

Telemetry tools support semantic stream bindings. A user can bind a raw stream key to a readable name in the OctoLink GUI. Mini gimbal defaults include:

| Raw key | Semantic key | Unit |
|---------|--------------|------|
| `stream:210` | `mini_gimbal_clock_ms` | `ms` |
| `stream:211` | `mini_gimbal_state` | - |
| `stream:212` | `mini_gimbal_imu_roll` | `deg` |
| `stream:213` | `mini_gimbal_imu_pitch` | `deg` |
| `stream:214` | `mini_gimbal_imu_yaw` | `deg` |
| `stream:215` | `mini_gimbal_pitch_error_deg` | `deg` |
| `stream:216` | `mini_gimbal_yaw_error_deg` | `deg` |
| `stream:217` | `mini_gimbal_pitch_cmd_rad_s` | `rad/s` |
| `stream:218` | `mini_gimbal_yaw_cmd_rad_s` | `rad/s` |
| `stream:219` | `mini_gimbal_pitch_cmd_rpm` | `rpm` |
| `stream:220` | `mini_gimbal_yaw_cmd_rpm` | `rpm` |
| `stream:221` | `mini_gimbal_imu_rx_count` | - |
| `stream:222` | `mini_gimbal_last_status` | - |

After binding, use raw keys, semantic keys, or natural-language queries. Prefer semantic keys in user-facing explanations. Examples:

- `telemetry_find_series({ "query": "yaw rpm", "source": "stream" })` should pick `mini_gimbal_yaw_cmd_rpm` / `stream:220`.
- `telemetry_find_series({ "query": "pitch rpm", "source": "stream" })` should pick `mini_gimbal_pitch_cmd_rpm` / `stream:219`.
- `telemetry_get_history({ "source": "stream", "key": "mini_gimbal_yaw_cmd_rpm" })` reads the same series as `stream:220`.

For control tuning, prefer high-level tuning tools:

```
Tool: telemetry_find_series
Args: { "query": "yaw", "source": "stream", "timeWindowMs": 10000 }

Tool: tuning_write_and_observe
Args: {
  "expression": "mini_gimbal_yaw_kp",
  "value": "12.0",
  "telemetrySource": "stream",
  "observeMs": 4000,
  "confirm": true
}
```

When `telemetryKey` is omitted, OctoLink infers a likely mini gimbal series from the expression name (`yaw_kp` -> yaw error, `yaw_cmd` -> yaw command rpm, etc.). Read `naturalSummary`, `trendTags`, `recommendedNextAction`, `overshoot`, `steadyStateError`, `responseTimeMs`, `settlingTimeMs`, and `oscillationTrend` before giving tuning advice. If the tool reports no usable telemetry, do not claim the test succeeded; use returned candidates or ask the user to enable the relevant watch/stream first.

### Get recent history

Use this when the user asks for "the last few seconds", "a dataset", "trend", "璋冨弬鏁版嵁", or "瀹炴椂鎬ф洿濂戒竴鐐?:

```
Tool: telemetry_get_history
Args: {
  "source": "stream",
  "timeWindowMs": 5000,
  "limitPerSeries": 1000,
  "maxSeries": 32,
  "includeSamples": false
}
```

Read `series[].summary` before raw samples:

| Field | Use |
|------|-----|
| `lastValue` | Latest numeric value |
| `min` / `max` / `mean` | Basic trend range |
| `delta` | Net change over the returned window |
| `sampleRateHz` | Whether data is fresh enough for tuning |
| `ageMs` | Whether the latest sample is stale |

Fetch raw points only when plotting or fitting:

```
Tool: telemetry_get_history
Args: {
  "source": "stream",
  "key": "motor_speed",
  "timeWindowMs": 3000,
  "limitPerSeries": 1000,
  "includeSamples": true
}
```

For a specific watched GDB expression:

```
Tool: telemetry_get_history
Args: {
  "source": "gdb",
  "key": "uwTick",
  "timeWindowMs": 3000,
  "limitPerSeries": 500,
  "includeSamples": false
}
```

Do not use this tool if the user expects a fresh target halt/read. It returns cached samples already observed by OctoLink.

### Triggered capture

Use triggered capture when the user or agent wants to record data only after a condition happens. **Before calling `telemetry_start_capture`, the agent must know the exact `source` + `key`** 鈥?discover via `telemetry_suggest_series` or `telemetry_list_series` if unknown.

Condition syntax is simple numeric comparison over `value` or `v`:

- `value > 10`
- `value <= 0.5`
- `v == 1`
- `v != 0`

Start a capture:

```
Tool: telemetry_start_capture
Args: {
  "name": "motor step response",
  "source": "stream",
  "key": "motor_speed",
  "startCondition": "value > 100",
  "stopCondition": "value > 2000",
  "maxDurationMs": 10000
}
```

Always set either `stopCondition` or `maxDurationMs` unless the user explicitly wants a manually stopped capture.

If `telemetry_start_capture` returns `captureUsable:false`, the capture was **NOT** successfully created 鈥?do not report it as a working capture to the user. The requested source/key has no recent data. Use the included `suggestedSeries` to pick an active series, or call `telemetry_suggest_series` / `telemetry_list_series` to discover alternatives. Pass `allowEmptyCapture:true` only when the agent explicitly needs a capture waiting for future data.

Check progress:

```
Tool: telemetry_list_captures
```

Fetch one capture:

```
Tool: telemetry_get_capture
Args: { "id": "cap-1", "includeSamples": false }
```

Manually stop:

```
Tool: telemetry_stop_capture
Args: { "id": "cap-1", "reason": "userFinished" }
```

### Ask the user when trigger intent is unclear

If the user says "绛夊彉閲忓彉鍖栧悗寮€濮嬮噰鏍? but does not specify all details, ask a concise question before creating the capture:

- Which source: `gdb` watch or `stream`?
- Which variable/key/expression?
- What starts capture?
- What stops capture: another threshold, time duration, or manual stop?

Example phrasing:

> 鎴戝彲浠ュ府浣犲紑涓€涓Е鍙戦噰鏍枫€備綘甯屾湜鐩戞帶鍝釜鍙橀噺锛熻揪鍒颁粈涔堟潯浠跺紑濮嬶紝浠€涔堟椂鍊欏仠姝紵

Capture sessions observe incoming telemetry only. They do not automatically add a GDB watch, open the serial port, or start polling. If `telemetry_start_capture` returns `captureUsable:false`, the capture was **not** created 鈥?this is a rejection, not a success. The requested source/key has no recent data. Use the included `suggestedSeries` or call `telemetry_suggest_series` / `telemetry_list_series` to discover an active series, or ask the user to enable the relevant watch/stream first. Pass `allowEmptyCapture:true` only when explicitly waiting for future data.

Recommended concise prompt when trigger details are missing:

> I can collect an OctoLink telemetry dataset. Should I watch a stream variable or a GDB watch expression, what is the variable name, and do you want the last N seconds or a triggered capture?

If the user asks the agent to decide, prefer `stream` for high-rate real-time data, prefer `gdb` only for slow watched expressions already visible in OctoLink, omit `startCondition` to start on the first matching sample unless a threshold is obvious, and set `maxDurationMs` to 5000-10000 ms as a safety guard.

### Writing variables (destructive - requires confirm)

```
Tool: write_gdb_variable
Args: {
  "expression": "motorSpeed",
  "value": "1500",
  "confirm": true
}
```

**Workflow:**

1. **Prefer stopped-target writes when exact verification matters** - halt first if needed; OctoLink writes, reads back, then resumes execution automatically.
2. **Running-target writes are allowed for live tuning** - OctoLink sends `set var` asynchronously and returns `verified:false` because it cannot safely read back while the target is running.
3. **Verify through telemetry** - after a running write, call `telemetry_get_history` for the related stream/GDB watch instead of immediately forcing a halt.
4. **Do not add an extra manual continue after `write_gdb_variable`** - the tool already resumes after stopped-target writes. Use `target_continue` only when the user explicitly halted the target outside the write flow.

### Read memory

```
Tool: read_memory_u32
Args: { "address": "0x40021000" }

Tool: read_memory_bytes
Args: { "address": "0x20000000", "length": 64 }
```

Returns hex string, raw bytes array, and u32 little-endian interpretation. Max 1024 bytes per call.

---

## Target Control Safety

All target control tools require `confirm: true`. **Never call reset/continue without explicit user intent.**

| Tool | Effect | Confirm |
|------|--------|:-------:|
| `target_continue` | Resume execution | **Yes** |
| `target_interrupt` | Halt running target | No |
| `target_reset_halt` | Reset MCU, halt immediately | **Yes** |
| `target_reset_run` | Reset MCU, let it run | **Yes** |
| `target_step` | Step into (one source line) | **Yes** |
| `target_next` | Step over (one source line) | **Yes** |
| `target_finish` | Step out of current function | **Yes** |

**Before calling any confirm-required target control tool, tell the user:**
- What the tool will do
- What the expected side effect is (e.g., "this will reset the MCU and halt at the reset vector")

### Target control return semantics

Target-control tools use both GDB/MI response and OctoLink's observed `target_state`. If GDB doesn't emit a fresh `*stopped` event before timeout, OctoLink may return `success: true` with a `warning` when the state already reached the expected value.

Prefer structured fields over raw GDB text:

- `success`
- `target_state_before` / `target_state_after`
- `warning`
- `next_suggestion`

If a command times out and the state is still not coherent, do not keep issuing GDB commands in a loop. Use:

1. `gdb_health_check` - quick probe to see whether GDB/MI still responds.
2. `gdb_recover_session` with `{ "confirm": true }` - restarts GDB with OctoLink's last GDB/ELF config and reconnects to OpenOCD.
3. `debug_get_session_state` - refresh the Inspector state after recovery.

Step tools (`target_step`, `target_next`, `target_finish`) return after GDB accepts the motion command. The final source location arrives through the next async stop event; call `debug_get_session_state` after the target stops.

---

## Breakpoint Workflow

```
# 1. List existing breakpoints
Tool: gdb_list_breakpoints

# 2. Set a breakpoint (confirm required)
Tool: gdb_set_breakpoint
Args: { "location": "main.c:42", "confirm": true }

# Optional: temporary breakpoint (auto-deletes on hit)
Args: { "location": "main.c:42", "temporary": true, "confirm": true }

# 3. Continue to breakpoint
Tool: target_continue
Args: { "confirm": true }

# 4. Read variables after hit
Tool: gdb_evaluate_expressions
Args: { "expressions": ["localVar", "ptr->field"] }

# 5. Clean up when done
Tool: gdb_delete_breakpoint
Args: { "breakpointId": 1, "confirm": true }
```

Use `temporary: true` for one-shot breakpoints. Always clean up breakpoints when no longer needed.

---

## Memory Workflow

```
# Read a peripheral register
Tool: read_memory_u32
Args: { "address": "0x40021000" }

# Dump a memory block (max 1024 bytes)
Tool: read_memory_bytes
Args: { "address": "0x20000000", "length": 128 }
```

Returns hex string, raw bytes array, and u32 little-endian interpretation. For larger dumps, make multiple calls with incremented addresses.

---

## Serial Workflow

### Lifecycle

1. **Check status** - `get_serial_status`
2. **Open port** - `serial_open` (requires `confirm: true`, closes any previous port first)
3. **Write data** - `serial_write` (requires `confirm: true`, supports text/hex encoding, optional CR/LF append)
4. **Close port** - `serial_close` (requires `confirm: true`)

```
# Check what's open
Tool: get_serial_status

# Open a port
Tool: serial_open
Args: { "portName": "COM3", "baudRate": 115200, "confirm": true }

# Send data
Tool: serial_write
Args: { "data": "help\r\n", "confirm": true }

# Close when done
Tool: serial_close
Args: { "confirm": true }
```

### Safety rules

- Always check `get_serial_status` before opening - avoid opening a port that's already in use.
- Do NOT spam the serial console with rapid writes. Batch data when possible.
- Close the port when the serial task is complete.
- Use `serial_clear_counters` (requires `confirm: true`) to reset RX/TX counters and clear the last serial error.

### OctoLite telemetry protocol

OctoLink supports two serial frame protocols, both decoded by the `octolink_binary` stream decoder:

| Protocol | Frame format |
|----------|-------------|
| Legacy | `A5 5A | version | var_id:u16le | value_type | shape_len | shape... | payload_len:u16le | payload | crc` |
| OctoLite V1 | `AA | var_id:u8 | type_shape:u8 | seq:u8 | payload_len:u8 | payload | crc` |

Set the serial stream decoder to `octolink_binary` in the OctoLink GUI to receive both protocols. OctoLite V1 is lighter and suited for high-rate scalar, array, matrix, and bit-matrix telemetry.

---

## Complete Example: Inspect RTOS Task State

```
# 1. Verify everything is ready
Tool: get_debug_context

# 2. Safe-read task control block fields
Tool: gdb_read_expressions_safe
Args: {
  "expressions": [
    "pxCurrentTCB->pcTaskName",
    "pxCurrentTCB->uxPriority",
    "pxCurrentTCB->pxTopOfStack",
    "uxTopReadyPriority"
  ],
  "haltIfRunning": true,
  "resumeAfterRead": true
}
```

## Complete Example: Set Breakpoint, Hit, Inspect, Resume

```
# 1. Set breakpoint
Tool: gdb_set_breakpoint
Args: { "location": "main.c:42", "confirm": true }

# 2. Continue execution
Tool: target_continue
Args: { "confirm": true }

# 3. (After breakpoint hit) Read local variables
Tool: gdb_evaluate_expressions
Args: { "expressions": ["localVar", "ptr->field"] }

# 4. Resume when inspection is done
Tool: target_continue
Args: { "confirm": true }
```

## Complete Example: Read and Modify a Variable

```
# 1. Halt the target
Tool: target_interrupt

# 2. Read current value
Tool: gdb_evaluate_expression
Args: { "expression": "motorSpeed" }

# 3. Write new value (confirm required)
Tool: write_gdb_variable
Args: { "expression": "motorSpeed", "value": "2000", "confirm": true }

# 4. Verify the readback in the response (verified: true, readBackValue)

# 5. Resume only when user confirms it's safe
Tool: target_continue
Args: { "confirm": true }
```

---

## Advanced Debug Workflow Tools

These tools provide higher-level agent-friendly workflows for common debug patterns.

### Write and verify a variable

```
Tool: debug_write_and_verify
Args: { "expression": "motorSpeed", "value": "2000", "confirm": true }
```

Response includes `haltedByTool` (true if this call halted a running target), `verified` (true if read-back matched), `readBackValue`, target state before/after, and raw MI responses. If verification fails, `nextSuggestion` is included.

### Capture fault context

```
Tool: debug_capture_fault_context
```

Read-only. Captures GDB status, key registers (`$pc`, `$sp`, `$lr`, `$xpsr`, `$ipsr`), Cortex-M fault registers (`CFSR`, `HFSR`, `MMFAR`, `BFAR`, `DFSR` via memory), current frame, and backtrace. Best-effort 鈥?individual read failures are reported in `warnings` but do not abort the tool. Use this before recovery to understand why the target faulted.

### Run to breakpoint

```
Tool: debug_run_to_breakpoint
Args: { "location": "main.c:42", "temporary": true, "confirm": true }
```

Sets a breakpoint, continues execution, and waits for the target to stop. Returns `breakpointHit` (true if stopped at the breakpoint), `stoppedReason`, `breakpointId`, and elapsed time. If timeout, `nextSuggestion` recommends `target_interrupt` or `debug_get_session_state`.

### Step until expression changes

```
Tool: debug_step_until
Args: { "expression": "state", "stepMode": "over", "maxSteps": 50, "confirm": true }
```

Repeatedly steps over and evaluates `state` after each step. Stops when the value changes. Returns `steps` taken, `initialValue`, `finalValue`, per-step `samples`, and `stoppedReason` (`expressionChanged`, `conditionTrue`, `maxStepsReached`, `stepTimeout`, etc.).

With a condition:

```
Tool: debug_step_until
Args: { "expression": "counter", "condition": "counter > 100", "stepMode": "into", "confirm": true }
```

### Watch expression until changed

```
Tool: debug_watch_expression_until_changed
Args: { "expression": "sensorValue", "intervalMs": 50, "timeoutMs": 5000, "haltIfRunning": true, "confirm": true }
```

Polls `sensorValue` every 50ms until it changes or 5s timeout. If `haltIfRunning` true and target is running, interrupts first. Returns `initialValue`, `finalValue`, `changed` flag, and `samples` array.

### Recover session

```
Tool: debug_recover_session
Args: { "confirm": true }
```

Agent-facing alias for `gdb_recover_session`. Restarts GDB with the last OctoLink config and reconnects to OpenOCD. Returns `success`, `gdbRestarted`, `reconnected`, status before/after, and step-by-step log.

---

## Agent Debug Inspector

OctoLink includes a separate **Agent Debug Inspector** window that provides a real-time view of the debug session: target state, breakpoints, watched variables, source code with current-line highlight, and an agent timeline showing every MCP tool call.

### Opening the Inspector

- **Manual:** In the Agent Bridge side panel, click **Open Inspector**.
- **Auto-open:** Toggle **Auto-open Inspector on MCP start** in the Agent Bridge side panel. When enabled, the Inspector window opens automatically whenever the MCP server starts.
- **MCP default:** The MCP server does **not** auto-open the Inspector. It runs headless unless you enable the toggle or open it manually.

### Inspector Panels

| Panel | Shows |
|-------|-------|
| **Target Status** | GDB connection, target running/stopped, PC, current function, file:line, stop reason |
| **Breakpoints** | All breakpoints with ID, type, enabled/disabled, location, condition, hit count |
| **Variables** | Watched variable expressions, last read values, errors |
| **Source Code** | Read-only source viewer with line numbers, current-line highlight, breakpoint markers, and search |
| **Call Stack / Registers** | Raw GDB/MI stack and register views for quick IDE-style inspection |
| **Locals** | Stack variables via `-stack-list-variables --simple-values` with Refresh button; target must be halted |
| **Disassembly** | Mixed source+asm around the current location via `-data-disassemble`; requires a stopped target with a known file:line |
| **Debug Console** | Lightweight command panel for read-oriented GDB/MI queries or expression evaluation |
| **Session Report** | Markdown report covering target state, breakpoints, watched variables, and recent timeline |
| **Agent Timeline** | Chronological log of every MCP tool call, target event, and breakpoint hit with success/fail indicators |

The **Debug Console** is for quick inspection, not session setup. It accepts GDB/MI commands that start with `-` and plain expressions for evaluation. It blocks session-destructive commands (`-gdb-exit`, `-exec-run`, `-exec-start`, `-exec-arguments`, `-gdb-set`). Use dedicated OctoLink controls or MCP tools for continue, step, reset, breakpoints, and recovery.

### Inspector Does NOT Affect the Main Window

The Inspector is a **read-only observation window**. It does not interfere with the main OctoLink workspace, TopBar, file operations, or window controls. If the Inspector window becomes unresponsive or shows an error, simply close it 鈥?the main window and MCP server continue working normally.

### Source Root for Path Mapping

The Inspector resolves source file paths relative to a **source root** directory. If source files are not found, set the source root in the Inspector's code panel header to the root of your firmware project's source tree (e.g., `C:\Projects\my-firmware`).

### Debug Inspector MCP Tools

High-level MCP tools for agent-driven debugging with the Inspector:

| Tool | Confirm | Description |
|------|:-------:|-------------|
| `debug_get_session_state` | No | Returns a snapshot of the Inspector state: target status, breakpoints, recent variables, timeline summary (last 20 events), current source location, and **recommendations** for next action. |
| `debug_diagnose_session` | No | Recommended first call in an active debug session. Returns target summary, recommendations, recent errors, timeline, breakpoint hits, and variable-change summary. |
| `debug_get_source_context` | No | Returns a source code snippet around a given file:line. If file/line are omitted, uses the Inspector's current execution position. Supports `contextLines` (default 20, max 100) and `sourceRoot` for path resolution. |
| `debug_open_source_location` | No | Sets the Inspector's current source location (file + line). The Inspector window will jump to this position on next refresh. Records a timeline event. |
| `debug_clear_session_timeline` | No | Clears the Inspector timeline log. Does not affect breakpoints or variable cache. |
| `debug_export_session_report` | No | Exports the current Inspector session as Markdown for debug review, handoff, or issue reports. |
| `debug_get_session_timeline` | No | Returns timeline events with optional filters (`limit`, `eventType`, `toolName`, `success`) for MCP calls, target actions, failures, and breakpoint hits. |
| `debug_get_breakpoint_hits` | No | Returns breakpoint-hit timeline events plus current breakpoint hit counts. |
| `debug_get_variable_history` | No | Returns recent Inspector variable samples for trend review. Supports optional `expression` filter and `limit`. |
| `debug_capture_fault_context` | No | Read-only fault snapshot: GDB status, registers, Cortex-M fault registers, frame, backtrace. Best-effort. |
| `debug_write_and_verify` | **Yes** | Write with auto halt, read-back verify, structured response. |
| `debug_run_to_breakpoint` | **Yes** | Set breakpoint, continue, wait for hit. Returns hit status and stopped reason. |
| `debug_step_until` | **Yes** | Step until expression changes or condition is true. Returns per-step samples. |
| `debug_watch_expression_until_changed` | **Yes** | Poll expression until value changes. Optional halt-if-running. |
| `debug_collect_variable_dataset` | **Yes** | Collect a short multi-expression GDB dataset. Halts target by default; use telemetry tools for live-running data. |
| `debug_recover_session` | **Yes** | Agent-facing GDB session recovery (alias for `gdb_recover_session`). |

### Recommended Agent Debug Flow

When debugging with the Inspector, follow this sequence:

```
# 1. Get overall session state
Tool: debug_diagnose_session
Tool: debug_get_session_state

# 2. Set breakpoints or control execution
Tool: gdb_set_breakpoint    (confirm: true)
Tool: target_continue        (confirm: true)

# 3. After breakpoint hit, read source context
Tool: debug_get_source_context
Args: { "file": "main.c", "line": 42 }

# 4. Read variables at the breakpoint
Tool: gdb_evaluate_expressions
Args: { "expressions": ["localVar", "ptr->field"] }

# 5. Optionally navigate the Inspector to a different location
Tool: debug_open_source_location
Args: { "file": "drivers/sensor.c", "line": 128 }

# 6. Summarize findings and continue or resume
Tool: debug_get_breakpoint_hits
Tool: debug_get_session_timeline
Tool: debug_get_variable_history
Tool: debug_export_session_report
Tool: target_continue        (confirm: true)
```

**Key points:**
- `debug_get_session_state` is the recommended first call 鈥?it gives you target state, breakpoints, recent timeline, and **recommendations** in one shot.
- To read variables at runtime, the target must be halted. Use `target_interrupt` or let a breakpoint hit first. If the target is running, `gdb_read_expressions_safe` will auto-halt for you.
- OctoLink guards synchronous GDB/MI reads while the target is running, but breakpoint commands are allowed during execution. If a read reports that the target is running, do not retry it in a loop; pause first, wait for a breakpoint, or use telemetry history.
- `target_interrupt` behaves like an IDE pause button: it sends GDB/MI `exec-interrupt` first and waits for a fresh stop event. If it cannot confirm a stop, assume the remote GDB session may be wedged and use `gdb_recover_session` instead of repeatedly interrupting.
- `debug_get_source_context` uses the same path resolution as the Inspector's source root setting.
- For automated workflows, use `debug_run_to_breakpoint` (set + continue + wait in one call) and `debug_step_until` (step until condition) instead of manual multi-step sequences.
- For tuning datasets, prefer `telemetry_get_history` / `telemetry_start_capture` while the MCU is running. Use `debug_collect_variable_dataset` only when it is acceptable to halt the target for a bounded GDB sample window.
- After a fault, call `debug_capture_fault_context` to snapshot registers and fault status before recovery.
- Use `debug_recover_session` when GDB becomes unresponsive 鈥?it restarts GDB and reconnects to OpenOCD.
- `debug_open_source_location` only changes the Inspector's view 鈥?it does not affect target execution.

---

## Response Field Reference

### `debug_get_session_state` 鈫?`recommendations`

The `recommendations` field is a compact object designed for agents to quickly decide the next action without parsing the full snapshot:

```json
{
  "recommendations": {
    "mcpReady": true,
    "gdbRunning": true,
    "targetState": "stopped",
    "currentLocation": "stopped 路 main:42",
    "recentErrors": [
      { "toolName": "gdb_evaluate_expression", "summary": "FAIL gdb_evaluate_expression foo: ..." }
    ],
    "recommendedAction": "Target is stopped. Read variables at current location or step through code.",
    "recommendedTool": "gdb_evaluate_expressions",
    "recommendedArgs": { "expressions": ["<your_variable>"] }
  }
}
```

| Field | Type | Description |
|-------|------|-------------|
| `mcpReady` | bool | `true` if GDB is running AND connected to target |
| `gdbRunning` | bool | Whether GDB process is alive |
| `targetState` | string | `"stopped"`, `"running"`, or `"unknown"` |
| `currentLocation` | string | Human-readable position summary |
| `recentErrors` | array | Last 5 failed timeline events (toolName + summary) |
| `recommendedAction` | string | Human-readable suggestion for the next step |
| `recommendedTool` | string | The tool name to call next |
| `recommendedArgs` | object | Suggested arguments for the recommended tool |

**Agent workflow:** Call `debug_get_session_state`, read `recommendations.recommendedTool` and `recommendations.recommendedAction` to decide what to do next. If `mcpReady` is `false`, check subsystem status first.

### `debug_get_source_context` metadata fields

In addition to the legacy fields (`file`, `line`, `startLine`, `endLine`, `totalLines`, `contextLines`, `truncated`, `snippet`), the response includes:

| Field | Type | Description |
|-------|------|-------------|
| `requestedFile` | string? | The `file` argument you passed (null if omitted) |
| `requestedLine` | int? | The `line` argument you passed (null if omitted) |
| `resolvedPath` | string | The resolved absolute path after normalization and source-root lookup |
| `lineCount` | int | Number of lines in the returned snippet |

Use `requestedFile`/`requestedLine` to confirm which query was served, and `resolvedPath` to reference the file in subsequent calls (e.g., `debug_open_source_location`).

### Timeline event format

Timeline events in `recentTimeline` have a consistent format:

```json
{
  "id": 42,
  "eventType": "expression_eval",
  "toolName": "gdb_evaluate_expression",
  "summary": "FAIL gdb_evaluate_expression myVar: target not halted",
  "success": false
}
```

**Error events** always include the tool name and a brief error in the `summary` field:
- `"FAIL gdb_set_breakpoint @ main.c:42: location not found"`
- `"FAIL gdb_evaluate_expression foo: GDB is not running"`
- `"FAIL write_gdb_variable x: target not halted"`
- `"FAIL memory_read @ 0x4000: ..."`
- `"FAIL gdb_send_mi_command: ..."`
- `"FAIL gdb_delete_breakpoint #3: ..."`

**Success events** have short descriptive summaries:
- `"Set breakpoint @ main.c:42"`
- `"Eval: myVar"` (with detail `"= 42"`)
- `"Write: x = 10"`
- `"Memory read @ 0x4000"`
- `"MI: -data-evaluate-expression ..."`

---

## FAQ: Inspector and Debug Tools

**Q: Does the Inspector window affect the main OctoLink window?**
A: No. The Inspector is a separate, read-only window. Closing or resizing it has no effect on the main workspace, TopBar, file operations, or window controls.

**Q: The Inspector window is blank or unresponsive. What do I do?**
A: Close it and reopen from the Agent Bridge side panel. The MCP server and main window are unaffected.

**Q: Source files show "not found" in the Inspector.**
A: Set the **source root** in the Inspector's code panel header to your firmware project's root directory. The Inspector resolves relative paths from the ELF's compilation directory against this root.

**Q: I can't read variables because the target is running.**
A: First, call `get_mcp_session_health` or `debug_get_session_state` — these now automatically probe the target and may detect it is actually halted (check `stateProbe.stateChanged`). If the probe confirms halted, proceed with variable reads directly. If the target is truly running: for trend/tuning/recent data, use `telemetry_find_series` and `telemetry_get_history` first. For exact GDB reads, use `gdb_read_expressions_safe` once; if it returns `readAttempted:false`, do not loop interrupts. Ask before `gdb_recover_session` or reset.

**Q: `debug_get_session_state` shows no breakpoints or variables.**
A: The Inspector only tracks breakpoints and variables set through MCP tools during the current session. Breakpoints set from the main OctoLink GUI are shown if they appear in GDB's breakpoint list (`gdb_list_breakpoints`).

**Q: How do I clear the timeline?**
A: Call `debug_clear_session_timeline`. This only clears the Inspector's event log 鈥?breakpoints and variable cache are preserved.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| GDB tools timeout repeatedly | GDB/OpenOCD connection wedged | Call `get_gdb_status` + `get_openocd_status`. Restart GDB from OctoLink. If persists, restart OpenOCD too. Then `target_reset_halt` and retry. |
| `write_gdb_variable` returns `verified:false` | Target is running, so OctoLink sent an async write without read-back | Verify the effect through `telemetry_get_history` or halt later for exact read-back. |
| `gdb_read_expressions_safe` returns `readAttempted: false` | Halt failed, target may be stuck or running-only workflow is safer | Do NOT retry reads or interrupts. Use returned `nextTool` / `nextArgs`, usually telemetry first. Ask user before recovery/reset. |
| `recommendedNextTool: "target_reset_halt"` | Reset is suggested as a last resort | Explain to user that this resets the MCU. Only proceed with user consent. |
| Proxy exits with code 2 | MCP server not running | Open Agent Bridge -> Start MCP Server. |
| `tools/call` returns "confirm required" | Missing `confirm` flag | Add `"confirm": true` to arguments. |
| GDB reports "target is running" on read | Target not halted (may be stale state) | First call `get_mcp_session_health` or `debug_get_session_state` to probe — the target may actually be halted. If probe confirms halted, proceed with reads. Otherwise use telemetry history for live data. Use `gdb_read_expressions_safe` only once when exact GDB values are mandatory. |
| Function call limitations | GDB `call` on target requires special setup | `gdb_evaluate_expression` evaluates expressions in GDB's context. Calling arbitrary target functions (e.g., HAL_Delay) may not work via MI commands. Use direct memory/variable reads instead. |
| `CLOSE_WAIT` on TCP socket | Client disconnected uncleanly | Restart the MCP server from Agent Bridge. |
| Proxy stale / no response | OctoLink GUI was restarted while proxy was running | Restart the proxy process. The proxy does not auto-reconnect. |
| Serial port already in use | Previous `serial_open` not closed | Call `serial_close` first, or check `get_serial_status`. |

---

## MCP Debug Log

The **MCP Debug Log** panel in the Agent Bridge UI provides a session/action audit trail. Every MCP tool call is logged with:

- Timestamp
- Tool name
- Log level (info / warn / error / ok)
- Action summary and detail

Use this to verify what the agent actually did during a session, diagnose unexpected behavior, and audit confirm-required operations.

---

## Safety Summary

1. **Check before you act.** `get_debug_context` or `get_gdb_status` first.
2. **Prefer safe reads.** `gdb_read_expressions_safe` with `haltIfRunning: true` when unsure.
3. **Do not write without reason.** Only `write_gdb_variable` when explicitly asked. Always explain side effects.
4. **Batch efficiently.** `gdb_evaluate_expressions` (up to 256) instead of looping single calls.
5. **Respect the lock.** All GDB tools serialize internally. Do not parallelize GDB commands from the client.
6. **Serial lifecycle.** Open -> write -> close. Don't leave ports dangling.
7. **Breakpoint hygiene.** Use `temporary: true` for one-shot. Clean up when done.
8. **Confirm gate.** Always pass `confirm: true` for state-changing tools. Always tell the user first.
9. **Resume carefully.** Only `target_continue` after the user confirms the target is ready.
