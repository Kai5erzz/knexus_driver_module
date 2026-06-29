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
| `get_gdb_status` | GDB running, PID, ELF path, connection state |
| `get_openocd_status` | OpenOCD running, PID, port, target detection |
| `get_serial_status` | Serial open/closed, baud, RX/TX counters |

---

## Confirmation Convention

State-changing operations require `"confirm": true` in the arguments. Without it the tool returns an error.

| Tool | Requires `confirm` |
|------|:------------------:|
| `get_debug_context`, `get_gdb_status`, `get_openocd_status`, `get_serial_status`, `list_serial_ports`, `get_offline_symbol_status` | No |
| `gdb_evaluate_expression`, `gdb_evaluate_expressions`, `gdb_read_expressions_safe` | No |
| `read_memory_u32`, `read_memory_bytes` | No |
| `gdb_list_breakpoints` | No |
| `target_interrupt` | No (halting is safe) |
| `write_gdb_variable` | **Yes** |
| `gdb_set_breakpoint`, `gdb_delete_breakpoint` | **Yes** |
| `target_continue` | **Yes** |
| `target_step`, `target_next` | **Yes** |
| `target_reset_halt`, `target_reset_run` | **Yes** |
| `serial_open`, `serial_close`, `serial_write`, `serial_clear_counters` | **Yes** |
| `debug_get_session_state`, `debug_get_source_context`, `debug_open_source_location`, `debug_clear_session_timeline` | No |

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
| `recommendedNextTool` | `"target_reset_halt"` when halt failed early |
| `warnings` | Non-fatal halt/resume problems |

**If `readAttempted: false`:** Do NOT retry single-expression reads. The target could not be halted cleanly. Ask the user before attempting `target_reset_halt`.

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

1. **Halt the target first** - `write_gdb_variable` refuses to write while target is running. Use `target_interrupt` or `target_reset_halt`.
2. **Write with `confirm: true`** - the tool writes the value then reads it back.
3. **Verify** - check `verified: true/false` and `readBackValue` in the response.
4. **Resume only when appropriate** - call `target_continue` (with `confirm: true`) only after the user confirms it's safe.

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

If a command times out and the state is still not coherent, call `get_gdb_status` before trying another GDB command.

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
| **Agent Timeline** | Chronological log of every MCP tool call, target event, and breakpoint hit with success/fail indicators |

### Inspector Does NOT Affect the Main Window

The Inspector is a **read-only observation window**. It does not interfere with the main OctoLink workspace, TopBar, file operations, or window controls. If the Inspector window becomes unresponsive or shows an error, simply close it — the main window and MCP server continue working normally.

### Source Root for Path Mapping

The Inspector resolves source file paths relative to a **source root** directory. If source files are not found, set the source root in the Inspector's code panel header to the root of your firmware project's source tree (e.g., `C:\Projects\my-firmware`).

### Debug Inspector MCP Tools

Four high-level MCP tools are available for agent-driven debugging with the Inspector:

| Tool | Confirm | Description |
|------|:-------:|-------------|
| `debug_get_session_state` | No | Returns a snapshot of the Inspector state: target status, breakpoints, recent variables, timeline summary (last 20 events), current source location, and **recommendations** for next action. |
| `debug_get_source_context` | No | Returns a source code snippet around a given file:line. If file/line are omitted, uses the Inspector's current execution position. Supports `contextLines` (default 20, max 100) and `sourceRoot` for path resolution. |
| `debug_open_source_location` | No | Sets the Inspector's current source location (file + line). The Inspector window will jump to this position on next refresh. Records a timeline event. |
| `debug_clear_session_timeline` | No | Clears the Inspector timeline log. Does not affect breakpoints or variable cache. |

### Recommended Agent Debug Flow

When debugging with the Inspector, follow this sequence:

```
# 1. Get overall session state
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
Tool: target_continue        (confirm: true)
```

**Key points:**
- `debug_get_session_state` is the recommended first call — it gives you target state, breakpoints, recent timeline, and **recommendations** in one shot.
- To read variables at runtime, the target must be halted. Use `target_interrupt` or let a breakpoint hit first. If the target is running, `gdb_read_expressions_safe` will auto-halt for you.
- `debug_get_source_context` uses the same path resolution as the Inspector's source root setting.
- `debug_open_source_location` only changes the Inspector's view — it does not affect target execution.

---

## Response Field Reference

### `debug_get_session_state` → `recommendations`

The `recommendations` field is a compact object designed for agents to quickly decide the next action without parsing the full snapshot:

```json
{
  "recommendations": {
    "mcpReady": true,
    "gdbRunning": true,
    "targetState": "stopped",
    "currentLocation": "stopped · main:42",
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

**Q: I can't read variables — the target is running.**
A: Use `target_interrupt` (no confirm needed) to halt the target first. Or use `gdb_read_expressions_safe` with `haltIfRunning: true` — it will halt, read, and optionally resume automatically.

**Q: `debug_get_session_state` shows no breakpoints or variables.**
A: The Inspector only tracks breakpoints and variables set through MCP tools during the current session. Breakpoints set from the main OctoLink GUI are shown if they appear in GDB's breakpoint list (`gdb_list_breakpoints`).

**Q: How do I clear the timeline?**
A: Call `debug_clear_session_timeline`. This only clears the Inspector's event log — breakpoints and variable cache are preserved.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| GDB tools timeout repeatedly | GDB/OpenOCD connection wedged | Call `get_gdb_status` + `get_openocd_status`. Restart GDB from OctoLink. If persists, restart OpenOCD too. Then `target_reset_halt` and retry. |
| `write_gdb_variable` refuses to write | Target is running | Halt first with `target_interrupt` or `target_reset_halt`. |
| `gdb_read_expressions_safe` returns `readAttempted: false` | Halt failed, target may be stuck | Do NOT retry reads. Ask user, then try `target_reset_halt` with `confirm: true`. |
| `recommendedNextTool: "target_reset_halt"` | Halt failed early | Explain to user that this resets the MCU. Only proceed with user consent. |
| Proxy exits with code 2 | MCP server not running | Open Agent Bridge -> Start MCP Server. |
| `tools/call` returns "confirm required" | Missing `confirm` flag | Add `"confirm": true` to arguments. |
| GDB reports "target is running" on read | Target not halted | Use `gdb_read_expressions_safe` with `haltIfRunning: true`, or halt manually first. |
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

