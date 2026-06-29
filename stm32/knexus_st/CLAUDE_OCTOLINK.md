# OctoLink MCP Integration

This project is configured to use OctoLink MCP for embedded target debugging.

## Quick Start

1. Start the **OctoLink GUI** application.
2. Open **Agent Bridge** (sidebar icon) and click **Start MCP Server**.
3. The MCP server listens on `127.0.0.1:48731`.
4. The proxy binary is at: `//?/D:/User/Project/OctoLink/ver_000/octolink/src-tauri/target/debug/resources/octolink-mcp-proxy.exe`

## Using with Claude Code

Claude Code is already configured via `.mcp.json` — the MCP server
`octolink` should appear in your available tools automatically.

If you want to start Claude with the standalone config instead, run:

````powershell
claude --mcp-config .\claude-octolink-mcp.json
````

### Recommended Prompts

- "Read the current value of `myVariable` using OctoLink"
- "Set a breakpoint at `main.c:42` and continue to it"
- "Dump 64 bytes starting at `0x20000000`"
- "Read RTOS task states safely"
- "Open serial port COM3 at 115200 baud"

### Safety Rules

- Read-only tools (`gdb_evaluate_expression`, `read_memory_u32`, etc.) require no confirmation.
- Write/control tools (`write_gdb_variable`, `target_continue`, `serial_write`, etc.) require `"confirm": true`.
- Always check `get_debug_context` before issuing commands.

## Skill Reference

The full OctoLink MCP skill is available at:

- `.claude/skills/octolink-mcp/SKILL.md` (installed when `installSkill` is enabled)
- `docs/octolink-mcp-skill.md` (human-readable copy)
