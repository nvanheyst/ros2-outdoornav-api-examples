"""onav-console: type natural language, the local model drives the robot.

A single loop around Ollama's native tool calling, pointed at the OutdoorNav
MCP server. The model picks tools, the console dispatches them (with a typed-yes
gate on the real robot), and results go back to the model until it answers in text.

Requires: Ollama running locally, onav MCP server reachable at $MCP_URL.

Usage:
  bash run.sh
  ONAV_TARGET=real MCP_URL=http://<robot-host>:8091/mcp bash run.sh
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import sys
import time
from pathlib import Path

import ollama
from rich.console import Console
from rich.markup import escape

from mcp_client import OnavMcp, to_ollama_tool
from safety import needs_confirmation
from transcript import Transcript

logger = logging.getLogger(__name__)

MAX_TOOL_ROUNDS = 8
HISTORY_LIMIT = 24

FALLBACK_SYSTEM = """\
You are onav-console, the operator console for an OutdoorNav outdoor robot.
Call sync_data to load available maps, missions, POIs, and docks, then help
the operator navigate and run missions. Answer in one or two sentences.
"""

HELP = """\
[b]Commands[/]
  /help           this message
  /tools /skills  list the available tools
  /sync           refresh maps, missions, POIs, and docks from the robot
  /clear          forget the conversation so far
  /quit /exit     leave

[b]Ctrl-C[/] during a running command cancels it and stops the robot.

[b]Try asking[/]
  what's the battery level?
  go to the charging station
  run the east perimeter patrol
  run the patrol then dock at base
  is the robot idle?
"""


def _build_system(data: dict, ns: str = "") -> str:
    maps = [m["name"] for m in data.get("maps", [])]
    missions = [m["name"] for m in data.get("missions", [])]
    pois = [p["name"] for p in data.get("pois", [])]
    docks = [d["name"] for d in data.get("docks", [])]

    lines = [
        "You are onav-console, the operator console for an OutdoorNav outdoor robot.",
        "",
    ]
    if ns:
        lines.append(f"Robot namespace: {ns}")
    if maps:
        lines.append(f"Maps: {', '.join(maps)}")
    if missions:
        lines.append(f"Missions: {', '.join(missions)}")
    if pois:
        lines.append(f"POIs: {', '.join(pois)}")
    if docks:
        lines.append(f"Docks: {', '.join(docks)}")
    lines += [
        "",
        "Rules:",
        "- Use get_state to check battery, position, and current activity.",
        "- go_to_poi and run_mission need a map loaded; dock requires map-based approach.",
        "- dock_local is for when the robot is already facing the dock.",
        "- undock requires the dock name the robot is currently at.",
        "- kill cancels the active goal and stops the robot immediately.",
        "- Report results honestly; never claim success when a tool returned an error.",
        "- Answer in one or two sentences.",
    ]
    return "\n".join(lines)


def _field(obj, key, default=None):
    if isinstance(obj, dict):
        return obj.get(key, default)
    return getattr(obj, key, default)


def _args_label(args: dict) -> str:
    parts = []
    for k, v in list(args.items())[:4]:
        s = str(v)
        parts.append(f"{k}={s[:21] + '…' if len(s) > 24 else s}")
    return ", ".join(parts)


def _result_label(result: dict) -> str:
    err = result.get("error")
    if err is not None:
        return f"[red]{escape(str(err)[:80])}[/]"
    for k, v in result.items():
        if v is None:
            continue
        s = str(v)
        if s:
            return escape(f"{k}={s[:77]}…" if len(s) > 80 else f"{k}={s}")
    return ""


def _textual_tool_call(content: str, names: set[str]) -> dict | None:
    """llama3.1 sometimes emits tool calls as plain JSON; recover them."""
    try:
        data = json.loads(content)
    except json.JSONDecodeError:
        return None
    if isinstance(data, dict) and data.get("name") in names:
        args = data.get("parameters") or data.get("arguments") or {}
        return {"function": {"name": data["name"], "arguments": args}}
    return None


def _model_error(e: Exception, host: str) -> str:
    name = type(e).__name__
    low = str(e).lower()
    if name == "ResponseError" and ("not found" in low or "pull" in low):
        return f"Ollama is up but the model is missing ({e}). Pull it: ollama pull <model>."
    if name in ("ConnectError", "ConnectionError", "ConnectionRefusedError",
                "TimeoutException", "ConnectTimeout", "ReadTimeout") or \
            "connect" in low or "refused" in low:
        return f"Can't reach Ollama at {host}. Is the ollama service running?"
    return f"Model call failed ({name}: {e})"


class OnavConsole:
    def __init__(self) -> None:
        self.out = Console()
        self.host = os.getenv("OLLAMA_HOST", "http://localhost:11434")
        self.model = os.getenv("OLLAMA_MODEL", "llama3.1:8b")
        self.mcp_url = os.getenv("MCP_URL", "http://127.0.0.1:8091/mcp")
        self.target = os.getenv("ONAV_TARGET", "mock")
        self.ollama = ollama.AsyncClient(host=self.host)
        self.mcp: OnavMcp | None = None
        self.specs: list[dict] = []
        self.messages: list[dict] = []
        self.transcript = Transcript()
        self._system = FALLBACK_SYSTEM
        self._synced_data: dict = {}
        self._robot_ns: str = ""

    async def connect(self) -> None:
        mcp = OnavMcp(self.mcp_url)
        try:
            self.specs = await mcp.list_tools()
            self.mcp = mcp
        except Exception as e:
            self.out.print(
                f"[red]onav MCP server unreachable at {self.mcp_url} "
                f"({type(e).__name__})[/]"
            )
            self.out.print("[yellow]  Is the server up? "
                           "docker compose run --rm -p 8091:8091 dev "
                           "python mini-projects/onav-console/server.py[/]")
            return

        await self._do_sync(quiet=True)
        self._print_banner()

    async def _do_sync(self, quiet: bool = False) -> None:
        if self.mcp is None:
            return
        try:
            data = await self.mcp.call_tool("sync_data", {})
            if "error" not in data:
                self._synced_data = data
                self._robot_ns = data.get("namespace", self._robot_ns)
                self._system = _build_system(data, self._robot_ns)
                if not quiet:
                    maps = len(data.get("maps", []))
                    missions = len(data.get("missions", []))
                    pois = len(data.get("pois", []))
                    self.out.print(f"[dim]synced: {maps} map(s), {missions} mission(s), {pois} POI(s)[/]")
            elif not quiet:
                self.out.print(f"[yellow]sync_data: {data['error']}[/]")
        except Exception as e:
            if not quiet:
                self.out.print(f"[yellow]sync failed: {type(e).__name__}: {e}[/]")

    def _print_banner(self) -> None:
        data = self._synced_data
        maps = data.get("maps", [])
        missions = data.get("missions", [])
        pois = data.get("pois", [])

        ns_part = f" · {self._robot_ns}" if self._robot_ns else ""
        counts = f"{len(maps)} map(s) · {len(missions)} mission(s) · {len(pois)} POI(s)"
        self.out.print(f"[b]onav-console[/]{ns_part} · target: "
                       f"[{'red' if self.target == 'real' else 'green'}]{self.target}[/]")
        self.out.print(f"[dim]{counts}[/]")

        if missions:
            hint = missions[0]["name"]
            self.out.print(f'[dim]Try: "run the {hint}" or "what\'s the battery level?"[/]')
        elif pois:
            hint = pois[0]["name"]
            self.out.print(f'[dim]Try: "go to {hint}" or "what\'s the battery level?"[/]')
        else:
            self.out.print('[dim]No missions or POIs yet — create them in the web UI, then /sync[/]')

        self.out.print(f"[dim]transcript: {self.transcript.path} · /help for commands[/]")

    async def _kill_out_of_band(self) -> None:
        try:
            result = await OnavMcp(self.mcp_url).call_tool("kill", {}, timeout_s=10)
            if result.get("error"):
                self.out.print(f"[red]kill: {result['error']}[/]")
        except Exception as e:
            self.out.print(f"[red]kill failed: {type(e).__name__}: {e}[/]")

    async def turn(self, text: str) -> None:
        self.transcript.write("user", text)
        self.messages.append({"role": "user", "content": text})
        tools = [to_ollama_tool(s) for s in self.specs]

        for _ in range(MAX_TOOL_ROUNDS):
            try:
                resp = await self.ollama.chat(
                    model=self.model,
                    messages=[{"role": "system", "content": self._system}, *self.messages],
                    tools=tools,
                    options={"temperature": 0.2},
                )
            except Exception as e:
                logger.error("model call failed: %s: %s", type(e).__name__, e)
                self._reply(_model_error(e, self.host))
                return

            msg = _field(resp, "message")
            calls = _field(msg, "tool_calls") or []
            if not calls:
                content = (_field(msg, "content") or "").strip()
                recovered = _textual_tool_call(content, {s["name"] for s in self.specs})
                if recovered is None:
                    self._reply(content or "(no reply)")
                    return
                calls = [recovered]

            self.messages.append({
                "role": "assistant",
                "content": _field(msg, "content") or "",
                "tool_calls": [self._plain_call(c) for c in calls],
            })
            for call in calls:
                fn = _field(call, "function")
                name = _field(fn, "name")
                args = _field(fn, "arguments") or {}
                result = await self._dispatch(name, args)
                self.transcript.write("tool", {"name": name, "args": args, "result": result})
                self.messages.append(
                    {"role": "tool", "name": name, "content": json.dumps(result, default=str)}
                )

        self._reply("(stopping: too many tool rounds in one turn)")

    @staticmethod
    def _plain_call(call) -> dict:
        fn = _field(call, "function")
        return {"function": {"name": _field(fn, "name"),
                             "arguments": _field(fn, "arguments") or {}}}

    def _reply(self, text: str) -> None:
        self.messages.append({"role": "assistant", "content": text})
        self._trim()
        self.transcript.write("reply", text)
        self.out.print(f"[b cyan]onav>[/] {escape(text)}")

    def _trim(self) -> None:
        while len(self.messages) > HISTORY_LIMIT:
            self.messages.pop(0)
        while self.messages and self.messages[0].get("role") != "user":
            self.messages.pop(0)

    async def _dispatch(self, name: str, args) -> dict:
        if isinstance(args, str):
            try:
                args = json.loads(args)
            except json.JSONDecodeError:
                return {"error": f"unparseable tool arguments: {args!r}"}
        args = dict(args)

        if needs_confirmation(name, self.target):
            self.out.print(
                f"[b yellow]about to move the REAL robot:[/] {name}({_args_label(args)})"
            )
            answer = await asyncio.to_thread(input, "confirm (y/yes to proceed)> ")
            if not answer.strip().lower().startswith("y"):
                self.out.print("[red]  cancelled[/]")
                return {"cancelled": "operator declined; do not retry"}

        self.out.print(f"  [yellow]->[/] [b]{name}[/]({escape(_args_label(args))})")
        t0 = time.monotonic()
        if self.mcp is not None:
            try:
                result = await self.mcp.call_tool(name, args)
            except Exception as e:
                logger.exception("tool %s failed", name)
                result = {"error": f"{type(e).__name__}: {e}"}
        else:
            result = {"error": "MCP server not connected"}
        elapsed_ms = int((time.monotonic() - t0) * 1000)
        self.out.print(f"  [green]<-[/] [b]{name}[/] [dim]{elapsed_ms}ms[/] {_result_label(result)}")
        return result

    async def repl(self) -> None:
        await self.connect()
        piped = not sys.stdin.isatty()
        while True:
            try:
                line = await asyncio.to_thread(input, "" if piped else "> ")
            except (EOFError, KeyboardInterrupt):
                break
            text = line.strip()
            if not text:
                continue
            if piped:
                self.out.print(f"> {escape(text)}")
            if text in ("/quit", "/exit"):
                break
            if text == "/help":
                self.out.print(HELP)
                continue
            if text in ("/tools", "/skills"):
                for s in self.specs:
                    desc = s["description"].strip().splitlines()
                    self.out.print(f"  [b]{s['name']}[/]: {escape(desc[0] if desc else '')}")
                continue
            if text == "/clear":
                self.messages.clear()
                self.out.print("[dim]history cleared[/]")
                continue
            if text == "/sync":
                await self._do_sync()
                continue
            try:
                await self.turn(text)
            except KeyboardInterrupt:
                self.out.print("\n[yellow]cancelling — stopping robot...[/]")
                await self._kill_out_of_band()
                self.out.print("[dim]stopped. ready for next command.[/]")

        self.transcript.write("system", "session_end")
        self.transcript.close()


def main() -> None:
    log_dir = Path.home() / ".onav-console"
    log_dir.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        filename=str(log_dir / "console.log"),
    )
    try:
        asyncio.run(OnavConsole().repl())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
