"""Streamable-HTTP MCP client for the onav server.

Each call opens a fresh session — cheap on localhost and survives server restarts
between turns. Per-tool timeouts prevent blocking on long navigation actions.
"""
from __future__ import annotations

import json
from datetime import timedelta

from mcp import ClientSession
from mcp.client.streamable_http import streamablehttp_client

TOOL_TIMEOUTS = {
    "go_to_poi": 620,
    "run_mission": 3610,
    "dock": 310,
    "dock_local": 310,
    "undock": 130,
    "stop": 15,
    "kill": 15,
    "sync_data": 35,
    "get_state": 35,
}
DEFAULT_TIMEOUT = 60


class OnavMcp:
    def __init__(self, url: str) -> None:
        self.url = url

    async def list_tools(self) -> list[dict]:
        """Tool specs as {name, description, parameters} dicts."""
        async with streamablehttp_client(self.url) as (read, write, _):
            async with ClientSession(read, write) as session:
                await session.initialize()
                result = await session.list_tools()
        return [
            {
                "name": t.name,
                "description": t.description or "",
                "parameters": t.inputSchema or {"type": "object", "properties": {}},
            }
            for t in result.tools
        ]

    async def call_tool(self, name: str, args: dict, timeout_s: float | None = None) -> dict:
        t = timeout_s or TOOL_TIMEOUTS.get(name, DEFAULT_TIMEOUT)
        async with streamablehttp_client(self.url) as (read, write, _):
            async with ClientSession(read, write) as session:
                await session.initialize()
                result = await session.call_tool(
                    name, args, read_timeout_seconds=timedelta(seconds=t)
                )
        return parse_result(result)


def parse_result(result) -> dict:
    """Flatten a CallToolResult into a plain dict for the model."""
    text = "\n".join(c.text for c in result.content if getattr(c, "text", None))
    if result.isError:
        return {"error": text or "tool call failed"}
    structured = getattr(result, "structuredContent", None)
    if isinstance(structured, dict):
        if set(structured) == {"result"}:  # FastMCP wraps non-dict returns
            inner = structured["result"]
            return inner if isinstance(inner, dict) else {"result": inner}
        return structured
    try:
        data = json.loads(text)
        return data if isinstance(data, dict) else {"result": data}
    except (json.JSONDecodeError, TypeError):
        return {"reply": text} if text else {}


def to_ollama_tool(spec: dict) -> dict:
    return {
        "type": "function",
        "function": {
            "name": spec["name"],
            "description": spec["description"],
            "parameters": spec["parameters"],
        },
    }
