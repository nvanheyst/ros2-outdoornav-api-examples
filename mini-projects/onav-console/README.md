# onav-console

Natural language console for OutdoorNav. Type plain English; the local Ollama model drives the robot.

The MCP server runs in the ROS 2 dev container (same machine as the sim or robot). The console runs natively on the operator's machine — it just needs Ollama and a network path to the server.

## Components

| File | Runs on | What it does |
|------|---------|--------------|
| `server.py` | dev container | FastMCP node — ROS 2 tools exposed over HTTP |
| `console.py` | operator machine | Ollama REPL — sends commands, shows results |

## Prerequisites

**Operator machine** (where the console runs):
- [Ollama](https://ollama.com) installed and running: `ollama serve`
- Model pulled: `ollama pull llama3.1:8b`

**Dev container machine** (where the MCP server runs):
- OutdoorNav stack up and healthy (sim or real robot)
- Dev image built — `mcp` and `fastmcp` are already in `docker/Dockerfile`

## Quickstart

**1. Start the server** (in the dev container, port exposed to host):

```bash
cd docker/
docker compose --env-file sim.env run --rm -p 8091:8091 dev \
  python mini-projects/onav-console/server.py
```

**2. Set up the console venv** (once, on the operator machine):

```bash
bash mini-projects/onav-console/setup.sh
```

**3. Run the console**:

```bash
# Sim (no confirmation prompts for motion):
ONAV_TARGET=sim MCP_URL=http://<sim-host>:8091/mcp \
  bash mini-projects/onav-console/run.sh

# Real robot via SSH tunnel:
ssh -L 8091:127.0.0.1:8091 <robot-hostname>
ONAV_TARGET=real MCP_URL=http://127.0.0.1:8091/mcp \
  bash mini-projects/onav-console/run.sh
```

## Environment variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ONAV_TARGET` | `sim` | `sim` (no confirmation) or `real` (requires typed yes for motion) |
| `MCP_URL` | `http://127.0.0.1:8091/mcp` | URL of the MCP server |
| `OLLAMA_HOST` | `http://localhost:11434` | Ollama instance URL |
| `OLLAMA_MODEL` | `llama3.1:8b` | Model to use |
| `ONAV_NAMESPACE` | *(auto-detect)* | Robot namespace; skip auto-detect when set |

## Console commands

```
/tools /skills   list available tools
/sync            refresh maps, missions, POIs, and docks
/clear           forget conversation history
/help            this message
/quit /exit      exit
Ctrl-C           cancel running command and stop the robot
```

## Demo scenarios

1. `what's the battery level?` → `get_state`
2. `go to the charging station` → `go_to_poi`
3. `run the east patrol then dock at base` → `run_mission` → `dock`
4. `is anything running?` → `get_state`
5. `stop the robot` → `stop`
