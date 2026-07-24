# onav-console

Natural language console for OutdoorNav. Type plain English; the local Ollama model drives the robot.

The MCP server runs in the ROS 2 dev container (same machine as the sim or robot). The console runs natively on the operator's machine (Jetson, laptop, or robot itself) — it just needs Ollama and a network path to the server.

## Components

| File | Runs on | What it does |
|------|---------|--------------|
| `server.py` | dev container | FastMCP node — ROS 2 tools exposed over HTTP |
| `console.py` | operator machine | Ollama REPL — sends commands, shows results |

## Quickstart

**1. Add `mcp` and `fastmcp` to the dev image** (once, then rebuild):

```bash
# In onav-lab/docker/Dockerfile, after the existing pip install line:
RUN pip install --no-cache-dir --break-system-packages mcp fastmcp
```

**2. Start the server** (in the dev container, port exposed to host):

```bash
cd ~/onav_ws_amd64/onav-lab/docker
docker compose --env-file sim.env run --rm -p 8091:8091 dev \
  python mini-projects/onav-console/server.py
```

**3. Set up the console venv** (once, on the operator machine):

```bash
bash ~/onav-jetson/onav-lab/mini-projects/onav-console/setup.sh
```

**4. Run the console**:

```bash
# Sim on UM890 (10.0.0.35):
ONAV_TARGET=mock MCP_URL=http://10.0.0.35:8091/mcp \
  bash ~/onav-jetson/onav-lab/mini-projects/onav-console/run.sh

# Real robot via SSH tunnel:
ssh -L 8091:127.0.0.1:8091 ssh-mamp.nathanvanheyst.space
ONAV_TARGET=real MCP_URL=http://127.0.0.1:8091/mcp \
  bash ~/onav-jetson/onav-lab/mini-projects/onav-console/run.sh
```

## Environment variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ONAV_TARGET` | `mock` | `mock` or `real` — real requires typed confirmation for motion |
| `MCP_URL` | `http://127.0.0.1:8091/mcp` | URL of the MCP server |
| `OLLAMA_HOST` | `http://localhost:11434` | Ollama instance URL |
| `OLLAMA_MODEL` | `llama3.1:8b` | Model to use |
| `ONAV_NAMESPACE` | *(auto-detect)* | Robot namespace; skip auto-detect when set |

## Console commands

```
/tools    list available tools
/sync     refresh maps, missions, POIs, and docks from the robot
/clear    forget conversation history
/help     this message
/quit     exit
Ctrl-C    cancel running command and stop the robot
```

## Demo scenarios

1. `what's the battery level?` → `get_state`
2. `go to the charging station` → `go_to_poi`
3. `run the east patrol then dock at base` → `run_mission` → `dock`
4. `is anything running?` → `get_state`
5. `stop the robot` → `stop`
