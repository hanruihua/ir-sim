"""Run the local Gemma/MCP example; see README.md for prerequisites."""

import sys
from pathlib import Path

from irsim_mcp.gemma_demo import main

if __name__ == "__main__":
    sys.argv[1:1] = ["--world", str(Path(__file__).with_name("robot.yaml"))]
    main()
