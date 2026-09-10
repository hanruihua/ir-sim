"""Executable checks for learning-page examples; run pytest docs/tests/."""

import re
from pathlib import Path

import irsim

DOCS = Path(__file__).resolve().parents[1]


def code_block(page, language):
    source = (DOCS / "source" / page).read_text()
    return re.search(rf"```{language}\n(.*?)\n```", source, re.DOTALL).group(1)


def test_kinematics_python_checkpoint():
    exec(code_block("get_started/kinematics.md", "python"), {})


def test_quick_start_reaches_goal(tmp_path):
    scene = tmp_path / "robot_world.yaml"
    scene.write_text(code_block("get_started/quick_start.md", "yaml"))
    env = irsim.make(str(scene), headless=True, seed=0)
    try:
        for _ in range(300):
            env.step()
            if env.done():
                break
        assert env.robot.arrive_flag
    finally:
        env.end(0)
