"""Record the actual native IR-SIM window (Windows GDI), not an animation."""

import hashlib

# Only the bundled ffmpeg executable is launched, with argument lists, no shell.
import subprocess  # nosec B404
import sys
import time
from pathlib import Path

import imageio_ffmpeg

from irsim_mcp.runtime import WINDOW_TITLE


class WindowRecording:
    """Own an ffmpeg recorder and finalize the MP4 before the window closes."""

    def __init__(self, path, window_title=WINDOW_TITLE):
        if sys.platform != "win32":
            raise RuntimeError("--record-window currently requires Windows gdigrab")
        self.path = Path(path)
        self.window_title = window_title
        self.log = self.path.with_suffix(".ffmpeg.log").open("wb")
        # The model never supplies executable names or command arguments.
        self.process = subprocess.Popen(  # nosec B603
            [
                imageio_ffmpeg.get_ffmpeg_exe(),
                "-y",
                "-hide_banner",
                "-f",
                "gdigrab",
                "-framerate",
                "10",
                "-draw_mouse",
                "0",
                "-i",
                f"title={self.window_title}",
                "-vf",
                "pad=ceil(iw/2)*2:ceil(ih/2)*2",
                "-c:v",
                "libx264",
                "-preset",
                "veryfast",
                "-crf",
                "25",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(self.path),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=self.log,
            creationflags=subprocess.CREATE_NO_WINDOW,
        )
        time.sleep(1)
        if self.process.poll() is not None:
            self.log.close()
            raise RuntimeError(
                f"window capture failed; inspect {self.path.with_suffix('.ffmpeg.log')}"
            )

    def close(self):
        if not self.log.closed:
            try:
                if self.process.poll() is None:
                    self.process.communicate(b"q", timeout=15)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.communicate()
                raise RuntimeError("recorder did not finalize") from None
            finally:
                self.log.close()
        if self.process.returncode != 0 or not self.path.is_file():
            raise RuntimeError("window recorder failed")
        # Fixed decoder arguments; the only input is our local recorded video.
        verified = subprocess.run(  # nosec B603
            [
                imageio_ffmpeg.get_ffmpeg_exe(),
                "-v",
                "error",
                "-i",
                str(self.path),
                "-f",
                "null",
                "-",
            ],
            capture_output=True,
            creationflags=subprocess.CREATE_NO_WINDOW,
            check=False,
        )
        if verified.returncode != 0:
            raise RuntimeError("recorded video failed full decoding")
        poster = self.path.with_suffix(".png")
        subprocess.run(  # nosec B603
            [
                imageio_ffmpeg.get_ffmpeg_exe(),
                "-y",
                "-v",
                "error",
                "-sseof",
                "-1",
                "-i",
                str(self.path),
                "-frames:v",
                "1",
                "-update",
                "1",
                str(poster),
            ],
            check=True,
            creationflags=subprocess.CREATE_NO_WINDOW,
        )
        return {
            "path": str(self.path.resolve()),
            "capture": "gdigrab",
            "window_title": self.window_title,
            "bytes": self.path.stat().st_size,
            "sha256": hashlib.sha256(self.path.read_bytes()).hexdigest(),
            "full_decode_verified": True,
            "poster": str(poster.resolve()),
        }
