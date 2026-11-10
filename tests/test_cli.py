from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from swarmlink.cli import main


class CliTests(unittest.TestCase):
    def test_simulate_then_inspect(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "mission.jsonl"
            self.assertEqual(main(["simulate", "--nodes", "4", "--duration", "1", "--step-s", "0.5", "--out", str(output)]), 0)
            self.assertTrue(output.exists())
            self.assertEqual(main(["inspect", str(output)]), 0)


if __name__ == "__main__":
    unittest.main()
