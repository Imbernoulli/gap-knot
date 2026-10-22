"""Every GaP authoring role must receive the selected knot task, including fresh generation."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from harness import selflearn, variants


class TestTaskWorkspaces(unittest.TestCase):
    def test_selected_brief_and_canonical_images(self):
        for task, brief, prefix in (("coin", "TASK.md", ""),
                                    ("overhand", "TASK_overhand.md", "overhand_"),
                                    ("overhand_tighten", "TASK_tighten.md", "overhand_")):
            with self.subTest(task=task), tempfile.TemporaryDirectory() as tmp, patch.object(variants, "KNOT", task):
                ws = selflearn.prepare_workspace(Path(tmp) / "agent")
                docs = selflearn.ROOT / "harness" / "docs"
                self.assertEqual((ws / "docs" / "TASK.md").read_bytes(), (docs / brief).read_bytes())
                expected = ("overhand_storyboard.jpg", "overhand_start_and_target.png") if prefix else ("video_storyboard.jpg", "start_and_target.png")
                for name, canonical in zip(expected, ("video_storyboard.jpg", "start_and_target.png")):
                    self.assertEqual((ws / "docs" / "img" / canonical).read_bytes(), (docs / "img" / name).read_bytes())
                self.assertEqual([p.name for p in selflearn.task_assets()[1]], list(expected))

    def test_fresh_overhand_generation_sends_correct_context_to_all_roles(self):
        seen = []

        def fake_agent(role, ws, prompt, *, images=(), **kwargs):
            seen.append((role, (ws / "docs" / "TASK.md").read_text(), [p.name for p in images]))
            if role == "coordinator":
                graph = ws / "graph"
                (graph / "workflow.json").write_text(json.dumps({"subgraphs": {"loop_sg": {"owner": "coordinator"}}}))
                (graph / "SEGMENTS.md").write_text("Make the overhand loop.")
            elif role == "skill_loop_sg":
                (ws / "graph" / "workflow.json").write_text(json.dumps({"subgraphs": {"loop_sg": {"owner": "skill"}}}))
                (ws / "graph" / "scripts" / "loop_sg_act.py").write_text("def run(ctx): return {}\n")
            return {"exit": 0, "wall_s": 0, "limited": False}

        with tempfile.TemporaryDirectory() as tmp, patch.object(variants, "KNOT", "overhand"), \
                patch.object(selflearn, "check", return_value=[]), patch.object(selflearn.codex_agent, "run", side_effect=fake_agent):
            final = selflearn.generate(Path(tmp))
            self.assertEqual(json.loads((final / "workflow.json").read_text())["subgraphs"]["loop_sg"]["owner"], "skill")
        self.assertEqual([row[0] for row in seen], ["coordinator", "skill_loop_sg"])
        for _, task, _ in seen:
            self.assertIn("Task: tie an overhand knot", task)
        self.assertEqual(seen[0][2], ["overhand_storyboard.jpg", "overhand_start_and_target.png"])
        self.assertEqual(seen[1][2], ["overhand_start_and_target.png"])
