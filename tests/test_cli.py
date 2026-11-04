"""The inspect-robots-b601 command and the checkout launcher; nothing here opens CAN or a model."""

import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from importlib.metadata import PackageNotFoundError
from pathlib import Path
from unittest.mock import patch

from inspect_robots import cli as inspect_cli

from inspect_robots_b601 import cli
from inspect_robots_b601.cli import AGENT_CONFIG
from inspect_robots_b601.embodiment import DEFAULT_PROFILE

ROOT = Path(__file__).resolve().parents[1]
READY = {
    "channel": "can3",
    "type": 280,
    "flags": ["UP"],
    "bitrate": 1_000_000,
    "ctrlmode": [],
    "can_state": "ERROR-ACTIVE",
}
HANDLERS = ("check", "read", "recover", "hold", "probe", "pose", "gripper", "agent")
CONFIRMED = {
    "recover": [],
    "hold": [],
    "probe": ["--joint", "0"],
    "pose": ["--stage", "joint3=0.1"],
    "gripper": [],
    "agent": [],
}


class CommandTests(unittest.TestCase):
    def setUp(self):
        self.calls = []
        for name in HANDLERS:
            patcher = patch.object(
                cli, name, side_effect=lambda args, name=name: self.calls.append(name) or 0
            )
            patcher.start()
            self.addCleanup(patcher.stop)

    def refused(self, *argv):
        with redirect_stderr(io.StringIO()) as stderr, self.assertRaises(SystemExit) as error:
            cli.main(list(argv))
        self.assertEqual(error.exception.code, 2)
        self.assertEqual(self.calls, [])
        return stderr.getvalue()

    def test_moving_or_rearming_requires_the_operator_confirmation(self):
        for name, extra in CONFIRMED.items():
            with self.subTest(name=name):
                self.assertIn("--confirm-supported-arm", self.refused(name, *extra))
                self.assertEqual(cli.main([name, *extra, "--confirm-supported-arm"]), 0)
                self.calls.clear()

    def test_read_only_commands_need_no_confirmation(self):
        for name in ("check", "read"):
            self.assertEqual(cli.main([name]), 0)
        self.assertEqual(self.calls, ["check", "read"])

    def test_motion_bounds_are_refused_before_anything_opens(self):
        for argv in (
            ["probe", "--joint", "7"],
            ["probe", "--joint", "0", "--delta", "0.2"],
            ["hold", "--hold-seconds", "11"],
            ["hold", "--max-deviation", "0.6"],
            ["pose", "--stage", "joint9=0.1"],
            ["pose", "--stage", "joint1=1.3"],
            ["gripper", "--torque-cap", "2"],
            ["agent", "--max-llm-calls", "31"],
            ["agent", "--voice"],
        ):
            with self.subTest(argv=argv):
                self.refused(*argv, "--confirm-supported-arm")

    def test_profile_and_state_defaults_follow_the_environment(self):
        with patch.dict(
            os.environ, {"B601_PROFILE": "/site/arm.json", "B601_STATE_DIR": "/site/state"}
        ):
            args = cli.build_parser().parse_args(["read"])
        self.assertEqual(
            (args.profile, args.state_dir), (Path("/site/arm.json"), Path("/site/state"))
        )
        with patch.dict(os.environ, {}, clear=True):
            args = cli.build_parser().parse_args(["pose", "--stage", "joint3=0.1"])
        self.assertEqual((args.profile, args.state_dir), (DEFAULT_PROFILE, Path(".hardware-state")))


class AgentCommandTests(unittest.TestCase):
    def test_agent_runs_the_b601_embodiment_with_every_motion_flag(self):
        with patch.object(cli, "_inspect", return_value=7) as run:
            self.assertEqual(
                cli.main(["agent", "--confirm-supported-arm", "--hold-seconds", "2"]), 7
            )
        args = inspect_cli.build_parser().parse_args(run.call_args.args[0])
        self.assertEqual((args.embodiment, args.policy, args.scorer), ("b601", "agent", "operator"))
        self.assertEqual(Path(args.config), AGENT_CONFIG.resolve())
        for flag in (
            "allow_motion=true",
            "activate_on_reset=true",
            "operator_supported_arm=true",
            "hold_seconds=2.0",
            f"profile={DEFAULT_PROFILE.resolve()}",
        ):
            self.assertIn(flag, args.embodiment_args)
        self.assertIsNone(args.max_action_delta)


class ReadOnlyCommandTests(unittest.TestCase):
    def check(self, info, motorbridge="0.5.6"):
        def version(name):
            if motorbridge is None:
                raise PackageNotFoundError(name)
            return motorbridge

        with tempfile.TemporaryDirectory() as directory:
            profile = Path(directory) / "robot.json"
            profile.write_text(json.dumps({"channel": "can3"}))
            with (
                patch("inspect_robots_b601.hardware.bus.interface_metadata", return_value=info),
                patch("importlib.metadata.version", version),
                redirect_stdout(io.StringIO()) as output,
            ):
                code = cli.main(["check", "--profile", str(profile)])
        return code, output.getvalue()

    def test_check_names_a_missing_motorbridge(self):
        code, output = self.check(READY, motorbridge=None)
        self.assertEqual(code, 1)
        self.assertIn(
            "motorbridge: not installed; python -m pip install 'inspect-robots-b601[motorbridge]'",
            output,
        )

    def test_check_names_what_to_fix(self):
        code, output = self.check(
            {
                "channel": "can3",
                "type": 280,
                "flags": [],
                "bitrate": 500_000,
                "ctrlmode": [],
                "can_state": "STOPPED",
            }
        )
        self.assertEqual(code, 1)
        self.assertIn(
            "can3: interface is down (sudo ip link set can3 up type can bitrate 1000000)", output
        )
        self.assertIn("bitrate is 500000", output)

    def test_check_accepts_a_ready_bus(self):
        code, output = self.check(
            {
                "channel": "can3",
                "type": 280,
                "flags": ["UP"],
                "bitrate": 1_000_000,
                "ctrlmode": [],
                "can_state": "ERROR-ACTIVE",
            }
        )
        self.assertEqual(code, 0, output)
        self.assertIn("can3: ready", output)


class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.script = self.root / "scripts/inspect-robots-b601"
        self.script.parent.mkdir()
        shutil.copyfile(ROOT / "scripts/inspect-robots-b601", self.script)
        self.calls = self.root / "calls.jsonl"

    def fake(self, name):
        executable = self.root / ".inspect-venv/bin" / name
        executable.parent.mkdir(parents=True, exist_ok=True)
        executable.write_text(
            f"#!{sys.executable}\nimport json, os, sys\n"
            f"with open({str(self.calls)!r}, 'a') as log:\n"
            "    log.write(json.dumps([os.getcwd(), os.environ.get('B601_STATE_DIR'), "
            "*sys.argv[1:]]) + '\\n')\n"
        )
        executable.chmod(0o755)

    def run_script(self, *args):
        return subprocess.run(["bash", str(self.script), *args], capture_output=True, text=True)

    def logged(self):
        return [json.loads(line) for line in self.calls.read_text().splitlines()]

    def test_help_needs_no_install(self):
        result = self.run_script("--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("setup", result.stdout)
        self.assertIn("preflight", result.stdout)
        self.assertIn("recover", result.stdout)

    def test_commands_need_setup_first(self):
        result = self.run_script("read")
        self.assertEqual(result.returncode, 1)
        self.assertIn("scripts/inspect-robots-b601 setup", result.stderr)

    def test_commands_run_from_the_checkout_with_env_loaded(self):
        self.fake("inspect-robots-b601")
        (self.root / ".env").write_text("B601_STATE_DIR=/site/state\n")
        result = self.run_script("pose", "--stage", "joint3=0.1 extra")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            self.logged(),
            [[str(self.root.resolve()), "/site/state", "pose", "--stage", "joint3=0.1 extra"]],
        )

    def test_setup_installs_the_checkout_with_its_extras(self):
        self.fake("python")
        result = self.run_script("setup")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(
            self.logged()[-1][-1].endswith("[motorbridge,agent,dev]"), self.logged()[-1]
        )
        self.assertEqual(self.run_script("setup", "--voice").returncode, 2)

    def test_preflight_runs_the_installed_preflight(self):
        self.fake("inspect-robots-b601-preflight")
        result = self.run_script("preflight", "--dry-run")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.logged()[-1][2:], ["--dry-run"])


if __name__ == "__main__":
    unittest.main()
