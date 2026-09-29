import unittest

from ros2_interface.interface import NavState
from tests.fakes import FakeRobot, make_locations, make_settings
from tools import build_registry
from tools.context import ToolContext


def registry(robot, **settings):
    return build_registry(ToolContext(make_settings(**settings), robot, make_locations()))


class TestRegistry(unittest.TestCase):
    def test_seven_whitelisted_tools_only(self):
        self.assertEqual(registry(FakeRobot()).names(), [
            "cancel_navigation", "get_locations", "get_navigation_status", "get_robot_pose",
            "get_robot_status", "navigate_to", "stop_robot"])

    def test_refuses_non_whitelisted_tools(self):
        reg = registry(FakeRobot())
        for name in ("execute_shell", "publish_cmd_vel", "modify_nav2", None):
            self.assertEqual(reg.execute(name, {})["error"], "tool_not_allowed")

    def test_rejects_bad_arguments(self):
        reg = registry(FakeRobot(), dry_run=True)
        self.assertEqual(reg.execute("navigate_to", {})["error"], "invalid_arguments")
        self.assertEqual(reg.execute("navigate_to", {"location": "accueil", "speed": 5})["error"],
                         "invalid_arguments")


class TestNavigation(unittest.TestCase):
    def test_dry_run_never_moves_robot(self):
        robot = FakeRobot()
        res = registry(robot, dry_run=True).execute("navigate_to", {"location": "administration"})
        self.assertTrue(res["ok"] and res["dry_run"])
        self.assertEqual(robot.calls, [])

    def test_unknown_destination_lists_available(self):
        res = registry(FakeRobot(), dry_run=False).execute("navigate_to", {"location": "cafétéria"})
        self.assertEqual(res["error"], "unknown_location")
        self.assertIn("administration", res["message"])

    def test_uncalibrated_destination_refused(self):
        robot = FakeRobot()
        res = registry(robot, dry_run=False).execute("navigate_to", {"location": "direction"})
        self.assertEqual(res["error"], "location_not_calibrated")
        self.assertEqual(robot.calls, [])

    def test_alias_and_article_resolution(self):
        robot = FakeRobot()
        res = registry(robot, dry_run=False).execute("navigate_to", {"location": "au labo"})
        self.assertTrue(res["ok"])
        self.assertEqual(robot.calls, [("navigate", "laboratoire")])

    def test_new_destination_replaces_current(self):
        robot = FakeRobot()
        reg = registry(robot, dry_run=False)
        reg.execute("navigate_to", {"location": "accueil"})
        res = reg.execute("navigate_to", {"location": "laboratoire"})
        self.assertEqual(res["replaced"], "accueil")
        self.assertEqual(robot.dest, "laboratoire")

    def test_emergency_stop_blocks_navigation(self):
        robot = FakeRobot()
        robot.safety_state = "EMERGENCY_STOP"
        res = registry(robot, dry_run=False).execute("navigate_to", {"location": "accueil"})
        self.assertEqual(res["error"], "emergency_stop_active")

    def test_robot_unreachable_message(self):
        robot = FakeRobot()
        robot.reachable = False
        res = registry(robot, dry_run=False).execute("navigate_to", {"location": "accueil"})
        self.assertEqual(res["message"],
                         "Je ne peux pas communiquer avec le système de navigation du robot actuellement.")
        self.assertEqual(robot.calls, [])

    def test_cancel_when_idle_is_not_an_error(self):
        res = registry(FakeRobot()).execute("cancel_navigation", {})
        self.assertTrue(res["ok"])
        self.assertFalse(res["canceled"])


class TestStop(unittest.TestCase):
    def test_already_stopped_no_estop(self):
        robot = FakeRobot()
        res = registry(robot).execute("stop_robot", {})
        self.assertTrue(res["ok"])
        self.assertNotIn(("estop",), robot.calls)

    def test_stop_while_navigating_cancels_and_estops_even_in_dry_run(self):
        robot = FakeRobot()
        robot.state, robot.dest, robot.moving = NavState.NAVIGATING, "accueil", True
        res = registry(robot, dry_run=True).execute("stop_robot", {})
        self.assertTrue(res["estop_confirmed"])
        self.assertIn(("cancel",), robot.calls)
        self.assertIn(("estop",), robot.calls)

    def test_unknown_motion_fails_safe_to_estop(self):
        robot = FakeRobot()
        robot.moving = None
        registry(robot).execute("stop_robot", {})
        self.assertIn(("estop",), robot.calls)


if __name__ == "__main__":
    unittest.main() 
