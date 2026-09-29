import unittest

from assistant.assistant import MSG_TECHNICAL, Assistant
from assistant.conversation import Conversation
from assistant.knowledge import KnowledgeBase
from assistant.llm_client import LLMError, LLMReply
from ros2_interface.interface import NavSnapshot, NavState
from tests.fakes import FakeLLM, FakeRobot, call, make_locations, make_settings
from tools import build_registry
from tools.context import ToolContext


def build(llm, robot=None, dry_run=False):
    robot = robot or FakeRobot()
    locations = make_locations()
    settings = make_settings(dry_run=dry_run)
    registry = build_registry(ToolContext(settings, robot, locations))
    return Assistant(settings, llm, registry, Conversation(6), KnowledgeBase({}), locations), robot


class TestAssistant(unittest.TestCase):
    def test_stop_uses_fast_path_without_llm(self):
        assistant, robot = build(FakeLLM())   # FakeLLM sans réponse : lèverait si appelé
        assistant.handle("Arrête !")
        self.assertIn(("estop",), robot.calls)

    def test_cancel_phrase_fast_path(self):
        robot = FakeRobot()
        robot.state, robot.dest = NavState.NAVIGATING, "accueil"
        assistant, robot = build(FakeLLM(), robot)
        reply = assistant.handle("Arrête la navigation.")
        self.assertIn(("cancel",), robot.calls)
        self.assertIn("annulée", reply)

    def test_tool_loop_executes_navigation_then_answers(self):
        llm = FakeLLM(call("navigate_to", location="administration"), LLMReply("Je vais à l'administration."))
        assistant, robot = build(llm)
        self.assertEqual(assistant.handle("Va à l'administration."), "Je vais à l'administration.")
        self.assertEqual(robot.calls, [("navigate", "administration")])

    def test_hallucinated_tool_is_refused_and_reported_to_llm(self):
        llm = FakeLLM(call("execute_shell", cmd="rm -rf /"), LLMReply("Je ne peux pas faire cela."))
        assistant, robot = build(llm)
        assistant.handle("Fais un truc")
        tool_msg = llm.seen[1][-1]
        self.assertEqual(tool_msg["role"], "tool")
        self.assertIn("tool_not_allowed", tool_msg["content"])
        self.assertEqual(robot.calls, [])

    def test_llm_failure_returns_technical_message(self):
        class Broken:
            def chat(self, *_):
                raise LLMError("boom")
        assistant, _ = build(Broken())
        self.assertEqual(assistant.handle("Bonjour"), MSG_TECHNICAL)

    def test_history_is_bounded_and_carried(self):
        llm = FakeLLM(*[LLMReply(f"r{i}") for i in range(10)])
        assistant, _ = build(llm)
        for i in range(10):
            assistant.handle(f"m{i}")
        self.assertEqual(len(llm.seen[-1]), 1 + 12 + 1)   # system + 6 échanges + message courant

    def test_announcements_only_from_nav2_events(self):
        spoken = []
        assistant, _ = build(FakeLLM())
        assistant.set_announcer(spoken.append)
        assistant.on_nav_event(NavSnapshot(NavState.SUCCEEDED, "administration"))
        assistant.on_nav_event(NavSnapshot(NavState.FAILED, "laboratoire"))
        assistant.on_nav_event(NavSnapshot(NavState.NAVIGATING, "laboratoire"))
        self.assertEqual(spoken, ["Je suis arrivé à l'administration.",
                                  "La navigation vers le laboratoire a échoué."])


if __name__ == "__main__":
    unittest.main() 
