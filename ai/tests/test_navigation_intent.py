import unittest

from assistant.llm_client import LLMReply
from tests.fakes import FakeLLM
from tests.test_assistant import build


class TestNavigationIntent(unittest.TestCase):
    def test_explicit_order_bypasses_llm(self):
        assistant, robot = build(FakeLLM())        # FakeLLM vide : lèverait si appelé
        assistant.handle("Va à l'administration.")
        self.assertEqual(robot.calls, [("navigate", "administration")])

    def test_finalement_replaces_current_goal(self):
        assistant, robot = build(FakeLLM())
        assistant.handle("Va à l'accueil.")
        assistant.handle("Finalement au labo.")
        self.assertEqual(robot.dest, "laboratoire")

    def test_unknown_place_goes_to_llm(self):
        assistant, robot = build(FakeLLM(LLMReply("Je ne connais pas cette destination.")))
        self.assertEqual(assistant.handle("Va à la cafétéria."), "Je ne connais pas cette destination.")
        self.assertEqual(robot.calls, [])

    def test_claim_without_tool_call_is_replaced(self):
        assistant, robot = build(FakeLLM(LLMReply("Je vais vous envoyer le robot vers l'administration.")))
        reply = assistant.handle("Je cherche l'administration.")
        self.assertIn("aucun déplacement", reply)
        self.assertEqual(robot.calls, [])


if __name__ == "__main__":
    unittest.main()
