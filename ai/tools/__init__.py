from assistant.tool_registry import ToolRegistry
from tools import locations, navigation, robot_control, robot_status
from tools.context import ToolContext


def build_registry(ctx: ToolContext) -> ToolRegistry:
    """Liste blanche : ces 7 outils sont les SEULS que le LLM peut appeler."""
    registry = ToolRegistry()
    for module in (robot_status, locations, navigation, robot_control):
        module.register(registry, ctx)
    return registry 
