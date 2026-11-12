"""Platform-side extension point for a read-only mission advisor."""
from typing import Literal, Protocol, TypedDict


class MissionContext(TypedDict):
    version: int
    status: str
    stage: str
    executor: str
    completed: int
    total: int
    autonomy_active: bool


class MissionAdvice(TypedDict):
    kind: Literal['explanation', 'route_review', 'task_suggestion']
    text: str


class MissionAdvisor(Protocol):
    """A future adapter receives only this summary and returns display text."""

    def advise(self, context: MissionContext) -> MissionAdvice:
        ...


def mission_context(nav, autonomy_active):
    route=nav.get('route') or {}
    executor=nav.get('executor') or nav.get('engine') or ('native' if route else '')
    return MissionContext(version=1,status=nav.get('status','idle'),
                          stage=nav.get('phase','idle'),executor=executor,
                          completed=nav.get('completed',0),total=len(route.get('points',[])),
                          autonomy_active=bool(autonomy_active))
