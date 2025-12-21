# ats_system2/models.py
from typing import List, Optional, Dict, Any, Literal
from pydantic import BaseModel, Field


class Step(BaseModel):
    # 스키마의 enum 5개와 정확히 매칭
    task: Literal[
        "move_to",
        "scan",
        "report_and_wait",
        "track",
        "return_to_home",
    ]
    params: Dict[str, Any] = Field(default_factory=dict)
    guard: Optional[str] = None
    retry: int = Field(0, ge=0)  # minimum: 0


class ReplanRules(BaseModel):
    # 스키마에서 정의된 기본값/범위 그대로 반영
    lost_target_sec: float = 5.0
    battery_rtb: float = Field(0.18, ge=0.0, le=1.0)
    hard_stuck_timeout_sec: float = 20.0
    # 추가 필드들은 스키마상으로 허용(additionalProperties=True)이지만
    # 지금은 명시적으로 안 쓰고, 들어와도 그냥 무시되는 상태(pydantic 기본 extra="ignore")


class HighLevelPlan(BaseModel):
    #  스키마의 const: "1.0.0" 에 맞춤
    version: Literal["1.0.0"] = "1.0.0"

    mission_id: str
    intent: str
    constraints: List[str] = Field(default_factory=list)
    steps: List[Step]
    replan_rules: ReplanRules = ReplanRules()


class VisionSnapshot(BaseModel):
    summary: Optional[str] = None


class System1State(BaseModel):
    mission_id: Optional[str] = None
    system1_state: Optional[str] = None
    current_task: Optional[str] = None
    step_index: Optional[int] = None
    pose: Optional[Dict[str, float]] = None
    vision_snapshot: Optional[VisionSnapshot] = None
    notes: Optional[str] = None
