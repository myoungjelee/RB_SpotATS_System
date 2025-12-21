from typing import Any, Dict

HIGH_LEVEL_PLAN_SCHEMA: Dict[str, Any] = {
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "title": "HighLevelPlan",
    "type": "object",
    "required": ["version", "mission_id", "intent", "steps"],
    "properties": {
        "version": {"type": "string", "const": "1.0.0"},

        "mission_id": {"type": "string", "minLength": 1},
        "intent": {"type": "string", "minLength": 1},
        "constraints": {"type": "array", "items": {"type": "string"}},

        "steps": {
            "type": "array",
            "minItems": 1,
            "items": {
                "type": "object",
                "required": ["task", "params"],
                "properties": {
                    # 여기서 5개 단위 액션만 허용
                    "task": {
                        "type": "string",
                        "enum": [
                            "move_to",
                            "scan",
                            "report_and_wait",
                            "track",
                            "return_to_home"
                        ]
                    },
                    "params": {"type": "object"},
                    "guard": {"type": "string"},
                    "retry": {
                        "type": "integer",
                        "minimum": 0,
                        "default": 0
                    }
                },
                "additionalProperties": False
            }
        },

        "replan_rules": {
            "type": "object",
            "properties": {
                "lost_target_sec": {
                    "type": "number",
                    "default": 5.0
                },
                "battery_rtb": {
                    "type": "number",
                    "minimum": 0.0,
                    "maximum": 1.0,
                    "default": 0.18
                },
                "hard_stuck_timeout_sec": {
                    "type": "number",
                    "default": 20.0
                },
                # 여기 이하로는 필요시 자유롭게 확장
            },
            "additionalProperties": True
        }
    },
    "additionalProperties": False
}
