import json, threading

class VisionCache:
    def __init__(self, w_default=640, h_default=480):
        self._lock = threading.Lock()
        self._v = {"targets": [], "primary_id": None, "lost_sec": 999.0,
                   "frame_w": w_default, "frame_h": h_default}

    def update_from_msg(self, msg_data: str, node_logger):
        try:
            v = json.loads(msg_data) if msg_data else {}
            with self._lock:
                self._v.update({
                    "targets": v.get("targets", []) or [],
                    "primary_id": v.get("primary_id"),
                    "lost_sec": float(v.get("lost_sec", 999.0)),
                    "frame_w": int(v.get("frame_w", self._v["frame_w"])),
                    "frame_h": int(v.get("frame_h", self._v["frame_h"]))
                })
        except Exception as e:
            node_logger.warn(f"[vision] parse failed: {e}")

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._v)
