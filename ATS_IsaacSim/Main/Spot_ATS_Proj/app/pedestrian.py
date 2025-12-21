import omni.usd
from pxr import Gf, UsdGeom, Usd
import numpy as np
import traceback

class PedestrianManager:
    def __init__(self, agent_name: str, waypoints: list):
        self.stage = omni.usd.get_context().get_stage()
        self.agent_name = agent_name
        self.prim_path = f"/World/warehouse_with_forklifts/{agent_name}" 
        self.waypoints = waypoints
        self.current_idx = 0
        
        # [설정] 움직임 파라미터
        self.move_speed = 1.5      
        self.turn_speed = 90.0     
        self.align_threshold = 5.0 
        self.dist_threshold = 0.5  

        # [수정됨] 헤딩 보정 (시계방향으로 90도 돌아가 있으므로, 반시계로 90도 돌려줌)
        self.rotation_offset = 90.0 

        self.wait_frames = 0
        self.is_valid = False
        self.rot_order = UsdGeom.XformCommonAPI.RotationOrderXYZ

        # 초기화 확인
        if self.stage:
            prim = self.stage.GetPrimAtPath(self.prim_path)
            if prim.IsValid():
                print(f"[Forklift] Found vehicle at {self.prim_path}")
                self.is_valid = True
                
                # 초기 회전 순서 감지
                xform_api = UsdGeom.XformCommonAPI(prim)
                result = xform_api.GetXformVectors(Usd.TimeCode.Default())
                if result:
                    self.rot_order = result[4] 
                    print(f"[Forklift] Detected Rotation Order: {self.rot_order}")
            else:
                print(f"[Forklift] WARNING: Vehicle NOT found at {self.prim_path}")

    def get_angle_diff(self, target_angle, current_angle):
        diff = (target_angle - current_angle + 180) % 360 - 180
        return diff

    def update(self, dt=0.016):
        if not self.is_valid: return
        
        if self.wait_frames < 60:
            self.wait_frames += 1
            return

        prim = self.stage.GetPrimAtPath(self.prim_path)
        if not prim.IsValid(): return

        xform_api = UsdGeom.XformCommonAPI(prim)
        
        result = xform_api.GetXformVectors(Usd.TimeCode.Default())
        if not result: return
        
        curr_pos = np.array(result[0])
        curr_rot_vec = np.array(result[1]) 
        curr_yaw = curr_rot_vec[2] 

        target_pos = np.array(self.waypoints[self.current_idx])
        direction_vec = target_pos - curr_pos
        dist = np.linalg.norm(direction_vec[:2]) 

        if dist < self.dist_threshold:
            print(f"[Forklift] Reached waypoint {self.current_idx}")
            self.current_idx = (self.current_idx + 1) % len(self.waypoints)
            return

        target_angle_rad = np.arctan2(direction_vec[1], direction_vec[0])
        target_angle_deg = np.degrees(target_angle_rad)
        
        # [로직] 현재 yaw에서 오프셋을 뺀 값이 '논리적 정면'
        real_current_yaw = curr_yaw - self.rotation_offset
        angle_diff = self.get_angle_diff(target_angle_deg, real_current_yaw)

        new_pos = curr_pos
        new_yaw = curr_yaw

        if abs(angle_diff) > self.align_threshold:
            # 회전
            rot_step = np.sign(angle_diff) * self.turn_speed * dt
            if abs(rot_step) > abs(angle_diff):
                rot_step = angle_diff
            new_yaw += rot_step
        else:
            # 직진
            norm_dir = direction_vec / (np.linalg.norm(direction_vec) + 1e-6)
            move_step = norm_dir * self.move_speed * dt
            new_pos = curr_pos + move_step
            
            # 주행 중 미세 조향
            rot_step = np.sign(angle_diff) * self.turn_speed * dt
            if abs(rot_step) > abs(angle_diff): rot_step = angle_diff
            new_yaw += rot_step

        # 적용
        xform_api.SetTranslate(Gf.Vec3d(new_pos[0], new_pos[1], new_pos[2]))
        xform_api.SetRotate((0, 0, new_yaw), self.rot_order)