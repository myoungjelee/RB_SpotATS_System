# USD_test.py  (Isaac Sim 5.0 런타임 내 import 전제)
from typing import Iterable, List, Optional
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
import omni.usd
import omni.kit.app
import omni.kit.commands as kitcmd


class USDStageHelper:
    """Isaac Sim 5.0 런타임(이미 SimulationApp 부팅된 상태)에서
    - 트리/레이어/트랜스폼 안전 덤프
    - 월드→로컬 변환 후 안전 리패런팅
    - (옵션) FixedJoint 생성
    를 도와주는 유틸 클래스
    """

    def __init__(self):
        self._app = omni.kit.app.get_app()
        self._ctx = omni.usd.get_context()
        self._stage: Usd.Stage = self._ctx.get_stage()
        if not self._stage:
            raise RuntimeError("Stage가 없습니다. SimulationApp 부팅 후에 import/생성하세요.")

    # ---------- 기본 유틸 ----------
    @staticmethod
    def _is_xformable(prim: Usd.Prim) -> bool:
        return bool(prim) and prim.IsA(UsdGeom.Xformable)

    def _world_xf(self, prim: Usd.Prim) -> Gf.Matrix4d:
        if not self._is_xformable(prim):
            return Gf.Matrix4d(1.0)
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    @staticmethod
    def _top_opinion_layer_id(prim: Usd.Prim) -> str:
        try:
            stack = prim.GetPrimStack()
            if stack:
                return stack[0].layer.identifier
        except Exception:
            pass
        return "?"

    @staticmethod
    def _list_xform_ops(prim: Usd.Prim) -> List[str]:
        if not prim or not prim.IsA(UsdGeom.Xformable):
            return []
        x = UsdGeom.Xformable(prim)
        return [op.GetOpName() for op in x.GetOrderedXformOps()]

    # ---------- 표시/점검 ----------
    def print_tree(self, root_path: str = "/World", max_depth: int = 64):
        stage = self._stage
        def _walk(path: str, indent: int = 0):
            prim = stage.GetPrimAtPath(path)
            if not prim:
                print("  " * indent + f"(missing) {path}")
                return
            inst = prim.IsInstance()
            opin = self._top_opinion_layer_id(prim)
            ops = self._list_xform_ops(prim)
            print("  " * indent + f"{prim.GetPath()}  inst={inst}  topOpinion={opin}  xformOps={ops}")
            if indent >= max_depth:
                return
            for c in prim.GetChildren():
                _walk(c.GetPath().pathString, indent + 1)
        _walk(root_path, 0)

    def print_ancestors(self, path: str):
        """조상 체인의 존재/인스턴스 상태 점검"""
        stage = self._stage
        parts = path.strip("/").split("/")
        cur = ""
        for p in parts:
            cur = (cur + "/" + p) if cur else ("/" + p)
            prim = stage.GetPrimAtPath(cur)
            print(f"[ANCESTOR] {cur:35s} exists={prim.IsValid():5}  IsInstance={prim.IsInstance():5}")

    # ---------- 편집 타겟 ----------
    def set_edit_target_root(self):
        """루트 레이어에 직접 기록(세션 레이어에만 쓰여서 날아가는 문제 방지)"""
        self._stage.SetEditTarget(Usd.EditTarget(self._stage.GetRootLayer()))

    def set_edit_target_session(self):
        """세션 레이어에 기록(일시 확인용)"""
        self._stage.SetEditTarget(Usd.EditTarget(self._stage.GetSessionLayer()))

    # ---------- 리패런팅(월드→로컬 포즈 유지) ----------
    def reparent_with_local_pose(
        self,
        device_path: str,
        parent_link_path: str,
        *,
        make_fixed_joint: bool = False,
        fixed_joint_path: Optional[str] = None,
    ) -> str:
        """
        device_path 프림을 parent_link_path 아래로 이동시키되,
        '현재 월드 포즈'를 유지하도록 '부모 기준 로컬 포즈'로 재작성.
        필요 시 FixedJoint(물리 용접)도 생성.

        Returns: 새 프림 경로(부모 아래)
        """
        stage = self._stage
        dev = stage.GetPrimAtPath(device_path)
        par = stage.GetPrimAtPath(parent_link_path)
        if not dev or not dev.IsValid():
            raise RuntimeError(f"Device prim not found: {device_path}")
        if not par or not par.IsValid():
            raise RuntimeError(f"Parent link prim not found: {parent_link_path}")

        # 인스턴스면 이동 불가 → instanceable 해제
        if dev.IsInstance():
            dev.SetInstanceable(False)

        # 월드->로컬 변환 계산
        W_dev = self._world_xf(dev)
        W_par = self._world_xf(par)
        L_dev = W_dev * W_par.GetInverse()

        # 새 경로(부모 아래)
        new_path = Sdf.Path(parent_link_path).AppendChild(dev.GetName()).pathString

        # 이미 그 아래면 Move 생략
        if dev.GetParent().GetPath() != par.GetPath():
            ok, _ = kitcmd.execute("MovePrim", path_from=device_path, path_to=new_path)
            if not ok:
                raise RuntimeError(f"MovePrim 실패: {device_path} -> {new_path}")
            dev = stage.GetPrimAtPath(new_path)

        # 로컬 포즈 재설정(translate + orient(Quatf) 사용 권장)
        if not self._is_xformable(dev):
            # Xformable이 아닌 경우 셸 Xform을 하나 덮어 씌우고 그 밑으로 옮기는 선택지도 있으나,
            # 여기선 단순 경고로 처리
            print(f"[WARN] {new_path} is not Xformable. Pose not written.")
        else:
            x = UsdGeom.Xformable(dev)
            # 기존 xformOp 제거
            for op in list(x.GetOrderedXformOps()):
                x.RemoveXformOp(op)

            # 변환 분해
            t = L_dev.ExtractTranslation()
            quatd = L_dev.ExtractRotation().GetQuat()
            # USD Xform은 float 선호 → Quatf로 캐스팅
            quatf = Gf.Quatf(float(quatd.GetReal()),
                             Gf.Vec3f(float(quatd.GetImaginary()[0]),
                                      float(quatd.GetImaginary()[1]),
                                      float(quatd.GetImaginary()[2])))

            # 새 op 작성
            to = x.AddTranslateOp()
            oo = x.AddOrientOp()  # quaternion
            so = x.AddScaleOp()
            to.Set(Gf.Vec3f(float(t[0]), float(t[1]), float(t[2])))
            oo.Set(quatf)
            so.Set(Gf.Vec3f(1.0, 1.0, 1.0))

        # (옵션) FixedJoint로 물리 용접
        if make_fixed_joint:
            jpath = fixed_joint_path or (new_path + "_fixedJoint")
            joint = UsdPhysics.FixedJoint.Define(stage, jpath)
            joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link_path)])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(new_path)])

        # flush
        for _ in range(2):
            self._app.update()
        return new_path

    def bulk_reparent(
        self,
        device_paths: Iterable[str],
        parent_link_path: str,
        *,
        make_fixed_joint: bool = False,
    ) -> List[str]:
        """여러 디바이스를 한 번에 부모 링크 하위로 안전 리패런팅."""
        out = []
        for p in device_paths:
            try:
                newp = self.reparent_with_local_pose(
                    p, parent_link_path, make_fixed_joint=make_fixed_joint
                )
                out.append(newp)
            except Exception as e:
                print(f"[ERR] reparent {p} -> {parent_link_path}: {e}")
        return out

    def scan_device_prims(self, roots=("/World/Spot", "/World/Spot/ATS")):
        """장면에서 카메라/라이다/IMU로 보이는 프림들을 찾아 경로 리스트로 반환."""
        stage = self._stage
        found = []
        type_keys = ("Camera", "RtxLidar", "OmniLidar", "Lidar", "Imu", "IMU")
        name_keys = ("camera", "lidar", "scan", "imu", "depth", "rgb")

        def _ok(prim):
            if not prim or not prim.IsValid():
                return False
            t = (prim.GetTypeName() or "")
            n = (prim.GetName() or "")
            if any(k in t for k in type_keys):
                return True
            if any(k in n.lower() for k in name_keys):
                return True
            return False

        for root in roots:
            rprim = stage.GetPrimAtPath(root)
            if not rprim.IsValid():
                continue
            for p in Usd.PrimRange(rprim):
                if _ok(p):
                    found.append(p.GetPath().pathString)
        # 중복 제거, 정렬
        return sorted(set(found))

    def exists(self, path: str) -> bool:
        return self._stage.GetPrimAtPath(path).IsValid()