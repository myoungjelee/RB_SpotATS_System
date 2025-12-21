from isaacsim import SimulationApp
simulation_app = SimulationApp({
    "headless": False,
    "load_extensions": [
        "omni.isaac.ros2_bridge",
        "omni.anim.people",
        "omni.anim.graph.bundle",
        "omni.anim.graph.schema",          
        "omni.anim.navigation.schema",    
        "omni.kit.scripting",
        "omni.anim.graph.ui",
    ]
})


from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core import World
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.rotations import quat_to_rot_matrix
import omni.usd
import carb.settings
import omni.timeline
from pxr import Usd, Gf, UsdGeom, Sdf

import numpy as np
import torch
#import omni.kit.app
#ext_mgr = omni.kit.app.get_app().get_extension_manager()
#ext_mgr.set_extension_enabled("omni.replicator.core", True) 



from pynput import keyboard


settings = carb.settings.acquire_settings_interface()
timeline = omni.timeline.get_timeline_interface()
settings.set("/app/player/useFixedTimeStepping", True)
timeline.set_play_every_frame(True)            # == settings.set("/app/player/useFastMode", True)
settings.set("/app/player/targetRunLoopFrequency", 240)



# === 설정 ===
USD_PATH = "/home/geunpilpark/isaacsim_ws/ATS_IsaacSim/4.5/ATS_Enviroment/World.usd"
SPOT_PRIM_PATH = "/World/Spot"
ats_PRIM_PATH = SPOT_PRIM_PATH + "/ATS/ATS"
POLICY_PATH = "/home/geunpilpark/isaacsim_ws/ATS_IsaacSim/4.5/ATS_Enviroment/spot_policy.pt"
DEVICE = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

ODOM_TOPIC = "odom"
ODOM_FRAME = "odom"
BASE_LINK = "base_link"


# === TorchScript 정책 모델 로드 ===
policy = torch.jit.load(POLICY_PATH, map_location=DEVICE)
policy.eval()
#import carb

# === 시뮬 및 환경 초기화 ===
open_stage(USD_PATH)
simulation_app.update()


timeline = omni.timeline.get_timeline_interface()
timeline.play()

world = World()
world.reset()
spot = ArticulationView(prim_paths_expr=SPOT_PRIM_PATH, name="spot_view")
world.scene.add(spot)
world.play()
ats = ArticulationView(prim_paths_expr=ats_PRIM_PATH, name="ats_view")
world.scene.add(ats)

# === PeopleAnimation 행동 모듈 적용 ===
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.anim.people")
enable_extension("omni.anim.graph.bundle")
enable_extension("omni.kit.scripting")
enable_extension("omni.anim.graph.ui")
enable_extension("omni.anim.graph.schema")
enable_extension("omni.anim.navigation.schema")
enable_extension("omni.isaac.ros2_bridge")
#enable_extension("omni.isaac.render_product")
# 캐릭터 움직임을 위한 People 모듈 사용

from omni.anim.people.scripts.character_behavior import CharacterBehavior

stage = omni.usd.get_context().get_stage()
simulation_app.update()

# 캐릭터 애니메이션 설정 및 명령 주기
from omni.isaac.core.utils.prims import define_prim
graph_path = "/World/CustomGraph"
define_prim(graph_path, "Xform")  # 루트 Prim
define_prim(graph_path + "/CharacterAnimation", "Xform")
define_prim(graph_path + "/CharacterAnimation/AnimationGraph", "AnimationGraph")
define_prim("/World/Spot/IMU", "Xform")


char_behaviors = []
for i in range(1):
    prim_path = "/World/Characters/Character_01/ManRoot/male_adult_police_04"
    animation_graph_path = "/World/Characters/Biped_Setup/CharacterAnimation/AnimationGraph"
    #omni.kit.commands.execute("ApplyAnimationGraphAPICommand", 
    #    paths=[Sdf.Path(prim_path)],
    #    animation_graph_path=Sdf.Path(animation_graph_path),
    #)
    
    #omni.kit.commands.execute("ApplyScriptingAPICommand", paths=[Sdf.Path(prim_path)])
    

    #behavior = CharacterBehavior(prim_path)
    #behavior.init_character()
    #behavior.inject_command(["Character_01 GoTo 2.0 0 0 0"])


# === 입력 초기화 ===
previous_action = np.zeros(12, dtype=np.float32)
default_pos = spot.get_joint_positions().squeeze(0) * 1.0
pressed_keys = set()

import omni.graph.core as og


keys = og.Controller.Keys
(ros_camera_graph, _, _, _) = og.Controller.edit(
    {
        "graph_path": "/ActionGraph",
        "evaluator_name": "push",
        "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
    },
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnTick"),
            ("createViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
            ("getRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
            ("setCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),
            ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            #("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
            ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
            ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
            
            ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
            ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
            ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
            ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
            #("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
            ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
            ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
            #("getRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
        ],
        keys.SET_VALUES: [
            ("createViewport.inputs:viewportId", 0),
            #("cameraHelperRgb.inputs:frameId", "sim_camera"),
            ("cameraHelperRgb.inputs:frameId", "Camera"),
            ("cameraHelperRgb.inputs:topicName", "yolo/image_raw"),
            ("cameraHelperRgb.inputs:type", "rgb"),
            #("cameraHelperInfo.inputs:frameId", "sim_camera"),
            ("cameraHelperInfo.inputs:frameId", "Camera"),
            ("cameraHelperInfo.inputs:topicName", "camera_info"),
            #("cameraHelperDepth.inputs:frameId", "sim_camera"),
            #("cameraHelperDepth.inputs:frameId", "Camera"),
            #("cameraHelperDepth.inputs:topicName", "depth"),
            #("cameraHelperDepth.inputs:type", "depth"),
            ("setCamera.inputs:cameraPrim", "/World/Spot/ATS/ATS/link2/Xform/Camera"),
        ],
    },
)
og.Controller.evaluate_sync(ros_camera_graph)
simulation_app.update()

BASE_LINK_PRIM = Sdf.Path(SPOT_PRIM_PATH)
CAMERA_PRIM    = Sdf.Path("/World/Spot/ATS/ATS/link2/Xform/Camera")
IMU_PRIM       = Sdf.Path(SPOT_PRIM_PATH)

(ros_ats_graph, _, _, _) = og.Controller.edit(
        {"graph_path": "/ATSActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("ATSPublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SpotPublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),

                # IMU Publish 추가
                ("OnTickIMU", "omni.graph.action.OnTick"),
                ("ImuComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("ImuReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ImuPublish", "isaacsim.ros2.bridge.ROS2PublishImu"),

                # ODOM 추가
                ("OdomCompute", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("OdomPublish", "isaacsim.ros2.bridge.ROS2PublishOdometry"),

                # TF(카메라/IMU) 추가
                ("TFReadTime_camimu", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("TFPubCam",          "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("TFPubImu",          "isaacsim.ros2.bridge.ROS2PublishTransformTree"),


            ],
            og.Controller.Keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "SpotPublishJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "ATSPublishJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "SubscribeTwist.inputs:execIn"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "SpotPublishJointState.inputs:context"),
                ("Context.outputs:context", "ATSPublishJointState.inputs:context"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand","ArticulationController.inputs:positionCommand",),
                ("SubscribeJointState.outputs:velocityCommand","ArticulationController.inputs:velocityCommand",),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),

                # IMU 트리거/연결
                # 실행 트리거
                #("OnImpulseEvent.outputs:execOut", "ImuComputeOdom.inputs:execIn"),
                #("OnImpulseEvent.outputs:execOut", "ImuPublish.inputs:execIn"),
                ("OnTickIMU.outputs:tick", "ImuComputeOdom.inputs:execIn"),
                ("OnTickIMU.outputs:tick", "ImuPublish.inputs:execIn"),
                

                # 컨텍스트
                ("Context.outputs:context", "ImuPublish.inputs:context"),

                # 데이터
                ("ImuComputeOdom.outputs:orientation",        "ImuPublish.inputs:orientation"),
                ("ImuComputeOdom.outputs:angularVelocity",    "ImuPublish.inputs:angularVelocity"),
                ("ImuComputeOdom.outputs:linearAcceleration", "ImuPublish.inputs:linearAcceleration"),
                
                # 시간
                ("ImuReadSimTime.outputs:simulationTime", "ImuPublish.inputs:timeStamp"),

                # ODOM 실행/컨텍스트/데이터
                ("OnImpulseEvent.outputs:execOut", "OdomCompute.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "OdomPublish.inputs:execIn"),
                ("Context.outputs:context",       "OdomPublish.inputs:context"),
                ("OdomCompute.outputs:position",        "OdomPublish.inputs:position"),
                ("OdomCompute.outputs:orientation",     "OdomPublish.inputs:orientation"),
                ("OdomCompute.outputs:linearVelocity",  "OdomPublish.inputs:linearVelocity"),
                ("OdomCompute.outputs:angularVelocity", "OdomPublish.inputs:angularVelocity"),
                # 같은 시뮬 타임을 오도메트리에도 사용
                ("ImuReadSimTime.outputs:simulationTime", "OdomPublish.inputs:timeStamp"),

                 # === TF(카메라/IMU) ===
                ("OnImpulseEvent.outputs:execOut", "TFPubCam.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "TFPubImu.inputs:execIn"),
                ("Context.outputs:context",        "TFPubCam.inputs:context"),
                ("Context.outputs:context",        "TFPubImu.inputs:context"),
                # 타임스탬프(데이터 연결만)
                ("TFReadTime_camimu.outputs:simulationTime", "TFPubCam.inputs:timeStamp"),
                ("TFReadTime_camimu.outputs:simulationTime", "TFPubImu.inputs:timeStamp"),

   

                
            ],
            og.Controller.Keys.SET_VALUES: [
                # Setting the /Franka target prim to Articulation Controller node
                ("ArticulationController.inputs:robotPath", "/World/Spot/ATS/ATS"),
                #("ArticulationController.inputs:topicName", "joint_states"),
                ("SubscribeJointState.inputs:topicName", "joint_state_cmd"),
                ("ArticulationController.inputs:targetPrim", "/World/Spot/ATS/ATS"),
                ("SubscribeTwist.inputs:topicName", "cmd_vel"),
                ("SpotPublishJointState.inputs:topicName", "spot_joint_states"),
                ("SpotPublishJointState.inputs:targetPrim", "/World/Spot"),
                ("ATSPublishJointState.inputs:topicName", "ats_joint_states"),
                #("ATSPublishJointState.inputs:targetPrim", "/World/Spot/ATS/ATS"),
                
                # IMU 세팅
                ("ImuComputeOdom.inputs:chassisPrim",        Sdf.Path(SPOT_PRIM_PATH)),
                ("ImuPublish.inputs:frameId", "spot_imu"),
                ("ImuPublish.inputs:topicName", "imu_slow"),
                ("ImuPublish.inputs:publishOrientation", True),
                ("ImuPublish.inputs:publishAngularVelocity", True),
                ("ImuPublish.inputs:publishLinearAcceleration", True),
                ("OnTickIMU.inputs:framePeriod", 0),  
                ("OnTickIMU.inputs:onlyPlayback", True),

                # ODOM 세팅
                ("OdomCompute.inputs:chassisPrim", Sdf.Path(SPOT_PRIM_PATH)),
                ("OdomPublish.inputs:topicName",      "odom"),
                ("OdomPublish.inputs:odomFrameId",    "odom"),
                ("OdomPublish.inputs:chassisFrameId", "base_link"),

                # === TF(카메라/IMU) prim 지정 ===
                # 카메라: base_link prim → camera prim
                ("TFPubCam.inputs:parentPrim",  BASE_LINK_PRIM),
                ("TFPubCam.inputs:targetPrims", [CAMERA_PRIM]),
                # IMU: base_link prim → Spot prim(자식 프레임이 'Spot'으로 퍼블리시)
                ("TFPubImu.inputs:parentPrim",  BASE_LINK_PRIM),
                ("TFPubImu.inputs:targetPrims", [Sdf.Path("/World/Spot/IMU")]), 


            ],
        },
    )

# Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
og.Controller.evaluate_sync(ros_ats_graph)
simulation_app.update()


def on_press(key):
    try:
        if key.char in ['a', 's', 'd', 'w']:
            pressed_keys.add(key.char)
    except AttributeError:
        if key in [keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right]:
            pressed_keys.add(key.name)

def on_release(key):
    try:
        if key.char in ['a', 's', 'd', 'w']:
            pressed_keys.discard(key.char)
    except AttributeError:
        if key.name in ["up", "down", "left", "right"]:
            pressed_keys.discard(key.name)

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.daemon = True
listener.start()

# === 시뮬 루프 ===

frame_count = 0
while simulation_app.is_running():
    world.step(render=True)

    vx = 1.5 if 'up' in pressed_keys else -1.5 if 'down' in pressed_keys else 0.0
    vz = 1.5 if 'left' in pressed_keys else -1.5 if 'right' in pressed_keys else 0.0
    qz = 32.5 if 'a' in pressed_keys else -32.5 if 'd' in pressed_keys else 0.0
    qy = -32.5 if 'w' in pressed_keys else 32.5 if 's' in pressed_keys else 0.0
    vy = 0

    joint_pos = spot.get_joint_positions()[0][0:12]
    joint_vel = spot.get_joint_velocities()[0][0:12]
    lin_vel_I = spot.get_linear_velocities()[0]
    ang_vel_I = spot.get_angular_velocities()[0]
    pos_IB, quat_IB = spot.get_world_poses()
    pos_IB = pos_IB[0]
    quat_IB = quat_IB[0]
    R_IB = quat_to_rot_matrix(quat_IB)
    R_BI = R_IB.T
    lin_vel_b = R_BI @ lin_vel_I
    ang_vel_b = R_BI @ ang_vel_I
    gravity_b = R_BI @ np.array([0.0, 0.0, -1.0], dtype=np.float32)
    
    [vx_r,vy_r,_]=og.Controller.get(og.Controller.attribute("/ATSActionGraph/SubscribeTwist.outputs:linearVelocity"))
    [_,_,vz_r]=og.Controller.get(og.Controller.attribute("/ATSActionGraph/SubscribeTwist.outputs:angularVelocity"))
    
    command = np.array([vx + vx_r, vy + vy_r, vz + vz_r], dtype=np.float32)
    ats_cmd = np.array([qz, qy], dtype=np.float32)

    obs = np.zeros(48, dtype=np.float32)
    obs[0:3] = lin_vel_b
    obs[3:6] = ang_vel_b
    obs[6:9] = gravity_b
    obs[9:12] = command
    obs[12:24] = joint_pos - default_pos[0:12]
    obs[24:36] = joint_vel
    obs[36:48] = previous_action[0:12]

    obs_tensor = torch.tensor(obs, dtype=torch.float32, device=DEVICE).unsqueeze(0)
    with torch.no_grad():
        action_tensor = policy(obs_tensor)
    action = action_tensor.squeeze(0).cpu().numpy()
    ats_joint = ats.get_joint_positions()
    
    spot.set_joint_position_targets(default_pos +  action * 0.15)
    
    ats.set_joint_position_targets(ats_joint + 0.2 * ats_cmd)
    previous_action = action.copy()
    og.Controller.set(og.Controller.attribute("/ATSActionGraph/OnImpulseEvent.state:enableImpulse"), True)
    frame_count += 1

simulation_app.close()