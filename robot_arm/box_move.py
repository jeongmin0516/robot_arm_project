import numpy as np
import time
import os

from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink

from dynamixel_sdk import PortHandler
from dynamixel_sdk.controllerXC import *
from dynamixel_sdk.controllerAX import *

# =========================================================
# 기본 설정
# =========================================================
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 115200

XM10_ID = 10
AX11_ID = 11
AX12_ID = 12
AX13_ID = 13
XC14_ID = 15 # 14망가짐 

THRESHOLD_XM = 1
THRESHOLD_AX = 1

# =========================================================
# 5차 trajectory 설정
# =========================================================
# move_xyz()를 호출하면 현재 관절각 -> 목표 관절각까지
# 5차 함수 s(t)=10t^3-15t^4+6t^5 로 부드럽게 이동합니다.
TRAJ_DURATION = 0.5  # [sec] 한 번의 move_xyz 이동 시간
TRAJ_DT = 0.01        # [sec] 명령 발행 간격
TRAJ_DEBUG = False    # True면 trajectory 중간 각도 출력

GRIPPER_ORIENTATION = "manual"

# =========================================================
# DH 파라미터 [mm]
# =========================================================
# 사진 DH 기준:
# 0 -> 1 : theta1 - 90deg, d1, 0,  90deg
# 1 -> 2 : theta2 + 90deg, 0,  a1, 0deg
# 2 -> 3 : theta3,         0,  a2, 0deg
# 3 -> 4 : theta4,         0,  a3, 0deg

D1 = 125.626
A1 = 134.5
A2 = 134.5
A3 = 102 # 130     102

# Base -> Frame0 offset [mm]
BASE_TO_FRAME0_XYZ = np.array([0, 0, 0.0], dtype=float)  # -80   이거 그냥 다 0으로 
 # -85 24.747
# =========================================================
# 목표 위치 [mm]
# =========================================================

# 실제로 바꿔야할 파라미터 


theta = np.deg2rad(110) 
distance = 200 # 200mm가 그리퍼를 아래로 향하면서 잡을 수 있는 최대거리 300mm(이 이상은 무서워서 안해봄...)가 거의 일직선으로 뻗을 수 있는 거리 
# 150 보낼때 15mm더 감 그래서 빼줘야할 듯 가까이 하면 더 안쪽으로 들어옴 

theta_2 = np.deg2rad(0)
distance_2 = 125

# theta = 60일때 3 offset
# theta = 0일때  7?
# theta_2 = 30에 distance_2 125하니까 overload뜸 

# 7번 둘때 -3
def apply_robot_side_offset(theta_rad):
    """
    좌표계:
    - theta = 0deg   : 동쪽(+X)
    - theta = 90deg  : 북쪽(+Y), 로봇팔 정면
    - theta = 180deg : 서쪽(-X)
    - theta는 반시계 방향 증가

    로봇 기준:
    - 오른쪽: 동쪽 쪽, cos(theta) > 0
    - 왼쪽: 서쪽 쪽, cos(theta) < 0

    보정:
    - 오른쪽에 있으면 오른쪽으로 3도 더 감 -> theta 감소
    - 왼쪽에 있으면 왼쪽으로 5도 더 감 -> theta 증가
    """

    if np.cos(theta_rad) > 0:
        theta_rad -= np.deg2rad(1)

    elif np.cos(theta_rad) <= 0:
        theta_rad += np.deg2rad(3)  # 5

    return theta_rad


theta = apply_robot_side_offset(theta)
theta_2 = apply_robot_side_offset(theta_2)

# 기본 
# offset_1 = 0  
# offset_1_z = 0
# offset_2 = 0
# offset_2_z = 0
# offset_3 = 0
# offset_3_z = 0
# offset_4 = 0
# offset_4_z = 0
# offset_5 = 0
# offset_5_z = 0
# offset_6 = 0
# offset_6_z = 0
# offset_7 = 0
# offset_7_z = 0
# offset_8 = 0
# offset_8_z = 0
# offset_9 = 0
# offset_9_z = 0


# 각도가 40~50도일때 150mm일때 offset                     0~30도까지는 offset = 0
# offset_1 = 0  
# offset_1_z = 0
# offset_2 = 0
# offset_2_z = 0
# offset_3 = -5
# offset_3_z = 0
# offset_4 = -10
# offset_4_z = 0
# offset_5 = -8
# offset_5_z = 0
# offset_6 = -3
# offset_6_z = 0
# offset_7 = -5
# offset_7_z = 0
# offset_8 = 0
# offset_8_z = 0
# offset_9 = 0
# offset_9_z = 0


# put_down_offset_1 = -5
# put_down_offset_z_1 = 0
# put_down_offset_2 = -5
# put_down_offset_z_2 = 0
# put_down_offset_3 = -5
# put_down_offset_z_3 = -15
# put_down_offset_4 = -5
# put_down_offset_z_4 = 0
# put_down_offset_5 = 0
# put_down_offset_z_5 = 0
# put_down_offset_6 = 0
# put_down_offset_z_6 = 0



# case1 theta = 90, theta_2 = 0 d_1 = 175, d_2 = 175 
# 집을 때의 offset
# offset_1 = -12
# offset_1_z = 0
# offset_2 = -5
# offset_2_z = 0
# offset_3 = -10
# offset_3_z = +5
# offset_4 = -15
# offset_4_z = 5
# offset_5 = -12
# offset_5_z = 5
# offset_6 = -6
# offset_6_z = 0
# offset_7 = 0
# offset_7_z = 0
# offset_8 = -10
# offset_8_z = 0
# offset_9 = 0
# offset_9_z = 0

# put_down_offset_1 = 0
# put_down_offset_z_1 = 0
# put_down_offset_2 = 0
# put_down_offset_z_2 = 0
# put_down_offset_3 = 0
# put_down_offset_z_3 = -10
# put_down_offset_4 = 0
# put_down_offset_z_4 = -3
# put_down_offset_5 = 0
# put_down_offset_z_5 = 0
# put_down_offset_6 = -5
# put_down_offset_z_6 = 0


# distance = 180
# put_down_offset_1 = 0
# put_down_offset_z_1 = 0
# put_down_offset_2 = 0
# put_down_offset_z_2 = 0
# put_down_offset_3 = 0
# put_down_offset_z_3 = -10
# put_down_offset_4 = 0
# put_down_offset_z_4 = -6
# put_down_offset_5 = 5
# put_down_offset_z_5 = 0
# put_down_offset_6 = -5
# put_down_offset_z_6 = 0


# case2 theta = 90, theta_2 = 0 d_1 = 125, d_2 = 125 
offset_1 = 10
offset_1_z = 0
offset_2 = 5
offset_2_z = 0
offset_3 = 5
offset_3_z = 0
offset_4 = -10
offset_4_z = -5
offset_5 = -3
offset_5_z = 0
offset_6 = 7
offset_6_z = 10
offset_7 = 0
offset_7_z = -5
offset_8 = 2
offset_8_z = -8
offset_9 = 5
offset_9_z = -3

# put_down_offset_1 = 0
# put_down_offset_z_1 = 0
# put_down_offset_2 = 0
# put_down_offset_z_2 = -5
# put_down_offset_3 = -10
# put_down_offset_z_3 = -30
# put_down_offset_4 = 0
# put_down_offset_z_4 = 0
# put_down_offset_5 = 0
# put_down_offset_z_5 = 0
# put_down_offset_6 = 0
# put_down_offset_z_6 = 0



# case2   distance = 200
put_down_offset_1 = -15
put_down_offset_z_1 = -5
put_down_offset_2 = -5
put_down_offset_z_2 = 0
put_down_offset_3 = -5
put_down_offset_z_3 = -10
put_down_offset_4 = -10
put_down_offset_z_4 = 0
put_down_offset_5 = -10
put_down_offset_z_5 = 0
put_down_offset_6 = +5
put_down_offset_z_6 = 20



# case3 distance_2 = 130 
# offset_1 = 10
# offset_1_z = 0
# offset_2 = 5
# offset_2_z = 0
# offset_3 = 5
# offset_3_z = 0
# offset_4 = -5
# offset_4_z = 0
# offset_5 = 0
# offset_5_z = 0
# offset_6 = 5
# offset_6_z = 10
# offset_7 = 0
# offset_7_z = 0
# offset_8 = 5
# offset_8_z = 0
# offset_9 = 5
# offset_9_z = 0


# distance = 140
# put_down_offset_1 = 0
# put_down_offset_z_1 = 0
# put_down_offset_2 = 0
# put_down_offset_z_2 = 0
# put_down_offset_3 = 0
# put_down_offset_z_3 = -15
# put_down_offset_4 = -10
# put_down_offset_z_4 = 0
# put_down_offset_5 = 3
# put_down_offset_z_5 = 0
# put_down_offset_6 = 0
# put_down_offset_z_6 = 0

# distance = 160
# put_down_offset_1 = 0
# put_down_offset_z_1 = 0
# put_down_offset_2 = 0
# put_down_offset_z_2 = 0
# put_down_offset_3 = 0
# put_down_offset_z_3 = -15
# put_down_offset_4 = -10
# put_down_offset_z_4 = +3
# put_down_offset_5 = 5
# put_down_offset_z_5 = 0
# put_down_offset_6 = 3
# put_down_offset_z_6 = 0

# distance_2  = 160
# offset_1 = 0  
# offset_1_z = 0
# offset_2 = 0
# offset_2_z = 0
# offset_3 = -10
# offset_3_z = 0
# offset_4 = -10
# offset_4_z = 0
# offset_5 = -8
# offset_5_z = 0
# offset_6 = -8
# offset_6_z = 0
# offset_7 = -5
# offset_7_z = 0
# offset_8 = -10
# offset_8_z = 0
# offset_9 = 0
# offset_9_z = 0
 

#=================================================== 
# distance = 137.8
# offset_1 = 3
# offset_1_z = 0
# offset_2 = 0
# offset_2_z = 0
# offset_3 = -5
# offset_3_z = 0
# offset_4 = -10
# offset_4_z = 0
# offset_5 = -8
# offset_5_z = 0
# offset_6 = 2
# offset_6_z = 0
# offset_7 = -2
# offset_7_z = -5
# offset_8 = 0
# offset_8_z = -5
# offset_9 = 0
# offset_9_z = 0


# distance = 170
# put_down_offset_1 = 0
# put_down_offset_z_1 = 0
# put_down_offset_2 = 0
# put_down_offset_z_2 = 0
# put_down_offset_3 = +5
# put_down_offset_z_3 = -10
# put_down_offset_4 = -7
# put_down_offset_z_4 = -3
# put_down_offset_5 = 0
# put_down_offset_z_5 = 0
# put_down_offset_6 = -5
# put_down_offset_z_6 = 0


# distance_2 = 180
# offset_1 = -15
# offset_1_z = 0
# offset_2 = -10
# offset_2_z = 0
# offset_3 = -5
# offset_3_z = +5
# offset_4 = -15
# offset_4_z = 5
# offset_5 = -15
# offset_5_z = 5
# offset_6 = -6
# offset_6_z = 0
# offset_7 = -3
# offset_7_z = 0
# offset_8 = -8
# offset_8_z = 0
# offset_9 = 0
# offset_9_z = 5

# distance_2 = 170
# offset_1 = -12 + 5
# offset_1_z = 0
# offset_2 = -5
# offset_2_z = 0
# offset_3 = -10 -10 
# offset_3_z = +5
# offset_4 = -15
# offset_4_z = 5
# offset_5 = -12
# offset_5_z = 5
# offset_6 = -6
# offset_6_z = 0
# offset_7 = 0
# offset_7_z = 0
# offset_8 = -10 + 10 -7 
# offset_8_z = 0 
# offset_9 = 0
# offset_9_z = 0

# distance = 120
# put_down_offset_1 = 0
# put_down_offset_z_1 = 0
# put_down_offset_2 = 0
# put_down_offset_z_2 = -5
# put_down_offset_3 = -10
# put_down_offset_z_3 = -30
# put_down_offset_4 = 0
# put_down_offset_z_4 = -5
# put_down_offset_5 = 0
# put_down_offset_z_5 = 0
# put_down_offset_6 = 0
# put_down_offset_z_6 = 0




  
# x축 기준 반시계 방향 증가 
TARGET_X = distance * np.cos(theta) #20.0 
TARGET_Y = distance * np.sin(theta) #20.0
TARGET_Z = 50

TARGET_X_2 = distance_2 * np.cos(theta_2) #20.0 
TARGET_Y_2 = distance_2 * np.sin(theta_2) #20.0
TARGET_Z_2 = 195


def get_drawer_place_points(distance, theta, lateral_offset=10.0, forward_offset = 0.0):
    """
    distance, theta로 정의된 중심점에서
    반지름 방향에 수직인 방향으로 좌우 lateral_offset(mm)만큼 떨어진 점 생성

    반환:
        center_xy, left_xy, right_xy
    """

    cx = distance * np.cos(theta)
    cy = distance * np.sin(theta)

    center = np.array([cx, cy], dtype=float)

    # theta 방향에 수직인 단위벡터
    perp = np.array([-np.sin(theta), np.cos(theta)], dtype=float)
    
    direction = np.array([np.cos(theta), np.sin(theta)], dtype=float)

    left = center + lateral_offset * perp + forward_offset * direction
    right = center - lateral_offset * perp + forward_offset * direction

    return center, left, right

#center_xy, left_xy, right_xy = get_drawer_place_points(distance,theta,lateral_offset=10.0)


# yaw: xy 평면 방향. 0deg는 +X, 90deg는 +Y 방향입니다.
# pitch: xy 평면 기준 상하 각도. -90deg는 아래, 0deg는 정면, +90deg는 위입니다.
# z가 30이 거의 지면이랑 닿을락 말락 

# =========================================================
# IKPy Chain 생성
# =========================================================
# 잘 되는 코드처럼 DHLink를 직접 사용.
# IKPy의 DHLink theta에는 "고정 offset"을 넣고,
# 실제 joint variable은 IKPy가 더해서 계산함.
#
# Joint1: theta1 - 90deg
# Joint2: theta2 + 90deg
# Joint3: theta3
# Joint4: theta4
# =========================================================
chain = Chain(name='robot_arm', links=[
    OriginLink(),

    DHLink(
        name="XM10",
        theta=-np.pi / 2,
        d=D1,
        a=0.0,
        alpha=np.pi / 2,
    ),

    DHLink(
        name="AX11",
        theta=np.pi / 2,
        d=0.0,
        a=A1,
        alpha=0.0,
    ),

    DHLink(
        name="AX12",
        theta=0.0,
        d=0.0,
        a=A2,
        alpha=0.0,
    ),

    DHLink(
        name="AX13",
        theta=0.0,
        d=0.0,
        a=A3,
        alpha=0.0,
    ),
])

# 4번째 앞에 잡음 5번째 z 올리기 
# OriginLink 포함해서 active mask 설정
chain.active_links_mask = [False, True, True, True, True]

chain.links[1].bounds = (np.deg2rad(-100), np.deg2rad(100))   # XM10
chain.links[2].bounds = (np.deg2rad(-30), np.deg2rad(150))   # XM10
chain.links[3].bounds = (np.deg2rad(-30), np.deg2rad(150))   # XM10
chain.links[4].bounds = (np.deg2rad(-30), np.deg2rad(150))   # XM10  # 150
print("========== chain bounds check ==========")
for i, link in enumerate(chain.links):
    b = getattr(link, "bounds", None)

    if b is None:
        print(f"{i}: {link.name}, bounds 없음")
    else:
        print(
            f"{i}: {link.name}, "
            f"bounds rad=({b[0]:.4f}, {b[1]:.4f}), "
            f"bounds deg=({np.rad2deg(b[0]):.2f}, {np.rad2deg(b[1]):.2f})"
        )
print("========================================")

# =========================================================
# 인코더 변환
# =========================================================
def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def ik_deg_to_xm_enc(deg):
    """
    XM/XC 계열:
    IK 각도 0deg -> encoder 2048
    +deg 방향 -> encoder 증가
    """
    enc = int(round((deg / 360.0) * 4095.0 + 2048.0))
    return clamp(enc, 0, 4095)


def ik_deg_to_ax_enc(deg):
    """
    AX 계열:
    IK 각도 0deg -> motor 150deg -> encoder 약 512
    IK 각도 -150deg -> encoder 0
    IK 각도 +150deg -> encoder 1023
    """
    motor_deg = deg + 150.0
    enc = int(round((motor_deg / 300.0) * 1023.0))
    return clamp(enc, 0, 1023)


def xm_enc_to_ik_deg(enc):
    """
    XM encoder -> IK 기준 각도
    encoder 2048 -> 0deg
    """
    return (enc - 2048.0) * 360.0 / 4095.0


def ax_enc_to_ik_deg(enc):
    """
    AX encoder -> IK 기준 각도
    encoder 512 -> 약 0deg
    """
    motor_deg = enc * 300.0 / 1023.0
    return motor_deg - 150.0

def get_gripper_direction(orientation, yaw_deg, pitch_deg):
    """
    orientation:
    - "down": 그리퍼가 아래를 봄
    - "up"  : 그리퍼가 위를 봄
    """

    if orientation == "down":
        return np.array([0.0, 0.0, -1.0], dtype=float)

    elif orientation == "front":
        return np.array([np.cos(theta), np.sin(theta), 0.0], dtype=float)
    
    elif orientation == "manual":
        return direction_from_yaw_pitch(yaw_deg, pitch_deg)

    else:
        raise ValueError("orientation은 'up' 또는 'front'만 가능")



# =========================================================
# 현재 모터각 읽기
# =========================================================
def read_current_ik_angles(XM10, AX11, AX12, AX13):
    xm10_pos = XM10.read_current_position()
    ax11_pos = AX11.read_current_position()
    ax12_pos = AX12.read_current_position()
    ax13_pos = AX13.read_current_position()

    theta1 = xm_enc_to_ik_deg(xm10_pos)
    theta2 = ax_enc_to_ik_deg(ax11_pos)
    theta3 = ax_enc_to_ik_deg(ax12_pos)
    theta4 = ax_enc_to_ik_deg(ax13_pos)

    return np.array([theta1, theta2, theta3, theta4], dtype=float)


# =========================================================
# 5차 함수 trajectory
# =========================================================
def quintic_smooth_step(t):
    """
    5차 시간 스케일링 함수.

    조건:
        s(0)=0, s(1)=1
        s_dot(0)=0, s_dot(1)=0
        s_ddot(0)=0, s_ddot(1)=0

    수식:
        s = 10t^3 - 15t^4 + 6t^5
    """
    return 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5


def make_quintic_joint_trajectory(start_angles_deg, goal_angles_deg,
                                  duration=TRAJ_DURATION, dt=TRAJ_DT):
    """
    현재 관절각 -> 목표 관절각까지 5차 함수로 joint trajectory 생성.
    """
    start_angles_deg = np.array(start_angles_deg, dtype=float)
    goal_angles_deg = np.array(goal_angles_deg, dtype=float)

    steps = max(2, int(duration / dt) + 1)
    traj = []

    for i in range(steps):
        tau = i / (steps - 1)
        s = quintic_smooth_step(tau)
        angles = start_angles_deg + s * (goal_angles_deg - start_angles_deg)
        traj.append(angles)

    return traj


def move_joint_angles_quintic(XM10, AX11, AX12, AX13,
                              start_angles_deg, goal_angles_deg,
                              duration=TRAJ_DURATION, dt=TRAJ_DT,
                              print_debug=TRAJ_DEBUG):
    """
    start_angles_deg -> goal_angles_deg까지 5차 함수로 부드럽게 이동.
    """
    traj = make_quintic_joint_trajectory(
        start_angles_deg=start_angles_deg,
        goal_angles_deg=goal_angles_deg,
        duration=duration,
        dt=dt
    )

    print(f"5차 trajectory 시작: steps={len(traj)}, duration={duration}, dt={dt}")

    for idx, angles_deg in enumerate(traj):
        xm10_enc = ik_deg_to_xm_enc(angles_deg[0])
        ax11_enc = ik_deg_to_ax_enc(angles_deg[1])
        ax12_enc = ik_deg_to_ax_enc(angles_deg[2])
        ax13_enc = ik_deg_to_ax_enc(angles_deg[3])

        XM10.move(xm10_enc)
        AX11.move(ax11_enc)
        AX12.move(ax12_enc)
        AX13.move(ax13_enc)

        if print_debug:
            print(
                f"idx={idx:03d}/{len(traj)-1:03d} | "
                f"angle=[{angles_deg[0]:.2f}, {angles_deg[1]:.2f}, {angles_deg[2]:.2f}, {angles_deg[3]:.2f}] | "
                f"enc=[{xm10_enc}, {ax11_enc}, {ax12_enc}, {ax13_enc}]"
            )

        time.sleep(dt)

    print("5차 trajectory 완료")


# =========================================================
# IK 계산
# =========================================================
def solve_ik(target_xyz,yaw_deg,pitch_deg, current_angles_deg=None, orientation="manual"):
    """
    target_xyz: Base frame 기준 목표 위치 [mm]
    current_angles_deg: 현재 IK 기준 joint angle [deg]
    orientation: "up" 또는 "down"
    """

    target_xyz = np.array(target_xyz, dtype=float)

    target_frame0 = target_xyz - BASE_TO_FRAME0_XYZ

    if current_angles_deg is None:
        current_angles_deg = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)

    current_rad = np.deg2rad(current_angles_deg)

    initial_position = [
        0.0,
        current_rad[0],
        current_rad[1],
        current_rad[2],
        current_rad[3],
    ]

    gripper_direction = get_gripper_direction(orientation, yaw_deg, pitch_deg)  # 각도 한번 입력하기

    result = chain.inverse_kinematics(
        target_position=target_frame0,
        target_orientation=gripper_direction,
        orientation_mode="X",
        initial_position=initial_position
    )

    theta1 = np.degrees(result[1])
    theta2 = np.degrees(result[2])
    theta3 = np.degrees(result[3])
    theta4 = np.degrees(result[4])

    angles_deg = np.array([theta1, theta2, theta3, theta4], dtype=float)
    
    print("IK raw result rad =", result)
    print("XM10 raw deg =", np.degrees(result[1]))
    print("XM10 bounds deg =", -100, 100)

    return angles_deg


# =========================================================
# FK 확인용
# =========================================================
def fk_position(angles_deg):
    """
    IK 기준 각도를 넣으면 Base frame 기준 end-effector 위치 [mm] 반환
    """
    angles_rad = np.deg2rad(angles_deg)

    joint_angles = [
        0.0,
        angles_rad[0],
        angles_rad[1],
        angles_rad[2],
        angles_rad[3],
    ]

    T = chain.forward_kinematics(joint_angles)
    pos_frame0 = T[:3, 3]
    pos_base = pos_frame0 + BASE_TO_FRAME0_XYZ

    return pos_base


# =========================================================
# 목표 위치로 이동
# =========================================================
def move_and_wait(XM10, AX11, AX12, AX13,
                  xm10_t, ax11_t, ax12_t, ax13_t,
                  timeout_sec=2.0):

    XM10.move(xm10_t)
    AX11.move(ax11_t)
    AX12.move(ax12_t)
    AX13.move(ax13_t)

    start_time = time.time()

    while True:
        xm10_pos = XM10.read_current_position()
        ax11_pos = AX11.read_current_position()
        ax12_pos = AX12.read_current_position()
        ax13_pos = AX13.read_current_position()

        print(
            f"현재: XM10={xm10_pos:4d}, AX11={ax11_pos:4d}, AX12={ax12_pos:4d}, AX13={ax13_pos:4d}"
        )
        print(
            f"목표: XM10={xm10_t:4d}, AX11={ax11_t:4d}, AX12={ax12_t:4d}, AX13={ax13_t:4d}"
        )

        arrived = (
            abs(xm10_pos - xm10_t) < THRESHOLD_XM and
            abs(ax11_pos - ax11_t) < THRESHOLD_AX and
            abs(ax12_pos - ax12_t) < THRESHOLD_AX and
            abs(ax13_pos - ax13_t) < THRESHOLD_AX
        )

        if arrived:
            print("목표 위치 도달")
            break

        if time.time() - start_time > timeout_sec:
            print("목표 위치 대기 시간 초과")
            break

        time.sleep(1.0)


def move_xyz(XM10, AX11, AX12, AX13, x, y, z,
             yaw_deg, pitch_deg, orientation="manual",
             use_trajectory=True,
             duration=TRAJ_DURATION, dt=TRAJ_DT):
    """
    Base frame 기준 목표 xyz [mm]로 이동.

    use_trajectory:
        True  -> 현재 관절각에서 목표 관절각까지 5차 함수 trajectory로 이동
        False -> 기존 방식처럼 목표 encoder로 바로 이동 후 move_and_wait()

    예:
        move_xyz(..., use_trajectory=True)   # 부드럽게 이동
        move_xyz(..., use_trajectory=False)  # 바로 이동
    """

    current_angles_deg = read_current_ik_angles(XM10, AX11, AX12, AX13)

    print("===================================")

    if use_trajectory:
        print("[move_xyz: IK + 5차 joint trajectory]")
    else:
        print("[move_xyz: IK + direct move]")

    print(f"현재 IK angle [deg] = {current_angles_deg}")
    print(f"목표 xyz [mm] = [{x}, {y}, {z}]")
    print(f"use_trajectory={use_trajectory}, duration={duration}, dt={dt}")

    target_angles_deg = solve_ik(
        target_xyz=[x, y, z],
        yaw_deg=yaw_deg,
        pitch_deg=pitch_deg,
        current_angles_deg=current_angles_deg,
        orientation=orientation
    )

    print(f"IK 결과 angle [deg] = {target_angles_deg}")

    fk_pos = fk_position(target_angles_deg)
    print(f"FK 확인 위치 [mm] = {fk_pos}")
    print(f"FK position error [mm] = {np.linalg.norm(np.array([x, y, z]) - fk_pos):.4f}")

    xm10_enc = ik_deg_to_xm_enc(target_angles_deg[0])
    ax11_enc = ik_deg_to_ax_enc(target_angles_deg[1])
    ax12_enc = ik_deg_to_ax_enc(target_angles_deg[2])
    ax13_enc = ik_deg_to_ax_enc(target_angles_deg[3])

    print(
        f"최종 encoder target = XM10:{xm10_enc}, AX11:{ax11_enc}, AX12:{ax12_enc}, AX13:{ax13_enc}"
    )
    
    
    print("========== chain link bounds ==========")
    for i, link in enumerate(chain.links):
        b = getattr(link, "bounds", None)

        if b is None:
            print(f"{i}: {link.name}, bounds 없음")
        else:
            print(
                f"{i}: {link.name}, "
                f"bounds rad=({b[0]:.4f}, {b[1]:.4f}), "
                f"bounds deg=({np.rad2deg(b[0]):.2f}, {np.rad2deg(b[1]):.2f})"
            )
    print("=======================================")

    if use_trajectory:
        move_joint_angles_quintic(
            XM10, AX11, AX12, AX13,
            start_angles_deg=current_angles_deg,
            goal_angles_deg=target_angles_deg,
            duration=duration,
            dt=dt,
            print_debug=TRAJ_DEBUG
        )
    else:
        move_and_wait(
            XM10, AX11, AX12, AX13,
            xm10_enc, ax11_enc, ax12_enc, ax13_enc
        )


    xm_now = XM10.read_current_position()
    ax11_now = AX11.read_current_position()
    ax12_now = AX12.read_current_position()
    ax13_now = AX13.read_current_position()

    print("========== 목표 encoder vs 현재 encoder ==========")
    print(f"XM10: target={xm10_enc}, now={xm_now}, error={xm10_enc - xm_now}")
    print(f"AX11: target={ax11_enc}, now={ax11_now}, error={ax11_enc - ax11_now}")
    print(f"AX12: target={ax12_enc}, now={ax12_now}, error={ax12_enc - ax12_now}")
    print(f"AX13: target={ax13_enc}, now={ax13_now}, error={ax13_enc - ax13_now}")
    print("================================================")

# =========================================================
# 그리퍼 제어
# =========================================================
def move_gripper(XC14, gripper_deg):
    """
    XC14 그리퍼 제어.
    gripper_deg = 0이면 기준 위치.
    처음에는 ±20도 정도만 테스트 추천.
    """

    enc = ik_deg_to_xm_enc(gripper_deg)
    enc = clamp(enc, 0, 4095)

    print(f"Gripper move: deg={gripper_deg}, enc={enc}")
    XC14.move(enc)
    time.sleep(0.3)


def open_gripper(XC14):
    move_gripper(XC14, 90) 
    
def close_and_open_gripper(XC14):
    move_gripper(XC14, 30) # 원래 20
    
def close_gripper(XC14):
    # 처음에는 크게 닫지 말고 20도만 테스트
    move_gripper(XC14, -80)
    
def direction_from_yaw_pitch(yaw_deg, pitch_deg):
    yaw = np.deg2rad(yaw_deg)
    pitch = np.deg2rad(pitch_deg)

    direction = np.array([
        np.cos(pitch) * np.cos(yaw),
        np.cos(pitch) * np.sin(yaw),
        np.sin(pitch),
    ], dtype=float)

    return direction / np.linalg.norm(direction)


# =========================================================
# 초기 위치 이동
# =========================================================
def move_initial_position(XM10, AX11, AX12, AX13, XC14):
    """
    IK 기준 초기 각도:
    XM10 = 0deg -> enc 2048
    AX = 0deg -> enc 약 512
    XC14 = 0deg -> enc 2048
    """

    xm10_enc = ik_deg_to_xm_enc(0.0)
    ax11_enc = ik_deg_to_ax_enc(0)
    ax12_enc = ik_deg_to_ax_enc(40)
    ax13_enc = ik_deg_to_ax_enc(90)
    xc14_enc = ik_deg_to_xm_enc(90)

    print("초기 위치로 이동")
    print(
        f"init enc = XM10:{xm10_enc}, AX11:{ax11_enc}, AX12:{ax12_enc}, AX13:{ax13_enc}, XC14:{xc14_enc}"
    )

    XM10.move(xm10_enc)
    AX11.move(ax11_enc)
    AX12.move(ax12_enc)
    AX13.move(ax13_enc)
    XC14.move(xc14_enc)

    time.sleep(2.0)
# 2번째 10 멀리3번째 10멀리 6번째는 그대로



# =========================================================
# 메인
# =========================================================
if __name__ == "__main__":

    portHandler = PortHandler(DEVICENAME)

    if not portHandler.openPort():
        print("포트 열기 실패")
        quit()

    if not portHandler.setBaudRate(BAUDRATE):
        print("Baudrate 설정 실패")
        quit()

    XM10 = ControllerXC(portHandler, BAUDRATE, XM10_ID)
    AX11 = ControllerAX(portHandler, BAUDRATE, AX11_ID)
    AX12 = ControllerAX(portHandler, BAUDRATE, AX12_ID)
    AX13 = ControllerAX(portHandler, BAUDRATE, AX13_ID)
    XC14 = ControllerXC(portHandler, BAUDRATE, XC14_ID)

    try:
        # 토크 ON
        XM10.torque_enable()
        AX11.torque_enable()
        AX12.torque_enable()
        AX13.torque_enable()
        XC14.torque_enable()

        # 속도 / 게인 설정
        AX11.set_moving_speed(70) #50
        AX12.set_moving_speed(70)# 50
        AX13.set_moving_speed(70) # 50

        XM10.set_position_p_gain(100) # 80
        XC14.set_position_p_gain(100) # 100

        # 초기 위치
        move_initial_position(XM10, AX11, AX12, AX13, XC14)
        
        # 그리퍼 테스트
        open_gripper(XC14)
        time.sleep(2.0)

        input("초기 위치 확인 후 Enter 입력...")

        # # # # # 목표 위치 이동
        move_xyz(XM10, AX11, AX12, AX13,(distance - 20) * np.cos(theta), (distance - 20) * np.sin(theta), TARGET_Z + 100 , yaw_deg= np.degrees(theta), pitch_deg = -90 , orientation= GRIPPER_ORIENTATION)
    
            
        move_xyz(XM10, AX11, AX12, AX13,(distance + 5) * np.cos(theta), (distance + 5) * np.sin(theta), TARGET_Z - 10 , yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        # 거리 125일때  TARGET_Z - 20
        # 거리 150이상일때 target_z -10(120일때는 됨)

        # 가까이있을 때 
        
        
        close_gripper(XC14)
       
        if distance > 150:
            move_xyz(XM10, AX11, AX12, AX13, max((distance -85), 50) * np.cos(theta), max((distance - 85), 50) * np.sin(theta), TARGET_Z - 10 ,yaw_deg=np.degrees(theta), pitch_deg = -130  ,orientation= GRIPPER_ORIENTATION) # -130
        else:
            move_xyz(XM10, AX11, AX12, AX13, max((distance -85), 50) * np.cos(theta), max((distance - 85), 50) * np.sin(theta), TARGET_Z ,yaw_deg=np.degrees(theta), pitch_deg = -130  ,orientation= GRIPPER_ORIENTATION) # -130
        
        
        open_gripper(XC14)
       
        
        move_xyz(XM10, AX11, AX12, AX13, max((distance-105), 30) * np.cos(theta), max((distance - 105),30) * np.sin(theta), TARGET_Z, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        move_xyz(XM10, AX11, AX12, AX13, max((distance - 35), 100) * np.cos(theta), max((distance - 35), 100) * np.sin(theta), TARGET_Z+60, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
    
    
        # 100일때 한번 더하는 거  찐 100일때는 최소 30 
        # move_xyz(XM10, AX11, AX12, AX13,(distance + 5) * np.cos(theta), (distance + 5) * np.sin(theta), TARGET_Z - 20 , yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        # move_xyz(XM10, AX11, AX12, AX13, max((distance -85), 30) * np.cos(theta), max((distance - 85), 30) * np.sin(theta), TARGET_Z+10,yaw_deg=np.degrees(theta), pitch_deg = -110  ,orientation= GRIPPER_ORIENTATION) # -130
        
        # move_xyz(XM10, AX11, AX12, AX13, max((distance-105), 30) * np.cos(theta), max((distance - 105),30) * np.sin(theta), TARGET_Z, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        # move_xyz(XM10, AX11, AX12, AX13, max((distance - 35), 100) * np.cos(theta), max((distance - 35), 100) * np.sin(theta), TARGET_Z+50, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        # # ==========
        
        
        move_initial_position(XM10, AX11, AX12, AX13, XC14)

        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 40 + offset_1) * np.cos(theta_2), (distance_2 + 40+ offset_1) * np.sin(theta_2), TARGET_Z_2 + 70 + offset_1_z, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
       
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 40 + offset_1) * np.cos(theta_2), (distance_2 + 40+ offset_1) * np.sin(theta_2), TARGET_Z_2 + offset_1_z, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        close_gripper(XC14) # 첫번째 블럭 집는 거 
      
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2) * np.cos(theta_2), (distance_2) * np.sin(theta_2), TARGET_Z_2 + 50, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
       
        
        move_xyz(XM10, AX11, AX12, AX13,((distance - 20) * np.cos(theta)+(distance_2) * np.cos(theta_2))/2, (distance - 20) * np.sin(theta), TARGET_Z+100 , yaw_deg= np.degrees(theta), pitch_deg = -90 , orientation= GRIPPER_ORIENTATION)
        
        move_xyz(XM10, AX11, AX12, AX13,(distance ) * np.cos(theta), (distance ) * np.sin(theta), TARGET_Z+ 30 , yaw_deg= np.degrees(theta), pitch_deg = -90 , orientation= GRIPPER_ORIENTATION)
        
        
        close_and_open_gripper(XC14) # 첫번째 블럭 놓는거 
       
        
        move_xyz(XM10, AX11, AX12, AX13,(distance - 10) * np.cos(theta), (distance - 10) * np.sin(theta), TARGET_Z+ 60 , yaw_deg= np.degrees(theta), pitch_deg = -90 , orientation= GRIPPER_ORIENTATION)
        
        move_initial_position(XM10, AX11, AX12, AX13, XC14)
      
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 45 + offset_2) * np.cos(theta_2), (distance_2 + 45+ offset_2) * np.sin(theta_2), TARGET_Z_2 + offset_2_z , yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 45+ offset_2) * np.cos(theta_2), (distance_2 + 45+ offset_2) * np.sin(theta_2), TARGET_Z_2-25 + offset_2_z, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        
        close_gripper(XC14) # 두번째 블럭 집는거 
        
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 - 20) * np.cos(theta_2), (distance_2- 20) * np.sin(theta_2), TARGET_Z_2+40, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
       
        
        
        center_xy, left_xy, right_xy = get_drawer_place_points(distance,theta,lateral_offset=30.0, forward_offset=0.0)
        
        move_xyz(XM10, AX11, AX12, AX13,(left_xy[0] + (distance_2 - 20) * np.cos(theta_2))/2 ,left_xy[1], TARGET_Z+ 100 , yaw_deg= np.degrees(theta), pitch_deg = -90 , orientation= GRIPPER_ORIENTATION)
        
        move_xyz(XM10, AX11, AX12, AX13,left_xy[0] ,left_xy[1], TARGET_Z+ 30 , yaw_deg= np.degrees(theta), pitch_deg = -90 , orientation= GRIPPER_ORIENTATION)
      
        
        close_and_open_gripper(XC14) # 두번째 블럭 놓는거 
       
        
        move_xyz(XM10, AX11, AX12, AX13,(distance - 10) * np.cos(theta), (distance - 10) * np.sin(theta), TARGET_Z+ 100 , yaw_deg= np.degrees(theta), pitch_deg = -90 , orientation= GRIPPER_ORIENTATION)
    
        
        move_initial_position(XM10, AX11, AX12, AX13, XC14)
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 70 + offset_3) * np.cos(theta_2), (distance_2 + 70+ offset_3) * np.sin(theta_2), TARGET_Z_2+30 + offset_3_z, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
      
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 70+ offset_3) * np.cos(theta_2), (distance_2 + 70+ offset_3)* np.sin(theta_2), TARGET_Z_2-25 + offset_3_z, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
      
        
        close_gripper(XC14) # 세번째 블럭 집는거
     
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 -20) * np.cos(theta_2), (distance_2 -20) * np.sin(theta_2), TARGET_Z_2 +40 , yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2) * np.cos(theta_2), (distance_2) * np.sin(theta_2), TARGET_Z_2, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        
        move_xyz(XM10, AX11, AX12, AX13,(right_xy[0]+ (distance_2) * np.cos(theta_2))/2,right_xy[1], TARGET_Z+ 100 , yaw_deg= np.degrees(theta), pitch_deg = -90 , orientation= GRIPPER_ORIENTATION)
        
        
        move_xyz(XM10, AX11, AX12, AX13,right_xy[0],right_xy[1], TARGET_Z+ 30 , yaw_deg= np.degrees(theta), pitch_deg = -90 , orientation= GRIPPER_ORIENTATION)
       
        
        close_and_open_gripper(XC14) # 세번째 블럭 놓는거 
       
        
        move_xyz(XM10, AX11, AX12, AX13, (distance-85) * np.cos(theta), (distance - 85) * np.sin(theta), TARGET_Z + 30 ,yaw_deg=np.degrees(theta), pitch_deg = -90  ,orientation= GRIPPER_ORIENTATION)

        open_gripper(XC14)
        
        if distance > 180:
            move_xyz(XM10, AX11, AX12, AX13, max((distance-85), 50) * np.cos(theta), max((distance - 85),50) * np.sin(theta), TARGET_Z - 13  ,yaw_deg=np.degrees(theta), pitch_deg = -120  ,orientation= GRIPPER_ORIENTATION)
        else:
            move_xyz(XM10, AX11, AX12, AX13, max((distance-85), 50) * np.cos(theta), max((distance - 85),50) * np.sin(theta), TARGET_Z  ,yaw_deg=np.degrees(theta), pitch_deg = -120  ,orientation= GRIPPER_ORIENTATION)
        
        if distance > 180:
            move_xyz(XM10, AX11, AX12, AX13,(distance + 10) * np.cos(theta), (distance + 10) * np.sin(theta), TARGET_Z - 8 , yaw_deg= np.degrees(theta), pitch_deg = -60 , orientation= GRIPPER_ORIENTATION)
        else:
            move_xyz(XM10, AX11, AX12, AX13,(distance + 15) * np.cos(theta), (distance + 15) * np.sin(theta), TARGET_Z - 5 , yaw_deg= np.degrees(theta), pitch_deg = -60 , orientation= GRIPPER_ORIENTATION)
        
        # 150mm이상일때 pitch_deg  = -60
        # 125mm일때 pitch_deg = -80

        
        move_xyz(XM10, AX11, AX12, AX13,(distance) * np.cos(theta), (distance) * np.sin(theta), TARGET_Z+50 , yaw_deg= np.degrees(theta), pitch_deg = -90 , orientation= GRIPPER_ORIENTATION)

      
        
        move_initial_position(XM10, AX11, AX12, AX13, XC14)
        
        #==== 여기까지 서랍 열고 박스 3개 넣고 닫는거




        




        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 30 + offset_4) * np.cos(theta_2), (distance_2 + 30 + offset_4) * np.sin(theta_2), TARGET_Z_2 - 15 + offset_4_z, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 50+ offset_4) * np.cos(theta_2), (distance_2 + 50+ offset_4) * np.sin(theta_2), TARGET_Z_2 - 50 + offset_4_z, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        # 35
        
        close_gripper(XC14) # 네번째 블럭 집는 거 
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 - 20) * np.cos(theta_2), (distance_2 - 20) * np.sin(theta_2), TARGET_Z_2 - 10, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        move_xyz(XM10, AX11, AX12, AX13, ((distance) * np.cos(theta) + (distance_2 - 20) * np.cos(theta_2)) / 2 , (distance) * np.sin(theta), TARGET_Z + 100, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        move_xyz(XM10, AX11, AX12, AX13, (distance) * np.cos(theta), (distance) * np.sin(theta), TARGET_Z + 80, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        move_xyz(XM10, AX11, AX12, AX13, (distance + 100 + put_down_offset_1) * np.cos(theta), (distance + 100 + put_down_offset_1) * np.sin(theta), TARGET_Z + 80 + put_down_offset_z_1, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        move_xyz(XM10, AX11, AX12, AX13, (distance+100 + put_down_offset_1) * np.cos(theta), (distance+ 100 + put_down_offset_1) * np.sin(theta), TARGET_Z + 60 + put_down_offset_z_1, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        close_and_open_gripper(XC14) # 4번째 블럭 놓는거 
        
        move_xyz(XM10, AX11, AX12, AX13, (distance + 50+ put_down_offset_1) * np.cos(theta), (distance + 50+ put_down_offset_1) * np.sin(theta), TARGET_Z + 200 + put_down_offset_z_1, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
      
        move_initial_position(XM10, AX11, AX12, AX13, XC14)
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 50 + offset_5) * np.cos(theta_2), (distance_2 +50+ offset_5) * np.sin(theta_2), TARGET_Z_2 + 25 + offset_5_z, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION,)
      
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 70+ offset_5) * np.cos(theta_2), (distance_2 + 70+ offset_5) * np.sin(theta_2), TARGET_Z_2 - 50 + offset_5_z, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
       
        # 55
        
        close_gripper(XC14) # 5번째 블럭 집는 거 
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 - 20) * np.cos(theta_2), (distance_2 - 20) * np.sin(theta_2), TARGET_Z_2 - 10, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
       
        
        move_xyz(XM10, AX11, AX12, AX13, ((distance) * np.cos(theta) + (distance_2 - 20) * np.cos(theta_2))/2, (distance) * np.sin(theta), TARGET_Z + 100, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)

        move_xyz(XM10, AX11, AX12, AX13, (distance) * np.cos(theta), (distance) * np.sin(theta), TARGET_Z + 60, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        move_xyz(XM10, AX11, AX12, AX13, (distance + 75 + put_down_offset_2) * np.cos(theta), (distance + 75 + put_down_offset_2) * np.sin(theta), TARGET_Z + 60 + put_down_offset_z_2, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        move_xyz(XM10, AX11, AX12, AX13, (distance+75+ put_down_offset_2) * np.cos(theta), (distance+75+ put_down_offset_2) * np.sin(theta), TARGET_Z + 50 + put_down_offset_z_2, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        
        close_and_open_gripper(XC14) # 5번째 블럭 놓는거 
        
        move_xyz(XM10, AX11, AX12, AX13, (distance+40+ put_down_offset_2) * np.cos(theta), (distance+40+ put_down_offset_2) * np.sin(theta), TARGET_Z + 200+ put_down_offset_z_2, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
      
        
        move_initial_position(XM10, AX11, AX12, AX13, XC14)
       
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 85 + offset_6) * np.cos(theta_2), (distance_2 + 85+offset_6) * np.sin(theta_2), TARGET_Z_2 + 25 + offset_6_z, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 85+offset_6) * np.cos(theta_2), (distance_2 + 85+offset_6) * np.sin(theta_2), TARGET_Z_2 - 45, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
      
        
        close_gripper(XC14) # 6번째 블럭 집는거 
     
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2- 20) * np.cos(theta_2), (distance_2 - 20) * np.sin(theta_2), TARGET_Z_2 - 10, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION, )
     
        move_xyz(XM10, AX11, AX12, AX13, ((distance) * np.cos(theta) +  (distance_2- 20) * np.cos(theta_2)) / 2, (distance) * np.sin(theta), TARGET_Z + 100, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
       
        move_xyz(XM10, AX11, AX12, AX13, (distance) * np.cos(theta), (distance) * np.sin(theta), TARGET_Z + 70 + put_down_offset_z_3, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        move_xyz(XM10, AX11, AX12, AX13, (distance + 50 + put_down_offset_3) * np.cos(theta), (distance + 50 + put_down_offset_3) * np.sin(theta), TARGET_Z + 70 + put_down_offset_z_3, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
       
        
        move_xyz(XM10, AX11, AX12, AX13, (distance +55 + put_down_offset_3) * np.cos(theta), (distance +55 + put_down_offset_3) * np.sin(theta), TARGET_Z + 50 + put_down_offset_z_3, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
       
        
        close_and_open_gripper(XC14) # 6번째 블럭 놓는거 
        
        move_xyz(XM10, AX11, AX12, AX13, (distance) * np.cos(theta), (distance) * np.sin(theta), TARGET_Z + 200, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
       
        
        move_initial_position(XM10, AX11, AX12, AX13, XC14)
       
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 105 + offset_7) * np.cos(theta_2), (distance_2 + 105+ offset_7) * np.sin(theta_2), TARGET_Z_2 + offset_7_z, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
       
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 105+ offset_7) * np.cos(theta_2), (distance_2 + 105+ offset_7) * np.sin(theta_2), TARGET_Z_2 - 60 + offset_7_z, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        close_gripper(XC14) # 7번째 블럭 집는거 
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 120) * np.cos(theta_2), (distance_2 + 120) * np.sin(theta_2), TARGET_Z_2 - 10, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
       
        
        move_xyz(XM10, AX11, AX12, AX13, ((distance+85) * np.cos(theta) +(distance_2 + 120) * np.cos(theta_2)) / 2 , (distance+85) * np.sin(theta), TARGET_Z + 200, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        move_xyz(XM10, AX11, AX12, AX13, (distance) * np.cos(theta), (distance) * np.sin(theta), TARGET_Z + 120 + put_down_offset_z_4, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        move_xyz(XM10, AX11, AX12, AX13, (distance+90 + put_down_offset_4) * np.cos(theta), (distance+90+put_down_offset_4) * np.sin(theta), TARGET_Z + 120 + put_down_offset_z_4, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        move_xyz(XM10, AX11, AX12, AX13, (distance+90 + put_down_offset_4) * np.cos(theta), (distance+90+put_down_offset_4) * np.sin(theta), TARGET_Z + 90 + put_down_offset_z_4, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
     
        close_and_open_gripper(XC14) # 7번째 블럭 놓는거 
        
        move_xyz(XM10, AX11, AX12, AX13, (distance) * np.cos(theta), (distance) * np.sin(theta), TARGET_Z + 200, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        move_initial_position(XM10, AX11, AX12, AX13, XC14)
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 80 + offset_8) * np.cos(theta_2), (distance_2 + 80+ offset_8) * np.sin(theta_2), TARGET_Z_2 + offset_8_z, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 80+ offset_8) * np.cos(theta_2), (distance_2 + 80+ offset_8) * np.sin(theta_2), TARGET_Z_2 - 60 + offset_8_z, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
       
        close_gripper(XC14)  # 8번째 블럭 집는거 
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 105) * np.cos(theta_2), (distance_2 + 105) * np.sin(theta_2), TARGET_Z_2 - 10, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
     
        
        move_xyz(XM10, AX11, AX12, AX13, 0 , 150, TARGET_Z + 200, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
       
        
        move_xyz(XM10, AX11, AX12, AX13, (distance + put_down_offset_5) * np.cos(theta), (distance + put_down_offset_5) * np.sin(theta), TARGET_Z + 100 + put_down_offset_5, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
     
        
        move_xyz(XM10, AX11, AX12, AX13, (distance+60 + put_down_offset_5) * np.cos(theta), (distance+60 + put_down_offset_5) * np.sin(theta), TARGET_Z + 82 + put_down_offset_z_5, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
      
        close_and_open_gripper(XC14) # 8번째 블럭 놓는거 
        
        move_xyz(XM10, AX11, AX12, AX13, (distance) * np.cos(theta), (distance) * np.sin(theta), TARGET_Z + 200, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        move_initial_position(XM10, AX11, AX12, AX13, XC14)
        
      
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 55 + offset_9) * np.cos(theta_2), (distance_2 + 55 + offset_9) * np.sin(theta_2), TARGET_Z_2 + offset_9_z , yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 55 + offset_9) * np.cos(theta_2), (distance_2 + 55 + offset_9) * np.sin(theta_2), TARGET_Z_2 - 70 + offset_9_z, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
       
        close_gripper(XC14) # 9번째 블럭 집는거 

        
        move_xyz(XM10, AX11, AX12, AX13, (distance_2 + 65) * np.cos(theta_2), (distance_2 + 65) * np.sin(theta_2), TARGET_Z_2 -10, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
       
        move_xyz(XM10, AX11, AX12, AX13, (distance_2- 20) * np.cos(theta_2), (distance_2 - 20) * np.sin(theta_2), TARGET_Z_2 - 10, yaw_deg=np.degrees(theta_2), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        move_xyz(XM10, AX11, AX12, AX13, ((distance+ 65) * np.cos(theta) + (distance_2- 20) * np.cos(theta_2)) / 2, (distance+65) * np.sin(theta), TARGET_Z + 130, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
      
        move_xyz(XM10, AX11, AX12, AX13, (distance) * np.cos(theta), (distance) * np.sin(theta), TARGET_Z + 125, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        

        move_xyz(XM10, AX11, AX12, AX13, (distance+65 + put_down_offset_6) * np.cos(theta), (distance+65+ put_down_offset_6) * np.sin(theta), TARGET_Z + 105 + put_down_offset_z_6, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        close_and_open_gripper(XC14) # 9번째 블럭 놓는거 
        
        move_xyz(XM10, AX11, AX12, AX13, (distance+ put_down_offset_6) * np.cos(theta), (distance+ put_down_offset_6) * np.sin(theta), TARGET_Z + 150 + put_down_offset_z_6, yaw_deg=np.degrees(theta), pitch_deg = -90 ,orientation= GRIPPER_ORIENTATION)
        
        
        
        
        
        
        
        
        # 14+(25/2)*4
        
        # 초기 위치 복귀
        # move_initial_position(XM10, AX11, AX12, AX13, XC14)

        print("동작 완료")

    except KeyboardInterrupt:
        print("\n사용자 중단")

    finally:
        # XM10.torque_disable()
        # AX11.torque_disable()
        # AX12.torque_disable()
        # AX13.torque_disable()
        # XC14.torque_disable()
        
        move_initial_position(XM10, AX11, AX12, AX13, XC14)
        print("동작 완료")