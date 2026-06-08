'''
해야될것
속도 확인
오프셋..
'''

# 0529
# move_xy_at_height  40 -> 더 높게하기 -> 심만놓고했을땐 안됨
# # R=30,20일때 전체 오프셋 확인하기 -> 심만놓고했을땐 훨씬많이찌그러짐...

# 일단테스트해보기, 마지막에 죽 긋는거 수정한거
# 처음에 닿을때도 살짝 그렇긴 함, 근데 고정이 잘안되어서 그런건진 다시 해봐야함

# 컴퓨터 느려져서 속도가 안 나온 거였음... 다시 보기


import numpy as np
import time
import os
from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink
from dynamixel_sdk.controllerXC import *
from dynamixel_sdk.controllerAX import *

# ─── DH 파라미터 ──────────────────────────────────────────
D1 = 127.0 
A2 = 134.5
A3 = 134.5
A4 = 0.0

# ---- 펜 오프셋 --------
PEN_Z   = 34.0
PEN_LEN = 38.0
# 플러스펜일때 38

BOX_DIST_FROM_BASE    = 50 +75 # 베이스 원점~박스 처음 부분(베이스 원점-베이스끝: 75, 베이스끝-박스시작: 72)

# ----- 원 값 ----------
'''1번 원 '''
CIRCLE_OFFSET_FORWARD = 70
CIRCLE_OFFSET_SIDE    = -20   # 중앙기준 왼쪽이 (-), 오른쪽이 (+)
CIRCLE_RADIUS         = 20

'''2번 원'''
CIRCLE_OFFSET_FORWARD2 = 50
CIRCLE_OFFSET_SIDE2    = -30   # 중앙기준 왼쪽이 (-), 오른쪽이 (+)
CIRCLE_RADIUS2         = 40
# ----- y축 오프셋 ---------
CIRCLE_OFFSET_SIDE    = CIRCLE_OFFSET_SIDE -2 +4

# ----- x축 오프셋 -----
''' R=40일때, x 5.3 +3, y -2 
    R=30일때,'''
''' 0529 다시할때, x+
x 5.3 +3, y -2 '''

# if CIRCLE_RADIUS == 20:
#     forward_offset = 5 # 이거는 수정 해야함, 테스트 안함
# else:
forward_offset = 5.3 +3 # 원래 4였음
CIRCLE_OFFSET_FORWARD = CIRCLE_OFFSET_FORWARD + forward_offset
CIRCLE_OFFSET_FORWARD2 = CIRCLE_OFFSET_FORWARD2 + forward_offset

# ---- 베이스로부터 거리 계산 ---
CIRCLE_CENTER_X = -(BOX_DIST_FROM_BASE + CIRCLE_OFFSET_FORWARD)
CIRCLE_CENTER_Y = CIRCLE_OFFSET_SIDE
CIRCLE_CENTER_Z = 10

CIRCLE_CENTER_X2 = -(BOX_DIST_FROM_BASE + CIRCLE_OFFSET_FORWARD2)
CIRCLE_CENTER_Y2 = CIRCLE_OFFSET_SIDE2

NUM_POINTS = 180 # 350

circle_speed = 100 # 50

OFFSETS = np.array([
    [0, 0],  # 1번  180°          # 상단
    [0, 0],  # 2번  157.5°
    [0, 0],  # 3번  135°
    [0, 0],  # 4번  112.5° 

    [0, 0],  # 5번  90°            # 오른쪽
    [0, 0],  # 6번  67.5°
    [0.0, 0],  # 7번  45°
    [0, 0],  # 8번  22.5°

    [0, 0.0],  # 9번  0°            # 하단
    [0, 0],  # 10번 -22.5°
    [0, 0],  # 11번 -45°
    [0, 0],  # 12번 -67.5°        

    [0, 0],  # 13번 -90°        # 왼쪽
    [0, 0],  # 14번 -112.5°
    [0.0, 0.0],  # 15번 -135°
    [0.0, 0.0],  # 16번 -157.5°
])

OFFSETS2 = np.array([
    [0, 0],  # 1번  180°          # 상단
    [0, 0],  # 2번  157.5°
    [0, 0],  # 3번  135°
    [0, 0],  # 4번  112.5° 

    [0, 0],  # 5번  90°            # 오른쪽
    [0, 0],  # 6번  67.5°
    [0.0, 0],  # 7번  45°
    [0, 0],  # 8번  22.5°

    [0, 0.0],  # 9번  0°            # 하단
    [0, 0],  # 10번 -22.5°
    [0, 0],  # 11번 -45°
    [0, 0],  # 12번 -67.5°        

    [0, 0],  # 13번 -90°        # 왼쪽
    [0, 0],  # 14번 -112.5°
    [0.0, 0.0],  # 15번 -135°
    [0.0, 0.0],  # 16번 -157.5°
])

# ─── 포트 설정 ────────────────────────────────────────────
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE   = 115200
XM1_ID, AX1_ID, AX2_ID, AX3_ID = 10, 11, 12, 13
THRESHOLD  = 7

# ─── ikpy 체인 ───────────────────────────────────────────
chain = Chain(name='robot_arm', links=[
    OriginLink(),
    DHLink(theta=0,        d=D1,    a=0,      alpha=np.pi/2),
    DHLink(theta=np.pi/2,  d=0,     a=A2,     alpha=0,
           bounds=(np.radians(-150), np.radians(225-150))),

    DHLink(theta=0,        d=0,     a=A3,     alpha=0),
    DHLink(theta=0, d=0, a=PEN_Z, alpha=0,
       bounds=(np.radians(-150), np.radians(150))),
    DHLink(theta=-np.pi/2, d=0, a=-PEN_LEN, alpha=-np.pi/2),
], active_links_mask=[False, True, True, True, True, False])

TARGET_ORIENTATION = np.array([
    [ 0,  0, -1],
    [ 0,  1,  0],
    [ 1,  0,  0]
])

INIT_ANGLES = [0, 0, np.radians(75-90), np.radians(72), np.radians(-58), 0]

# ─── 인코더 변환 ──────────────────────────────────────────
def deg_to_xm_enc(deg):
    '''xm 180도 기준'''
    return int((deg / 360.0) * 4095 + 2048)

def deg_to_ax_enc(deg):
    '''ax 150도 기준'''
    motor_deg = deg + 150.0
    return int(motor_deg / 300.0 * 1023)


# ─── IK -> 인코더 변환 ─────────────────────────────────────
def ik_to_encoders(target_xyz, prev_angles=None):
    x, y, z = target_xyz

    if prev_angles is None:
        init = INIT_ANGLES
    else:
        init = [0, prev_angles[0], prev_angles[1], prev_angles[2], prev_angles[3], 0]

    angles = chain.inverse_kinematics(
        target_position=[x, y, z],
        target_orientation=TARGET_ORIENTATION,
        # target_orientation=None,
        orientation_mode="all",
        initial_position=init
    )

    t1 = np.degrees(angles[1])
    t2 = np.degrees(angles[2])
    t3 = np.degrees(angles[3])
    # t4 = np.degrees(angles[4])
    t4 = 90.0 - t2 - t3 # 큰 차이는 없음

    xm1_enc = deg_to_xm_enc(t1)
    ax1_enc = deg_to_ax_enc(t2)
    ax2_enc = deg_to_ax_enc(t3)
    ax3_enc = deg_to_ax_enc(t4)

    return xm1_enc, ax1_enc, ax2_enc, ax3_enc, (t1, t2, t3, t4), [angles[1], angles[2], angles[3], angles[4]]


def cal_z(x, y):
    ''' x 20까지는 7
    20~ 100까지는 10 ㄱㅊ
     100 ~ 120  12?
     140일때 14 정도  '''
    
    D = abs(x) - BOX_DIST_FROM_BASE
    # print(D)

    if D < 20:
        z= 8
    elif 20 <= D < 30:
        z= 10
    elif 30 <= D < 40:
        z= 10
    elif 40 <= D < 60:
        z= 12
    elif 60 <= D < 100:
        z= 12
    elif 100<= D < 120:
        z= 17
    elif 120<= D < 130:
        z= 19
    else:
        z= 21

    z = z-2


    # # R = 40에서 보정
    # if 50< abs(y) < 60 and D > 60:
    #     z+= 2
    
    # if 60< abs(y) < 70 and D > 100:
    #     z+= 2

    # if 30 < D < 40 and 35< abs(y) <45:
    #     z -= 3
    

    return z


   

# ─── 원 경로 생성 ───────────────────────────────────────── 

def cubic(OFFSETS):

    # x값만 부호 뒤집기
    OFFSETS[:, 0] = -OFFSETS[:, 0]

    ''' 쿠빅스플라인'''
    from scipy.interpolate import CubicSpline

    # points_1: 0→π 방향으로 정의
    angles_1 = np.linspace(0, np.pi, 9)        # 증가 ✓
    cs_dx_1 = CubicSpline(angles_1, OFFSETS[0:9, 0][::-1])  # 9번→1번 순서로 뒤집기
    cs_dy_1 = CubicSpline(angles_1, OFFSETS[0:9, 1][::-1])

    # points_2: π→2π 방향으로 정의
    angles_2 = np.linspace(np.pi, 2*np.pi, 8)  # 증가 ✓
    cs_dx_2 = CubicSpline(angles_2, OFFSETS[15:7:-1, 0])
    cs_dy_2 = CubicSpline(angles_2, OFFSETS[15:7:-1, 1])

    return cs_dx_1, cs_dy_1, cs_dx_2, cs_dy_2



def generate_circle(OFFSETS, CIRCLE_RADIUS):
    cs_dx_1, cs_dy_1, cs_dx_2, cs_dy_2 = cubic(OFFSETS)

    points_1 = []
    for i in range(NUM_POINTS // 2 + 1):
        angle = np.pi - np.pi * i / (NUM_POINTS // 2)
        x = CIRCLE_CENTER_X + CIRCLE_RADIUS * np.cos(angle)
        y = CIRCLE_CENTER_Y + CIRCLE_RADIUS * np.sin(angle)
        z = cal_z(x,y)
        dx = float(cs_dx_1(angle))
        dy = float(cs_dy_1(angle))
        points_1.append((x + dx, y + dy, z))

    EXTRA = 12
    points_2 = []
    for i in range(NUM_POINTS // 2 + 1 + EXTRA):
        angle = np.pi + np.pi * i / (NUM_POINTS // 2)
        x = CIRCLE_CENTER_X + CIRCLE_RADIUS * np.cos(angle)
        y = CIRCLE_CENTER_Y + CIRCLE_RADIUS * np.sin(angle)
        z = cal_z(x,y)
        dx = float(cs_dx_2(angle))
        dy = float(cs_dy_2(angle))
        points_2.append((x + dx, y + dy, z))

    return points_1, points_2



def move_xy_at_height(XM1, AX1, AX2, AX3, x, y, z_high,SPEED = 50):
    
    AX1.set_moving_speed(SPEED)
    AX2.set_moving_speed(SPEED)
    AX3.set_moving_speed(SPEED)
    xm1_e, ax1_e, ax2_e, ax3_e, _, _ = ik_to_encoders((x, y, z_high))
    move_and_wait(XM1, AX1, AX2, AX3, xm1_e, ax1_e, ax2_e, ax3_e)

def move_z_down(XM1, AX1, AX2, AX3, x, y, z):
    SPEED = 10
    AX1.set_moving_speed(SPEED)
    AX2.set_moving_speed(SPEED)
    AX3.set_moving_speed(SPEED)

    xm1_s = XM1.read_current_position()
    ax1_s = AX1.read_current_position()
    ax2_s = AX2.read_current_position()
    ax3_s = AX3.read_current_position()

    xm1_e, ax1_e, ax2_e, ax3_e, _, _ = ik_to_encoders((x, y, z))

    steps = 8
    for i in range(steps + 1):
        print('step중임')
        t = i / steps
        xm1_t = int(xm1_s + t * (xm1_e - xm1_s))
        ax1_t = int(ax1_s + t * (ax1_e - ax1_s))
        ax2_t = int(ax2_s + t * (ax2_e - ax2_s))
        ax3_t = int(ax3_s + t * (ax3_e - ax3_s))
        move_and_wait(XM1, AX1, AX2, AX3, xm1_t, ax1_t, ax2_t, ax3_t)
    print('step 끝')

    # AX1.set_moving_speed(5)
    # AX2.set_moving_speed(5)
    # AX3.set_moving_speed(5)
    print('속도 끝')


def move_z_up(XM1, AX1, AX2, AX3, x, y, z):
    AX1.set_moving_speed(20)
    AX2.set_moving_speed(20)
    AX3.set_moving_speed(20)

    xm1_s = XM1.read_current_position()
    ax1_s = AX1.read_current_position()
    ax2_s = AX2.read_current_position()
    ax3_s = AX3.read_current_position()

    xm1_e, ax1_e, ax2_e, ax3_e, _, _ = ik_to_encoders((x, y, z + 60))

    steps = 5
    for i in range(steps + 1):
        t = i / steps
        xm1_t = int(xm1_s + t * (xm1_e - xm1_s))
        ax1_t = int(ax1_s + t * (ax1_e - ax1_s))
        ax2_t = int(ax2_s + t * (ax2_e - ax2_s))
        ax3_t = int(ax3_s + t * (ax3_e - ax3_s))
        move_and_wait(XM1, AX1, AX2, AX3, xm1_t, ax1_t, ax2_t, ax3_t)
        

# ─── 목표 위치로 이동 및 대기 ──────────────────────────────
def move_and_wait(XM1, AX1, AX2, AX3, xm1_t, ax1_t, ax2_t, ax3_t):
    XM1.move(xm1_t)
    AX1.move(ax1_t)
    AX2.move(ax2_t)
    AX3.move(ax3_t)

    timeout = time.time() + 5.0
    # start = time.time()
    while True:
        xm1_pos = XM1.read_current_position()
        ax1_pos = AX1.read_current_position()
        ax2_pos = AX2.read_current_position()
        ax3_pos = AX3.read_current_position()

        if (abs(xm1_pos - xm1_t) < THRESHOLD and
            abs(ax1_pos - ax1_t) < THRESHOLD and
            abs(ax2_pos - ax2_t) < THRESHOLD and
            abs(ax3_pos - ax3_t) < THRESHOLD):
            # print(f'대기시간: {time.time()-start:.3f}')
            break

        if time.time() > timeout:
            print(f"타임아웃 - 현재:({xm1_pos},{ax1_pos},{ax2_pos},{ax3_pos}) 목표:({xm1_t},{ax1_t},{ax2_t},{ax3_t})")
            break
        time.sleep(0.003)



# ─── 메인 ────────────────────────────────────────────────
if __name__ == "__main__":

    print("=== 1번 원 설정 ===")
    print(f"원 위치:    박스 밑변 중앙 기준 앞 {CIRCLE_OFFSET_FORWARD}mm / 옆 {CIRCLE_OFFSET_SIDE}mm")
    print(f"원 반지름:  {CIRCLE_RADIUS}mm")
    print(f"원 중심 좌표: ({CIRCLE_CENTER_X:.1f}, {CIRCLE_CENTER_Y:.1f}, {CIRCLE_CENTER_Z:.1f})\n")

    print("=== 2번 원 설정 ===")
    print(f"원 위치:    박스 밑변 중앙 기준 앞 {CIRCLE_OFFSET_FORWARD2}mm / 옆 {CIRCLE_OFFSET_SIDE2}mm")
    print(f"원 반지름:  {CIRCLE_RADIUS2}mm")
    print(f"원 중심 좌표: ({CIRCLE_CENTER_X2:.1f}, {CIRCLE_CENTER_Y2:.1f}, {CIRCLE_CENTER_Z:.1f})\n")

    portHandler = PortHandler(DEVICENAME)
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)

    XM1 = ControllerXC(portHandler, BAUDRATE, XM1_ID)
    AX1 = ControllerAX(portHandler, BAUDRATE, AX1_ID)
    AX2 = ControllerAX(portHandler, BAUDRATE, AX2_ID)
    AX3 = ControllerAX(portHandler, BAUDRATE, AX3_ID)

    def enable_torque_with_retry(controller, port, motor_id, addr_torque, max_retry=3):
        for i in range(max_retry):
            controller.torque_enable()
            time.sleep(0.05)
            val, _, _ = controller.packetHandler.read1ByteTxRx(port, motor_id, addr_torque)
            if val == 1:
                # print(f"ID {motor_id} 토크f ON 성공")
                return True
        print(f"ID {motor_id} 토크 ON 실패")
        return False

    enable_torque_with_retry(XM1, portHandler, XM1_ID, 64)
    enable_torque_with_retry(AX1, portHandler, AX1_ID, 24)
    enable_torque_with_retry(AX2, portHandler, AX2_ID, 24)
    enable_torque_with_retry(AX3, portHandler, AX3_ID, 24)

    print(f"토크 ON 성공")

    AX1.set_moving_speed(30)
    AX2.set_moving_speed(30)
    AX3.set_moving_speed(30)
    XM1.set_position_p_gain(800) # 조정하기 -> 근데 더낮추면 XM에서 타임아웃

    # IK 사전 계산
    print("=== IK 사전 계산 중 ===")
    points_1, points_2 = generate_circle(OFFSETS, CIRCLE_RADIUS)
    encoder_path_1 = []
    encoder_path_2 = []

    points_12, points_22 = generate_circle(OFFSETS2, CIRCLE_RADIUS2)
    encoder_path_12 = []
    encoder_path_22 = []

    ######### 첫번째 원  

    def generate_list(points, encoder_path,prev_angles = None):
        for x, y, z in points:
            xm1_e, ax1_e, ax2_e, ax3_e, angles, prev_angles = ik_to_encoders((x, y, z), prev_angles)
            encoder_path.append((xm1_e, ax1_e, ax2_e, ax3_e))
        return encoder_path
    
    encoder_path_1 = generate_list(points_1,encoder_path_1)
    encoder_path_2 = generate_list(points_2,encoder_path_2)
    encoder_path_12 = generate_list(points_12,encoder_path_12)
    encoder_path_22 = generate_list(points_22,encoder_path_22)

    # for x, y, z in points_1:
    #     xm1_e, ax1_e, ax2_e, ax3_e, angles, prev_angles = ik_to_encoders((x, y, z), prev_angles)
    #     encoder_path_1.append((xm1_e, ax1_e, ax2_e, ax3_e))

    # prev_angles = None
    # for x, y, z in points_2: 
    #     xm1_e, ax1_e, ax2_e, ax3_e, angles, prev_angles = ik_to_encoders((x, y, z), prev_angles)
    #     encoder_path_2.append((xm1_e, ax1_e, ax2_e, ax3_e))

    # prev_angles = None
    # ######### 두번째 원 
    # for x, y, z in points_12:
    #     xm1_e, ax1_e, ax2_e, ax3_e, angles, prev_angles = ik_to_encoders((x, y, z), prev_angles)
    #     encoder_path_1.append((xm1_e, ax1_e, ax2_e, ax3_e))

    # prev_angles = None
    # for x, y, z in points_22: 
    #     xm1_e, ax1_e, ax2_e, ax3_e, angles, prev_angles = ik_to_encoders((x, y, z), prev_angles)
    #     encoder_path_2.append((xm1_e, ax1_e, ax2_e, ax3_e))


    input("\n Enter 입력하면 그리기 시작")

    try:
        ############################################################################################## 1번 원
        
        # ADDR_AX_MOVING_SPEED = 32
        # AX1.set_moving_speed(100)
        # speed_val, result, error = AX1.packetHandler.read2ByteTxRx(AX1.port_handler, AX1.motor_id, ADDR_AX_MOVING_SPEED)
        # print(f"현재 speed 레지스터 값: {speed_val}")
        x0, y0, _ = points_1[0]
        z0 = cal_z(x0, y0)

        # 1. 시작점 xy로 이동 (z+60)
        print(" 첫번째 시작점 이동 ")
        ### 3번째 ax 먼저 이동하고 나서 
        move_xy_at_height(XM1, AX1, AX2, AX3, x0, y0, z0 + 150)
        time.sleep(0.5)
        move_xy_at_height(XM1, AX1, AX2, AX3, x0, y0, z0 + 40)

        first_s = 10 # 바로 서클 스피드로 가면 불안정
        AX1.set_moving_speed(first_s)
        AX2.set_moving_speed(first_s)
        AX3.set_moving_speed(first_s)
        # 하고나서 슬립 하지 말기


        print(" 첫번째 시작점으로 이동 끝 ")

        #########################################  첫 번째 반원 시작
        for i, (xm1_e, ax1_e, ax2_e, ax3_e) in enumerate(encoder_path_1):
            if i == int(5): #NUM_POINTS/2/30
                AX1.set_moving_speed(circle_speed)
                AX2.set_moving_speed(circle_speed)
                AX3.set_moving_speed(circle_speed)
                # time.sleep(1)

            print(f"[반원1 {i+1:3d}/{len(encoder_path_1)}] 이동 중...")
            move_and_wait(XM1, AX1, AX2, AX3, xm1_e, ax1_e, ax2_e, ax3_e)

        # 반원 끝난 후 AX1만 먼저 올리기
        final_up_s = 30
        AX1.set_moving_speed(final_up_s)
        AX2.set_moving_speed(final_up_s)
        AX3.set_moving_speed(final_up_s)
        ax3_cur = AX3.read_current_position()
        ax1_cur = AX1.read_current_position()
        AX3.move(ax3_cur - 30)
        AX1.move(ax1_cur - 20)
        
        time.sleep(0.5)  # 도달할 시간만 줌
        print(f"이전:{ax1_cur}, 현재 {AX1.read_current_position()}")

        AX1.set_moving_speed(first_s)
        AX2.set_moving_speed(first_s)
        AX3.set_moving_speed(first_s)

        # # 4. z축으로만 올라가기
        cur_x, cur_y, _ = points_1[-1]
        cur_z = cal_z(cur_x, cur_y)
        move_z_up(XM1, AX1, AX2, AX3, cur_x, cur_y, cur_z+3)

        ############## 두번쨰로 이동
        # 5. 다음 시작점 xy로 이동 
        print(" 두번째 시작점으로 이동 ")
        next_x, next_y, _ = points_2[0]
        next_z = cal_z(next_x,next_y)
        move_xy_at_height(XM1, AX1, AX2, AX3, next_x, next_y, next_z + 150) # 이거 낮게 하면 움직일때 그림, 낮추지 말기 
        time.sleep(0.5)
        s = 20
        AX1.set_moving_speed(s)
        AX2.set_moving_speed(s)
        AX3.set_moving_speed(s)
        move_xy_at_height(XM1, AX1, AX2, AX3, x0, y0, z0 + 30,20)

        print(" 두번째 시작점으로 이동 끝 ")

        ###################################   두 번째 반원 시작
        for i, (xm1_e, ax1_e, ax2_e, ax3_e) in enumerate(encoder_path_2):
            if i == int(NUM_POINTS/2/30):
                AX1.set_moving_speed(circle_speed)
                AX2.set_moving_speed(circle_speed)
                AX3.set_moving_speed(circle_speed)
            print(f"[반원2 {i+1:3d}/{len(encoder_path_2)}] 이동 중...")
            move_and_wait(XM1, AX1, AX2, AX3, xm1_e, ax1_e, ax2_e, ax3_e)
    
        # 반원 끝난 후 모터만 올림
        # final_up_s
        AX1.set_moving_speed(final_up_s)
        AX2.set_moving_speed(final_up_s)
        AX3.set_moving_speed(final_up_s)
        ax3_cur = AX3.read_current_position()
        ax1_cur = AX1.read_current_position()
        AX3.move(ax3_cur - 30)
        AX1.move(ax1_cur - 20)
        time.sleep(0.5)
        print(f"이전:{ax1_cur}, 현재 {AX1.read_current_position()}")
        
        
        # # time.sleep(2)  # 도달할 시간만 줌
        # print(f"이전:{ax1_cur}, 현재 {AX1.read_current_position()}")

        # AX1.set_moving_speed(first_s)
        # AX2.set_moving_speed(first_s)
        # AX3.set_moving_speed(first_s)
        # 8. z축으로만 올라가기
        print('올라가기')
        last_x, last_y, _ = points_2[-1]
        last_z = cal_z(last_x, last_y)
        move_z_up(XM1, AX1, AX2, AX3, last_x, last_y, last_z+3)

        ############################################################################################################################################# 2번원
        AX1.set_moving_speed(30)
        AX2.set_moving_speed(30)
        AX3.set_moving_speed(30)

        x0, y0, _ = points_12[0]
        z0 = cal_z(x0, y0)

        # 1. 시작점 xy로 이동 (z+60)
        print(" 첫번째 시작점 이동 ")
        move_xy_at_height(XM1, AX1, AX2, AX3, x0, y0, z0 + 150)
        time.sleep(0.5)
        
        move_xy_at_height(XM1, AX1, AX2, AX3, x0, y0, z0 + 40)

        # first_s = 15 # 바로 서클 스피드로 가면 불안정
        AX1.set_moving_speed(first_s)
        AX2.set_moving_speed(first_s)
        AX3.set_moving_speed(first_s)
        # 하고나서 슬립 하지 말기

        # circle_speed = 50 # 위에서 정의

        print(" 원 2 첫번째 시작점으로 이동 끝 ")

        #########################################  첫 번째 반원 시작
        for i, (xm1_e, ax1_e, ax2_e, ax3_e) in enumerate(encoder_path_12):
            if i == int(NUM_POINTS/2/30):
                AX1.set_moving_speed(circle_speed)
                AX2.set_moving_speed(circle_speed)
                AX3.set_moving_speed(circle_speed)

            print(f"[원 2의 반원1 {i+1:3d}/{len(encoder_path_1)}] 이동 중...")
            move_and_wait(XM1, AX1, AX2, AX3, xm1_e, ax1_e, ax2_e, ax3_e)

        # 반원 끝난 후
        # 1. AX1만 먼저 올리기
        # final_up_s = 20
        AX1.set_moving_speed(final_up_s)
        AX2.set_moving_speed(final_up_s)
        AX3.set_moving_speed(final_up_s)
        ax3_cur = AX3.read_current_position()
        ax1_cur = AX1.read_current_position()
        AX3.move(ax3_cur - 30)
        AX1.move(ax1_cur - 20)
        
        time.sleep(0.5)  # 도달할 시간만 줌
        print(f"이전:{ax1_cur}, 현재 {AX1.read_current_position()}")

        AX1.set_moving_speed(first_s)
        AX2.set_moving_speed(first_s)
        AX3.set_moving_speed(first_s)

        # # 4. z축으로만 올라가기
        cur_x, cur_y, _ = points_12[-1]
        cur_z = cal_z(cur_x, cur_y)
        move_z_up(XM1, AX1, AX2, AX3, cur_x, cur_y, cur_z+3)

        ############## 두번쨰로 이동
        # 5. 다음 시작점 xy로 이동 (z+60)
        print(" 두번째 시작점으로 이동 ")
        next_x, next_y, _ = points_22[0]
        next_z = cal_z(next_x,next_y)
        move_xy_at_height(XM1, AX1, AX2, AX3, next_x, next_y, next_z + 150) # 이거 낮게 하면 움직일때 그림, 낮추지 말기 
        time.sleep(0.5)
        s = 20
        AX1.set_moving_speed(s)
        AX2.set_moving_speed(s)
        AX3.set_moving_speed(s)
        move_xy_at_height(XM1, AX1, AX2, AX3, x0, y0, z0 + 30,20)

        print(" 두번째 시작점으로 이동 끝 ")


        ########################################################################################   두 번째 반원 시작
        for i, (xm1_e, ax1_e, ax2_e, ax3_e) in enumerate(encoder_path_22):
            if i == int(NUM_POINTS/2/30):
                AX1.set_moving_speed(circle_speed)
                AX2.set_moving_speed(circle_speed)
                AX3.set_moving_speed(circle_speed)
            print(f"[두번째 반원2 {i+1:3d}/{len(encoder_path_2)}] 이동 중...")
            move_and_wait(XM1, AX1, AX2, AX3, xm1_e, ax1_e, ax2_e, ax3_e)
    
        # 모터만 올림
        # final_up_s
        AX1.set_moving_speed(final_up_s)
        AX2.set_moving_speed(final_up_s)
        AX3.set_moving_speed(final_up_s)
        ax3_cur = AX3.read_current_position()
        ax1_cur = AX1.read_current_position()
        AX3.move(ax3_cur - 30)
        AX1.move(ax1_cur - 20)
        time.sleep(0.5)
        print(f"이전:{ax1_cur}, 현재 {AX1.read_current_position()}")
        
        
        # # time.sleep(2)  # 도달할 시간만 줌
        # print(f"이전:{ax1_cur}, 현재 {AX1.read_current_position()}")

        # AX1.set_moving_speed(first_s)
        # AX2.set_moving_speed(first_s)
        # AX3.set_moving_speed(first_s)
        # 8. z축으로만 올라가기
        print('올라가기')
        last_x, last_y, _ = points_22[-1]
        last_z = cal_z(last_x, last_y)
        move_z_up(XM1, AX1, AX2, AX3, last_x, last_y, last_z+3)

        print("원 그리기 완료")


        time.sleep(20)

    except KeyboardInterrupt:
        print("\n중단")
    finally:
        XM1.torque_disable()
        AX1.torque_disable()
        AX2.torque_disable()
        AX3.torque_disable()
        portHandler.closePort()