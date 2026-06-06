"""
SrOmBURo direction consistency diagnostic

순서대로 실행:
  [1] IMU 방향 확인  — 로봇을 기울여서 각도 부호가 맞는지 확인
  [2] 엔코더 방향 확인 — 바퀴를 손으로 돌려서 속도 부호가 맞는지 확인
  [3] 제어 방향 확인  — 기울었을 때 속도 명령 부호가 복원 방향인지 확인

실행: python3 test_directions.py [1|2|3]
      인자 없으면 세 가지 모두 순서대로 실행
"""

import math
import struct
import sys
import threading
import time

import serial

sys.path.insert(0, "/home/omburo/Documents/SROmBURo")
from Omburo import Omburo

# control.py와 동일한 설정값
from control import (
    IMU_PORT, IMU_BAUD, IMU_RATE_HZ,
    ROLL_EU_IDX, ROLL_EU_SIGN, PITCH_EU_IDX, PITCH_EU_SIGN,
    ROLLDOT_IDX, ROLLDOT_SIGN, PITCHDOT_IDX, PITCHDOT_SIGN,
    N_ROLLER, K11, K12, K13, H1, K21, K22, K23, H2,
    _imu_thread, IMUState,
)

SAMPLE_HZ = 20   # 화면 갱신 주기

# ── IMU 읽기 ─────────────────────────────────────────────────────────────────
def start_imu():
    state = IMUState()
    t = threading.Thread(target=_imu_thread, args=(state,), daemon=True)
    t.start()
    print("IMU 연결 대기 중...", end="", flush=True)
    if not state.updated.wait(timeout=8.0):
        print(" 실패 (연결 확인)")
        sys.exit(1)
    print(" 완료\n")
    return state


# ── 테스트 1: IMU 방향 ────────────────────────────────────────────────────────
def test_imu(imu: IMUState):
    print("=" * 60)
    print("테스트 1: IMU 방향 확인")
    print("=" * 60)
    print("확인 방법:")
    print("  ① 로봇을 앞으로 기울이면 → pitch(θ2)가 양수(+)여야 함")
    print("  ② 로봇을 오른쪽으로 기울이면 → roll(θ1)이 양수(+)여야 함")
    print("  방향이 반대면 control.py의 ROLL_EU_SIGN / PITCH_EU_SIGN 부호를 -1로 변경")
    print("\nCtrl+C 를 누르면 다음 테스트로\n")

    fmt = ("roll(θ1)={:+7.2f}°  pitch(θ2)={:+7.2f}°  "
           "rolldot={:+6.2f} rad/s  pitchdot={:+6.2f} rad/s")
    try:
        while True:
            euler, gyro = imu.get()
            roll     = ROLL_EU_SIGN  * euler[ROLL_EU_IDX]
            pitch    = PITCH_EU_SIGN * euler[PITCH_EU_IDX]
            rolldot  = ROLLDOT_SIGN  * gyro[ROLLDOT_IDX]
            pitchdot = PITCHDOT_SIGN * gyro[PITCHDOT_IDX]
            print("\r" + fmt.format(roll, pitch, rolldot, pitchdot), end="", flush=True)
            time.sleep(1 / SAMPLE_HZ)
    except KeyboardInterrupt:
        print("\n")


# ── 테스트 2: 엔코더 방향 ────────────────────────────────────────────────────
def test_encoders(robot: Omburo):
    print("=" * 60)
    print("테스트 2: 엔코더(모터 속도) 방향 확인")
    print("=" * 60)
    print("확인 방법:")
    print("  ① 바퀴(wheel)를 앞으로 굴리면 → vel_w가 양수(+)여야 함")
    print("  ② 롤러(roller)를 오른쪽으로 밀면 → vel_r가 양수(+)여야 함")
    print("  방향이 반대면 Omburo.readback() 에서 해당 값에 -1 곱하기")
    print("\nCtrl+C 를 누르면 다음 테스트로\n")

    fmt = "vel_w(wheel)={:+7.3f} rad/s  vel_r(roller)={:+7.3f} rad/s"
    try:
        while True:
            _, vel_w, _, vel_r = robot.readback()
            phi1dot = vel_w
            phi2dot = N_ROLLER * (vel_w + vel_r)
            print("\r" + fmt.format(vel_w, vel_r) +
                  f"  │  φ̇1={phi1dot:+6.3f}  φ̇2={phi2dot:+6.3f}",
                  end="", flush=True)
            time.sleep(1 / SAMPLE_HZ)
    except KeyboardInterrupt:
        print("\n")


# ── 테스트 3: 제어 방향 ──────────────────────────────────────────────────────
def test_control(imu: IMUState, robot: Omburo):
    print("=" * 60)
    print("테스트 3: 제어 방향 확인 (토크 OFF 상태에서 계산만)")
    print("=" * 60)
    print("확인 방법:")
    print("  ① 로봇을 앞으로 기울인다 → v2_cmd 부호 확인")
    print("     OK:   v2_cmd 양수(+) → 롤러가 앞으로 굴러서 로봇을 따라감 (복원)")
    print("     BAD:  v2_cmd 음수(-) → 반대로 굴러서 더 쓰러짐")
    print("  ② 로봇을 오른쪽으로 기울인다 → v1_cmd 부호 확인")
    print("     OK:   v1_cmd 양수(+) → 바퀴가 오른쪽으로 굴러서 복원")
    print("     BAD:  v1_cmd 음수(-) → 부호 문제, K 게인이나 IMU 부호 수정 필요")
    print("\n※ 모터에 실제 명령을 보내지 않습니다 (계산만)\n")
    print("Ctrl+C 를 누르면 종료\n")

    fmt = ("roll={:+6.2f}°  pitch={:+6.2f}°  │  "
           "v1_cmd(wheel)={:+7.3f}  v2_cmd(roller)={:+7.3f}  motor2={:+7.3f}")
    try:
        while True:
            euler, gyro = imu.get()
            _, vel_w, _, vel_r = robot.readback()

            roll     = ROLL_EU_SIGN  * euler[ROLL_EU_IDX]
            pitch    = PITCH_EU_SIGN * euler[PITCH_EU_IDX]
            rolldot  = ROLLDOT_SIGN  * gyro[ROLLDOT_IDX]
            pitchdot = PITCHDOT_SIGN * gyro[PITCHDOT_IDX]

            phi1dot = vel_w
            phi2dot = N_ROLLER * (vel_w + vel_r)

            # 속도 기준 0 (정지), PI 보정 없음 — 순수 상태피드백만 확인
            theta1_ref = math.radians(roll)
            theta2_ref = math.radians(pitch)

            v1_cmd = K11 * theta1_ref + K12 * rolldot + K13 * phi1dot
            v2_cmd = (K21 * theta2_ref + K22 * pitchdot + K23 * phi2dot) * N_ROLLER

            print("\r" + fmt.format(roll, pitch, v1_cmd, v2_cmd, v2_cmd - v1_cmd),
                  end="", flush=True)
            time.sleep(1 / SAMPLE_HZ)
    except KeyboardInterrupt:
        print("\n")


# ── 진입점 ───────────────────────────────────────────────────────────────────
def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"

    imu   = start_imu() if mode in ("1", "3", "all") else None
    robot = Omburo()    if mode in ("2", "3", "all") else None

    try:
        if mode in ("1", "all"):
            test_imu(imu)
            input("Enter 를 눌러 테스트 2 시작...\n")

        if mode in ("2", "all"):
            test_encoders(robot)
            input("Enter 를 눌러 테스트 3 시작...\n")

        if mode in ("3", "all"):
            test_control(imu, robot)

    finally:
        if robot:
            robot.close()


if __name__ == "__main__":
    main()
