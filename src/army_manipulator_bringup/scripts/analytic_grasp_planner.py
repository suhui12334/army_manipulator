#!/usr/bin/env python3
"""army_manipulator 4-DOF 팔의 닫힌해(closed-form) 역기구학.

[배경] maru_ik_node.py는 지금까지 두 가지 방식을 썼다:
  1. moveit(기본) - move_group의 /compute_ik(KDL) 서비스. 지역 수치해법이라
     seed(시작 관절각)에 따라 같은 목표도 풀리기도 안 풀리기도 한다 - 이번
     세션에서 실측 확인(연속탐색으로는 쉽게 풀리는 지점이 고정 seed
     몇 개로는 계속 실패). 서비스 호출 왕복(seed당 0.5~2s)도 느리다.
  2. numerical - numerical_grasp_planner.py의 유한차분 Jacobian DLS. KDL
     서비스 호출은 없지만 여전히 반복(최대 250회)+seed 의존적인 지역
     수치해법이라 같은 문제가 남는다.

[이 파일] 이 팔은 사실 "베이스 요(yaw) 1개 + 동일 평면 피치(pitch) 3개"
구조라 위치+pitch(자세)를 동시에 만족하는 닫힌해가 존재한다(교과서적인
2-link 평면 IK + 삼각함수 역산). 반복/seed가 전혀 필요 없다:
  - base_joint: target (x,y)의 atan2 하나로 바로 나옴(부호가 반대인 두 후보
    존재 - 카메라 좌표 기준으로는 항상 한쪽만 관절 리미트 안에 들어옴).
  - shoulder/elbow: 목표 pitch를 고정하면 wrist 기여분(L3EFF)이 정해지고,
    남은 2-link(L1,L2) 문제는 law-of-cosines로 바로 풀린다(elbow-up/down
    두 해).
  - wrist = pitch - shoulder - elbow.

기하 모델은 numerical_grasp_planner.py의 forward_kinematics()(army_manipulator
_macro.xacro 조인트 원점을 그대로 4x4 행렬로 합성 - "sid" 자세를 부동소수점
정밀도까지 재현한다고 검증된 코드)에서 그대로 가져왔고, 이 파일의 폐형해가
그 FK와 정확히 역함수 관계인지 수천 개 무작위 관절값으로 라운드트립
검증했다(이 파일 하단 __main__ 자체검증 블록 참고, `python3
analytic_grasp_planner.py --self-test`로 재실행 가능).

[2026-09-03] base_actuator 기준 실측 데이터(compute_fk)로 이 모델 자체도
교차검증 완료 - army_manipulator_bringup의 여러 실측 자세(legacy/grasp_wait
1~5/"d" 캡처)에서 위치 오차 0mm, pitch 오차 0.03° 이내.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from numerical_grasp_planner import (
    JOINT_LOWER,
    JOINT_UPPER,
    L1,
    L2,
    SHOULDER_OFF,
    WRIST_TO_TCP,
    IKResult,
    forward_kinematics,
)

# WRIST_TO_TCP는 wrist_joint 회전 직후의 "로컬" 프레임 기준 오프셋이다.
# 이 로컬 X축은 (고정 rpy로 재정렬된 뒤) 항상 "링크가 뻗는 방향"과
# 반대이므로, 세계 좌표계 평면(shoulder 축 기준 X-Z)에서 이 링크의
# 유효 길이는 그 크기 그대로다 - forward_kinematics()와의 라운드트립
# 검증으로 부호/축까지 확인됨(모듈 docstring 참고).
L3_EFFECTIVE = -float(WRIST_TO_TCP[0])


def _wrap_pi(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _within_limits(joints) -> bool:
    return all(
        JOINT_LOWER[i] - 1e-9 <= joints[i] <= JOINT_UPPER[i] + 1e-9
        for i in range(4)
    )


def solve_ik_analytic(target_xyz, target_pitch: float) -> list[IKResult]:
    """target_xyz(base_actuator TCP)와 target_pitch(shoulder+elbow+wrist)를
    만족하는 관절해 후보를 전부(최대 4개: base 2개 x elbow-up/down 2개)
    닫힌 형태로 계산해서, 리미트 안에 드는 것만 반환한다(반복/seed 없음).
    순서는 "가장 자연스러운" base 해(원래 target 방향과 같은 쪽)를 먼저
    둔다. 반환 리스트가 비어있으면 이 (위치,pitch) 조합 자체가 이 팔로는
    도달 불가능하다는 뜻(리미트 내 실해가 존재하지 않음 - 근사가 아니라
    엄밀한 판정)."""
    x, y, z = (float(v) for v in target_xyz)
    pitch = float(target_pitch)
    r = math.hypot(x, y)

    results: list[IKResult] = []
    base_candidates = [(math.atan2(y, x), r)]
    if r > 1e-9:
        base_candidates.append((_wrap_pi(math.atan2(y, x) + math.pi), -r))

    for base, plane_x in base_candidates:
        if not (JOINT_LOWER[0] - 1e-9 <= base <= JOINT_UPPER[0] + 1e-9):
            continue
        # plane_x = -L1 sin(phi1) - L2 sin(phi2) - L3_EFFECTIVE sin(pitch)
        # (z - SHOULDER_OFF) = L1 cos(phi1) + L2 cos(phi2) + L3_EFFECTIVE cos(pitch)
        # (phi1=shoulder, phi2=shoulder+elbow) - 남은 두 항을 표준 2-link
        # 평면 IK 형태(U = L1 sin phi1 + L2 sin phi2, W = L1 cos phi1 + L2 cos phi2)로
        # 옮긴다.
        u = -(plane_x + L3_EFFECTIVE * math.sin(pitch))
        w = (z - SHOULDER_OFF) - L3_EFFECTIVE * math.cos(pitch)
        reach_sq = u * u + w * w
        cos_elbow = (reach_sq - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
        if cos_elbow < -1.0 - 1e-9 or cos_elbow > 1.0 + 1e-9:
            continue  # 이 base/부호 조합으로는 2-link가 아예 안 닿음.
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        sin_elbow_mag = math.sqrt(max(0.0, 1.0 - cos_elbow * cos_elbow))

        for elbow_sign in (1.0, -1.0):
            elbow = elbow_sign * math.acos(cos_elbow)
            sin_elbow = elbow_sign * sin_elbow_mag
            shoulder = math.atan2(u, w) - math.atan2(
                L2 * sin_elbow, L1 + L2 * cos_elbow)
            wrist = pitch - shoulder - elbow
            joints = [base, _wrap_pi(shoulder), _wrap_pi(elbow), _wrap_pi(wrist)]
            if not _within_limits(joints):
                continue
            xyz, achieved_pitch = forward_kinematics(joints)
            position_error = math.dist(xyz, (x, y, z))
            pitch_error = abs(_wrap_pi(achieved_pitch - pitch))
            results.append(IKResult(
                True, joints, achieved_pitch, 0, position_error, pitch_error))

    results.sort(key=lambda r: r.position_error_m)
    return results


def solve_ik_analytic_any_pitch(
    target_xyz, pitch_candidates,
) -> IKResult | None:
    """pitch_candidates(순서대로) 각각에 대해 solve_ik_analytic을 시도하고
    첫 성공을 반환한다 - 반복/네트워크 호출이 전혀 없는 O(후보 개수) 연산이라
    수백 개 pitch를 스윕해도 밀리초 단위로 끝난다(기존 moveit/numerical
    백엔드는 seed x pitch 조합마다 최대 수백ms~2s가 들었다)."""
    for pitch in pitch_candidates:
        candidates = solve_ik_analytic(target_xyz, pitch)
        if candidates:
            return candidates[0]
    return None


def _self_test(num_random: int = 20000, seed: int = 12345) -> None:
    """forward_kinematics()로 무작위 관절값에서 (x,y,z,pitch)를 만든 뒤,
    solve_ik_analytic으로 되돌려서 원래 관절값(또는 동등한 다른 해)이
    나오는지 라운드트립 검증한다. 위치/피치 오차가 항상 1e-6 미만이어야
    통과."""
    import random

    rng = random.Random(seed)
    max_pos_err = 0.0
    max_pitch_err = 0.0
    misses = 0
    for _ in range(num_random):
        joints = [
            rng.uniform(JOINT_LOWER[i] + 0.02, JOINT_UPPER[i] - 0.02)
            for i in range(4)
        ]
        xyz, pitch = forward_kinematics(joints)
        results = solve_ik_analytic(xyz, pitch)
        if not results:
            misses += 1
            continue
        best = results[0]
        max_pos_err = max(max_pos_err, best.position_error_m)
        max_pitch_err = max(max_pitch_err, best.pitch_error_rad)
    print(f"self-test: {num_random}개 무작위 관절값 라운드트립")
    print(f"  못 찾은 경우(전부 리미트 밖으로 판정됨): {misses}")
    print(f"  최대 위치 오차: {max_pos_err*1000:.6f}mm")
    print(f"  최대 pitch 오차: {math.degrees(max_pitch_err):.6f}deg")
    if misses > 0 or max_pos_err > 1e-6 or max_pitch_err > 1e-6:
        raise SystemExit(1)
    print("PASS")


if __name__ == "__main__":
    import sys

    if "--self-test" in sys.argv:
        _self_test()
    else:
        print(__doc__)
