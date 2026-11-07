# nav — 자율 로봇 주행·미션 층

`ball_color/`(인식 층)가 만든 토픽을 소비해 실제로 경기를 뛰는 층.
설계 원칙은 인식 층과 동일: **순수 로직 모듈(테스트 가능) + 얇은 ROS 래퍼.**

> 현재 체크아웃은 원본 대회용 Pi 코드의 백업이 아닌 복구·재구성 자료입니다.
> [실행 안내](../docs/getting-started.md)와 [복구 기록](../docs/pi_session_recovery.md)을 함께 확인하세요.

## 파일

| 파일 | 역할 | 토픽 |
|---|---|---|
| `steering.py` | 레인 오프셋 → (전진, 회전) 듀티. PD + 레인 상실 정책 | — |
| `goal_sides.py` | 검출 → 좌/우 H·R 판정 (dedupe·근거리 게이트·투표) | — |
| `mission_logic.py` | 경기 상태 기계 (SENSE_SIDE→DESCEND→WAIT_GOALS→SORT→DONE) | — |
| `drive_node.py` | `/cmd_vel` → Cytron MDD3A 바퀴 PWM (데드맨 워치독) | in `/cmd_vel` |
| `goal_sides_node.py` | `/dummy_detections` → 래치드 `/goal_sides` ("HR"/"RH") | in `/dummy_detections`, out `/goal_sides` |
| `mission_node.py` | 오케스트레이터 | in `/team_side` `/line_offset` `/arrived` `/goal_sides` `/ball_color`, out `/cmd_vel` `/servo/*` `/mission_state` |
| `servo_node.py` | s1 팔 / s2 피더 / s3 플랩 | in `/servo/arms` `/servo/feed` `/servo/flap` |
| `test_logic.py` | 로직 테스트 25건 — ROS·하드웨어·numpy 불필요 | — |
| `launch_all.sh` | 전체 스택 기동 | — |

## 데이터 흐름

```
IMX500 ──► lane_node ──► /line_offset ──► mission ──► /cmd_vel ──► drive
   │                     /lanes_visible ─► arrival ─► /arrived ──► mission
   └──► (검출노드 TBD) ─► /dummy_detections ─► goal_sides ─► /goal_sides ─► mission
Cam3 ──► ball_color_node ──► /ball_color ──► mission ──► /servo/flap, /servo/feed
IMX500 ──► team_side ──► /team_side ──► mission
```

## 경기 정책 (실사진 분석 근거, docs/lr_field_report.html)

- `/goal_sides`는 **양쪽 박스 모두 근거리(높이 ≥25%)이고 정확히 H+R 쌍인
  프레임만 투표** — 원거리 H→R 플립과 클래스별 NMS 이중 박스를 모두 차단.
- 분류 불능 공은 `unknown_side`로: 오배송 1개는 그 골의 **더블 배점**을
  깨므로, 덜 채우는 골 쪽으로 보낸다.
- 진영은 5분 전 발표(param `cfg_side`)가 진실이고, `/team_side` 감지는
  설정 실수 방어용. 감지가 설정과 다르면 감지를 따른다.
- 드라이브 데드맨 0.5s: 미션 노드가 죽으면 즉시 정지 (탈장 금지 [RB p8-g]).

## 테스트

```bash
python3 nav/test_logic.py     # 25 checks, 의존성 없음
```

## 실기 전 캘리브레이션 (전부 파라미터/상수로 노출됨)

1. **핀 배치**: 모터 기본값은 복구된 BCM 12/17, 13/23입니다. 좌우 채널과
   극성은 실물에서 확인해야 합니다. 서보 핀은 여전히 PLACEHOLDER이며
   기본값 12/13이 모터 핀과 겹칩니다. 실제 배선에 맞는 독립 핀으로 수정 후 실행하세요.
2. **듀티 스케일**: `steering.py` V_CRUISE/V_MIN, `drive_node` TRIM —
   직진 주행으로 캘리브레이션 (경사 5° 내리막 주의: 낮게 시작).
3. **서보 각도**: `servo_node` flap/feed/arms 각도 — 링키지 확정 후.
4. **레인 기하 검증**: 실제 마운트 1프레임으로 `field_spec.py`
   CAM_HEIGHT/CAM_TILT_DEG 확인 → PAIR_SPACING 자동 갱신.
5. **IMX500 검출 노드**(별도 작업): `/dummy_detections`를
   vision_msgs/Detection2DArray, **정규화 좌표(0~1)**, class_id
   "0"/"human"·"1"/"robot"로 발행할 것.
