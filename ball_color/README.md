# ball_color — 경량 CV 서브시스템 모음 (모델 불필요)

공 색 판정 + 레인 중앙 주행 + 진영 판정 + 도착 판정.
전부 순수 numpy, 프레임당 ~1ms대, Pi 5 CPU 오차 범위 수준.

## 파일

| 파일 | 역할 | 토픽 |
|---|---|---|
| `field_spec.py` | 필드 상수 단일 소스 (출처·TODO-FIELD 태그, 카메라 기하 파생 함수) | — |
| `classify.py` | 공 색 분류 코어 + 디바운서 | — |
| `config.json` | ROI·임계값·카메라 설정 전부 | — |
| `ball_color_node.py` | Camera 3 → 공 색 (Pi/테스트 겸용) | `/ball_color`, `/ball_color/state` |
| `capture_reference.py` | 장착 후 5분 캘리브레이션 | — |
| `lane_tracker.py` | 두 레인 중점 추적 코어 | — |
| `lane_node.py` | IMX500 프리뷰 → 조향 오프셋 | `/line_offset`, `/lanes_visible` |
| `team_side.py` | 시작 시 빨강/파랑 진영 1회 판정 (latched) | `/team_side` |
| `arrival_node.py` | 골 사이 도착 판정: (a) bbox 크기 + (b) 레인 소실 | `/arrived` |

## 카메라 기하 (현재 코드 기본값)

IMX500: **아래로 20° 틸트**, 4:3 모드(VFOV 52.3°), 높이 30cm.
`field_spec.py`의 복구된 설정이며, 실물 마운트에 맞춰 재검증해야 합니다.
- 하단 45% (`lane_node.BAND_FRAC`)를 레인 추적에 사용합니다.
- 지면 거리와 레인 간격은 `field_spec.py`의 기하 함수에서 계산합니다.
- 이전 15° / 25cm 값은 복구 과정에서 대체되었습니다.

[실행 안내](../docs/getting-started.md) · [복구 기록](../docs/pi_session_recovery.md)

## 토픽

- `/ball_color` (std_msgs/String) — **이벤트**: 디바운스 통과 시 공 하나당 한 번 `"orange"`/`"silver"`. 포켓이 비면 재무장.
- `/ball_color/state` (std_msgs/String) — 프레임별 원시 라벨 (디버그용).

## 설계 근거 (실측 기반)

- 판별력의 거의 전부는 **채도(S)**: 실제 경기장 프레임의 오렌지 공 S ≥ 0.47, 은색 ≈ 무채색.
- Hue 창은 일부러 넓게 (0–60°): 경기장 라임 반사가 오렌지 공 hue를
  **+20° (→31–40°)** 밀어놓은 것을 실측. auto-WB면 더 흔들린다.
- 따라서 카메라는 **AWB off·노출 고정·수동 포커스 고정** (config의 capture 블록).
- 회색·흰색 표면은 전부 "silver"로 읽힌다 → 빈-포켓 기준 프레임으로
  변화 픽셀만 투표 (presence gating). 포켓 내부는 **무광 검정** 권장:
  검정은 V가 낮아 silver 조건도 못 넘으므로 기준 프레임이 없어도 안전.

## 배선 순서 (로봇에서)

```bash
python3 capture_reference.py preview   # ROI 확인 → config.json 수정
python3 capture_reference.py empty     # 빈 포켓 기준
python3 capture_reference.py orange    # 공 넣고 몇 장
python3 capture_reference.py silver
python3 ball_color_node.py --images 'shots/*.jpg'   # 전 샷 판정 확인
python3 ball_color_node.py             # 운용
```

## 하드웨어 권장 (신뢰도 최대 레버)

1. 포켓 안에 **소형 확산 백색 LED** 상시 점등 — 외부 조명 완전 무관화.
   이거 하나로 WB 표류 문제가 사라진다.
2. 포켓 내부 무광 검정 도색.
3. Camera Module 3 초점은 포켓 거리에서 수동 고정 (config `lens_position`,
   1/거리[m]; 예: 10cm → 10.0). 흐려도 색 투표에는 무해 — AF 헌팅이 해롭다.
