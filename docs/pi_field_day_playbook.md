# [복원] 필드데이 플레이북 — Pi 세션 원본 (2026-08-20)

> 분실된 경기용 Pi의 Claude 세션이 발행한 아티팩트 "FIELD_DAY.md"에서
> 2026-08-25 복원. 원본은 Pi의 `~/idc/robocon_bot` 리포 기준으로 작성됨
> (그 리포는 소실). **하드웨어 배선·측정값을 보존한 과거 기록이며, 접속 정보는 제거했습니다.**

## 핵심 하드웨어 사실 (이 리포 코드에 반영됨)

- **MDD3A 신호 배선 = 물리핀 32(M1A) · 11(M1B) · 33(M2A) · 16(M2B) · 6(GND)**
  - BCM 번호로: M1A=GPIO12, M1B=GPIO17, M2A=GPIO13, M2B=GPIO23
  - 어느 채널이 좌/우 바퀴인지는 기록에 없음 — teleop으로 확인 후
    `MOTOR_*_INVERT`(Pi) / drive_node의 핀 파라미터 교환으로 처리
- Pi 접속 계정·주소·SSID·암호: 공개용 문서에서 제거. 현재 장비의 연결 정보는 별도로 설정.
- IMX500는 부팅의 ~40%에서 I2C -121 프로브 실패 이력 → 재부팅 1회 먼저, 그다음 리본 재장착
- 서보 플랩(s3): 좌우가 반대면 `SERVO3_LEFT_DEG ↔ SERVO3_RIGHT_DEG` 90↔180 교환
- Pi 저전압: 서보 부하가 원인 — 5V 서보 전원 분리
- AWB 워밍업 부족 → 초록 색조; `CAM_WARMUP_S` 4→6

## Pi 리포의 도구 목록 (코드는 소실, 역할 기록)

`tools/webview.py`(포트 8000 비전 뷰어, 픽셀 탭=HSV 판독, SNAP 저장),
`selftest.py`, `teleop.py`, `calib_drive.py`(슬로프 위 내리막 방향에서!),
`check_geometry.py`, `preview_vision.py goalcolor --loop`,
`test_ball_pipeline.py`, `record_run.py`(경로 기록·재생 플랜B), `figure_eval.py`

## 실측 절차에서 나온 값들

| 항목 | 값 |
|---|---|
| 카메라1 pitch 권장 | 18–20° (블라인드존 0.34–0.36 m; 크로스 카운팅 매끄러움) |
| 20° 미만이면 | 스타트 테이프가 블라인드존 → `--team` 수동 지정 필수 |
| silver HSV (공 실측) | (88,90,60)–(112,220,255); 인형 페인트는 별도 튜닝, 회색 천이 silver에 매칭되면 안 됨 |
| 크로스라인 피치 | `GRID_ROW_SPACING_M` 0.478 (우리 TILE_PITCH와 일치) |
| 분류(소트) 위치 | 골 라인에서 0.38 m 목표, `FINAL_ROW_OFFSET_M` −0.15 |
| 수집 대기 | `BALL_COLLECT_WAIT_S` = 180 − 실측 주행+분류 − 15 |
| 배치 지그 | 패드에 로봇 외곽 테이프. 전후 오차 ±15 cm까지 sim 검증 |
| 마지막 ~1.3 m | 레인이 골 라인에서 끝나 헤딩 재잠금 없이 직진 (설계 의도) |

## 매치 데이

```
python3 main.py --team red|blue     # 심판 GO에 실행 = 매치 시계 0초
```
- 시작 자세: 패드 위, 레인 쌍을 향해 조준 (초기 헤딩 자유)
- 플랜 B-1 (인형 식별 불가): `GOAL_COLOR_METHOD=forced`, 눈으로 보고
  `GOAL_COLOR_FORCED=('orange','silver')` (왼쪽 골이 사람 인형일 때)
- 플랜 B-2 (비전 전체 불가): `record_run.py` 기록·재생
