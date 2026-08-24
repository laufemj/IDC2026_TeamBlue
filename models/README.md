# models — 배포 가중치

## dummy_yolo11n_v25f2.pt

더미(human/robot) 검출기 최종본. 현재까지 가장 검증된 가중치.

- 아키텍처: YOLO11n, imgsz 640, 클래스 0=human, 1=robot
- 학습: 100% 합성(Blender 렌더, dataset_v25 7.7k train), 실사진 0장
  - v25a(22ep) → v25_full(6ep) → v25f2(+8ep, plateau 자동중단) 웜스타트 체인
- 합성 val: mAP50 0.987, mAP50-95 0.768
- 실사진 검증(127장, docs/lr_field_report.html):
  - 근거리(박스 높이 ≥ 화면 24%) H/R 분류 100%
  - 은색 robot 판정은 전 거리 100%, 오류는 원거리 human→robot 단방향뿐
- 운용점 (ball_color/field_spec.py에 상수화):
  conf 0.25, agnostic NMS(클래스 교차 dedupe), 가장자리 3% 필터,
  좌/우 래칭은 박스 높이 ≥ 0.25에서만
- 사용 예:
  from ultralytics import YOLO
  YOLO("models/dummy_yolo11n_v25f2.pt").predict(img, imgsz=640,
      conf=0.25, agnostic_nms=True)
- IMX500 온센서 배포는 별도 export 필요 (진행 예정)

## dummy_yolo11n_v25f2.onnx (폴백)

IMX500 변환 실패 시 Pi CPU에서 돌릴 폴백 (opset 13, simplify, 640).
"ONNX = 항상 빠름"이 아님에 주의: 개발 데스크톱(x86) 실측은
PyTorch 14.7 ms vs ONNX 39.2 ms/frame으로 PyTorch가 우세했다.
Pi 5(ARM)에서는 별도 벤치마크 후 빠른 쪽을 쓸 것. 1차 계획은 어디까지나
IMX500 온센서 실행(Pi CPU 점유 ~0)이다.

## dummy_yolo11n_v25f2_imx_model/ (IMX500 온센서 배포용)

`packerOut.zip`이 IMX500 패키징 입력. INT8 양자화 (MCT 2.4.5, 캘리브레이션
= dataset_v25 val 5%; fraction 1.0은 30GB 램에서도 OOM).
- 칩 적합: **Fit In Chip true** (7.12/8.00 MB, 90%)
- 실사진 127장 재평가: 착석 좌/우 판정 사용가능 65/81 = 80%
  (FP32 85% 대비 소폭 저하; 근거리 판정 구간은 동일하게 동작)
- labels: 0=human_dummy, 1=robot_dummy
- Pi에서 패키징: imx500-package -i packerOut.zip -o <out> 후
  picamera2 IMX500 데모 흐름으로 로드 (검출노드 작성 시 참조)
