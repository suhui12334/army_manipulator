# config/models/

리포 전체에서 쓰는 학습된 모델 가중치를 한곳에 모아둔다. dolbotZ(9o9hz/dolbotZ)의
`config/models/` 컨벤션과 동일하게 맞춘 것 — army_manipulator가 나중에
dolbotZ/src/arm으로 합쳐질 예정이라, 절대경로 하드코딩으로 사용자/머신이
바뀌면서 깨졌던 dolbotZ의 과거 버그(`/home/j/dolbotZ/...` -> `/home/jecs/dolbotZ/...`,
dolbotZ 커밋 `6056cf8`)를 여기서도 처음부터 피한다.

`target_detector_node.py`가 기본으로 읽는 가중치 경로 (get_package_share_directory
로 런타임에 계산, 하드코딩 없음):
`<install-space>/share/army_manipulator_bringup/config/models/supplybest_openvino_model`

## 넣어야 할 파일

dolbotZ의 `arm_pickup_node`와 동일하게 OpenVINO IR로 변환한 모델을 기본으로 쓴다
(GPU 없는 환경에서 CPU 추론 속도용 — 이 리포/이 머신과 같은 상황):

| 파일/디렉토리 | 용도 |
|---|---|
| `supplybest_openvino_model/` (`*.bin`, `*.xml`, `metadata.yaml`) | 기본 로드 대상. CPU 추론용 OpenVINO IR 변환본 |
| `supplybest.pt` | GPU/일반 PyTorch용 원본 가중치 (선택, 재변환 필요시 보관) |

dolbotZ 쪽 `config/models/supplybest_openvino_model/`, `config/models/supplybest.pt`를
그대로 복사해 넣으면 된다 (클래스명 `supplybox` 동일).

다른 경로/파일명을 쓰려면 `model_path` 파라미터로 덮어쓸 것:

```
ros2 run army_manipulator_bringup target_detector_node.py --ros-args \
  -p model_path:=/path/to/other_model
```
