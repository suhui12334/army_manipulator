# config/models/

[2026-08-30] `target_detector_node.py`가 삭제되고 서플라이박스 인식은
`dolbotz` 패키지의 `summer_supply`(`ArmPickupNode`,
`dolbotz/dolbotz/missions/summer_supply.py`)로 통일됐다. 그 노드가 쓰는
모델 가중치는 이 디렉토리가 아니라 `src/dolbotz/config/models/`에 있다
(`supplyboxv3.pt`/`supplybox_v2.pt` 등 — `dolbotz/config/models/README.md`
참고). 여기 army_manipulator_bringup의 `config/models/`는 더 이상 런타임에
읽히지 않으니, 이 폴더에 남아있는 `supplybest_openvino_model/`은 참고용
과거 산출물일 뿐이다.
