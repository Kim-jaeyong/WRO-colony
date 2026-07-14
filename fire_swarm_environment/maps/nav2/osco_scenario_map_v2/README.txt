OSCO Scenario Map V2

목표
- 실제 오스코 1층 도면의 주요 방·복도 연결 구조를 유지하면서,
  3대 군집 로봇이 의미 있게 탐색·분담·중계할 수 있는 범위를 제공한다.

포함 공간
- 운영자 사무실, 회의실, 케이터링, 미술관
- 창고 #4, 서비스실, MDF실, 창고 #3
- VIP 대기실, VIP 라운지/보안, 탕비·지원실
- 남자 화장실, 여자 화장실
- 상설전시장 #1, #2
- 장애인 화장실(남/여), 로비

파일
- osco_scenario_map_v2.pgm / .yaml : Nav2 정적 지도
- osco_scenario_map_v2_doors.yaml : 문 위치·폭·연결 공간
- osco_scenario_map_v2_geometry.json : 벽·방·기둥·동적 객체 원본 데이터
- osco_scenario_map_v2_demo.yaml : 화재·연기·생존자 데모 시나리오
- overlay.png : 원본 도면과 재구성 결과 비교

정확도
- 주요 방 경계와 복도 연결은 도면 기반으로 유지했다.
- 글자, 가구, 배관, 세부 칸막이는 제외했다.
- 문 일부는 도면 기호가 겹쳐 medium/low confidence로 표시했다.
- 최종 디지털 트윈 확정 전에는 해당 문들만 현장 사진 또는 CAD 원본으로 검증하면 된다.
