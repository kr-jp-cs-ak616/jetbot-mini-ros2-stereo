# Nano 드라이버 패치

JetBot Nano의 모터 드라이버 소스(`/home/ros2/jetbot_ros2`)는 git 저장소가 아닙니다. 그래서 변경 내역을 이 폴더에 패치 파일로 남깁니다.

| 파일 | 날짜 | 내용 |
|---|---|---|
| [`20261120-battery-fix.diff`](20261120-battery-fix.diff) | 2026-11-20 | 모터 보드가 256번째 읽기마다 상위 바이트를 0으로 보내는 문제 우회. 200회마다 부저 OFF를 쓰고, 무효값을 버리며, 5회 연속 저전압일 때만 정지합니다. 변경 파일: `jetbot_mini_ros2/core.py`, `jetbotctl`, `tests/test_safety.py`. 자세한 설명은 [기술 노트](../technical-notes.md#모터-보드-전압-오보-2026-11-20-해결)에 있습니다. |

## 적용과 되돌리기 (Nano에서)

패치는 수정 전 백업(`/home/ros2/jetbot_ros2.bak-20261120-battery`)을 기준으로 만들었습니다. 이 백업에 적용하면 현재 소스와 같아지는 것을 확인했습니다.

```bash
cd /home/ros2/jetbot_ros2
patch -p1 --dry-run < 20261120-battery-fix.diff   # 적용 가능한지 먼저 확인
patch -p1 < 20261120-battery-fix.diff             # 적용
patch -p1 -R < 20261120-battery-fix.diff          # 되돌리기
./jetbotctl build && ./jetbotctl recreate         # 적용·되돌리기 후 이미지 재빌드
```

노트북에서 Nano로 패치를 보낼 때는 `scp docs/patches/20261120-battery-fix.diff ros2@<Nano IP>:/home/ros2/jetbot_ros2/`를 사용합니다.
