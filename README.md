need main.py
from app.main_window import MainWindow


<--------------------------------- xtgt_ui 폴더별 사용 --------------------------------->

1. xtgt_ui_m00
 - 납품용 M10 ui

1. xtgt_ui_p00
 - p05/L용 ui


<--------------------------------- 시간 동기화 설정 --------------------------------->

1. chronyc.conf

1-1) master pc 추가

local stratum 10

allow {robot_ip}

1-2) robot 추가
주석처리
pool ntp.ubuntu.com        iburst maxsources 4

pool 0.ubuntu.pool.ntp.org iburst maxsources 1

pool 1.ubuntu.pool.ntp.org iburst maxsources 1

pool 2.ubuntu.pool.ntp.org iburst maxsources 2

server {master_pc_ip} iburst prefer

2. 로봇에서 해야 될 작업

2-1) 스크립트 생성
sudo tee /usr/local/sbin/time-sync-now >/dev/null <<'EOF'

2-2) 아래 내용 작성
#!/usr/bin/env bash
set -euo pipefail

timeout 8 systemctl restart chrony
sleep 1
timeout 6 chronyc -a makestep

echo "OK"
EOF

2-3) 권한 부여
sudo chmod +x /usr/local/sbin/time-sync-now

2-4) sudoers 등록 (비번 없이 이 스크립트만 실행 허용)
sudo visudo -f /etc/sudoers.d/time-sync

* visudo로 만든거 아니면 'sudo chmod 440 /etc/sudoers.d/time-sync'을 실행해서 권한 주기

2-5) 내부 내용
m1 ALL=(root) NOPASSWD:/usr/local/sbin/time-sync-now
* m1은 robot sbc 이름

2-6) 실제 테스트
sudo -n /usr/local/sbin/time-sync-now
echo $?
0이면 성공
실패하면 에러메세지 뜸

* 실제 UI에서 사용 시 timeout error가 많이 뜰텐데 동기화 하는데 시간이 걸리는 거라 계속 해줘야 함


<--------------------------------- waypoint 창 안꺼지는 문제 --------------------------------->

1.  m10_waypoint_re.py코드 m10_waypoint.py에 넣어주면 됨


<--------------------------------- 하드 코딩된 IP --------------------------------->

1. master ip [self.master_ip = "{master_ip}"] 
 - 사용 하는 master pc에 맞춰 ip 수정 할 것

2. robot ip [self.robot_ip = {"xtgt1": "{robot_ip}", "xtgt2": "{robot_ip}"}]
 - 로봇 ip에 맞춰 수정할 것

3. robot user [self.robot_user = {"xtgt1": "{sbc}", "xtgt2": "{sbc}"}]
 - 실제 sbc 유저명으로 수정할 것


<--------------------------------- 하드 코딩된 경로 --------------------------------->

1. 수동 조작 키보드 함수 [def manual_key()]
 - 실제 teleop...py 코드가 있는 경로에 맞출 것

 - 경로 수정 할 부분 → script_path = Path.home() / "temp" / f"teleop_keyboard_{robot}.py"


