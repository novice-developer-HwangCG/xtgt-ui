import os, glob
import re, time
import subprocess, signal
import paramiko
import pygame
import threading
from datetime import datetime
from pathlib import Path
from inputs import get_gamepad

from PyQt5 import uic
from PyQt5.QtCore import QObject, QThread, pyqtSignal, QSignalBlocker, QTimer, Qt
from PyQt5.QtWidgets import QMainWindow, QLabel, QPushButton, QProgressBar, QRadioButton, QButtonGroup, QMessageBox, QLineEdit, QPlainTextEdit
from PyQt5.QtGui import QPixmap, QIntValidator

"""
    P05 / P05L 용 ui
"""

class MainWindow(QMainWindow):
    rssi_stop_req = pyqtSignal()
    def __init__(self, ui_path: str):
        super().__init__()

        if not os.path.isfile(ui_path):
            raise FileNotFoundError(f"UI file not found : {ui_path}")
        
        uic.loadUi(ui_path, self)
        self.setFixedSize(self.size())  # 창 크기 고정

        # UI 배치 이상하다 싶으면 아래 주석 풀고 위 고정 코드 주석
        #self.adjustSize()
        #self.setFixedSize(self.size())

        self.robot_num = ["xtgt1", "xtgt2"] # 로봇 name

        self.running = {r: False for r in self.robot_num}
        # self.hit_log_buf = {r: [] for r in self.robot_num}
        
        self.master_ip = "192.168.0.20"
        self.robot_ip = {"xtgt1": "192.168.0.103", "xtgt2": "192.168.0.105"}
        self.robot_pw = {"xtgt1": "0000", "xtgt2": "0000"}
        self.robot_user = {"xtgt1": "xtgt1", "xtgt2": "xtgt"}     # 로봇의 topic name은 xtgt1, xtgt2, 실제 sbc명은 m1, m2 xtgt1 = m1, xtgt2 = m2
        self.ssh_port = 22
        # 로봇별 sync 상태: 0=idle, 1=syncing, 2=done, 3=fail
        self.sync_state = {r: 0 for r in self.robot_num}
        self.manual_term = {r: None for r in self.robot_num}
        self.manual_pidf = {r: Path.home() / "temp" / f"teleop_{r}.pid" for r in self.robot_num}

        self.time_threads = {}  # robot -> (thread, worker)

        self.bringup_pidfile = {
            r: {
                "auto":   f"/tmp/xtgt_bringup_{r}_auto.pid",
                "manual": f"/tmp/xtgt_bringup_{r}_manual.pid",
            } for r in self.robot_num
        }
        self.bringup_logfile = {
            r: {
                "auto":   f"/tmp/xtgt_bringup_{r}_auto.log",
                "manual": f"/tmp/xtgt_bringup_{r}_manual.log",
            } for r in self.robot_num
        }

        self.wps_proc = None
        self.rviz_proc = None

        self.count_dict = { "xtgt1": 0, "xtgt2": 0}     # 이동거리
        self.lidar_on_state = True  # lidar 켜기 끄기
        self.hit_history = []
        self.last_hit_line = None # 중복 방지용
        self.hit_count = 0
        self.mnq = 0    # 0 자동, 1 up, 2 down
        self.critical_hit = 0
        self.non_critical_hit = 0
        self.ch1 = 0        # 0 채널 사용, 1 무시
        self.ch2 = 0
        self.ch3 = 0

        self.buttons = {}
        self.Progressbars = {}
        self.labels = {}
        self.radio = {}
        self.radio_groups = {}
        self.common = {}

        for robot_num in self.robot_num:
            btn_names = [
                f"{robot_num}_stop_btn",
                f"{robot_num}_hit_rst_btn",
                f"{robot_num}_lidar_btn",
                f"{robot_num}_wheel_arm_btn",
                f"{robot_num}_time_set_btn",
                f"{robot_num}_auto_run_btn",
                f"{robot_num}_auto_exit_btn",
                f"{robot_num}_manual_run_btn",
                f"{robot_num}_manual_exit_btn",
                f"{robot_num}_key_btn",
                f"{robot_num}_mnq_ctrl_btn",
                f"{robot_num}_mnq_up_btn",
                f"{robot_num}_mnq_down_btn",
                f"{robot_num}_ch1_ignore_btn",
                f"{robot_num}_ch2_ignore_btn",
                f"{robot_num}_ch3_ignore_btn",
                f"{robot_num}_stick_btn",
                f"{robot_num}_stick_off_btn",
                f"{robot_num}_reboot_btn",
                f"{robot_num}_shutdown_btn"
            ]

            bar_names = [f"{robot_num}_bat_bar"]

            label_names = [
                f"{robot_num}_cur_spd_val",
                f"{robot_num}_cur_dir_val",
                f"{robot_num}_dist_val",
                f"{robot_num}_rssi_val",
                f"{robot_num}_rtk_val",
                f"{robot_num}_tcp_val",
                f"{robot_num}_water_val",
                f"{robot_num}_mnq_img_lbl",
                f"{robot_num}_hit_time_lbl",
                f"{robot_num}_hit_num_lbl",
                f"{robot_num}_hit_area_lbl"
            ]

            for obj_name in btn_names:
                btn = self.findChild(QPushButton, obj_name)
                if btn is None:
                    raise RuntimeError(f"Button not found : object name='{obj_name}'")
                self.buttons[obj_name] = btn

            for obj_name in bar_names:
                bar = self.findChild(QProgressBar, obj_name)
                if bar is None:
                    raise RuntimeError(f"ProgressBar not found : object name='{obj_name}'")
                self.Progressbars[obj_name] = bar

            for obj_name in label_names:
                lbl = self.findChild(QLabel, obj_name)
                if lbl is None:
                    raise RuntimeError(f"Label not found : object name='{obj_name}'")
                self.labels[obj_name] = lbl

        common_set = [
            "common_wps_btn",
            "common_rviz_btn",
            "common_hit_save_btn"
        ]

        for obj_name in common_set:
            common = self.findChild(QPushButton, obj_name)
            if common is None:
                raise RuntimeError(f"CommonButton not found : object name='{obj_name}'")
            self.common[obj_name] = common

        self.critical_edit = self.findChild(QLineEdit, "critical_val")
        if self.critical_edit is None:
            raise RuntimeError("Input not found : object name='critical_val'")

        self.non_critical_edit = self.findChild(QLineEdit, "non_critical_val")
        if self.non_critical_edit is None:
            raise RuntimeError("Input not found : object name='non_critical_val'")

        v = QIntValidator(0, 9, self)
        for w, default in [(self.critical_edit, "0"), (self.non_critical_edit, "0")]:
            w.setValidator(v)     # 0~9
            w.setMaxLength(1)     # 한 자리만
            w.setText(default)    # 초기값
            w.setAlignment(Qt.AlignCenter)

        self.logbox_text = self.findChild(QPlainTextEdit, "logbox_text")
        if self.logbox_text is None:
            raise RuntimeError("PlainTextEdit not found : object name='logbox_text'")

        self.logbox_text.setReadOnly(True)          # 사용자 입력 막기
        self.logbox_text.setMaximumBlockCount(2000)

        self.mnq_img_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "resource", "MNQ_IMG_RE")   #MNQ_IMG
        self.mnq_img_map = {
            0: "normal.jpg",
            1: "hit_bothside.jpg",
            2: "hit_center.jpg",
            3: "hit_bothside.jpg",
        }
        
        self.connect_signals_and_init()

        # ===== RSSI Worker =====
        self.rssi_thread = QThread(self)
        self.rssi_worker = RssiWorker(self.robot_num, self.ssh_exec, interval_ms=3000)
        self.rssi_worker.moveToThread(self.rssi_thread)

        self.rssi_thread.started.connect(self.rssi_worker.start)

        self.rssi_stop_req.connect(self.rssi_worker.stop, Qt.QueuedConnection)
        self.rssi_thread.finished.connect(self.rssi_worker.deleteLater)
        self.rssi_thread.finished.connect(self.rssi_thread.deleteLater)

        self.rssi_worker.updated.connect(self.rssi_updated)
        self.rssi_thread.start()

    def set_toggle_text_lidar(self, btn: QPushButton, checked: bool):
        btn.setText("On" if checked else "Off")

    def set_toggle_text_stop(self, btn: QPushButton, checked: bool):
        btn.setText("Go" if checked else "Stop")

    def set_toggle_text_wps(self, checked: bool):
        self.common["common_wps_btn"].setText("WPS Exit" if checked else "WPS Run")

    def set_toggle_text_rviz(self, checked: bool):
        self.common["common_rviz_btn"].setText("RViz Exit" if checked else "RViz Run")

    def set_toggle_text_ch_ignore(self, btn: QPushButton, checked: bool):
        btn.setText("On" if checked else "Off")

    def set_toggle_text_time(self, btn: QPushButton, state: int):
        if state == 0:
            btn.setText("Time Sync")
        elif state == 1:
            btn.setText("Syncing...")
        elif state == 2:
            btn.setText("Sync Done")
        elif state == 3:
            btn.setText("Sync Fail")

    def set_toggle_text_manual_key(self, btn: QPushButton, checked: bool):
        btn.setText("Exit" if checked else "Manual (key)")

    # def set_toggle_text_manual_stick(self, btn: QPushButton, checked: bool):
    #     btn.setText("Exit" if checked else "Manual (stick)")

    def set_toggle_text_reboot(self, btn: QPushButton, checked: bool):
        btn.setText("Rebooting" if checked else "Reboot")

    def set_toggle_text_shutdown(self, btn: QPushButton, checked: bool):
        btn.setText("Down Done" if checked else "Shut Down")

    # 버튼 복구 함수 (reboot, shutdown)
    def reset_toggle_btn(self, btn: QPushButton, set_text_fn):
        with QSignalBlocker(btn):
            btn.setChecked(False)
        set_text_fn(btn, False)

    def connect_signals_and_init(self):
        for robot in self.robot_num:
            stop_btn = self.buttons[f"{robot}_stop_btn"]
            stop_btn.setCheckable(True)
            self.set_toggle_text_stop(stop_btn, stop_btn.isChecked())
            stop_btn.toggled.connect(lambda checked, r=robot: self.stop(r, checked))

            key_btn = self.buttons[f"{robot}_key_btn"]
            key_btn.setCheckable(True)
            self.set_toggle_text_manual_key(key_btn, key_btn.isChecked())
            key_btn.toggled.connect(lambda checked, r=robot: self.manual_key(r, checked))

            reboot_btn = self.buttons[f"{robot}_reboot_btn"]
            reboot_btn.setCheckable(True)
            self.set_toggle_text_reboot(reboot_btn, reboot_btn.isChecked())
            reboot_btn.toggled.connect(lambda checked, r=robot: self.sudo_reboot(r, checked))

            shutdown_btn = self.buttons[f"{robot}_shutdown_btn"]
            shutdown_btn.setCheckable(True)
            self.set_toggle_text_shutdown(shutdown_btn, shutdown_btn.isChecked())
            shutdown_btn.toggled.connect(lambda checked, r=robot: self.sudo_shutdown(r, checked))

            lidar_btn = self.buttons[f"{robot}_lidar_btn"]
            lidar_btn.setCheckable(True)
            self.set_toggle_text_lidar(lidar_btn, lidar_btn.isChecked())
            lidar_btn.setEnabled(False)
            lidar_btn.toggled.connect(lambda checked, r=robot: self.lidar_set(r, checked))

            for ch in (1, 2, 3):
                b = self.buttons[f"{robot}_ch{ch}_ignore_btn"]
                b.setCheckable(True)
                self.set_toggle_text_ch_ignore(b, b.isChecked())
                b.toggled.connect(lambda checked, btn=b: self.set_toggle_text_ch_ignore(btn, checked))

            self.buttons[f"{robot}_auto_run_btn"].clicked.connect(lambda _=False, r=robot: self.auto_bringup_run(r))
            self.buttons[f"{robot}_auto_exit_btn"].clicked.connect(lambda _=False, r=robot: self.auto_bringup_exit(r))

            self.buttons[f"{robot}_manual_run_btn"].clicked.connect(lambda _=False, r=robot: self.manual_bringup_run(r))
            self.buttons[f"{robot}_manual_exit_btn"].clicked.connect(lambda _=False, r=robot: self.manual_bringup_exit(r))

            # self.buttons[f"{robot}_stick_btn"].clicked.connect(lambda _=False, r=robot: self.manual_stick_on(r))
            # self.buttons[f"{robot}_stick_off_btn"].clicked.connect(lambda _=False, r=robot: self.manual_stick_off(r))

            #self.buttons[f"{robot}_mnq_ctrl_btn"].clicked.connect(lambda _=False, r=robot: self.mnq_control(r))
            self.buttons[f"{robot}_hit_rst_btn"].clicked.connect(lambda _=False, r=robot: self.hit_rst(r))

            time_btn = self.buttons[f"{robot}_time_set_btn"]
            self.set_toggle_text_time(time_btn, self.sync_state[robot])
            time_btn.clicked.connect(lambda _=False, r=robot: self.time_sync(r))

            self.labels[f"{robot}_hit_time_lbl"].setText("00:00:00")
            self.labels[f"{robot}_hit_num_lbl"].setText("0")
            self.labels[f"{robot}_hit_area_lbl"].setText("-")

        self.critical_edit.editingFinished.connect(self.on_threshold_changed)
        self.non_critical_edit.editingFinished.connect(self.on_threshold_changed)
        self.on_threshold_changed()

        # 공통
        waypoint_btn = self.common["common_wps_btn"]
        rviz_btn = self.common["common_rviz_btn"]
        hit_save_btn = self.common["common_hit_save_btn"]
        #rtk_set_btn = self.common["common_rtk_base_btn"]

        waypoint_btn.setCheckable(True)
        rviz_btn.setCheckable(True)
        hit_save_btn.setCheckable(False)
        
        self.set_toggle_text_wps(waypoint_btn.isChecked())
        self.set_toggle_text_rviz(rviz_btn.isChecked())

        waypoint_btn.setEnabled(False)
        rviz_btn.setEnabled(False)

        waypoint_btn.toggled.connect(self.waypoint_run)
        rviz_btn.toggled.connect(self.rviz_run)
        hit_save_btn.clicked.connect(lambda _=False: self.save_hit_log())

    # ==================== ssh helper ====================
    def ssh_exec(self, robot: str, cmd: str, timeout: int = 10, sudo: bool = False):
        ssh = None
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(
                self.robot_ip[robot],
                username=self.robot_user[robot],
                password=self.robot_pw[robot],
                port=self.ssh_port,
                timeout=timeout,
                banner_timeout=timeout,
                auth_timeout=timeout,
                look_for_keys=False,
                allow_agent=False
            )

            if sudo:
                # sudo는 TTY/비번이 필요할 수 있으니 필요할 때만 처리
                full = f"sudo -S -p '' {cmd}"
                stdin, stdout, stderr = ssh.exec_command(full, get_pty=True)
                stdin.write(self.robot_pw[robot] + "\n")
                stdin.flush()
            else:
                stdin, stdout, stderr = ssh.exec_command(cmd)

            out = stdout.read().decode(errors="ignore")
            err = stderr.read().decode(errors="ignore")
            return out, err

        finally:
            if ssh:
                try: ssh.close()
                except Exception: pass

    # ==================== log box ====================
    def log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self.logbox_text.appendPlainText(f"[{ts}] {msg}")
        sb = self.logbox_text.verticalScrollBar()
        sb.setValue(sb.maximum())

    # ==================== bringup ====================
    def auto_bringup_run(self, robot: str):
        # # Run -> Running (토글 아님, 한번 누르면 유지)
        if self.running.get(robot, False):
            return

        launch_cmd = "ros2 launch main_xtgt main_launch.py"
        # launch_cmd = "ros2 launch main_xtgt main_launch.py rtk:=n"

        pidfile = self.bringup_pidfile[robot]["auto"]
        logfile = self.bringup_logfile[robot]["auto"]

        remote_cmd = f"""
            pidfile="{pidfile}"
            logfile="{logfile}"

            if [ -f "$pidfile" ]; then
                oldpid="$(cat "$pidfile" 2>/dev/null || true)"
                if [ -n "$oldpid" ]; then
                    kill -TERM -- -"$oldpid" 2>/dev/null || true
                    sleep 2
                    kill -KILL -- -"$oldpid" 2>/dev/null || true
                fi
                rm -f "$pidfile"
            fi

            bash -lc '
                source /opt/ros/humble/setup.bash;
                source ~/ros2_ws/install/setup.bash 2>/dev/null || true;
                export ROS_DOMAIN_ID=20;
                export ROS_DISCOVERY_SERVER={self.master_ip}:11811;
                export RMW_IMPLEMENTATION=rmw_fastrtps_cpp;
                echo "[ENV] ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER" >> "{logfile}";
                setsid {launch_cmd} > "{logfile}" 2>&1 < /dev/null &
                echo $! > "{pidfile}";
                disown || true
            '
            """

        try:
            out, err = self.ssh_exec(robot, remote_cmd, timeout=10)

            chk_out, chk_err = self.bringup_check(robot, "auto")
            # print(f"[{robot}] bringup_check:\n{chk_out}\n{chk_err}")
        except Exception as e:
            QMessageBox.critical(self, "Bringup Run Failed", f"{robot}: {e}")
            return

        self.running[robot] = True
        auto_run_btn = self.buttons[f"{robot}_auto_run_btn"]
        auto_run_btn.setText("Running")
        auto_run_btn.setEnabled(False)  # 다시 못누르게 막기

        manual_run_btn = self.buttons[f"{robot}_manual_run_btn"]
        manual_run_btn.setText("Auto Running")
        manual_run_btn.setEnabled(False)  # 메뉴얼 실행 막기

        manual_exit_btn = self.buttons[f"{robot}_manual_exit_btn"]
        manual_exit_btn.setText("Auto Running")
        manual_exit_btn.setEnabled(False)  # 메뉴얼 종료 막기

        # Run 이후 기능 버튼 활성화
        self.buttons[f"{robot}_lidar_btn"].setEnabled(True)
        self.common["common_wps_btn"].setEnabled(True)
        self.common["common_rviz_btn"].setEnabled(True)
        # self.buttons[f"{robot}_mapviz_btn"].setEnabled(True)

    def manual_bringup_run(self, robot: str):
        # # Run -> Running (토글 아님, 한번 누르면 유지)
        if self.running.get(robot, False):
            return

        launch_cmd = f"ros2 launch xtgt_bringup xtgt_bringup_launch.py ns:={robot}"

        pidfile = self.bringup_pidfile[robot]["manual"]
        logfile = self.bringup_logfile[robot]["manual"]

        remote_cmd = f"""
            pidfile="{pidfile}"
            logfile="{logfile}"

            if [ -f "$pidfile" ]; then
                oldpid="$(cat "$pidfile" 2>/dev/null || true)"
                if [ -n "$oldpid" ]; then
                    kill -TERM -- -"$oldpid" 2>/dev/null || true
                    sleep 2
                    kill -KILL -- -"$oldpid" 2>/dev/null || true
                fi
                rm -f "$pidfile"
            fi

            bash -lc '
                source /opt/ros/humble/setup.bash;
                source ~/ros2_ws/install/setup.bash 2>/dev/null || true;
                export ROS_DOMAIN_ID=20;
                export ROS_DISCOVERY_SERVER={self.master_ip}:11811;
                export RMW_IMPLEMENTATION=rmw_fastrtps_cpp;
                echo "[ENV] ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER" >> "{logfile}";
                setsid {launch_cmd} > "{logfile}" 2>&1 < /dev/null &
                echo $! > "{pidfile}";
                disown || true
            '
            """

        try:
            out, err = self.ssh_exec(robot, remote_cmd, timeout=10)

            chk_out, chk_err = self.bringup_check(robot, "manual")
            # print(f"[{robot}] bringup_check:\n{chk_out}\n{chk_err}")
        except Exception as e:
            QMessageBox.critical(self, "Bringup Run Failed", f"{robot}: {e}")
            return

        self.running[robot] = True
        manual_run_btn = self.buttons[f"{robot}_manual_run_btn"]
        manual_run_btn.setText("Running")
        manual_run_btn.setEnabled(False)  # 다시 못누르게 막기

        auto_run_btn = self.buttons[f"{robot}_auto_run_btn"]
        auto_run_btn.setText("Manual Running")
        auto_run_btn.setEnabled(False)  # 자동 실행 막기

        auto_exit_btn = self.buttons[f"{robot}_auto_exit_btn"]
        auto_exit_btn.setText("Manual Running")
        auto_exit_btn.setEnabled(False)  # 자동 종료 막기

        # manual은 활성화 안함
        # self.buttons[f"{robot}_lidar_btn"].setEnabled(True)
        # self.common["common_wps_btn"].setEnabled(True)
        # self.common["common_rviz_btn"].setEnabled(True)
        # self.buttons[f"{robot}_mapviz_btn"].setEnabled(True)

    def bringup_check(self, robot: str, mode: str):
        pidfile = self.bringup_pidfile[robot][mode]
        logfile = self.bringup_logfile[robot][mode]

        check_cmd = f"""
        bash -lc '
        pidfile="{pidfile}";
        logfile="{logfile}";
        if [ ! -f "$pidfile" ]; then
            echo "NO_PIDFILE";
            exit 0;
        fi

        pid="$(cat "$pidfile" 2>/dev/null || true)";
        if [ -z "$pid" ]; then
            echo "EMPTY_PID";
            exit 0;
        fi

        if kill -0 "$pid" 2>/dev/null; then
            echo "ALIVE pid=$pid";
        else
            echo "DEAD pid=$pid";
        fi

        echo "---- lastlog ----";
        tail -n 30 "$logfile" 2>/dev/null || true
        '
        """
        out, err = self.ssh_exec(robot, check_cmd, timeout=8)
        return out, err

    def auto_bringup_exit(self, robot: str):
        # exit 누르면 Running -> Run 복귀 + 초기화
        pidfile = self.bringup_pidfile[robot]["auto"]

        remote_cmd = f"""
            pidfile="{pidfile}"
            if [ -f "$pidfile" ]; then
                pid="$(cat "$pidfile" 2>/dev/null || true)"
                if [ -n "$pid" ]; then
                    kill -TERM -- -"$pid" 2>/dev/null || true
                    sleep 2
                    kill -KILL -- -"$pid" 2>/dev/null || true
                fi
            rm -f "$pidfile"
            fi
        """

        try:
            self.ssh_exec(robot, remote_cmd, timeout=10)
        except Exception as e:
            QMessageBox.warning(self, "Bringup Exit Warning", f"{robot}: {e}")

        self.running[robot] = False
        auto_run_btn = self.buttons[f"{robot}_auto_run_btn"]
        auto_run_btn.setText("Auto_Run")
        auto_run_btn.setEnabled(True)

        manual_run_btn = self.buttons[f"{robot}_manual_run_btn"]
        manual_run_btn.setText("Manual_Run")
        manual_run_btn.setEnabled(True)

        manual_exit_btn = self.buttons[f"{robot}_manual_exit_btn"]
        manual_exit_btn.setText("Manual Exit")
        manual_exit_btn.setEnabled(True)

        self.stop_common_processes()

        # 눌려있으면 OFF toggled 시그널도 같이 발생
        lidar_btn = self.buttons[f"{robot}_lidar_btn"]
        wps_btn  = self.common["common_wps_btn"]
        rviz_btn = self.common["common_rviz_btn"]

        # 공통 사용 예정 시 (모든 로봇이 종료되었을 때만 비활성화/종료)
        # if not any(self.running.values()):
        #     self.stop_common_processes()
        #     if wps_btn.isChecked():
        #         wps_btn.setChecked(False)
        #     if rviz_btn.isChecked():
        #         rviz_btn.setChecked(False)

        if lidar_btn.isChecked():
            lidar_btn.setChecked(False)
        if wps_btn.isChecked():
            wps_btn.setChecked(False)
        if rviz_btn.isChecked():
            rviz_btn.setChecked(False)

        # 비활성화
        lidar_btn.setEnabled(False)
        wps_btn.setEnabled(False)
        rviz_btn.setEnabled(False)

        # 종료 시 누적 이동 거리 저장 -> 해당 부분 ui 끄면 종료되게끔 수정 필요
        self.save_dist_vals(("xtgt1", "xtgt2"))

    def manual_bringup_exit(self, robot: str):
        # exit 누르면 Running -> Run 복귀 + 초기화
        pidfile = self.bringup_pidfile[robot]["manual"]

        remote_cmd = f"""
            pidfile="{pidfile}"
            if [ -f "$pidfile" ]; then
                pid="$(cat "$pidfile" 2>/dev/null || true)"
                if [ -n "$pid" ]; then
                    kill -TERM -- -"$pid" 2>/dev/null || true
                    sleep 2
                    kill -KILL -- -"$pid" 2>/dev/null || true
                fi
            rm -f "$pidfile"
            fi
        """

        try:
            self.ssh_exec(robot, remote_cmd, timeout=10)
        except Exception as e:
            QMessageBox.warning(self, "Bringup Exit Warning", f"{robot}: {e}")

        self.running[robot] = False
        manual_run_btn = self.buttons[f"{robot}_manual_run_btn"]
        manual_run_btn.setText("Manual_Run")
        manual_run_btn.setEnabled(True)

        auto_run_btn = self.buttons[f"{robot}_auto_run_btn"]
        auto_run_btn.setText("Auto_Run")
        auto_run_btn.setEnabled(True)

        auto_exit_btn = self.buttons[f"{robot}_auto_exit_btn"]
        auto_exit_btn.setText("Auto Exit")
        auto_exit_btn.setEnabled(True)

        # 종료 시 누적 이동 거리 저장 -> 해당 부분 ui 끄면 종료되게끔 수정 필요
        self.save_dist_vals(("xtgt1", "xtgt2"))

    def stop_common_processes(self):
        if self.wps_proc is not None and self.wps_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.wps_proc.pid), signal.SIGTERM)
            except Exception:
                pass
            self.wps_proc = None

        if self.rviz_proc is not None and self.rviz_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.rviz_proc.pid), signal.SIGTERM)
            except Exception:
                pass
            self.rviz_proc = None

    # ==================== lidar ====================
    def lidar_set(self, robot: str, checked: bool):
        self.set_toggle_text_lidar(self.buttons[f"{robot}_lidar_btn"], checked)
        # todo

    # 라이다 on,off 시 메모장 기록함수(블랙박스)
    def save_lidar_mode(self):
        pass

    # ==================== RSSI ====================
    def rssi_updated(self, robot: str, value: str):
        text = self.rssi_to_quality(value)
        self.labels[f"{robot}_rssi_val"].setText(text)

    def rssi_to_quality(self, value: str) -> str:
        if value is None or value == "-" or str(value).strip() == "":
            return "통신 품질"

        try:
            rssi = int(str(value).strip())
        except ValueError:
            return "통신 품질"

        if rssi >= -50:
            return "매우 양호"
        elif rssi >= -60:
            return "양호"
        elif rssi >= -70:
            return "보통"
        elif rssi >= -80:
            return "나쁨"
        else:
            return "매우 나쁨"

    # ==================== manual ====================
    def manual_key(self, robot: str, checked: bool):
        key_btn = self.buttons[f"{robot}_key_btn"]
        self.set_toggle_text_manual_key(key_btn, checked)

        script_path = Path.home() / "temp" / f"teleop_keyboard_{robot}.py"  #  / "te1.py"
        pidf = self.manual_pidf[robot]

        if checked:
            if not script_path.exists():
                print(f"[ERR] not found: {script_path}")
                with QSignalBlocker(key_btn):
                    key_btn.setChecked(False)
                self.set_toggle_text_manual_key(key_btn, False)
                return

            # 중복 실행 방지
            old = self.manual_term.get(robot)
            if old and old.poll() is None:
                return

            # pid 파일 초기화
            try:
                pidf.unlink(missing_ok=True)
            except Exception:
                pass

            inner = f"echo $$ > '{pidf}'; exec python3 '{script_path}'"

            cmd = [
                "gnome-terminal",
                "--wait",
                "--title", f"{robot} manual",
                "--", "bash", "-lc", inner
            ]

            proc = subprocess.Popen(cmd, start_new_session=True)
            self.manual_term[robot] = proc

        else:
            # 먼저 python에 Ctrl+C(SIGINT) 보내서 정상 종료 유도
            if pidf.exists():
                try:
                    pid = int(pidf.read_text().strip())
                    os.kill(pid, signal.SIGINT)  # Ctrl+C
                except Exception as e:
                    print(f"[WARN] stop failed: {e}")
                try:
                    pidf.unlink(missing_ok=True)
                except Exception:
                    pass

            # 터미널 프로세스도 남아있으면 정리
            proc = self.manual_term.get(robot)
            if proc and proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
            self.manual_term[robot] = None

    # def manual_stick_on(self, robot: str):
        pass

    # def manual_stick_off(self, robot: str):
        pass

    # ==================== time sync ====================
    # 버튼 눌려서 동기화가 되었다면 버튼 비활성화 (동기화 완료되면 추가적으로 눌림 방지), 동기화가 되지 않았다면 다시 버튼 활성화 시켜서 동기화 요청 다시 할 수 있게 하기
    def time_sync(self, robot: str):
        ip = self.robot_ip.get(robot)
        pw = self.robot_pw.get(robot)
        user = self.robot_user.get(robot)

        if not ip or not user:
            QMessageBox.warning(self, "Time Sync", f"Unknown robot: {robot}")
            return

        # 이미 동기화 진행 중이면 중복 실행 막기
        if robot in self.time_threads:
            return

        time_btn = self.buttons[f"{robot}_time_set_btn"]

        # UI: Syncing 표시 + 잠깐 비활성화(진행중)
        self.sync_state[robot] = 1
        self.set_toggle_text_time(time_btn, 1)
        time_btn.setEnabled(False)

        # Worker thread 시작
        thread = QThread(self)
        worker = TimeSyncWorker(robot, ip, user, pw, self.ssh_port, self.master_ip)
        worker.moveToThread(thread)

        thread.started.connect(worker.run)
        worker.finished.connect(self.time_sync_finished)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)

        self.time_threads[robot] = (thread, worker)
        thread.start()

    def time_sync_finished(self, robot: str, ok: bool, out: str, err: str):
        # thread 정리
        if robot in self.time_threads:
            self.time_threads.pop(robot, None)

        time_btn = self.buttons[f"{robot}_time_set_btn"]

        if ok:
            self.sync_state[robot] = 2
            self.set_toggle_text_time(time_btn, 2)
            time_btn.setEnabled(False)  # 성공이면 잠금(추가 클릭 방지)

            # print(out)

        else:
            self.sync_state[robot] = 3
            self.set_toggle_text_time(time_btn, 3)
            time_btn.setEnabled(True)   # 실패면 다시 클릭 

            QMessageBox.critical(self, "Time Sync Failed", f"{robot} ({self.robot_ip[robot]})\n{err or 'Unknown error'}")

    # ==================== stop ====================
    def stop(self, robot: str, checked: bool):
        self.set_toggle_text_stop(self.buttons[f"{robot}_stop_btn"], checked)

    # ==================== waypoint ====================
    def waypoint_run(self, checked: bool):
        wps_btn = self.common["common_wps_btn"]
        self.set_toggle_text_wps(checked)

        if checked:
            if self.wps_proc is None or self.wps_proc.poll() is not None:
                try:
                    cmd = """
                    source /opt/ros/humble/setup.bash
                    source ~/ros2_ws/install/setup.bash
                    ros2 run m10 m10_waypoint
                    """
                    self.wps_proc = subprocess.Popen(
                        ["bash", "-lc", cmd],
                        preexec_fn=os.setsid
                    )
                except Exception as e:
                    QMessageBox.warning(self, "WPS Run Error", str(e))
                    wps_btn.blockSignals(True)
                    wps_btn.setChecked(False)
                    wps_btn.blockSignals(False)
                    self.set_toggle_text_wps(False)
        else:
            if self.wps_proc is not None and self.wps_proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self.wps_proc.pid), signal.SIGTERM)
                except Exception as e:
                    QMessageBox.warning(self, "WPS Exit Error", str(e))
                self.wps_proc = None

    # ==================== Rviz ====================
    def rviz_run(self, checked: bool):
        rviz_btn = self.common["common_rviz_btn"]
        self.set_toggle_text_rviz(checked)

        if checked:
            if self.rviz_proc is None or self.rviz_proc.poll() is not None:
                try:
                    cmd = """
                    source /opt/ros/humble/setup.bash
                    source ~/ros2_ws/install/setup.bash
                    ros2 launch m10 m10_rviz_launch.py
                    """
                    self.rviz_proc = subprocess.Popen(
                        ["bash", "-lc", cmd],
                        preexec_fn=os.setsid
                    )
                except Exception as e:
                    QMessageBox.warning(self, "RViz Run Error", str(e))
                    rviz_btn.blockSignals(True)
                    rviz_btn.setChecked(False)
                    rviz_btn.blockSignals(False)
                    self.set_toggle_text_rviz(False)
        else:
            if self.rviz_proc is not None and self.rviz_proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self.rviz_proc.pid), signal.SIGTERM)
                except Exception as e:
                    QMessageBox.warning(self, "RViz Exit Error", str(e))
                self.rviz_proc = None

    # ==================== Mapviz ====================
    # def mapviz_run(self, robot: str, checked: bool):
    #     self.set_toggle_text_mapviz(self.buttons["common_mapviz_btn"], checked)

    # ==================== MNQ ====================
    # 단순 버튼 누르면 버튼 변화 없이 self.mnq 값 0으로만
    #def mnq_control(self, robot: str):
    #    self.mnq = 0

    # def mnq_up_down(self):

    # def sensing_ignore(self):

    def on_threshold_changed(self):
        c_txt = self.critical_edit.text().strip()
        n_txt = self.non_critical_edit.text().strip()

        self.critical_hit = int(c_txt) if c_txt.isdigit() else 0
        self.non_critical_hit = int(n_txt) if n_txt.isdigit() else 0

    def change_mnq(self, robot: str, mnq: int):
        lbl = self.labels[f"{robot}_mnq_img_lbl"]

        filename = self.mnq_img_map.get(mnq, "normal.jpg")
        img_path = os.path.join(self.mnq_img_dir, filename)

        if not os.path.isfile(img_path):
            lbl.setPixmap(QPixmap())
            lbl.setText(f"Image not found: {filename}")
            return

        pixmap = QPixmap(img_path)
        if pixmap.isNull():
            lbl.setPixmap(QPixmap())
            lbl.setText(f"Failed to load: {filename}")
            return

        lbl.setAlignment(Qt.AlignCenter)
        lbl.setText("")
        scaled = pixmap.scaled(lbl.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        lbl.setPixmap(scaled)

        #hit 카운트 증가
        if mnq in (1, 2, 3):
            num_key = f"{robot}_hit_num_lbl"
            time_key = f"{robot}_hit_time_lbl"
            area_key = f"{robot}_hit_area_lbl"

            try:
                cur = int(self.labels[num_key].text().strip())
            except:
                cur = 0

            cur += 1
            self.labels[num_key].setText(str(cur))
            self.labels[time_key].setText(datetime.now().strftime("%H:%M:%S"))
            if mnq == 1 or mnq == 3:
                self.labels[area_key].setText("Non-Cri")
            elif mnq == 2:
                self.labels[area_key].setText("Critical")
            else:
                self.labels[area_key].setText("-")
            
            self.record_hit_log()

    # Text
    def hit_line(self):
        parts = []
        for r in self.robot_num:  # ["xtgt1","xtgt2"]
            t_key = f"{r}_hit_time_lbl"
            n_key = f"{r}_hit_num_lbl"
            a_key = f"{r}_hit_area_lbl"

            hit_time = self.labels[t_key].text().strip()
            hit_num  = self.labels[n_key].text().strip()
            hit_area = self.labels[a_key].text().strip()

            if (not hit_time) or hit_time == "-":
                hit_time = "00:00:00"
            if (not hit_num) or hit_num == "-":
                hit_num = "0"
            if (not hit_area) or hit_area == "-":
                hit_area = "-"

            parts.append(f"{r}_{hit_time}_{hit_area}_{hit_num}")
        return " / ".join(parts)

    # Hit log 기록
    def record_hit_log(self):
        line = self.hit_line()

        if line == self.last_hit_line:
            return
        self.last_hit_line = line

        self.hit_history.append(line)

    # 탄 감지 초기화
    def hit_rst(self, robot: str):
        self.hit_history.clear()
        self.last_hit_line = None

        for r in self.robot_num:
            self.labels[f"{r}_hit_time_lbl"].setText("00:00:00")
            self.labels[f"{r}_hit_num_lbl"].setText("0")
            self.labels[f"{r}_hit_area_lbl"].setText("-")
    
    # ==================== Save log ==================== 
    def save_hit_log(self, out_dir: str = "logs"):
        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.now().strftime("%y%m%d_%H%M%S")
        path = os.path.join(out_dir, f"Hit_Log_{ts}.txt")

        if self.hit_history:
            lines = self.hit_history
        else:
            lines = [self.hit_line()]

        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")

    def save_dist_vals(self, robots=("xtgt1", "xtgt2"), out_dir: str = "logs"):
        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.now().strftime("%y%m%d_%H%M%S")
        path = os.path.join(out_dir, f"Distance_Log_{ts}.txt")

        with open(path, "w", encoding="utf-8") as f:
            for robot in robots:
                dist_text = self.labels[f"{robot}_dist_val"].text()
                f.write(f"{robot}_travel_distance_{dist_text}\n")

    # ==================== SBC reboot / shutdown ====================
    def sudo_reboot(self, robot: str, checked: bool):
        btn = self.buttons[f"{robot}_reboot_btn"]
        self.set_toggle_text_reboot(btn, checked)

        if not checked:
            return

        btn.setEnabled(False)
        self.ssh_exec(robot, "reboot", sudo=True, timeout=3)

        # 3초 뒤 텍스트 원복
        QTimer.singleShot(3000, lambda r=robot: self.reset_toggle_btn(self.buttons[f"{r}_reboot_btn"], self.set_toggle_text_reboot))
        # 30초 뒤 버튼 재 활성화
        QTimer.singleShot(30000, lambda r=robot: self.buttons[f"{r}_reboot_btn"].setEnabled(True))

    def sudo_shutdown(self, robot: str, checked: bool):
        btn = self.buttons[f"{robot}_shutdown_btn"]
        self.set_toggle_text_shutdown(btn, checked)

        if not checked:
            return

        btn.setEnabled(False)
        self.ssh_exec(robot, "shutdown now", sudo=True, timeout=3)

        # 3초 뒤 텍스트 원복
        QTimer.singleShot(3000, lambda r=robot: self.reset_toggle_btn(self.buttons[f"{r}_shutdown_btn"], self.set_toggle_text_shutdown))
        # 30초 뒤 버튼 재 활성화
        QTimer.singleShot(30000, lambda r=robot: self.buttons[f"{r}_shutdown_btn"].setEnabled(True))

    # ==================== closeEvent ====================
    def closeEvent(self, event):
        try:
            if hasattr(self, "rssi_thread") and self.rssi_thread.isRunning():
                self.rssi_stop_req.emit()
                self.rssi_thread.quit()
                self.rssi_thread.wait(1000)
        except Exception:
            pass

        try:
            self.save_dist_vals(("xtgt1", "xtgt2"))
        except Exception as e:
            print(f"[WARN] save_dist_vals failed: {e}")
        super().closeEvent(event)

# bringup 독립 클래스(쓰레드) 예정
"""
class BringupWorker(QObject):
    def __init__(self, robot: str, ip: str, user: str, password: str, port: int, master_ip: str):

    def bringup_run_func(self, robot: str, launch_cmd: str, mode_tag: str):
        if self.running.get(robot, False):
            return

        pidfile = f"/tmp/xtgt_bringup_{robot}_{mode_tag}.pid"
        logfile = f"/tmp/xtgt_bringup_{robot}_{mode_tag}.log"

        remote_cmd = f 여기에 " 세 개 넣기
            pidfile="{pidfile}"
            logfile="{logfile}"

            if [ -f "$pidfile" ]; then
                oldpid="$(cat "$pidfile" 2>/dev/null || true)"
                if [ -n "$oldpid" ]; then
                    kill -TERM -- -"$oldpid" 2>/dev/null || true
                    sleep 2
                    kill -KILL -- -"$oldpid" 2>/dev/null || true
                fi
                rm -f "$pidfile"
            fi

            bash -lc '
                source /opt/ros/humble/setup.bash;
                source ~/ros2_ws/install/setup.bash 2>/dev/null || true;
                export ROS_DOMAIN_ID=10;
                export ROS_DISCOVERY_SERVER={self.master_ip}:11811;
                export RMW_IMPLEMENTATION=rmw_fastrtps_cpp;
                
                echo "[ENV] ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER RMW=$RMW_IMPLEMENTATION" >> "{logfile}";
                setsid {launch_cmd} > "{logfile}" 2>&1 < /dev/null &
                echo $! > "{pidfile}";
                disown || true
            '
            여기에 " 세 개 넣기
        # unset ROS_LOCALHOST_ONLY; (실행 이상하면 이 코드 빈칸에 집어넣기)
        out, err = self.ssh_exec(robot, remote_cmd, timeout=10)
        self.running[robot] = True
        return pidfile, logfile
"""
# 시간 동기화 독립 클래스
class TimeSyncWorker(QObject):
    finished = pyqtSignal(str, bool, str, str)  # robot, ok, out, err
    def __init__(self, robot: str, ip: str, user: str, password: str, port: int, master_ip: str):
        super().__init__()
        self.robot = robot
        self.ip = ip
        self.user = user
        self.password = password
        self.port = port
        self.master_ip = master_ip

    def read_sources(self, ssh) -> str:
        _, stdout, _ = ssh.exec_command("chronyc -n sources -v", timeout=6)
        return stdout.read().decode("utf-8", errors="ignore")

    def get_master_mark(self, ssh):
        text = self.read_sources(ssh)
        for line in text.splitlines():
            if self.master_ip in line:
                parts = line.strip().split()
                if parts:
                    return parts[0], line, text  # ms, master_line, full_sources
        return None, "", text

    def wait_master_selected(self, ssh, retries=20, delay_sec=0.5):
        """
        오프라인에서 늦을 수 있어서 10초 기다림
        """
        last_ms, last_line, last_full = None, "", ""
        for _ in range(retries):
            ms, line, full = self.get_master_mark(ssh)
            last_ms, last_line, last_full = ms, line, full
            if ms == "^*":
                return True, f"{ms} {line}", full
            time.sleep(delay_sec)
        return False, f"master not selected (ms={last_ms})\n{last_line}", last_full

    def run(self):
        ssh = None
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(
                self.ip,
                username=self.user,
                password=self.password,
                port=self.port,
                timeout=5,
                banner_timeout=5,
                auth_timeout=5,
                look_for_keys=False,
                allow_agent=False,
            )

            # # chrony 재시작
            # cmd_restart = "sudo -n timeout 10 systemctl restart chrony"
            # _, o1, e1 = ssh.exec_command(cmd_restart, timeout=15)
            # rc1 = o1.channel.recv_exit_status()
            # err1 = e1.read().decode("utf-8", errors="ignore").strip()
            # if rc1 != 0:
            #     self.finished.emit(self.robot, False, "", err1 or f"restart chrony failed (rc={rc1})")
            #     return

            # #  즉시 보정 시도 - 이게 있어야 ^*가 빨리 뜨는 경우가 많다고 함
            # cmd_step = "sudo -n timeout 10 chronyc -a makestep"
            # _, o2, e2 = ssh.exec_command(cmd_step, timeout=15)
            # rc2 = o2.channel.recv_exit_status()
            # out2 = o2.read().decode("utf-8", errors="ignore").strip()
            # err2 = e2.read().decode("utf-8", errors="ignore").strip()
            # if rc2 != 0:
            #     # makestep이 실패해도 sources 확인은 해볼 수 있으니, 경고로만 기록
            #     warn_step = err2 or f"makestep failed (rc={rc2})"
            # else:
            #     warn_step = ""

            cmd_sync = "sudo -n /usr/local/sbin/time-sync-now"
            _, o1, e1 = ssh.exec_command(cmd_sync, timeout=25)
            rc1 = o1.channel.recv_exit_status()

            out1 = o1.read().decode("utf-8", errors="ignore").strip()
            err1 = e1.read().decode("utf-8", errors="ignore").strip()

            if rc1 != 0:
                self.finished.emit(self.robot, False, out1, err1 or f"time-sync-now failed (rc={rc1})")
                return

            warn_step = ""
            # warn_step = out1 if out1 else ""

            # ^*인지 확인
            ok_master, master_detail, sources_full = self.wait_master_selected(ssh, retries=20, delay_sec=0.5)
            if not ok_master:
                err = f"{master_detail}\n\n[SOURCES]\n{sources_full}"
                if warn_step:
                    err = f"{warn_step}\n\n" + err
                self.finished.emit(self.robot, False, "", err)
                return

            # tracking 출력
            # _, t_out, _ = ssh.exec_command("chronyc tracking | head -n 5", timeout=6)
            _, t_out, _ = ssh.exec_command("timeout 3 chronyc tracking | head -n 5 || true", timeout=8)
            tracking = t_out.read().decode("utf-8", errors="ignore").strip()

            final_out = f"[MASTER]\n{master_detail}\n\n[TRACKING]\n{tracking}"
            if warn_step:
                final_out = f"[WARN]\n{warn_step}\n\n" + final_out

            self.finished.emit(self.robot, True, final_out, "")

        except Exception as e:
            msg = f"{type(e).__name__}: {e}"
            self.finished.emit(self.robot, False, "", msg)
        finally:
            try:
                if ssh:
                    ssh.close()
            except Exception:
                pass

# RSSI 독립 클래스
class RssiWorker(QObject):
    updated = pyqtSignal(str, str)   # robot, value
    log     = pyqtSignal(str)        # optional

    def __init__(self, robot_list, ssh_exec_fn, interval_ms=3000, parent=None):
        super().__init__(parent)
        self.robot_list = list(robot_list)
        self.ssh_exec = ssh_exec_fn
        self.interval_ms = interval_ms
        self.timer = None
        self.busy = {r: False for r in self.robot_list}
        self.alive = True

    def start(self):
        # QThread 안에서 호출되어야 함
        self.timer = QTimer(self)
        self.timer.setInterval(self.interval_ms)
        self.timer.timeout.connect(self.tick)
        self.timer.start()
        self.tick()  # 시작하자마자 1회

    def stop(self):
        self.alive = False
        if self.timer:
            self.timer.stop()

    def tick(self):
        if not self.alive:
            return

        for robot in self.robot_list:
            if self.busy.get(robot, False):
                continue
            self.busy[robot] = True
            threading.Thread(target=self.fetch_one, args=(robot,), daemon=True).start()

    def fetch_one(self, robot: str):
        try:
            out, err = self.ssh_exec(robot, "bash -lc 'rssi'", timeout=5)
            text = (out or "") + "\n" + (err or "")
            val = self.parse_rssi_value(text)

            if val is None:
                out2, err2 = self.ssh_exec(robot, "bash -lc \"iw dev wlP1p1s0 link | grep -i signal\"", timeout=5)
                # out2, err2 = self.ssh_exec(
                #     robot,
                #     "bash -lc \"IF=$(iw dev | awk '$1==\\\"Interface\\\"{print $2; exit}'); "
                #     "iw dev $IF link | grep -i signal\"",
                #     timeout=5
                # )
                text2 = (out2 or "") + "\n" + (err2 or "")
                val = self.parse_rssi_value(text2)

            if not self.alive:
                return

            self.updated.emit(robot, str(val) if val is not None else "-")
        except Exception as e:
            self.updated.emit(robot, "-")
            self.log.emit(f"[RSSI] {robot} error: {e}")
        finally:
            self.busy[robot] = False

    @staticmethod
    def parse_rssi_value(text: str):
        if not text:
            return None
        m = re.search(r"signal\s*[:=]\s*(-?\d+)\s*d?bm", text, re.IGNORECASE)
        if not m:
            m = re.search(r"signal\s+level\s*[:=]?\s*(-?\d+)\s*d?bm", text, re.IGNORECASE)
        return int(m.group(1)) if m else None