import sys
import serial
import serial.tools.list_ports
import re
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QLineEdit, QPushButton,
                             QComboBox, QTextEdit, QGroupBox, QGridLayout, QMessageBox)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
import pyqtgraph as pg


# --------------------------
# 串口接收工作线程
# --------------------------
class SerialThread(QThread):
    data_received = pyqtSignal(str)  # 信号：接收到原始字符串

    def __init__(self):
        super().__init__()
        self.ser = None
        self.is_running = False

    def open_port(self, port_name, baud_rate):
        try:
            self.ser = serial.Serial(port_name, baud_rate, timeout=1)
            self.is_running = True
            self.start()
            return True
        except Exception as e:
            return False

    def close_port(self):
        self.is_running = False
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_data(self, text):
        if self.ser and self.ser.is_open:
            try:
                # 发送数据，自动附加换行符
                self.ser.write((text + "\r\n").encode('utf-8'))
            except Exception as e:
                print(f"Send Error: {e}")

    def run(self):
        while self.is_running and self.ser and self.ser.is_open:
            try:
                # 读取一行，以 \n 为结束符
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.data_received.emit(line)
            except Exception as e:
                print(f"Read Error: {e}")
                break


# --------------------------
# 主窗口
# --------------------------
class PIDTunerWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 PID 增量式调试助手")
        self.resize(1000, 700)

        ###################################################################
        self.recv_count = 0  # 1秒内接收到多少条
        self.recv_freq = 0  # 当前频率

        # 启动定时器，每秒更新一次接收频率
        self.freq_timer = QTimer()
        self.freq_timer.timeout.connect(self.update_frequency)
        self.freq_timer.start(1000)

        ####################################################################

        # 数据存储用于绘图
        self.data_buffer_size = 500  # 显示最近500个点
        self.target_data = [0] * self.data_buffer_size
        self.actual_data = [0] * self.data_buffer_size
        self.current_target = 0.0

        # 串口线程
        self.serial_thread = SerialThread()
        self.serial_thread.data_received.connect(self.handle_serial_data)

        self.init_ui()

    ####################################################################
    def update_frequency(self):
        self.recv_freq = self.recv_count
        self.recv_count = 0  # 重新开始计数

        # 更新到UI
        self.label_freq.setText(f"接收频率: {self.recv_freq} Hz")

    ####################################################################
    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout()
        main_widget.setLayout(main_layout)

        # --- 左侧控制面板 ---
        control_panel = QVBoxLayout()
        control_panel.setSpacing(10)

        # 1. 串口设置区域
        group_serial = QGroupBox("串口设置")
        layout_serial = QGridLayout()

        self.combo_port = QComboBox()
        self.btn_refresh = QPushButton("刷新串口")
        self.btn_refresh.clicked.connect(self.refresh_ports)

        self.combo_baud = QComboBox()
        self.combo_baud.addItems(["9600", "115200", "256000", "921600"])
        self.combo_baud.setCurrentText("115200")

        self.btn_connect = QPushButton("打开串口")
        self.btn_connect.setCheckable(True)
        self.btn_connect.clicked.connect(self.toggle_connection)

        layout_serial.addWidget(QLabel("端口:"), 0, 0)
        layout_serial.addWidget(self.combo_port, 0, 1)
        layout_serial.addWidget(self.btn_refresh, 0, 2)
        layout_serial.addWidget(QLabel("波特率:"), 1, 0)
        layout_serial.addWidget(self.combo_baud, 1, 1)
        # layout_serial.addWidget(self.btn_connect, 2, 0, 1, 3)

        ##########################################################################
        self.label_freq = QLabel("接收频率: 0 Hz")
        layout_serial.addWidget(self.label_freq, 2, 0, 1, 3)
        layout_serial.addWidget(self.btn_connect, 3, 0, 1, 3)

        ########################################################################
        group_serial.setLayout(layout_serial)

        # 2. PID 参数设置区域
        group_pid = QGroupBox("PID 参数 & 目标值")
        layout_pid = QGridLayout()

        self.input_kp = QLineEdit("1.0")
        self.input_ki = QLineEdit("0.1")
        self.input_kd = QLineEdit("0.0")
        self.input_target = QLineEdit("0.0")

        self.btn_send = QPushButton("发送参数")
        self.btn_send.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; height: 40px;")
        self.btn_send.clicked.connect(self.send_parameters)

        layout_pid.addWidget(QLabel("Kp (比例):"), 0, 0)
        layout_pid.addWidget(self.input_kp, 0, 1)
        layout_pid.addWidget(QLabel("Ki (积分):"), 1, 0)
        layout_pid.addWidget(self.input_ki, 1, 1)
        layout_pid.addWidget(QLabel("Kd (微分):"), 2, 0)
        layout_pid.addWidget(self.input_kd, 2, 1)
        layout_pid.addWidget(QLabel("Target (目标):"), 3, 0)
        layout_pid.addWidget(self.input_target, 3, 1)
        layout_pid.addWidget(self.btn_send, 4, 0, 1, 2)
        group_pid.setLayout(layout_pid)

        # 3. 接收数据显示区域
        group_log = QGroupBox("接收数据日志")
        layout_log = QVBoxLayout()
        self.text_log = QTextEdit()
        self.text_log.setReadOnly(True)
        layout_log.addWidget(self.text_log)
        group_log.setLayout(layout_log)

        # 添加到左侧布局
        control_panel.addWidget(group_serial)
        control_panel.addWidget(group_pid)
        control_panel.addWidget(group_log)
        control_panel.addStretch()  # 弹簧，把控件顶上去

        # --- 右侧绘图区域 ---
        # 设置 PyQtGraph
        pg.setConfigOption('background', 'w')  # 白色背景
        pg.setConfigOption('foreground', 'k')  # 黑色坐标轴
        self.plot_widget = pg.PlotWidget(title="实时速度曲线 (m/s)")
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setLabel('left', '速度', units='m')
        self.plot_widget.setLabel('bottom', '采样点')
        self.plot_widget.addLegend()

        self.curve_target = self.plot_widget.plot(pen=pg.mkPen('r', width=2, style=2), name="目标值")  # 红色虚线
        self.curve_actual = self.plot_widget.plot(pen=pg.mkPen('b', width=2), name="实际值")  # 蓝色实线

        # 组装整体布局 (左1 : 右3 比例)
        main_layout.addLayout(control_panel, 1)
        main_layout.addWidget(self.plot_widget, 3)

        self.refresh_ports()

    def refresh_ports(self):
        self.combo_port.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_port.addItem(p.device)

    def toggle_connection(self):
        if self.btn_connect.isChecked():
            port = self.combo_port.currentText()
            baud = int(self.combo_baud.currentText())
            if not port:
                return

            if self.serial_thread.open_port(port, baud):
                self.btn_connect.setText("关闭串口")
                self.btn_connect.setStyleSheet("background-color: #f44336; color: white;")
                self.enable_inputs(False)  # 连接后禁止修改波特率
            else:
                self.btn_connect.setChecked(False)
                QMessageBox.critical(self, "错误", "无法打开串口")
        else:
            self.serial_thread.close_port()
            self.btn_connect.setText("打开串口")
            self.btn_connect.setStyleSheet("")
            self.enable_inputs(True)

    def enable_inputs(self, enable):
        self.combo_port.setEnabled(enable)
        self.combo_baud.setEnabled(enable)

    def send_parameters(self):
        if not self.serial_thread.ser or not self.serial_thread.ser.is_open:
            QMessageBox.warning(self, "警告", "请先打开串口")
            return

        try:
            kp = float(self.input_kp.text())
            ki = float(self.input_ki.text())
            kd = float(self.input_kd.text())
            target = float(self.input_target.text())

            # 更新本地目标值用于绘图
            self.current_target = target

            # 定义发送协议：例如 "P1.0I0.1D0.05T100" (具体取决于你STM32怎么解析)
            # 这里我设计一个通用的格式: P=xx,I=xx,D=xx,T=xx
            send_str = f"P={kp:.3f},I={ki:.3f},D={kd:.3f},T={target:.3f}"
            self.serial_thread.send_data(send_str)

            self.log_message(f"发送: {send_str}", color="green")

        except ValueError:
            QMessageBox.warning(self, "错误", "请输入有效的数字")

    def handle_serial_data(self, line):
        # 1. 显示原始数据
        ###################################################################################
        self.recv_count += 1

        ###################################################################################

        self.log_message(f"接收: {line}")

        # 2. 解析数据
        # 预期格式: (符号)(速度)(单位m) 例子: "+1.03" 或 "+1.03m"
        # 也可以处理纯数字
        try:
            # 移除 'm', 空格, \r, \n
            clean_str = line.replace('m', '').strip()
            # 尝试转换为浮点数
            val = float(clean_str)

            self.update_plot(val)
        except ValueError:
            # 如果解析失败（例如收到的是调试文本而不是数据），则忽略绘图
            pass

    def update_plot(self, value):
        # 更新数据列表
        self.actual_data.pop(0)
        self.actual_data.append(value)

        self.target_data.pop(0)
        self.target_data.append(self.current_target)

        # 刷新曲线
        self.curve_actual.setData(self.actual_data)
        self.curve_target.setData(self.target_data)

    def log_message(self, msg, color="black"):
        self.text_log.append(f"<span style='color:{color}'>{msg}</span>")
        # 自动滚动到底部
        cursor = self.text_log.textCursor()
        cursor.movePosition(cursor.End)
        self.text_log.setTextCursor(cursor)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = PIDTunerWindow()
    window.show()
    sys.exit(app.exec_())