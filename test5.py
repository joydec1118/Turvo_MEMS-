"""
Serial Assistant (PyQt6) — HEX I/O + quick buttons + DEC→HEX builder + CRC16(Modbus-like) header/packet

- RX/TX 固定 HEX 格式
- 快速按鈕：
  * 採樣 → 40 01 02 00 B1 0F F7 06 E8 03
  * 溫度平均 → 40 0B 00 00 FF FF AA AA
- DEC → HEX Builder：輸入 ADC1、溫度1、ADC2、溫度2（十進位，16-bit little‑endian），生成 8 bytes HEX 到輸入框
- 溫度校正指令：對「輸入框」的 8 bytes（若空會嘗試先從 DEC→HEX 生成）計算
  packet_crc = crc16_mb(payload8)，形成 header = 40 0C 08 00 + packet_crc(LE)，再計算 header_crc = crc16_mb(header)，
  最後輸出：40 0C 08 00 + packet_crc(LE) + header_crc(LE) + payload8

依賴：pip install pyqt6 pyserial
"""
from __future__ import annotations

import sys
import time
from typing import Optional

from PyQt6 import QtCore, QtGui, QtWidgets
import serial
import serial.tools.list_ports


# ---------------------- Helpers ----------------------

def list_serial_ports() -> list[str]:
    return [p.device for p in serial.tools.list_ports.comports()]


def bytes_to_hex(bs: bytes) -> str:
    return " ".join(f"{b:02X}" for b in bs)


def hex_to_bytes(s: str) -> bytes:
    cleaned = (
        s.replace(",", " ")
        .replace("\n", " ")
        .replace("\t", " ")
        .replace("-", " ")
        .replace(":", " ")
    )
    parts = cleaned.split()
    if len(parts) == 0 and s.strip():
        s2 = "".join(ch for ch in s if ch.strip())
        if len(s2) % 2 != 0:
            raise ValueError("HEX string length must be even")
        parts = [s2[i : i + 2] for i in range(0, len(s2), 2)]
    try:
        return bytes(int(p, 16) for p in parts)
    except Exception as e:
        raise ValueError(f"Invalid HEX: {e}")


def u16_to_le(v: int) -> bytes:
    return bytes([v & 0xFF, (v >> 8) & 0xFF])


class SerialReader(QtCore.QObject):
    data_received = QtCore.pyqtSignal(bytes)
    error = QtCore.pyqtSignal(str)

    def __init__(self, ser: serial.Serial, parent=None):
        super().__init__(parent)
        self._ser = ser
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(10)
        self._timer.timeout.connect(self._poll)

    def start(self):
        self._timer.start()

    def stop(self):
        self._timer.stop()

    def _poll(self):
        try:
            n = self._ser.in_waiting
            if n:
                data = self._ser.read(n)
                if data:
                    self.data_received.emit(data)
        except Exception as e:
            self.error.emit(str(e))


# ---------------------- Main UI ----------------------

class SerialAssistant(QtWidgets.QMainWindow):
    FRAME_PREFIX = bytes([0x40, 0x0B, 0x02, 0x00])

    # CRC16 MODBUS 查表（依照使用者提供的演算法）
    CRC16_MB_TABLE = [
        0x0000, 0xC1C0, 0x81C1, 0x4001, 0x01C3, 0xC003, 0x8002, 0x41C2,
        0x01C6, 0xC006, 0x8007, 0x41C7, 0x0005, 0xC1C5, 0x81C4, 0x4004,
        0x01CC, 0xC00C, 0x800D, 0x41CD, 0x000F, 0xC1CF, 0x81CE, 0x400E,
        0x000A, 0xC1CA, 0x81CB, 0x400B, 0x01C9, 0xC009, 0x8008, 0x41C8,
        0x01D8, 0xC018, 0x8019, 0x41D9, 0x001B, 0xC1DB, 0x81DA, 0x401A,
        0x001E, 0xC1DE, 0x81DF, 0x401F, 0x01DD, 0xC01D, 0x801C, 0x41DC,
        0x0014, 0xC1D4, 0x81D5, 0x4015, 0x01D7, 0xC017, 0x8016, 0x41D6,
        0x01D2, 0xC012, 0x8013, 0x41D3, 0x0011, 0xC1D1, 0x81D0, 0x4010,
        0x01F0, 0xC030, 0x8031, 0x41F1, 0x0033, 0xC1F3, 0x81F2, 0x4032,
        0x0036, 0xC1F6, 0x81F7, 0x4037, 0x01F5, 0xC035, 0x8034, 0x41F4,
        0x003C, 0xC1FC, 0x81FD, 0x403D, 0x01FF, 0xC03F, 0x803E, 0x41FE,
        0x01FA, 0xC03A, 0x803B, 0x41FB, 0x0039, 0xC1F9, 0x81F8, 0x4038,
        0x0028, 0xC1E8, 0x81E9, 0x4029, 0x01EB, 0xC02B, 0x802A, 0x41EA,
        0x01EE, 0xC02E, 0x802F, 0x41EF, 0x002D, 0xC1ED, 0x81EC, 0x402C,
        0x01E4, 0xC024, 0x8025, 0x41E5, 0x0027, 0xC1E7, 0x81E6, 0x4026,
        0x0022, 0xC1E2, 0x81E3, 0x4023, 0x01E1, 0xC021, 0x8020, 0x41E0,
        0x01A0, 0xC060, 0x8061, 0x41A1, 0x0063, 0xC1A3, 0x81A2, 0x4062,
        0x0066, 0xC1A6, 0x81A7, 0x4067, 0x01A5, 0xC065, 0x8064, 0x41A4,
        0x006C, 0xC1AC, 0x81AD, 0x406D, 0x01AF, 0xC06F, 0x806E, 0x41AE,
        0x01AA, 0xC06A, 0x806B, 0x41AB, 0x0069, 0xC1A9, 0x81A8, 0x4068,
        0x0078, 0xC1B8, 0x81B9, 0x4079, 0x01BB, 0xC07B, 0x807A, 0x41BA,
        0x01BE, 0xC07E, 0x807F, 0x41BF, 0x007D, 0xC1BD, 0x81BC, 0x407C,
        0x01B4, 0xC074, 0x8075, 0x41B5, 0x0077, 0xC1B7, 0x81B6, 0x4076,
        0x0072, 0xC1B2, 0x81B3, 0x4073, 0x01B1, 0xC071, 0x8070, 0x41B0,
        0x0050, 0xC190, 0x8191, 0x4051, 0x0193, 0xC053, 0x8052, 0x4192,
        0x0196, 0xC056, 0x8057, 0x4197, 0x0055, 0xC195, 0x8194, 0x4054,
        0x019C, 0xC05C, 0x805D, 0x419D, 0x005F, 0xC19F, 0x819E, 0x405E,
        0x005A, 0xC19A, 0x819B, 0x405B, 0x0199, 0xC059, 0x8058, 0x4198,
        0x0188, 0xC048, 0x8049, 0x4189, 0x004B, 0xC18B, 0x818A, 0x404A,
        0x004E, 0xC18E, 0x818F, 0x404F, 0x018D, 0xC04D, 0x804C, 0x418C,
        0x0044, 0xC184, 0x8185, 0x4045, 0x0187, 0xC047, 0x8046, 0x4186,
        0x0182, 0xC042, 0x8043, 0x4183, 0x0041, 0xC181, 0x8180, 0x4040
    ]

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Serial Assistant (PyQt6)")
        self.resize(1020, 720)

        self.ser: Optional[serial.Serial] = None
        self.reader: Optional[SerialReader] = None

        # CRC16 MODBUS 查表（高位在前的 256 筆表）
        self.crc16_mb_table = [
            0x0000, 0xC1C0, 0x81C1, 0x4001, 0x01C3, 0xC003, 0x8002, 0x41C2,
            0x01C6, 0xC006, 0x8007, 0x41C7, 0x0005, 0xC1C5, 0x81C4, 0x4004,
            0x01CC, 0xC00C, 0x800D, 0x41CD, 0x000F, 0xC1CF, 0x81CE, 0x400E,
            0x000A, 0xC1CA, 0x81CB, 0x400B, 0x01C9, 0xC009, 0x8008, 0x41C8,
            0x01D8, 0xC018, 0x8019, 0x41D9, 0x001B, 0xC1DB, 0x81DA, 0x401A,
            0x001E, 0xC1DE, 0x81DF, 0x401F, 0x01DD, 0xC01D, 0x801C, 0x41DC,
            0x0014, 0xC1D4, 0x81D5, 0x4015, 0x01D7, 0xC017, 0x8016, 0x41D6,
            0x01D2, 0xC012, 0x8013, 0x41D3, 0x0011, 0xC1D1, 0x81D0, 0x4010,
            0x01F0, 0xC030, 0x8031, 0x41F1, 0x0033, 0xC1F3, 0x81F2, 0x4032,
            0x0036, 0xC1F6, 0x81F7, 0x4037, 0x01F5, 0xC035, 0x8034, 0x41F4,
            0x003C, 0xC1FC, 0x81FD, 0x403D, 0x01FF, 0xC03F, 0x803E, 0x41FE,
            0x01FA, 0xC03A, 0x803B, 0x41FB, 0x0039, 0xC1F9, 0x81F8, 0x4038,
            0x0028, 0xC1E8, 0x81E9, 0x4029, 0x01EB, 0xC02B, 0x802A, 0x41EA,
            0x01EE, 0xC02E, 0x802F, 0x41EF, 0x002D, 0xC1ED, 0x81EC, 0x402C,
            0x01E4, 0xC024, 0x8025, 0x41E5, 0x0027, 0xC1E7, 0x81E6, 0x4026,
            0x0022, 0xC1E2, 0x81E3, 0x4023, 0x01E1, 0xC021, 0x8020, 0x41E0,
            0x01A0, 0xC060, 0x8061, 0x41A1, 0x0063, 0xC1A3, 0x81A2, 0x4062,
            0x0066, 0xC1A6, 0x81A7, 0x4067, 0x01A5, 0xC065, 0x8064, 0x41A4,
            0x006C, 0xC1AC, 0x81AD, 0x406D, 0x01AF, 0xC06F, 0x806E, 0x41AE,
            0x01AA, 0xC06A, 0x806B, 0x41AB, 0x0069, 0xC1A9, 0x81A8, 0x4068,
            0x0078, 0xC1B8, 0x81B9, 0x4079, 0x01BB, 0xC07B, 0x807A, 0x41BA,
            0x01BE, 0xC07E, 0x807F, 0x41BF, 0x007D, 0xC1BD, 0x81BC, 0x407C,
            0x01B4, 0xC074, 0x8075, 0x41B5, 0x0077, 0xC1B7, 0x81B6, 0x4076,
            0x0072, 0xC1B2, 0x81B3, 0x4073, 0x01B1, 0xC071, 0x8070, 0x41B0,
            0x0050, 0xC190, 0x8191, 0x4051, 0x0193, 0xC053, 0x8052, 0x4192,
            0x0196, 0xC056, 0x8057, 0x4197, 0x0055, 0xC195, 0x8194, 0x4054,
            0x019C, 0xC05C, 0x805D, 0x419D, 0x005F, 0xC19F, 0x819E, 0x405E,
            0x005A, 0xC19A, 0x819B, 0x405B, 0x0199, 0xC059, 0x8058, 0x4198,
            0x0188, 0xC048, 0x8049, 0x4189, 0x004B, 0xC18B, 0x818A, 0x404A,
            0x004E, 0xC18E, 0x818F, 0x404F, 0x018D, 0xC04D, 0x804C, 0x418C,
            0x0044, 0xC184, 0x8185, 0x4045, 0x0187, 0xC047, 0x8046, 0x4186,
            0x0182, 0xC042, 0x8043, 0x4183, 0x0041, 0xC181, 0x8180, 0x4040
        ]

        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8); root.setSpacing(8)

        # Left controls
        left = QtWidgets.QVBoxLayout(); root.addLayout(left, 0)
        self.port_combo = QtWidgets.QComboBox(); self.refresh_ports()
        self.refresh_btn = QtWidgets.QPushButton("Refresh"); self.refresh_btn.clicked.connect(self.refresh_ports)
        left.addLayout(self._row("Port:", self.port_combo, self.refresh_btn))

        self.baud_combo = QtWidgets.QComboBox();
        self.baud_combo.addItems(["1200","2400","4800","9600","19200","38400","57600","115200","230400","460800","921600"]) ; self.baud_combo.setCurrentText("115200")
        left.addLayout(self._row("Baud:", self.baud_combo))

        self.databits_combo = QtWidgets.QComboBox(); self.databits_combo.addItems(["5","6","7","8"]) ; self.databits_combo.setCurrentText("8")
        self.parity_combo   = QtWidgets.QComboBox(); self.parity_combo.addItems(["None","Even","Odd","Mark","Space"]) ; self.parity_combo.setCurrentText("None")
        self.stopbits_combo = QtWidgets.QComboBox(); self.stopbits_combo.addItems(["1","1.5","2"]) ; self.stopbits_combo.setCurrentText("1")
        left.addLayout(self._row("Data bits:", self.databits_combo))
        left.addLayout(self._row("Parity:", self.parity_combo))
        left.addLayout(self._row("Stop bits:", self.stopbits_combo))

        self.connect_btn = QtWidgets.QPushButton("Connect"); self.connect_btn.setCheckable(True); self.connect_btn.clicked.connect(self.toggle_connection)
        left.addWidget(self.connect_btn)

        left.addWidget(self._separator("Receive"))
        self.ts_chk = QtWidgets.QCheckBox("Add timestamp")
        self.autoscroll_chk = QtWidgets.QCheckBox("Auto scroll"); self.autoscroll_chk.setChecked(True)
        left.addWidget(self.ts_chk); left.addWidget(self.autoscroll_chk)
        self.clear_btn = QtWidgets.QPushButton("Clear"); self.clear_btn.clicked.connect(lambda: self.rx_view.clear())
        self.save_btn = QtWidgets.QPushButton("Save log…"); self.save_btn.clicked.connect(self.save_log)
        left.addWidget(self.clear_btn); left.addWidget(self.save_btn)
        left.addStretch(1)

        # Right side
        right = QtWidgets.QVBoxLayout(); root.addLayout(right, 1)
        self.rx_view = QtWidgets.QPlainTextEdit(readOnly=True)
        self.rx_view.setLineWrapMode(QtWidgets.QPlainTextEdit.LineWrapMode.NoWrap)
        mono = QtGui.QFont("Cascadia Mono, Fira Code, Consolas, Monaco"); mono.setStyleHint(QtGui.QFont.StyleHint.Monospace)
        self.rx_view.setFont(mono)
        right.addWidget(self.rx_view, 1)

        # TX row 1: quick buttons + checksum option
        tx_row1 = QtWidgets.QHBoxLayout(); right.addLayout(tx_row1)
        self.add8_combo = QtWidgets.QComboBox(); self.add8_combo.addItems(["No checksum","ADD8 (append)"])
        tx_row1.addWidget(self.add8_combo)
        self.btn_sample = QtWidgets.QPushButton("採樣")
        self.btn_tempavg = QtWidgets.QPushButton("溫度平均")
        self.btn_tempcal = QtWidgets.QPushButton("溫度校正指令")
        self.btn_sample.clicked.connect(lambda: self.send_bytes(bytes.fromhex("40 01 02 00 B1 0F F7 06 E8 03")))
        self.btn_tempavg.clicked.connect(lambda: self.send_bytes(bytes.fromhex("40 0B 00 00 FF FF AA AA")))
        self.btn_tempcal.clicked.connect(self.build_and_show_temp_cal)
        tx_row1.addWidget(self.btn_sample); tx_row1.addWidget(self.btn_tempavg); tx_row1.addWidget(self.btn_tempcal)
        #self.btn_tempcali = QtWidgets.QPushButton("溫度校正指令")
        #self.btn_tempcali.clicked.connect(self.build_temp_cal)
        #tx_row1.addWidget(self.btn_tempcali)
        tx_row1.addStretch(1)

        # TX row 2: manual HEX send
        tx_row2 = QtWidgets.QHBoxLayout(); right.addLayout(tx_row2)
        self.tx_edit = QtWidgets.QLineEdit(); self.tx_edit.setPlaceholderText("HEX: e.g. 40 0B 00 00 FF FF AA AA")
        self.send_btn = QtWidgets.QPushButton("Send"); self.send_btn.clicked.connect(self.send_data)
        tx_row2.addWidget(self.tx_edit, 1); tx_row2.addWidget(self.send_btn)

        # DEC → HEX builder
        build_box = QtWidgets.QGroupBox("DEC → HEX Builder"); right.addWidget(build_box)
        g = QtWidgets.QGridLayout(build_box)
        self.in_adc1 = QtWidgets.QLineEdit(); self.in_adc1.setPlaceholderText("ADC1 (DEC)")
        self.in_tmp1 = QtWidgets.QLineEdit(); self.in_tmp1.setPlaceholderText("溫度1 (DEC)")
        self.in_adc2 = QtWidgets.QLineEdit(); self.in_adc2.setPlaceholderText("ADC2 (DEC)")
        self.in_tmp2 = QtWidgets.QLineEdit(); self.in_tmp2.setPlaceholderText("溫度2 (DEC)")
        g.addWidget(QtWidgets.QLabel("ADC1"), 0, 0); g.addWidget(self.in_adc1, 0, 1)
        g.addWidget(QtWidgets.QLabel("溫度1"), 0, 2); g.addWidget(self.in_tmp1, 0, 3)
        g.addWidget(QtWidgets.QLabel("ADC2"), 1, 0); g.addWidget(self.in_adc2, 1, 1)
        g.addWidget(QtWidgets.QLabel("溫度2"), 1, 2); g.addWidget(self.in_tmp2, 1, 3)
        self.btn_build_hex = QtWidgets.QPushButton("轉HEX到輸入框")
        self.btn_build_hex.clicked.connect(self.build_hex_from_dec)
        g.addWidget(self.btn_build_hex, 2, 0, 1, 4)

        # Decoded display
        dec_row = QtWidgets.QHBoxLayout(); right.addLayout(dec_row)
        dec_row.addWidget(QtWidgets.QLabel("Decoded (last 2 bytes, little-endian) → DEC:"))
        self.dec_label = QtWidgets.QLabel("--"); f = self.dec_label.font(); f.setPointSize(16); f.setBold(True); self.dec_label.setFont(f)
        dec_row.addWidget(self.dec_label); dec_row.addStretch(1)

        self.status = self.statusBar(); self._update_status()

    # ----- helpers -----
    def _row(self, label: str, *widgets: QtWidgets.QWidget) -> QtWidgets.QHBoxLayout:
        h = QtWidgets.QHBoxLayout(); lab = QtWidgets.QLabel(label); lab.setMinimumWidth(90); h.addWidget(lab)
        for w in widgets: h.addWidget(w)
        return h

    def _separator(self, title: str) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox(title); QtWidgets.QVBoxLayout(box); return box

    # ----- serial mgmt -----
    def refresh_ports(self):
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = list_serial_ports()
        self.port_combo.addItems(ports or ["(no ports)"])
        idx = self.port_combo.findText(current)
        if idx >= 0:
            self.port_combo.setCurrentIndex(idx)

    def _open_serial(self):
        if self.ser and self.ser.is_open: return
        port = self.port_combo.currentText()
        if port == "(no ports)" or not port: raise RuntimeError("No serial port selected")
        baud = int(self.baud_combo.currentText())
        databits_map = {"5": serial.FIVEBITS, "6": serial.SIXBITS, "7": serial.SEVENBITS, "8": serial.EIGHTBITS}
        parity_map = {"None": serial.PARITY_NONE, "Even": serial.PARITY_EVEN, "Odd": serial.PARITY_ODD, "Mark": serial.PARITY_MARK, "Space": serial.PARITY_SPACE}
        stopbits_map = {"1": serial.STOPBITS_ONE, "1.5": serial.STOPBITS_ONE_POINT_FIVE, "2": serial.STOPBITS_TWO}
        self.ser = serial.Serial(port=port, baudrate=baud,
                                 bytesize=databits_map[self.databits_combo.currentText()],
                                 parity=parity_map[self.parity_combo.currentText()],
                                 stopbits=stopbits_map[self.stopbits_combo.currentText()],
                                 timeout=0, write_timeout=1)
        self.reader = SerialReader(self.ser)
        self.reader.data_received.connect(self.on_rx)
        self.reader.error.connect(self.on_error)
        self.reader.start()

    def _close_serial(self):
        if self.reader:
            self.reader.stop(); self.reader = None
        if self.ser:
            try: self.ser.close()
            finally: self.ser = None

    def toggle_connection(self):
        if self.connect_btn.isChecked():
            try:
                self._open_serial(); self.connect_btn.setText("Disconnect"); self._append_line("[Connected]\n")
            except Exception as e:
                self.connect_btn.setChecked(False); self.on_error(str(e))
        else:
            self._close_serial(); self.connect_btn.setText("Connect"); self._append_line("[Disconnected]\n")
        self._update_status()

    # ----- CRC -----
    def crc16_mb(self, data_bytes: bytes) -> int:
        """計算 CRC16（依使用者提供表與位元序流程）。回傳 0..65535。"""
        crc_value = 0xFFFF
        for byte in data_bytes:
            index = ((crc_value >> 8) ^ byte) & 0xFF
            crc_value = ((crc_value << 8) ^ self.CRC16_MB_TABLE[index]) & 0xFFFF
        return crc_value

    # ----- RX/TX -----
    def on_rx(self, data: bytes):
        ts = (time.strftime("%H:%M:%S.") + f"{int((time.time()%1)*1000):03d} ") if self.ts_chk.isChecked() else ""
        self._append_line(ts + bytes_to_hex(data))
        if len(data) >= 10 and data[:4] == self.FRAME_PREFIX:
            lo, hi = data[-2], data[-1]
            val = (hi << 8) | lo
            self.dec_label.setText(str(val))
            self._append_line(f"[Decode] 0x{val:04X} ({val} DEC)")

    def send_fixed(self, hex_str: str):
        # Backward compatible: parse string then send
        self.send_bytes(hex_to_bytes(hex_str))

    def send_bytes(self, payload: bytes):
        try:
            if self.add8_combo.currentIndex() == 1:
                payload = payload + bytes([sum(payload) & 0xFF])
            if not (self.ser and self.ser.is_open):
                raise RuntimeError("Serial not connected")
            self.ser.write(payload)
            self._append_line(f">> {bytes_to_hex(payload)}")
        except Exception as e:
            self.on_error(str(e))

    def send_data(self):
        s = self.tx_edit.text().strip()
        if not s: return
        self.send_fixed(s)

    # ----- DEC → HEX builder & Temp Cal header -----
    def _parse_dec16(self, widget: QtWidgets.QLineEdit, name: str) -> int:
        text = widget.text().strip()
        if not text:
            raise ValueError(f"{name} 空白")
        try:
            v = int(text, 10)
        except Exception:
            raise ValueError(f"{name} 必須是十進位整數")
        if not (0 <= v <= 0xFFFF):
            raise ValueError(f"{name} 超出 0..65535")
        return v

    def build_hex_from_dec(self):
        try:
            v_adc1 = self._parse_dec16(self.in_adc1, "ADC1")
            v_tmp1 = self._parse_dec16(self.in_tmp1, "溫度1")
            v_adc2 = self._parse_dec16(self.in_adc2, "ADC2")
            v_tmp2 = self._parse_dec16(self.in_tmp2, "溫度2")
            bs = bytes([
                v_adc1 & 0xFF, (v_adc1 >> 8) & 0xFF,
                v_tmp1 & 0xFF, (v_tmp1 >> 8) & 0xFF,
                v_adc2 & 0xFF, (v_adc2 >> 8) & 0xFF,
                v_tmp2 & 0xFF, (v_tmp2 >> 8) & 0xFF,
            ])
            hex_out = bytes_to_hex(bs)
            self.tx_edit.setText(hex_out)
            self._append_line(f"[Build] DEC→HEX: {hex_out}")
        except Exception as e:
            self.on_error(str(e))

    def build_and_show_temp_cal(self):
        """組出：40 0C 08 00 + packet_crc(LE) + header_crc(LE) + payload8
        其中 payload8 來源為輸入框（若空，嘗試先從 DEC→HEX Builder 生成）。"""
        try:
            s = self.tx_edit.text().strip()
            if not s:
                # 若輸入框為空，嘗試用目前 DEC 欄位生成
                self.build_hex_from_dec()
                s = self.tx_edit.text().strip()
                if not s:
                    raise RuntimeError("沒有可用的 8 bytes HEX 輸入")
            payload8 = hex_to_bytes(s)
            if len(payload8) != 8:
                raise ValueError("輸入框需為 8 個位元組的 HEX（由 DEC→HEX Builder 產生）")

            packet_crc = self.crc16_mb(payload8)
            header = bytes([0x40, 0x0C, 0x08, 0x00]) + u16_to_le(packet_crc)
            header_crc = self.crc16_mb(header)
            final_packet = header + u16_to_le(header_crc) + payload8
            hex_out = bytes_to_hex(final_packet)
            self._append_line(f"[TempCal] {hex_out}")
            self.tx_edit.setText(hex_out)
        except Exception as e:
            self.on_error(str(e))

    def crc16_mb(self, data_bytes: bytes) -> int:
        """計算 CRC16 MODBUS（使用查表法，與 self.crc16_mb_table 搭配）。返回 0..0xFFFF。"""
        crc_value = 0xFFFF
        for b in data_bytes:
            index = ((crc_value >> 8) ^ b) & 0xFF
            crc_value = ((crc_value << 8) ^ self.crc16_mb_table[index]) & 0xFFFF
        return crc_value

    def build_temp_cal(self):
        """從輸入框（若為空則嘗試先用 DEC→HEX Builder）取得 8 bytes payload，
        依規則輸出：40 0C 08 00 + packet_crc(LE) + header_crc(LE) + payload8，
        並顯示在 log 與輸入框，不自動發送。"""
        try:
            hex_str = self.tx_edit.text().strip()
            if not hex_str:
                # 若輸入框為空，嘗試先以 DEC→HEX Builder 生成
                self.build_hex_from_dec()
                hex_str = self.tx_edit.text().strip()
            payload8 = hex_to_bytes(hex_str)
            if len(payload8) != 8:
                raise ValueError("輸入框需為 8 bytes（十六進位）")

            # 1) packet_crc = crc16_mb(payload8)
            packet_crc = self.crc16_mb(payload8)
            packet_crc_le = bytes([packet_crc & 0xFF, (packet_crc >> 8) & 0xFF])

            # 2) header = 40 0C 08 00 + packet_crc(LE)
            header = bytes([0x40, 0x0C, 0x08, 0x00]) + packet_crc_le

            # 3) header_crc = crc16_mb(header)
            header_crc = self.crc16_mb(header)
            header_crc_le = bytes([header_crc & 0xFF, (header_crc >> 8) & 0xFF])

            # 4) Final output = header + header_crc + payload8
            final_packet = header + header_crc_le + payload8
            hex_out = bytes_to_hex(final_packet)
            self._append_line(f"[TempCal] {hex_out}")
            self.tx_edit.setText(hex_out)
        except Exception as e:
            self.on_error(str(e))

    # ----- misc -----
    def _append_line(self, s: str):
        self.rx_view.appendPlainText(s)
        if self.autoscroll_chk.isChecked():
            sb = self.rx_view.verticalScrollBar(); sb.setValue(sb.maximum())

    def save_log(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save log", "serial_log.txt", "Text Files (*.txt)")
        if not path: return
        with open(path, "w", encoding="utf-8") as f: f.write(self.rx_view.toPlainText())
        self.status.showMessage(f"Saved to {path}", 3000)

    def on_error(self, msg: str):
        QtWidgets.QMessageBox.critical(self, "Serial Error", msg)
        self.status.showMessage(msg, 4000)

    def _update_status(self):
        if self.ser and self.ser.is_open:
            self.status.showMessage(f"Connected: {self.ser.port} @ {self.ser.baudrate}")
        else:
            self.status.showMessage("Disconnected")


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = SerialAssistant(); w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
