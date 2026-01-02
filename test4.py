"""
Serial Assistant (PyQt6) — fixed HEX I/O + quick-send buttons

- RX/TX 都使用 HEX 格式（UI 已固定為十六進位顯示與送出）
- 兩個快速按鈕：
  * 採樣 → 發送 40 01 02 00 B1 0F F7 06 E8 03
  * 溫度平均 → 發送 40 0B 00 00 FF FF AA AA
- 僅在封包前綴為 40 0B 02 00（且長度≥10）時解碼最後兩個位元組為小端序（little‑endian），
  例如 84 02 → 0x0284 → 644（十進位），顯示在 UI。

依賴：pip install pyqt6 pyserial
執行：python serial_assistant.py
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

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Serial Assistant (PyQt6)")
        self.resize(1000, 680)

        self.ser: Optional[serial.Serial] = None
        self.reader: Optional[SerialReader] = None

        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # Left controls
        left = QtWidgets.QVBoxLayout(); root.addLayout(left, 0)
        self.port_combo = QtWidgets.QComboBox(); self.refresh_ports()
        self.refresh_btn = QtWidgets.QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        left.addLayout(self._row("Port:", self.port_combo, self.refresh_btn))

        self.baud_combo = QtWidgets.QComboBox();
        self.baud_combo.addItems(["1200","2400","4800","9600","19200","38400","57600","115200","230400","460800","921600"])
        self.baud_combo.setCurrentText("115200")
        left.addLayout(self._row("Baud:", self.baud_combo))

        self.databits_combo = QtWidgets.QComboBox(); self.databits_combo.addItems(["5","6","7","8"]) ; self.databits_combo.setCurrentText("8")
        self.parity_combo   = QtWidgets.QComboBox(); self.parity_combo.addItems(["None","Even","Odd","Mark","Space"]) ; self.parity_combo.setCurrentText("None")
        self.stopbits_combo = QtWidgets.QComboBox(); self.stopbits_combo.addItems(["1","1.5","2"]) ; self.stopbits_combo.setCurrentText("1")
        left.addLayout(self._row("Data bits:", self.databits_combo))
        left.addLayout(self._row("Parity:", self.parity_combo))
        left.addLayout(self._row("Stop bits:", self.stopbits_combo))

        self.connect_btn = QtWidgets.QPushButton("Connect"); self.connect_btn.setCheckable(True)
        self.connect_btn.clicked.connect(self.toggle_connection)
        left.addWidget(self.connect_btn)

        left.addWidget(self._separator("Receive"))
        # 固定為 HEX 顯示
        self.ts_chk = QtWidgets.QCheckBox("Add timestamp")
        self.autoscroll_chk = QtWidgets.QCheckBox("Auto scroll"); self.autoscroll_chk.setChecked(True)
        left.addWidget(self.ts_chk)
        left.addWidget(self.autoscroll_chk)
        self.clear_btn = QtWidgets.QPushButton("Clear"); self.clear_btn.clicked.connect(lambda: self.rx_view.clear())
        self.save_btn = QtWidgets.QPushButton("Save log…"); self.save_btn.clicked.connect(self.save_log)
        left.addWidget(self.clear_btn); left.addWidget(self.save_btn)
        left.addStretch(1)

        # Right side: RX view and TX controls
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
        self.btn_sample.clicked.connect(lambda: self.send_fixed("40 01 02 00 B1 0F F7 06 E8 03"))
        self.btn_tempavg.clicked.connect(lambda: self.send_fixed("40 0B 00 00 FF FF AA AA"))
        tx_row1.addWidget(self.btn_sample)
        tx_row1.addWidget(self.btn_tempavg)
        tx_row1.addStretch(1)

        # TX row 2: manual HEX send
        tx_row2 = QtWidgets.QHBoxLayout(); right.addLayout(tx_row2)
        self.tx_edit = QtWidgets.QLineEdit(); self.tx_edit.setPlaceholderText("HEX: e.g. 40 0B 00 00 FF FF AA AA")
        self.send_btn = QtWidgets.QPushButton("Send")
        self.send_btn.clicked.connect(self.send_data)
        tx_row2.addWidget(self.tx_edit, 1); tx_row2.addWidget(self.send_btn)

        # ---- DEC inputs -> HEX builder ----
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
                self._open_serial(); self.connect_btn.setText("Disconnect"); self._append_line("[Connected]")
            except Exception as e:
                self.connect_btn.setChecked(False); self.on_error(str(e))
        else:
            self._close_serial(); self.connect_btn.setText("Connect"); self._append_line("[Disconnected]")
        self._update_status()

    # ----- RX/TX -----
    def on_rx(self, data: bytes):
        # RX 固定以 HEX 顯示
        ts = (time.strftime("%H:%M:%S.") + f"{int((time.time()%1)*1000):03d} ") if self.ts_chk.isChecked() else ""
        self._append_line(ts + bytes_to_hex(data))

        # 只在偵測到前綴 40 0B 02 00 且至少 10 bytes 時解碼最後兩個位元組（little-endian）
        if len(data) >= 10 and data[:4] == self.FRAME_PREFIX:
            lo, hi = data[-2], data[-1]
            val = (hi << 8) | lo
            self.dec_label.setText(str(val))
            self._append_line(f"[Decode] 0x{val:04X} ({val} DEC)")

    def send_fixed(self, hex_str: str):
        try:
            payload = hex_to_bytes(hex_str)
            if self.add8_combo.currentIndex() == 1:
                # 可選擇是否附加 ADD8 校驗
                checksum = sum(payload) & 0xFF
                payload = payload + bytes([checksum])
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
                v_adc1 & 0xFF, (v_adc1 >> 8) & 0xFF,  # little-endian
                v_tmp1 & 0xFF, (v_tmp1 >> 8) & 0xFF,
                v_adc2 & 0xFF, (v_adc2 >> 8) & 0xFF,
                v_tmp2 & 0xFF, (v_tmp2 >> 8) & 0xFF,
            ])
            hex_out = bytes_to_hex(bs)
            self.tx_edit.setText(hex_out)
            self._append_line(f"[Build] DEC→HEX: {hex_out}")
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
