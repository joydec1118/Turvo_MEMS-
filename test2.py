"""
Simple Serial Port UI (PyQt6)

Features
- Port & baud selection, data bits/parity/stop bits
- Connect / Disconnect
- Receive window with optional hex view and timestamps
- Send box with ASCII or HEX mode
- Optional ADD8 checksum helper (sum of bytes mod 256 appended)
- Clear RX / Save log
- Decode last 2 bytes of incoming frame as little-endian (e.g. 0x0284)

Dependencies: pip install pyqt6 pyserial
Run: python serial_assistant.py
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
    return bytes(int(p, 16) for p in parts)


def add8_checksum(payload: bytes) -> bytes:
    checksum = sum(payload) & 0xFF
    return payload + bytes([checksum])


def decode_last_two(data: bytes) -> Optional[int]:
    if len(data) < 2:
        return None
    lo, hi = data[-2], data[-1]
    return (hi << 8) | lo  # little-endian, e.g. 0x0284


# ---------------------- Worker ----------------------

class SerialReader(QtCore.QObject):
    data_received = QtCore.pyqtSignal(bytes)
    error = QtCore.pyqtSignal(str)

    def __init__(self, ser: serial.Serial, parent: Optional[QtCore.QObject] = None):
        super().__init__(parent)
        self._ser = ser
        self._timer = QtCore.QTimer()
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
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Serial Assistant (PyQt6)")
        self.resize(980, 640)

        self.ser: Optional[serial.Serial] = None
        self.reader: Optional[SerialReader] = None

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)

        left = QtWidgets.QVBoxLayout(); root.addLayout(left, 0)
        self.port_combo = QtWidgets.QComboBox(); self.refresh_ports()
        self.refresh_btn = QtWidgets.QPushButton("Refresh")
        row = QtWidgets.QHBoxLayout(); row.addWidget(QtWidgets.QLabel("Port:")); row.addWidget(self.port_combo); row.addWidget(self.refresh_btn)
        left.addLayout(row)
        self.refresh_btn.clicked.connect(self.refresh_ports)

        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems(["9600","19200","38400","57600","115200"])
        self.baud_combo.setCurrentText("115200")
        row = QtWidgets.QHBoxLayout(); row.addWidget(QtWidgets.QLabel("Baud:")); row.addWidget(self.baud_combo); left.addLayout(row)

        self.connect_btn = QtWidgets.QPushButton("Connect"); self.connect_btn.setCheckable(True)
        self.connect_btn.clicked.connect(self.toggle_connection); left.addWidget(self.connect_btn)

        left.addWidget(QtWidgets.QLabel("Receive options:"))
        self.hex_view_chk = QtWidgets.QCheckBox("Hex view"); left.addWidget(self.hex_view_chk)
        self.ts_chk = QtWidgets.QCheckBox("Add timestamp"); left.addWidget(self.ts_chk)
        self.clear_btn = QtWidgets.QPushButton("Clear"); self.clear_btn.clicked.connect(lambda: self.rx_view.clear()); left.addWidget(self.clear_btn)
        self.save_btn = QtWidgets.QPushButton("Save log"); self.save_btn.clicked.connect(self.save_log); left.addWidget(self.save_btn)
        left.addStretch(1)

        right = QtWidgets.QVBoxLayout(); root.addLayout(right, 1)
        self.rx_view = QtWidgets.QPlainTextEdit(readOnly=True); right.addWidget(self.rx_view, 1)

        tx_row = QtWidgets.QHBoxLayout()
        self.hex_send_chk = QtWidgets.QCheckBox("HEX send"); tx_row.addWidget(self.hex_send_chk)
        self.add8_combo = QtWidgets.QComboBox(); self.add8_combo.addItems(["No checksum","ADD8 (append)"]); tx_row.addWidget(self.add8_combo)
        tx_row.addStretch(1)
        right.addLayout(tx_row)

        tx_row2 = QtWidgets.QHBoxLayout()
        self.tx_edit = QtWidgets.QLineEdit(); tx_row2.addWidget(self.tx_edit, 1)
        self.send_btn = QtWidgets.QPushButton("Send"); self.send_btn.clicked.connect(self.send_data); tx_row2.addWidget(self.send_btn)
        right.addLayout(tx_row2)

        dec_row = QtWidgets.QHBoxLayout()
        dec_row.addWidget(QtWidgets.QLabel("Decoded (last 2 bytes, DEC):"))
        self.dec_label = QtWidgets.QLabel("--")
        f = self.dec_label.font(); f.setPointSize(14); f.setBold(True); self.dec_label.setFont(f)
        dec_row.addWidget(self.dec_label); dec_row.addStretch(1)
        right.addLayout(dec_row)

        self.status = self.statusBar()
        self._update_status()

    def refresh_ports(self):
        current = self.port_combo.currentText()
        self.port_combo.clear(); ports = list_serial_ports()
        self.port_combo.addItems(ports or ["(no ports)"])
        if current in ports: self.port_combo.setCurrentText(current)

    def _open_serial(self):
        port = self.port_combo.currentText()
        self.ser = serial.Serial(port, int(self.baud_combo.currentText()), timeout=0, write_timeout=1)
        self.reader = SerialReader(self.ser)
        self.reader.data_received.connect(self.on_rx)
        self.reader.error.connect(self.on_error)
        self.reader.start()

    def _close_serial(self):
        if self.reader: self.reader.stop(); self.reader=None
        if self.ser: self.ser.close(); self.ser=None

    def toggle_connection(self):
        if self.connect_btn.isChecked():
            try:
                self._open_serial(); self.connect_btn.setText("Disconnect"); self._append("[Connected]")
            except Exception as e:
                self.connect_btn.setChecked(False); self.on_error(str(e))
        else:
            self._close_serial(); self.connect_btn.setText("Connect"); self._append("[Disconnected]")
        self._update_status()

    def on_rx(self, data: bytes):
        ts = time.strftime("%H:%M:%S ") if self.ts_chk.isChecked() else ""
        text = bytes_to_hex(data) if self.hex_view_chk.isChecked() else data.decode(errors="replace")
        self._append(ts + text)
        # Decode only when prefix 40 0B 02 00 and length>=10 (example frame)
        prefix = bytes([0x40, 0x0B, 0x02, 0x00])
        if len(data) >= 10 and data[:4] == prefix:
            lo, hi = data[-2], data[-1]  # little-endian
            val = (hi << 8) | lo
            self.dec_label.setText(str(val))
            self._append(f"[Decode] 0x{val:04X} ({val} DEC)")

    def send_data(self):
        s = self.tx_edit.text().strip()
        if not s: return
        payload = hex_to_bytes(s) if self.hex_send_chk.isChecked() else s.encode()
        if self.add8_combo.currentIndex()==1: payload = add8_checksum(payload)
        if not (self.ser and self.ser.is_open): raise RuntimeError("Not connected")
        self.ser.write(payload)
        self._append(">> " + (bytes_to_hex(payload) if self.hex_view_chk.isChecked() or self.hex_send_chk.isChecked() else payload.decode(errors="replace")))

    def _append(self, s: str):
        self.rx_view.appendPlainText(s)
        self.rx_view.verticalScrollBar().setValue(self.rx_view.verticalScrollBar().maximum())

    def save_log(self):
        path,_=QtWidgets.QFileDialog.getSaveFileName(self,"Save log","serial_log.txt","Text Files (*.txt)")
        if path: open(path,"w",encoding="utf-8").write(self.rx_view.toPlainText())

    def on_error(self,msg:str):
        QtWidgets.QMessageBox.critical(self,"Serial Error",msg)
        self.status.showMessage(msg,5000)

    def _update_status(self):
        if self.ser and self.ser.is_open: self.status.showMessage(f"Connected {self.ser.port} @ {self.ser.baudrate}")
        else: self.status.showMessage("Disconnected")


def main():
    app = QtWidgets.QApplication(sys.argv)
    w=SerialAssistant(); w.show(); sys.exit(app.exec())

if __name__=="__main__":
    main()
