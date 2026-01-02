"""
Simple Serial Port UI (PyQt6)

Features
- Port & baud selection, data bits/parity/stop bits
- Connect / Disconnect
- Receive window with optional hex view and timestamps
- Send box with ASCII or HEX mode
- Optional ADD8 checksum helper (sum of bytes mod 256 appended)
- Clear RX / Save log

Dependencies: pip install pyqt6 pyserial
Run: python serial_assistant.py
"""
from __future__ import annotations

import sys
import time
from dataclasses import dataclass
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
    # Accept formats like "40 0B 00 00 FF AA" or "40,0B,00" or "400B0000FFAA"
    cleaned = (
        s.replace(",", " ")
        .replace("\n", " ")
        .replace("\t", " ")
        .replace("-", " ")
        .replace(":", " ")
    )
    parts = cleaned.split()
    if len(parts) == 0 and s.strip():
        # No spaces — try every two chars
        s2 = "".join(ch for ch in s if ch.strip())
        if len(s2) % 2 != 0:
            raise ValueError("HEX string length must be even")
        parts = [s2[i : i + 2] for i in range(0, len(s2), 2)]
    try:
        return bytes(int(p, 16) for p in parts)
    except Exception as e:
        raise ValueError(f"Invalid HEX: {e}")


def add8_checksum(payload: bytes) -> bytes:
    """Append ADD8 checksum: sum(payload) & 0xFF."""
    checksum = sum(payload) & 0xFF
    return payload + bytes([checksum])


# ---------------------- Worker ----------------------

class SerialReader(QtCore.QObject):
    data_received = QtCore.pyqtSignal(bytes)
    error = QtCore.pyqtSignal(str)

    def __init__(self, ser: serial.Serial, parent: Optional[QtCore.QObject] = None):
        super().__init__(parent)
        self._ser = ser
        self._timer = QtCore.QTimer()
        self._timer.setInterval(10)  # ms
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

        # Serial object
        self.ser: Optional[serial.Serial] = None
        self.reader: Optional[SerialReader] = None

        # Central layout
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # Left panel (controls)
        left = QtWidgets.QVBoxLayout()
        root.addLayout(left, 0)

        # Port row
        self.port_combo = QtWidgets.QComboBox()
        self.refresh_ports()
        self.refresh_btn = QtWidgets.QPushButton("Refresh")
        port_row = self._row("Port:", self.port_combo, self.refresh_btn)
        left.addLayout(port_row)
        self.refresh_btn.clicked.connect(self.refresh_ports)

        # Baud rate
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems([
            "1200","2400","4800","9600","19200","38400","57600","115200","230400","460800","921600"
        ])
        self.baud_combo.setCurrentText("115200")
        left.addLayout(self._row("Baud:", self.baud_combo))

        # Data bits / Parity / Stop bits
        self.databits_combo = QtWidgets.QComboBox(); self.databits_combo.addItems(["5","6","7","8"]); self.databits_combo.setCurrentText("8")
        self.parity_combo = QtWidgets.QComboBox(); self.parity_combo.addItems(["None","Even","Odd","Mark","Space"])
        self.stopbits_combo = QtWidgets.QComboBox(); self.stopbits_combo.addItems(["1","1.5","2"])
        left.addLayout(self._row("Data bits:", self.databits_combo))
        left.addLayout(self._row("Parity:", self.parity_combo))
        left.addLayout(self._row("Stop bits:", self.stopbits_combo))

        # Connect button
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.setCheckable(True)
        self.connect_btn.clicked.connect(self.toggle_connection)
        left.addWidget(self.connect_btn)

        # Receive options
        left.addWidget(self._separator("Receive"))
        self.hex_view_chk = QtWidgets.QCheckBox("Hex view")
        self.ts_chk = QtWidgets.QCheckBox("Add timestamp")
        self.autoscroll_chk = QtWidgets.QCheckBox("Auto scroll")
        self.autoscroll_chk.setChecked(True)
        left.addWidget(self.hex_view_chk)
        left.addWidget(self.ts_chk)
        left.addWidget(self.autoscroll_chk)

        self.clear_btn = QtWidgets.QPushButton("Clear")
        self.save_btn = QtWidgets.QPushButton("Save log…")
        left.addWidget(self.clear_btn)
        left.addWidget(self.save_btn)
        self.clear_btn.clicked.connect(lambda: self.rx_view.clear())
        self.save_btn.clicked.connect(self.save_log)
        left.addStretch(1)

        # Right panel (RX/TX)
        right = QtWidgets.QVBoxLayout()
        root.addLayout(right, 1)

        # RX view
        self.rx_view = QtWidgets.QPlainTextEdit(readOnly=True)
        self.rx_view.setLineWrapMode(QtWidgets.QPlainTextEdit.LineWrapMode.NoWrap)
        font = QtGui.QFont("Cascadia Mono, Fira Code, Consolas, Monaco")
        font.setStyleHint(QtGui.QFont.StyleHint.Monospace)
        self.rx_view.setFont(font)
        right.addWidget(self.rx_view, 1)

        # TX controls
        tx_row1 = QtWidgets.QHBoxLayout()
        self.hex_send_chk = QtWidgets.QCheckBox("HEX send")
        self.add8_combo = QtWidgets.QComboBox(); self.add8_combo.addItems(["No checksum","ADD8 (append)"])
        tx_row1.addWidget(self.hex_send_chk)
        tx_row1.addWidget(self.add8_combo)
        tx_row1.addStretch(1)
        right.addLayout(tx_row1)

        tx_row2 = QtWidgets.QHBoxLayout()
        self.tx_edit = QtWidgets.QLineEdit()
        self.send_btn = QtWidgets.QPushButton("Send")
        self.send_btn.clicked.connect(self.send_data)
        tx_row2.addWidget(self.tx_edit, 1)
        tx_row2.addWidget(self.send_btn)
        right.addLayout(tx_row2)

        # Status bar
        self.status = self.statusBar()
        self._update_status()

    # ----- UI helpers -----
    def _row(self, label: str, *widgets: QtWidgets.QWidget) -> QtWidgets.QHBoxLayout:
        h = QtWidgets.QHBoxLayout()
        lab = QtWidgets.QLabel(label)
        lab.setMinimumWidth(90)
        h.addWidget(lab)
        for w in widgets:
            h.addWidget(w)
        return h

    def _separator(self, title: str) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox(title)
        lay = QtWidgets.QVBoxLayout(box)
        lay.setContentsMargins(8, 8, 8, 8)
        return box

    # ----- Serial management -----
    def refresh_ports(self):
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = list_serial_ports()
        self.port_combo.addItems(ports or ["(no ports)"])
        idx = self.port_combo.findText(current)
        if idx >= 0:
            self.port_combo.setCurrentIndex(idx)

    def _open_serial(self):
        if self.ser and self.ser.is_open:
            return
        port = self.port_combo.currentText()
        if port == "(no ports)" or not port:
            raise RuntimeError("No serial port selected")
        baud = int(self.baud_combo.currentText())
        databits_map = {"5": serial.FIVEBITS, "6": serial.SIXBITS, "7": serial.SEVENBITS, "8": serial.EIGHTBITS}
        parity_map = {
            "None": serial.PARITY_NONE,
            "Even": serial.PARITY_EVEN,
            "Odd": serial.PARITY_ODD,
            "Mark": serial.PARITY_MARK,
            "Space": serial.PARITY_SPACE,
        }
        stopbits_map = {"1": serial.STOPBITS_ONE, "1.5": serial.STOPBITS_ONE_POINT_FIVE, "2": serial.STOPBITS_TWO}
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=databits_map[self.databits_combo.currentText()],
            parity=parity_map[self.parity_combo.currentText()],
            stopbits=stopbits_map[self.stopbits_combo.currentText()],
            timeout=0,  # non-blocking
            write_timeout=1,
        )
        self.reader = SerialReader(self.ser)
        self.reader.data_received.connect(self.on_data)
        self.reader.error.connect(self.on_error)
        self.reader.start()

    def _close_serial(self):
        if self.reader:
            self.reader.stop()
            self.reader = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def toggle_connection(self):
        if self.connect_btn.isChecked():
            try:
                self._open_serial()
                self.connect_btn.setText("Disconnect")
                self._append_line("[Connected]\n")
            except Exception as e:
                self.connect_btn.setChecked(False)
                self.on_error(str(e))
        else:
            self._close_serial()
            self.connect_btn.setText("Connect")
            self._append_line("[Disconnected]\n")
        self._update_status()

    # ----- RX/TX -----
    def on_data(self, data: bytes):
        if self.ts_chk.isChecked():
            ts = time.strftime("%H:%M:%S.") + f"{int((time.time()%1)*1000):03d} "
        else:
            ts = ""
        if self.hex_view_chk.isChecked():
            text = bytes_to_hex(data)
        else:
            try:
                text = data.decode(errors="replace")
            except Exception:
                text = bytes_to_hex(data)
        self._append_line(ts + text)

    def send_data(self):
        payload_str = self.tx_edit.text().strip()
        if not payload_str:
            return
        try:
            if self.hex_send_chk.isChecked():
                payload = hex_to_bytes(payload_str)
            else:
                payload = payload_str.encode()
            if self.add8_combo.currentIndex() == 1:  # ADD8
                payload = add8_checksum(payload)
            if not (self.ser and self.ser.is_open):
                raise RuntimeError("Serial not connected")
            self.ser.write(payload)
            # Show what we sent
            line = bytes_to_hex(payload) if self.hex_view_chk.isChecked() or self.hex_send_chk.isChecked() else payload.decode(errors="replace")
            self._append_line(f">> {line}")
        except Exception as e:
            self.on_error(str(e))

    def _append_line(self, s: str):
        self.rx_view.appendPlainText(s)
        if self.autoscroll_chk.isChecked():
            self.rx_view.verticalScrollBar().setValue(self.rx_view.verticalScrollBar().maximum())

    def save_log(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save log", "serial_log.txt", "Text Files (*.txt)")
        if not path:
            return
        with open(path, "w", encoding="utf-8") as f:
            f.write(self.rx_view.toPlainText())
        self.status.showMessage(f"Saved to {path}", 3000)

    def on_error(self, msg: str):
        QtWidgets.QMessageBox.critical(self, "Serial Error", msg)
        self.status.showMessage(msg, 5000)

    def _update_status(self):
        if self.ser and self.ser.is_open:
            self.status.showMessage(f"Connected: {self.ser.port} @ {self.ser.baudrate}")
        else:
            self.status.showMessage("Disconnected")


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = SerialAssistant()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
