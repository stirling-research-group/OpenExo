import asyncio
import os
import sys
import threading

from typing import List, Tuple

try:
    from PySide6 import QtCore, QtWidgets
except ImportError as e:
    raise SystemExit("PySide6 is required. Install with: pip install PySide6") from e


from services.QtExoDeviceManager import QtExoDeviceManager


UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  # Nordic UART Service


class DeviceScannerWorker(QtCore.QObject):
    resultsReady = QtCore.Signal(list)  # List[Tuple[str, str]] (name, address)
    error = QtCore.Signal(str)

    def __init__(self, qt_dev: QtExoDeviceManager):
        super().__init__()
        self._qt_dev = qt_dev
        self._qt_dev.scanResults.connect(self._forward)
        self._qt_dev.error.connect(self.error)

    @QtCore.Slot()
    def scan_once(self):
        self._qt_dev.scan()

    @QtCore.Slot(list)
    def _forward(self, results):
        self.resultsReady.emit(results)


class ScanWindowQt(QtWidgets.QWidget):
    # Signal to request a connection via the Qt device manager (address)
    connectRequested = QtCore.Signal(str)
    SETTINGS_FILE = os.path.join(os.path.dirname(os.path.dirname(__file__)), "Saved_Data", "saved_device.txt")

    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenExo - Scan (Qt)")
        # Compact default size (resizable)
        self.setMinimumSize(700, 400)
        self.resize(900, 500)

        self.selected_address: str | None = None
        self.selected_name: str | None = None
        self._qt_dev: QtExoDeviceManager | None = None
        self._scanner: DeviceScannerWorker | None = None
        self._connected = False
        self._pending_scan = False

        self._build_ui()
        self._wire_workers()
        self._load_saved_device()

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 0, 8, 3)  # Zero top margin - start at very top
        layout.setSpacing(0)  # No automatic spacing
        
        # Add spacing at the top
        layout.addSpacing(20)

        # Header row with logos and centered title
        header_row = QtWidgets.QHBoxLayout()
        header_row.setContentsMargins(0, 0, 0, 0)
        header_row.setSpacing(4)
        
        # Add OpenExo logo at left (smaller)
        try:
            from PySide6 import QtGui
            openexo_logo_label = QtWidgets.QLabel()
            openexo_logo_label.setContentsMargins(0, 0, 0, 0)
            base_dir = os.path.dirname(os.path.dirname(__file__))  # Qt directory
            logo_path = os.path.join(base_dir, "Images", "OpenExo.png")
            logo_pixmap = QtGui.QPixmap(logo_path)
            if not logo_pixmap.isNull():
                scaled_logo = logo_pixmap.scaled(160, 40, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
                openexo_logo_label.setPixmap(scaled_logo)
                openexo_logo_label.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
                header_row.addWidget(openexo_logo_label)
            else:
                print(f"Failed to load OpenExo logo from: {logo_path}")
        except Exception as e:
            print(f"Error loading OpenExo logo: {e}")
        
        # Add centered title
        self.title = QtWidgets.QLabel("OpenExo GUI - v2.0")
        self.title.setAlignment(QtCore.Qt.AlignCenter)
        self.title.setContentsMargins(0, 0, 0, 0)
        font = self.title.font()
        font.setPointSize(20)  # Larger title
        self.title.setFont(font)
        header_row.addWidget(self.title, 1)  # Stretch factor to center
        
        try:
            from PySide6 import QtGui
            lab_logo_label = QtWidgets.QLabel()
            lab_logo_label.setContentsMargins(0, 0, 0, 0)
            base_dir = os.path.dirname(os.path.dirname(__file__))  # Qt directory
            lab_logo_path = os.path.join(base_dir, "Images", "LabLogo.png")
            lab_logo_pixmap = QtGui.QPixmap(lab_logo_path)
            if not lab_logo_pixmap.isNull():
                scaled_lab_logo = lab_logo_pixmap.scaled(200, 32, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
                lab_logo_label.setPixmap(scaled_lab_logo)
                lab_logo_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                header_row.addWidget(lab_logo_label)
            else:
                print(f"Failed to load Lab logo from: {lab_logo_path}")
        except Exception as e:
            print(f"Error loading Lab logo: {e}")
        
        layout.addLayout(header_row)
        layout.addSpacing(35)  # Add more spacing between header and status to push content down

        self.status = QtWidgets.QLabel("Not Connected")
        self.status.setAlignment(QtCore.Qt.AlignCenter)
        self.status.setContentsMargins(0, 0, 0, 0)
        f2 = self.status.font(); f2.setPointSize(14); self.status.setFont(f2)
        layout.addWidget(self.status)
        layout.addSpacing(8)  # Add spacing between status and buttons

        # Button row
        btn_row = QtWidgets.QHBoxLayout()
        btn_row.setSpacing(6)
        btn_row.setContentsMargins(0, 0, 0, 0)
        self.btn_scan = QtWidgets.QPushButton("1. Start Scan")
        self.btn_load = QtWidgets.QPushButton("Load Saved Device")
        btn_row.addWidget(self.btn_scan)
        btn_row.addWidget(self.btn_load)
        layout.addLayout(btn_row)
        layout.addSpacing(2)

        # Devices list - fills space between top and bottom buttons
        self.list_devices = QtWidgets.QListWidget()
        self.list_devices.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        lf = self.list_devices.font(); lf.setPointSize(14); self.list_devices.setFont(lf)
        self.list_devices.setStyleSheet("QListWidget::item{ height: 36px; padding: 3px 6px; }")
        self.list_devices.setMinimumHeight(100)  # Minimum height
        layout.addWidget(self.list_devices, 1)  # Stretch factor 1 to fill space
        
        layout.addSpacing(5)

        # Action row (Connect, Calibrate Torque, Start Trial) - at bottom
        action_row = QtWidgets.QHBoxLayout()
        action_row.setSpacing(6)
        action_row.setContentsMargins(0, 0, 0, 0)
        self.btn_save_connect = QtWidgets.QPushButton("2. Connect")
        self.btn_save_connect.setEnabled(False)  # Disabled initially
        self.btn_start_trial = QtWidgets.QPushButton("4. Start Trial")
        self.btn_start_trial.setEnabled(False)
        self.btn_calibrate_torque = QtWidgets.QPushButton("3. Calibrate Torque")
        self.btn_calibrate_torque.setEnabled(False)
        action_row.addWidget(self.btn_save_connect)
        action_row.addWidget(self.btn_calibrate_torque)
        action_row.addWidget(self.btn_start_trial)
        layout.addLayout(action_row)
        
        # Add spacing at the bottom
        layout.addSpacing(20)

        # Signals
        self.btn_scan.clicked.connect(self.on_scan)
        self.btn_load.clicked.connect(self.on_load_saved)
        self.btn_save_connect.clicked.connect(self.on_save_and_connect)
        self.btn_calibrate_torque.clicked.connect(self.on_calibrate_torque)
        self.list_devices.itemSelectionChanged.connect(self.on_selected)

        # Touch-friendly styling for iPad (compact)
        def _style(btn: QtWidgets.QPushButton):
            fb = btn.font(); fb.setPointSize(14); btn.setFont(fb)
            btn.setMinimumHeight(44)  # More compact
            btn.setMaximumHeight(44)  # Lock height
            btn.setMinimumWidth(160)  # Smaller width
            btn.setStyleSheet("padding: 6px 12px; margin: 0px;")  # Minimal padding

        for b in (self.btn_scan, self.btn_load, self.btn_save_connect, self.btn_start_trial, self.btn_calibrate_torque):
            _style(b)

    def _wire_workers(self):
        # Use the shared QtExoDeviceManager instance provided by MainWindow after construction
        # We lazy-bind scanner to the device manager via bind_device_manager()
        self._scanner: DeviceScannerWorker | None = None
        # Connections are made in bind_device_manager()

    def _load_saved_device(self):
        try:
            if os.path.exists(self.SETTINGS_FILE):
                with open(self.SETTINGS_FILE, "r") as f:
                    addr = f.read().strip()
                if addr:
                    self.status.setText(f"Saved device available: {addr}")
                    # Do not auto-enable connect until user selects or loads
                    return
        except Exception:
            pass

    # UI handlers
    @QtCore.Slot()
    def on_scan(self):
        if self._scanner is None:
            self.status.setText("Scanner not ready")
            return
        self.btn_scan.setEnabled(False)
        self.btn_save_connect.setEnabled(False)
        self.list_devices.clear()
        self._pending_scan = True
        if self._qt_dev is not None and self._connected:
            try:
                self.status.setText("Disconnecting…")
                self._qt_dev.disconnect()
            except Exception:
                self._connected = False
                self._start_scan_now()
        else:
            self._start_scan_now()

    @QtCore.Slot()
    def on_load_saved(self):
        if os.path.exists(self.SETTINGS_FILE):
            try:
                with open(self.SETTINGS_FILE, "r") as f:
                    addr = f.read().strip()
                if addr:
                    self.selected_address = addr
                    self.status.setText(f"Connecting to saved device: {addr}")
                    # Disable Load and Save & Connect buttons during connection
                    # Keep Start Scan enabled so user can disconnect and start new scan
                    self.btn_load.setEnabled(False)
                    self.btn_save_connect.setEnabled(False)
                    # Auto-connect to saved device
                    self.connectRequested.emit(addr)
                    return
            except Exception:
                pass
        self.status.setText("No saved device found")

    @QtCore.Slot()
    def on_selected(self):
        items = self.list_devices.selectedItems()
        if items:
            item = items[0]
            self.selected_name = item.data(QtCore.Qt.UserRole)[0]
            self.selected_address = item.data(QtCore.Qt.UserRole)[1]
            self.btn_save_connect.setEnabled(True)
        else:
            self.selected_name = None
            self.selected_address = None
            self.btn_save_connect.setEnabled(False)

    @QtCore.Slot()
    def on_save_and_connect(self):
        if not self.selected_address:
            self.status.setText("Select a device first")
            return
        # Save selection
        try:
            os.makedirs(os.path.dirname(self.SETTINGS_FILE), exist_ok=True)
            with open(self.SETTINGS_FILE, "w") as f:
                f.write(self.selected_address)
        except Exception as ex:
            self.status.setText(f"Save failed: {ex}")
            return

        self.status.setText(f"Connecting to: {self.selected_name or ''} {self.selected_address}")
        self.btn_save_connect.setEnabled(False)
        # Emit request for Qt device manager to handle connection
        self.connectRequested.emit(self.selected_address)

    @QtCore.Slot()
    def on_calibrate_torque(self):
        if self._qt_dev is None:
            self.status.setText("Device manager not ready")
            return
        try:
            # Send torque calibration command
            self._qt_dev.calibrateTorque()

            # Keep Start Trial disabled for a short delay to allow calibration to settle
            self.btn_start_trial.setEnabled(False)

            self.status.setText("Torque calibration sent. Start Trial will be enabled in 1 second...")

            # Enable Start Trial after 3 seconds (only if still connected)
            def _enable_start_trial_if_connected():
                if self._connected and self._qt_dev is not None:
                    self.btn_start_trial.setEnabled(True)

            QtCore.QTimer.singleShot(1000, _enable_start_trial_if_connected)
        except Exception as ex:
            self.status.setText(f"Torque calibration failed: {ex}")

    # Called by MainWindow after it creates QtExoDeviceManager
    def bind_device_manager(self, qt_dev: QtExoDeviceManager):
        self._qt_dev = qt_dev
        self._scanner = DeviceScannerWorker(qt_dev)
        self._scanner.resultsReady.connect(self._on_scan_results)
        self._scanner.error.connect(self._on_error)
        qt_dev.connected.connect(self._on_device_connected)
        qt_dev.disconnected.connect(self._on_device_disconnected)

    # Worker callbacks
    @QtCore.Slot(list)
    def _on_scan_results(self, results: List[Tuple[str, str]]):
        self._pending_scan = False
        self.btn_scan.setEnabled(True)
        self.list_devices.clear()
        
        # Always re-enable Load button if saved device exists
        if os.path.exists(self.SETTINGS_FILE):
            try:
                with open(self.SETTINGS_FILE, "r") as f:
                    if f.read().strip():
                        self.btn_load.setEnabled(True)
            except Exception:
                pass
        
        if not results:
            self.status.setText("No devices found")
            return
        self.status.setText("Scan complete")
        for name, addr in results:
            item = QtWidgets.QListWidgetItem(f"{name} - {addr}")
            item.setData(QtCore.Qt.UserRole, (name, addr))
            self.list_devices.addItem(item)

    @QtCore.Slot(str, str)
    def _on_device_connected(self, name: str, address: str):
        self._connected = True
        # Keep Save & Connect disabled after connection
        self.btn_save_connect.setEnabled(False)

    @QtCore.Slot()
    def _on_device_disconnected(self):
        self._connected = False
        if self._pending_scan:
            self._start_scan_now()

    def _start_scan_now(self):
        self.status.setText("Scanning…")
        if self._scanner is None:
            self.status.setText("Scanner not ready")
            self.btn_scan.setEnabled(True)
            self._pending_scan = False
            return
        self._scanner.scan_once()

    @QtCore.Slot(bool, str)
    def _on_connected(self, ok: bool, msg: str):
        self.btn_save_connect.setEnabled(True)
        if ok:
            self.status.setText(f"Connected: {self.selected_name or ''} {self.selected_address}")
            self.btn_start_trial.setEnabled(True)
        else:
            self.status.setText(f"Connection failed: {msg}")

    @QtCore.Slot(str)
    def _on_error(self, message: str):
        self.status.setText(message)

    @QtCore.Slot(str)
    def _on_connect(self, message: str):
        self.status.setText(message)

def main():
    app = QtWidgets.QApplication(sys.argv)
    w = ScanWindowQt()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

