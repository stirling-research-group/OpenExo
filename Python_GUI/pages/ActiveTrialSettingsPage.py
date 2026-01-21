try:
    from PySide6 import QtCore, QtWidgets
except ImportError as e:
    raise SystemExit("PySide6 is required. Install with: pip install PySide6") from e


class ActiveTrialSettingsPage(QtWidgets.QWidget):
    """Settings page to manually enter controller/parameter/value without text fields.
    Uses only spinboxes and checkboxes to avoid on-screen keyboard usage.
    """

    applyRequested = QtCore.Signal(list)  # [isBilateral, joint, controller, parameter, value]
    cancelRequested = QtCore.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("ActiveTrialSettingsPage")
        self._controller_matrix: list[list[str]] = []
        self._joint_controllers: dict = {}  # Maps joint name to list of controller indices
        self._bilateral_state = False  # Store bilateral state
        self._last_selection = {
            "bilateral": False,
            "joint": None,
            "controller": None,
            "parameter": None,
            "value": 0.0,
        }
        self._build_ui()
        self._load_settings()  # Load saved settings

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        title = QtWidgets.QLabel("Update Controller Settings")
        title.setAlignment(QtCore.Qt.AlignCenter)
        f = title.font(); f.setPointSize(22); title.setFont(f)
        layout.addWidget(title)

        # Joint selector at the top
        joint_selector_layout = QtWidgets.QHBoxLayout()
        lbl_joint_selector = QtWidgets.QLabel("Select Joint:")
        ljsf = lbl_joint_selector.font(); ljsf.setPointSize(18); lbl_joint_selector.setFont(ljsf)
        self.combo_joint = QtWidgets.QComboBox()
        jcf = self.combo_joint.font(); jcf.setPointSize(18); self.combo_joint.setFont(jcf)
        self.combo_joint.setMinimumHeight(56)
        self.combo_joint.currentIndexChanged.connect(self._on_joint_changed)
        joint_selector_layout.addWidget(lbl_joint_selector)
        joint_selector_layout.addWidget(self.combo_joint, 1)
        layout.addLayout(joint_selector_layout)

        # Controller/parameter matrix viewer
        self.table = QtWidgets.QTableWidget()
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        tf = self.table.font(); tf.setPointSize(16); self.table.setFont(tf)
        self.table.verticalHeader().setDefaultSectionSize(44)
        self.table.horizontalHeader().setDefaultSectionSize(140)
        # Auto-populate controller/parameter on selection
        self.table.cellClicked.connect(self._on_cell_clicked)
        layout.addWidget(self.table, 1)

        # Controls area
        form = QtWidgets.QGridLayout()
        row = 0

        self.chk_bilateral = QtWidgets.QCheckBox("Bilateral mode")
        bf = self.chk_bilateral.font(); bf.setPointSize(18); self.chk_bilateral.setFont(bf)
        self.chk_bilateral.setChecked(self._bilateral_state)  # Load saved state
        self.chk_bilateral.stateChanged.connect(self._on_bilateral_changed)
        form.addWidget(self.chk_bilateral, row, 0, 1, 2)
        row += 1

        lbl_controller = QtWidgets.QLabel("Controller")
        lcf = lbl_controller.font(); lcf.setPointSize(18); lbl_controller.setFont(lcf)
        self.combo_controller = QtWidgets.QComboBox()
        ccf = self.combo_controller.font(); ccf.setPointSize(18); self.combo_controller.setFont(ccf)
        self.combo_controller.setMinimumHeight(56)
        self.combo_controller.currentIndexChanged.connect(self._on_controller_changed)
        form.addWidget(lbl_controller, row, 0)
        form.addWidget(self.combo_controller, row, 1)
        row += 1

        lbl_param = QtWidgets.QLabel("Parameter")
        lpf = lbl_param.font(); lpf.setPointSize(18); lbl_param.setFont(lpf)
        self.combo_param = QtWidgets.QComboBox()
        cpf = self.combo_param.font(); cpf.setPointSize(18); self.combo_param.setFont(cpf)
        self.combo_param.setMinimumHeight(56)
        form.addWidget(lbl_param, row, 0)
        form.addWidget(self.combo_param, row, 1)
        row += 1

        lbl_value = QtWidgets.QLabel("Value")
        lvf = lbl_value.font(); lvf.setPointSize(18); lbl_value.setFont(lvf)
        self.spin_value = QtWidgets.QDoubleSpinBox()
        self.spin_value.setDecimals(4)
        self.spin_value.setRange(-100000.0, 100000.0)
        self.spin_value.setSingleStep(0.1)
        self.spin_value.setValue(0.0)
        svf = self.spin_value.font(); svf.setPointSize(18); self.spin_value.setFont(svf)
        self.spin_value.setMinimumHeight(56)
        form.addWidget(lbl_value, row, 0)
        form.addWidget(self.spin_value, row, 1)

        layout.addLayout(form)

        # Buttons
        btn_row = QtWidgets.QHBoxLayout()
        self.btn_apply = QtWidgets.QPushButton("Apply")
        self.btn_cancel = QtWidgets.QPushButton("Cancel")
        for b in (self.btn_apply, self.btn_cancel):
            fb = b.font(); fb.setPointSize(18); b.setFont(fb)
            b.setMinimumHeight(56)
            b.setMinimumWidth(200)
            b.setStyleSheet("padding: 10px 16px;")
        btn_row.addStretch(1)
        btn_row.addWidget(self.btn_cancel)
        btn_row.addWidget(self.btn_apply)
        layout.addLayout(btn_row)

        # Wire buttons
        self.btn_apply.clicked.connect(self._on_apply)
        self.btn_cancel.clicked.connect(self.cancelRequested.emit)

    def set_controller_matrix(self, matrix: list):
        """Populate the table with a 2D matrix: [ [Joint, Controller, p1, p2, ...], ... ]."""
        try:
            self._controller_matrix = list(matrix) if matrix else []
        except Exception:
            self._controller_matrix = []

        # Build joint mapping: { "Ankle(L) (68)": [0, 1, 2, ...], "Ankle(R) (36)": [10, 11, ...] }
        self._joint_controllers = {}
        for idx, row in enumerate(self._controller_matrix):
            if row and len(row) >= 2:
                joint_name = str(row[0])
                if joint_name not in self._joint_controllers:
                    self._joint_controllers[joint_name] = []
                self._joint_controllers[joint_name].append(idx)

        # Populate joint combo with unique joint names
        try:
            self.combo_joint.blockSignals(True)
            self.combo_joint.clear()
            joint_names = list(self._joint_controllers.keys())
            if not joint_names:
                joint_names = ["(none)"]
            self.combo_joint.addItems(joint_names)
            self.combo_joint.blockSignals(False)
            # Trigger joint change to populate table and controllers for first joint
            self._on_joint_changed(0)
            # Restore last selection after populating
            self._restore_last_selection()
        except Exception as e:
            print(f"Error populating joint combo: {e}")
            pass

    def _load_settings(self):
        """Load saved settings from file."""
        import os
        base_dir = os.path.dirname(os.path.dirname(__file__))  # Qt directory
        settings_file = os.path.join(base_dir, "Saved_Data", "gui_settings.txt")
        try:
            if os.path.exists(settings_file):
                with open(settings_file, 'r') as f:
                    lines = f.readlines()
                    for line in lines:
                        if line.startswith("bilateral="):
                            self._bilateral_state = line.split("=")[1].strip() == "True"
                            self._last_selection["bilateral"] = self._bilateral_state
                            print(f"[Settings] Loaded bilateral state: {self._bilateral_state}")
                        elif line.startswith("last_joint="):
                            val = line.split("=")[1].strip()
                            if val and val != "None":
                                self._last_selection["joint"] = val
                        elif line.startswith("last_controller="):
                            val = line.split("=")[1].strip()
                            if val and val != "None":
                                self._last_selection["controller"] = val
                        elif line.startswith("last_parameter="):
                            try:
                                val = line.split("=")[1].strip()
                                if val and val != "None":
                                    self._last_selection["parameter"] = int(val)
                            except:
                                pass
                        elif line.startswith("last_value="):
                            try:
                                val = line.split("=")[1].strip()
                                if val and val != "None":
                                    self._last_selection["value"] = float(val)
                            except:
                                pass
        except Exception as e:
            print(f"Error loading settings: {e}")

    def _save_settings(self):
        """Save settings to file."""
        import os
        base_dir = os.path.dirname(os.path.dirname(__file__))  # Qt directory
        save_dir = os.path.join(base_dir, "Saved_Data")
        settings_file = os.path.join(save_dir, "gui_settings.txt")
        try:
            os.makedirs(save_dir, exist_ok=True)
            # Read existing settings to preserve them
            existing = {}
            if os.path.exists(settings_file):
                with open(settings_file, 'r') as f:
                    for line in f.readlines():
                        if '=' in line:
                            key, val = line.strip().split('=', 1)
                            existing[key] = val
            # Update with current values (only save non-None values)
            existing["bilateral"] = str(self._bilateral_state)
            
            joint = self._last_selection.get("joint")
            if joint and joint != "None":
                existing["last_joint"] = str(joint)
            
            controller = self._last_selection.get("controller")
            if controller and controller != "None":
                existing["last_controller"] = str(controller)
            
            parameter = self._last_selection.get("parameter")
            if parameter is not None:
                existing["last_parameter"] = str(parameter)
            
            value = self._last_selection.get("value")
            if value is not None:
                existing["last_value"] = str(value)
            # Write all settings
            with open(settings_file, 'w') as f:
                for key, val in existing.items():
                    f.write(f"{key}={val}\n")
            print(f"[Settings] Saved settings to {settings_file}")
        except Exception as e:
            print(f"Error saving settings: {e}")

    def _on_bilateral_changed(self, state):
        """Save bilateral state when checkbox changes."""
        self._bilateral_state = bool(state)
        self._save_settings()

    def _restore_last_selection(self):
        """Restore UI controls to last saved selection."""
        try:
            print(f"[Settings] Attempting to restore last selection: {self._last_selection}")
            
            # Restore bilateral checkbox
            bilateral = self._last_selection.get("bilateral", False)
            self.chk_bilateral.blockSignals(True)
            self.chk_bilateral.setChecked(bilateral)
            self.chk_bilateral.blockSignals(False)
            
            # Restore joint selection
            joint_name = self._last_selection.get("joint")
            if joint_name:
                self.combo_joint.blockSignals(True)
                idx = self.combo_joint.findText(joint_name)
                if idx >= 0:
                    self.combo_joint.setCurrentIndex(idx)
                    self.combo_joint.blockSignals(False)
                    self._on_joint_changed(idx)
                else:
                    self.combo_joint.blockSignals(False)
                    print(f"[Settings] Joint '{joint_name}' not found in dropdown")
            
            # Restore controller selection
            controller_name = self._last_selection.get("controller")
            if controller_name:
                self.combo_controller.blockSignals(True)
                idx = self.combo_controller.findText(controller_name)
                if idx >= 0:
                    self.combo_controller.setCurrentIndex(idx)
                    self.combo_controller.blockSignals(False)
                    self._on_controller_changed(idx)
                    print(f"[Settings] Restored controller: {controller_name} at index {idx}")
                else:
                    self.combo_controller.blockSignals(False)
                    print(f"[Settings] Controller '{controller_name}' not found in dropdown")
            
            # Restore parameter selection
            param_idx = self._last_selection.get("parameter", 0)
            if param_idx is None:
                param_idx = 0
            self.combo_param.blockSignals(True)
            if param_idx < self.combo_param.count() and param_idx >= 0:
                self.combo_param.setCurrentIndex(param_idx)
                print(f"[Settings] Restored parameter index: {param_idx}")
            self.combo_param.blockSignals(False)
            
            # Restore value
            value = self._last_selection.get("value", 0.0)
            if value is None:
                value = 0.0
            self.spin_value.setValue(float(value))
            
            print(f"[Settings] Successfully restored last selection")
        except Exception as e:
            print(f"[Settings] Error restoring last selection: {e}")
            import traceback
            traceback.print_exc()

    @QtCore.Slot()
    def _on_apply(self):
        """Emit the selected joint, controller, parameter, and value."""
        try:
            is_bilateral = self.chk_bilateral.isChecked()
            
            # Get the selections
            joint_idx = self.combo_joint.currentIndex()
            joint_name = self.combo_joint.itemText(joint_idx)
            controller_local_idx = self.combo_controller.currentIndex()
            controller_name = self.combo_controller.currentText()
            parameter_idx = self.combo_param.currentIndex()
            parameter_name = self.combo_param.currentText()
            value = float(self.spin_value.value())
            
            print(f"\n=== Apply Settings ===")
            print(f"Joint: {joint_name} (dropdown idx: {joint_idx})")
            print(f"Controller: {controller_name} (local idx: {controller_local_idx})")
            print(f"Parameter: {parameter_name} (idx: {parameter_idx})")
            print(f"Value: {value}")
            print(f"Bilateral: {is_bilateral}")
            
            # Get the actual controller matrix index
            if joint_name in self._joint_controllers:
                controller_indices = self._joint_controllers[joint_name]
                if controller_local_idx < len(controller_indices):
                    matrix_row_idx = controller_indices[controller_local_idx]
                    
                    if matrix_row_idx < len(self._controller_matrix):
                        row = self._controller_matrix[matrix_row_idx]
                        # Matrix format: [Joint(ID), JointID, ControllerName, ControllerID, Param1, Param2, ...]
                        # row[0] = "Ankle(L) (68)"
                        # row[1] = "68" (joint ID)
                        # row[2] = "pjmc_plus" (controller name)
                        # row[3] = "11" (controller ID)
                        # row[4:] = parameters
                        
                        # Extract joint ID from row[1]
                        joint_id_raw = None
                        if len(row) > 1:
                            try:
                                joint_id_raw = int(row[1])
                            except (ValueError, IndexError):
                                print(f"Warning: Could not parse joint ID from row[1]='{row[1]}'")
                        
                        # Prefer sending the raw joint ID from the controller list.
                        if joint_id_raw is not None:
                            joint_num = joint_id_raw
                        else:
                            joint_num = 0
                            print("Warning: Missing joint ID, defaulting to 0")
                        
                        # Extract actual controller ID from row[3]
                        controller_id = controller_local_idx  # Default to local index if parsing fails
                        if len(row) > 3:
                            try:
                                controller_id = int(row[3])
                                print(f"Extracted controller ID {controller_id} from row[3]='{row[3]}'")
                            except (ValueError, TypeError):
                                print(f"Warning: Could not parse controller ID from row[3]='{row[3]}', using local idx {controller_local_idx}")
                        else:
                            print(f"Warning: Row too short (len={len(row)}), cannot extract controller ID from row[3]")
                        
                        payload = [is_bilateral, joint_num, controller_id, parameter_idx, value]
                        print(f"Payload: is_bilateral={is_bilateral}, joint_num={joint_num} (joint_id={joint_id_raw}), controller_id={controller_id}, param_idx={parameter_idx}, value={value}")
                        print(f"Full row: {row}")
                        print(f"======================\n")
                        
                        # Save last selection for next time
                        self._last_selection = {
                            "bilateral": is_bilateral,
                            "joint": joint_name,
                            "controller": controller_name,
                            "parameter": parameter_idx,
                            "value": value,
                        }
                        print(f"[Settings] Saving last selection: {self._last_selection}")
                        self._save_settings()
                        
                        self.applyRequested.emit(payload)
                        return
            
            # Fallback if something goes wrong
            print(f"Warning: Falling back to default payload")
            payload = [is_bilateral, 1, 0, 0, value]
            self.applyRequested.emit(payload)
        except Exception as e:
            print(f"Error in _on_apply: {e}")
            import traceback
            traceback.print_exc()

    @QtCore.Slot(int, int)
    def _on_cell_clicked(self, row: int, column: int):
        """When table cell is clicked, update the dropdowns to match that selection."""
        try:
            # The table now shows filtered rows for the selected joint
            # row index in table corresponds to controller index within the joint
            self.combo_controller.setCurrentIndex(row)
            
            # Set parameter based on column
            # Table columns: 0=Controller, 1+=Params
            # So param_index is column - 1, with minimum of 0
            param_index = max(0, int(column) - 1)
            self.combo_param.setCurrentIndex(param_index)
        except Exception as e:
            print(f"Error in _on_cell_clicked: {e}")
            pass

    @QtCore.Slot(int)
    def _on_joint_changed(self, idx: int):
        """When joint selection changes, update table to show only that joint's controllers."""
        try:
            joint_name = self.combo_joint.itemText(idx)
            
            # Filter the table to show only controllers for the selected joint
            if joint_name in self._joint_controllers:
                controller_indices = self._joint_controllers[joint_name]
                filtered_rows = [self._controller_matrix[i] for i in controller_indices if i < len(self._controller_matrix)]
                
                # Determine max columns from filtered rows
                max_cols = 0
                for row in filtered_rows:
                    if len(row) > max_cols:
                        max_cols = len(row)
                if max_cols == 0:
                    max_cols = 2
                
                # Update table with filtered rows
                # Matrix format: [Joint(ID), JointID, ControllerName, ControllerID, Param1, Param2, ...]
                # We want to display: [ControllerName, Param1, Param2, ...]
                # So skip columns 0 (Joint), 1 (JointID), and 3 (ControllerID)
                self.table.clear()
                self.table.setRowCount(len(filtered_rows))
                # Count displayable columns: ControllerName + parameters
                num_params = max(0, max_cols - 4)  # Subtract Joint, JointID, ControllerName, ControllerID
                self.table.setColumnCount(1 + num_params)  # Controller + parameters
                
                # Headers: Controller and parameter names
                headers = ["Controller"] + [f"Param {i}" for i in range(1, num_params + 1)]
                self.table.setHorizontalHeaderLabels(headers)
                
                for r, data in enumerate(filtered_rows):
                    # Display controller name from column 2
                    if len(data) > 2:
                        item = QtWidgets.QTableWidgetItem(str(data[2]))
                        self.table.setItem(r, 0, item)
                    # Display parameters from column 4 onwards
                    for param_idx, c in enumerate(range(4, len(data))):
                        text = data[c] if c < len(data) else ""
                        item = QtWidgets.QTableWidgetItem(str(text))
                        self.table.setItem(r, param_idx + 1, item)
                
                self.table.resizeColumnsToContents()
                
                # Update controller dropdown
                # Matrix format: [Joint(ID), JointID, ControllerName, ControllerID, ...]
                # Controller name is at index 2
                self.combo_controller.blockSignals(True)
                self.combo_controller.clear()
                controller_names = [row[2] if len(row) > 2 else "(unknown)" for row in filtered_rows]
                if controller_names:
                    self.combo_controller.addItems(controller_names)
                else:
                    self.combo_controller.addItem("(none)")
                self.combo_controller.blockSignals(False)
                
                # Trigger parameter update for first controller
                self._on_controller_changed(0)
            else:
                # No controllers for this joint
                self.table.clear()
                self.table.setRowCount(0)
                self.table.setColumnCount(2)
                self.table.setHorizontalHeaderLabels(["Controller", "Parameters"])
                
                self.combo_controller.blockSignals(True)
                self.combo_controller.clear()
                self.combo_controller.addItem("(none)")
                self.combo_controller.blockSignals(False)
                
        except Exception as e:
            print(f"Error in _on_joint_changed: {e}")
            pass

    @QtCore.Slot(int)
    def _on_controller_changed(self, idx: int):
        """Populate parameters combo from selected controller."""
        try:
            self.combo_param.blockSignals(True)
            self.combo_param.clear()
            
            # Get the current joint
            joint_idx = self.combo_joint.currentIndex()
            joint_name = self.combo_joint.itemText(joint_idx)
            
            if joint_name in self._joint_controllers:
                controller_indices = self._joint_controllers[joint_name]
                if idx < len(controller_indices):
                    matrix_idx = controller_indices[idx]
                    if matrix_idx < len(self._controller_matrix):
                        row = self._controller_matrix[matrix_idx]
                        # Matrix format: [Joint(ID), JointID, ControllerName, ControllerID, Param1, Param2, ...]
                        # Parameters start at index 4
                        params = row[4:] if len(row) > 4 else []
                        if not params:
                            params = ["(no params)"]
                        self.combo_param.addItems([str(p) for p in params])
                    else:
                        self.combo_param.addItem("(no params)")
                else:
                    self.combo_param.addItem("(no params)")
            else:
                self.combo_param.addItem("(no params)")
        except Exception as e:
            print(f"Error in _on_controller_changed: {e}")
            pass
        finally:
            try:
                self.combo_param.blockSignals(False)
            except Exception:
                pass
