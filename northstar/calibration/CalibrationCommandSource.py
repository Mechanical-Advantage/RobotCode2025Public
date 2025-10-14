# Copyright (c) 2025 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import ntcore
from config.config import ConfigStore


class CalibrationCommandSource:
    def get_calibrating(self) -> bool:
        return False

    def get_capture_flag(self) -> bool:
        return False


class NTCalibrationCommandSource(CalibrationCommandSource):
    _init_complete: bool = False
    _active_entry: ntcore.BooleanEntry
    _capture_flag_entry: ntcore.BooleanEntry

    def _init(self, config_store: ConfigStore):
        if not self._init_complete:
            nt_table = ntcore.NetworkTableInstance.getDefault().getTable(
                "/" + config_store.local_config.device_id + "/calibration"
            )
            self._active_entry = nt_table.getBooleanTopic("active").getEntry(False)
            self._capture_flag_entry = nt_table.getBooleanTopic("capture_flag").getEntry(False)
            self._active_entry.set(False)
            self._capture_flag_entry.set(False)
            self._init_complete = True

    def get_calibrating(self, config_store: ConfigStore) -> bool:
        self._init(config_store)
        calibrating = self._active_entry.get()
        if not calibrating:
            self._capture_flag_entry.set(False)
        return calibrating

    def get_capture_flag(self, config_store: ConfigStore) -> bool:
        self._init(config_store)
        if self._capture_flag_entry.get():
            self._capture_flag_entry.set(False)
            return True
        return False
