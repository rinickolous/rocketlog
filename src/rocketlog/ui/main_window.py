from PySide6 import QtWidgets, QtCore

from rocketlog.ui.video_panel import VideoPanel


class MainWindow(QtWidgets.QMainWindow):
    """
    Main application window.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("RocketLog")

        self.init_main_panel()
        self.init_status_panel()

        root = QtWidgets.QWidget()
        root_layout = QtWidgets.QVBoxLayout(root)
        root_layout.setSpacing(10)

        root_layout.addWidget(self.status_panel)
        root_layout.addWidget(self.main_panel)

        self.setCentralWidget(root)

    # ---------------------------------------- #

    def init_main_panel(self):
        """
        Main Panel with Tabs for Rocket Telemetry and CanSat Telemetry
        """

        self.main_panel = QtWidgets.QGroupBox()
        self.main_panel.setObjectName("main_panel")

        self.main_layout = QtWidgets.QGridLayout(self.main_panel)

        self.main_tabs = QtWidgets.QTabWidget()
        self.main_tabs.setObjectName("main_tabs")
        self.main_layout.addWidget(self.main_tabs)

        self.init_main_tab("Rocket")
        self.init_main_tab("CanSat")

    # ---------------------------------------- #

    def init_main_tab(self, name: str):
        # Telemetry Tab
        tab = QtWidgets.QWidget()
        tab_layout = QtWidgets.QGridLayout(tab)
        video_panel = VideoPanel(self)
        telemetry_preview_panel = QtWidgets.QTextEdit("Telemetry Preview Placeholder")
        telemetry_graph_panel = QtWidgets.QTextEdit("Telemetry Graph Placeholder")
        telemetry_log_panel = QtWidgets.QTextEdit("Telemetry Log Placeholder")

        tab_layout.addWidget(video_panel, 0, 0)
        tab_layout.addWidget(telemetry_preview_panel, 0, 1)
        tab_layout.addWidget(telemetry_log_panel, 1, 0)
        tab_layout.addWidget(telemetry_graph_panel, 1, 1)

        tab_layout.setColumnStretch(0, 2)  # left side wider
        tab_layout.setColumnStretch(1, 1)
        tab_layout.setRowStretch(0, 2)
        tab_layout.setRowStretch(1, 1)

        self.main_tabs.addTab(tab, name)

    # ---------------------------------------- #

    def init_status_panel(self):
        """
        Status Widgets Panel (GPS Status, Cyberdeck Battery, etc.)
        """

        self.status_panel = QtWidgets.QFrame()
        self.status_panel.setObjectName("status_panel")

        self.chip_gps = self.create_status_chip("GPS", "NO FIX", "chip_gps")
        self.chip_storage = self.create_status_chip("STORAGE", "OK", "chip_storage")

        status_layout = QtWidgets.QHBoxLayout(self.status_panel)
        status_layout.setContentsMargins(5, 5, 5, 5)
        status_layout.setSpacing(10)

        status_layout.addWidget(self.chip_gps)
        status_layout.addWidget(self.chip_storage)

    # ---------------------------------------- #

    def create_status_chip(
        self, title_value: str, initial_value: str, object_name: str
    ) -> QtWidgets.QFrame:
        """
        Creates a status chip with a title and a value. Used for GPS status, storage status, etc.
        """

        container = QtWidgets.QFrame()
        container.setObjectName(f"{object_name}_container")

        container_layout = QtWidgets.QVBoxLayout(container)
        title = QtWidgets.QLabel(title_value)
        title.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)

        label = QtWidgets.QLabel(initial_value)
        label.setObjectName(object_name)
        label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        label.setMinimumWidth(100)

        container_layout.addWidget(title)
        container_layout.addWidget(label)

        return container
