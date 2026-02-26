# rocketlog/ui/style.py

APP_QSS = """
/*
 * RocketLog — industrial dark theme
 *
 * Hard edges, monochrome ramp + green accent (retro ground-station feel).
 *
 * Palette:
 *   BG:          #0C0F12
 *   PANEL:       #10151A
 *   WELL:        #0A0D10
 *   BORDER:      #27313A
 *   BORDER_STR:  #36424D
 *   TEXT:        #D6DADF
 *   TEXT_MUTED:  #9AA6B2
 *   DISABLED:    #66727D
 *   ACCENT:      #4dcc66
 */

QMainWindow, QWidget {
    background: #0C0F12;
    color: #D6DADF;
    font-family: sans-serif;
    font-size: 13px;
}

/* ---- Status bar ---- */

#StatusBar {
    background: #10151A;
    border: 2px solid #27313A;
}

/* ---- Tab widget ---- */

QTabWidget::pane {
    border: 2px solid #27313A;
    background: #0C0F12;
}

QTabBar::tab {
    background: #10151A;
    border: 2px solid #27313A;
    border-bottom: none;
    padding: 6px 18px;
    font-weight: 700;
    letter-spacing: 0.8px;
    color: #9AA6B2;
}

QTabBar::tab:selected {
    background: #0C0F12;
    color: #D6DADF;
    border-bottom: 2px solid #0C0F12;
}

QTabBar::tab:hover:!selected {
    color: #D6DADF;
}

/* ---- Panels ---- */

#Panel {
    background: #10151A;
    border: 2px solid #27313A;
}

#PanelTitle {
    background: transparent;
    border: none;
    font-weight: 800;
    font-size: 11px;
    letter-spacing: 1.2px;
    color: #9AA6B2;
    padding: 0px;
}

/* ---- Telemetry rows ---- */

#TelemetryRow {
    background: transparent;
}

#TelemetryKey {
    font-size: 11px;
    font-weight: 700;
    letter-spacing: 0.8px;
    color: #9AA6B2;
}

#TelemetryVal {
    font-family: monospace;
    font-size: 13px;
    color: #D6DADF;
}

/* ---- Log view ---- */

#LogView {
    background: #0A0D10;
    border: none;
    font-family: monospace;
    font-size: 12px;
    padding: 2px;
}

/* ---- Inputs ---- */

QPlainTextEdit, QLineEdit, QComboBox {
    background: #0A0D10;
    border: 2px solid #27313A;
    padding: 6px;
    selection-background-color: #4dcc66;
    selection-color: #0C0F12;
}

QPlainTextEdit:focus, QLineEdit:focus, QComboBox:focus {
    border: 2px solid #4dcc66;
}

QComboBox::drop-down {
    border-left: 2px solid #27313A;
    width: 22px;
    background: #10151A;
}

/* ---- Buttons ---- */

QPushButton {
    background: #10151A;
    border: 2px solid #27313A;
    padding: 8px 14px;
    font-weight: 600;
    letter-spacing: 0.4px;
}

QPushButton:hover  { border: 2px solid #36424D; }
QPushButton:pressed { background: #0A0D10; }
QPushButton:focus  { border: 2px solid #4dcc66; }
QPushButton:disabled { color: #66727D; background: #0A0D10; }

/* ---- Status chips ---- */

QLabel#ChipNeutral,
QLabel#ChipGood,
QLabel#ChipCaution,
QLabel#ChipWarn {
    padding: 5px 10px;
    border: 2px solid #27313A;
    font-weight: 800;
    font-size: 11px;
    letter-spacing: 1.0px;
}

QLabel#ChipNeutral  { color: #D6DADF;  background: rgba(255,255,255,0.03); }
QLabel#ChipGood     { color: #4dcc66;  background: rgba(77,204,102,0.10);  border: 2px solid rgba(77,204,102,0.35); }
QLabel#ChipCaution  { color: #ffcc66;  background: rgba(255,204,102,0.12); border: 2px solid rgba(255,204,102,0.35); }
QLabel#ChipWarn     { color: #ff4b4b;  background: rgba(255,75,75,0.12);   border: 2px solid rgba(255,75,75,0.35); }

/* ---- Scroll bars (minimal) ---- */

QScrollBar:vertical {
    background: #0A0D10;
    width: 8px;
    margin: 0;
}
QScrollBar::handle:vertical {
    background: #27313A;
    min-height: 20px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }

QScrollBar:horizontal {
    background: #0A0D10;
    height: 8px;
    margin: 0;
}
QScrollBar::handle:horizontal {
    background: #27313A;
    min-width: 20px;
}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }
"""
