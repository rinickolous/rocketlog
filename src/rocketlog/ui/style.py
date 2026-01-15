APP_QSS = """
/*
Subtle industrial dark theme
- Hard edges (minimal rounding)
- Monochrome ramp + single blue accent
*/

/* ---- Palette (documentation) ----
BG:          #0C0F12
PANEL:       #10151A
WELL:        #0A0D10
BORDER:      #27313A
BORDER_STR:  #36424D
TEXT:        #D6DADF
TEXT_MUTED:  #9AA6B2
DISABLED:    #66727D
ACCENT:      #7AA2FF
*/

QMainWindow, QWidget {
	background: #0C0F12;
	color: #D6DADF;
	font-family: sans-serif;
	font-size: 13px;
}

#AnnBar {
	background: #10151A;
	border: 2px solid #27313A;
	border-radius: 0px;
}

QGroupBox {
	background: #10151A;
	border: 2px solid #27313A;
	border-radius: 0px;
	/* Original (working) title layout */
	margin-top: 8px;
	padding: 10px;
	padding-top: 16px;
}

QGroupBox::title {
	/* Built-in titles are not used; panels render header labels. */
	padding: 0px;
}

QGroupBox#Panel {
	margin-top: 0px;
	padding-top: 10px;
}

QLabel#PanelTitle {
	/* Match chip typography, but keep title visually "unboxed" */
	padding: 0px;
	border: none;
	background: transparent;
	font-family: sans-serif;
	font-weight: 800;
	font-size: 13px;
	letter-spacing: 0.9px;
	text-transform: uppercase;
	color: #D6DADF;
}

/* Inputs as inset wells */
QPlainTextEdit, QLineEdit, QComboBox {
	background: #0A0D10;
	border: 2px solid #27313A;
	border-radius: 0px;
	padding: 6px;
	selection-background-color: #7AA2FF;
	selection-color: #0C0F12;
}

QPlainTextEdit {
	font-family: monospace;
	padding: 2px;
}

QPlainTextEdit:focus, QLineEdit:focus, QComboBox:focus {
	border: 2px solid #7AA2FF;
}

QComboBox::drop-down {
	border-left: 2px solid #27313A;
	width: 22px;
	background: #10151A;
}

QComboBox::down-arrow {
	/* Reliable arrow: let the platform draw it (QSS overrides can easily break). */
}

QPushButton {
	background: #10151A;
	border: 2px solid #27313A;
	border-radius: 0px;
	padding: 10px 14px;
	font-weight: 600;
	letter-spacing: 0.4px;
}

QPushButton:hover {
	border: 2px solid #36424D;
}

QPushButton:pressed {
	background: #0A0D10;
}

QPushButton:focus {
	border: 2px solid #7AA2FF;
}

QPushButton:disabled {
	color: #66727D;
	background: #0A0D10;
	border: 2px solid #27313A;
}

/* Status chips: monochrome plates; state conveyed by text + subtle fill */
QLabel#ChipNeutral, QLabel#ChipGood, QLabel#ChipCaution, QLabel#ChipWarn {
	padding: 6px 10px;
	border-radius: 0px;
	border: 2px solid #27313A;
	font-family: sans-serif;
	font-weight: 800;
	font-size: 13px;
	letter-spacing: 0.9px;
	text-transform: uppercase;
}

QLabel#ChipNeutral { color: #D6DADF; background: rgba(255,255,255,0.03); }
/* bring back bolder semantics for quick glance */
QLabel#ChipGood    { color: #2fe37a; background: rgba(47,227,122,0.10); border: 2px solid rgba(47,227,122,0.35); }
QLabel#ChipCaution { color: #ffcc66; background: rgba(255,204,102,0.12); border: 2px solid rgba(255,204,102,0.35); }
QLabel#ChipWarn    { color: #ff4b4b; background: rgba(255,75,75,0.12); border: 2px solid rgba(255,75,75,0.35); }
"""

