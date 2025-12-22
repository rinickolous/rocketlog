APP_QSS = """
QMainWindow, QWidget {
	background: #0b0f14;
	color: #c8d2dc;
	font-family: sans-serif;
}

#AnnBar {
	background: #0f1620;
	border: 1px solid #1b2a34;
	border-radius: 10px;
}

QGroupBox {
	border: 1px solid #1b2a34;
	border-radius: 12px;
	margin-top: 12px;
	padding: 10px;
	background: #0f1620;
}

QGroupBox::title {
	subcontrol-origin: margin;
	left: 12px;
	padding: 0 6px;
	color: #8aa2b2;
}

QPushButton {
	background: #13202b;
	border: 1px solid #1b2a34;
	border-radius: 10px;
	padding: 10px 14px;
}

QPushButton:hover {
	border: 1px solid #2a4353;
}

QPushButton:pressed {
	background: #0f1620;
}

QPushButton:disabled {
	color: #5b6c77;
	background: #0f1620;
}

QLabel#ChipNeutral, QLabel#ChipGood, QLabel#ChipCaution, QLabel#ChipWarn {
	padding: 6px 10px;
	border-radius: 10px;
	border: 1px solid #1b2a34;
	font-weight: 700;
	letter-spacing: 0.5px;
}

QLabel#ChipNeutral { color: #c8d2dc; background: rgba(255,255,255,0.03); }
QLabel#ChipGood    { color: #2fe37a; background: rgba(47,227,122,0.06); }
QLabel#ChipCaution { color: #ffcc66; background: rgba(255,204,102,0.07); }
QLabel#ChipWarn    { color: #ff4b4b; background: rgba(255,75,75,0.08); }
"""

