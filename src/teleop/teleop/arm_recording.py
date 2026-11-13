"""Position-recording panel and entry point, separate from manual controls."""
from PyQt5 import QtCore, QtWidgets


class CapturePanel(QtWidgets.QGroupBox):
    def __init__(self, window, port, capture):
        super().__init__('Position recording')
        self.window, self.port, self.capture = window, port, capture
        layout = QtWidgets.QVBoxLayout(self)
        for name in ('capture_label','capture_result','capture_readiness','capture_progress'):
            label = QtWidgets.QLabel();label.setWordWrap(True)
            label.setTextFormat(QtCore.Qt.PlainText)
            setattr(window,name,label);layout.addWidget(label)
        font=window.capture_result.font();font.setBold(True);window.capture_result.setFont(font)
        window.capture_button=QtWidgets.QPushButton('CAPTURE SHOWN STEP (C) — save measured position')
        window.capture_button.setFocusPolicy(QtCore.Qt.NoFocus)
        window.capture_button.clicked.connect(lambda:window.invoke(lambda:window.keys.press('c')))
        layout.addWidget(window.capture_button)
        window.capture_next=QtWidgets.QPushButton();window.capture_next.setFocusPolicy(QtCore.Qt.NoFocus)
        window.capture_next.clicked.connect(window.select_next_capture);layout.addWidget(window.capture_next)

    def refresh(self):
        w,p,c = self.window,self.port,self.capture
        w.capture_label.setText(c.describe(p.selected))
        w.capture_result.setText(w.keys.capture_message)
        w.capture_progress.setText(c.progress())
        remaining=c.remaining_joints()
        reason=w.keys.capture_block_reason()
        w.capture_readiness.setText('Saving to disk…' if c.pending else
            'All requested positions are saved.' if not remaining else
            'Selected motor is saved. Continue with the next motor.' if p.selected not in remaining else
            reason or 'Ready — press C or click Capture to save this position.')
        w.capture_button.setEnabled(not (c.pending or w.keys.failed or w.keys.recovering) and p.selected in remaining)
        w.capture_next.setText(f'Finish the shown A{p.selected} step before Next' if p.selected in remaining else
            f'NEXT: select A{remaining[0]} ({remaining[0]}) — other joints keep holding' if remaining else 'All positions saved')
        w.capture_next.setEnabled(bool(remaining) and p.selected not in remaining and
                                 not (c.pending or w.keys.failed or w.keys.recovering))
        for label in (w.capture_label,w.capture_result,w.capture_readiness,w.capture_progress):
            label.setMinimumHeight(label.heightForWidth(max(1,label.width())))


def main(argv=None):
    from .arm_keyboard import main as run
    return run(argv,recording=True)
