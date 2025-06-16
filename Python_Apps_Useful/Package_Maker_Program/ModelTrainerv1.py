#!/usr/bin/env python3
import os
# Force XCB backend and point to system Qt plugins
os.environ["QT_QPA_PLATFORM"] = "xcb"
import PyQt5
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = os.path.join(
    os.path.dirname(PyQt5.__file__), "Qt5", "plugins", "platforms"
)

import sys
import glob
import subprocess
import webbrowser
import cv2
import numpy as np
import requests

from PyQt5.QtCore import Qt, QPoint
from PyQt5.QtGui import QImage, QPixmap, QKeySequence
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QComboBox, QMessageBox,
    QTabWidget, QFileDialog, QShortcut, QSlider
)

class AnnotationTab(QWidget):
    def __init__(self):
        super().__init__()
        self.image_paths   = []
        self.curr_idx      = -1
        # history per image: list of (mask, class_label)
        self.mask_history  = {}
        # color map for classes
        self.colormap      = {
            "grass": (0,255,0),
            "lines": (0,0,255),
            "road":  (255,0,0)
        }
        # auto tool parameters
        self.gray_tol      = 20
        self.int_min       = 50
        self.int_max       = 200
        self.drawing       = False
        self.curr_poly     = []
        self.brush_mask    = None
        self.temp_mask     = None
        self.init_ui()

    def init_ui(self):
        # Canvas
        self.canvas     = QLabel()
        self.canvas.setFixedSize(640, 480)
        self.canvas.setStyleSheet("background: #222;")
        self.canvas.mousePressEvent   = self.start_draw
        self.canvas.mouseMoveEvent    = self.update_draw
        self.canvas.mouseReleaseEvent = self.finish_draw

        # Buttons and controls
        self.load_btn    = QPushButton("Load Image Folder")
        self.load_btn.clicked.connect(self.load_folder)
        self.prev_btn    = QPushButton("‹ Prev")
        self.prev_btn.clicked.connect(lambda: self.change_image(-1))
        self.next_btn    = QPushButton("Next ›")
        self.next_btn.clicked.connect(lambda: self.change_image(1))

        self.tool_combo  = QComboBox()
        self.tool_combo.addItems(["Brush","Lasso","Magic","Auto"])
        self.class_combo = QComboBox()
        self.class_combo.addItems(["grass","lines","road"])

        # Sliders for auto parameters
        self.gray_tol_slider = QSlider(Qt.Horizontal)
        self.gray_tol_slider.setRange(0, 100)
        self.gray_tol_slider.setValue(self.gray_tol)
        self.gray_tol_slider.valueChanged.connect(self.update_gray_tol)
        self.gray_tol_label  = QLabel(f"Gray tol: {self.gray_tol}")

        self.int_min_slider  = QSlider(Qt.Horizontal)
        self.int_min_slider.setRange(0, 255)
        self.int_min_slider.setValue(self.int_min)
        self.int_min_slider.valueChanged.connect(self.update_int_min)
        self.int_min_label   = QLabel(f"Min int: {self.int_min}")

        self.int_max_slider  = QSlider(Qt.Horizontal)
        self.int_max_slider.setRange(0, 255)
        self.int_max_slider.setValue(self.int_max)
        self.int_max_slider.valueChanged.connect(self.update_int_max)
        self.int_max_label   = QLabel(f"Max int: {self.int_max}")

        self.save_btn    = QPushButton("Save Masks")
        self.save_btn.clicked.connect(self.save_masks)

        # Undo shortcut
        self.undo_sc     = QShortcut(QKeySequence("Ctrl+Z"), self)
        self.undo_sc.activated.connect(self.undo_last_mask)

        # Layout
        nav = QHBoxLayout()
        nav.addWidget(self.load_btn)
        nav.addWidget(self.prev_btn)
        nav.addWidget(self.next_btn)
        nav.addStretch()
        nav.addWidget(QLabel("Tool:"));   nav.addWidget(self.tool_combo)
        nav.addWidget(QLabel("Class:"));  nav.addWidget(self.class_combo)
        nav.addWidget(self.gray_tol_label)
        nav.addWidget(self.gray_tol_slider)
        nav.addWidget(self.int_min_label)
        nav.addWidget(self.int_min_slider)
        nav.addWidget(self.int_max_label)
        nav.addWidget(self.int_max_slider)
        nav.addWidget(self.save_btn)

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        layout.addLayout(nav)
        self.setLayout(layout)

    def update_gray_tol(self, v):
        self.gray_tol = v
        self.gray_tol_label.setText(f"Gray tol: {v}")

    def update_int_min(self, v):
        self.int_min = v
        self.int_min_label.setText(f"Min int: {v}")

    def update_int_max(self, v):
        self.int_max = v
        self.int_max_label.setText(f"Max int: {v}")

    def load_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Image Folder")
        if not folder:
            return
        self.image_paths = sorted(glob.glob(f"{folder}/*.[jp][pn]g"))
        if not self.image_paths:
            QMessageBox.warning(self, "No Images", "No JPG/PNG found.")
            return
        self.curr_idx     = 0
        self.mask_history.clear()
        self.show_image()

    def show_image(self):
        path           = self.image_paths[self.curr_idx]
        self.curr_img  = cv2.imread(path)
        self.display   = self.curr_img.copy()
        self.temp_mask = None
        self.redraw()

    def change_image(self, delta):
        if self.curr_idx < 0:
            return
        self.curr_idx = (self.curr_idx + delta) % len(self.image_paths)
        self.show_image()

    def _update_canvas(self):
        rgb = cv2.cvtColor(self.display, cv2.COLOR_BGR2RGB)
        h, w, _ = rgb.shape
        qimg = QImage(rgb.data, w, h, 3*w, QImage.Format_RGB888)
        pix  = QPixmap.fromImage(qimg).scaled(
            self.canvas.size(), Qt.KeepAspectRatio
        )
        self.canvas.setPixmap(pix)

    def start_draw(self, ev):
        tool = self.tool_combo.currentText()
        x, y  = self._to_img(ev.pos())
        if tool == "Auto":
            mask = self.auto_road_mask()
            self.record_mask(mask)
            return
        if tool == "Magic" and ev.button() == Qt.LeftButton:
            mask = self.magic_mask(x, y)
            self.record_mask(mask)
            return
        if ev.button() != Qt.LeftButton:
            return
        self.drawing = True
        if tool == "Brush":
            h, w = self.curr_img.shape[:2]
            self.brush_mask = np.zeros((h, w), np.uint8)
            self.paint_brush(ev.pos())
        else:  # Lasso
            self.curr_poly = [ev.pos()]

    def update_draw(self, ev):
        if not self.drawing:
            return
        tool = self.tool_combo.currentText()
        if tool == "Brush":
            self.paint_brush(ev.pos())
        else:
            self.curr_poly.append(ev.pos())
            self.redraw(temp_poly=self.curr_poly)

    def finish_draw(self, ev):
        if not self.drawing:
            return
        self.drawing = False
        tool         = self.tool_combo.currentText()
        if tool == "Brush":
            self.record_mask(self.brush_mask)
        elif tool == "Lasso":
            h, w = self.curr_img.shape[:2]
            mask = np.zeros((h, w), np.uint8)
            pts  = np.array([self._to_img(p) for p in self.curr_poly], np.int32)
            cv2.fillPoly(mask, [pts], 255)
            self.record_mask(mask)

    def paint_brush(self, pos):
        x, y = self._to_img(pos)
        cv2.circle(self.brush_mask, (x, y), 10, 255, -1)
        self.temp_mask = self.brush_mask.copy()
        self.redraw(temp_mask=self.temp_mask)

    def magic_mask(self, x, y):
        hsv   = cv2.cvtColor(self.curr_img, cv2.COLOR_BGR2HSV)
        seed  = hsv[y, x].astype(int)
        tol   = np.array([10, 60, 60])
        lower = np.clip(seed - tol, 0, 255).astype(np.uint8)
        upper = np.clip(seed + tol, 0, 255).astype(np.uint8)
        base  = cv2.inRange(hsv, lower, upper)
        k     = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        base  = cv2.morphologyEx(base, cv2.MORPH_OPEN, k)
        num, labels = cv2.connectedComponents(base)
        comp_id     = labels[y, x]
        if comp_id == 0:
            return np.zeros_like(base)
        return (labels == comp_id).astype(np.uint8)*255

    def auto_road_mask(self):
        img       = self.curr_img
        b, g, r   = cv2.split(img)
        diff1     = cv2.absdiff(b, g)
        diff2     = cv2.absdiff(g, r)
        gray_mask = np.bitwise_and(diff1 < self.gray_tol,
                                   diff2 < self.gray_tol).astype(np.uint8)*255
        gray      = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        inten     = cv2.inRange(gray, self.int_min, self.int_max)
        mask      = cv2.bitwise_and(gray_mask, inten)
        kern      = cv2.getStructuringElement(cv2.MORPH_RECT, (15,15))
        mask      = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kern)
        mask      = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern)
        num, labels = cv2.connectedComponents(mask)
        if num <= 1:
            return mask
        max_area, best = 0, 1
        for i in range(1, num):
            area = np.sum(labels == i)
            if area > max_area:
                max_area, best = area, i
        return (labels == best).astype(np.uint8)*255

    def record_mask(self, new_mask):
        name = os.path.basename(self.image_paths[self.curr_idx])
        cls  = self.class_combo.currentText()
        hist = self.mask_history.setdefault(name, [])
        hist.append((new_mask.copy(), cls))
        self.temp_mask = None
        self.redraw()

    def undo_last_mask(self):
        name = os.path.basename(self.image_paths[self.curr_idx])
        hist = self.mask_history.get(name, [])
        if not hist:
            return
        hist.pop()
        if not hist:
            self.mask_history.pop(name)
        self.redraw()

    def redraw(self, temp_mask=None, temp_poly=None):
        self.display = self.curr_img.copy()
        name         = os.path.basename(self.image_paths[self.curr_idx])
        # overlay saved masks
        for mask, cls in self.mask_history.get(name, []):
            color = self.colormap.get(cls, (255,255,0))
            self.display[mask > 0] = color
        # overlay temp mask
        if temp_mask is not None:
            self.display[temp_mask > 0] = (0,0,255)
        # lasso outline
        if temp_poly and len(temp_poly) > 1:
            pts = np.array([self._to_img(p) for p in temp_poly], np.int32)
            cv2.polylines(self.display, [pts], False, (255,0,0), 1)
        self._update_canvas()

    def save_masks(self):
        if not self.mask_history:
            QMessageBox.information(self, "No Data", "Nothing to save.")
            return
        folder = os.path.dirname(self.image_paths[0])
        mdir   = os.path.join(folder, "masks")
        os.makedirs(mdir, exist_ok=True)
        for name, hist in self.mask_history.items():
            agg = np.zeros_like(hist[0][0])
            for m, _ in hist:
                agg = cv2.bitwise_or(agg, m)
            out = os.path.join(mdir, name.rsplit(".",1)[0] + ".png")
            cv2.imwrite(out, agg)
        QMessageBox.information(self, "Saved", f"Masks saved to:\n{mdir}")

    def _to_img(self, pt):
        ih, iw = self.curr_img.shape[:2]
        cw, ch = self.canvas.width(), self.canvas.height()
        s       = min(cw/iw, ch/ih)
        w0, h0  = int(iw*s), int(ih*s)
        mx, my  = (cw-w0)//2, (ch-h0)//2
        xw, yw  = pt.x()-mx, pt.y()-my
        xi, yi  = int(xw/s), int(yw/s)
        return max(0, min(iw-1, xi)), max(0, min(ih-1, yi))

class UploadTab(QWidget):
    def __init__(self):
        super().__init__()
        self.folder = None
        self.init_ui()

    def init_ui(self):
        self.sel_btn = QPushButton("Select Dataset Folder")
        self.sel_btn.clicked.connect(self.select_folder)
        self.up_btn  = QPushButton("Upload to Edge Impulse")
        self.up_btn.clicked.connect(self.upload)
        layout = QVBoxLayout()
        layout.addWidget(self.sel_btn)
        layout.addWidget(self.up_btn)
        layout.addStretch()
        self.setLayout(layout)

    def select_folder(self):
        f = QFileDialog.getExistingDirectory(self, "Select Dataset Folder")
        if f:
            self.folder = f
            QMessageBox.information(self, "Folder Selected", f)

    def upload(self):
        if not self.folder:
            QMessageBox.warning(self, "No Folder", "Select a folder first.")
            return
        proc = subprocess.run(
            ["edge-impulse-uploader", "--directory", self.folder],
            capture_output=True, text=True
        )
        if proc.returncode == 0:
            QMessageBox.information(self, "Uploaded", "Dataset uploaded.")
        else:
            QMessageBox.critical(self, "Failed", proc.stderr or "Unknown error")

class ModelTab(QWidget):
    def __init__(self):
        super().__init__()
        self.onnx = None
        self.init_ui()

    def init_ui(self):
        self.open_btn = QPushButton("Open Edge Impulse Studio")
        self.open_btn.clicked.connect(lambda:
            webbrowser.open(
                "https://studio.edgeimpulse.com/project/YOUR_PROJECT_ID/impulse"
            )
        )
        self.sel_btn  = QPushButton("Select ONNX Model")
        self.sel_btn.clicked.connect(self.select_onnx)
        self.conv_btn = QPushButton("Convert ONNX → .blob")
        self.conv_btn.clicked.connect(self.convert_blob)
        layout = QVBoxLayout()
        layout.addWidget(self.open_btn)
        layout.addWidget(self.sel_btn)
        layout.addWidget(self.conv_btn)
        layout.addStretch()
        self.setLayout(layout)

    def select_onnx(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Select ONNX Model", "", "ONNX Files (*.onnx)"
        )
        if path:
            self.onnx = path
            QMessageBox.information(self, "Selected Model", path)

    def convert_blob(self):
        if not self.onnx:
            QMessageBox.warning(self, "No Model", "Select an ONNX model first.")
            return
        url = "https://blobconverter.luxonis.com/convert"
        with open(self.onnx, "rb") as f:
            resp = requests.post(url, files={"file": f}, data={"ai": "myriad"})
        if resp.status_code == 200:
            out = os.path.splitext(self.onnx)[0] + ".blob"
            with open(out, "wb") as o:
                o.write(resp.content)
            QMessageBox.information(self, "Done", f"Saved blob to:\n{out}")
        else:
            QMessageBox.critical(self, "Error", f"{resp.status_code}: {resp.text}")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OAK-D LR Trainer GUI")
        tabs = QTabWidget()
        tabs.addTab(AnnotationTab(), "1 • Annotate")
        tabs.addTab(UploadTab(),     "2 • Upload")
        tabs.addTab(ModelTab(),      "3 • Model→Blob")
        self.setCentralWidget(tabs)
        self.resize(820, 700)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
