#!/usr/bin/env python3
"""Generate the exact A4 print target used by handeye_math.py."""
from __future__ import annotations
import argparse
from pathlib import Path
import cv2
import numpy as np
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.units import mm
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_CENTER
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Image, Flowable, KeepTogether
)
from reportlab.lib import colors


MARKER_DICTIONARY_NAME = "DICT_6X6_50"
MARKER_ID = 0
MARKER_SIZE_MM = 75.0


class ScaleBar(Flowable):
    def __init__(self, length_mm=100.0):
        super().__init__()
        self.length_mm = float(length_mm)
        self.width = self.length_mm * mm
        self.height = 12 * mm
        self.hAlign = "CENTER"

    def draw(self):
        c = self.canv
        y = 7 * mm
        c.setLineWidth(0.7)
        c.setStrokeColor(colors.black)
        c.line(0, y, self.width, y)
        c.line(0, y - 2 * mm, 0, y + 2 * mm)
        c.line(self.width, y - 2 * mm, self.width, y + 2 * mm)
        c.setFont("Helvetica", 8)
        c.drawCentredString(self.width / 2, 1.5 * mm,
                            f"Scale check: {self.length_mm:.0f} mm between ticks")


def generate(output_pdf: Path, output_png: Path):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("This OpenCV build lacks cv2.aruco; use an OpenCV-contrib build.")
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, MARKER_DICTIONARY_NAME))
    pixels = 1500  # 20 px/mm, crisp for a 75 mm marker (~508 dpi).
    if hasattr(aruco, "generateImageMarker"):
        marker = aruco.generateImageMarker(dictionary, MARKER_ID, pixels, borderBits=1)
    else:
        marker = aruco.drawMarker(dictionary, MARKER_ID, pixels, borderBits=1)
    output_png.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_png), marker):
        raise RuntimeError(f"failed to write marker PNG: {output_png}")

    styles = getSampleStyleSheet()
    title_style = ParagraphStyle(
        "TargetTitle", parent=styles["Title"], fontName="Helvetica-Bold",
        fontSize=12, leading=15, alignment=TA_CENTER, spaceAfter=0
    )
    body_style = ParagraphStyle(
        "TargetBody", parent=styles["Normal"], fontName="Helvetica",
        fontSize=8, leading=10, alignment=TA_CENTER, spaceAfter=0
    )
    doc = SimpleDocTemplate(
        str(output_pdf), pagesize=landscape(A4),
        rightMargin=10*mm, leftMargin=10*mm,
        topMargin=8*mm, bottomMargin=8*mm,
        title="A4 ArUco 6x6_50 ID 0 - 75 mm",
        author="FR3 D435i Hand-Eye Calibration"
    )
    marker_img = Image(str(output_png), width=MARKER_SIZE_MM*mm,
                       height=MARKER_SIZE_MM*mm)
    marker_img.hAlign = "CENTER"
    story = [
        Paragraph("FR3 + D435i Hand-Eye Calibration Target", title_style),
        Spacer(1, 5*mm),
        marker_img,
        Spacer(1, 4*mm),
        Paragraph(
            f"ArUco dictionary: {MARKER_DICTIONARY_NAME} | ID: {MARKER_ID} | "
            f"outer black-border edge: {MARKER_SIZE_MM:.1f} mm",
            body_style
        ),
        Spacer(1, 7*mm),
        ScaleBar(100.0),
        Spacer(1, 2*mm),
        Paragraph(
            "Print A4 landscape at 100% / Actual Size. Disable Fit to Page. "
            "After printing, verify the scale bar measures exactly 100 mm.",
            body_style
        ),
    ]
    doc.build(story)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pdf", type=Path, default=Path("A4_ArUco_6x6_50_ID0_75mm.pdf"))
    parser.add_argument("--png", type=Path, default=Path("aruco_6x6_50_id0_75mm.png"))
    args = parser.parse_args()
    generate(args.pdf, args.png)
    print(f"[PASS] PDF: {args.pdf.resolve()}")
    print(f"[PASS] PNG: {args.png.resolve()}")
    print(f"[TARGET] {MARKER_DICTIONARY_NAME}, ID={MARKER_ID}, size={MARKER_SIZE_MM:.1f} mm outer edge")


if __name__ == "__main__":
    main()
