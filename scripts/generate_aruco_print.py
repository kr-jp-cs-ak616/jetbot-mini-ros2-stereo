#!/usr/bin/env python3
"""Generate actual-size ArUco markers on A4 using OpenCV and ReportLab."""

import argparse
from pathlib import Path

import cv2
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen.canvas import Canvas


DEFAULT_MARKERS = [(22, 100), (23, 150)]
DICTIONARY = 'DICT_4X4_50'


def marker_spec(value):
    try:
        identifier, size = map(int, value.split(':'))
    except ValueError as exc:
        raise argparse.ArgumentTypeError('Use ID:SIZE_MM, for example 26:120.') from exc
    if not 0 <= identifier <= 49 or not 25 <= size <= 180:
        raise argparse.ArgumentTypeError('ID must be 0-49 and size must be 25-180 mm.')
    return identifier, size


def label(pdf, y, message, size=10, bold=False):
    pdf.setFont('Helvetica-Bold' if bold else 'Helvetica', size)
    pdf.drawCentredString(105 * mm, y * mm, message)


def scale_bar(pdf, y):
    pdf.setLineWidth(.6)
    pdf.line(55 * mm, y * mm, 155 * mm, y * mm)
    for x in (55, 155):
        pdf.line(x * mm, (y - 1.5) * mm, x * mm, (y + 1.5) * mm)
    label(pdf, y + 3, '100 mm print check', 8)


def draw_marker(pdf, pattern, size, x, y):
    cell = size / pattern.shape[0]
    for row, values in enumerate(pattern):
        for column, value in enumerate(values):
            if value == 0:
                pdf.rect((x + column * cell) * mm, (y + (len(pattern) - row - 1) * cell) * mm,
                         cell * mm, cell * mm, stroke=0, fill=1)


def marker_page(pdf, pattern, identifier, size):
    label(pdf, 273, f'ArUco / {size} x {size} mm', 20, True)
    label(pdf, 260, f'{DICTIONARY}  |  ID {identifier}', 11)
    label(pdf, 248, 'A4 portrait  /  Actual size (100%)')
    draw_marker(pdf, pattern, size, (210 - size) / 2, 154 - size / 2)
    scale_bar(pdf, 39)
    label(pdf, 25, 'Measure the outer black square. Keep a clean white border.', 9)
    pdf.showPage()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('markers', nargs='*', type=marker_spec, metavar='ID:SIZE_MM',
                        help='Defaults: ID 22 at 100 mm and ID 23 at 150 mm.')
    parser.add_argument('--output', type=Path, default=Path('assets/aruco-100mm-150mm-a4.pdf'))
    args = parser.parse_args()
    specs = args.markers or DEFAULT_MARKERS
    if len({identifier for identifier, _ in specs}) != len(specs):
        parser.error('Use distinct IDs when putting multiple markers in one print set.')

    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    generate = aruco.generateImageMarker if hasattr(aruco, 'generateImageMarker') else aruco.drawMarker
    args.output.parent.mkdir(parents=True, exist_ok=True)
    pdf = Canvas(str(args.output), pagesize=A4, pageCompression=1, invariant=1)
    pdf.setTitle('ArUco DICT_4X4_50 | A4 Actual-Size Markers')
    for identifier, size in specs:
        marker_page(pdf, generate(dictionary, identifier, 6), identifier, size)
    pdf.save()
    print(args.output)


if __name__ == '__main__':
    main()
