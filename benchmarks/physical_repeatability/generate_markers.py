#!/usr/bin/env python3
"""Generate a vector A4 ArUco sheet; requires OpenCV with cv2.aruco."""
import argparse
from pathlib import Path

import cv2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dictionary', default='DICT_4X4_50')
    parser.add_argument('--ids', type=int, nargs=9, default=list(range(9)))
    parser.add_argument('--fixed-size-mm', type=float, default=40)
    parser.add_argument('--gripper-size-mm', type=float, default=30)
    parser.add_argument('--output', type=Path, default=Path('aruco_9_A4.pdf'))
    args = parser.parse_args()
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dictionary))
    if len(set(args.ids)) != 9 or any(i < 0 or i >= len(dictionary.bytesList) for i in args.ids):
        parser.error('Choose nine different IDs within the dictionary.')
    if not all(10 <= size <= 45 for size in (args.fixed_size_mm, args.gripper_size_mm)):
        parser.error('Marker sides must be between 10 and 45 mm for this layout.')
    pt = 72 / 25.4
    commands = []

    def label(x, y, text, size=10):
        escaped = text.replace('\\', '\\\\').replace('(', '\\(').replace(')', '\\)')
        commands.append(f'BT /F1 {size} Tf {x*pt:.5f} {(297-y)*pt:.5f} Td ({escaped}) Tj ET')

    def line(x1, y1, x2, y2):
        commands.append(f'{x1*pt:.5f} {(297-y1)*pt:.5f} m {x2*pt:.5f} {(297-y2)*pt:.5f} l S')

    label(15, 16, 'DeepSight-Nebula | Physical repeatability markers', 13)
    label(15, 24, f'{args.dictionary} | 1-bit black border | Print A4 at 100%', 10)
    label(15, 31, 'Size = outer black square. Keep at least 5 mm of white on every side.', 9)
    for index, marker_id in enumerate(args.ids):
        column, row = index % 3, index // 3
        side = args.gripper_size_mm if index == 8 else args.fixed_size_mm
        center_x, top = 45 + column * 60, 46 + row * 66
        x, y = center_x - side / 2, top + (40-side) / 2
        cells = dictionary.markerSize + 2
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, cells, borderBits=1)
        # Filled vector cells preserve exact physical dimensions at any resolution.
        for r in range(cells):
            for c in range(cells):
                if marker[r, c] == 0:
                    cell = side / cells
                    commands.append(f'{(x+c*cell)*pt:.5f} {(297-y-(r+1)*cell)*pt:.5f} '
                                    f'{cell*pt:.5f} {cell*pt:.5f} re f')
        role = 'gripper (suggested)' if index == 8 else 'fixed reference'
        label(center_x - 25, top + 48, f'ID {marker_id} | {side:g} x {side:g} mm', 10)
        label(center_x - 25, top + 54, role, 9)

    commands.append('0.5 w')
    line(20, 263, 120, 263)
    for i in range(11):
        x = 20 + i * 10
        line(x, 263, x, 260 if i % 5 else 258)
    label(20, 270, 'Control: this line must measure exactly 100 mm.', 10)
    label(15, 280, 'Mount flat on rigid matte supports. Do not cut through the white margin.', 9)
    label(15, 286, 'IDs and roles are provisional; no final target positions are defined.', 9)
    stream = '\n'.join(commands).encode('ascii')
    objects = [
        b'<< /Type /Catalog /Pages 2 0 R >>',
        b'<< /Type /Pages /Kids [3 0 R] /Count 1 >>',
        (f'<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {210*pt:.6f} {297*pt:.6f}] '
         '/Resources << /Font << /F1 4 0 R >> >> /Contents 5 0 R >>').encode(),
        b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>',
        f'<< /Length {len(stream)} >>\nstream\n'.encode() + stream + b'\nendstream',
    ]
    pdf = bytearray(b'%PDF-1.4\n')
    offsets = [0]
    for number, obj in enumerate(objects, 1):
        offsets.append(len(pdf))
        pdf.extend(f'{number} 0 obj\n'.encode() + obj + b'\nendobj\n')
    xref = len(pdf)
    pdf.extend(f'xref\n0 {len(objects)+1}\n0000000000 65535 f \n'.encode())
    for offset in offsets[1:]:
        pdf.extend(f'{offset:010d} 00000 n \n'.encode())
    pdf.extend(f'trailer\n<< /Size {len(objects)+1} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n'.encode())
    args.output.write_bytes(pdf)
    print(args.output)


if __name__ == '__main__':
    main()
