"""Experimental image-space edge candidates; requires verification on real weld joints.

No metric calibration or weld classification is provided.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class Settings:
    processing_width: int = 640
    roi: tuple = (.2, .2, .8, .95)
    canny_low: int = 50
    canny_high: int = 150
    min_width: int = 8
    max_width: int = 140
    max_angle: float = 40.

    def validate(self):
        if not 160 <= self.processing_width <= 1920:
            raise ValueError('processing_width must be 160..1920')
        if len(self.roi) != 4 or not all(math.isfinite(v) for v in self.roi):
            raise ValueError('roi must contain four finite fractions')
        x0, y0, x1, y1 = self.roi
        if not (0 <= x0 < x1 <= 1 and 0 <= y0 < y1 <= 1):
            raise ValueError('roi must satisfy 0 <= x0 < x1 <= 1 and 0 <= y0 < y1 <= 1')
        if not 0 < self.canny_low < self.canny_high <= 255:
            raise ValueError('Canny thresholds must satisfy 0 < low < high <= 255')
        if not 2 <= self.min_width < self.max_width <= self.processing_width:
            raise ValueError('width bounds must satisfy 2 <= min < max <= processing_width')
        if not 0 < self.max_angle < 80:
            raise ValueError('max_angle must be between 0 and 80 degrees from image vertical')


@dataclass
class Result:
    image: np.ndarray
    edges: np.ndarray
    # Endpoints in the resized full-image coordinate system; not robot coordinates.
    left: np.ndarray | None = None
    right: np.ndarray | None = None
    support: float = 0.


def detect(bgr, settings):
    settings.validate()
    if bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
        raise ValueError('Expected an 8-bit BGR image')
    height = max(1, round(bgr.shape[0] * settings.processing_width / bgr.shape[1]))
    image = cv2.resize(bgr, (settings.processing_width, height), interpolation=cv2.INTER_AREA)
    x0, y0, x1, y1 = [round(v * size) for v, size in
                       zip(settings.roi, (image.shape[1], height, image.shape[1], height))]
    crop = image[y0:y1, x0:x1]
    if min(crop.shape[:2]) < 16:
        raise ValueError('ROI must cover at least 16 pixels in each dimension')
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0),
                      settings.canny_low, settings.canny_high)
    full_edges = np.zeros(image.shape[:2], dtype=np.uint8)
    full_edges[y0:y1, x0:x1] = edges
    result = Result(image, full_edges)
    h, w = edges.shape
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=max(15, h//8),
                            minLineLength=max(12, h//3), maxLineGap=max(5, h//12))
    candidates = []
    if lines is not None:
        for xa, ya, xb, yb in lines[:, 0]:
            if ya == yb:
                continue
            slope = (xb-xa) / (yb-ya)
            if abs(slope) > math.tan(math.radians(settings.max_angle)):
                continue
            intercept = xa-slope*ya
            candidates.append((slope, intercept, min(ya, yb), max(ya, yb)))
    # Bound pair search on textured surfaces, prioritizing long observations.
    candidates.sort(key=lambda line: line[3]-line[2], reverse=True)
    candidates = candidates[:40]
    best = 0.
    for i, first in enumerate(candidates):
        for second in candidates[i+1:]:
            if abs(first[0]-second[0]) > .25:
                continue
            top, bottom = max(first[2], second[2]), min(first[3], second[3])
            if bottom-top < .45*h:
                continue
            ys = np.arange(top, bottom+1)
            xs1, xs2 = first[0]*ys+first[1], second[0]*ys+second[1]
            if xs1[len(ys)//2] > xs2[len(ys)//2]:
                xs1, xs2 = xs2, xs1
            widths = xs2-xs1
            if widths.min() < settings.min_width or widths.max() > settings.max_width:
                continue
            if min(xs1.min(), xs2.min()) < 0 or max(xs1.max(), xs2.max()) >= w:
                continue
            supports = []
            for xs in (xs1, xs2):
                ix = np.rint(xs).astype(int)
                supports.append(np.mean(np.any(np.stack([
                    edges[ys, np.clip(ix+d, 0, w-1)] > 0 for d in (-2, -1, 0, 1, 2)
                ]), axis=0)))
            support = min(supports)
            if support < .65:
                continue
            score = support*(bottom-top)/h
            if score > best:
                best = score
                result.left = np.array([[xs1[0]+x0, top+y0], [xs1[-1]+x0, bottom+y0]])
                result.right = np.array([[xs2[0]+x0, top+y0], [xs2[-1]+x0, bottom+y0]])
                result.support = float(support)
    cv2.rectangle(image, (x0, y0), (x1-1, y1-1), (170, 170, 170), 1)
    label = 'NO EDGE PAIR'
    if result.left is not None:
        center = (result.left+result.right)/2
        for points, color in [(result.left, (0, 255, 0)), (result.right, (0, 255, 255)),
                              (center, (255, 0, 255))]:
            a, b = np.rint(points).astype(int)
            cv2.line(image, tuple(a), tuple(b), color, 2, cv2.LINE_AA)
        width = np.mean(result.right[:, 0]-result.left[:, 0])
        label = f'CANDIDATE | horizontal width {width:.1f}px | edge support {result.support:.0%}'
    cv2.putText(image, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
    return result
