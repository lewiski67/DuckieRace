#!/usr/bin/env python3
"""
ColorDetector
=============
HSV-based color mask detector for gym-duckietown simulation.
Supports 'red' and 'yellow' line detection.

Usage:
    from color_detector import ColorDetector
    detector = ColorDetector()
    mask = detector.get_mask(bgr_image, 'red')   # or 'yellow'
"""

import cv2
import numpy as np


class ColorDetector:
    """
    Detects colored lines in BGR images using HSV thresholding.
    Tuned for gym-duckietown's udem1 map colors.
    """

    # HSV ranges: [H_min, S_min, V_min], [H_max, S_max, V_max]
    # OpenCV HSV: H=0-179, S=0-255, V=0-255
    COLOR_RANGES = {
        'red': [
            # Red wraps around 0/180 in HSV, so two ranges needed
            (np.array([0,   120, 80],  dtype=np.uint8),
             np.array([8,   255, 255], dtype=np.uint8)),
            (np.array([170, 120, 80],  dtype=np.uint8),
             np.array([179, 255, 255], dtype=np.uint8)),
        ],
        'yellow': [
            (np.array([18, 100, 100], dtype=np.uint8),
             np.array([35, 255, 255], dtype=np.uint8)),
        ],
        'white': [
            (np.array([0,   0,   180], dtype=np.uint8),
             np.array([179, 40,  255], dtype=np.uint8)),
        ],
    }

    def __init__(self, kernel_size: int = 5):
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

    def get_mask(self, bgr: np.ndarray, color: str) -> np.ndarray:
        """
        Return a binary mask (uint8, 0 or 255) for the given color.

        Parameters
        ----------
        bgr   : np.ndarray  H×W×3 BGR image (as from cv_bridge or cv2.imread)
        color : str         'red', 'yellow', or 'white'

        Returns
        -------
        mask  : np.ndarray  H×W uint8 binary mask
        """
        if color not in self.COLOR_RANGES:
            raise ValueError(
                f"Unknown color '{color}'. "
                f"Supported: {list(self.COLOR_RANGES.keys())}")

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for lo, hi in self.COLOR_RANGES[color]:
            mask |= cv2.inRange(hsv, lo, hi)

        # Clean up noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        return mask


# ---------------------------------------------------------------------------
# Quick visual test: python3 color_detector.py <image.png>
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    img_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/duck_obs.png"
    bgr = cv2.imread(img_path)
    if bgr is None:
        print(f"Cannot read image: {img_path}")
        sys.exit(1)

    detector = ColorDetector()
    for color in ("red", "yellow", "white"):
        mask = detector.get_mask(bgr, color)
        pixels = np.sum(mask > 0)
        print(f"{color:8s}: {pixels} pixels detected")
        cv2.imwrite(f"/tmp/mask_{color}.png", mask)
        print(f"           saved to /tmp/mask_{color}.png")