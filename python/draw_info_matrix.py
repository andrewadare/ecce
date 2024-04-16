#!/usr/bin/env python

import numpy as np
import cv2


def main():
    pass


if __name__ == "__main__":
    # with open("info-matrix.txt") as f:

    A = np.loadtxt("info-matrix.txt")

    im = cv2.cvtColor(255 * A.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    h, w, _ = im.shape
    im = cv2.resize(im, dsize=(8 * w, 8 * h), interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(str("info-matrix.png"), im)

    print(A.shape)

    # main()
