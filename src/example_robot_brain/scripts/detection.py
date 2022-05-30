#!/usr/bin/env python3

import numpy as np

DETECTIONS = [
    ["weed", np.array([1.055, 2.011])],
    ["litter", np.array([2.357, 1.049])],
    ["litter", np.array([2.497, 1.972])],
    ["weed", np.array([2.217, 2.032])],
]
i = 0

# Does not actually detect anything, just returns hardcoded locations
def detect():
    global i

    i += 1

    # Just return one predefined weed position, we expect you to implement a better
    # solution.
    return DETECTIONS[(i - 1) % len(DETECTIONS)]
