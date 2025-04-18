from __future__ import print_function
from __future__ import division

import numpy as np

# Example code, computes the coefficients of a low-pass windowed-sinc filter.

# Configuration.
fS = 32000  # Sampling rate.
fL = 600  # Cutoff frequency.
N = 247  # Filter length, must be odd.

# Compute sinc filter.
h = np.sinc(2 * fL / fS * (np.arange(N) - (N - 1) / 2))

# Apply window.
h *= np.blackman(N)

# Normalize to get unity gain.
h /= np.sum(h)

# print(h)

print("float h[] = {")
print(",".join(f"    {coef:.10f}f" for coef in h))
print("};")
# Applying the filter to a signal s can be as simple as writing
# s = np.convolve(s, h)
