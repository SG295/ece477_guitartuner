from __future__ import print_function
from __future__ import division

import numpy as np

# Example code, computes the coefficients of a low-pass windowed-sinc filter.

# Configuration.
fS = 16000  # Sampling rate.
fL = 1120  # Cutoff frequency.
N = 461  # Filter length, must be odd.

# Compute sinc filter.
h = np.sinc(2 * fL / fS * (np.arange(N) - (N - 1) / 2))

# Apply window.
h *= np.blackman(N)

# Normalize to get unity gain.
h /= np.sum(h)

# Print the filter coefficients in C array format.
print("float h[] = {")
print(",".join("    {:.8f}".format(coeff) for coeff in h))
print("};")

# Applying the filter to a signal s can be as simple as writing
# s = np.convolve(s, h)