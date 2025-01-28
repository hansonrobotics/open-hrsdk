# -*- coding: utf-8 -*-
#
# Copyright (C) 2017-2024 Hanson Robotics
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
#
# Portions of this file incorporate code by endolith (MIT License).
# Refer to the NOTICES file for the full MIT license text.
#


import math
import struct

import numpy as np
from numpy import arange, argmax, copy, diff, log, mean
from numpy.fft import rfft
from scipy.signal import decimate, fftconvolve, kaiser


def parabolic(f, x):
    """
    Quadratic interpolation for estimating the true position of an
    inter-sample maximum when nearby samples are known.

    f is a vector and x is an index for that vector.

    Returns (vx, vy), the coordinates of the vertex of a parabola that goes
    through point x and its two neighbors.

    Example:
    Defining a vector f with a local maximum at index 3 (= 6), find local
    maximum if points 2, 3, and 4 actually defined a parabola.

    In [3]: f = [2, 3, 1, 6, 4, 2, 3, 1]

    In [4]: parabolic(f, argmax(f))
    Out[4]: (3.2142857142857144, 6.1607142857142856)
    """
    xv = 1 / 2.0 * (f[x - 1] - f[x + 1]) / (f[x - 1] - 2 * f[x] + f[x + 1]) + x
    yv = f[x] - 1 / 4.0 * (f[x - 1] - f[x + 1]) * (xv - x)
    return (xv, yv)


def freq_from_fft(signal, fs):
    """Estimate frequency from peak of FFT

    Pros: Accurate, usually even more so than zero crossing counter
    (1000.000004 Hz for 1000 Hz, for instance).  Due to parabolic
    interpolation being a very good fit for windowed log FFT peaks?
    https://ccrma.stanford.edu/~jos/sasp/Quadratic_Interpolation_Spectral_Peaks.html
    Accuracy also increases with signal length

    Cons: Doesn't find the right value if harmonics are stronger than
    fundamental, which is common.

    """
    N = len(signal)

    # Compute Fourier transform of windowed signal
    windowed = signal * kaiser(N, 100)
    f = rfft(windowed)
    # Find the peak and interpolate to get a more accurate peak
    i_peak = argmax(abs(f))  # Just use this value for less-accurate result
    i_interp = parabolic(log(abs(f)), i_peak)[0]

    # Convert to equivalent frequency
    return fs * i_interp / N  # Hz


def freq_from_hps(signal, fs):
    """Estimate frequency using harmonic product spectrum

    Low frequency noise piles up and overwhelms the desired peaks
    """
    N = len(signal)
    signal -= mean(signal)  # Remove DC offset

    # Compute Fourier transform of windowed signal
    windowed = signal * kaiser(N, 100)

    # Get spectrum
    X = log(abs(rfft(windowed)))

    # Downsample sum logs of spectra instead of multiplying
    hps = copy(X)
    for h in arange(2, 9):  # TODO: choose a smarter upper limit
        dec = decimate(X, h)
        hps[: len(dec)] += dec

    # Find the peak and interpolate to get a more accurate peak
    i_peak = argmax(hps[: len(dec)])
    i_interp = parabolic(hps, i_peak)[0]

    # Convert to equivalent frequency
    return fs * i_interp / N  # Hz


def get_decibel(signal):
    """The Energy of Sound per chunk is calculated"""
    rms = np.sqrt(np.mean(np.absolute(signal) ** 2))
    if rms > 1:
        p = 20 * math.log10(rms)
    else:
        p = 0
    return p


def bytes2numpy(bytes):
    # converts stream (which is in byte format) to list of +ve and -ve integers
    count = len(bytes) / 2
    format = "%dh" % (count)
    data = np.asarray(struct.unpack(format, bytes))
    return data
