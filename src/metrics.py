"""Evaluation metrics for a control run.

These operate on the logged arrays, so the identical functions apply to the
baseline and to every later controller -- one definition of "error" for all.
"""
import numpy as np


def rms_error(actual, desired):
    """Root-mean-square error of a logged signal against its setpoint.

    actual  : (T, n) array -- logged values over T timesteps for n joints
    desired : (n,) setpoint, broadcast across all timesteps
    Returns : (n,) array -- one RMS value per joint
    """
    actual = np.asarray(actual)
    error = actual - np.asarray(desired)
    return np.sqrt(np.mean(error ** 2, axis=0))
