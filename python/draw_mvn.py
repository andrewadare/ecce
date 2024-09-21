#!/usr/bin/env python

import numpy as np
import corner
import matplotlib.pyplot as plt

from g2o_interop import read_g2o_se3_quat

from scipy.linalg import cholesky


def sample_mvn(mu: np.ndarray, cov: np.ndarray, n: int):
    """Return n samples from a multivariate normal distribution with mean vector mu
    and covariance matrix cov.

    Parameters
    ----------
    mu : ndarray - shape (m,)
        Mean vector
    cov : ndarray - shape (m, m)
        Covariance matrix
    n : int
        Number of samples to draw

    Returns
    -------
    ndarray - shape (n, m)
    """
    A = cholesky(cov)  # Find an m x m matrix A such that A @ A.T = sigma
    z = np.random.randn(len(mu), n)
    samples = mu[:, np.newaxis] + A @ z
    return samples.T


if __name__ == "__main__":
    np.set_printoptions(precision=3, suppress=True)

    # Onboard camera pose
    id, pose = read_g2o_se3_quat(
        "VERTEX_SE3:QUAT 0 0.108204 -1.76393 1.87108 -0.119958 -0.0139812 0.00178393 0.992679"
    )

    print(pose.vec)

    cov = np.loadtxt("c0_cov.txt")

    mu = pose.vec
    labels = ["x", "y", "z", "r1", "r2", "r3"]

    print(cov)

    samples = sample_mvn(np.zeros(6), cov, n=100000)

    figure = corner.corner(samples)
    figure.savefig("onboard-camera-mvn.png")
    plt.close()
