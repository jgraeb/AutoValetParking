# Tung Phan
# This is an implementation of Curvature-Bounded Traversability
# Analysis for Motion Planning of Mobile Robots
# https://users.wpi.edu/~rvcowlagi/publications/2014-tro-cbta.pdf

from ipdb import set_trace as st
import numpy as np

def solve_Acos_Bsin_C(A, B, C):
    # solve A cos(x) + B sin(x) = C, assuming cos(x) >= 0
    # compute coefficients of quadratic equations
    # a sinx^2 + b sinx + c = 0
    a = A**2 + B**2
    b = -2*B*C
    c = C**2 - A**2
    return [np.arcsin(sol) for sol in np.roots([a, b, c])]

if __name__ == '__main__':
    # Example 1
    d = 10 # grid size
    y = 0 #
    z = 5
    w = 5
    r = 45 # maximum radius of curvature (assuming larger than box for now)
    beta_lo = -40/180*np.pi
    beta_hi = 10/180*np.pi
    a_star_w = np.arccos(1 - (d-w)/r)

    if 2*r*(d-w) < d**2 + (d-w)**2:
        n2 = w + np.sqrt(r**2 - (r*np.sin(a_star_w)-d)**2) - r * np.cos(a_star_w)
    else:
        n2 = d

    for x in np.linspace(y, min(n2, z)):
        sigma3 = (d**2 + (x-w)**2)/(2*r)
        A = (x-w)
        B = -d
        C = sigma3
        gamma_star_x_arr = solve_Acos_Bsin_C(A, B, C)
        for gamma_star_x in gamma_star_x_arr:
            if gamma_star_x < beta_lo:
                sigma1 = r*np.cos(beta_lo) + (x - w)
                sigma2 = r*np.sin(beta_lo) - d
                A = sigma1
                B = sigma2
                C = 1-(x-w)*np.cos(beta_lo)+d*np.sin(beta_lo)
                Upsilon_prime_x = np.array(solve_Acos_Bsin_C(A, B, C))
                print(Upsilon_prime_x / np.pi * 180)
            elif beta_lo <= gamma_star_x and gamma_star_x < beta_hi:
                A = -(x-w)
                B = d
                C = sigma3
                Upsilon_prime_x = np.array(solve_Acos_Bsin_C(A, B, C))
                print(Upsilon_prime_x / np.pi * 180)
            else:
                print('does not exist')

