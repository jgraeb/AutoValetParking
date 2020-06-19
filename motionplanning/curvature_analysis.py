# Tung Phan
# This is an implementation of Curvature-Bounded Traversability
# Analysis for Motion Planning of Mobile Robots
# https://users.wpi.edu/~rvcowlagi/publications/2014-tro-cbta.pdf

import matplotlib.pyplot as plt
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

def get_Lambda_prime_x(spec):
    new_spec = spec.get_altered_spec_for_lambda_prime_x_s1()
    Xs, Us, _ = get_Upsilon_prime_x(new_spec)
    Ls = [-u for u in Us]
    return Xs, Ls

def get_Upsilon_prime_x(spec):
    d, y, z, w, r, beta_lo, beta_hi = spec.extract_params()
    UPS = []
    gammas = []
    Xs = []


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
                max_val = np.min(Upsilon_prime_x / np.pi * 180)
                if not np.iscomplex(max_val):
                    UPS.append(max_val)
                    Xs.append(x)
                    break
            elif beta_lo <= gamma_star_x and gamma_star_x < beta_hi:
                A = -(x-w)
                B = d
                C = sigma3
                Upsilon_prime_x = np.array(solve_Acos_Bsin_C(A, B, C))
                max_val = np.min(Upsilon_prime_x / np.pi * 180)
                if not np.iscomplex(max_val):
                    UPS.append(max_val)
                    gammas.append(gamma_star_x / np.pi * 180)
                    Xs.append(x)
                    break
            else:
                print('does not exist')
    return Xs, UPS, gammas

class Spec:
    def __init__(self, d, y, z, w, r, beta_lo, beta_hi):
        self.d = d
        self.y = y
        self.z = z
        self.w = w
        self.r = r
        self.beta_lo = beta_lo
        self.beta_hi = beta_hi
    def extract_params(self):
        return self.d, self.y, self.z, self.w, self.r, self.beta_lo, self.beta_hi

    def get_altered_spec_for_lambda_prime_x_s1(self):
        w = self.d -self.w
        beta_lo = -self.beta_hi * (self.d - self.x)
        beta_hi = -self.beta_lo * (self.d - self.x)
        return Spec(d=self.d, y=self.y, z=self.z, w=w, r=self.r,
                beta_lo=beta_lo, beta_hi=beta_hi)

if __name__ == '__main__':
    # Example 1
    d = 10 # grid size
    y = 0 #
    z = 5
    w = 1.25
    r = 45 # maximum radius of curvature (assuming larger than box for now)
    beta_lo = -40/180*np.pi
    beta_hi = 10/180*np.pi
    spec = Spec(d=d,y=y,z=z,w=w,r=r,beta_lo=beta_lo,beta_hi=beta_hi)
    Xs, UPS, gammas = get_Upsilon_prime_x(spec)
    Xs, LS = get_Lambda_prime_x(spec)
    plt.plot(Xs, gammas, 'b')
    plt.plot(Xs, UPS, 'k')
    plt.plot(Xs, Ls, 'r')
    plt.show()


