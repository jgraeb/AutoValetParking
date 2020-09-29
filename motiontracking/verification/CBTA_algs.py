# Tung Phan, Josefine Graebener
# California Institute of Technology
# This is an implementation of Curvature-Bounded Traversability
# Analysis for Motion Planning of Mobile Robots by R. V. Cowlagi and P. Tsiotras
# https://users.wpi.edu/~rvcowlagi/publications/2014-tro-cbta.pdf

import matplotlib.pyplot as plt
from ipdb import set_trace as st
import numpy as np
from scipy.optimize import fsolve

def solve_Acos_Bsin_C(A, B, C):
    # solve A cos(x) + B sin(x) = C, assuming cos(x) >= 0
    # compute coefficients of quadratic equations
    # a sinx^2 + b sinx + c = 0
    a = A**2 + B**2
    b = -2*B*C
    c = C**2 - A**2
    return [np.arcsin(sol) for sol in np.roots([a, b, c])]

def solve_abc(A, B, C, guess):
    # solve Acos(x) + Bsin(x) = C
    # print('A'+str(A)+'B'+str(B)+'C'+str(C))
    # R = np.sqrt(A**2+B**2)
    # if A == 0.0:
    #     alpha = np.arctan(np.inf)
    # else:
    #     alpha = np.arctan(B/A)
    # sola = + np.arccos(C/R) + alpha
    # solb = - np.arccos(C/R) + alpha
    # sol = [sola, solb]
    root = fsolve(lambda x: A*np.cos(x) + B*np.sin(x) - C, guess)
    #print(root)
    return root

def get_Lambda_prime_x_CBTA_S1(spec):
    Xs, LS, gammas = Lambda_prime_alg_CBTA_S1(spec)
    LS = [-u if u is not None else None for u in LS]
    return Xs, LS, gammas

def get_Upsilon_prime_x_CBTA_S1(spec): # computation of Ypsilon_x(W) for CBTA-S1
    d, y, z, w, r, beta_lo, beta_hi = spec.extract_params()
    UPS = []
    gammas = []
    Xs = []
    gamma_arr = []
    ups_2 = []

    a_star_w = np.arccos(1-(d-w)/r)

    n1 = w - np.sqrt(2*r*d-d**2)
    if 2*r*(d-w) < d**2 + (d-w)**2:
        n2 = w + np.sqrt(r**2 - (r*np.sin(a_star_w)-d)**2) - r * np.cos(a_star_w)
    else:
        n2 = d

    for x in np.linspace(y, min(n2, z),100):
        print('x')
        print(x)
        sigma3 = (d**2 + (x-w)**2)/(2*r)
        A = (x-w)
        B = -d
        C = sigma3
        gamma_star_x_arr = solve_Acos_Bsin_C(A, B, C) # equation (3)
        print(gamma_star_x_arr)
        gamma_star_x_arr_2 = solve_abc(A,B,C,0)
        print('G'+str(gamma_star_x_arr_2))
        gamma_x = np.min(gamma_star_x_arr_2)
        for gamma_star_x in [gamma_x]:
            # if gamma_star_x < beta_lo:
            #     sigma1 = r*np.cos(beta_lo) + (x - w)
            #     sigma2 = r*np.sin(beta_lo) - d
            #     A = sigma1
            #     B = sigma2
            #     C = 1-(x-w)*np.cos(beta_lo)+d*np.sin(beta_lo)
            #     #Upsilon_prime_x = np.array(solve_Acos_Bsin_C(A, B, C)) # equation (4)
            #     Upsilon_prime_x = solve_abc(A,B,C)
            #     print('U'+str(np.rad2deg(Upsilon_prime_x)))
            #     # ups_2.append(Ups_prime_x_2)
            #     # max_val_min = np.min(Upsilon_prime_x / np.pi * 180)
            #     # max_val_max = np.max(Upsilon_prime_x / np.pi * 180)
            #     #print('A')
            #     max_val = np.max(np.rad2deg(Upsilon_prime_x))
            #     if not np.iscomplex(max_val):
            #         UPS.append(max_val)
            #         # UPS_max.append(max_val_max)
            #         gammas.append(gamma_star_x / np.pi * 180)
            #         Xs.append(x)
            #     break
            if beta_lo <= gamma_star_x and gamma_star_x < beta_hi or gamma_star_x < beta_lo:
                A = -(x-w)
                B = d
                C = sigma3
                Upsilon_prime_x = solve_abc(A,B,C,0) # equation (5)
                # print('U'+str(np.rad2deg(Upsilon_prime_x)))
                max_val = np.max(np.rad2deg(Upsilon_prime_x))
                if not np.iscomplex(max_val):
                    UPS.append(max_val)
                    gammas.append(gamma_star_x / np.pi * 180)
                    Xs.append(x)
                    break
            else:
                print('does not exist')
                UPS.append(None)
                Xs.append(x)
                gammas.append(gamma_star_x / np.pi * 180)
    return Xs, UPS, gammas

def Lambda_prime_alg_CBTA_S1(spec): # computation of Lambda for CBTA-S1
    # transformation of the algorithm for CBTA-S1 Lambda calculation done before function call
    d, y, z, w, r, beta_lo, beta_hi = spec.extract_params()
    LS = []
    gammas = []
    Xs = []

    a_star_w = np.arccos(1-(d-w)/r)
    n1 = r - np.sqrt(r**2-w**2)

    if 2*r*(d-w) < d**2 + (d-w)**2:
        n2 = (d-w) + np.sqrt(r**2 - (r*np.sin(a_star_w)-d)**2) - r * np.cos(a_star_w)
    else:
        n2 = d

    for x in np.linspace(max(n1,y), min(n2, z) ,100):
        sigma3 = (d**2 + (x-w)**2)/(2*r)
        A = (x-w)
        B = -d
        C = sigma3
        gamma_star_x_arr_2 = solve_abc(A,B,C,0)
        print('G'+str(np.rad2deg(gamma_star_x_arr_2)))
        gamma_x = np.max(gamma_star_x_arr_2)
        for gamma_star_x in [gamma_x]:
            # if gamma_star_x < beta_lo:
            #     sigma1 = r*np.cos(beta_lo) + (x - w)
            #     sigma2 = r*np.sin(beta_lo) - d
            #     A = sigma1
            #     B = sigma2
            #     C = 1-(x-w)*np.cos(beta_lo)+d*np.sin(beta_lo)
            #     Lambda_prime_x = solve_abc(A,B,C,0)
            #     print('L'+str(np.rad2deg(Lambda_prime_x)))
            #     max_val = np.max(np.rad2deg(Lambda_prime_x))
            #     if not np.iscomplex(max_val):
            #         LS.append(max_val)
            #         gammas.append(gamma_star_x / np.pi * 180)
            #         Xs.append(x)
            #     break
            if beta_lo <= gamma_star_x and gamma_star_x < beta_hi or gamma_star_x < beta_lo:
                A = -(x-w)
                B = d
                C = sigma3
                Lambda_prime_x = solve_abc(A,B,C,0) # equation (5)
                print('L'+str(np.rad2deg(Lambda_prime_x)))
                max_val = np.max(np.rad2deg(Lambda_prime_x))
                if not np.iscomplex(max_val):
                    LS.append(max_val)
                    gammas.append(gamma_star_x / np.pi * 180)
                    Xs.append(x)
                    break
            else:
                print('Lambda does not exist')
                LS.append(None)
                Xs.append(x)
                gammas.append(gamma_star_x / np.pi * 180)
    Xs = [(d-x) for x in Xs]
    return Xs, LS, gammas

def get_Upsilon_prime_x_CBTA_S2(spec):
    d, y, z, w, r, beta_lo, beta_hi = spec.extract_params()
    UPS = []
    gammas = []
    Xs = []
    gamguess = 0.5
    guess = 0.5
    for x in np.linspace(y,z,100):
        sigma6 = (w**2 + x**2)/(2*r)
        A = x
        B = -w
        C = sigma6
        gamma_star_x_arr = solve_abc(A,B,C, gamguess)
        gamma_star_x = np.min(gamma_star_x_arr)
        gamguess = gamma_star_x
        print('betalo'+str(beta_lo))
        print('beta_hi'+str(beta_hi))
        print('G'+str(gamma_star_x_arr))
        if gamma_star_x < beta_lo:
            sigma4 = np.sin(beta_lo)-w/r
            sigma5 = -np.cos(beta_lo)-x/r
            A = sigma4
            B = sigma5
            C = 1-x/r*np.cos(beta_lo)+w/r*np.sin(beta_lo)-sigma6/r
            Upsilon_prime_x = solve_abc(A,B,C, guess)
            print('U'+str(Upsilon_prime_x))
            guess = Upsilon_prime_x
            max_val = np.max(Upsilon_prime_x)
            if not np.iscomplex(max_val):
                UPS.append(np.rad2deg(max_val))
                Xs.append(x)
                gammas.append(np.rad2deg(gamma_star_x))
        elif beta_lo <= gamma_star_x < beta_hi:
            A = w
            B = x
            C = sigma6
            Upsilon_prime_x = solve_abc(A,B,C, guess)
            print('U'+str(Upsilon_prime_x))
            guess = Upsilon_prime_x
            max_val = np.max(Upsilon_prime_x)
            if not np.iscomplex(max_val):
                UPS.append(np.rad2deg(max_val))
                Xs.append(x)
                gammas.append(np.rad2deg(gamma_star_x))
        else:
            print('Upsilon does not exist')
            UPS.append(None)
            Xs.append(x)
            gammas.append(np.rad2deg(gamma_star_x))
    return Xs, UPS, gammas

def get_Lambda_prime_x_CBTA_S2(spec):
    d, y, z, w, r, beta_lo, beta_hi = spec.extract_params()
    LS = []
    gammas = []
    Xs = []

    if w > r:
        alpha_low_star = np.pi()/2
    elif w <= r:
        alpha_low_star = np.arccos(1-w/r)

    m_1 = r - np.sqrt(3*r**2+2*w*r-w**2)
    m_2 = 2*r*np.sqrt(2)-r*np.sin(alpha_low_star)

    n_1 = r - np.sqrt(r**2-w**2)
    n_2 = np.sqrt(2*r*w-w**2)
    #st()
    if m_1 >= y or n_2 <= z:
        print('do other stuff C.14 - not the case in this example')

    for x in np.linspace(n_1,z,100):
        sigma6 = (w**2 + x**2)/(2*r)
        A = -x
        B = w
        C = sigma6
        gamma_star_x_arr = solve_abc(A,B,C, 0.5)
        gamma_star_x = np.min(gamma_star_x_arr)
        if beta_hi < gamma_star_x:
            sigma7 = np.sin(beta_lo)-w/r
            sigma8 = - np.cos(beta_lo)-x/r
            A = sigma7
            B = sigma8
            C = 1-x/r*np.cos(beta_lo)+w/r*np.sin(beta_lo)-(w**2+x**2)/(2*r**2)
            Lambda_prime_x = solve_abc(A,B,C, -0.5)
            max_val = np.max(Lambda_prime_x)
            if not np.iscomplex(max_val):
                LS.append(np.rad2deg(max_val))
                Xs.append(x)
                gammas.append(np.rad2deg(gamma_star_x))
        elif beta_lo <= gamma_star_x < beta_hi or beta_hi < gamma_star_x:
            A = -w
            B = -x
            C = sigma6
            Lambda_prime_x = solve_abc(A,B,C, -0.5)
            max_val = np.max(Lambda_prime_x)
            if not np.iscomplex(max_val):
                LS.append(np.rad2deg(max_val))
                Xs.append(x)
                gammas.append(np.rad2deg(gamma_star_x))
        else:
            print('Lambda does not exist')
            LS.append(None)
            Xs.append(x)
            gammas.append(np.rad2deg(gamma_star_x))
    return Xs, LS, gammas

def find_alphas(UPS, LS):
    UPS_mod = [entry for entry in UPS if entry!=None]
    alpha_hi = np.max(UPS_mod)
    LS_mod = [entry for entry in LS if entry!=None]
    alpha_lo = np.min(LS_mod)
    return alpha_hi, alpha_lo

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
    # AVP Car parameters
    # Radius of curvature
    rad_car = 6 #m
    c_car = 1/rad_car #1/m

    # Choose which set of plots
    PLOT = 'SingleW' # 'SingleW' (check for specific w value),'Alphas' (find alpha_high and alpha_low)
    CASE = 'CBTA-S1' # 'CBTA-S1', 'CBTA-S2'

    # Example 1: Traversing a single rectangle
    d = 10 # grid size in meters
    y = 0 #
    z = 5
    w_s = 6 # enter value for w if want to check for specific w value
    w_arr = np.linspace(0,10,20) # interval of w for Alphas case
    r = 45 # maximum radius of curvature (assuming larger than box for now)
    beta_lo = -40/180*np.pi
    beta_hi = 10/180*np.pi
    alpha_his = []
    alpha_los = []
    if PLOT == 'Alphas':
        for w_cur in w_arr:
            if CASE == 'CBTA-S1':
                # spec for Upsilon analysis
                spec = Spec(d=d,y=y,z=z,w=w_cur,r=r,beta_lo=beta_lo,beta_hi=beta_hi)
                Xs, UPS, gammas = get_Upsilon_prime_x_CBTA_S1(spec)
                # alter spec for Lambda analysis (Flip horizontally)
                spec = Spec(d=d,y=d-z,z=d,w=d-w_cur,r=r,beta_lo=-beta_hi,beta_hi=-beta_lo)
                Xs2, LS, gammas2 = get_Lambda_prime_x_CBTA_S1(spec)
            elif CASE == 'CBTA-S2':
                # same spec for both algorithms
                spec = Spec(d=d,y=y,z=z,w=w_cur,r=r,beta_lo=beta_lo,beta_hi=beta_hi)
                Xs, UPS, gammas = get_Upsilon_prime_x_CBTA_S2(spec)
                Xs2, LS, gammas2 = get_Lambda_prime_x_CBTA_S2(spec)
            alpha_hi, alpha_lo = find_alphas(UPS, LS)
            alpha_his.append(alpha_hi)
            alpha_los.append(alpha_lo)
        st()
        plt.plot(w_arr, alpha_his, 'b',  label='Alpha High')
        plt.plot(w_arr, alpha_los, 'r', label='Alpha Low')
        #plt.ylim((-90,90))
        plt.xlim((0,10))
        plt.legend()
        plt.title('Upper and lower Angle Bounds')
        plt.show()
    elif PLOT == 'SingleW':
        spec = Spec(d=d,y=y,z=z,w=w_s,r=r,beta_lo=beta_lo,beta_hi=beta_hi)
        if CASE == 'CBTA-S1':
            Xs, UPS, gammas = get_Upsilon_prime_x_CBTA_S1(spec)
            spec = Spec(d=d,y=y,z=z,w=d-w_s,r=r,beta_lo=-beta_hi,beta_hi=-beta_lo)
            Xs2, LS, gammas2 = get_Lambda_prime_x_CBTA_S1(spec)
            Xs2 = [-(a-10) for a in Xs2]
        elif CASE == 'CBTA-S2':
            Xs, UPS, gammas = get_Upsilon_prime_x_CBTA_S2(spec)
            Xs2, LS, gammas2 = get_Lambda_prime_x_CBTA_S2(spec)
        plt.plot(Xs, gammas, 'b',label='Gamma')
        plt.plot(Xs, UPS, 'k',label='Upsilon')
        plt.plot(Xs2, LS, 'g', label='Lambda')
        # Plot the bounds
        plt.plot(Xs, np.rad2deg(beta_lo)*np.ones(len(Xs)), 'r-')
        plt.plot(Xs, np.rad2deg(beta_hi)*np.ones(len(Xs)), 'r-')
        # Configure plot axis to match graph in paper
        #plt.ylim((-90,90))
        plt.xlim((0,10))
        plt.legend()
        plt.title(CASE)
        st()
        plt.show()
