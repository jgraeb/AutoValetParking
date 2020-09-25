# Tung Phan, Josefine Graebener
# California Institute of Technology
# This is an implementation of Curvature-Bounded Traversability
# Analysis for Motion Planning of Mobile Robots by R. V. Cowlagi and P. Tsiotras
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

def solve_abc(a,b,c):
    denom = np.sqrt(a**2+b**2)
    beta = np.arctan(a/b)
    sol1 = - np.arcsin(c/denom) - beta
    sol2 = + np.arcsin(c/denom) - beta
    return [sol1, sol2]

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

    for x in np.linspace(max(n1,y), min(n2, z),100):
    #for x in np.linspace(y,n2,100):
        sigma3 = (d**2 + (x-w)**2)/(2*r)
        A = (x-w)
        B = -d
        C = sigma3
        gamma_star_x_arr = solve_Acos_Bsin_C(A, B, C) # equation (3)
        #print(np.rad2deg(gamma_star_x_arr))
        gamma_star_x_arr_2 = solve_abc(A,B,C)
        #print('G'+str(np.rad2deg(gamma_star_x_arr_2)))
        gamma_x = np.min(gamma_star_x_arr_2)
        #gamma_star_x_arr_deg = [g/ np.pi * 180 for g in gamma_star_x_arr]
        #gamma_arr.append(gamma_star_x_arr_2)
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
                #Upsilon_prime_x = np.array(solve_Acos_Bsin_C(A, B, C)) # equation (5)
                Upsilon_prime_x = solve_abc(A,B,C)
                # ups_2.append(Ups_prime_x_2)
                #print('U'+str(np.rad2deg(Upsilon_prime_x)))
                max_val = np.max(np.rad2deg(Upsilon_prime_x))
                #max_val_max = np.max(Upsilon_prime_x / np.pi * 180)
                if not np.iscomplex(max_val):
                    UPS.append(max_val)
                    #UPS_max.append(max_val_max)
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
    d, y, z, w, r, beta_lo, beta_hi = spec.extract_params()
    LS = []
    gammas = []
    Xs = []
    y = 5
    z = 10
    # transformation of the algorithm for CBTA-S1 Lambda calculation
    beta_lo = np.deg2rad(-10)#-beta_hi # betas are constant in x
    beta_hi = np.deg2rad(40)#-beta_lo
    #w = d-w

    a_star_w = np.arccos(1-(d-w)/r)

    n1 = r - np.sqrt(r**2-w**2)

    if 2*r*(d-w) < d**2 + (d-w)**2:
        n2 = (d-w) + np.sqrt(r**2 - (r*np.sin(a_star_w)-d)**2) - r * np.cos(a_star_w)
    else:
        n2 = d

    #for x in np.linspace(y, min(n2, z),100):
    for x in np.linspace(max(n1,y), min(n2, z) ,100):
        sigma3 = (d**2 + (x-w)**2)/(2*r)
        A = (x-w)
        B = -d
        C = sigma3
        gamma_star_x_arr_2 = solve_abc(A,B,C)
        print('G'+str(np.rad2deg(gamma_star_x_arr_2)))
        gamma_x = np.max(gamma_star_x_arr_2)
        for gamma_star_x in [gamma_x]:
            if gamma_star_x < beta_lo:
                sigma1 = r*np.cos(beta_lo) + (x - w)
                sigma2 = r*np.sin(beta_lo) - d
                A = sigma1
                B = sigma2
                C = 1-(x-w)*np.cos(beta_lo)+d*np.sin(beta_lo)
                # Lambda_prime_x = np.array(solve_Acos_Bsin_C(A, B, C)) # equation (4)
                Lambda_prime_x = solve_abc(A,B,C)
                print('L'+str(np.rad2deg(Lambda_prime_x)))
                # ups_2.append(Ups_prime_x_2)
                # max_val_min = np.min(Lambda_prime_x / np.pi * 180)
                # max_val_max = np.max(Lambda_prime_x / np.pi * 180)
                #print('A')
                max_val = np.min(np.rad2deg(Lambda_prime_x))
                if not np.iscomplex(max_val):
                    LS.append(max_val)
                    # UPS_max.append(max_val_max)
                    gammas.append(gamma_star_x / np.pi * 180)
                    Xs.append(x)
                break
            if beta_lo <= gamma_star_x and gamma_star_x < beta_hi or gamma_star_x < beta_lo:
                A = -(x-w)
                B = d
                C = sigma3
                Lambda_prime_x = solve_abc(A,B,C) # equation (5)
                print('L'+str(np.rad2deg(Lambda_prime_x)))
                max_val = np.min(np.rad2deg(Lambda_prime_x))
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
    for x in np.linspace(y,z,100):
        sigma6 = (w**2 + x**2)/(2*r)
        A = x
        B = -w
        C = sigma6
        gamma_star_x_arr = solve_abc(A,B,C)
        gamma_star_x = np.min(gamma_star_x_arr)
        if gamma_star_x < beta_lo:
            sigma4 = np.sin(beta_lo)-w/r
            sigma5 = -np.cos(beta_lo)-x/r
            A = sigma4
            B = sigma5
            C = 1-x/r*np.cos(beta_lo)+w/r*np.sin(beta_lo)-sigma6/r
            Upsilon_prime_x = solve_abc(A,B,C)
            max_val = np.max(Upsilon_prime_x)
            if not np.iscomplex(max_val):
                UPS.append(np.rad2deg(Upsilon_prime_x))
                Xs.append(x)
                gammas.append(np.rad2deg(gamma_star_x))
        elif beta_lo <= gamma_star_x < beta_hi:
            A = w
            B = x
            C = sigma6
            Upsilon_prime_x = solve_abc(A,B,C)
            max_val = np.max(Upsilon_prime_x)
            if not np.iscomplex(max_val):
                UPS.append(np.rad2deg(Upsilon_prime_x))
                Xs.append(x)
                gammas.append(np.rad2deg(gamma_star_x))
        else:
            print('Upsilon does not exist')
            UPS.append(None)
            Xs.append(x)
            gammas.append(np.rad2deg(gamma_star_x))

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

    if m_1 >= y or n_2 <= z:
        print('do other stuff C.14')

    for x in np.linspace(max(n1,y),min(n_2,z),100):
        sigma6 = (w**2 + x**2)/(2*r)
        A = -x
        B = w
        C = sigma6
        gamma_star_x_arr = solve_abc(A,B,C)
        gamma_star_x = np.min(gamma_star_x_arr)
        if beta_hi < gamma_star_x:
            sigma7 = np.sin(beta_lo)-w/r
            sigma8 = - np.cos(beta_lo)-x/r
            A = sigma7
            B = sigma8
            C = 1-x/r*np.cos(beta_lo)+w/r*np.sin(beta_lo)-(w**2+x**2)/(2*r**2)
            Lambda_prime_x = solve_abc(A,B,C)
            max_val = np.max(Lambda_prime_x)
            if not np.iscomplex(max_val):
                LS.append(np.rad2deg(max_val))
                Xs.append(x)
                gammas.append(np.rad2deg(gamma_star_x))
        elif beta_lo <= gamma_star_x < beta_hi:
            A = -w
            B = -x
            C = sigma6
            Lambda_prime_x = solve_abc(A,B,C)
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
    return Ls, gammas

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
    # Choose which set of plots
    PLOT = 'SingleW' # 'SingleW','Alphas'
    CASE = 'CBTA-S1' # 'CBTA-S1', 'CBTA-S2'
    # Example 1: Traversing a single rectangle
    d = 10 # grid size in meters
    y = 0 #
    z = 5
    w_s = 5
    w_arr = np.linspace(0,10,20)
    r = 45 # maximum radius of curvature (assuming larger than box for now)
    beta_lo = -40/180*np.pi
    beta_hi = 10/180*np.pi
    alpha_his = []
    alpha_los = []
    if PLOT == 'Alphas':
        for w in w_arr:
            #print(w)
            spec = Spec(d=d,y=y,z=z,w=w,r=r,beta_lo=beta_lo,beta_hi=beta_hi)
            Xs, UPS, gammas = get_Upsilon_prime_x_CBTA_S1(spec)
            spec = Spec(d=d,y=y,z=z,w=d-w,r=r,beta_lo=beta_lo,beta_hi=beta_hi)
            Xs2, LS, gammas2 = get_Lambda_prime_x_CBTA_S1(spec)
            #st()
            alpha_hi, alpha_lo = find_alphas(UPS, LS)
            alpha_his.append(alpha_hi)
            alpha_los.append(alpha_lo)
        plt.plot(w_arr, alpha_his, 'b',  label='Alpha High')
        plt.plot(w_arr, alpha_los, 'r', label='Alpha Low')
        plt.ylim((-90,90))
        plt.xlim((0,10))
        plt.legend()
        plt.title('Upper and lower Angle Bounds')
        plt.show()
    elif PLOT == 'SingleW':
        spec = Spec(d=d,y=y,z=z,w=w_s,r=r,beta_lo=beta_lo,beta_hi=beta_hi)
        if CASE == 'CBTA-S1':
            Xs, UPS, gammas = get_Upsilon_prime_x_CBTA_S1(spec)
            Xs2, LS, gammas2 = get_Lambda_prime_x_CBTA_S1(spec)
            #st()
        elif CASE == 'CBTA-S2':
            Xs, UPS, gammas = get_Upsilon_prime_x_CBTA_S2(spec)
            Xs2, LS, gammas2 = get_Lambda_prime_x_CBTA_S2(spec)
        plt.plot(Xs, gammas, 'b',label='Gamma')
        plt.plot(Xs[:len(UPS)], UPS, 'k',label='Upsilon')
        plt.plot(Xs2, LS, 'g', label='Lambda')
        #plt.plot(Xs2, gammas2, 'b',label='GammaL')
        # Plot the bounds
        plt.plot(Xs, np.rad2deg(beta_lo)*np.ones(len(Xs)), 'r-')
        plt.plot(Xs, np.rad2deg(beta_hi)*np.ones(len(Xs)), 'r-')
        # Configure plot axis to match graph in paper
        plt.ylim((-90,90))
        #plt.xlim((0,10))
        plt.legend()
        plt.title(CASE)
        plt.show()
    # check equations from paper
    #w = 1.25
    # diff3 = []
    # diff4 = []
    # diff5 =  []
    # for i in range(0,len(Xs)-1):
    #     sigma1 = r*np.cos(beta_lo) + (Xs[i]-w)
    #     sigma2 = r*np.sin(beta_lo) - d
    #     sigma3 = (d**2 + (Xs[i]-w)**2)/(2*r)
    #     # check equation 3
    #     diff3.append((Xs[i]-w)*np.cos(np.deg2rad(gammas[i]))-d*np.sin(np.deg2rad(gammas[i]))-sigma3)
    #     # check equation 4
    #     diff4.append(sigma1*np.cos(np.deg2rad(UPS_min[i]))+sigma2*np.sin(np.deg2rad(UPS_min[i]))+(Xs[i]-w)*np.cos(beta_lo)-d*np.sin(beta_lo)+sigma3-1)
    #     # check equation 5
    #     diff5.append(-(Xs[i]-w)*np.cos(np.deg2rad(UPS_min[i]))+d*np.sin(np.deg2rad(UPS_min[i]))-sigma3)
    #st()
