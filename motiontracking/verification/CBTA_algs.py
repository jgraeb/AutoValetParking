# Tung Phan, Josefine Graebener
# California Institute of Technology
# An implementation of the algorithms in Curvature-Bounded Traversability
# Analysis for Motion Planning of Mobile Robots by R. V. Cowlagi and P. Tsiotras
# https://users.wpi.edu/~rvcowlagi/publications/2014-tro-cbta.pdf
# and https://users.wpi.edu/~rvcowlagi/publications/rvcowlagi-phdthesis.pdf

from scipy.optimize import fsolve
#import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from typing import List, Tuple
from ipdb import set_trace as st
import operator


class ChannelProblemSpecifications:
    """
    Problem specification class.
    """
    def __init__(self, d: float, y: float, z: float, w: float, r: float, beta_lo: float, beta_hi: float):
        """
        Init class object with params.
        :param d: Square channel side length.
        :param y: Lower bound on exit edge.
        :param z: Upper bound on exit edge.
        :param w: Position of entry point on entry edge.
        :param r: Minimum radicus of curvature.
        :param beta_lo: Min allowable heading on exit.
        :param beta_hi: Max allowable  heading on exit.
        """
        self.d = d
        self.y = y
        self.z = z
        self.w = w
        self.r = r
        self.beta_lo = beta_lo
        self.beta_hi = beta_hi

    def extract_params(self) -> Tuple[float]:
        """
        Return all parameters as a tuple.
        """
        return self.d, self.y, self.z, self.w, self.r, self.beta_lo, self.beta_hi


def solve_abc(A: float, B: float, C: float, guess: float) -> List[float]:
    """
    Solves the equation A cos(x) + B sin(x) - C for x, given guess.
    :param A: A coefficent.
    :param B: B coefficent.
    :param C: C coefficent.
    :param guess: An initial guess.
    :return: A list of solutions.
    """
    if guess is None:
        #st()
        root0 = fsolve(lambda x: A*np.cos(x) + B*np.sin(x) - C, 0)
        root1 = fsolve(lambda x: A*np.cos(x) + B*np.sin(x) - C, -np.pi/2)
        root2 = fsolve(lambda x: A*np.cos(x) + B*np.sin(x) - C, np.pi/2)
        print(root0,root1,root2)
        if np.abs(root0)< np.pi/2:
            return root0
        root = [root0,root1,root2]
        absroot = [np.abs(root0),np.abs(root1),np.abs(root2)]
        idx = absroot.index(min(absroot))
        sol = check_if_solution_exists(A,B,C,root[idx])
        if sol:
            return root[idx]
        else:
            return sol
    else:
        root = fsolve(lambda x: A*np.cos(x) + B*np.sin(x) - C, guess)
        print(root)
        sol = check_if_solution_exists(A,B,C,root)
        if sol:
            return root
        else:
            return sol

def check_if_solution_exists(A,B,C,x):
    rem = A*np.cos(x)+B*np.sin(x)-C
    if np.abs(rem) < 0.001:
        return True
    else:
        return False

def solve_abc_and_bound_max(A: float, B: float, C: float, guess: float, a_star: float) -> float:
    return max(solve_abc(A, B, C, guess), a_star)

def get_Lambda_prime_x_CBTA_S1(spec: ChannelProblemSpecifications):
    Xs, LS, gammas = get_Upsilon_prime_x_CBTA_S1(spec) # use same function after flipping the spec
    LS = [-u if u is not None else None for u in LS] # flip the result as well
    return Xs, LS, gammas

def get_union_interval(abounds, bbounds, stepsize = 0.1):
    y,m1 = abounds
    m2,z = bbounds
    linspace_a = list(np.arange(y,m1,stepsize))
    linspace_b = list(np.arange(m2,z,stepsize))
    linspace_c = linspace_a + linspace_b
    return linspace_c

def get_intersection_of_set_with_interval(listinterval, intervalb):
    return [item for item in listinterval if item <= intervalb[-1] and item >= intervalb[0]]

def get_Upsilon_prime_x_CBTA_S1(spec: ChannelProblemSpecifications): # computation of Ypsilon_x(W) for CBTA-S1 # Case 3
    d, y, z, w, r, beta_lo, beta_hi = spec.extract_params()
    UPS = []
    gammas = []
    Xs = []
    gamma_arr = []
    ups_2 = []
    # squares instead of rectangles
    d1 = d2 = d
    guess = None
    gamguess = None

    a_star_w = np.arccos(1-(d-w)/r)

    if False:#r<=(d1**2+(d2-w)**2)/(2*(d2-w)) and r+np.sqrt(2*r*(d2-w)-(d2-w)**2) < d1:
        print('execute proc C.4') # not used in this case - gives issues
    else:
        m1 = w - np.sqrt(4*r**2-(r*np.sin(a_star_w)-d1)**2) - r*np.cos(a_star_w)
        m2 = w + np.sqrt(4*r**2-(r*np.sin(a_star_w)-(d1-r))**2) - r*np.cos(a_star_w)
        n1 = w - np.sqrt(2*r*d1-d1**2)
        n2 = w + np.sqrt(r**2 - (r*np.sin(a_star_w)-d1)**2) - r * np.cos(a_star_w)
        #st()
        if m1>=y or m2<=z:
            for x in np.linspace(min(y,m2), max(m1, z),100):
            #for x in np.linspace(y,z,100):#get_union_interval((y,m2),(m1, z)):
                Upsilon_prime_x = a_star_w
                print('Ups'+str(Upsilon_prime_))
                max_val = np.max(np.rad2deg(Upsilon_prime_x))
                if not np.iscomplex(max_val):
                    UPS.append(max_val)
                    #gammas.append(beta_star_x / np.pi * 180)
                    Xs.append(x)
                    #st()
        #elif n1>=y or n2<=z:
            # # execute 8-14 of Fig. C.4
            # listinterval = get_union_interval((m1,n1), (n2,m2))
            # #for x in np.linspace(max(m1,y), min(m2,z)):
            # for x in get_intersection_of_set_with_interval(listinterval, (y,z)): # check n1
            #     A = np.cos(a_star_w)+(x-w)/r
            #     B = np.sin(a_star_w)-d1/r
            #     C = 1 - (x-w)/r*np.cos(a_star_w) + d1/r*np.sin(a_star_w) - ((x-w)**2+d1**2)/(2*r**2)
            #     beta_star_x = solve_abc(A,B,C,gamguess)
            #     print('beta'+str(beta_star_x))
            #     if beta_star_x < beta_lo:
            #         A = np.cos(beta_lo)+(x-w)/r
            #         B = np.sin(beta_lo)- d1/r
            #         C = 1 - (x-w)/r*np.cos(beta_lo) + d1/r*np.sin(beta_lo)-((x-w)**2+d1**2)/(2*r**2)
            #         Upsilon_prime_x = solve_abc(A,B,C,guess)
            #         guess = Upsilon_prime_x
            #     else:
            #         Upsilon_prime_x = a_star_w
            #     print('Ups'+str(Upsilon_prime_x))
            #     max_val = np.max(np.rad2deg(Upsilon_prime_x))
            #     if not np.iscomplex(max_val):
            #         UPS.append(max_val)
            #         gammas.append(beta_star_x / np.pi * 180)
            #         Xs.append(x)
            #         #st()

        for x in np.linspace(y, min(n2, z),100):
        #for x in np.linspace(max(y,n1), min(n2, z),100):
            sigma3 = (d1**2 + (x-w)**2)/(2*r)
            A = (x-w)
            B = -d1
            C = sigma3
            gamma_star_x_arr_2 = solve_abc(A,B,C,0)
            print('Gamma'+str(gamma_star_x_arr_2))
            gamma_x = np.min(gamma_star_x_arr_2)
            for gamma_star_x in [gamma_x]:
                if gamma_star_x < beta_lo:
                    sigma1 = np.cos(beta_lo) + (x - w)/r
                    sigma2 = np.sin(beta_lo) - d1/r
                    A = sigma1
                    B = sigma2
                    C = 1-(x-w)/r*np.cos(beta_lo)+d1/r*np.sin(beta_lo)-sigma3/r
                    Upsilon_prime_x = solve_abc(A,B,C,guess) # equation (4)
                    guess = Upsilon_prime_x
                    print('Ups'+str(Upsilon_prime_x))
                    max_val = np.max(np.rad2deg(Upsilon_prime_x))
                    if not np.iscomplex(max_val):
                        UPS.append(max_val)
                        gammas.append(gamma_star_x / np.pi * 180)
                        Xs.append(x)
                        #st()
                    break
                if beta_lo <= gamma_star_x and gamma_star_x < beta_hi:# or gamma_star_x < beta_lo:
                    A = -(x-w)
                    B = d1
                    C = sigma3
                    Upsilon_prime_x = solve_abc(A,B,C,guess) # equation (5)
                    guess = Upsilon_prime_x
                    print('Ups'+str(Upsilon_prime_x))
                    max_val = np.max(np.rad2deg(Upsilon_prime_x))
                    if not np.iscomplex(max_val):
                        UPS.append(max_val)
                        gammas.append(gamma_star_x / np.pi * 180)
                        Xs.append(x)
                        #st()
                        break
                else:
                    #st()
                    print('does not exist')
                    UPS.append(None)
                    Xs.append(x)
                    gammas.append(gamma_star_x / np.pi * 180)
    return Xs, UPS, gammas

def get_a_star_max(d1,d2,w,r):
    """
    Return a_star for the adjacent channel case.
    """
    # (C.11) and (C.12) of thesis
    # Define alpha_1_star
    if d2 - w > r:
        a_star_w_1 = np.pi/2
    else:
        a_star_w_1 = np.arccos(1-(d2-w)/r)

    # Define alpha_2_star
    if r <= d1/2:
        a_star_w_2 = np.pi/2
    elif r > d1/2 and w < np.sqrt(2*d1*r-d1**2):
        a_star_w_2 = np.arcsin(d1/r-1)
    else:
        d_hat = np.sqrt((d1**2+w**2)*(4*r**2-1))/(d1**2+2*r*w+w**2)
        val1 = 2*np.arctan(d1+d_hat)
        val2 = 2*np.arctan(d1-d_hat)
        a_star_w_2 = min(val1, val2)

    # Take the min
    a_star_w = min(a_star_w_1, a_star_w_2)
    return a_star_w

def get_a_star_min(w: float, r: float) -> float:
    """
    Compute a_star_min in (C.14) from thesis.
    """
    if w < r:
        a_star_w = np.pi/2
    else:
        a_star_w = np.arccos(1-w/r)
    return -a_star_w

def get_Upsilon_prime_x_CBTA_S2(spec): # traversal across adjacent edges Case 3A - NOT RELIABLE
    d, y, z, w, r, beta_lo, beta_hi = spec.extract_params()
    UPS = []
    gammas = []
    Xs = []
    # now for square boxes
    d1 = d2 = d
    # guesses for solver
    gamguess = None
    guess = None
    # find a_star_w
    a_star_w = get_a_star_max(d1,d2,w,r)
    # st()
    # print('astarw'+str(a_star_w))
    # start algorithm Fig.12
    if r+r*np.cos(a_star_w)<w:
        print('do fig C.11') # not used in example
        #st()
    else:
        #st()
        m = r*np.sin(a_star_w) + np.sqrt(2*r**2*(1-np.cos(a_star_w)+0.5*np.sin(a_star_w)**2) +2*w*r*(1 + np.cos(a_star_w)) -w**2)
        n = r*np.sin(a_star_w) + np.sqrt(r**2*np.sin(a_star_w)**2 + 2*w*r*np.cos(a_star_w) - w**2)

        if m <= z:
            for x in np.linspace(m, z, 100):
                Upsilon_prime_x = a_star_w
                UPS.append(np.rad2deg(Upsilon_prime_x))
                Xs.append(x)
                #gammas.append(np.rad2deg(gamma_star_x))
        for x in np.linspace(max(n,y), min(m,z), 100): ## TODO: check this
            #print('6-12 of C.11')
            A = -np.sin(a_star_w) + x/r
            B = np.cos(a_star_w) - w/r
            C = 1 + (w/r)*np.cos(a_star_w) + (x/r)*np.sin(a_star_w)-(w**2+x**2)/(2*r**2)
            beta_star_x = solve_abc(A,B,C, gamguess)
            if beta_star_x:
                gamguess = beta_star_x
                print('beta'+str(beta_star_x))
                if beta_star_x < beta_lo:
                    A = np.sin(beta_lo)-w/r
                    B = -np.cos(beta_lo)-x/r
                    C = 1 - (x/r)*np.cos(beta_lo) + (w/r)*np.sin(beta_lo) - (w**2+x**2)/(2*r**2)
                    Upsilon_prime_x = solve_abc(A,B,C, guess)
                    guess = Upsilon_prime_x
                    if Upsilon_prime_x > a_star_w:
                        Upsilon_prime_x = a_star_w
                    #print('U'+str(Upsilon_prime_x))
                    UPS.append(np.rad2deg(Upsilon_prime_x))
                    Xs.append(x)
                    gammas.append(np.rad2deg(beta_star_x))
                else:
                    Upsilon_prime_x = a_star_w
                    UPS.append(np.rad2deg(Upsilon_prime_x))
                    Xs.append(x)
                    gammas.append(np.rad2deg(beta_star_x))
            else:
                print('skipped')
                #st()
        for x in np.linspace(y,n,100):
            sigma6 = (w**2 + x**2)/(2*r)
            A = x
            B = -w
            C = sigma6
            gamma_star_x_arr = solve_abc(A,B,C, gamguess)
            if gamma_star_x_arr:
                gamma_star_x = np.min(gamma_star_x_arr)
                gamguess = gamma_star_x
                print('gamma'+str(gamma_star_x))
                #print('G'+str(gamma_star_x_arr))
                if gamma_star_x < beta_lo:
                    sigma4 = np.sin(beta_lo)-w/r
                    sigma5 = -np.cos(beta_lo)-x/r
                    A = sigma4
                    B = sigma5
                    C = 1-x/r*np.cos(beta_lo)+w/r*np.sin(beta_lo)-(w**2+x**2)/(2*r**2)
                    Upsilon_prime_x = solve_abc(A,B,C, guess)
                    if Upsilon_prime_x > a_star_w:
                        Upsilon_prime_x = a_star_w
                    #print('U'+str(Upsilon_prime_x))
                    guess = Upsilon_prime_x
                    UPS.append(np.rad2deg(Upsilon_prime_x))
                    Xs.append(x)
                    gammas.append(np.rad2deg(gamma_star_x))
                elif beta_lo <= gamma_star_x < beta_hi:
                    A = w
                    B = x
                    C = sigma6
                    Upsilon_prime_x = solve_abc(A,B,C,guess)
                    if Upsilon_prime_x > a_star_w:
                        Upsilon_prime_x = a_star_w
                    #print('U'+str(Upsilon_prime_x))
                    guess = Upsilon_prime_x
                    UPS.append(np.rad2deg(Upsilon_prime_x))
                    Xs.append(x)
                    gammas.append(np.rad2deg(gamma_star_x))
                else:
                    #print('Upsilon does not exist')
                    UPS.append(None)
                    Xs.append(x)
                    gammas.append(np.rad2deg(gamma_star_x))
            # else:
            #     st()
    return Xs, UPS, gammas

def get_Lambda_prime_x_CBTA_S2(spec): # traversal across adjacent edges Case 3B - NOT RELIABLE
    d, y, z, w, r, beta_lo, beta_hi = spec.extract_params()
    LS = []
    gammas = []
    Xs = []
    # squares
    d1 = d2 = d
    #
    guess = -0.5
    gamguess = 0
    #st()
    # find a_star_w
    # if w!=0:
    #     if r<=(d1**2+w**2)/(2*w):
    #         a_star_w = get_a_star_min(w,r)
    #         LS.append(np.rad2deg(a_star_w))
    #         return Xs, LS, gammas
    a_star_w = get_a_star_min(w,r)
    #else:
    #    print(r,w)
    #    st()
    m1 = r - np.sqrt(3*r**2+2*w*r-w**2)
    m2 = 2*r*np.sqrt(2)-r*np.sin(a_star_w)
    n1 = r - np.sqrt(r**2-w**2)
    n2 = np.sqrt(2*r*w-w**2)

    if m1 >= y or m2 <= z:
        #st()
        m = r*np.sin(a_star_w) + np.sqrt(2*r**2*(1-np.cos(a_star_w)+0.5*np.sin(a_star_w)**2) +2*w*r*(1 + np.cos(a_star_w)) -w**2)
        for x in np.linspace(m, z, 100):
            LS.append(np.rad2deg(a_star_w))
            Xs.append(x)
    if n1>=y or n2<=z:
        #st()
        for x in np.linspace(max(n1,y),min(m2,z),100):
            #print('lines 5-11 of C.14')
            A = -(np.sin(a_star_w)+x/r)
            B = np.cos(a_star_w)+w/r
            C = 1 - w/r*np.cos(a_star_w)-x/r*np.sin(a_star_w)-(w**2+x**2)/(2*r**2)
            beta_star_x = solve_abc(A,B,C, gamguess)
            gamguess = beta_star_x
            if beta_lo < beta_star_x:
                A = np.sin(beta_lo)-w/r
                B = -np.cos(beta_lo)-x/r
                C = 1 - x/r*np.cos(beta_lo)+w/r*np.sin(beta_lo)-(w**2+x**2)/(2*r**2)
                Lambda_prime_x = solve_abc_and_bound_max(A,B,C,guess, a_star_w)
                guess = Lambda_prime_x
                #print('L'+str(Lambda_prime_x))
                LS.append(np.rad2deg(Lambda_prime_x))
                Xs.append(x)
                gammas.append(np.rad2deg(beta_star_x))
            else:
                LS.append(np.rad2deg(a_star_w))
                Xs.append(x)


    for x in np.linspace(max(n1,y), min(n2,z), 100):
        sigma6 = (w**2 + x**2)/(2*r)
        A = -x
        B = w
        C = sigma6
        gamma_star_x = solve_abc(A,B,C, gamguess)
        gamguess = gamma_star_x
        # gamma_star_x = np.min(gamma_star_x_arr)
        if beta_hi < gamma_star_x:
            sigma7 = np.sin(beta_lo)-w/r
            sigma8 = - np.cos(beta_lo)-x/r
            A = sigma7
            B = sigma8
            C = 1-x/r*np.cos(beta_lo)+w/r*np.sin(beta_lo)-(w**2+x**2)/(2*r**2)
            Lambda_prime_x = solve_abc_and_bound_max(A,B,C, guess, a_star_w)
            guess = Lambda_prime_x
            #print('L'+str(Lambda_prime_x))
            LS.append(np.rad2deg(Lambda_prime_x))
            Xs.append(x)
            gammas.append(np.rad2deg(gamma_star_x))
        elif beta_lo <= gamma_star_x < beta_hi:
            A = -w
            B = -x
            C = sigma6
            Lambda_prime_x = solve_abc_and_bound_max(A,B,C, guess, a_star_w)
            guess = Lambda_prime_x
            #print('L'+str(Lambda_prime_x))
            LS.append(np.rad2deg(Lambda_prime_x))
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

def sort(Xs,UPS):
    U = sorted(zip(Xs,UPS), key=operator.itemgetter(0))
    new_Xs, new_UPS = zip(*U)
    return new_Xs, new_UPS

def smooth(w_arr,alpha_his):
    delidxs = []
    w_arr1 = w_arr
    for idx, val in enumerate(alpha_his):
        if idx >= 1:
            if np.abs(val - alpha_his[idx-1])> 10:
                delidxs.append(idx)
            elif idx-1 in delidxs:
                if np.abs(val - alpha_his[idx-2])> 10:
                    delidxs.append(idx)
            elif np.abs(val)>90:
                delidxs.append(idx)
    delidxs = np.flip(delidxs)
    for idx in delidxs:
        alpha_his.pop(idx)
        w_arr1 = np.delete(w_arr1, idx)
    return w_arr1, alpha_his


if __name__ == '__main__':
    # AVP Car parameters
    # Radius of curvature
    rad_car = 6 # m
    c_car = 1/rad_car # 1/m

    # Choose which set of plots
    PLOT = 'Alphas' # 'SingleW' (check for specific w value),'Alphas' (find alpha_high and alpha_low)
    CASE = 'CBTA-S1' # 'CBTA-S1', 'CBTA-S2'

    # Example 1: Traversing a single square
    buffer = 2.0
    d = 10 # Grid size in meters
    y = buffer/2 # Lower bound on exit edge
    z = d-buffer/2 # Upper bound on exit edge

    w_s = 0 # enter value for w if want to check for specific w value
    w_arr = np.linspace(0,d,100) # interval of w for Alphas case
    r = 20.0 # maximum radius of curvature (assuming larger than box for now)
    beta_lo = -5/180.0 * np.pi
    beta_hi = 5/180.0 * np.pi

    alpha_his = []
    alpha_los = []

    if PLOT == 'Alphas':
        for w_cur in w_arr:
            if CASE == 'CBTA-S1':
                # spec for Upsilon analysis
                spec = ChannelProblemSpecifications(d=d,y=y,z=z,w=w_cur,r=r,beta_lo=beta_lo,beta_hi=beta_hi)
                Xs, UPS, gammas = get_Upsilon_prime_x_CBTA_S1(spec)
                # alter spec for Lambda analysis (Flip horizontally)
                spec = ChannelProblemSpecifications(d=d,y=d-z,z=d,w=d-w_cur,r=r,beta_lo=-beta_hi,beta_hi=-beta_lo)
                Xs2, LS, gammas2 = get_Lambda_prime_x_CBTA_S1(spec)
                Xs, UPS = sort(Xs,UPS)
                Xs2, LS = sort(Xs2,LS)
            elif CASE == 'CBTA-S2':
                # same spec for both algorithms
                spec = ChannelProblemSpecifications(d=d,y=y,z=z,w=w_cur,r=r,beta_lo=beta_lo,beta_hi=beta_hi)
                Xs, UPS, gammas = get_Upsilon_prime_x_CBTA_S2(spec)
                Xs2, LS, gammas2 = get_Lambda_prime_x_CBTA_S2(spec)
            try:
                alpha_hi, alpha_lo = find_alphas(UPS, LS)
            except:
                st()
            alpha_his.append(alpha_hi)
            alpha_los.append(alpha_lo)
        # scale w_arr down again
        w_arr = [item*0.3 for item in list(w_arr)]
        w_arr1, alpha_his = smooth(w_arr,alpha_his)
        w_arr2, alpha_los = smooth(w_arr,alpha_los)
        plt.rcParams.update({"text.usetex": True,"font.family": "sans-serif","font.sans-serif": ["Helvetica"]})
        plt.plot(w_arr1, alpha_his, 'b',  label=r'$\overline{\alpha}$')
        plt.plot(w_arr2, alpha_los, 'r', label=r'$\underline{\alpha}$')
        plt.xlabel('w')
        plt.ylabel('Angle')
        plt.ylim((-90,90))
        plt.xlim((0,w_arr[-1]))
        plt.legend()
        plt.title('Angle Bounds '+str(CASE))
        plt.show()
    elif PLOT == 'SingleW':
        spec = ChannelProblemSpecifications(d=d,y=y,z=z,w=w_s,r=r,beta_lo=beta_lo,beta_hi=beta_hi)
        if CASE == 'CBTA-S1':
            Xs, UPS, gammas = get_Upsilon_prime_x_CBTA_S1(spec)
            spec = ChannelProblemSpecifications(d=d,y=y,z=z,w=d-w_s,r=r,beta_lo=-beta_hi,beta_hi=-beta_lo)
            Xs2, LS, gammas2 = get_Lambda_prime_x_CBTA_S1(spec)
            Xs2 = [-(a-d) for a in Xs2]
        elif CASE == 'CBTA-S2':
            Xs, UPS, gammas = get_Upsilon_prime_x_CBTA_S2(spec)
            Xs2, LS, gammas2 = get_Lambda_prime_x_CBTA_S2(spec)
        new_Xs, new_UPS = sort(Xs,UPS)
        new_Xs2, new_LS = sort(Xs2,LS)
        new_Xs0, new_gammas = sort(Xs,gammas)
        plt.plot(new_Xs0, new_gammas, 'b',label='Gamma')
        plt.plot(new_Xs, new_UPS, 'k',label='Upsilon')
        plt.plot(new_Xs2, new_LS, 'g', label='Lambda')
        # Plot the bounds
        plt.plot(Xs, np.rad2deg(beta_lo)*np.ones(len(Xs)), 'r-')
        plt.plot(Xs, np.rad2deg(beta_hi)*np.ones(len(Xs)), 'r-')
        # Configure plot axis to match graph in paper
        #plt.ylim((-90,90))
        #plt.xlim((0,10))
        st()
        plt.legend()
        plt.title(CASE)
        plt.show()
