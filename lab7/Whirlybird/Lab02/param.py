# Whirlybird Parameter File
import numpy as np
import control as cnt

# From parameters list of the Whirlybird lab documentation
g = 9.81       # Gravity, m/s**2
L1 = 0.85      # Length of rod from vertex to the rotors, m
L2 = 0.3048    # Length of rod from vertex to counterbalance, m
m1 = 0.891     # Mass of weight at the end of l1 (between the rotors), kg
m2 = 1.00      # Mass of counterbalance at the end of l2, kg
d = 0.178      # Distance between m1 and a rotor (half the distance between rotors), m
h = 0.65       # Height of support rod, m
r = 0.12       # Length of one of the square rotors (maybe radius instead?), m
Jx = 0.0047    # Some force in x direction???, kg-m^2
Jy = 0.0014    # Some force in y direction???, kg-m^2
Jz = 0.0041    # Some force in z direction???, kg-m^2
#km = ????     # On parameters list, will add in later
#omega_gyro    # "
#omega_pixel   # "

# Other parameters for the animation
ell =  5      # For the axis, m

# Simulation Parameters
Ts = 0.0033
sigma = 0.05

# For conversion to pwm
pwmConversionFactor = 10.875
pwm_e = .48
km = (m1*L1*g - m2*L2*g) / (L1 *(2*pwm_e))

# Initial Conditions
phi0 = 0.0*np.pi/180    # Pitch of Whirlybird relative to the ground, rads
phidot0 = 0.0           # Derivative of the pitch
theta0 = 0.0*np.pi/180  # Roll of Whirlybird relative to the ground, rads
thetadot0 = 0.0         # Derivative of the roll
psi0 = 0.0*np.pi/180    # Yaw of Whirlybird relative to the ground, rads
psidot0 = 0.0           # Derivative of the yaw

F_e = (m1 - m2*(L2/L1))*g
phimax = 45.0 *np.pi / 180.0
Fmax = 50.0
taumax = 50.0
thetamax = 60.0*np.pi / 180.0
####################################################
#                 PID Control:  Longitudinal
####################################################
# Flong_max = 10.85

# Open Loop
# kp*b0/(S**2 + kd*b*S + kp*b)
th_b0 = (L1/(m1*L1**2+m2*L2**2+Jy))
th_a1 = 0.0
th_a0 = 0.0

# Desired Closed Loop tuning parameters
# S**2 + 2*zeta*wn*S + wn**2

th_tr = 1.4           # Rise time, s
th_zeta = 0.7      # Damping Coefficient
th_wn = 2.2/th_tr     # Natural frequency

# S**2 + alpha1*S + alpha0
th_alpha1 = 2.0*th_zeta*th_wn
th_alpha0 = th_wn**2

# Gains
# b0*kp/[S**2 + (a1 + b0*kd)S + (a0 + b0*kp)]
# th_kp = (th_alpha0-th_a0)/th_b0
# th_kd = (th_alpha1-th_a1)/th_b0

# th_ki = 0.1
# th_windup = 2

## Full-state feedback stuffs
# Matices
# TODO what do we use for theta_e???
A_lon = np.array([[0, 1],
                  [((m1*L1 - m2*L2)*g*np.sin(theta0))/(m1*L1**2 + m2*L2**2 + Jy), 0]])

B_lon = np.array([[0],
                  [L1/(m1*L1**2 + m2*L2**2 + Jy)]])

C_lon = np.array([[1,0]])

Cr_lon = C_lon

A1_lon = np.concatenate((
            np.concatenate((A_lon, np.zeros((2,1))), axis=1),
            np.concatenate((-Cr_lon, np.matrix([[0.0]])),axis=1)), axis=0)

B1_lon = np.concatenate((B_lon, np.matrix([[0.0]])),axis=0)

integrator_pole_lon = -10

des_char_poly_lon = np.convolve([1, th_alpha1, th_alpha0], np.poly(integrator_pole_lon))
des_poles_lon = np.roots(des_char_poly_lon)

if np.linalg.matrix_rank(cnt.ctrb(A1_lon,B1_lon))!=3:
    print("The system is not controllable")
else:
    K1_lon = cnt.acker(A1_lon,B1_lon, des_poles_lon)
    K_lon = K1_lon[0, 0:2]
    ki_lon = K1_lon[0,2]
    print('K1_lon: ', K1_lon)
    print('K_lon: ', K_lon)
    print('ki_lon: ', ki_lon)


####################################################
#                 PID Control:  Lateral
####################################################
#Flat_max = ?

#---------------------------------------------------
#                    Roll (PHI)
#---------------------------------------------------
# Open Loop
# (kp/Jx)/(S**2 + (kp/Jx)*S + (kp/Jx))
phi_b0 = 1/Jx
# these are not used for some reason?
phi_a1 = 0.0
phi_a0  = 0.0

# Desired Closed Loop tuning parameters
# S**2 + 2*zeta*wn*S + wn**2

phi_zeta = 0.7       # Damping Coefficient
phi_tr = 0.2          # Rise time, s
phi_wn = 2.2/phi_tr     # Natural frequency

# S**2 + alpha1*S + alpha0
phi_alpha1 = 2.0*phi_zeta*phi_wn
phi_alpha0 = phi_wn**2

# Gains
# b0*kp/[S**2 + (a1 + b0*kd)S + (a0 + b0*kp)]
phi_kp = (phi_alpha0)/phi_b0
phi_kd = (phi_alpha1)/phi_b0
phi_DC = 1   #DC gain

phi_ki = 0.0

#---------------------------------------------------
#                    Yaw (PSI)
#---------------------------------------------------
M = 10.0    # bandwidth separation factor
# Open Loop
# kp*b0/(S**2 + kd*b*S + kp*b)
psi_b0 = (L1*F_e/(m1*L1**2+m2*L2**2+Jz))
psi_a1 = 0.0
psi_a0 = 0.0

# Desired Closed Loop tuning parameters
# S**2 + 2*zeta*wn*S + wn**2

psi_tr = M*phi_tr  # Rise time, s
psi_zeta = 0.707      # Damping Coefficient
psi_wn = 2.2/psi_tr  # Natural frequency


# S**2 + alpha1*S + alpha0
psi_alpha1 = 2.0*psi_zeta*psi_wn
psi_alpha0 = psi_wn**2

# Gains
# b0*kp/[S**2 + (a1 + b0*kd*IL_DC)S + (a0 + b0*kp*IL_DC)]
psi_kp = (psi_alpha0-psi_a0)/(phi_DC*psi_b0)
psi_kd = (psi_alpha1-psi_a1)/(phi_DC*psi_b0)

psi_ki = 0.1
psi_windup = 0.15

A_lat = np.array([[0.,0.,1.,0.],
                  [0.,0.,0.,1.],
                  [0.,0.,0.,0.],
                  [L1*F_e/(m1*L1**2 + m2*L2**2 + Jz), 0.,0.,0.]])

B_lat = np.array([[0.],
                  [0.],
                  [1./Jx],
                  [0.]])

C_lat = np.array([[1.,0.,0.,0.],
                 [0.,1.,0.,0.]])

Cr_lat = np.matrix(C_lat[1,:])

A1_lat = np.concatenate((
            np.concatenate((A_lat, np.zeros((4,1))), axis=1),
            np.concatenate((-Cr_lat, np.matrix([[0.0]])),axis=1)),axis=0)

B1_lat = np.concatenate((B_lat,np.matrix([[0.0]])),axis=0)

integrator_pole_lat = 0.0

des_char_poly_lat = np.convolve([1, psi_alpha1, psi_alpha0],
                        [1, phi_alpha1 ,phi_alpha0])
des_char_poly_lat = np.convolve(des_char_poly_lat, np.poly(integrator_pole_lat))
des_poles_lat = np.roots(des_char_poly_lat)

if np.linalg.matrix_rank(cnt.ctrb(A1_lat,B1_lat)) != 5:
    print("The lateral system is not controllable")
else:
    K1_lat = cnt.acker(A1_lat,B1_lat, des_poles_lat)
    K_lat = K1_lat[0, 0:4]
    ki_lat = K1_lat[0,4]


# print('km: ', km)
# print('th_kp: ', th_kp)
# print('th_kd: ', th_kd)
# print('phi_kp: ',phi_kp)
# print('phi_kd: ',phi_kd)
# print('phi_DC:', phi_DC)
# print('psi_kp: ',psi_kp)
# print('psi_kd: ',psi_kd)
