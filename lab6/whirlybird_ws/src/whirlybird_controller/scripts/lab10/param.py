
import numpy as np

# System Parameters

g = 9.81               # Gravity, m/s**2
l1 = 0.85              # Distance between fulcrum and the head , m
l2 = 0.3048            # Distance between the fulcrum and end of Whirlybird's tail, m
m1 = 0.891             # Mass of whirlybird head, kg
m2 = 1                 # Mass of counter weight, kg
d = 0.178              # Crossbar Lengh, m
h = 0.65               # Post Height, m
r = 0.12               # Propeller Radius, m
Jx = 0.0047            # , kg-m**2
Jy = 0.0014            # , kg-m**2
Jz = 0.0041            # , kg-m**2
km = 5.8               # , N/PWM

# Initial Conditions
phi0 =   0.0*np.pi/180 # ,rads
theta0 = 0.0*np.pi/180 # ,rads
psi0 = 0.0*np.pi/180   # ,rads
phidot0 = 0.0          # ,rads/s
thetadot0 = 0.0        # ,rads/s
psidot0 = 0.0          # ,rads/s

# Simulation parameters
Ts = 0.01                # Time step
sigma = 0.05

F_e = (m1 - m2*(L2/L1))*g
phimax = 45.0 *np.pi / 180.0
F_max = 50.0
tau_max = 50.0
thetamax = 60.0*np.pi / 180.0

pwmConversionFactor = 10.875
pwm_e = .48
####################################################
#     PD Control: Design Time Strategy
####################################################



#==================================================
#                 Longitudinal
#==================================================
# Flong_max = 10.85

# Open Loop
# kp*b0/(S**2 + kd*b*S + kp*b)
th_b0 = (l1/(m1*L1**2+m2*l2**2+Jy))
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
th_kp = (th_alpha0-th_a0)/th_b0
th_kd = (th_alpha1-th_a1)/th_b0

th_ki = 0.1
th_windup = 2

#==================================================
#                 Lateral
#==================================================


#---------------------------------------------------
#                    Inner Loop
#---------------------------------------------------
#Flat_max = ?

#---------------------------------------------------
#                    Inner Loop: Roll
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
phi_tr = 0.2           # Rise time, s
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
#                  Outer Loop         
#---------------------------------------------------
M = 10.0    # bandwidth separation factor
# Open Loop
# kp*b0/(S**2 + kd*b*S + kp*b)
psi_b0 = (l1*F_e/(m1*l1**2+m2*l2**2+Jz))
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

print("th_kp: ", th_kp)
print("th_kd: ", th_kd)
print("phi_kp: ", phi_kp)
print("phi_kd: ", phi_kd)
print("psi_kp: ", psi_kp)
print("psi_kd: ", psi_kd)
print("F_max: ", F_max)
print("tau_max: ", tau_max)

#################################################
#          Uncertainty Parameters
#################################################
UNCERTAINTY_PARAMETERS = False
if UNCERTAINTY_PARAMETERS:
	alpha = 0.2;                                    # uncertainty parameter
	l1 = 0.85*(1+2*alpha*np.random.rand()-alpha)    # Distance between fulcrum 
													# and the head , m
	l2 = 0.3048*(1+2*alpha*np.random.rand()-alpha)  # Distance between the fulcrum
													# and end of Whirlybird's tail, m
	m1 = 0.891*(1+2*alpha*np.random.rand()-alpha)   # Mass of whirlybird head, kg
	m2 = 1*(1+2*alpha*np.random.rand()-alpha)       # Mass of counter weight, kg
	d = 0.178*(1+2*alpha*np.random.rand()-alpha)    # Crossbar Lengh, m
	h = 0.65*(1+2*alpha*np.random.rand()-alpha)     # Post Height, m
	r = 0.12*(1+2*alpha*np.random.rand()-alpha)     # Propeller Radius, m
	Jx = 0.0047*(1+2*alpha*np.random.rand()-alpha)  # , kg-m**2
	Jy = 0.0014*(1+2*alpha*np.random.rand()-alpha)  # , kg-m**2
	Jz = 0.0041*(1+2*alpha*np.random.rand()-alpha)  # , kg-m**2

