# Whirlybird Parameter File
import numpy as np

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
Ts = 0.01

# Initial Conditions
phi0 = 0.0*np.pi/180    # Pitch of Whirlybird relative to the ground, rads
phidot0 = 0.0           # Derivative of the pitch
theta0 = 0.0*np.pi/180  # Roll of Whirlybird relative to the ground, rads
thetadot0 = 0.0         # Derivative of the roll
psi0 = 0.0*np.pi/180    # Yaw of Whirlybird relative to the ground, rads
psidot0 = 0.0           # Derivative of the yaw
