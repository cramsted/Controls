import time
import sys
import numpy as np
import matplotlib.pyplot as plt
import param as P
from slider_input import Sliders

# The Animation.py file is kept in the parent directory,
# so the parent directory path needs to be added.
sys.path.append('..')
from dynamics import WhirlybirdDynamics
from Animation import WhirlybirdAnimation

def convertForces(u):
		fl = .5*u[0]+(1/(2*P.d))*u[1]	   # This calculates fl from f and tau (from the sliders)
		fr = .5*u[0]-(1/(2*P.d))*u[1]	   # This calculates fr from f and tau (from the sliders)
		# import pdb; pdb.set_trace()
		propagateDerivativeIn = [fl,fr]	 # Store the forces in a list
		return propagateDerivativeIn		# The list will be passed in to propogateDerivative


t_start = 0.0   # Start time of simulation
t_end = 20.0	# End time of simulation
t_Ts = P.Ts	 # Simulation time step
t_elapse = 0.01 # Simulation time elapsed between each iteration
t_pause = 0.01  # Pause between each iteration

user_input = Sliders()			  # Instantiate Sliders class
simAnimation = WhirlybirdAnimation()  # Instantiate Animate class
dynam = WhirlybirdDynamics()			# Instantiate Dynamics class

t = t_start			   # Declare time variable to keep track of simulation time elapsed

while t < t_end:
	plt.ion()						   # Make plots interactive
	plt.figure(user_input.fig.number)   # Switch current figure to user_input figure
	plt.pause(0.001)					# Pause the simulation to detect user input

	# Thy dynamics of the model will be propagated in time by t_elapse
	# at intervals of t_Ts
	t_temp = t + t_elapse

	while t < t_temp:
		F = convertForces(user_input.getInputValues())
		dynam.propagateDynamics(F)			# Propogate the dynamics of the model in time
		t = t+t_elapse					  # Update time elapsed

	plt.figure(simAnimation.fig.number) # Switch current figure to animation figure
	simAnimation.drawWhirlybird(		# Update animation with current user input
		dynam.Outputs())
	# time.sleep(t_pause)

# This function takes in the output u from the sliders F and tau
# and convert them into fl and fr as a list
