import time
import sys
import numpy as np
import matplotlib.pyplot as plt
import param as P
# from slider_input import Sliders
from signal_generator import mySignal
from sim_plot import plotGenerator
import contorlPD as ctrl

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

plotGen = plotGenerator()
simAnimation = WhirlybirdAnimation()  # Instantiate Animate class
dynam = WhirlybirdDynamics()			# Instantiate Dynamics class
square_wave = mySignal(A=15,f=.02)

t = t_start			   # Declare time variable to keep track of simulation time elapsed

while t < t_end:
    # Get referenced inputs from signal generators
	ref_input = [mySignal.square_wave(t)]

	# The dynamics of the model will be propagated in time by t_elapse
	# at intervals of t_Ts.
	t_temp = t +t_elapse
	while t < t_temp:

		states = dynam.Outputs()             # Get current states
		u = ctrl.getForces(ref_input,states) # Calculate the forces
		u_converted = convertForces(u)
		dynam.propagateDynamics(u_converted)           # Propagate the dynamics of the model in time
		t = round(t +t_Ts,2)                 # Update time elapsed

	plt.figure(simAnimation.fig.number) # Switch current figure to animation figure
	simAnimation.drawShapes(          # Update animation with current user input
		dynam.Outputs(), [0.0])
	plt.pause(0.0001)

	# Organizes the new data to be passed to plotGen
	new_data = [[ref_input[0],states[0]],
				# [states[1]],
			    [u[0]]]
	plotGen.updateDataHistory(t, new_data)

	# plt.figure(plotGen.fig.number)		# Switch current figure to plotGen figure
	# plotGen.update_plots()              # Update the plot
	# plt.pause(0.0001)

	# time.sleep(t_pause)

	# plt.ion()						   # Make plots interactive
	# plt.figure(user_input.fig.number)   # Switch current figure to user_input figure
	# plt.pause(0.001)					# Pause the simulation to detect user input
	#
	# # Thy dynamics of the model will be propagated in time by t_elapse
	# # at intervals of t_Ts
	# t_temp = t + t_elapse
	#
	# while t < t_temp:
	# 	F = convertForces(user_input.getInputValues())
	# 	dynam.propagateDynamics(F)			# Propogate the dynamics of the model in time
	# 	t = t+t_elapse					  # Update time elapsed
	#
	# plt.figure(simAnimation.fig.number) # Switch current figure to animation figure
	# simAnimation.drawWhirlybird(		# Update animation with current user input
	# 	dynam.Outputs())
	# # time.sleep(t_pause)

# This function takes in the output u from the sliders F and tau
# and convert them into fl and fr as a list
