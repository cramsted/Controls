import numpy as np
import param as P

def getForces(y_r,y):
  # y_r is the referenced input
  # y is the current state
  theta = y[1]
  theta_r = y_r[0]
  thetadot = y[4]

  F_t = P.kp * (theta_r - theta) - P.kd * thetadot # Calculate the force output
  # F_e value is not the actual F_e value, but we couldn't multiply it by cos(theta)
  # in the params file, so we do it here
  F = P.F_e * np.cos(theta) + F_t
  tau = 0

  return [F, tau]
