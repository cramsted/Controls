import numpy as np
import param as P

def getForces(y_r,y):
  # y_r is the referenced input
  # y is the current state
  h = y[4]
  h_r = y_r[0]
  hdot = y[5]

  F_t = P.kp * (h_r - h) - P.kd * hdot # Calculate the force output
  F = P.F_e + F_t
  tau = 0

  return [F, tau]
