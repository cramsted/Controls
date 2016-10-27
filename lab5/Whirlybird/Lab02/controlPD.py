import numpy as np
import param as P

def getForces(y_r,y):
  # y_r is the referenced input
  # y is the current state
  theta = y[1]
  psi = y[2]
  phi = y[0]
  theta_r = y_r[0]
  psi_r = y_r[0]
  thetadot = y[4]
  psidot = y[5]
  phidot = y[3]

  F_t = P.th_kp * (theta_r - theta) - P.th_kd * thetadot # Calculate the force output
  # F_e value is not the actual F_e value, but we couldn't multiply it by cos(theta)
  # in the params file, so we do it here
  F = P.F_e * np.cos(theta)  + F_t

  phi_r = P.psi_kp * (psi_r - psi) - P.psi_kd * psidot
  tau = P.phi_kp * (phi_r - phi) - P.phi_kd * phidot
  # if F > P.F_max:
  #     F = P.F_max
  return [F, tau]
