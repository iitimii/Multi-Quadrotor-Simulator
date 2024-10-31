import numpy as np

def RPY2Rot(phi, theta, psi) -> np.ndarray:
    R_3_psi = np.array([[np.cos(psi), np.sin(psi), 0.0],
                        [-np.sin(psi), np.cos(psi), 0.0],
                        [0.0, 0.0, 1.0]])
    
    R_2_theta = np.array([[np.cos(theta), 0.0, -np.sin(theta)],
                          [0.0, 1.0, 0.0],
                          [np.sin(theta), 0.0, np.cos(theta)]])  # Corrected row
    
    R_1_phi = np.array([[1.0, 0.0, 0.0],
                        [0.0, np.cos(phi), np.sin(phi)],
                        [0.0, -np.sin(phi), np.cos(phi)]])
    
    bRi = np.dot(np.dot(R_1_phi, R_2_theta), R_3_psi)
    return bRi

def Re_param_matrix(phi, theta):
    sec_theta = 1 / np.cos(theta)
    R = np.array([[1.0, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                  [0.0, np.cos(phi), -np.sin(phi)],
                  [0.0, np.sin(phi) * sec_theta, np.cos(phi) * sec_theta]])
    return R
