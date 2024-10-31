import numpy as np
import yaml
from controllers.pid_controller import PIDController
from controllers.attitude_controller import AttittudeController
from utils.transformations import RPY2Rot, Re_param_matrix

class QuadCopter:
    def __init__(self, quad_params="config/quad1_params.yaml", 
                 global_params="config/global_params.yaml") -> None:

        with open(quad_params, 'r') as file:
            self.quad_params = yaml.safe_load(file)

        with open(global_params, 'r') as file:
            self.global_params = yaml.safe_load(file)

        # Global
        self.t = self.global_params['t']
        self.g = self.global_params['g']
        self.dt = self.global_params['dt']
        self.t_final = self.global_params['t_final']

        # Physical
        self.mass = self.quad_params['mass']
        self.arm_length = self.quad_params['arm_length']
        self.motor_height = self.quad_params['motor_height']

        # Inertia
        self.ixx = self.quad_params['ixx']
        self.iyy = self.quad_params['iyy']
        self.izz = self.quad_params['izz']
        self.inertia_matrix = np.diag([self.ixx, self.iyy, self.izz])

        self.x, self.y, self.z = self.quad_params['x'], self.quad_params['y'], self.quad_params['z']
        self.vx, self.vy, self.vz = self.quad_params['vx'], self.quad_params['vy'], self.quad_params['vz']

        self.phi, self.theta, self.psi = self.quad_params['phi'], self.quad_params['theta'], self.quad_params['psi']
        self.p, self.q, self.r = self.quad_params['p'], self.quad_params['q'], self.quad_params['r']

        # State vectors
        self.linear_displacement = np.array([self.x, self.y, self.z]).reshape(3, 1)
        self.angular_displacement = np.array([self.phi, self.theta, self.psi]).reshape(3, 1)
        self.linear_velocity = np.array([self.vx, self.vy, self.vz]).reshape(3, 1)
        self.angular_velocity = np.array([self.p, self.q, self.r]).reshape(3, 1)

        self.state_vector = np.concatenate([self.linear_displacement, self.linear_velocity, self.angular_displacement, self.angular_velocity], axis=0).reshape(12, 1)
        self.state_dot_vector = np.zeros((12, 1))

        # Inputs
        self.T_sigma, self.M1, self.M2, self.M3 = self.quad_params['T_sigma'], self.quad_params['M1'], self.quad_params['M2'], self.quad_params['M3']
        self.moments = np.array([self.M1, self.M2, self.M3]).reshape(3, 1)
        self.input_vector = np.array([self.T_sigma, self.M1, self.M2, self.M3]).reshape(4, 1)

        # Body structure
        self.body = np.array([[self.arm_length, 0, 0, 1],
                                      [0, -self.arm_length, 0, 1],
                                      [-self.arm_length, 0, 0, 1],
                                      [0, self.arm_length, 0, 1],
                                      [0, 0, 0, 1],  # Center
                                      [0, 0, -0.15, 1]])  # Payload

        # Controllers
        self.phi_pid_controller = PIDController(kp=self.quad_params['kp_phi'], ki=self.quad_params['ki_phi'], kd=self.quad_params['kd_phi'], dt=self.dt)
        self.theta_pid_controller = PIDController(kp=self.quad_params['kp_theta'], ki=self.quad_params['ki_theta'], kd=self.quad_params['kd_theta'], dt=self.dt)
        self.psi_pid_controller = PIDController(kp=self.quad_params['kp_psi'], ki=self.quad_params['ki_psi'], kd=self.quad_params['kd_psi'], dt=self.dt)
        self.zdot_pid_controller = PIDController(kp=self.quad_params['kp_zdot'], ki=self.quad_params['ki_zdot'], kd=self.quad_params['kd_zdot'], dt=self.dt)

        # self.attitude_controller = AttittudeController()

    def update_state_dot_vector(self):
        # Rotation matrix from body to inertial frame
        bRi = RPY2Rot(self.phi, self.theta, self.psi)
        R = bRi.T

        x_y_z_dot_dot = (1/self.mass) * (np.array([0.0, 0.0, self.mass * self.g]).reshape(3, 1) + np.dot(R, self.T_sigma * np.array([0, 0, -1]).reshape(3, 1)))

        R_ = Re_param_matrix(self.phi, self.theta)

        angular_acceleration = np.linalg.inv(self.inertia_matrix).dot(self.moments - np.cross(self.angular_velocity.T, self.inertia_matrix.dot(self.angular_velocity).T).T)

        return np.concatenate([self.linear_velocity, x_y_z_dot_dot, np.dot(R_, self.angular_velocity), angular_acceleration], axis=0).reshape(12, 1)

    
    def update_state(self):
        self.t += self.dt
        self.state_dot_vector = self.update_state_dot_vector()
        self.state_vector += (self.state_dot_vector * self.dt)

        self.linear_displacement = self.state_vector[:3]
        self.linear_velocity = self.state_vector[3:6]
        self.angular_displacement = self.state_vector[6:9]
        self.angular_velocity = self.state_vector[9:]

        self.x, self.y, self.z = self.state_vector[0, 0], self.state_vector[1, 0], self.state_vector[2, 0]
        self.vx, self.vy, self.vz = self.state_vector[3, 0], self.state_vector[4, 0], self.state_vector[5, 0]
        self.phi, self.theta, self.psi = self.state_vector[6, 0], self.state_vector[7, 0], self.state_vector[8, 0]
        self.p, self.q, self.r = self.state_vector[9, 0], self.state_vector[10, 0], self.state_vector[11, 0]

    def update(self):
        self.update_state()

if __name__ == "__main__":
    print("QuadCopter Class!")
    my_quad = QuadCopter()
    print(my_quad.linear_displacement.flatten())
    for _ in range(100):
        my_quad.update()
        print(my_quad.linear_displacement.flatten())

