from quad_copter import QuadCopter
from typing import List
import yaml
import numpy as np
import time
import os
from datetime import datetime
from utils.transformations import RPY2Rot
from animation import animate

class Simulation:
    def __init__(self, quads: List[QuadCopter], global_params="config/global_params.yaml") -> None:
        with open(global_params, 'r') as file:
            global_params = yaml.safe_load(file)

        self.t = global_params['t']
        self.g = global_params['g']
        self.dt = global_params['dt']
        self.t_final = global_params['t_final']

        self.quads = quads

        num_time_steps = int((self.t_final/self.dt)+1)
        self.t_all = np.zeros(num_time_steps)
        self.state_all = np.zeros((num_time_steps, len(self.quads), len(self.quads[0].state_vector)))
        self.linear_displacement_all = np.zeros((num_time_steps, len(self.quads), len(self.quads[0].linear_displacement)))
        self.linear_velocity_all = np.zeros((num_time_steps, len(self.quads), len(self.quads[0].linear_velocity)))
        self.angular_displacement_all = np.zeros((num_time_steps, len(self.quads), len(self.quads[0].angular_displacement)))
        self.angular_velocity_all = np.zeros((num_time_steps, len(self.quads), len(self.quads[0].angular_velocity)))        


    def save(self, save_path):
        os.makedirs(save_path, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"run_{timestamp}.npz"
        file_path = os.path.join(save_path, filename)

        arm_lengths = [quad.arm_length for quad in self.quads]

        save_dict = {
            "arm_lengths": arm_lengths,
            "t_all": self.t_all,
            "state_all": self.state_all
        }
        np.savez_compressed(file_path, **save_dict)
        print(f"Data saved successfully to {file_path}")


    def run(self, save_path="saved_runs", show_animation=True):
        start_time = time.time()
        i = 0
        while self.t < self.t_final:
            self.t_all[i] = self.t

            for num, quad in enumerate(self.quads):
                self.state_all[i, num, :] = quad.state_vector.squeeze(-1)
                self.linear_displacement_all[i, num, :] = quad.linear_displacement.squeeze(-1)
                self.linear_velocity_all[i, num, :] = quad.linear_velocity.squeeze(-1)
                self.angular_displacement_all[i, num, :] = quad.angular_displacement.squeeze(-1)
                self.angular_velocity_all[i, num, :] = quad.angular_velocity.squeeze(-1)

                quad.update()

            self.t += self.dt
            i += 1

        end_time = time.time()
        print("Simulated {:.2f}s in {:.6f}s.".format(self.t, end_time - start_time))

        if save_path:
            folder_list = os.listdir(save_path)
            max_num = 0
            for folder_name in folder_list:
                number = int(folder_name.split('.')[0].split('_')[1])
                max_num = max(number, max_num)

            new_folder_name = f"run_{max_num+1}"
            new_folder_path = os.path.join(save_path, new_folder_name)
            self.save(new_folder_path)

        if show_animation:
            arm_lengths = [quad.arm_length for quad in self.quads]
            new_folder_path = new_folder_path if save_path else None
            animate(self.t_all, self.state_all, arm_lengths, save_path=new_folder_path)
 


if __name__ == "__main__":
    my_quad = QuadCopter()
    my_quad2 = QuadCopter("config/quad2_params.yaml")
    quad_list = [my_quad, my_quad2]
    sim = Simulation(quad_list)

    sim.run()