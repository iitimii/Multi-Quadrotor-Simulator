import os
import numpy as np
import yaml
from datetime import datetime
from matplotlib import animation
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

from utils.transformations import RPY2Rot



def animate(t_all, state_all, arm_lengths, save_path=None, num_frames=8):
    with open("config/global_params.yaml", 'r') as file:
        global_params = yaml.safe_load(file)

    x = state_all[:,:,0]
    y = state_all[:,:,1]
    z = state_all[:,:,2]
    linear_displacement_all = state_all[:, :, :3]
    angular_displacement_all = state_all[:, :, 6:9]

    if (global_params["orient"] == "NED"):
        z = -z

    fig = plt.figure()
    ax = p3.Axes3D(fig,auto_add_to_figure=False)
    fig.add_axes(ax)
    line1, = ax.plot([], [], [], lw=2, color='red')
    line2, = ax.plot([], [], [], lw=2, color='blue')
    line3, = ax.plot([], [], [], '--', lw=1, color='blue')

    extraEachSide = 0.5
    maxRange = 0.5*np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() + extraEachSide
    mid_x = 0.5*(x.max()+x.min())
    mid_y = 0.5*(y.max()+y.min())
    mid_z = 0.5*(z.max()+z.min())

    ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
    ax.set_xlabel('X')
    if (global_params["orient"] == "NED"):
        ax.set_ylim3d([mid_y+maxRange, mid_y-maxRange])
    elif (global_params["orient"] == "ENU"):
        ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
    ax.set_ylabel('Y')
    ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])
    ax.set_zlabel('Altitude')

    titleTime = ax.text2D(0.05, 0.95, "", transform=ax.transAxes)


    def update_lines(i):
        time = t_all[i*num_frames]
        linear_displacement = linear_displacement_all[i*num_frames, 0]

        x = linear_displacement[0]
        y = linear_displacement[1]
        z = linear_displacement[2]

        x_from0 = linear_displacement_all[0:i*num_frames, 0, 0]
        y_from0 = linear_displacement_all[0:i*num_frames, 0, 1]
        z_from0 = linear_displacement_all[0:i*num_frames, 0, 2]

        angular_displacement = angular_displacement_all[i*num_frames, 0]

        dxm = arm_lengths[0]
        dym = arm_lengths[0]
        dzm = 0.05

        if (global_params["orient"] == "NED"):
            z = -z
            z_from0 = -z_from0

        R = RPY2Rot(angular_displacement[0], angular_displacement[1], angular_displacement[2])
        motorPoints = np.array([[dxm, -dym, dzm], [0, 0, 0], [dxm, dym, dzm], [-dxm, dym, dzm], [0, 0, 0], [-dxm, -dym, dzm]])
        motorPoints = np.dot(R, np.transpose(motorPoints))
        motorPoints[0,:] += x 
        motorPoints[1,:] += y 
        motorPoints[2,:] += z 
        
        line1.set_data(motorPoints[0,0:3], motorPoints[1,0:3])
        line1.set_3d_properties(motorPoints[2,0:3])
        line2.set_data(motorPoints[0,3:6], motorPoints[1,3:6])
        line2.set_3d_properties(motorPoints[2,3:6])
        line3.set_data(x_from0, y_from0)
        line3.set_3d_properties(z_from0)
        titleTime.set_text(u"Time = {:.2f} s".format(time))
        
        return line1, line2
    
    def ini_plot():
        line1.set_data(np.empty([1]), np.empty([1]))
        line1.set_3d_properties(np.empty([1]))
        line2.set_data(np.empty([1]), np.empty([1]))
        line2.set_3d_properties(np.empty([1]))
        line3.set_data(np.empty([1]), np.empty([1]))
        line3.set_3d_properties(np.empty([1]))

        return line1, line2, line3
    

    line_ani = animation.FuncAnimation(fig, update_lines, init_func=ini_plot, frames=len(t_all[0:-2:num_frames]), interval=(global_params["dt"]*1000*num_frames), blit=False)
    if (save_path):
        os.makedirs(save_path, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        line_ani.save(f'{save_path}/animation_{timestamp}.gif', dpi=100, writer='imagemagick', fps=30)

    plt.show()
    return line_ani


def replay_from_npz(run_path):
    load_dict =  np.load(run_path)
    t_all = load_dict["t_all"]
    state_all = load_dict["state_all"]
    arm_lengths = load_dict["arm_lengths"]

    animate(t_all, state_all, arm_lengths)



# This is the code for displaying multiple quads, but issue is that xlim arent set well
# import os
# import numpy as np
# import yaml
# from datetime import datetime
# from matplotlib import animation
# import matplotlib.pyplot as plt
# import mpl_toolkits.mplot3d.axes3d as p3

# from utils.transformations import RPY2Rot

# def animate(t_all, state_all, arm_lengths, save_path=None, num_frames=8):
#     with open("config/global_params.yaml", 'r') as file:
#         global_params = yaml.safe_load(file)

#     num_drones = state_all.shape[1]
#     x = state_all[:, :, 0]
#     y = state_all[:, :, 1]
#     z = state_all[:, :, 2]
#     linear_displacement_all = state_all[:, :, :3]
#     angular_displacement_all = state_all[:, :, 6:9]

#     fig = plt.figure()
#     ax = p3.Axes3D(fig, auto_add_to_figure=False)
#     fig.add_axes(ax)

#     lines1 = [ax.plot([], [], [], lw=2, color='red')[0] for _ in range(num_drones)]
#     lines2 = [ax.plot([], [], [], lw=2, color='blue')[0] for _ in range(num_drones)]
#     lines3 = [ax.plot([], [], [], '--', lw=1, color='blue')[0] for _ in range(num_drones)]

#     extraEachSide = 0.5
#     maxRange = 0.5 * np.array([x.max() - x.min(), y.max() - y.min(), z.max() - z.min()]).max() + extraEachSide
#     mid_x = 0.5 * (x.max() + x.min())
#     mid_y = 0.5 * (y.max() + y.min())
#     mid_z = 0.5 * (z.max() + z.min())

#     ax.set_xlim3d([mid_x - maxRange, mid_x + maxRange])
#     ax.set_xlabel('X')
#     if global_params["orient"] == "NED":
#         ax.set_ylim3d([mid_y + maxRange, mid_y - maxRange])
#     else:
#         ax.set_ylim3d([mid_y - maxRange, mid_y + maxRange])
#     ax.set_ylabel('Y')
#     ax.set_zlim3d([mid_z - maxRange, mid_z + maxRange])
#     ax.set_zlabel('Altitude')

#     titleTime = ax.text2D(0.05, 0.95, "", transform=ax.transAxes)

#     def update_lines(i):
#         time = t_all[i * num_frames]
#         titleTime.set_text(f"Time = {time:.2f} s")

#         for d in range(num_drones):
#             # Get current displacement and orientation for the drone
#             linear_displacement = linear_displacement_all[i * num_frames, d].copy()
#             angular_displacement = angular_displacement_all[i * num_frames, d]

#             # Apply NED adjustment if needed
#             if global_params["orient"] == "NED":
#                 linear_displacement[2] = -linear_displacement[2]  # invert z-axis

#             # Path from the start up to current frame
#             x_from0 = linear_displacement_all[0:i * num_frames, d, 0]
#             y_from0 = linear_displacement_all[0:i * num_frames, d, 1]
#             z_from0 = linear_displacement_all[0:i * num_frames, d, 2]

#             # Adjust the path history for NED
#             if global_params["orient"] == "NED":
#                 z_from0 = -z_from0

#             # Compute rotation and motor points for each drone
#             R = RPY2Rot(angular_displacement[0], angular_displacement[1], angular_displacement[2])
#             dxm, dym, dzm = arm_lengths[d], arm_lengths[d], 0.05
#             motorPoints = np.array([[dxm, -dym, dzm], [0, 0, 0], [dxm, dym, dzm], 
#                                     [-dxm, dym, dzm], [0, 0, 0], [-dxm, -dym, dzm]])
#             motorPoints = np.dot(R, motorPoints.T)

#             # Apply translation (adjust for NED if necessary)
#             motorPoints[0, :] += linear_displacement[0]
#             motorPoints[1, :] += linear_displacement[1]
#             motorPoints[2, :] += linear_displacement[2]

#             # Update line objects for each drone
#             lines1[d].set_data(motorPoints[0, 0:3], motorPoints[1, 0:3])
#             lines1[d].set_3d_properties(motorPoints[2, 0:3])
#             lines2[d].set_data(motorPoints[0, 3:6], motorPoints[1, 3:6])
#             lines2[d].set_3d_properties(motorPoints[2, 3:6])
#             lines3[d].set_data(x_from0, y_from0)
#             lines3[d].set_3d_properties(z_from0)

#         return lines1 + lines2 + lines3

#     def ini_plot():
#         for line in lines1 + lines2 + lines3:
#             line.set_data([], [])
#             line.set_3d_properties([])

#         return lines1 + lines2 + lines3

#     line_ani = animation.FuncAnimation(fig, update_lines, init_func=ini_plot,
#                                        frames=len(t_all[0:-2:num_frames]),
#                                        interval=(global_params["dt"] * 1000 * num_frames),
#                                        blit=False)
#     if save_path:
#         os.makedirs(save_path, exist_ok=True)
#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         line_ani.save(f'{save_path}/animation_{timestamp}.gif', dpi=80, writer='imagemagick', fps=30)

#     plt.show()
#     return line_ani

# def replay_from_npz(run_path):
#     load_dict = np.load(run_path)
#     t_all = load_dict["t_all"]
#     state_all = load_dict["state_all"]
#     arm_lengths = load_dict["arm_lengths"]

#     animate(t_all, state_all, arm_lengths)
