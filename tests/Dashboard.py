from tests.ZRobot import ZRobot
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from time import time, sleep
import numpy as np
import math
from tests.aw_utils import sleep_until_end_of_cycle

# helper function that controls the response of each primary color as we move along the color map (see Color_from_map below)
def color_map_transform(location_in_quantile, exp1, exp2):
    return (1 - abs(location_in_quantile - 1) ** exp1) ** exp2
# end function color_map_transform

# for a given value in [0,1], return a color as [r, g, b]
def color_from_map(val):
    exp1 = 2.0
    exp2 = 1.2
    if val < 0.2:  # first quintile of range
        # green goes from 0 to 1 across quintile
        location_in_quintile = val / 0.2
        r = 0.0
        g = color_map_transform(location_in_quintile, exp1, exp2)
        b = 1.0
    elif val < 0.4:  # second quintile of range
        # blue goes from 1 to 0 across quintile
        location_in_quintile = (val - 0.2) / 0.2
        r = 0.0
        g = 1.0
        b = color_map_transform(1 - location_in_quintile, exp1, exp2)
    elif val < 0.6:  # third quintile of range
        # red goes from 0 to 0.8 across quintile
        location_in_quintile = (val - 0.4) / 0.2
        r = 0.8 * color_map_transform(location_in_quintile, exp1, exp2)
        g = 1.0
        b = 0.0
    elif val < 0.8:  # fourth quintile of range
        # green goes from 1 to 0 across quintile
        location_in_quintile = (val - 0.6) / 0.2
        r = 0.8;
        g = color_map_transform(1 - location_in_quintile, exp1, exp2)
        b = 0.0
    else:  # fifth quintile of range
        # blue goes from 0 to 1 across quintile
        location_in_quintile = (val - 0.8) / 0.2
        r = 0.8
        g = 0.0
        b = color_map_transform(location_in_quintile, exp1, exp2)

    return [r, g, b]
# end function color_from_map

class Dashboard():
    def __init__(self):
        self.bot = ZRobot()
    # end constructor

    # run the dashboard
    def run(self):
        # initialize dashboard window
        dash = tk.Tk()
        dash.config(background='white')
        dash.geometry("950x320")
        dash.title("Dashboard")

        # set up position labels
        xpos_lbl = tk.StringVar()
        ypos_lbl = tk.StringVar()
        zpos_lbl = tk.StringVar()
        w = 6
        h = 1
        lbl = tk.Label(dash, text="Position (m)")
        lbl.grid(row=0, column=0, columnspan=2)
        lbl = tk.Label(dash, text="pos x", width=w, height=h, anchor="w", padx=10)
        lbl.grid(row=1, column=0)
        lbl = tk.Label(dash, text="pos y", width=w, height=h, anchor="w", padx=10)
        lbl.grid(row=2, column=0)
        lbl = tk.Label(dash, text="pos z", width=w, height=h, anchor="w", padx=10)
        lbl.grid(row=3, column=0)
        lbl = tk.Label(dash, textvariable=xpos_lbl, width=w, height=h, anchor="e", padx=10)
        lbl.grid(row=1, column=1)
        lbl = tk.Label(dash, textvariable=ypos_lbl, width=w, height=h, anchor="e", padx=10)
        lbl.grid(row=2, column=1)
        lbl = tk.Label(dash, textvariable=zpos_lbl, width=w, height=h, anchor="e", padx=10)
        lbl.grid(row=3, column=1)

        # set up orientation labels
        bear_lbl = tk.StringVar()
        pitch_lbl = tk.StringVar()
        roll_lbl = tk.StringVar()
        grade_lbl = tk.StringVar()
        w = 6
        h = 1
        lbl = tk.Label(dash, text="Orientation (deg)")
        lbl.grid(row=4, column=0, columnspan=2)
        lbl = tk.Label(dash, text="bearing", width=w, height=h, anchor="w", padx=10)
        lbl.grid(row=5, column=0)
        lbl = tk.Label(dash, text="pitch", width=w, height=h, anchor="w", padx=10)
        lbl.grid(row=6, column=0)
        lbl = tk.Label(dash, text="roll", width=w, height=h, anchor="w", padx=10)
        lbl.grid(row=7, column=0)
        lbl = tk.Label(dash, text="grade", width=w, height=h, anchor="w", padx=10)
        lbl.grid(row=8, column=0)
        lbl = tk.Label(dash, textvariable=bear_lbl, width=w, height=h, anchor="e", padx=10)
        lbl.grid(row=5, column=1)
        lbl = tk.Label(dash, textvariable=pitch_lbl, width=w, height=h, anchor="e", padx=10)
        lbl.grid(row=6, column=1)
        lbl = tk.Label(dash, textvariable=roll_lbl, width=w, height=h, anchor="e", padx=10)
        lbl.grid(row=7, column=1)
        lbl = tk.Label(dash, textvariable=grade_lbl, width=w, height=h, anchor="e", padx=10)
        lbl.grid(row=8, column=1)

        ### rate-of-change column
        speed_lbl = tk.StringVar()
        acceleration_lbl = tk.StringVar()
        yaw_rate_lbl = tk.StringVar()
        pitch_rate_lbl = tk.StringVar()
        roll_rate_lbl = tk.StringVar()
        grade_rate_lbl = tk.StringVar()
        wobble_lbl = tk.StringVar()
        title_width = 20
        readout_width = 6
        h = 1
        col_num = 2
        lbl = tk.Label(dash, text="Speed (m/s)", width=title_width, height=h)
        lbl.grid(row=0, column=col_num)
        lbl = tk.Label(dash, textvariable=speed_lbl, width=readout_width, height=h, anchor="e")
        lbl.grid(row=1, column=col_num)
        lbl = tk.Label(dash, text="Acceleration (m/s/s)", width=title_width, height=h)
        lbl.grid(row=2, column=col_num)
        lbl = tk.Label(dash, textvariable=acceleration_lbl, width=readout_width, height=h, anchor="e")
        lbl.grid(row=3, column=col_num)
        lbl = tk.Label(dash, text="Rate of Change (deg/s)", width=title_width, height=h)
        lbl.grid(row=4, column=col_num)
        lbl = tk.Label(dash, textvariable=yaw_rate_lbl, width=readout_width, height=h, anchor="e")
        lbl.grid(row=5, column=col_num)
        lbl = tk.Label(dash, textvariable=pitch_rate_lbl, width=readout_width, height=h, anchor="e")
        lbl.grid(row=6, column=col_num)
        lbl = tk.Label(dash, textvariable=roll_rate_lbl, width=readout_width, height=h, anchor="e")
        lbl.grid(row=7, column=col_num)
        lbl = tk.Label(dash, textvariable=grade_rate_lbl, width=readout_width, height=h, anchor="e")
        lbl.grid(row=8, column=col_num)

        ### height/wobble column
        title_width = 20
        readout_width = 6
        h = 1
        col_num = 3
        # set up height-above-ground readout
        height_above_ground_lbl = tk.StringVar()
        lbl = tk.Label(dash, text="Height Above Ground (m)", width=title_width, height=h)
        lbl.grid(row=0, column=col_num)
        lbl = tk.Label(dash, textvariable=height_above_ground_lbl, width=readout_width, height=h, anchor="e", padx=10)
        lbl.grid(row=1, column=col_num)
        # set up wobble readout
        wobble_lbl = tk.StringVar()
        lbl = tk.Label(dash, text="Wobble", width=title_width, height=h)
        lbl.grid(row=2, column=col_num)
        lbl = tk.Label(dash, textvariable=wobble_lbl, width=readout_width, height=h, anchor="e")
        lbl.grid(row=3, column=col_num)

        ### LIDAR
        lbl = tk.Label(dash, text="LIDAR")
        lbl.grid(row=0, column=4, columnspan=3)

        # set up LIDAR color plot
        fig = Figure(figsize=(2.5, 2.5), frameon=False)
        lidar_ax = fig.add_subplot(111)
        lidar_ax.spines['top'].set_visible(False)
        lidar_ax.spines['right'].set_visible(False)
        lidar_ax.spines['bottom'].set_visible(False)
        lidar_ax.spines['left'].set_visible(False)
        lidar_ax.get_xaxis().set_visible(False)
        lidar_ax.get_yaxis().set_visible(False)
        lidar_plot = FigureCanvasTkAgg(fig, master=dash)
        lidar_plot.get_tk_widget().grid(row=1, column=4, rowspan=7)
        # NEW:
        angles = [2 * math.pi * 5 * x / 360 for x in range(72)]
        radii = np.concatenate((np.linspace(1, 0.75, 6), np.linspace(0.6, 0.35, 6)))
        lidar_matrix_size = (12, 72)
        num_lidar_values = lidar_matrix_size[0] * lidar_matrix_size[1]
        lidar_plt_x_vals = np.zeros(lidar_matrix_size)
        lidar_plt_y_vals = np.zeros(lidar_matrix_size)
        for row_num in range(radii.size):
            r = radii[row_num]
            lidar_plt_x_vals[row_num,:] = [r * math.sin(a) for a in angles]
            lidar_plt_y_vals[row_num,:] = [r * math.cos(a) for a in angles]
        lidar_colors = np.ndarray(lidar_matrix_size, dtype=object)
        ''' 
        # OLD
        angles = [2 * math.pi * 5 * x / 360 for x in range(72)]
        lidar_plt_x_vals = [math.sin(a) for a in angles]
        lidar_plt_y_vals = [math.cos(a) for a in angles]
        '''

        # set up labels for LIDAR colorbar
        lidar_range = 300  # 2FIX: THIS USED TO BE AVAILABLE IN THE ROBOT'S SENSOR INFO...
        w = 3
        h = 1
        lbl = tk.Label(dash, text="{}".format(round(lidar_range)), width=w, height=h, anchor="se", padx=10)
        lbl.grid(row=1, column=5)
        lbl = tk.Label(dash, text="{}".format(round(lidar_range / 2)), width=w, height=h, anchor="e", padx=10)
        lbl.grid(row=4, column=5)
        lbl = tk.Label(dash, text="0", width=w, height=h, anchor="ne", padx=10)
        lbl.grid(row=7, column=5)

        # build LIDAR colorbar
        fig = Figure(figsize=(0.5, 2.5), frameon=False)
        colorbar_ax = fig.add_subplot(111)
        colorbar_ax.spines['top'].set_visible(False)
        colorbar_ax.spines['right'].set_visible(False)
        colorbar_ax.spines['bottom'].set_visible(False)
        colorbar_ax.spines['left'].set_visible(False)
        colorbar_plot = FigureCanvasTkAgg(fig, master=dash)
        colorbar_plot.get_tk_widget().grid(row=1, column=6, rowspan=7)
        colorbar_x_vals = [1 for x in range(round(lidar_range))]
        colorbar_y_vals = [y + 1 for y in range(round(lidar_range))]
        colorbar_colors = np.zeros((round(lidar_range), 3))
        for idx in range(round(lidar_range)):
            colorbar_colors[idx, :] = color_from_map(1 - (colorbar_y_vals[idx] / lidar_range))
        colorbar_ax.get_xaxis().set_visible(False)
        colorbar_ax.get_yaxis().set_visible(False)
        #colorbar_ax.cla()
        colorbar_ax.scatter(colorbar_x_vals, colorbar_y_vals, s=5, color=colorbar_colors)
        colorbar_plot.draw()

        cycle_duration = 0.333  # 333 milliseconds (3 Hz)  # plotting is sloooooooow...
        while True:
            cycle_start_time = time()

            # NOTE: ANY ACCESS TO SENSOR-BASED VARIABLES SHOULD BE INSIDE THE LOCK
            self.bot.sensor_lock.acquire()

            # update position
            pos = self.bot.smoothed_position
            xpos_lbl.set("{:+0,.2f}".format(pos[0]))
            ypos_lbl.set("{:+0,.2f}".format(pos[1]))
            zpos_lbl.set("{:+0,.2f}".format(pos[2]))

            ### update orientation
            # bearing
            bearing = self.bot.smoothed_bearing
            if bearing is not None:
                bear_lbl.set("{:+0.1f}".format(bearing))
            else:
                bear_lbl.set("----")
            # pitch
            pitch = self.bot.smoothed_pitch
            pitch_lbl.set("{:+0.1f}".format(pitch))
            # roll
            roll = self.bot.smoothed_roll
            roll_lbl.set("{:+0.1f}".format(roll))
            # grade
            grade = self.bot.smoothed_grade
            grade_lbl.set("{:+0.1f}".format(grade))

            ### update height above ground (altitude)
            height_above_ground = self.bot.smoothed_height_above_ground
            height_above_ground_lbl.set("{:0.1f}".format(height_above_ground))

            ### update rates of change
            speed = self.bot.smoothed_speed
            acceleration = self.bot.smoothed_acceleration
            speed_lbl.set("{:+0.1f}".format(speed))
            acceleration_lbl.set("{:+0.1f}".format(acceleration))
            yaw_rate = self.bot.smoothed_yaw_rate
            if yaw_rate is not None:
                yaw_rate_lbl.set("{:+0.1f}".format(yaw_rate))
            else:
                yaw_rate_lbl.set("----")
            pitch_rate = self.bot.smoothed_pitch_rate
            pitch_rate_lbl.set("{:+0.1f}".format(pitch_rate))
            roll_rate = self.bot.smoothed_roll_rate
            roll_rate_lbl.set("{:+0.1f}".format(roll_rate))
            grade_rate = self.bot.smoothed_grade_rate
            grade_rate_lbl.set("{:+0.1f}".format(grade_rate))
            wobble = self.bot.wobble
            wobble_lbl.set("{:0.1f}".format(wobble))

            # update the lidar color plot
            '''
            # OLD
            lidar_vals = self.bot.smoothed_lidar_values[5,:] # 2FIX: row 5 is supposed to be +5 degrees -- is it?
            lidar_vals = [1 - x / lidar_range for x in lidar_vals]  # map to [0,1] range
            lidar_num_samples = len(lidar_vals)
            lidar_colors = np.zeros((lidar_num_samples, 3))
            for idx in range(lidar_num_samples):
                lidar_colors[idx,:] = color_from_map(lidar_vals[idx])
            '''
            # NEW:
            lidar_vals = self.bot.smoothed_lidar_values
            lidar_vals = 1 - (lidar_vals / lidar_range)  # map to [0,1] range
            for r in range(lidar_matrix_size[0]):
                for c in range(lidar_matrix_size[1]):
                    lidar_colors[r,c] = color_from_map(lidar_vals[r,c])

            self.bot.sensor_lock.release()

            # plotting is slow -- do it after releasing the thread lock
            lidar_ax.cla()
            lidar_ax.scatter(lidar_plt_x_vals.reshape(num_lidar_values), lidar_plt_y_vals.reshape(num_lidar_values),\
                             s=3, color=lidar_colors.reshape(num_lidar_values))
            lidar_plot.draw()

            # update the dashboard window
            dash.update()

            sleep_until_end_of_cycle(cycle_start_time, cycle_duration)
        # end while (true)
    # end method run
# end class Dashboard

if __name__ == "__main__":
    dash = Dashboard()
    dash.run()
# end if main
