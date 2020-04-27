import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from time import time

class ControlDashboard():
    def __init__(self, bot):
        self.bot = bot

        # initialize dashboard window
        self.dash = tk.Tk()
        self.dash.config(background='white')
        self.dash.geometry('600x350')
        self.dash.title('Control Dashboard')

        # torque
        col = 0
        # title label
        lbl = tk.Label(self.dash, text='Torque', anchor='e')
        lbl.grid(row=0, column=col)
        # plot
        fig = Figure(figsize=(0.75, 2.5), frameon=False)
        fig.subplots_adjust(left=0.75)
        ax = fig.add_subplot(111)
        plot = FigureCanvasTkAgg(fig, master=self.dash)
        plot.get_tk_widget().grid(row=1, column=col, padx=(10,0))
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['bottom'].set_visible(False)
        ax.spines['left'].set_visible(False)
        ax.get_xaxis().set_visible(False)
        ax.get_yaxis().set_visible(True)
        self.torque_plot = plot
        self.torque_ax = ax

        # acceleration
        col = 1
        # title label
        lbl = tk.Label(self.dash, text='Acceleration', anchor='e')
        lbl.grid(row=0, column=col)
        # plot
        fig = Figure(figsize=(0.75, 2.5), frameon=False)
        fig.subplots_adjust(left=0.75)
        ax = fig.add_subplot(111)
        plot = FigureCanvasTkAgg(fig, master=self.dash)
        plot.get_tk_widget().grid(row=1, column=col, padx=(10,0))
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['bottom'].set_visible(False)
        ax.spines['left'].set_visible(False)
        ax.get_xaxis().set_visible(False)
        ax.get_yaxis().set_visible(True)
        self.acceleration_plot = plot
        self.acceleration_ax = ax

        # speed
        col = 2
        # title label
        lbl = tk.Label(self.dash, text='Speed', anchor='e')
        lbl.grid(row=0, column=col)
        # plot
        fig = Figure(figsize=(0.75, 2.5), frameon=False)
        fig.subplots_adjust(left=0.75)
        ax = fig.add_subplot(111)
        plot = FigureCanvasTkAgg(fig, master=self.dash)
        plot.get_tk_widget().grid(row=1, column=col, padx=(10,0))
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['bottom'].set_visible(False)
        ax.spines['left'].set_visible(False)
        ax.get_xaxis().set_visible(False)
        ax.get_yaxis().set_visible(True)
        self.speed_plot = plot
        self.speed_ax = ax

        # altitude (height above ground)
        col = 3
        # title label
        lbl = tk.Label(self.dash, text='Altitude', anchor='e')
        lbl.grid(row=0, column=col)
        # plot
        fig = Figure(figsize=(0.75, 2.5), frameon=False)
        fig.subplots_adjust(left=0.75)
        ax = fig.add_subplot(111)
        plot = FigureCanvasTkAgg(fig, master=self.dash)
        plot.get_tk_widget().grid(row=1, column=col, padx=(10,0))
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['bottom'].set_visible(False)
        ax.spines['left'].set_visible(False)
        ax.get_xaxis().set_visible(False)
        ax.get_yaxis().set_visible(True)
        self.altitude_plot = plot
        self.altitude_ax = ax

        # wobble
        col = 4
        # title label
        lbl = tk.Label(self.dash, text='Wobble', anchor='e')
        lbl.grid(row=0, column=col)
        # plot
        fig = Figure(figsize=(0.75, 2.5), frameon=False)
        fig.subplots_adjust(left=0.75)
        ax = fig.add_subplot(111)
        plot = FigureCanvasTkAgg(fig, master=self.dash)
        plot.get_tk_widget().grid(row=1, column=col, padx=(10,0))
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['bottom'].set_visible(False)
        ax.spines['left'].set_visible(False)
        ax.get_xaxis().set_visible(False)
        ax.get_yaxis().set_visible(True)
        self.wobble_plot = plot
        self.wobble_ax = ax

    # end constructor

    # run the dashboard
    def update(self):
        # update torque
        self.torque_ax.cla()
        self.torque_ax.set_xlim([-1, 1])
        self.torque_ax.set_ylim([-self.bot.max_abs_torque, self.bot.max_abs_torque])
        self.torque_ax.plot(0, self.bot.smoothed_torque, 'ko')
        self.torque_plot.draw()

        # update acceleration
        self.acceleration_ax.cla()
        self.acceleration_ax.set_xlim([-1, 1])
        self.acceleration_ax.set_ylim([-30, 30])
        self.acceleration_ax.plot(0, self.bot.smoothed_acceleration, 'go')
        self.acceleration_plot.draw()

        # update speed
        self.speed_ax.cla()
        self.speed_ax.set_xlim([-1, 1])
        self.speed_ax.set_ylim([-25, 25])
        self.speed_ax.plot(0, self.bot.smoothed_speed, 'ro')
        self.speed_plot.draw()

        # update altitude
        self.altitude_ax.cla()
        self.altitude_ax.set_xlim([-1, 1])
        self.altitude_ax.set_ylim([0, 5])
        self.altitude_ax.plot(0, self.bot.smoothed_height_above_ground, 'bo')
        self.altitude_plot.draw()

        # update wobble
        self.wobble_ax.cla()
        self.wobble_ax.set_xlim([-1, 1])
        self.wobble_ax.set_ylim([0, 10])
        self.wobble_ax.plot(0, self.bot.wobble, 'mo')
        self.wobble_plot.draw()

        self.dash.update()
    # end method update
# end class Dashboard
