#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

import matplotlib.pyplot as plt
import numpy as np
import time
plt.ion()
class DynamicUpdate():
    #Suppose we know the x range
    min_x = 0
    time_of_simulation = 10 #in seconds
    real_y = 0
    frequency = 0.002
    max_x = time_of_simulation/frequency
    def on_launch(self):
        #Set up plot
        self.figure, self.ax = plt.subplots()
        self.lines, = self.ax.plot([],[], 'o')
        #Autoscale on unknown axis and known lims on the other
        self.ax.set_autoscaley_on(True)
        #self.ax.set_xlim(self.min_x, self.max_x)
        #Other stuff
        self.ax.grid()

    def on_running(self, xdata, ydata):
        #Update data (with the new _and_ the old points)
        self.lines.set_xdata(xdata)
        self.lines.set_ydata(ydata)
        #Need both of these in order to rescale
        self.ax.relim()
        self.ax.autoscale_view()
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

    def callback(self, data):
        self.real_y = data.data
        print self.real_y

    def listener(self):
        rospy.init_node('plotIRP6HomogMatrix', anonymous=True)
        rospy.Subscriber("seeByIRP6HomogMatrix", Float32, self.callback)
        xdata = []
        ydata = []
        for x in np.arange(self.min_x,self.max_x,self.frequency):
            xdata.append(x)
            ydata.append(self.real_y)
            self.on_running(xdata, ydata)
            time.sleep(self.frequency)
        return xdata, ydata
        rospy.spin()

    #Example
    def __call__(self):
        self.on_launch()
        self.listener()
d = DynamicUpdate()
d()
