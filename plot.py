import matplotlib.pyplot as plt
import numpy as np

# use ggplot style for more sophisticated visuals


def live_plotter(x_vec,y1_data,line1,y2_data,line2,identifier='',pause_time=0.1):
    if line1==[]:
        # this is the call to matplotlib that allows dynamic plotting
        plt.ion()
        fig = plt.figure(figsize=(13,6))
        ax = fig.add_subplot(111)
        # create a variable for the line so we can later update it
        line1, = ax.plot(x_vec,y1_data,'-o',alpha=0.8)  
        line2, = ax.plot(x_vec,y2_data,'-o',alpha=0.8)       
        #update plot label/title
        plt.ylabel('Y Label')
        plt.title('Title:IMU DATA {}'.format(identifier))
        plt.show()
    
    # after the figure, axis, and line are created, we only need to update the y-data
    line1.set_ydata(y1_data)
    line2.set_ydata(y2_data)
    
    # adjust limits if new data goes beyond bounds
    if np.min(y1_data)<=line1.axes.get_ylim()[0] or np.max(y1_data)>=line1.axes.get_ylim()[1]:
        plt.ylim([np.min(y1_data)-np.std(y1_data),np.max(y1_data)+np.std(y1_data)])
    if np.min(y2_data)<=line2.axes.get_ylim()[0] or np.max(y2_data)>=line1.axes.get_ylim()[1]:
        plt.ylim([np.min(y2_data)-np.std(y2_data),np.max(y2_data)+np.std(y2_data)])
    # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
    plt.pause(pause_time)
    
    # return line so we can update it again in the next iteration
    return line1,line2

# the function below is for updating both x and y values (great for updating dates on the x-axis)
def live_plotter_xy(x_vec,y1_data,line1,identifier='',pause_time=0.01):
    if line1==[]:
        plt.ion()
        fig = plt.figure(figsize=(13,6))
        ax = fig.add_subplot(111)
        line1, = ax.plot(x_vec,y1_data,'r-o',alpha=0.8)
        plt.ylabel('Y Label')
        plt.title('Title: {}'.format(identifier))
        plt.show()
        
    line1.set_data(x_vec,y1_data)
    plt.xlim(np.min(x_vec),np.max(x_vec))
    if np.min(y1_data)<=line1.axes.get_ylim()[0] or np.max(y1_data)>=line1.axes.get_ylim()[1]:
        plt.ylim([np.min(y1_data)-np.std(y1_data),np.max(y1_data)+np.std(y1_data)])

    plt.pause(pause_time)
    
    return line1


import numpy as np

size = 100
x_vec = np.linspace(0,1,size+1)[0:-1]
y_vec = np.zeros(len(x_vec))
y_2vec = np.zeros(len(x_vec))
line1 = []
line2 = []
while True:
    rand_val = np.random.randn(1)
    rand_val2 = np.random.randn(1)
    y_vec[-1] = rand_val
    y_2vec[-1] = rand_val2
    line1,line2 = live_plotter(x_vec,y_vec,line1,y_2vec,line2)
    y_vec = np.append(y_vec[1:],0.0)
    y_2vec = np.append(y_2vec[1:],0.0)


