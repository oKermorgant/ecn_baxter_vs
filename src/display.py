#!/usr/bin/python

'''
Plots relative joints and VS error
'''
import pylab as pl
import roslib, rospy, rospkg
from std_msgs.msg import Float32MultiArray
import os

side = "right"

class Listener:
    def __init__(self, topic):
        
        self.history = []
        self.t = []
        self.t0 = 0
        self.dim = 0
        self.lock = False
        
        # subscriber to joints and visual error
        for side in ('left', 'right'):
            rospy.Subscriber('/'.join(['','display',side,topic]), Float32MultiArray, self.read, (side))
        
        # init lines
        self.lines = []
        self.joints = "joint" in topic
        
        # init figure
        self.F = pl.figure()
        self.ax = self.F.gca()
        self.ax.set_xlabel('Time [s]')
        self.F.tight_layout()

    def read(self, msg, args):
        if args == side and not self.lock:
            # append history 
            self.history.append(msg.data)
            if not self.t0:
                self.t0 = rospy.Time.now().to_sec()
            t = rospy.Time.now().to_sec() - self.t0
            self.t.append(t)
            
            self.dim = len(msg.data)
            
    def update(self):
        # remove old measurements
        self.lock = True
        if self.dim:
            idx = 0
            while self.t[-1] - self.t[idx] > 10:
                idx += 1
            self.history = self.history[idx:]
            self.t = self.t[idx:]
            
            if len(self.lines):
                try:
                    # update lines
                    for i in xrange(self.dim):
                        self.lines[i].set_data(self.t, [h[i] for h in self.history])
                    
                    # update zero
                    self.lines[i+1].set_data([self.t[0],self.t[-1]],[0,0])
                    
                    if self.joints:
                        self.lines[i+2].set_data([self.t[0],self.t[-1]],[1,1])
                        yM = 1
                        ym = 0
                    else:
                        y = [h[i] for h in self.history for i in xrange(self.dim)]
                        yM = max(y)
                        ym = min(y)
                    self.ax.axis((self.t[0],self.t[-1],ym - 0.05*(yM-ym), yM+0.05*(yM-ym)))
                except:
                    pass
                    
            else:
                # init plot
                if self.joints:
                    names = ["$q_{}$".format(i) for i in xrange(1,8)]
                else:
                    names = ["x","y","a"]

                plot_colors = ['b','g','r','c','m','y']
                plot_markers = ['', 'o', 's', 'x', 'D', 'p', '+']                    
                for i in xrange(self.dim):
                    col = plot_colors[i % len(plot_colors)]
                    mk = plot_markers[i % len(plot_markers)]
                    self.lines += self.ax.plot([],[], col+mk+'-',label=names[i],linewidth=2)
                    pl.legend(loc='center right')

                if self.joints:
                    # joint limits
                    for i in (1,2):
                        self.lines += self.ax.plot([],[],'k-',linewidth=4)
                else:
                    # zero line
                    self.lines += self.ax.plot([],[],'k--')                
        self.lock = False
        
if __name__ == '__main__':
    '''
    Begin of main code
    '''
    
    rospy.init_node('display')
    
    pl.ion()
    pl.close('all')
    
    joint_listener = Listener("joints")
    #vs_listener = Listener("vs")
    
    # get source file to read side... will do if binary has changed
    pkg = 'ecn_baxter_vs'
    src = os.path.abspath(__file__)
    src = src.replace('src/display.py', 'main.cpp')
    tree = src.split('/')
    l = len(tree)
    ws = src
    while not (os.path.exists(ws + '/src') and os.path.exists(ws + '/devel')):
        l -= 1
        ws = '/'.join(tree[:l])
    # get to binary file
    binary = ws + '/devel/.private/' + pkg + '/lib/' + pkg + '/baxter_vs'
    
    t0 = os.stat(binary).st_mtime
    while not rospy.is_shutdown():
        t = os.stat(binary).st_mtime
        if t - t0 > 1:
            # update last change
            t0 = t
            # re-read source to get side
            with open(src) as f:
                content = f.read().splitlines()
            side = 'right'
            for line in content:
                if 'BaxterArm' in line:
                    if 'left' in line:
                        side = 'left'
                    break
        
        joint_listener.update()
        vs_listener.update()
        try:
            pl.draw()
            pl.pause(0.001)
        except:
            pass

        rospy.sleep(0.01)
