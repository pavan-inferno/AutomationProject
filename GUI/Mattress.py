'''
File: c:\Users\nuthip\Desktop\Mattress project\Mattress.py
Project: c:\Users\nuthip\Desktop\Mattress project
Created Date: Thursday March 7th 2019
Author: Pavan Nuthi
-----
Last Modified: Thursday, 30th January 2020 11:03:33 am
Modified By: Pavan Nuthi (pavan.nuthi@uta.edu>)
-----
Copyright (c) 2019 UTARI
'''

import binascii
import os.path
import struct
import sys
import time

# confirm the version of Gtk
import pygtk
pygtk.require('2.0')
import gtk

import gobject
import matplotlib
import serial
import serial.tools.list_ports as port_list
import xlsxwriter
from matplotlib import cm
from matplotlib.font_manager import FontProperties
from matplotlib.backends.backend_gtkagg import \
    FigureCanvasGTKAgg as FigureCanvas
from matplotlib.backends.backend_gtkagg import \
    NavigationToolbar2GTKAgg as NavigationToolbar
from matplotlib.figure import Figure
from matplotlib.mlab import griddata
from mpl_toolkits.mplot3d import Axes3D
from numpy import arange, array, int16, linspace, meshgrid, ones, pi, sin, sqrt
from scipy import interpolate, io
from datetime import datetime

class GUI():

    # initializes the gui
    def __init__(self):
        ##Building Gtk objects from glade file
        # make a builder object
        self.ui = gtk.Builder()
        # read the builder object from glade file
        self.ui.add_from_file('./Mattress.glade')
        # iterate through gtkobjects in glade file and import them as self.*
        # optionally prints all the gtk objects in the glade file 
        print "Imported Gtk objects:"
        for i in self.ui.get_objects():
            if issubclass(type(i), gtk.Buildable):
                name = gtk.Buildable.get_name(i)
                setattr(self, name, i)
                # print str(name), ",",
            else:
                print >>sys.stderr, "WARNING: cannot get name for '%s'" % i
        # connect all callbacks mentioned in glade file
        self.ui.connect_signals(self)
        # initialize the z axis upper limit
        self.z_scale = 1
        # setup initial plot objects
        self.setup_plot()
        # create a gtk toolbar for the canvas
        self.toolbar = NavigationToolbar(self.canvas, self.main_window)
        # pack the plotbox with the drawing area and toolbar
        self.plotbox.pack_start(self.canvas)
        self.plotbox.pack_start(self.toolbar,False, False)
        # self.remoteplotbox.pack_start(self.canvas)
        # self.remoteplotbox.pack_start(self.toolbar,False, False)

        # initialize serial port
        self.serialport = serial.Serial()
        # initialize comport selection ui
        print "Setting initial port"
        # get a list of serial ports
        ports = list(port_list.comports())
        comportlist = [str(i.device) for i in ports ]
        self.comportlstore = gtk.ListStore(int, str)
        self.make_comboboxlist(self.comportlstore, self.comportbox, comportlist, defaultindex = 1)
        # set the baudrate as 250kbits pers second
        self.serialport.baudrate = 250000

        # initialize weight selection ui
        weightslist = ["0-100 lb", "100-200 lb", "200-300 lb"]
        self.weightlstore = gtk.ListStore(int,str)
        self.make_comboboxlist(self.weightlstore, self.weightbox, weightslist, defaultindex = 2 )

        # initialize unit selection ui
        unitslist = ["psi", "kPa", "Kg/cm^2", "mmHg"]
        self.unitlstore = gtk.ListStore(int,str)
        self.make_comboboxlist(self.unitlstore, self.unitsbox, unitslist, defaultindex = 1 )

        # initialize time unit selection ui
        timelist = ["minutes", "hours"]
        self.timelstore = gtk.ListStore(int,str)
        self.make_comboboxlist(self.timelstore, self.timebox1, timelist, defaultindex = 0 )
        self.make_comboboxlist(self.timelstore, self.timebox2, timelist, defaultindex = 0 )

        # initialize bubble set selection ui
        bubblelist = ["Lower back", "Buttocks", "Shoulders"]
        self.bubblelstore = gtk.ListStore(int,str)
        self.make_comboboxlist(self.bubblelstore, self.bubblebox, bubblelist, defaultindex = 0 )

        self.method = "thin_plate"
        
        # initialize flags
        self.isfullpacket = False   # indicates whether a valid data packet is received
        self.isrecording = False    # indicates whether the GUI is currently recording
        self.ascii_switch = False   # indicates whether the device is sending ascii or binary packets
        # initialize counter for recording frames
        self.rec_counter = 1        # current number of recorded frames
        # set a list of unit multipliers corresponding to unit selection
        self.unit_sel = [1, 6.8946, 0.070307, 51.7149326]
        # initialize list of target pressures sent to the device
        self.setpressuremap = [1 for i in range(96)]
        # initialize the instantaneous rate at which full data packets are received 
        self.daq_rate = 0           # indicates the rate of full packet reception
        # set labels to show bubble numbers (static)
        self.bubble_number.set_text('\n'.join([str(i) for i in range(1,96+1)]))
        # set a timeout function which refreshes display at 1000/50  = 20 fps
        self.gui_update_object = gobject.timeout_add(
            200, self.update_gui)
        # set a timeout function for plot object separate from other displays at a lower refresh rate
        self.plot_update_object = gobject.timeout_add(600, self.update_plot)
        # set an idle function which keeps the counter running as well as updates self.t1, t2, t3
        self.gui_mainloop_object = gobject.idle_add(
            self.collect_serialdata, priority=gobject.PRIORITY_DEFAULT_IDLE)
        # create a list of checkbox objects which is easier to access
        self.checkbox_obj_list = []
        for self.checkbox_number in range(1,96+1):
            self.temp_obj = self.ui.get_object("cb%s" % self.checkbox_number)
            self.temp_obj.set_active(True)
            self.checkbox_obj_list.append(self.temp_obj)
        # initialize state machines for manual and automatic offloading and redistribution
        self.offload_state = 0              # indicates state of manual offloading algorithm
        self.redistribute_state = 0         # indicates state of redistribution algorithm
        self.auto_state = 0                 # indicates state of automatic offloading algorithm
        self.controller_state = 0           # indicates the controller's state (Busy or idle)

        # initialise redistribution parameters 
        self.Na = 50                # indicates no. of smaller bubbbles
        self.Nb = 6                # indicates no. of larger bubbbles
        self.na = 0                 # indicates no. of smaller bubbles offloaded
        self.nb = 0                 # indicates no. of larger bubbles offloaded
        self.rhoa = 0.05            # indicates offloading pressure for larger bubbles
        self.rhob = 0.01            # indicates offloading pressure for smaller bubbbles
        self.Aa = 346.36            # surface area of smaller bubbles 
        self.Ab = 1068.39           # surface area of larger bubbles
        self.alpha = self.Ab/self.Aa# ratio of areas

        #initializea pressure under which a bubble may not go under
        self.min_pressure = 0.4

        #variable to track what the last completed operation was
        self.last_was_offload = False

        # initialize the slider for parameter selection
        ad1 = gtk.Adjustment(70, 0, 100, 5, 10, 0)  # make an adjustment object
        self.h_scale = gtk.HScale(adjustment=ad1)   # create a horizontal scale        
        self.vbox12.pack_start(self.h_scale,True)    # shove it in the assigned box
        self.h_scale.connect("value-changed", self.on_scale_valuechanged)   # connect its callback manually

        # #second scale never used...delete?
        # # initialize the slider for parameter selection
        # ad2 = gtk.Adjustment(5, 0, 30,  1, 1, 1)  # make an adjustment object
        # self.h_scale2 = gtk.HScale(adjustment=ad2)   # create a horizontal scale      
        # self.h_scale2.set_increments(1.0,0)
        # self.h_scale2.set_digits(0)  
        # self.vbox13.pack_start(self.h_sca le2,True)    # shove it in the assigned box
        # self.h_scale2.connect("value-changed", self.on_scale2_valuechanged)   # connect its callback manually

        # show the main window and all its contents
        self.main_window.show_all()
    
    # plots the seat outline on top of the plot box 
    def plot_seat_outline(self):
        # read the data file containing bubble locations and contours
        mat = io.loadmat('mattress_vertices.mat')
        self.bound = 20                     # boundary for extrapolation
        font0 = FontProperties()
        alignment = {'horizontalalignment': 'center', 'verticalalignment': 'baseline'}
        # extract a cell array of (x,y) coordinates of contours for every bubble
        vertices = mat['vertices']
        # extract a cell array of (x,y) coordinates for all bubbles
        centers = mat['centers']
        # extract useless parameters
        height = mat['height']
        width = mat['width']

        self.xmax = mat['xmax'][0][0] + self.bound
        self.xmin = mat['xmin'][0][0] - self.bound
        self.ymax = mat['ymax'][0][0] + self.bound
        self.ymin = mat['ymin'][0][0] - self.bound
        self.control_numbering = mat['control_number'][0]

        # print(self.control_numbering[0])

        # extract x coordinates of centroids of all bubbles
        self.x_centers = array([centers[0][i][0][0] for i in range(96)])
        # extract y coordinates of centroids of all bubbles
        self.y_centers = array([centers[0][i][0][1] for i in range(96)])      
        # plot the markers on top of everything else
        z_centers = self.z_scale*ones(self.x_centers.shape) 
        # create list of x cooridnates and y coordinates for points on the contour of each bubble
        # eg. x_vertices[0] will contain x coordinates of points on the contour plotting bubble1
        x_vertices = []
        y_vertices = []
        for i in range(96):
            x_vertices.append(array([vertices[0][i][j][1] for j in range(len(vertices[0][i]))],dtype=int16))
            y_vertices.append(array([vertices[0][i][j][0] for j in range(len(vertices[0][i]))],dtype=int16))
        
        # plot the centroids of all bubbles
        if hasattr(self,'scatter'):
            self.scatter.remove()
        if hasattr(self,'contours'):
            for i in self.contours:
                i[0].remove()
        if hasattr(self,'text'):
            for i in self.text:
                i.remove()

        self.scatter = self.ax.scatter(-self.x_centers, self.y_centers, z_centers, c = 'r', marker = '.')
        # plot contours outlining all bubbles after erasing them if already present
        self.text =  list()
        for i in range(96):
            self.text.append(self.ax.text(-self.x_centers[i], self.y_centers[i]+15, z_centers[i], str(self.control_numbering[i]), color = 'white',fontsize=7, horizontalalignment='center'))
        
        self.contours = list()
        for i in range(96):
            z_vertices = self.z_scale*ones(x_vertices[i].shape)
            self.contours.append( self.ax.plot(-x_vertices[i], y_vertices[i], z_vertices, c = 'w') )
    
    # initialize the plot objects and data
    def setup_plot(self):
        # make a figure object
        self.fig = Figure()
        # create a gtk.DrawingArea through a backend function from matplotlib which will contain the figure
        self.canvas = FigureCanvas(self.fig)
        # add an axis to the figure, you can do this with add_subplot if you have more axes, this is just easier
        self.ax = self.fig.gca(projection='3d')
        # make the margins zero
        self.fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0 )
        
        # set axis limitsproperties
        # self.ax.set_xlim(140, 600)
        # self.ax.set_ylim(143, 720)
        # self.ax.set_zlim(0,self.z_scale)

        # set invisible ticks
        self.ax.get_xaxis().set_ticks([])
        self.ax.get_yaxis().set_ticks([])
        self.ax.set_zticks([])
        # set initial view angle
        self.ax.view_init(90, 90)
        # plot the outline on top of the view plane
        self.plot_seat_outline()
        # set number of subdivision for finer grid
        self.npts = 50
        # make x and y arrays with finer spacing
        
        xfine = linspace(self.xmin, self.xmax, self.npts)
        yfine = linspace(self.ymin, self.ymax, self.npts)
        # create a mesh
        self.xfine, self.yfine = meshgrid(xfine, yfine)
        # grid the data to plot a sine wave
        self.zfine = griddata(self.x_centers, self.y_centers,0*(self.y_centers), self.xfine, self.yfine, interp='linear')
        # fit an RBF to the data
        rbf = interpolate.Rbf(self.x_centers, self.y_centers,array(0*(self.y_centers)), function='thin_plate', smooth=1 )
        # interpolate/extrapolate over the area
        self.zfine = rbf(self.xfine,self.yfine)
        # plot the initial surface
        self.surf = self.ax.plot_surface(-self.xfine, self.yfine, self.zfine, cmap = cm.jet, linewidth=0, antialiased = False, vmin = 0, vmax = self.z_scale)
        # create mappable
        sm = cm.ScalarMappable(cmap=cm.jet, norm=matplotlib.colors.Normalize(vmin = 0, vmax = self.z_scale))
        sm.set_array([])
        # draw the canvas for the first time
        self.canvas.draw()
        # draw the colorbar
        self.colorbar = self.fig.colorbar(sm, ax = self.ax, fraction=.15, pad = 0.0, shrink=0.7, aspect=15)
        # set its label
        self.colorbar.ax.set_title("units")

    # updates the plot frame(timer frame)
    def update_plot(self):
        if self.serialport.is_open and self.isfullpacket:
            ## Old way of interpolation using griddata which is crude with 'nearest' or doesn't extrapolation for 'linear' and 'cubic'
            #  grid the data.
            # self.zfine = griddata(self.x_centers, self.y_centers,array(self.pressure_data), self.xfine, self.yfine, interp='nn')
            # self.zfine = interpolate.griddata((self.x_centers, self.y_centers),array(self.pressure_data), (self.xfine, self.yfine), method=self.method, fill_value=0)
            ## another way to interpolate
            # spline = interpolate.SmoothBivariateSpline(self.x_centers, self.y_centers, array(self.pressure_data), kx=1, ky=1)
            # self.zfine = spline(self.xf, self.yf)
            
            # Fit an RBF to the measurements
            # print len(self.pressure_data)

            rbf = interpolate.Rbf(self.x_centers, self.y_centers,array(self.pressure_data), function=self.method, smooth=1 )
            # interpolate/extrapolate over the region
            self.zfine = rbf(self.xfine,self.yfine)
            # delete the previous surface
            self.surf.remove()
            # plot the new surface
            self.surf = self.ax.plot_surface(-self.xfine, self.yfine, self.zfine, cmap = cm.jet,  linewidth=0, antialiased = False , vmin = 0, vmax = self.z_scale)
            # get z scale
            lims =  self.ax.get_zlim()
            self.z_scale = lims[1]
            # set colorbar limits to match with z scale
            self.colorbar.set_clim(0, lims[1])
            # redraw the colorbar
            self.colorbar.draw_all()
            # redraw the canvas
            self.canvas.draw()
        return True

    def transform_pressure(self):
        for i in range(96):
            pass
            
    # updates the gui(timer function)
    def update_gui(self):
        # if self.serialport.is_open:
            # when serial port is open
            # write time elapsed
            # self.elapsed_time.set_text(str(int(time.time() - self.start_time)))
            # write daq rate 
            # self.daq_rate_label.set_text(str(int(self.daq_rate)))

        # if self.serialport.is_open and hasattr(self,'bin_line'):
        #     if (len(self.bin_line)>0):
        #         print (self.bin_line)

        if self.isfullpacket and self.serialport.is_open:
            # when a full packet is recieved
            # write recieved packaet length
            # self.received_packet_len.set_text(str(self.rcvd_pkt_len))
            # write controller state

            if self.controller_state==0:
                self.state.set_text("Idle")
            else:
                # if the controller's  not idle it shows the number of bubble its working on 
                self.state.set_text(str(self.controller_state))
            
            # self.state.set_text(str(self.controller_state))
            


            # send the setpressure map
            self.send_packet()
            # write sent packet length if a packet is already sent
            # if hasattr(self,'sent_packet'):
            #     self.sent_packet_len.set_text(str(len(self.sent_packet)))
            # write teh rate of data transmitted from controller
            # self.arduino_rate_label.set_text(str(int(self.arduino_rate)))
            # compute unit multiplier based on user selection
            unit_multiplier = self.unit_sel[self.unitsbox.get_active()]

            # when tab display is on
            # write sensed pressure

            self.sensed_pressure.set_text('\n'.join([str(round(i*unit_multiplier,3)) for i in self.pressure_data]))
            
            # # get an ascending sorder
            # templist = sorted((e,i) for i,e in enumerate(self.pressure_data))
            # # make it descending
            # templist = templist[::-1]
            # # write the sorted indices list
            # self.sorted_indices.set_text('\n'.join([str(i[1]+1) for i in templist]))

            # make a target pressure string
            target_string = list()

            for i in range(96):
                if self.checkbox_obj_list[i].get_active():
                    # when the bubble is activated
                    target_string.append(str(round(self.setpressuremap[i]*unit_multiplier,3)))
                else:
                    # when its not activated
                    target_string.append("-")
            # write the resultant target string
            self.target_pressure.set_text('\n'.join(target_string))
            
            # # when avg display is on
            # if(self.avg_display_checkbox.get_active()):
            #     self.avg_r_thigh.set_text(str(round(unit_multiplier*sum(self.pressure_data[25:28])/3, 2)))
            #     self.avg_l_thigh.set_text(str(round(unit_multiplier*sum(self.pressure_data[56:59])/3, 2)))
            #     self.avg_r_butt.set_text(str(round(unit_multiplier*sum(self.pressure_data[0:25])/25, 2)))
            #     self.avg_l_butt.set_text(str(round(unit_multiplier*sum(self.pressure_data[31:56])/25, 2)))            
            #     self.avg_r.set_text(str(round(unit_multiplier*sum(self.pressure_data[0:31])/31, 2)))
            #     self.avg_l.set_text(str(round(unit_multiplier*sum(self.pressure_data[31:62])/31, 2)))
            #     self.avg_butt.set_text(str(round(unit_multiplier*sum(self.pressure_data[31:56]+self.pressure_data[0:25])/50,2)))
            #     self.avg_thigh.set_text(str(round(sum(self.pressure_data[25:28]+self.pressure_data[56:59])*unit_multiplier/6,2)))
            #     self.avg_pressure.set_text(str(round(unit_multiplier*sum(self.pressure_data)/62, 2)))
            #     self.min.set_text(str(round(unit_multiplier*min(self.pressure_data),2)))
            #     self.max.set_text(str(round(unit_multiplier*max(self.pressure_data),2)))
            #     self.avg_bndry.set_text(str(round(unit_multiplier*sum(self.pressure_data[28:31]+self.pressure_data[59:62])/6,2)))
        return True
    
    # main loop to process the serial information (idle function)
    def collect_serialdata(self):
        # when the port's open
        if self.serialport.is_open:
            # read a line
            self.bin_line = self.serialport.readline()                              #Reading data from serial port
            # determine if its binary packet or ascii
            if len(self.bin_line) == 133: # this part needs to change(byte numbers)
                # binary full packet conditions
                if self.bin_line[132] == '\n' and self.bin_line[131] == '\r' and self.bin_line[130] == 'z' and self.bin_line[0]== 'b':
                    self.isfullpacket = True
                    self.ascii_switch = False
                    self.rcvd_packet = self.bin_line[1:130]
                    self.rcvd_pkt_len = len(self.rcvd_packet)
                    self.rcvd_data = struct.unpack("<hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhfB", self.rcvd_packet)
                    self.pressure_data = [((i/1023.0)-0.04)*16.11 for i in self.rcvd_data[0:62]]
                    self.arduino_rate = self.rcvd_data[62]
                    self.controller_state = int(self.rcvd_data[63])
                else:
                    self.isfullpacket = False
            else:
                if self.bin_line[0] == 'a' and self.bin_line[-1]=='\n' and self.bin_line[-2]=='\r' and self.bin_line[-3]== 'z':
                    self.rcvd_csv = self.bin_line[1:-3]
                    temp = [i for i in self.rcvd_csv.split(",")]
                    if len(temp)==(96+2):
                        # ascii full packet condition                        
                        self.pressure_data = [float(i) for i in temp[0:96]]
                        self.arduino_rate = float(temp[96])
                        self.rcvd_pkt_len = len(self.rcvd_csv)
                        self.controller_state = int(temp[96+1])
                        self.isfullpacket = True
                        self.ascii_switch = True
                    else:
                        self.ascii_switch = False
                        self.isfullpacket = False
                else:
                    self.isfullpacket = False
                    self.ascii_switch = False
            # if there's a full packet capture
            if self.isfullpacket:
                # calculate the rate of full packet capture
                if not hasattr(self,"t_now"):
                    self.t_now = time.time()-1          # handle  initial value of t_now
                self.t_prev = self.t_now                # update  t_prev with old time
                self.t_now = time.time()                # update t_now with new time
                if self.t_now !=self.t_prev:            # calculate daq_rate if its not too fast
                    self.daq_rate = 1/(self.t_now- self.t_prev)
                self.avg_seating = sum(self.pressure_data[0:25]+ self.pressure_data[31:56])/50
                # update the file if recording
                if self.isrecording:
                    for i in range(0, len(self.pressure_data)):          #For loop to select row in the worksheet 
                        self.worksheet_rec.write(i+1,self.rec_counter, (self.pressure_data[i]))
                    self.worksheet_rec.write(0, self.rec_counter, self.t_now - self.start_time)
                    self.worksheet_rec.write(63,self.rec_counter, self.arduino_rate )
                    self.worksheet_rec.write(64,self.rec_counter, self.daq_rate )                
                    self.rec_counter = self.rec_counter +1
        return True

    # cleans up and destroys gtk objects
    def on_main_window_destroy(self, *args):
        # cleanup
        # delete idle and timer functions
        gobject.source_remove(self.gui_update_object)
        gobject.source_remove(self.plot_update_object)
        gobject.source_remove(self.gui_mainloop_object)
        # close serial port if it's open
        if self.serialport.is_open:
            print "Serial port closed on exit"
            self.serialport.close()
        # quit gtk
        gtk.main_quit(*args)

    # creates a combobox from a list of strings and sets a given index
    def make_comboboxlist(self, lststore, combobox, combolist, defaultindex=0):
        # create a liststore
        lststore = gtk.ListStore(int, str)
        # populate it
        for i in range(len(combolist)):
            lststore.append([i, combolist[i]])
        # set the dropdown box to the liststore
        combobox.set_model(lststore)
        # create a  cell renderer object
        self.cell = gtk.CellRendererText()
        # clear the combobox
        combobox.clear()
        # pack the cell renderer into the box
        combobox.pack_start(self.cell, True)
        combobox.add_attribute(self.cell, "text", 1)
        # set given default option
        combobox.set_active(defaultindex)
    
    # displays a message on both notification area and console
    def display_all(self,data = ""):
        # prints data to console as well as notification area
        print data
        self.notification_area.set_text(data)
    
    # converts list into a grouped list
    def split_by_n(self, seq, n):
        while seq:
            yield seq[:n]
            seq = seq[n:]

    # sets the scale manually to a given value
    def on_manual_scale_activate(self, widget):
        # set  new z scale
        self.z_scale = float(widget.get_text())
        # replot the outline on top
        self.plot_seat_outline()
        # set z limit again
        self.ax.set_zlim(0, self.z_scale)
        # clear the input 
        widget.set_text("")

    # turns off all checkboxes
    def on_all_off_clicked(self, widget):
        # turn all checkboxes off
        for i in self.checkbox_obj_list:
            if(self.actuate_thighs.get_active()):
                i.set_active(False)
            else:
                # if thighs are activated
                if not (self.checkbox_obj_list.index(i)+1==26 or self.checkbox_obj_list.index(i)+1==27 or 
                self.checkbox_obj_list.index(i)+1==28 or self.checkbox_obj_list.index(i)+1==29 or 
                self.checkbox_obj_list.index(i)+1==30 or self.checkbox_obj_list.index(i)+1==31 or
                self.checkbox_obj_list.index(i)+1==57 or self.checkbox_obj_list.index(i)+1==58 or
                self.checkbox_obj_list.index(i)+1==59 or self.checkbox_obj_list.index(i)+1==60 or
                self.checkbox_obj_list.index(i)+1==61 or self.checkbox_obj_list.index(i)+1==62):
                    i.set_active(False)
    
    # turns on all checkboxes
    def on_all_on_clicked(self, widget):
        # turn all checkboxes on
        for i in self.checkbox_obj_list:
            if(self.actuate_thighs.get_active()):
                i.set_active(True)
            else:
                if not (self.checkbox_obj_list.index(i)+1==26 or self.checkbox_obj_list.index(i)+1==27 or 
                self.checkbox_obj_list.index(i)+1==28 or self.checkbox_obj_list.index(i)+1==29 or 
                self.checkbox_obj_list.index(i)+1==30 or self.checkbox_obj_list.index(i)+1==31 or
                self.checkbox_obj_list.index(i)+1==57 or self.checkbox_obj_list.index(i)+1==58 or
                self.checkbox_obj_list.index(i)+1==59 or self.checkbox_obj_list.index(i)+1==60 or
                self.checkbox_obj_list.index(i)+1==61 or self.checkbox_obj_list.index(i)+1==62):
                    i.set_active(True)

    # inverts the checkbox selection
    def on_invert_clicked(self, widget):
        # invert the checkbox selection
        for i in self.checkbox_obj_list:
            if self.checkbox_obj_list.index(i) in range(28) or self.checkbox_obj_list.index(i) in range(31,59):
                if(self.actuate_thighs.get_active()):
                    if(i.get_active()):
                        i.set_active(False)
                    else:
                        i.set_active(True)
                else:
                    if not (self.checkbox_obj_list.index(i)+1==26 or self.checkbox_obj_list.index(i)+1==27 or 
                    self.checkbox_obj_list.index(i)+1==28 or self.checkbox_obj_list.index(i)+1==29 or 
                    self.checkbox_obj_list.index(i)+1==30 or self.checkbox_obj_list.index(i)+1==31 or
                    self.checkbox_obj_list.index(i)+1==57 or self.checkbox_obj_list.index(i)+1==58 or
                    self.checkbox_obj_list.index(i)+1==59 or self.checkbox_obj_list.index(i)+1==60 or
                    self.checkbox_obj_list.index(i)+1==61 or self.checkbox_obj_list.index(i)+1==62):
                        if(i.get_active()):
                            i.set_active(False)
                        else:
                            i.set_active(True)
                                        
    # connects to the or disconnects from the serial port
    def on_connectbutton_toggled(self, widget):
        # open/close serial port
        if widget.get_active():                     #Connection button status
            # if its already open close it
            if self.serialport.is_open:
                self.serialport.close()
            self.display_all("Opening"+self.serialport.port+"@"+str(self.serialport.baudrate)+"baud")
            widget.set_label("Disconnect")
            self.serialport.open()                      #Opening serial port for user selected port and baudratebox
            self.display_all("connected")
            self.display_all("Reading data..")
            # self.start()                            
            self.start_time = time.time()               #Recording time when connect button is pressed 
            self.start_datetime = datetime.now()        # Recording date and time when its connected
        else:
            if self.serialport.is_open:
                self.serialport.close()                 #Closing serial port
            self.display_all("Serial port closed !")
            widget.set_label("Connect")
    
    # refreshes the list of available serial ports
    def on_refreshbutton_clicked(self,widget):
        ports = list(port_list.comports())
        comportlist = [str(i.device) for i in ports ]
        self.comportlstore = gtk.ListStore(int, str)
        self.comportbox.set_model(None)
        self.make_comboboxlist(self.comportlstore, self.comportbox, comportlist, defaultindex = 1)
    
    # aborts the present operation and resets the controller 
    def on_abortbutton_clicked(self,widget):
        # close the serial port
        if self.serialport.is_open:
            self.serialport.close()
        # open the port again
        self.display_all("Opening"+self.serialport.port+"@"+str(self.serialport.baudrate)+"baud")
        self.serialport.open()                      #Opening serial port for user selected port and baudratebox
        self.display_all("connected")
        self.display_all("Reading data..")                          
        self.start_time = time.time()               #Recording time when connect button is pressed 
        self.start_datetime = datetime.now()        # Recording date and time when its connected

    # sets the filename to save the snapshot to the given one
    def on_snapshot_name_activate(self, widget):
        self.snapshot_file_name = widget.get_text()
        # widget.set_text('')

    # sets the filename to save the recording to the given one
    def on_rec_name_activate(self, widget):
        self.rec_file_name = widget.get_text()
        # widget.set_text('')

    # sets uniform target pressure map
    def on_uni_pressure_activate(self, widget):
        uni= float(widget.get_text())
        unit_multiplier = self.unit_sel[self.unitsbox.get_active()]
        self.setpressuremap = [uni/unit_multiplier for i in range(96)]
        widget.set_text('')
       
    # sets given comma separated pressure values to  the map
    def on_csv_activate(self, widget):
        unit_multiplier = self.unit_sel[self.unitsbox.get_active()]
        txt = widget.get_text()
        a = txt.split(',')
        if len(a)==62:
            for i in a:
                self.setpressuremap[a.index(i)] =float(i)/unit_multiplier
            widget.set_text('')
        else:
            self.display_all("Given CSV string is not properly formatted")

    # sets individual elements of target pressure map
    def on_setp_no_activate(self, widget):
        # get bubble no
        b_no = int(self.bubble_no.get_text())
        # get given pressure
        pres = float(widget.get_text())
        # get units
        unit_multiplier = self.unit_sel[self.unitsbox.get_active()]
        # change set pressure map
        self.setpressuremap[b_no-1] = pres/unit_multiplier
    
    # saves the snapshot into an excel file with given name
    def on_snapshot_clicked(self, snapshot):
        # needs to be overhauled
        if self.serialport.is_open:
            if not hasattr(self,'snapshot_file_name'):
                self.snapshot_file_name = self.snapshot_name.get_text()
            self.workbook = xlsxwriter.Workbook(self.snapshot_file_name+'.xlsx')          #Creating excel file using xlswriter module
            self.worksheet = self.workbook.add_worksheet()          #Creating worksheet in the created excel file
            self.worksheet.write('A1', 'Bubble No.')                #Writing to the column    
            self.worksheet.write('B1', 'Pressure')                  #Writing to the column
            self.worksheet.write(63,0, "Arduino frequency (Hz)" )
            self.worksheet.write(64,0, "Daq frequency (Hz)" )
            self.worksheet.write(65,0, "units" )
            self.worksheet.write(65,1,self.pressure_unit)
            self.worksheet.write(66,0, "Snapshot time" )
            self.worksheet.write(66,1,unicode(datetime.now()))
            self.display_all("snapshot saved to "+ self.snapshot_file_name)

            for i in range(0, 2):                                   #For loop to select column in the worksheet
                for j in range(0, len(self.pressure_data)):          #For loop to select row in the worksheet 
                    if(i==0):
                        self.worksheet.write(j+1, i, j+1)           #Writing bubble numbers in first column
                    if(i==1):                                       #Writing bubble pressure(psi/kPa/kg per cm^2) values in second column
                        self.worksheet.write(j+1, i, (self.pressure_data[j]))
            self.workbook.close()               #Closing the file after writing data
        else:
            self.display_all("Connect before saving")
        
    # starts and stops recording data into an excel file of given name
    def on_record_toggled(self, widget):
        if self.serialport.is_open:
            if not hasattr(self,'rec_file_name'):
                self.rec_file_name = self.rec_name.get_text()
            if widget.get_active():      
                self.workbook_rec = xlsxwriter.Workbook(self.rec_file_name+'.xlsx')          #Creating excel file using xlswriter module
                self.worksheet_rec = self.workbook_rec.add_worksheet()          #Creating worksheet in the created excel file
                self.worksheet_rec.write(0,0, "Relative time (s)")
                self.display_all("Recording " +self.rec_file_name+ "  started")
                for i in range(62):
                    self.worksheet_rec.write(i+1, 0,"Bubble"+ str(i+1))
                self.worksheet_rec.write(63,0, "Arduino frequency (Hz)" )
                self.worksheet_rec.write(64,0, "Daq frequency (Hz)" )
                self.worksheet_rec.write(65,0, "units" )
                self.worksheet_rec.write(65,1,self.pressure_unit)
                self.worksheet_rec.write(66,0, "Reference time" )
                self.worksheet_rec.write(66,1,unicode(self.start_datetime))
                widget.set_label("Stop")
                self.isrecording = True                                                        
            else:
                widget.set_label("Record")
                self.display_all("Recording " +self.rec_file_name+ "  saved !")
                self.workbook_rec.close()
                self.isrecording = False
                self.rec_counter = 1
        else:
            self.display_all("Connect before saving")
            
    # sets the comport
    def on_comportbox_changed(self, widget):
        # When comport is selected
        if not hasattr(self, "serialport"):
            self.serialport = serial.Serial()
        index = widget.get_active()
        model = widget.get_model()
        temp = model[index][1]
        self.serialport.port = temp.split(" ")[0]
        self.display_all(self.serialport.port+" selected")

    # sets the display units
    def on_unitsbox_changed(self, widget):
        # When units are selected
        index = widget.get_active()
        model = widget.get_model()
        current_unit = model[index][1]
        self.pressure_unit = current_unit
        self.display_all("Units changed to "+ current_unit)
        self.colorbar.ax.set_title(current_unit)
    

        # When display type is changed
        index = widget.get_active()
        model = widget.get_model()
        current_display_type = model[index][1]
        self.method = current_display_type
        self.display_all("Display type changed to "+ current_display_type)

    # resets the view angle to top view when clicked
    def on_resetviewbutton_clicked(self,widget):
        # set initial view angle
        self.ax.view_init(90, 90)
        # redraw the plot 
        self.canvas.draw()

    # sets a uniform distribution of 0.5 psi, top-down wise
    def on_half_psi_button_clicked(self,widget):
        # self.setpressuremap = [0.3 for i in range(96)]
        # self.setpressuremap[47] = 0.7
        for i in self.checkbox_obj_list:
            i.set_active(True)
        # # widget.set_label("Inflating..")
        # self.wait_for_completion("bs")
        # # widget.set_label("Inflate again")
        for i in range(10):
            self.send_packet()
        self.serialport.write('bo20000'+'\n')

    # sets a uniform distribution of 0.5 psi, top-down wise
    def on_equalize_button_clicked(self,widget):
        if self.serialport.is_open :
            self.serialport.flushInput()
            self.serialport.flushOutput()
            self.serialport.write('br'+'\n')
            self.serialport.flushInput()
            self.serialport.flushOutput()
        # widget.set_label("Inflate again")

# sets a uniform distribution of 0.5 psi, top-down wise
    def on_rest_button_clicked(self,widget):
        if self.serialport.is_open :
            self.serialport.flushInput()
            self.serialport.flushOutput()
            self.serialport.write('b*'+'\n')
            self.serialport.flushInput()
            self.serialport.flushOutput()

    # commands the controller to perform various tasks
    def on_gobutton_clicked(self,widget):
        index = self.functionbox.get_active()
        msglist = ["bf","bg","be","bx","bc","bd","bi","bj","bt","br"]
        # "Bottom-up regulate", "Top-down regulate","Open all", "Close all", "Open 1-25 ", "Close 1-25 ", "Open 26-32 ", "Close 26-32 ", "Toggle test loop"
        # print msglist[index]
        # print index
        if self.serialport.is_open :
            self.serialport.flushInput()
            self.serialport.flushOutput()
            self.serialport.write(msglist[index]+'\n')
            self.serialport.flushInput()
            self.serialport.flushOutput()
            # print("Done Once")
            if index==0 or index==1:  
                while self.controller_state==0:
                    # print("In loop",index)
                    self.send_packet()
                    self.serialport.flushInput()
                    self.serialport.flushOutput()
                    self.serialport.write(msglist[index]+'\n')
                    self.serialport.flushInput()
                    self.serialport.flushOutput()
                    self.collect_serialdata()
                self.controller_state=0

    # sends the typed message through the open serial port 
    def on_write_buffer_activate(self, widget):
        if self.serialport.is_open:
            self.serialport.flushInput()
            self.serialport.flushOutput()
            self.serialport.write(widget.get_text()+'\n')
            self.serialport.flushInput()
            self.serialport.flushOutput()  
            widget.set_text("")

    # sets the function box value and following function
    def on_functionbox_changed(self,widget):
        index = widget.get_active()
        msglist = ["bf","bg","be","bx","bc","bd","bi","bj","bt","br"]
        # "Bottom-up regulate", "Top-down regulate","Open all", "Close all", "Open 1-25 ", "Close 1-25 ", "Open 26-32 ", "Close 26-32 ", "Toggle test loop"
        # print msglist[index]
        # print index
        if self.serialport.is_open :
            self.serialport.flushInput()
            self.serialport.flushOutput()
            self.serialport.write(msglist[index]+'\n')
            self.serialport.flushInput()
            self.serialport.flushOutput()
            # print("Done Once")
            if index==0 or index==1:  
                while self.controller_state==0:
                    # print("In loop",index)
                    self.send_packet()
                    self.serialport.flushInput()
                    self.serialport.flushOutput()
                    self.serialport.write(msglist[index]+'\n')
                    self.serialport.flushInput()
                    self.serialport.flushOutput()
                    self.collect_serialdata()
                self.controller_state=0
            self.serialport.flushInput()
            self.serialport.flushOutput()
            self.serialport.write(msglist[index]+'\n')
            self.serialport.flushInput()
            self.serialport.flushOutput()

    # sends the packet with target pressure map and flags
    def send_packet(self):
        coded_bytes1 = 1   # make LSB 1
        coded_bytes2 = 1
        # first half masks
        for i in range(48):
            mask = 1
            if self.checkbox_obj_list[i].get_active():
                mask = mask<<i+1
            coded_bytes1 = coded_bytes1|mask
        # second half masks
        for i in range(48):
            mask = 1
            if self.checkbox_obj_list[i+48].get_active():
                mask = mask<<i+1
            coded_bytes2 = coded_bytes2|mask

        self.sent_packet = "b"
        lowresmap = [int((i/14.5)*1023) for i in self.setpressuremap]
        self.sent_packet += struct.pack('%sH' %len(lowresmap), *lowresmap )
        self.sent_packet += struct.pack('Q', coded_bytes1)
        self.sent_packet += struct.pack('Q', coded_bytes2)
        self.sent_packet += '\n'
        self.serialport.flushInput()
        self.serialport.flushOutput()
        self.serialport.write(self.sent_packet)
        self.serialport.flushInput()
        self.serialport.flushOutput()

    def on_operation_clicked(self, widget):
        if gtk.Buildable.get_name(widget) == 'auto_offload' or gtk.Buildable.get_name(widget) == 'manul_offload':
            self.last_was_offload = True
        else:
            self.last_was_offload = False

    # state machine for manual offloading which is called by the manual offloading button
    def on_manual_offload_clicked(self,widget):
        # state is advanced every time the button is pressed
        self.offload_state = 0
        self.offload_state +=1
        if self.offload_state==1:
            print "Offload state 1:"
            print "blank area selction and get the offloading areas "
            # clear the bubble selection
            for i in self.checkbox_obj_list:
                i.set_active(False)
            # set the number of offlaoded bubbles to zero
            self.na = 0
            self.nb = 0
            # compute units
            unit_multiplier = self.unit_sel[self.unitsbox.get_active()]
            # set offloading pressures depending on regions
            for i in range(62):
                if i in range(25,31) or i in range(56,62):
                    self.setpressuremap[i] = unit_multiplier* self.rhob
                else:
                    self.setpressuremap[i] = unit_multiplier* self.rhoa
            widget.set_label("Select areas n click")
        
        if self.offload_state==2:
            print "Offload state 2"
            print "obtain clicked areas"
            # populate the list of bubbles to be offloaded
            self.off_list = list()
            sumP = 0    # temporary sum of pressures
            for i in self.checkbox_obj_list:
                if i.get_active():
                    if self.checkbox_obj_list.index(i) in range(25) or self.checkbox_obj_list.index(i) in range(31,56): 
                        self.na +=1
                        sumP -= self.rhoa
                    if self.checkbox_obj_list.index(i) in range(25,28) or self.checkbox_obj_list.index(i) in range(56,59):
                        self.nb +=1
                        sumP -= self.alpha * self.rhob
                    self.off_list.append(self.checkbox_obj_list.index(i))
                else:
                    if self.checkbox_obj_list.index(i) in range(25) or self.checkbox_obj_list.index(i) in range(31,56):
                        sumP += self.pressure_data[self.checkbox_obj_list.index(i)]
                    if self.checkbox_obj_list.index(i) in range(25,28) or self.checkbox_obj_list.index(i) in range(56,59):
                        sumP += self.alpha * self.pressure_data[self.checkbox_obj_list.index(i)]    
            self.sumP = sumP
            print "note down average pressure"
            self.initial_pressure = self.pressure_data
            print "send setpressure packet"
            self.send_packet()
            print "give regulate command"
            widget.set_label("Offloading selected areas")
            # keeps sending the regulate command until the controller starts regulating
            while self.controller_state==0:
            # for i in range(10):
                self.send_packet()
                self.serialport.flushInput()
                self.serialport.flushOutput()
                self.serialport.write("bg"+'\n')
                self.serialport.flushInput()
                self.serialport.flushOutput()
                self.collect_serialdata()
                print "offloading command given"                
            # go on to the next state when the controller starts

            #automatically move on to the final state
            # wait here until the controller is finished offloading
            while self.controller_state!=0:
                self.collect_serialdata()
                print "manual offloading started"
            # after the offloading is done reinitialize the state machine
            print "offloading finished"
            print "Offload state 0 again"
            print "get here when regulation is done, reset !"
            widget.set_label("Done... manual offloading again")
            self.after_pressure = self.pressure_data
            self.last_was_offload = True

       # state machine for redistribution which is called by corresponding button
  
    def on_redistribute_clicked(self,widget):
        # state is advanced every time the button is pressed 
        # print "Redistribute state 1: "
        # print "select all the area excluding boundaries"
        #activate all inner bubbles, deactive exterior ones
                    
        # compute unit multiplier based on user selection
        unit_multiplier = self.unit_sel[self.unitsbox.get_active()]
        for i in self.checkbox_obj_list:
            if self.checkbox_obj_list.index(i) in range(28) or self.checkbox_obj_list.index(i) in range(31,59):
                i.set_active(True)
                self.setpressuremap[self.checkbox_obj_list.index(i)] = unit_multiplier*sum(self.pressure_data)/62
            else:
                i.set_active(False)
        # widget.set_label("Redistributing")
        # Give command for redistribute among all bubbles
        self.wait_for_completion("br")
        # widget.set_label("Redistribute again")

    #call to redistribute to 0.1 psi under the average
    def on_redistribute_increase_clicked(self,widget):
        self.redistribute_offset(0.1)

    #call to redistribute to 0.1 psi over the average
    def on_redistribute_decrease_clicked(self,widget):
        self.redistribute_offset(-0.1)

    def redistribute_offset(self, offset):
        # compute uniform target pressures for each region 
        # self.sumP = 0    # temporary sum of pressures
        # self.active_number = 0 # temporary count of bubbles active in current redistribute call
        # for i in self.checkbox_obj_list:
        #     if i.get_active():
        #         self.sumP += self.pressure_data[self.checkbox_obj_list.index(i)]
        #         self.active_number += 1
        # if self.active_number == 0:
            # return 
        # self.P_avg = self.sumP/self.active_number
        self.P_avg = self.setpressuremap[0]
        self.P_avg += offset
        if(self.P_avg < self.min_pressure and offset < 0):
            self.display_all("Pressure cannot be set below " + str(self.min_pressure))
            return 
        print "set the pressure to the %f" % self.P_avg
        for i in self.checkbox_obj_list:
            if i.get_active():
                    self.setpressuremap[self.checkbox_obj_list.index(i)] = self.P_avg  
        for index in range(50):
            self.send_packet()
            self.serialport.flushOutput()

        self.wait_for_completion("bg")

    def on_redistribute_after_offload_clicked(self,widget):
        if self.last_was_offload == False:
            print 'This can only be called after offloading'
            return
        #flip pressure map set by offload functions
        for i in self.checkbox_obj_list:
            if self.checkbox_obj_list.index(i) in range(28) or self.checkbox_obj_list.index(i) in range(31,59):
                if i.get_active():
                    i.set_active(False)
                else:
                    i.set_active(True)
            else:
                i.set_active(False)
        widget.set_label("Redistributing")
        # Give command for redistribute among all bubbles
        self.wait_for_completion("br")
        widget.set_label("Redistribute after offload again")

    #call to redistribute to 0.1 psi under the average
    def on_redistribute_after_increase_clicked(self,widget):
        self.redistribute_after_offset(0.1)

    #call to redistribute to 0.1 psi under the average
    def on_redistribute_after_decrease_clicked(self,widget):
        self.redistribute_after_offset(-0.1)

    def redistribute_after_offset(self, offset):
        if self.last_was_offload == False:
            self.display_all("This can only be called after offloading")
            print("Ahoy")
            return
        else:
            #flip pressure map set by offload functions
            for i in self.checkbox_obj_list:
                if self.checkbox_obj_list.index(i) in range(28) or self.checkbox_obj_list.index(i) in range(31,59):
                    if i.get_active():
                        i.set_active(False)
                    else:
                        i.set_active(True)
                else:
                    i.set_active(False)

            #redistribute with given offset after setting pressuremap
            self.redistribute_offset(offset)

    #loops to wait until packets have been recieved and orders executed 
    def wait_for_completion(self, regulateFunction):
        self.send_packet()
        print("Sending command until it starts..")
        while self.controller_state==0:
            # print "Redistribution command given again"
            self.send_packet()
            self.serialport.flushInput()
            self.serialport.flushOutput()
            self.serialport.write(regulateFunction+'\n')
            self.serialport.flushInput()
            self.serialport.flushOutput()
            self.collect_serialdata()
            self.update_gui()
            self.update_plot()
        # go on to the next state when the controller starts
        print "operation started"
                    
        # print "Waiting for command to finish"
        # # wait here until the controller is finished offloading
        # while self.controller_state!=0:
        #     self.serialport.flushInput()
        #     self.serialport.flushOutput()
        #     self.collect_serialdata()
        #     self.update_gui()
        #     self.update_plot()
        # print("Command finished..")

    # state machine for automatic offloading which is called by corresponding button
    def on_auto_offload_clicked(self,widget):
        # state is advanced every time the button is pressed
        self.auto_state +=1
        unit_multiplier = self.unit_sel[self.unitsbox.get_active()]
        if self.auto_state == 1:
            print "Auto state 1:"
            print "confirm areas to offload"
            widget.set_label("confirm areas to offload ")
            self.on_scale2_valuechanged(self.h_scale2)
            # set offloading pressures depending on regions
            for i in range(62):
                if i in range(25,31) or i in range(56,62):
                    self.setpressuremap[i] = unit_multiplier* self.rhob
                else:
                    self.setpressuremap[i] = unit_multiplier* self.rhoa
            # the user is expected to change the slider to desired position and then click the button again to move onto next state
        
        if self.auto_state == 2:
            print "Auto state 2:"
            print "obtain selected areas"
            # populate the list of bubbles to be offloaded
            self.off_list = list()
            sumP = 0    # temporary sum of pressures
            for i in self.checkbox_obj_list:
                if i.get_active():
                    if self.checkbox_obj_list.index(i) in range(25) or self.checkbox_obj_list.index(i) in range(31,56): 
                        self.na +=1
                        sumP -= self.rhoa
                    else:
                        self.nb +=1
                        sumP -= self.alpha * self.rhob
                    self.off_list.append(self.checkbox_obj_list.index(i))
                else:
                    if self.checkbox_obj_list.index(i) in range(25) or self.checkbox_obj_list.index(i) in range(31,56):
                        sumP += self.pressure_data[self.checkbox_obj_list.index(i)]
                    else:
                        sumP += self.alpha * self.pressure_data[self.checkbox_obj_list.index(i)]    
            self.sumP = sumP
            print "note down average pressure"
            self.initial_pressure = self.pressure_data
            print "send setpressure packet"
            self.send_packet()
            print "give regulate command"
            widget.set_label("Offloading selected areas")
            # keeps sending the regulate command until the controller starts regulating
            for index in range(50):
                self.send_packet()
                self.serialport.flushInput()
                self.serialport.flushOutput()
            self.wait_for_completion("bg")
            # go on to the next state when the controller starts

            #move to state 3 automatically after finishing state 2
            print "Auto state 3:"
            print "get here when offload is done"
            widget.set_label("Done... auto offload again ?")
            self.last_was_offload = True
    
    # thresholding paramater change prompts a change in bubble selection map
    def on_scale_valuechanged(self,widget):
        # get the current value
        val =  widget.get_value()
        # print val
        # if auto offloading is initiated
        if self.auto_state==1:
            # clear the selection area
            for i in self.checkbox_obj_list:
                i.set_active(False)
                        # set the number of offlaoded bubbles to zero
            self.na = 0
            self.nb = 0
            # self.display_all('Choose the parameter using the slider and click the auto button again')
            # compute min max and consequently the threshold pressure
            pmin = min(self.pressure_data)
            pmax = max(self.pressure_data)
            pthres = (pmax*(100-val) + pmin*(val))/100
            # mark the bubbles above a certain threshold for offloading 
            for i in self.checkbox_obj_list:
                # compensate the algorithm for varying areas
                if self.checkbox_obj_list.index(i) in range(25) or self.checkbox_obj_list.index(i) in range(31,56):
                    if self.pressure_data[self.checkbox_obj_list.index(i)]>pthres:
                        i.set_active(True)
                else:
                    if self.pressure_data[self.checkbox_obj_list.index(i)]>self.alpha*pthres:
                        i.set_active(True)

     # thresholding paramater change prompts a change in bubble selection map
    
    # number of offloaded bubbles change...
    def on_scale2_valuechanged(self,widget):
        num =  int(widget.get_value())   
        # print val
        # if auto offloading is initiated
        if self.auto_state==1:
            # clear the selection area
            for i in self.checkbox_obj_list:
                i.set_active(False)
            # set the number of offlaoded bubbles to zero
            # compute min max and consequently the threshold pressure
            a = self.pressure_data
            # mark the bubbles above a certain threshold for offloading 
            ind = sorted(range(len(a)), key=lambda i: a[i], reverse=True)[:num]
            # print ind
            for i in self.checkbox_obj_list:
                 if self.checkbox_obj_list.index(i) in range(25) or self.checkbox_obj_list.index(i) in range(31,56):
                    # compensate the algorithm for varying areas
                    if self.checkbox_obj_list.index(i) in ind:
                        i.set_active(True)
                    else:
                        i.set_active(False)
                

    # docks/undocks the plot window
    def on_dockbutton_toggled(self,widget):
        if widget.get_active():
            self.plotbox.remove(self.canvas)
            self.plotbox.remove(self.toolbar)
            self.remoteplotbox.pack_start(self.canvas)
            self.remoteplotbox.pack_start(self.toolbar,False, False)
            self.plot_window.show_all()
            widget.set_label("Dock")
        else:
            self.remoteplotbox.remove(self.canvas)
            self.remoteplotbox.remove(self.toolbar)
            self.plotbox.pack_start(self.canvas)
            self.plotbox.pack_start(self.toolbar,False, False)
            self.plot_window.hide()
            widget.set_label("Undock")

    def on_plot_window_destroy(self, *args):
        print "Closed plot window"
        self.remoteplotbox.remove(self.canvas)
        self.remoteplotbox.remove(self.toolbar)
        self.plotbox.pack_start(self.canvas)
        self.plotbox.pack_start(self.toolbar,False, False)
        self.plot_window.hide()
        self.dockbutton.set_label("Undock")

    # turn all checkboxes on    
    def on_b_on_clicked(self,widget):
        for i in self.checkbox_obj_list:
            if (self.checkbox_obj_list.index(i)+1==29 or self.checkbox_obj_list.index(i)+1==30 or 
                self.checkbox_obj_list.index(i)+1==31 or self.checkbox_obj_list.index(i)+1==60 or 
                self.checkbox_obj_list.index(i)+1==61 or self.checkbox_obj_list.index(i)+1==62):
                    i.set_active(True)

    # turn all chec  kboxes on
    def on_b_off_clicked(self,widget):
        for i in self.checkbox_obj_list:
            if (self.checkbox_obj_list.index(i)+1==29 or self.checkbox_obj_list.index(i)+1==30 or 
                self.checkbox_obj_list.index(i)+1==31 or self.checkbox_obj_list.index(i)+1==60 or 
                self.checkbox_obj_list.index(i)+1==61 or self.checkbox_obj_list.index(i)+1==62):
                    i.set_active(False)

    def on_weightbox_changed(self,widget):
        pass
        

if __name__ == "__main__":
    GUI_object = GUI()
    gtk.main()
