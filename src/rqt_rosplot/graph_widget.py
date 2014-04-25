#!/usr/bin/env python
# Ros Plot Tool by fly.fontana@gmail.com
# this is a modified version of the rqt_plot tool written by Dorian Scholz

import math
import sys

from PyQt4.QtCore import *
from PyQt4.QtGui import *
import PyQt4.Qwt5 as Qwt

from numpy import arange, zeros, concatenate 
from scipy.constants.codata import precision


# create real QwtDataPlot class
class GraphWidget(Qwt.QwtPlot):
    _colors = [Qt.red, Qt.blue, Qt.magenta, Qt.cyan, Qt.green, Qt.darkYellow, Qt.black, Qt.darkRed, Qt.gray, Qt.darkCyan]
    _num_value_saved = 1000
    _num_values_ploted = 1000

    def __init__(self, *args):
        super(GraphWidget, self).__init__(*args)
        self._legend = Qwt.QwtLegend()
        self.setCanvasBackground(Qt.white)
        self._legend.setItemMode(Qwt.QwtLegend.CheckableItem);
        self.insertLegend(self._legend, Qwt.QwtPlot.BottomLegend)
        
        self._curves = {}

        self._last_canvas_x = 0
        self._last_canvas_y = 0
        self._pressed_canvas_y = 0
        self._last_click_coordinates = None
        self._color_index = 0

        marker_axis_y = Qwt.QwtPlotMarker()
        marker_axis_y.setLabelAlignment(Qt.AlignRight | Qt.AlignTop)
        marker_axis_y.setLineStyle(Qwt.QwtPlotMarker.HLine)
        marker_axis_y.setYValue(0.0)
        marker_axis_y.attach(self)

        self.zoom_enabled = True

        # Initialize data
        self._newest_time = 0
        self._y_upper_limit = 0.01
        self._y_lower_limit = -0.01
        self._yaxis_updated = 0
        self._canvas_display_width = 10
        self.redraw()
        self.move_canvas(0, 0)
        self.canvas().setMouseTracking(True)
        self.canvas().installEventFilter(self)
    
    def resizeEvent(self, event):
        #super(QwtDataPlot, self).resizeEvent(event)
        Qwt.QwtPlot.resizeEvent(self, event)
        self.xlim()
        self.ylim()
        self.redraw()

    def add_curve(self, curve_id, curve_name, values_x, values_y):
        curve_id = str(curve_id)
        if self._curves.get(curve_id):
            return
        curve_object = Qwt.QwtPlotCurve(curve_name)
        curve_object.attach(self)
        curve_object.setPen(QPen(self._colors[self._color_index % len(self._colors)]))
        self._color_index += 1
        self._curves[curve_id] = {
            'name': curve_name,
            'data': zeros((2,self._num_value_saved)),
            'object': curve_object,
        }

    def remove_curve(self, curve_id):
        curve_id = str(curve_id)
        if curve_id in self._curves:
            self._curves[curve_id]['object'].hide()
            self._curves[curve_id]['object'].attach(None)
            del self._curves[curve_id]['object']
            del self._curves[curve_id]

    def update_values(self, curve_id, values_x, values_y):
        for value_x, value_y in zip(values_x, values_y):
            self.update_value(curve_id, value_x, value_y)

    def update_value(self, curve_id, value_x, value_y):
        curve_id = str(curve_id)
        # update data plot
        if curve_id in self._curves:
            # shift back
            self._curves[curve_id]['data'] = concatenate( (self._curves[curve_id]['data'][:,1:], self._curves[curve_id]['data'][:,:1] ), 1)
            self._curves[curve_id]['data'][0,-1] = float(value_x)            
            self._curves[curve_id]['data'][1,-1] = float(value_y)

            if float(value_x) > self._newest_time:
                self._newest_time = float(value_x)
            
            if self.zoom_enabled:    
                if float(value_y) > self._y_upper_limit: 
                   self._y_upper_limit = float(value_y)
                   self._yaxis_updated = 1
                    
                if float(value_y) < self._y_lower_limit :                
                    self._y_lower_limit = float(value_y)
                    self._yaxis_updated = 1
                
    def redraw(self):
        for curve_id in self._curves.keys():
            self._curves[curve_id]['object'].setData(
                self._curves[curve_id]['data'][0,-self._num_values_ploted:], 
                self._curves[curve_id]['data'][1,-self._num_values_ploted:])
        self.xlim()
        if self._yaxis_updated:
            self.ylim()
        self.replot()

    def xlim(self):
        if(self._canvas_display_width>2):        
            y_step_size = 2
        else:
            y_step_size = 0.2
        self.setAxisScale(Qwt.QwtPlot.xBottom, self._newest_time-self._canvas_display_width, self._newest_time,y_step_size)

    def ylim(self):
        y_num_ticks = self.height() / 40
        # calculate a fitting step size for nice, round tick labels, depending on the displayed value area
        y_delta = self._y_upper_limit - self._y_lower_limit
        exponent = int(math.log10(y_delta))
        presicion = -(exponent - 2)
        y_step_size = round(y_delta / y_num_ticks, presicion)
        self.setAxisScale(Qwt.QwtPlot.yLeft, self._y_lower_limit, self._y_upper_limit, y_step_size)     
        
    def move_canvas(self, delta_x, delta_y):
        #self._canvas_offset_y += delta_y * self._canvas_display_height / self.canvas().height()
        height = self._y_upper_limit - self._y_lower_limit
        self._y_lower_limit += delta_y * height / self.canvas().height()
        self._y_upper_limit += delta_y * height / self.canvas().height()        
        self.xlim()
        self.ylim()

    def zoom_y(self,zoom_factor_y):
        height = self._y_upper_limit - self._y_lower_limit
        delta_height = zoom_factor_y*height
        self._y_lower_limit += delta_height
        self._y_upper_limit -= delta_height
        self.ylim()

    def zoom_x(self,zoom_factor_x):
        self._canvas_display_width = max(1,self._canvas_display_width + zoom_factor_x*self._canvas_display_width)
        self.xlim()

    def mousePressEvent(self, event):
        self._last_canvas_x = event.x() - self.canvas().x()
        self._last_canvas_y = event.y() - self.canvas().y()
        self._pressed_canvas_y = event.y() - self.canvas().y()

    def mouseMoveEvent(self, event):
        canvas_x = event.x() - self.canvas().x()
        canvas_y = event.y() - self.canvas().y()
        if ( (event.buttons() & Qt.MiddleButton) or (event.buttons() & Qt.LeftButton ) ) :  # middle button moves the canvas
            delta_y = canvas_y - self._last_canvas_y
            self.move_canvas(0, delta_y)
            self.redraw()
        elif event.buttons() & Qt.RightButton:   # right button zooms
            delta_x = self._last_canvas_x - canvas_x
            delta_y = canvas_y - self._last_canvas_y

            zoom_factor_x = max(-0.6, min(0.6, delta_x / 40.0 ))           
            zoom_factor_y = max(-0.6, min(0.6, delta_y / 40.0 ))

            self.zoom_x(zoom_factor_x)            
            self.zoom_y(zoom_factor_y)            
            self.redraw()
        self._last_canvas_x = canvas_x
        self._last_canvas_y = canvas_y

    def wheelEvent(self, event):  # mouse wheel zooms the y-axis
        zoom_factor_y = max(-0.6, min(0.6, (event.delta() / 120) / 6.0))
        self.zoom_y(zoom_factor_y)            
        self.redraw()
    
