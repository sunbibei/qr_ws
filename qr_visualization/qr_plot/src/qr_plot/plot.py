import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .plot_widget import PlotWidget
from .rqt_plot_lf.src.rqt_plot_lf.plot import Plot as LFPlot
from .rqt_plot_lb.src.rqt_plot_lb.plot import Plot as LBPlot
from .rqt_plot_rf.src.rqt_plot_rf.plot import Plot as RFPlot
from .rqt_plot_rb.src.rqt_plot_rb.plot import Plot as RBPlot
from .rqt_plot_legs.src.rqt_plot_legs.plot import Plot as LegsPlot
from .rqt_plot_lf_foot.src.rqt_plot_lf_foot.plot import Plot as LfFootPlot
from .rqt_plot_lb_foot.src.rqt_plot_lf_foot.plot import Plot as LbFootPlot
from .rqt_plot_rf_foot.src.rqt_plot_lf_foot.plot import Plot as RfFootPlot
from .rqt_plot_rb_foot.src.rqt_plot_lf_foot.plot import Plot as RbFootPlot
from .rqt_plot_foots.src.rqt_plot_lf_foot.plot import  Plot as FootsPlot

class Plot(Plugin):

    def __init__(self, context):
        super(Plot, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('QrPlot')
        self._widget = PlotWidget()
        self.plot_class_all = [LFPlot(), LBPlot(), RFPlot(), RBPlot(),
                               LfFootPlot(), LbFootPlot(), RfFootPlot(), RbFootPlot(),
                               LegsPlot(), FootsPlot()]

        self.plot_widget_all = [self._widget.switch_widget_lf, self._widget.switch_widget_lb,
                                self._widget.switch_widget_rf, self._widget.switch_widget_rb,
                                self._widget.switch_widget_lf_foot, self._widget.switch_widget_lb_foot,
                                self._widget.switch_widget_rf_foot, self._widget.switch_widget_rb_foot,
                                self._widget.switch_widget_legs, self._widget.switch_widget_foots]
        
        for i in range(len(self.plot_class_all)):
            self.plot_widget_all[i](self.plot_class_all[i])
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        
        
    
    