#!/usr/bin/env python2
##################################################
# GNU Radio Python Flow Graph
# Title: Top Block
# Generated: Wed Jan 20 00:25:55 2016
##################################################

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import wxgui
from gnuradio.eng_option import eng_option
from gnuradio.fft import window
from gnuradio.filter import firdes
from gnuradio.wxgui import fftsink2
from grc_gnuradio import wxgui as grc_wxgui
from optparse import OptionParser
import qs1r
import wx


class top_block(grc_wxgui.top_block_gui):

    def __init__(self):
        grc_wxgui.top_block_gui.__init__(self, title="Top Block")
        _icon_path = "/usr/share/icons/hicolor/32x32/apps/gnuradio-grc.png"
        self.SetIcon(wx.Icon(_icon_path, wx.BITMAP_TYPE_ANY))

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 125000

        ##################################################
        # Blocks
        ##################################################
        self.wxgui_fftsink2_0 = fftsink2.fft_sink_c(
        	self.GetWin(),
        	baseband_freq=0,
        	y_per_div=10,
        	y_divs=10,
        	ref_level=0,
        	ref_scale=2.0,
        	sample_rate=samp_rate,
        	fft_size=1024,
        	fft_rate=15,
        	average=False,
        	avg_alpha=None,
        	title="FFT Plot",
        	peak_hold=False,
        )
        self.Add(self.wxgui_fftsink2_0.win)
        self.qs1r_qs1r_src_0 = qs1r.qs1r_src(10000000, 125000, True, True, False, 0)
        self.blocks_int_to_float_0 = blocks.int_to_float(1, 262144)
        self.blocks_float_to_complex_0 = blocks.float_to_complex(1)
        self.blocks_deinterleave_0 = blocks.deinterleave(gr.sizeof_float*1, 1)

        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_deinterleave_0, 0), (self.blocks_float_to_complex_0, 0))    
        self.connect((self.blocks_deinterleave_0, 1), (self.blocks_float_to_complex_0, 1))    
        self.connect((self.blocks_float_to_complex_0, 0), (self.wxgui_fftsink2_0, 0))    
        self.connect((self.blocks_int_to_float_0, 0), (self.blocks_deinterleave_0, 0))    
        self.connect((self.qs1r_qs1r_src_0, 0), (self.blocks_int_to_float_0, 0))    


    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.wxgui_fftsink2_0.set_sample_rate(self.samp_rate)


if __name__ == '__main__':
    parser = OptionParser(option_class=eng_option, usage="%prog: [options]")
    (options, args) = parser.parse_args()
    tb = top_block()
    tb.Start(True)
    tb.Wait()
