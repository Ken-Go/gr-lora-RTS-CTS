#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: RTS_Send
# Author: zjhao
# GNU Radio version: v3.8.5.0-6-g57bd109d

from distutils.version import StrictVersion

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print("Warning: failed to XInitThreads()")

from gnuradio import blocks
from gnuradio import filter
from gnuradio.filter import firdes
from gnuradio import gr
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import uhd
import time
from gnuradio.filter import pfb
import lora
import math

from gnuradio import qtgui

class RTS_Send(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "RTS_Send")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("RTS_Send")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "RTS_Send")

        try:
            if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
                self.restoreGeometry(self.settings.value("geometry").toByteArray())
            else:
                self.restoreGeometry(self.settings.value("geometry"))
        except:
            pass

        ##################################################
        # Variables
        ##################################################
        self.spreading_factor = spreading_factor = 7
        self.samp_rate = samp_rate = 1e6
        self.payload_len = payload_len = 8
        self.offset = offset = 0
        self.ldr = ldr = False
        self.header = header = True
        self.frequency = frequency = 470.3e6
        self.crc = crc = True
        self.code_rate = code_rate = 4
        self.bw = bw = 250e3

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_source_0 = uhd.usrp_source(
            ",".join(("", "")),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
        )
        self.uhd_usrp_source_0.set_center_freq(frequency, 0)
        self.uhd_usrp_source_0.set_gain(30, 0)
        self.uhd_usrp_source_0.set_antenna('RX2', 0)
        self.uhd_usrp_source_0.set_bandwidth(bw, 0)
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        # No synchronization enforced.
        self.uhd_usrp_sink_0 = uhd.usrp_sink(
            ",".join(("", "")),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
            '',
        )
        self.uhd_usrp_sink_0.set_center_freq(frequency, 0)
        self.uhd_usrp_sink_0.set_gain(30, 0)
        self.uhd_usrp_sink_0.set_antenna('TX/RX', 0)
        self.uhd_usrp_sink_0.set_bandwidth(bw, 0)
        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_0.set_time_unknown_pps(uhd.time_spec())
        self.pfb_arb_resampler_xxx_0_0 = pfb.arb_resampler_ccf(
            samp_rate/bw,
            taps=None,
            flt_size=32)
        self.pfb_arb_resampler_xxx_0_0.declare_sample_delay(0)
        self.pfb_arb_resampler_xxx_0 = pfb.arb_resampler_ccf(
            2*bw/samp_rate,
            taps=None,
            flt_size=32)
        self.pfb_arb_resampler_xxx_0.declare_sample_delay(0)
        self.low_pass_filter_0 = filter.interp_fir_filter_ccf(
            1,
            firdes.low_pass(
                1,
                samp_rate,
                bw/2+10e3,
                1e3,
                firdes.WIN_HAMMING,
                6.76))
        self.lora_mod_0 = lora.mod(spreading_factor, 0x34)
        self.lora_encode_0 = lora.encode(spreading_factor, code_rate, False, ldr, header)
        self.lora_demod_0 = lora.demod(spreading_factor, header, payload_len, code_rate, crc, ldr, 25.0, 16, 0, 4, 2)
        self.lora_decode_0 = lora.decode(spreading_factor, header, payload_len, code_rate, crc, ldr)
        self.lora_RTS_Sender_0 = lora.RTS_Sender(1, 100)
        self.blocks_socket_pdu_0 = blocks.socket_pdu('TCP_SERVER', '127.0.0.1', '52002', 10000, False)
        self.blocks_rotator_cc_0 = blocks.rotator_cc((2 * math.pi * offset) / samp_rate)
        self.blocks_message_debug_0 = blocks.message_debug()


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.blocks_socket_pdu_0, 'pdus'), (self.lora_RTS_Sender_0, 'inmsg'))
        self.msg_connect((self.lora_RTS_Sender_0, 'outmsg'), (self.lora_encode_0, 'in'))
        self.msg_connect((self.lora_decode_0, 'out'), (self.blocks_message_debug_0, 'print_pdu'))
        self.msg_connect((self.lora_decode_0, 'out'), (self.lora_RTS_Sender_0, 'inCTS'))
        self.msg_connect((self.lora_decode_0, 'header'), (self.lora_demod_0, 'header'))
        self.msg_connect((self.lora_demod_0, 'out'), (self.lora_decode_0, 'in'))
        self.msg_connect((self.lora_encode_0, 'out'), (self.lora_mod_0, 'in'))
        self.connect((self.blocks_rotator_cc_0, 0), (self.pfb_arb_resampler_xxx_0_0, 0))
        self.connect((self.lora_RTS_Sender_0, 0), (self.low_pass_filter_0, 0))
        self.connect((self.lora_mod_0, 0), (self.blocks_rotator_cc_0, 0))
        self.connect((self.low_pass_filter_0, 0), (self.pfb_arb_resampler_xxx_0, 0))
        self.connect((self.pfb_arb_resampler_xxx_0, 0), (self.lora_demod_0, 0))
        self.connect((self.pfb_arb_resampler_xxx_0_0, 0), (self.uhd_usrp_sink_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.lora_RTS_Sender_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "RTS_Send")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_spreading_factor(self):
        return self.spreading_factor

    def set_spreading_factor(self, spreading_factor):
        self.spreading_factor = spreading_factor

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.blocks_rotator_cc_0.set_phase_inc((2 * math.pi * self.offset) / self.samp_rate)
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.samp_rate, self.bw/2+10e3, 1e3, firdes.WIN_HAMMING, 6.76))
        self.pfb_arb_resampler_xxx_0.set_rate(2*self.bw/self.samp_rate)
        self.pfb_arb_resampler_xxx_0_0.set_rate(self.samp_rate/self.bw)
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)
        self.uhd_usrp_source_0.set_samp_rate(self.samp_rate)

    def get_payload_len(self):
        return self.payload_len

    def set_payload_len(self, payload_len):
        self.payload_len = payload_len

    def get_offset(self):
        return self.offset

    def set_offset(self, offset):
        self.offset = offset
        self.blocks_rotator_cc_0.set_phase_inc((2 * math.pi * self.offset) / self.samp_rate)

    def get_ldr(self):
        return self.ldr

    def set_ldr(self, ldr):
        self.ldr = ldr

    def get_header(self):
        return self.header

    def set_header(self, header):
        self.header = header

    def get_frequency(self):
        return self.frequency

    def set_frequency(self, frequency):
        self.frequency = frequency
        self.uhd_usrp_sink_0.set_center_freq(self.frequency, 0)
        self.uhd_usrp_source_0.set_center_freq(self.frequency, 0)

    def get_crc(self):
        return self.crc

    def set_crc(self, crc):
        self.crc = crc

    def get_code_rate(self):
        return self.code_rate

    def set_code_rate(self, code_rate):
        self.code_rate = code_rate

    def get_bw(self):
        return self.bw

    def set_bw(self, bw):
        self.bw = bw
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.samp_rate, self.bw/2+10e3, 1e3, firdes.WIN_HAMMING, 6.76))
        self.pfb_arb_resampler_xxx_0.set_rate(2*self.bw/self.samp_rate)
        self.pfb_arb_resampler_xxx_0_0.set_rate(self.samp_rate/self.bw)
        self.uhd_usrp_sink_0.set_bandwidth(self.bw, 0)
        self.uhd_usrp_source_0.set_bandwidth(self.bw, 0)





def main(top_block_cls=RTS_Send, options=None):

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()

    tb.show()

    def sig_handler(sig=None, frame=None):
        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    def quitting():
        tb.stop()
        tb.wait()

    qapp.aboutToQuit.connect(quitting)
    qapp.exec_()

if __name__ == '__main__':
    main()
