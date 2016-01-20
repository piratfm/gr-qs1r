QS1R Gnuradio source block
=========

This is source block for GnuRadio which implements QS1R Direct Sampling Receiver source block.
Required GnuRadio version at least 3.7.1

Build instructions:
```
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ../
make
sudo make install
```

Allowed samplerates in Hz:
* 25000
* 50000
* 125000
* 250000
* 500000
* 625000
* 1250000
* 1562500
* 2500000

Actual bandwidth will be ~85% of the setted samplerate. The output format - is interleaved 32bit integer values of: QIQIQI...


If frequency is more than 60 MHz, then additional tuner is used KSH-146(148).

Here is example of graph:
![qs1r_rx_screenshot](/examples/qs1r_rx_screenshot.png "qs1r_rx_screenshot")
