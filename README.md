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
25000
50000
125000
250000
500000
625000
1250000
1562500
2500000

Actual bandwidth will be ~80% of the setted samplerate. The output format - is interleaved 32bit integer values of: QIQIQI...


Usage examples:


TODO:
 * Implement KSH-148 (KSH-146) downconverter interface by I2C.
