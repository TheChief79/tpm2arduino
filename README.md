tpm2arduino
===========

Tpm2 Firmware for Arduino to build an Ambilight for VDR/XBMC. It has been tested with WS2801/WS2811/WS2812B LED stripes 
and Arduino Duemilanove with FTDI onBoard.

Requirements
============

protohreads library installed in the Arduino libraries directory (http://dunkels.com/adam/pt/download.html)
FastLED library installed in the Arduino libraries directory (https://github.com/FastLED/FastLED)

VDR
===

For VDR use the seduatmo-Plugin and set the mode to tpm2.

BOBLIGHT
========

To get boblight running well, apply boblight.patch with patch -p0 <boblight.patch in boblight-source directory and setup your boblight.conf.

```
[device]
name ambilight
output /dev/ttyUSB0
channels 768
type momo
interval 20000
rate 500000
prefix C9 DA 03 00
postfix 36
debug off
```

03 00 in prefix means 768 channels. With 642 channels this value would be 02 82.