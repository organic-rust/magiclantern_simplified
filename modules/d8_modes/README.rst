DIGIC 8 sensor modes
====================

Movie sensor modes for DIGIC 8 cameras (so far the M6 Mark II 1.1.1):
the Sensor mode item in the Movie menu (3x3, 2x2, 1:1), its frame rate
and the band the sensor reads (Readout: 2.39:1, 16:9, 3:2), with
settings per mode.

3x3 and 2x2 are Canon's 1080p and 4K, chosen by writing Canon's movie
size (PROP_MOVIE_PARAM) as its menu does; 1:1 is Canon's RAW burst
mode, started at REC (core preshoot.c) and recorded by mlv_lite.
Readouts other than Canon's own are made by editing Canon's LiveView
tables in RAM (sensor registers, timing, image processor heights) and
take effect when LiveView restarts.

:Author: Natasha
:License: GPL
:Summary: DIGIC 8: movie sensor modes and read-out bands
