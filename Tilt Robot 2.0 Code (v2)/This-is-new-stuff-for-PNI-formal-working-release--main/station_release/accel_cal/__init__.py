"""accel_cal — the pure math core of the calibration station.

This package knows NOTHING about hardware, serial ports, or threads. Give it
numbers, get calibration answers. Everything in here can be tested with just
numpy, no equipment. (That's the whole point — see the walkthrough.)
"""
