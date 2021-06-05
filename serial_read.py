# -*- coding: utf-8 -*-
"""
Created on Thu May 27 11:59:08 2021

@author: janusz
"""

import serial


def read_from_serial_():
    with serial.Serial('COM4', 9600, timeout=1) as ser:
        line = ser.readline()   # read a '\n' terminated line
    return print(line.decode('utf-8'))

read_from_serial_()