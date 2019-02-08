#!/usr/bin/env python
from OpenGL.GLUT import *
import sys

glutInit(sys.argv)

# Select type of Display mode:
# Double buffer
# RGBA color
# Depth buffer
# Alpha blending
glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_ALPHA)

# get a 640 x 480 window
glutInitWindowSize(640, 480)

# the window starts at the upper left corner of the screen
glutInitWindowPosition(0, 0)

# Open a window
window = glutCreateWindow("Testing DRI")
