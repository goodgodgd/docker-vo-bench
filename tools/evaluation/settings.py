import os
import sys

curdir = os.path.dirname(os.path.abspath(__file__))
if curdir not in sys.path:
    sys.path.append(curdir)

pardir = os.path.dirname(curdir)
if pardir not in sys.path:
    sys.path.append(pardir)
