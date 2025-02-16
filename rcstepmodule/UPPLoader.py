# Personality Loader
# Universal Personality Programmer

# Ilan E. Moyer
# July 7, 2015

# This script is inteded to be run from within a makefile.
# Loads and then runs a program provided as a command-line argument.

# Note that for now, this utility assumes that you have UPPv2Node.py, or a symlink to it, in a folder ~/symlinks/


from pygestalt import nodes
from pygestalt import interfaces
from pygestalt import config
import time
import sys, os
import random
import UPPGestaltNode

# config.verboseDebugOn()

serialInterface1 = interfaces.serialInterface(baudrate = 115200, interfaceType = 'ftdi')
programmer1 = nodes.soloGestaltNode(name = 'programmer1', interface = serialInterface1, module = UPPGestaltNode)


for programmer in [programmer1]:
	programmer.identifyChip()
	 
	if programmer.program(sys.argv[1]):
		print("PROGRAMMED " + sys.argv[1] + " SUCCESSFULLY!")
	
	try:
		lfuse = int(sys.argv[2],0)
		programmer.setFuses(lfuse = lfuse, hfuse = None, efuse = None)	#sets clk/1
		print("LFUSE BURNED TO: " + hex(lfuse))
	except:
		programmer.setFuses(lfuse = 0xE2, hfuse = None, efuse = None)	#sets clk/1
		print("NO LFUSE PROVIDED. BURNED LFUSE TO DEFAULT: 0xE2")
		
	for address in range(4):
		programmer.writeEEPROMRequest(address, random.randint(0,255))
	print("WROTE RANDOMIZED ADDRESS")
	programmer.runChipRequest()   #start chip application
	time.sleep(1)