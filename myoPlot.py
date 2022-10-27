from re import T
import pygame
from pygame.locals import *
import multiprocessing as mp

from myoRaw import Myo, emgMode

import numpy as np
import curses

# ------------ Myo Setup ---------------
emgQueue = mp.Queue()
quatQueue = mp.Queue()
accQueue = mp.Queue()
gyroQueue = mp.Queue()
poseQueue = mp.Queue()

def worker(emgQueue, quatQueue, accQueue, gyroQueue):
	m = Myo(mode=emgMode.PREPROCESSED)
	m.connect()
	
	def addEmgToQueue(emg, movement):
		emgQueue.put(emg)
	
	def addImuToQueue(quat, acc, gyro):
		quatQueue.put(quat)
		accQueue.put(acc)
		gyroQueue.put(gyro)

	m.addEmgHandler(addEmgToQueue)
	m.addImuHandler(addImuToQueue)

	def printBattery(bat):
		print('>>> Battery Level:', bat)

	m.addBatteryHandler(printBattery)

	m.setLEDs([144, 180, 75], [144, 180, 75])
	# Vibrate to know we connected okay
	m.vibrate(1)
	
	"""worker function"""
	while True:
		m.run()
	print("Worker Stopped")

lastVals = None
def plot(scr, vals):
	DRAW_LINES = False

	global lastVals
	if lastVals is None:
		lastVals = vals
		return

	D = 5
	scr.scroll(-D)
	scr.fill((0, 0, 0), (w - D, 0, w, h))
	for i, (u, v) in enumerate(zip(lastVals, vals)):
		if DRAW_LINES:
			pygame.draw.line(scr, (249, 191, 69),
							 (w - D, int(h/9 * (i+1 - u))),
							 (w, int(h/9 * (i+1 - v))),
							 width=3)
			pygame.draw.line(scr, (255, 255, 255),
							 (w - D, int(h/9 * (i+1))),
							 (w, int(h/9 * (i+1))),
							 width=2)
		else:
			c = int(255 * max(0, min(1, v)))
			scr.fill((c, c, c), (w - D, i * h / 8, D, (i + 1) * h / 8 - i * h / 8))

	pygame.display.flip()
	lastVals = vals

# -------- Main Program Loop -----------
if __name__ == "__main__":
	p = mp.Process(target=worker, args=(emgQueue, quatQueue, accQueue, gyroQueue))
	p.start()

	w, h = 1000, 800
	scr = pygame.display.set_mode((w, h))

	data_window = curses.initscr()

	def getMatrixString(matrix):
		x = ''
		for row in matrix:
			x += '| '.join(str(item) for item in row)
			x += "\n"
		return x

	try:
		while True:
			# Handle pygame events to keep the window responding
			pygame.event.pump()
			# Get the emg data and plot it
			while not(emgQueue.empty()):
				emg = list(emgQueue.get())
				quat = list(quatQueue.get())
				gyro = list(gyroQueue.get())
				acc = list(accQueue.get())
				dataMap = [emg, quat, gyro, acc]
				
				for i in range(0,4):
					dataMap[i] += [-40000]*(8-len(dataMap[i]))
				
				dataMat = np.char.ljust((np.asarray(dataMap) / 200).astype(np.int).astype(np.str), 4)
				dataMat[:][dataMat[:]=='-200'] = '/   '
				title = np.array(['>>> EMG         : ', '>>> Quaternion  : ', '>>> Gyroscope   : ', '>>> Acceleration: '])
				titleV = np.reshape(title,(4,1))
				outPutMat = np.concatenate((titleV, dataMat), axis=1)

				data_window.addstr(8, 0, getMatrixString(outPutMat))
				data_window.refresh()

				#plot(scr, [e / 2000. for e in emg])

	except KeyboardInterrupt:
		print(">>> Quitting")
		pygame.quit()
		curses.endwin()
		quit()