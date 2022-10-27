from collections import Counter, deque
import struct
import sys
import time

import pygame
from pygame.locals import *
import numpy as np

from myoRaw import Myo, emgMode

SUBSAMPLE = 3
K = 15

class Classifier(object):
	'''A wrapper for nearest-neighbor classifier that stores
	training data in vals0, ..., vals9.dat.'''

	def __init__(self, name="Classifier", color=(0,200,0)):
		# Add some identifiers to the classifier to identify what model was used in different screenshots
		self.name = name
		self.color = color
		self.device = 'mac'

		for i in range(10):
			if self.device == 'mac':
				with open('/Users/nekospirin/Desktop/Myo/test/data/vals%d.dat' % i, 'ab') as f: pass
			elif self.device == 'rpi':
				with open('/home/pi/Desktop/Myo/test/data/vals%d.dat' % i, 'ab') as f: pass
		self.readData()

	def storeData(self, cls, vals):
		if self.device == 'mac':
			with open('/Users/nekospirin/Desktop/Myo/test/data/vals%d.dat' % cls, 'ab') as f:
				f.write(pack('8H', *vals))
		elif self.device == 'rpi':
			with open('/home/pi/Desktop/Myo/test/data/vals%d.dat' % cls, 'ab') as f:
				f.write(pack('8H', *vals))


		self.train(np.vstack([self.X, vals]), np.hstack([self.Y, [cls]]))

	def readData(self):
		X = []
		Y = []
		for i in range(10):
			if self.device == 'mac':
				X.append(np.fromfile('/Users/nekospirin/Desktop/Myo/test/data/vals%d.dat' % i, dtype=np.uint16).reshape((-1, 8)))
			elif self.device == 'rpi':
				X.append(np.fromfile('/home/pi/Desktop/Myo/test/data/vals%d.dat' % i, dtype=np.uint16).reshape((-1, 8)))
			Y.append(i + np.zeros(X[-1].shape[0]))

		self.train(np.vstack(X), np.hstack(Y))

	def deleteData(self):
		for i in range(10):
			if self.device == 'mac':
			    with open('/Users/nekospirin/Desktop/Myo/test/data/vals%d.dat' % i, 'wb') as f: pass
			elif self.device == 'rpi':
			    with open('/home/pi/Desktop/Myo/test/data/vals%d.dat' % i, 'wb') as f: pass
		self.readData()

	def train(self, X, Y):
		self.X = X
		self.Y = Y
		self.model = None

	def nearest(self, d):
		dists = ((self.X - d)**2).sum(1)
		ind = dists.argmin()
		return self.Y[ind]

	def classify(self, d):
		if self.X.shape[0] < K * SUBSAMPLE: return 0
		return self.nearest(d)

class MyoClassifier(Myo):
	'''Adds higher-level pose classification and handling onto Myo.'''

	HIST_LEN = 25

	def __init__(self, cls, tty=None, mode=emgMode.PREPROCESSED):
		Myo.__init__(self, tty, mode=mode)
		# Add a classifier
		self.cls = cls

		self.history = deque([0] * MyoClassifier.HIST_LEN, MyoClassifier.HIST_LEN)
		self.historyCnt = Counter(self.history)
		self.addEmgHandler(self.emgHandler)
		self.lastPose = None

		self.poseHandlers = []

	def emgHandler(self, emg, moving):
		y = self.cls.classify(emg)
		self.historyCnt[self.history[0]] -= 1
		self.historyCnt[y] += 1
		self.history.append(y)

		r, n = self.historyCnt.most_common(1)[0]
		if self.lastPose is None or (n > self.historyCnt[self.lastPose] + 5 and n > MyoClassifier.HIST_LEN / 2):
			self.onRawPose(r)
			self.lastPose = r

	def addRawPoseHandler(self, h):
		self.poseHandlers.append(h)

	def onRawPose(self, pose):
		for h in self.poseHandlers:
			h(pose)

	def runGui(self, hnd, scr, font, w, h):
		# Handle keypresses
		for ev in pygame.event.get():
			if ev.type == QUIT or (ev.type == KEYDOWN and ev.unicode == 'q'):
				raise KeyboardInterrupt()
			elif ev.type == KEYDOWN:
				if K_0 <= ev.key <= K_9:
					# Labelling using row of numbers
					hnd.recording = ev.key - K_0
				elif K_KP0 <= ev.key <= K_KP9:
					# Labelling using Keypad
					hnd.recording = ev.key - K_KP0
				elif ev.unicode == 'r':
					hnd.cl.read_data()
				elif ev.unicode == 'e':
					print("Pressed e, erasing local data")
					self.cls.delete_data()
			elif ev.type == KEYUP:
				if K_0 <= ev.key <= K_9 or K_KP0 <= ev.key <= K_KP9:
					# Don't record incoming data
					hnd.recording = -1

		# Plotting
		scr.fill((0, 0, 0), (0, 0, w, h))
		r = self.historyCnt.most_common(1)[0][0]

		for i in range(10):
			x = 0
			y = 0 + 30 * i
			# Set the barplot color
			clr = self.cls.color if i == r else (255,255,255)

			txt = font.render('%5d' % (self.cls.Y == i).sum(), True, (255,255,255))
			scr.blit(txt, (x + 20, y))

			txt = font.render('%d' % i, True, clr)
			scr.blit(txt, (x + 110, y))

			# Plot the barchart plot
			scr.fill((0,0,0), (x+130, y + txt.get_height() / 2 - 10, len(self.history) * 20, 20))
			scr.fill(clr, (x+130, y + txt.get_height() / 2 - 10, self.historyCnt[i] * 20, 20))

		pygame.display.flip()

def pack(fmt, *args):
	return struct.pack('<' + fmt, *args)

def unpack(fmt, *args):
	return struct.unpack('<' + fmt, *args)

def text(scr, font, txt, pos, clr=(255,255,255)):
	scr.blit(font.render(txt, True, clr), pos)


class EMGHandler(object):
	def __init__(self, m):
		self.recording = -1
		self.m = m
		self.emg = (0,) * 8

	def __call__(self, emg, moving):
		self.emg = emg
		if self.recording >= 0:
			self.m.cls.storeData(self.recording, emg)

class liveClassifier(Classifier):
	'''
	General class for all Sklearn classifiers
	Expects something you can call .fit and .predict on
	'''
	def __init__(self, classifier, name="Live Classifier", color=(0,55,175)):
		self.model = classifier
		Classifier.__init__(self, name=name, color=color)

	def train(self, X, Y):
		self.X = X
		self.Y = Y

		if self.X.shape[0] > 0 and self.Y.shape[0] > 0: 
			self.model.fit(self.X, self.Y)

	def classify(self, emg):
		if self.X.shape[0] == 0 or self.model == None:
			# We have no data or model, return 0
			return 0

		x = np.array(emg).reshape(1,-1)
		pred = self.model.predict(x)
		return int(pred[0])

if __name__ == '__main__':
	pygame.init()
	w, h = 800, 320
	scr = pygame.display.set_mode((w, h))
	font = pygame.font.Font(None, 30)

	m = MyoClassifier(Classifier())
	hnd = EMGHandler(m)
	m.addEmgHandler(hnd)
	m.connect()
	
	def printPose(poseInput):
		print('\r'+'>>> Pose: '+'{}'.format(int(poseInput)),end='')
	m.addRawPoseHandler(printPose)

	# Set Myo LED color to model color
	m.setLEDs(m.cls.color, m.cls.color)
	# Set pygame window name
	pygame.display.set_caption(m.cls.name)

	try:
		while True:
			m.run()
			#m.runGui(hnd, scr, font, w, h)

	except KeyboardInterrupt:
		pass
	finally:
		m.disconnect()
		print()
		pygame.quit()
