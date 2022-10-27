from collections import Counter, deque
import struct
import sys
import time

import pygame
from pygame.locals import *
import numpy as np

from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC
from sklearn.pipeline import make_pipeline
from sklearn.linear_model import LogisticRegression
from sklearn.tree import DecisionTreeClassifier
from sklearn.naive_bayes import GaussianNB

from myoRaw import Myo, emgMode
from myoClassifier import liveClassifier, MyoClassifier, EMGHandler

class SVM_Classifier(liveClassifier):
	'''
	Live implimentation of an SVM Classifier
	'''
	def __init__(self):
		liveClassifier.__init__(self, None, "SVM", (100,0,100))

	def train(self, X, Y):
		self.X = X
		self.Y = Y
		try:
			if self.X.shape[0] > 0: 
				clf = make_pipeline(StandardScaler(), SVC(gamma='auto'))
				#clf = make_pipeline(StandardScaler(), SVC(kernel="linear", C=0.025))

				clf.fit(self.X, self.Y)
				self.model = clf
		except:
			# SVM Errors when we only have data for 1 class.
			self.model = None

	def classify(self, emg):
		if self.X.shape[0] == 0 or self.model == None:
			# We have no data or model, return 0
			return 0

		x = np.array(emg).reshape(1,-1)
		pred = self.model.predict(x)
		return int(pred[0])


class DC_Classifier(liveClassifier):
	'''
	Live implimentation of Decision Trees
	'''
	def __init__(self):
		liveClassifier.__init__(self, DecisionTreeClassifier(), name="DC_Classifier", color=(212,175,55))

class LR_Classifier(liveClassifier):
	'''
	Live implimentation of Logistic Regression
	'''
	def __init__(self):
		liveClassifier.__init__(self, None, name="LR", color=(100,0,100))

	def train(self, X, Y):
		self.X = X
		self.Y = Y
		try:
			if self.X.shape[0] > 0: 
				self.model = LogisticRegression()
				self.model.fit(self.X, self.Y)
		except:
			# LR Errors when we only have data for 1 class.
			self.model = None

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

	# SVM Example
	m = MyoClassifier(DC_Classifier(), mode=emgMode.PREPROCESSED)
	hnd = EMGHandler(m)
	m.addEmgHandler(hnd)
	m.connect()

	def printPose(poseInput):
		print('\r'+'>>> Pose: '+'{}'.format(int(poseInput)),end='')
	m.addRawPoseHandler(printPose)

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
