import enum
import re
import struct
import sys
import threading
import time

import serial
from serial.tools.list_ports import comports

# The common function
def pack(fmt, *args):
	return struct.pack('<' + fmt, *args)

def unpack(fmt, *args):
	return struct.unpack('<' + fmt, *args)

def multichr(ords):
	if sys.version_info[0] >= 3:
		return bytes(ords)
	else:
		return ''.join(map(chr, ords))
# /The common function


def multiord(b):
	if sys.version_info[0] >= 3:
		return list(b)
	else:
		return map(ord, b)

class emgMode(enum.Enum):
	NO_DATA = 0 # Do not send EMG data
	PREPROCESSED = 1 # Sends 50Hz rectified and band pass filtered data
	FILTERED = 2 # Sends 200Hz filtered but not rectified data
	RAW = 3 # Sends raw 200Hz data from the ADC ranged between -128 and 127

class Arm(enum.Enum):
	UNKNOWN = 0
	RIGHT = 1
	LEFT = 2


class XDirection(enum.Enum):
	UNKNOWN = 0
	X_TOWARD_WRIST = 1
	X_TOWARD_ELBOW = 2


class Pose(enum.Enum):
	REST = 0
	FIST = 1
	WAVE_IN = 2
	WAVE_OUT = 3
	FINGERS_SPREAD = 4
	THUMB_TO_PINKY = 5
	UNKNOWN = 255


class Packet(object):
	def __init__(self, ords):
		self.typ = ords[0]
		self.cls = ords[2]
		self.cmd = ords[3]
		self.payload = multichr(ords[4:])

	def __repr__(self):
		return 'Packet(%02X, %02X, %02X, [%s])' % \
			(self.typ, self.cls, self.cmd,
			 ' '.join('%02X' % b for b in multiord(self.payload)))


class BT(object):
	'''Implements the non-Myo-specific details of the Bluetooth protocol.'''
	def __init__(self, tty):
		self.ser = serial.Serial(port=tty, baudrate=9600, dsrdtr=1)
		self.buf = []
		self.lock = threading.Lock()
		self.handlers = []

	# internal data-handling methods
	def recvPacket(self):
		n = self.ser.inWaiting() # Windows fix

		while True:
			c = self.ser.read()
			if not c:
				return None

			ret = self.procByte(ord(c))
			if ret:
				if ret.typ == 0x80:
					self.handleEvent(ret)
					# Windows fix
					if n >= 5096:
						print("Clearning",n)
						self.ser.flushInput()
					# End of Windows fix
				return ret

	def procByte(self, c):
		if not self.buf:
			if c in [0x00, 0x80, 0x08, 0x88]:  # [BLE response pkt, BLE event pkt, wifi response pkt, wifi event pkt]
				self.buf.append(c)
			return None
		elif len(self.buf) == 1:
			self.buf.append(c)
			self.packet_len = 4 + (self.buf[0] & 0x07) + self.buf[1]
			return None
		else:
			self.buf.append(c)

		if self.packet_len and len(self.buf) == self.packet_len:
			p = Packet(self.buf)
			self.buf = []
			return p
		return None

	def handleEvent(self, p):
		for h in self.handlers:
			h(p)

	def addHandler(self, h):
		self.handlers.append(h)

	def removeHandler(self, h):
		try:
			self.handlers.remove(h)
		except ValueError:
			pass

	def waitEvent(self, cls, cmd):
		res = [None]

		def h(p):
			if p.cls == cls and p.cmd == cmd:
				res[0] = p
		self.addHandler(h)
		while res[0] is None:
			self.recvPacket()
		self.removeHandler(h)
		return res[0]

	# specific BLE commands
	def connect(self, addr):
		return self.sendCommand(6, 3, pack('6sBHHHH', multichr(addr), 0, 6, 6, 64, 0))

	def getConnections(self):
		return self.sendCommand(0, 6)

	def discover(self):
		return self.sendCommand(6, 2, b'\x01')

	def endScan(self):
		return self.sendCommand(6, 4)

	def disconnect(self, h):
		return self.sendCommand(3, 0, pack('B', h))

	def readAttr(self, con, attr):
		self.sendCommand(4, 4, pack('BH', con, attr))
		return self.waitEvent(4, 5)

	def writeAttr(self, con, attr, val):
		self.sendCommand(4, 5, pack('BHB', con, attr, len(val)) + val)
		return self.waitEvent(4, 1)

	def sendCommand(self, cls, cmd, payload=b'', wait_resp=True):
		s = pack('4B', 0, len(payload), cls, cmd) + payload
		self.ser.write(s)

		while True:
			p = self.recvPacket()
			# no timeout, so p won't be None
			if p.typ == 0:
				return p
			# not a response: must be an event
			self.handleEvent(p)


class Myo(object):
	'''Implements the Myo-specific communication protocol.'''

	def __init__(self, tty=None, mode=1):
		if tty is None:
			tty = self.detectTty()
		if tty is None:
			raise ValueError('Myo dongle not found!')

		self.bt = BT(tty)
		self.conn = None
		self.emgHandlers = []
		self.imuHandlers = []
		self.armHandlers = []
		self.poseHandlers = []
		self.batteryHandlers = []
		self.mode = mode

	def detectTty(self):
		for p in comports():
			if re.search(r'PID=2458:0*1', p[2]):
				print('>>> Using Device:', p[0])
				return p[0]

		return None

	def run(self):
		self.bt.recvPacket()

	def connect(self, addr=None):
		'''
		Connect to a Myo
		Addr is the MAC address in format: [93, 41, 55, 245, 82, 194]
		'''
		# stop everything from before
		self.bt.endScan()
		self.bt.disconnect(0)
		self.bt.disconnect(1)
		self.bt.disconnect(2)

		# start scanning
		if (addr is None):
			print('>>> Scanning...')
			self.bt.discover()
			while True:
				p = self.bt.recvPacket()
				print('\r'+'>>> Scan Response:', p, end='',flush=True)

				if p.payload.endswith(b'\x06\x42\x48\x12\x4A\x7F\x2C\x48\x47\xB9\xDE\x04\xA9\x01\x00\x06\xD5'):
					addr = list(multiord(p.payload[2:8]))
					break
			self.bt.endScan()
		# connect and wait for status event
		conn_pkt = self.bt.connect(addr)
		self.conn = multiord(conn_pkt.payload)[-1]
		self.bt.waitEvent(3, 0)

		# get firmware version
		fw = self.readAttr(0x17)
		_, _, _, _, v0, v1, v2, v3 = unpack('BHBBHHHH', fw.payload)
		print('>>> Firmware Version: %d.%d.%d.%d' % (v0, v1, v2, v3))

		self.old = (v0 == 0)

		if self.old:
			# don't know what these do; Myo Connect sends them, though we get data
			# fine without them
			self.writeAttr(0x19, b'\x01\x02\x00\x00')
			# Subscribe for notifications from 4 EMG data channels
			self.writeAttr(0x2f, b'\x01\x00')
			self.writeAttr(0x2c, b'\x01\x00')
			self.writeAttr(0x32, b'\x01\x00')
			self.writeAttr(0x35, b'\x01\x00')

			# enable EMG data
			self.writeAttr(0x28, b'\x01\x00')
			# enable IMU data
			self.writeAttr(0x1d, b'\x01\x00')

			# Sampling rate of the underlying EMG sensor, capped to 1000. If it's
			# less than 1000, emgHZ is correct. If it is greater, the actual
			# framerate starts dropping inversely. Also, if this is much less than
			# 1000, EMG data becomes slower to respond to changes. In conclusion,
			# 1000 is probably a good value.f
			C = 1000
			emgHZ = 50
			# strength of low-pass filtering of EMG data
			emgSmooth = 100

			imuHZ = 50

			# send sensor parameters, or we don't get any data
			self.writeAttr(0x19, pack('BBBBHBBBBB', 2, 9, 2, 1, C, emgSmooth, C // emgHZ, imuHZ, 0, 0))

		else:
			name = self.readAttr(0x03)
			print('>>> Device Name: %s' % name.payload)

			# enable IMU data
			self.writeAttr(0x1d, b'\x01\x00')
			# enable on/off arm notifications
			self.writeAttr(0x24, b'\x02\x00')
			# enable EMG notifications
			if (self.mode == emgMode.PREPROCESSED):
				# Send the undocumented filtered 50Hz.
				print(">>> Starting filtered, 0x01")
				self.startFiltered() # 0x01
			elif (self.mode == emgMode.FILTERED):
				print(">>> Starting raw filtered, 0x02")
				self.startRaw() # 0x02
			elif (self.mode == emgMode.RAW):
				print(">>> Starting raw, unfiltered, 0x03")
				self.startRawUnfiltered() #0x03
			else:
				print(">>> No EMG mode selected, not sending EMG data")
			# Stop the Myo Disconnecting
			self.sleepMode(1)

			# enable battery notifications
			self.writeAttr(0x12, b'\x01\x10')

		# add data handlers
		def handleData(p):
			if (p.cls, p.cmd) != (4, 5):
				return

			c, attr, typ = unpack('BHB', p.payload[:4])
			pay = p.payload[5:]

			if attr == 0x27:
				# Unpack a 17 byte array, first 16 are 8 unsigned shorts, last one an unsigned char
				vals = unpack('8HB', pay)
				# not entirely sure what the last byte is, but it's a bitmask that
				# seems to indicate which sensors think they're being moved around or
				# something
				emg = vals[:8]
				moving = vals[8]
				self.onEmg(emg, moving)
			# Read notification handles corresponding to the for EMG characteristics
			elif attr == 0x2b or attr == 0x2e or attr == 0x31 or attr == 0x34:

				emg1 = struct.unpack('<8b', pay[:8])
				emg2 = struct.unpack('<8b', pay[8:])
				self.onEmg(emg1, 0)
				self.onEmg(emg2, 0)
			# Read IMU characteristic handle
			elif attr == 0x1c:
				vals = unpack('10h', pay)
				quat = vals[:4]
				gyro = vals[4:7]
				acc = vals[7:10]
				self.onImu(quat, acc, gyro)
			# Read classifier characteristic handle
			elif attr == 0x23:
				typ, val, xdir, _, _, _ = unpack('6B', pay)

				if typ == 1:  # on arm
					self.onArm(Arm(val), XDirection(xdir))
				elif typ == 2:  # removed from arm
					self.onArm(Arm.UNKNOWN, XDirection.UNKNOWN)
				elif typ == 3:  # pose
					self.onPose(Pose(val))
			# Read battery characteristic handle
			elif attr == 0x11:
				batteryLevel = ord(pay)
				self.onBattery(batteryLevel)
			else:
				print('>>> Data With Unknown attr: %02X %s' % (attr, p))

		self.bt.addHandler(handleData)

	def writeAttr(self, attr, val):
		if self.conn is not None:
			self.bt.writeAttr(self.conn, attr, val)

	def readAttr(self, attr):
		if self.conn is not None:
			return self.bt.readAttr(self.conn, attr)
		return None

	def disconnect(self):
		if self.conn is not None:
			self.bt.disconnect(self.conn)

	def sleepMode(self, mode):
		self.writeAttr(0x19, pack('3B', 9, 1, mode))

	def powerOff(self):

		self.writeAttr(0x19, b'\x04\x00')

	def startRaw(self):

		self.writeAttr(0x2c, b'\x01\x00')  # Suscribe to EmgData0Characteristic
		self.writeAttr(0x2f, b'\x01\x00')  # Suscribe to EmgData1Characteristic
		self.writeAttr(0x32, b'\x01\x00')  # Suscribe to EmgData2Characteristic
		self.writeAttr(0x35, b'\x01\x00')  # Suscribe to EmgData3Characteristic

		# struct.pack('<5B', 1, 3, emgMode, imu_mode, classifier_mode)
		self.writeAttr(0x19, b'\x01\x03\x02\x01\x01')


	def startFiltered(self):
	

		self.writeAttr(0x28, b'\x01\x00')
		self.writeAttr(0x19, b'\x01\x03\x01\x01\x00')

	def startRawUnfiltered(self):
	
		self.writeAttr(0x2c, b'\x01\x00')  # Suscribe to EmgData0Characteristic
		self.writeAttr(0x2f, b'\x01\x00')  # Suscribe to EmgData1Characteristic
		self.writeAttr(0x32, b'\x01\x00')  # Suscribe to EmgData2Characteristic
		self.writeAttr(0x35, b'\x01\x00')  # Suscribe to EmgData3Characteristic

		# struct.pack('<5B', 1, 3, emgMode, imu_mode, classifier_mode)
		self.writeAttr(0x19, b'\x01\x03\x03\x01\x00')

	def mcStartCollection(self):


		self.writeAttr(0x28, b'\x01\x00')  # Suscribe to EMG notifications
		self.writeAttr(0x1d, b'\x01\x00')  # Suscribe to IMU notifications
		self.writeAttr(0x24, b'\x02\x00')  # Suscribe to classifier indications
		self.writeAttr(0x19, b'\x01\x03\x01\x01\x01')  # Set EMG and IMU, payload size = 3, EMG on, IMU on, classifier on
		self.writeAttr(0x28, b'\x01\x00')  # Suscribe to EMG notifications
		self.writeAttr(0x1d, b'\x01\x00')  # Suscribe to IMU notifications
		self.writeAttr(0x19, b'\x09\x01\x01\x00\x00')  # Set sleep mode, payload size = 1, never go to sleep, don't know, don't know
		self.writeAttr(0x1d, b'\x01\x00')  # Suscribe to IMU notifications
		self.writeAttr(0x19, b'\x01\x03\x00\x01\x00')  # Set EMG and IMU, payload size = 3, EMG off, IMU on, classifier off
		self.writeAttr(0x28, b'\x01\x00')  # Suscribe to EMG notifications
		self.writeAttr(0x1d, b'\x01\x00')  # Suscribe to IMU notifications
		self.writeAttr(0x19, b'\x01\x03\x01\x01\x00')  # Set EMG and IMU, payload size = 3, EMG on, IMU on, classifier off

	def mcEndCollection(self):


		self.writeAttr(0x28, b'\x01\x00')
		self.writeAttr(0x1d, b'\x01\x00')
		self.writeAttr(0x24, b'\x02\x00')
		self.writeAttr(0x19, b'\x01\x03\x01\x01\x01')
		self.writeAttr(0x19, b'\x09\x01\x00\x00\x00')
		self.writeAttr(0x1d, b'\x01\x00')
		self.writeAttr(0x24, b'\x02\x00')
		self.writeAttr(0x19, b'\x01\x03\x00\x01\x01')
		self.writeAttr(0x28, b'\x01\x00')
		self.writeAttr(0x1d, b'\x01\x00')
		self.writeAttr(0x24, b'\x02\x00')
		self.writeAttr(0x19, b'\x01\x03\x01\x01\x01')

	def vibrate(self, length):
		if length in range(1, 4):
			# first byte tells it to vibrate; purpose of second byte is unknown (payload size?)
			self.writeAttr(0x19, pack('3B', 3, 1, length))

	def setLEDs(self, logo, line):
		self.writeAttr(0x19, pack('8B', 6, 6, *(logo + line)))

	# def get_battery_level(self):
	#     batteryLevel = self.readAttr(0x11)
	#     return ord(batteryLevel.payload[5])

	def addEmgHandler(self, h):
		self.emgHandlers.append(h)

	def addImuHandler(self, h):
		self.imuHandlers.append(h)

	def addPoseHandler(self, h):
		self.poseHandlers.append(h)

	def addArmHandler(self, h):
		self.armHandlers.append(h)

	def addBatteryHandler(self, h):
		self.batteryHandlers.append(h)

	def onEmg(self, emg, moving):
		for h in self.emgHandlers:
			h(emg, moving)

	def onImu(self, quat, acc, gyro):
		for h in self.imuHandlers:
			h(quat, acc, gyro)

	def onPose(self, p):
		for h in self.poseHandlers:
			h(p)

	def onArm(self, arm, xdir):
		for h in self.armHandlers:
			h(arm, xdir)

	def onBattery(self, batteryLevel):
		for h in self.batteryHandlers:
			h(batteryLevel)

if __name__ == '__main__':
	m = Myo(sys.argv[1] if len(sys.argv) >= 2 else None, mode=emgMode.RAW)

	def procEmg(emg, moving, times=[]):
		outEMG = []
		for cache in emg:
			outEMG.append(str(cache).ljust(4))
		print('\r'+' '.join('[{}]'.format(part) for part in outEMG), end='')
	
	m.addEmgHandler(procEmg)
	m.connect()

	m.addArmHandler(lambda arm, xdir: print('arm', arm, 'xdir', xdir))
	m.addPoseHandler(lambda p: print('pose', p))
	# m.addImuHandler(lambda quat, acc, gyro: print('quaternion', quat))
	m.sleepMode(1)
	m.setLEDs([128, 128, 255], [128, 128, 255])  # purple logo and bar LEDs
	m.vibrate(1)

	try:
		while True:
			m.run()

	except KeyboardInterrupt:
		m.disconnect()
		quit()
