MAX30102 Pulse Oximetry Sensor code for Raspberry Pi

YouTube link to the project:
https://youtu.be/QhFH0FVt4hI?si=gP6WJS7oiO5sgZ4C


Setup:
One Raspberry Pi and one laptop with Linux installed.


How to run the program:
	
	1. On Linux open subscriber and run with command: 
		python3 subRaw.py
	-> When you run subRaw, in terminal you can find the link to the website
	
	2. On Raspberry Pi open publisher and run with command: 
		python3 pub.py
		
	3. On Linux open subscriber and run with commands in order:
		python3 subSPO2.py
		python3 subBPM.py
 

How the system works:

-> The system measures heartrate and SpO2 using the MAX30102 sensor processes the signal and displays the data via web interface.
-> The architecture is split into publisher (Raspberry Pi) and subscriber (UI + alarm), communicating through ZeroMQ PUB/SUB model. 

PUBLISHER:
-> MAX30102 driver reads IR and RED PPG signals over I2C.
-> Heartrate monitor detects finger presence and computes BPM and SpO2 using the hrcalc algorithm.
-> pub.py publishes raw IR data every 0.1s for plotting, as well as BPM and SpO2 values at a lower rate every 15 seconds.

SUBSCRIBER:
# subRaw.py:
	-> Receives raw IR data.
	-> Streams it to the web interface for real-time plotting.
	
# subBPM.py:
	-> Receives BPM values.
	-> Triggers the alarm when the BPM drops to zero.
	-> Immediately stops the alarm when BPM > 0.
	-> Forwards the BPM to the web application.
	
WEB INTERFACE:
-> Displays real-time PPG waveform, SpO2 and BPM.
