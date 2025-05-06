"""
serial_usb_simple.py  Demo that communicates over USB using serial I/O
from a Raspberry Pi to an Arduino.

To show that it work, this writes a '1' to the Arduino which then 
blinks the builtin LED on and off. The Arduino also sends back an 'A'
to acknowledge that it got the message. This does a read() to get
the 'A', demonstrating that reading also works. Two seconds later,
this writes a '0' to the Arduino which then stops the blinking.
The Arduino again sends back an 'A' to acknowledge that it got the
message and this reads the 'A'.

This was tested between a Raspberry Pi 3B (running Raspbian) and
an Arduino Mega 2560 and also between an NVIDIA Jetson TX1 (running 
Ubuntu) and the same Arduino.
"""

import serial
import time

ser = serial.Serial('COM19', 115200, timeout=1)

# Reset the Arduino's line. This is key to getting the write to work.
# Without it, the first few writes don't work.
# Clear DTR, wait one second, flush input, then set DTR.
# Without this, the first write fails.
# This trick was learned from:
# https://github.com/miguelasd688/4-legged-robot-model

ser.setDTR(False)
time.sleep(1)
ser.flushInput()
ser.setDTR(True)
time.sleep(2)

while True:

	print('Telling the Arduino to start blinking...')
	ser.write(b'1')

	# read to get the acknowledgement from the Arduino
	while True:
		ack = ser.read()
		if ack == b'A':
			break
	print('Arduino sent back %s' % ack)

	time.sleep(5)

	print('Telling the Arduino to stop blinking...')
	ser.write(b'0')

	# read to get the acknowledgement from the Arduino
	while True:
		ack = ser.read()
		if ack == b'A':
			break
	print('Arduino sent back %s' % ack)

	time.sleep(5)

    
/*
 * serial_usb_simple_arduino - For communicating over USB serial. Send it a '1' (character one) 
 * and it will make the builtin LED start blinking every one second. Send it a '0' 
 * (character zero) and it will make it stop blinking.
 * 
 * Each time it receives one of the commands, it sends back an 'A' for acknowledge.
 * But send it a commmand it doesn't recognize and it sends back an 'E' for error.
 */

bool blinking = false;
bool led_on = false;
int target_time;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  pinMode(13, OUTPUT);
  pinMode(11, OUTPUT);

  digitalWrite(13, LOW);
}

void loop() {
  char c;

  
  if (Serial.available() > 0) {
    c = Serial.read();
    switch (c) {
    case '0':
      // stop blinking
      blinking = false;
      analogWrite(11, 0);

      Serial.write("A", 1);
      break;
    case '1':
      // start blinking
      if (blinking == false) {
        blinking = true;
        analogWrite(11, 150);
        
      }
      Serial.write("A", 1);
      break;
    default:
      Serial.write("E", 1);
      break;    
    }
  } else if (blinking) {
    analogWrite(11, 150);
  }
}