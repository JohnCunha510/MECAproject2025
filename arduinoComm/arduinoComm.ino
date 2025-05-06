
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