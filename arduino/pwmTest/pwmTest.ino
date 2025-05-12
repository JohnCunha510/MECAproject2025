int motor_current = 0;
float alpha = 0.2;
int motor_current_smooth = 0;
int motor_current_smooth_last = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  pinMode(11, OUTPUT);

  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);

  TCCR2B = _BV(CS22);

  OCR2A = 180;

  OCR2B = 50;

}

void loop() {

  motor_current = analogRead(A1);

  motor_current_smooth = alpha * motor_current + (1 - alpha) * motor_current_smooth_last;
  motor_current_smooth_last = motor_current_smooth;
  

  Serial.print("motor-current:");
  Serial.print(motor_current);
  Serial.print(",");
  Serial.print("motor-current-smooth:");
  Serial.print(motor_current_smooth);  
  //Serial.print(",");
  //Serial.print("encoder-count:");
  //Serial.print(encoder_count);  
  //Serial.print(",");
  //Serial.print("rotary-count:");
  //Serial.print(rotary_count);   
  Serial.println("");   
  delay (100);

}

