
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                            I/O PIN DEFINITIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const int DIR_B = 13;
const int PWM_B = 11;
const int SNS_B = A1;

const int ENCODER_A = 3;
const int ENCODER_B = 4;

const int DAT = 2;
const int CLK = 5;
const int BUTTON = 6;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                            Global Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool blinking = false;
bool motor_direction = false;
int target_time;

bool pressed = false;
bool lastPressed = false;
int rotary_count = 0;

long encoder_count = 0;
long encoder_count_last = 0;
int pwm_output = 100;
int motor_current = 0;
float alpha = 0.2;
int motor_current_smooth = 0;
int motor_current_smooth_last = 0;
int speed_int = 0;
long serial_time = 0;

float pid_Kp = 5;
float pid_Ki = 0;
float pid_Kd = 0;

float error = 0;
float error_i = error;
float error_d = error;
int set_speed = 0;
int motor_command = 0;



void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), rightEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DAT), RoteStateChanged, FALLING);

  pinMode(DIR_B, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(SNS_B, INPUT);

  pinMode(ENCODER_B, INPUT);
  pinMode(CLK, INPUT);
  pinMode(BUTTON, INPUT);

  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);

  TCCR2B = _BV(CS22);

  OCR2A = 180;

  OCR2B = 50;


}

void loop() {

  lastPressed = pressed;
  pressed = digitalRead(BUTTON);
  if (lastPressed == false && pressed == true) {
    Serial.println("Change Direction!!");
    motor_direction = !motor_direction;
  }

  if (motor_direction) {
    digitalWrite(DIR_B, HIGH);
  } else {
    digitalWrite(DIR_B, LOW);
  }

  rotary_count = min(100, max(0, rotary_count));
  set_speed = map(rotary_count, 0, 100, 0, 255);

  if(millis() > serial_time + 100) {// Every 100 ms
    serial_time = millis();
    speed_int = (int) ((abs(encoder_count - encoder_count_last) / 748.0) * (60 / 0.1));
    encoder_count_last = encoder_count;



    error = speed_int - set_speed;

    error_i = error_i + error / 0.1;

    motor_command = error * pid_Kp + error_i * pid_Ki ;

    motor_command = max(0, min(500, abs(motor_command)));

    pwm_output = map(motor_command, 0, 500, 0, 255);
    OCR2A = pwm_output;

    
    motor_current = analogRead(SNS_B); //map(analogRead(SNS_B), 0, 676, 0, 2);
    


    motor_current_smooth = alpha * motor_current + (1 - alpha) * motor_current_smooth_last;
    motor_current_smooth_last = motor_current_smooth;

  }

  //Serial.print("motor-current:");
  //Serial.print(motor_current);
  //Serial.print(",");
  Serial.print("motor-current-smooth:");
  Serial.print(motor_current_smooth);  
  Serial.print(",");
  //Serial.print("encoder-count:");
  //Serial.print(encoder_count);  
  //Serial.print(",");
  Serial.print("motor-speed:");
  Serial.print(speed_int);  
  Serial.print(",");
  Serial.print("error:");
  Serial.print(error);  
  Serial.print(",");
  Serial.print("motor-command:");
  Serial.print(motor_command);  
  Serial.print(",");
  Serial.print("rotary-count:");
  Serial.print(rotary_count);   
  Serial.println("");   
  delay (100);


}

void rightEncoderEvent() {
  if (digitalRead(ENCODER_A) == HIGH) {
    if (digitalRead(ENCODER_B) == LOW) {
      encoder_count++; }
    else {
      encoder_count--; }
  } else {
    if (digitalRead(ENCODER_B) == LOW) {
      encoder_count--;
    } else {
      encoder_count++;
    }
  }
}


void RoteStateChanged() {
  if (digitalRead(DAT) == LOW) {
    if (digitalRead(CLK) == LOW) {
      rotary_count++; }
    else {
      rotary_count--; }
  }

}

