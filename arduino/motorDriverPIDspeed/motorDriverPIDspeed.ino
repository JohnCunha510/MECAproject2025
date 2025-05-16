
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
float speed_float = 0;
long serial_time = 0;

float pid_Kp = 5;
float pid_Ki = 0;
float pid_Kd = 0;

double dt = 0;
int dt_print = 0;
float Kp = 2.0; // adjust this experimentally
int error = 0;
int error_last = 0;
float Ki = 0.5; // adjust this experimentally
float integral = 0;
float Kd = 1.0;
float derivative = 0;


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
  set_speed = map(rotary_count, 0, 100, 0, 255) * 2;

  if(millis() > serial_time + 10) {// Every 100 ms

    dt = (double) (millis() - serial_time) / 1000;
    dt_print = (int) dt * 1000;
    serial_time = millis();

    speed_float = ((abs(encoder_count - encoder_count_last) / 748.0) * (60 / dt));
    speed_int = (int) speed_float;
    encoder_count_last = encoder_count;
    //set_speed = 100;

    // Proportional controller
    Kp = 1.2; // adjust this experimentally
    error = set_speed - speed_int;
    Ki = 1;
    integral += error * dt;
    integral = max(-100, min(100, integral));
    Kd = 0.05;
    derivative = (error - error_last) / dt;
    error_last = error;

    motor_command = Kp * error + Ki * integral + Kd * derivative;

    motor_command = max(0, min(500, motor_command));

    pwm_output = map(motor_command, 0, 200, 0, 255);
    OCR2A = pwm_output;

    
    motor_current = analogRead(SNS_B); //map(analogRead(SNS_B), 0, 676, 0, 2);
    


    motor_current_smooth = alpha * motor_current + (1 - alpha) * motor_current_smooth_last;
    motor_current_smooth_last = motor_current_smooth;

    Serial.print("dt:");
    Serial.print(dt);
    Serial.print(",");
    Serial.print("encode:");
    Serial.print(((abs(encoder_count - encoder_count_last) / 748.0) * (60 / dt)));
    Serial.print(",");
    Serial.print("pwm-output:");
    Serial.print(pwm_output);  
    Serial.print(",");
    Serial.print("motor-speed:");
    Serial.print(speed_int);  
    Serial.print(",");
    Serial.print("error:");
    Serial.print(error);  
    Serial.print(",");
    Serial.print("motor-command:");
    Serial.print(motor_command);  
    Serial.print(",");
    Serial.print("set-speed:");
    Serial.print(set_speed);   
    Serial.println("");   

  }


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

