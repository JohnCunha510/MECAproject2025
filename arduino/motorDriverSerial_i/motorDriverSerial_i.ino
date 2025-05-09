
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
int pwm_output = 100;
int motor_current = 0;
float alpha = 0.2;
int motor_current_smooth = 0;
int motor_current_smooth_last = 0;

char FrameBuffer;
long serial_time = 0;

void setup() {
  Serial.begin(9600);
  delay(100);
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


}

void loop() {

  /*
  if(Serial.available() > 0)
    {
      byte receivedByte = (byte)Serial.read();
      if(receivedByte == FRAME_ESCAPE_CHAR)
      {
          g_xorValue = FRAME_XOR_CHAR;
      } else
      {
        receivedByte ^= g_xorValue;
        g_xorValue = 0x00;

        switch(g_ReceiverStatus)
        {
          case RCV_ST_IDLE:
          {
            if(receivedByte == FRAME_START)
            {   
              g_BufferIndex = 0;
              g_InputBuffer[g_BufferIndex++] = receivedByte;
              g_Checksum = receivedByte;
              g_ReceiverStatus = RCV_ST_CMD;
            }
          } break;

          case RCV_ST_CMD:
          {
            g_InputBuffer[g_BufferIndex++] = receivedByte;
            g_Checksum += receivedByte;
            g_ReceiverStatus = RCV_ST_DATA_LENGTH;
          } break;

          case RCV_ST_DATA_LENGTH:
          {
            g_DataLength = receivedByte;
            if(g_DataLength > 0)
            {
                g_InputBuffer[g_BufferIndex++] = receivedByte;
                g_Checksum += receivedByte;
                g_ReceiverStatus = RCV_ST_DATA;
            } else
            {   
                g_ReceiverStatus = RCV_ST_IDLE;
            }
          } break;

          case RCV_ST_DATA:
          {
            g_InputBuffer[g_BufferIndex++] = receivedByte;
            g_Checksum += receivedByte;
            if(--g_DataLength == 0)
                g_ReceiverStatus = RCV_ST_CHECKSUM;
          } break;

          case RCV_ST_CHECKSUM:
          {
            if(receivedByte == g_Checksum)
            {   
              g_ReceiverStatus = RCV_ST_IDLE;
              g_InputBuffer[g_BufferIndex++] = receivedByte;

              switch(g_InputBuffer[INDEX_CMD])
              {
                case CMD_PWM_LED_R:
                {
                    ledcWrite(ledChRed, GetDataByte(g_InputBuffer));
                } break;

                case CMD_PWM_LED_G:
                {
                    ledcWrite(ledChGreen, GetDataByte(g_InputBuffer));
                } break;

                case CMD_PWM_LED_B:
                {
                    ledcWrite(ledChBlue, GetDataByte(g_InputBuffer));  
                } break; 

                case CMD_ADC_ENABLE:
                {
                    g_AdcEnabled = (bool)GetDataByte(g_InputBuffer);
                }
              }
            }
          } break;
        }
      }
    } else {

      */

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
  pwm_output = map(rotary_count, 0, 100, 0, 255);
  analogWrite(PWM_B, pwm_output);

  motor_current = map(analogRead(SNS_B), 0, 676, 0, 2000);
  
  /*
  motor_current_smooth = 0;
  for (int i = 0; i < 10; i++) {
    motor_current_values[i] = motor_current_values[i+1];
    motor_current_smooth += motor_current_values[i+1];
  }
  motor_current_values[10] = motor_current;
  motor_current_smooth += motor_current_values[10];
  motor_current_smooth = motor_current_smooth / 10;
  */

  motor_current_smooth = alpha * motor_current + (1 - alpha) * motor_current_smooth_last;
  motor_current_smooth_last = motor_current_smooth;

  //Serial.print("motor-current:");
  //Serial.print(motor_current);
  //Serial.print(",");
  //Serial.print("motor-current-smooth:");
  //Serial.print(motor_current_smooth);  
  //Serial.print(",");
  //Serial.print("encoder-count:");
  //Serial.print(encoder_count);  
  //Serial.print(",");
  //Serial.print("rotary-count:");
  //Serial.print(rotary_count);   
  //Serial.println("");

  if(millis() > serial_time + 100) {// Every 100 ms
    serial_time = millis();
    byte FrameBuffer = motor_current_smooth;
    Serial.write(&FrameBuffer, 1);
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

