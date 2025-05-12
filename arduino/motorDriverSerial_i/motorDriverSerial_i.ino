///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                             DEFINES
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define LENGTH_INPUT_BUFFER      50
#define LENGTH_OUT_DATA_BUFFER   50
#define LENGTH_OUT_FRAME_BUFFER  60

#define RCV_ST_IDLE              0
#define RCV_ST_CMD               1
#define RCV_ST_DATA_LENGTH       2
#define RCV_ST_DATA              3
#define RCV_ST_CHECKSUM          4

#define FRAME_START              0x8A
#define FRAME_ESCAPE_CHAR        0x8B
#define FRAME_XOR_CHAR           0x20

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
byte input_buffer[LENGTH_INPUT_BUFFER];            // Incoming data buffer
byte out_frame_buffer[LENGTH_OUT_FRAME_BUFFER];     // Data buffer for output Frame
byte out_buffer[LENGTH_OUT_DATA_BUFFER];           // Data buffer for output data

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

  // FastPWM pin 11
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS22);
  OCR2A = 180;
  OCR2B = 50;

  //speed_time = millis()

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
    //Serial.println("Change Direction!!");
    motor_direction = !motor_direction;
  }

  if (motor_direction) {
    digitalWrite(DIR_B, HIGH);
  } else {
    digitalWrite(DIR_B, LOW);
  }

  rotary_count = min(100, max(0, rotary_count));
  pwm_output = map(rotary_count, 0, 100, 0, 255);
  //analogWrite(PWM_B, pwm_output);
  if (pwm_output <= 0) {
    pinMode(11, INPUT);
  } else {
    pinMode(11, OUTPUT);
  }
  OCR2A = pwm_output;

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
    
    speed_int = (int) ((abs(encoder_count - encoder_count_last) / 748.0) * (60 / 0.1));
    encoder_count_last = encoder_count;
    
    byte current[] = {(motor_current_smooth & 0xFF00) >> 8, (motor_current_smooth & 0x00FF)};
    //byte checksum = FRAME_START + 0x0B + 0x02 + current[0] + current[1];
    //byte FrameBuffer[] = {FRAME_START, 0x0B, 0x02, current[0], current[1], checksum};
    //Serial.write(FrameBuffer, 6);

    sendFrameBuffer(0x0B, current, 2);

    //float voltage_int = map(pwm_output, 0, 255, 0, 9000);
    byte voltage[] = {(rotary_count & 0xFF00) >> 8, (rotary_count & 0x00FF)};
    sendFrameBuffer(0x0F, voltage, 2);

    byte speed[] = {(speed_int & 0xFF00) >> 8, (speed_int & 0x00FF)};
    sendFrameBuffer(0x0C, speed, 2);

    /*
    Serial.print("motor-speed:");
    Serial.print(speed_int);
    Serial.print(",");
    Serial.print("ratio:");
    Serial.print(abs(encoder_count - encoder_count_last) / 748.0);
    Serial.print(",");
    Serial.print("encoder-count:");
    Serial.print(encoder_count);  
    Serial.print(",");
    Serial.print("encoder-count-last:");
    Serial.print(encoder_count_last);  
    Serial.println("");

    */

        
  }


}


void sendFrameBuffer(byte cmd, byte* data_buffer, int data_length)
{
  int i = 0;
  out_frame_buffer[0] = FRAME_START;                             // Start Frame
  out_frame_buffer[1] = cmd;                                     // Comando
  out_frame_buffer[2] = data_length;                                  // Lunghezza campo dati
  for(i = 0; i < data_length; i++)
    out_frame_buffer[i + 3] = data_buffer[i];
  out_frame_buffer[data_length + 3] = calculateChecksum(data_length + 3);  // Checksum

  sendFrame(data_length + 4);
}

byte calculateChecksum(byte length)
{
  byte rv = 0, index;
  for(index = 0; index < length; index++)
  {
    rv += out_frame_buffer[index];
  }
  return rv;
}

void sendFrame(int length)
{
  int i;
  byte dataToSend = 0;

  out_buffer[dataToSend++] = FRAME_START;

  for(i = 1; i < length; i++)
  {
    if(out_frame_buffer[i] == FRAME_START || out_frame_buffer[i] == FRAME_ESCAPE_CHAR)
    {
      out_buffer[dataToSend++] = FRAME_ESCAPE_CHAR;
      out_buffer[dataToSend++] = out_frame_buffer[i] ^ FRAME_XOR_CHAR;
    } else
      out_buffer[dataToSend++] = out_frame_buffer[i];
  }
  
  Serial.write(out_buffer, dataToSend);
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

