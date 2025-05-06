const int interruptA = 0;       
const int interruptB = 1;       
int CLK = 4;     // PIN2
int DAT = 3;     // PIN3
int BUTTON = 2;  // PIN4
int LED1 = 13;    // PIN5
int LED2 = 6;    // PIN6
int COUNT = 0;
int pressed = false;
int lastPressed = false;

void setup() 
 {
  attachInterrupt(digitalPinToInterrupt(DAT), RoteStateChanged, FALLING);
 // attachInterrupt(interruptB, buttonState, FALLING);
  pinMode(CLK, INPUT); 
  digitalWrite(2, HIGH);  // Pull High Restance  
  pinMode(DAT, INPUT); 
  digitalWrite(3, HIGH);  // Pull High Restance 
 
  pinMode(BUTTON, INPUT); 
  digitalWrite(4, HIGH);  // Pull High Restance
  pinMode(LED1, OUTPUT); 
  pinMode(LED2, OUTPUT); 
  Serial.begin(9600);
 }


void loop() 
{
  lastPressed = pressed;
  pressed = digitalRead(BUTTON);
  if (lastPressed == false && pressed == true) {
    COUNT = 0;
    Serial.println("STOP COUNT = 0");
  }
  
  Serial.print("Count: ");
  Serial.println(COUNT);  
  delay (100);
  
}

//-------------------------------------------
void RoteStateChanged() //When CLK  FALLING READ DAT
{
  
  if (digitalRead(CLK) == LOW) {
    COUNT++; }
  else {
    COUNT--; }
  

  /*
  Serial.println(COUNT);  
  COUNT = COUNT +1;
  */
}

