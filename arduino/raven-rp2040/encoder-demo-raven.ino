#define CLK 2
#define DT 3

volatile long encoderPos = 0;

void setup() {
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(CLK), updateEncoder, CHANGE);
}

void loop() {
  Serial.println(encoderPos);
  delay(200);
}

void updateEncoder() {
  int clkState = digitalRead(CLK);
  int dtState = digitalRead(DT);

  if (clkState == dtState) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}
//phobia was here :)