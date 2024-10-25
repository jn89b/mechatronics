#define IN2 10
#define IN1 9
int dir;

/*
Used to make sure motor is working correctly 
Sends a high low to and makes rotate clockwise or counter clockwise
 */

void setup() {
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  dir = 1;
  setMotor(dir, IN1, IN2);
  dir = 0;
  setMotor(dir, IN1, IN2);
  dir = -1;
  setMotor(dir, IN1, IN2);
  dir = 0;
  setMotor(dir, IN1, IN2);
}

void loop() {
  dir = 1;
  setMotor(dir, IN1, IN2);
  dir = 0;
  setMotor(dir, IN1, IN2);
  dir = -1;
  setMotor(dir, IN1, IN2);
  dir = 0;
  setMotor(dir, IN1, IN2);

}

void setMotor(int dir, int in1, int in2) {
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print(1);
    delay(1000);
  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print(-1);
    delay(1000);
  } else if (dir == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print(0);
    delay(500);
  }
  Serial.println("");
}