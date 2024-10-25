#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define MOT_A1_PIN 10
#define MOT_A2_PIN 9

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/

// #define PPR 7           // Pulses per revolution from the encoder
// #define gearRatio 20       // Example gear ratio (change as needed)
// #define CPR (4 * PPR * gearRatio)  // Calculated Counts Per Revolution 

int PPR = 7;
int gearRatio = 40;
float CPR;

volatile int16_t encoderPosition = 0;  // Position of the encoder (in counts)
volatile uint8_t lastEncoded = 0;      // Last encoded value to detect direction
// Variables for target angle and counts
// For some reason I need to set the value to negative to get positve target counts
// Setting Target Angle to 180 will compute a negative target counts
int16_t targetAngle = 25;              // Target angle (set by Serial input)
int16_t targetCounts = 0;             // Target counts (calculated from target angle)
bool printTargetOnce = true;

bool done = false;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);

  // Set all the motor control inputs to OUTPUT
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  delay(1000);
}

void loop() {
  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0; 
  CPR = (2 * PPR * gearRatio);
  targetCounts = (CPR * targetAngle) / 360;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    encoderPosition = posi;
  }


  // encoderPosition = pos;
  // set_motor_pwm(255, MOT_A1_PIN, MOT_A2_PIN);
  int pwm_command = 60;
  int16_t buffer = 10; 

  // if (!done)
  // {

  if ((encoderPosition <= targetCounts + buffer) && 
    (encoderPosition >= targetCounts - buffer))
  {
    set_motor_pwm(0, MOT_A1_PIN, MOT_A2_PIN);
  }
  else if (encoderPosition < targetCounts + buffer)
  {
    Serial.println("Less than");
    set_motor_pwm(-pwm_command, MOT_A1_PIN, MOT_A2_PIN);
  }
  else if (encoderPosition >= targetCounts - buffer)
  {
    Serial.println("Greater than");
    set_motor_pwm(pwm_command, MOT_A1_PIN, MOT_A2_PIN);
  }
  else{
    Serial.println("Tolerance met");
    set_motor_pwm(0, MOT_A1_PIN, MOT_A2_PIN);
  }
  // {  
  //   Serial.println("Not doing anything");
  //   set_motor_pwm(0, MOT_A1_PIN, MOT_A2_PIN);
  //   done = true;
  // }

  Serial.print("Encoder Position (counts): ");
  Serial.print(encoderPosition);
  Serial.print("\tTarget Counts: ");
  Serial.println(targetCounts);



  Serial.println(pos);
}

void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN)
{
  if (pwm < 0) {  // reverse speeds
    analogWrite(IN1_PIN, -pwm);  // IN1 set to reverse speed
    analogWrite(IN2_PIN, 0);     // IN2 set to LOW
  } else if (pwm > 0) {  // forward speeds
    analogWrite(IN1_PIN, 0);     // IN1 set to LOW
    analogWrite(IN2_PIN, pwm);   // IN2 set to forward speed
  } else {  // stop
    Serial.println("Doing nothing");
    analogWrite(IN1_PIN, 0);     // Stop motor
    analogWrite(IN2_PIN, 0);
  }
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}