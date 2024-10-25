// Define the control inputs
#define MOT_A1_PIN 10
#define MOT_A2_PIN 9
#define ENCODER_PIN_A 2   // Channel A (encoder)
#define ENCODER_PIN_B 3   // Channel B (encoder)

#define PPR 7           // Pulses per revolution from the encoder
#define gearRatio 50       // Example gear ratio (change as needed)
#define CPR (4 * PPR * gearRatio)  // Calculated Counts Per Revolution 

volatile int16_t encoderPosition = 0;  // Position of the encoder (in counts)
volatile uint8_t lastEncoded = 0;      // Last encoded value to detect direction

// Variables for target angle and counts
// For some reason I need to set the value to negative to get positve target counts
// Setting Target Angle to 180 will compute a negative target counts
int16_t targetAngle = -270;              // Target angle (set by Serial input)
int16_t targetCounts = 0;             // Target counts (calculated from target angle)
bool printTargetOnce = true;
void setup(void)
{


  // Set encoder pins as input
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);

  // Enable internal pull-up resistors (optional, depending on your encoder)
  digitalWrite(ENCODER_PIN_A, HIGH);
  digitalWrite(ENCODER_PIN_B, HIGH);

  // Set all the motor control inputs to OUTPUT
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);

  // Attach interrupts to handle changes on channel A and B
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);

  // Initialize the serial UART at 9600 baud
  Serial.begin(9600);

}

void loop(void)
{
  targetCounts = (CPR * targetAngle) / 360;
  if (printTargetOnce)
  {
    Serial.print("Target Angle: ");
    Serial.print(targetAngle);
    Serial.print(" Target Counts: ");
    Serial.print(targetCounts);
    Serial.println();
    printTargetOnce = false;
  }

  // set_motor_pwm(255, MOT_A1_PIN, MOT_A2_PIN);
  int16_t buffer = 5;   
  int pwm_command = 75;
  if (encoderPosition < targetCounts + buffer)
  {
    Serial.println("Less than");
    set_motor_pwm(pwm_command, MOT_A1_PIN, MOT_A2_PIN);
  }
  else if (encoderPosition >= targetCounts - buffer)
  {
    Serial.println("Greater than");
    set_motor_pwm(-pwm_command, MOT_A1_PIN, MOT_A2_PIN);
  }
  // Don't do anything
  else
  {  
    Serial.println("Not doing anything");
    set_motor_pwm(0, MOT_A1_PIN, MOT_A2_PIN);
  }

  Serial.print("Encoder Position (counts): ");
  Serial.print(encoderPosition);
  Serial.print("\tTarget Counts: ");
  Serial.println(targetCounts);

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

void updateEncoder() 
{
  bool MSB = digitalRead(ENCODER_PIN_A); // Most Significant Bit (Channel A)
  bool LSB = digitalRead(ENCODER_PIN_B); // Least Significant Bit (Channel B)

  uint8_t encoded = (MSB << 1) | LSB;     // Combine both channels to get a 2-bit value
  uint8_t sum = (lastEncoded << 2) | encoded;  // Form a 4-bit combination of the last and current state

  // Use a lookup table or logic to update position

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPosition++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPosition--;

  // if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPosition--;

  // if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPosition++;

  lastEncoded = encoded;  // Save current state for the next interrupt

}
