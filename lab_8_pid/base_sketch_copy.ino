// Define encoder pin numbers
#define ENCODER_PIN_A 2   // Channel A (encoder)
#define ENCODER_PIN_B 3   // Channel B (encoder)

// Define motor control pins (DRV8833)
#define MOTOR_IN1 9       // Control pin 1 (for speed and direction)
#define MOTOR_IN2 10      // Control pin 2 (for speed and direction)

// Define encoder and gear parameters
#define PPR 7             // Pulses per revolution from the encoder
#define gearRatio 50      // Example gear ratio (change as needed)
const uint32_t CPR = 4 * PPR * gearRatio;  // Calculated Counts Per Revolution

// Allowable error in counts
#define tolerance 10

volatile int32_t encoderPosition = 0;  // Position of the encoder (in counts)
volatile int32_t lastEncoded = 0;      // Last encoded value to detect direction

// Variables for target angle, target counts, and motor speed
int16_t targetAngle = 180;                // Target angle (set by Serial input)
int32_t targetCounts = 0;               // Target counts (calculated from target angle)
int pwmValue = 255;
bool reachedTarget = true;             // Flag to track if target is reached
unsigned long start_time = millis();

void setup() {
  // Initialize serial communication for debugging and input
  Serial.begin(9600);
  Serial.println("Enter target angle in this format: angle (e.g., 90) or enter 'stop' to stop the motor");

  // Set encoder pins as input
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);

  // Set motor control pins as output
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  reachedTarget = false;  // 

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);
}

void loop() {
  // Handle Serial input for stopping or setting target angle
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');  // Read input string until newline
    parseSerialInput(inputString);  // Function to parse input for angle or stop command
      // Attach interrupts to handle changes on channel A and B
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);
    reachedTarget = false;  // 
  }

  // targetCounts =  static_cast<int32_t>((static_cast<int64_t>(CPR) * targetAngle) / 360);  // Recalculate target counts when target angle changes
  int error = targetCounts - encoderPosition;

  double current_time_sec = (millis() - start_time)/1000.0;
  // Print the encoderPosition Desired Position 
  Serial.print(current_time_sec);
  Serial.print(",");
  Serial.print(encoderPosition);
  Serial.print(",");
  Serial.print(targetCounts);
  Serial.println("");  


  if (abs(error) <= tolerance && !reachedTarget) {
   stopMotor();
  //  Serial.print("Moving- Encoder Position (counts): ");
  //  Serial.println(encoderPosition);
  //  Serial.print("Target Count Set To: ");
  //  Serial.println(targetCounts);
  //  Serial.println("Target reached within tolerance range. Motor stopped.");
  }
   
  else if (!reachedTarget) {
    if (error > tolerance) {  // Forward
      analogWrite(MOTOR_IN1, pwmValue);
      digitalWrite(MOTOR_IN2, 0);
      // Serial.print("Moving Forward - Encoder Position (counts): ");
    } 
    else if (error < tolerance) {  // Reverse
      digitalWrite(MOTOR_IN1, 0);
      analogWrite(MOTOR_IN2, pwmValue);
      // Serial.print("Moving Backward - Encoder Position (counts): ");
    }
    // Serial.println(encoderPosition);
    // Serial.print("Target Count Set To: ");
    // Serial.println(targetCounts);
  }

    // Serial.print("Encoder Position ");
    // Serial.print("| TargetCounts: ");
    // Serial.print(encoderPosition);
    // Serial.print(",");
    // Serial.println(targetCounts);
  //delay(10);  // Small delay for smoother motor control and less noisy Serial Monitor output
}

void parseSerialInput(String input) {
  // Remove any spaces or unwanted characters
  input.trim();
  if (input.equalsIgnoreCase("stop")) {
    stopMotor();
    Serial.println("Motor stopped by user command.");
  } else {
    targetAngle = input.toInt();
    targetCounts =  static_cast<int32_t>((static_cast<int64_t>(CPR) * targetAngle) / 360);  // Recalculate target counts when target angle changes
    Serial.print("Target Count Set To: ");
    Serial.println(targetCounts);
  }
}

// Function to stop the motor
void stopMotor() {
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, 0);
  detachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A));
  detachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B));
  reachedTarget = true;  // Set flag to avoid restarting
}

// Interrupt service routine (ISR) to handle encoder changes
void updateEncoder() {
  bool MSB = digitalRead(ENCODER_PIN_A); // Most Significant Bit (Channel A)
  bool LSB = digitalRead(ENCODER_PIN_B); // Least Significant Bit (Channel B)

  uint8_t encoded = (MSB << 1) | LSB;         // Combine both channels to get a 2-bit value
  uint8_t sum = (lastEncoded << 2) | encoded; // Form a 4-bit combination of the last and current state

  // Use logic to update position
  if (sum == 0b0100 || sum == 0b0010 || sum == 0b1011 || sum == 0b1101) encoderPosition--;
  if (sum == 0b1110 || sum == 0b1000 || sum == 0b0001 || sum == 0b0111) encoderPosition++;

  lastEncoded = encoded;  // Save current state for the next interrupt
}
