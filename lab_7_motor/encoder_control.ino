// Define encoder pin numbers

#define ENCODER_PIN_A 2   // Channel A (encoder)

#define ENCODER_PIN_B 3   // Channel B (encoder)

 

// Define motor control pins (DRV8838)

#define MOTOR_PWM_PIN 9    // PWM control for motor speed (connect to DRV8838 IN2)

#define MOTOR_DIR_PIN 10    // Direction control for motor (connect to DRV8838 IN1)

 

// Define encoder and gear parameters

#define PPR 7           // Pulses per revolution from the encoder
#define gearRatio 50       // Example gear ratio (change as needed)
#define CPR (4 * PPR * gearRatio)  // Calculated Counts Per Revolution

 

volatile int16_t encoderPosition = 0;  // Position of the encoder (in counts)

volatile uint8_t lastEncoded = 0;      // Last encoded value to detect direction

 

// Variables for target angle and counts

int16_t targetAngle = 0;              // Target angle (set by Serial input)

int16_t targetCounts = 0;             // Target counts (calculated from target angle)

 

void setup() {

  // Initialize serial communication for debugging and input

  Serial.begin(9600);

  Serial.println("Enter target angle in degrees:");

 

  // Set encoder pins as input

  pinMode(ENCODER_PIN_A, INPUT);

  pinMode(ENCODER_PIN_B, INPUT);

 

  // Enable internal pull-up resistors (optional, depending on your encoder)

  digitalWrite(ENCODER_PIN_A, HIGH);

  digitalWrite(ENCODER_PIN_B, HIGH);

 

  // Set motor control pins as output

  pinMode(MOTOR_PWM_PIN, OUTPUT);

  pinMode(MOTOR_DIR_PIN, OUTPUT);

 

  // Attach interrupts to handle changes on channel A and B

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);

 

  // Stop the motor initially

  analogWrite(MOTOR_PWM_PIN, 0);

}

 

void loop() {

  // Check if data is available on the Serial Monitor

  if (Serial.available() > 0) {

    // Read the input and convert to integer (target angle)

    targetAngle = Serial.parseInt();

 

    // Calculate target counts based on the target angle

    targetCounts = (CPR * targetAngle) / 360;

 

    // Display target angle and target counts

    Serial.print("Target Angle: ");

    Serial.print(targetAngle);

    Serial.print(" degrees, Target Counts: ");

    Serial.println(targetCounts);

  }

 

  // Output the encoder position and the target counts for debugging

  Serial.print("Encoder Position (counts): ");

  Serial.print(encoderPosition);

  Serial.print("\tTarget Counts: ");

  Serial.println(targetCounts);

 

  // Control the motor to reach the target counts corresponding to the target angle

  // while (encoderPosition <= targetCounts)
  // {
  //   digitalWrite(MOTOR_DIR_PIN, LOW);  // Set motor direction (LOW for forward)

  //   analogWrite(MOTOR_PWM_PIN, 200);   // Set motor speed (PWM value between 0 and 255)


  // }

  // analogWrite(MOTOR_PWM_PIN, 0);


  if (encoderPosition < targetCounts) {

    // Move the motor forward
    Serial.println("Moving forward");

    digitalWrite(MOTOR_DIR_PIN, LOW);  // Set motor direction (LOW for forward)

    analogWrite(MOTOR_PWM_PIN, 30);   // Set motor speed (PWM value between 0 and 255)

  }

  else if (encoderPosition > targetCounts) {

    // Move the motor backward
    Serial.println("Moving backkward");
    digitalWrite(MOTOR_DIR_PIN, HIGH);  // Set motor direction (HIGH for backward)
    analogWrite(MOTOR_PWM_PIN, 30);    // Set motor speed
    analogWrite(MOTOR_PWM_PIN, 30);    // Set motor speed
  }

  else {

    // Stop the motor when the target is reached

    analogWrite(MOTOR_PWM_PIN, 0);

  }

 

  delay(100); // Slow down the serial output for readability

}

 

// Interrupt service routine (ISR) to handle encoder changes

void updateEncoder() {

  bool MSB = digitalRead(ENCODER_PIN_A); // Most Significant Bit (Channel A)

  bool LSB = digitalRead(ENCODER_PIN_B); // Least Significant Bit (Channel B)

 

  uint8_t encoded = (MSB << 1) | LSB;     // Combine both channels to get a 2-bit value

  uint8_t sum = (lastEncoded << 2) | encoded;  // Form a 4-bit combination of the last and current state

 

  // Use a lookup table or logic to update position

  // if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPosition++;

  // if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPosition--;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPosition--;

  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPosition++;

 

  lastEncoded = encoded;  // Save current state for the next interrupt

}