// Define encoder pin numbers
#define ENCODER_PIN_A 2   // Channel A (encoder)
#define ENCODER_PIN_B 3   // Channel B (encoder)

// Define motor control pins (DRV8833)
#define MOTOR_IN1 10       // Control pin 1 (for speed and direction)
#define MOTOR_IN2 9      // Control pin 2 (for speed and direction)

 

// Define encoder and gear parameters

#define PPR 7             // Pulses per revolution from the encoder

#define gearRatio 20      // Example gear ratio (change as needed)

#define CPR (4 * PPR * gearRatio)  // Calculated Counts Per Revolution

 

// Allowable error in counts

const float tolerance = 3;

 

volatile int32_t encoderPosition = 0;  // Position of the encoder (in counts)

volatile int32_t lastEncoded = 0;      // Last encoded value to detect direction

 

// Variables for target angle, target counts, and motor speed

int16_t targetAngle = 90;                // Target angle (set by Serial input)

int32_t targetCounts = 0;               // Target counts (calculated from target angle)

int targetRPM = 100;                      // Target RPM (set by Serial input)

int maxRPM = 200;                       // Maximum motor speed in RPM

 

void setup() {

  // Initialize serial communication for debugging and input

  Serial.begin(9600);

  // Serial.println("Enter target angle and RPM in this format: angle,rpm (e.g., 90,150)");

 

  // Set encoder pins as input

  pinMode(ENCODER_PIN_A, INPUT);

  pinMode(ENCODER_PIN_B, INPUT);

 

  // Set motor control pins as output

  pinMode(MOTOR_IN1, OUTPUT);

  pinMode(MOTOR_IN2, OUTPUT);

 

  // Attach interrupts to handle changes on channel A and B

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);

 

  // Stop the motor initially

  analogWrite(MOTOR_IN1, 0);

  analogWrite(MOTOR_IN2, 0);

}

 

void loop() {

  // Check if data is available on the Serial Monitor

  // if (Serial.available() > 0) {

  //   String inputString = Serial.readStringUntil('\n');  // Read input string until newline

  //   parseSerialInput(inputString);  // Function to parse input for angle and RPM

 

  //   // Calculate target counts based on the target angle

  targetCounts = (CPR * targetAngle) / 360;

 

  //   // Display target angle, target RPM, and target counts

  //   Serial.print("Target Angle: ");

  //   Serial.print(targetAngle);

  //   Serial.print(" degrees, Target Counts: ");

  //   Serial.print(targetCounts);

  //   Serial.print(", Target RPM: ");

  //   Serial.println(targetRPM);

  // }

 

  // Calculate PWM value based on target RPM

  int pwmValue = (targetRPM * 255) / maxRPM;

  pwmValue = constrain(pwmValue, 0, 255); // Constrain PWM to a max of 255

  Serial.print("PWM value: ");
  Serial.print(pwmValue);
  Serial.print("TargetCount value: ");
  Serial.print(targetCounts);

  int error = targetCounts - encoderPosition;


  if (abs(error) <= tolerance) {

    // Stop the motor when within the tolerance range

    // analogWrite(MOTOR_IN1, 0);

    // analogWrite(MOTOR_IN2, 0);
    set_motor_pwm(0, MOTOR_IN1, MOTOR_IN2);
    Serial.println("Target reached within tolerance range. Motor stopped.");

  }

  else if (error > 0) {

    // Forward motion

    // analogWrite(MOTOR_IN1, pwmValue);  // Set PWM for IN1 (forward motion)

    // analogWrite(MOTOR_IN2, 0);                 // Set IN2 to 0 for forward motion

    set_motor_pwm(pwmValue, MOTOR_IN1, MOTOR_IN2);
    Serial.print("Moving Forward - Encoder Position (counts): ");

    Serial.println(encoderPosition);

  }

  else if (error < 0) {

    // Reverse motion

    // analogWrite(MOTOR_IN1, 0);                 // Set IN1 to 0 for reverse motion
    // analogWrite(MOTOR_IN2, pwmValue);  // Set PWM for IN2 (reverse motion)

    set_motor_pwm(-pwmValue, MOTOR_IN1, MOTOR_IN2);
    Serial.print("Moving Backward - Encoder Position (counts): ");
    Serial.println(encoderPosition);

  }

 

  delay(10);  // Small delay for smoother motor control and less noisy Serial Monitor output

}

 

// Function to parse the serial input (targetAngle, targetRPM)

void parseSerialInput(String input) {

  // Remove any spaces or unwanted characters

  input.trim();

 

  // Split the input string into targetAngle and targetRPM using comma as a delimiter

  int commaIndex = input.indexOf(',');  // Find the position of the comma

  if (commaIndex > 0) {

    String angleString = input.substring(0, commaIndex);  // Get the angle part

    String rpmString = input.substring(commaIndex + 1);   // Get the RPM part

 

    // Convert the strings to integers

    targetAngle = angleString.toInt();

    targetRPM = rpmString.toInt();

  } else {

    Serial.println("Invalid input. Use the format: angle,rpm (e.g., 90,150)");

  }

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
    analogWrite(IN1_PIN, 0);     // Stop motor
    analogWrite(IN2_PIN, 0);
  }
}


// Interrupt service routine (ISR) to handle encoder changes

void updateEncoder() {

  bool MSB = digitalRead(ENCODER_PIN_A); // Most Significant Bit (Channel A)

  bool LSB = digitalRead(ENCODER_PIN_B); // Least Significant Bit (Channel B)

 

  uint8_t encoded = (MSB << 1) | LSB;     // Combine both channels to get a 2-bit value

  uint8_t sum = (lastEncoded << 2) | encoded;  // Form a 4-bit combination of the last and current state

 

  // Use a lookup table or logic to update position

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPosition++;

  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPosition--;

 

  lastEncoded = encoded;  // Save current state for the next interrupt

}