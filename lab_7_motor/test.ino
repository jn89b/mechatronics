// Define encoder pins
const int encoderPinA = 2;   // Channel A
const int encoderPinB = 3;   // Channel B

// Variables to store the pulse counts and position
volatile int pulseCount = 0;  // Total pulse count
float degreesPerPulse = 51.43;  // For 7 pulses per revolution, each pulse is 51.43 degrees
float currentPosition = 0;    // Current position in degrees
float lastPosition = 0;       // Previous position in degrees
unsigned long lastPulseTime = 0;  // Time of the last pulse in microseconds
float motorSpeed = 0;         // Estimated speed of the motor in degrees per second

void setup() {
  // Set up the pin modes for encoder channels
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  // Attach interrupt for the A channel
  attachInterrupt(digitalPinToInterrupt(encoderPinA), pulseISR, RISING);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Calculate the time elapsed since the last pulse
  unsigned long currentTime = micros();
  unsigned long timeSinceLastPulse = currentTime - lastPulseTime;

  // Estimate the position using linear interpolation based on motor speed
  float interpolatedPosition = lastPosition + (motorSpeed * timeSinceLastPulse / 1e6);

  // Display the interpolated position
  Serial.print("Interpolated Position: ");
  Serial.print(interpolatedPosition);
  Serial.println(" degrees");

  // Add a delay to avoid flooding the serial monitor
  delay(50);
}

// Interrupt service routine (ISR) for counting pulses
void pulseISR() {
  unsigned long currentTime = micros();
  unsigned long pulseInterval = currentTime - lastPulseTime;

  // Calculate motor speed based on the time between pulses
  motorSpeed = degreesPerPulse / (pulseInterval / 1e6);  // Speed in degrees per second

  // Update position and time
  lastPosition = pulseCount * degreesPerPulse;
  pulseCount++;
  lastPulseTime = currentTime;
}
