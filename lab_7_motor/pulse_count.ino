volatile int pulseCountA = 0;
volatile int pulseCountB = 0;

void setup() {
  // Set up the pin modes for A and B channels
  pinMode(2, INPUT);  // Channel A
  pinMode(3, INPUT);  // Channel B

  // Attach interrupts to the A and B channels
  attachInterrupt(digitalPinToInterrupt(2), countPulseA, RISING);
  attachInterrupt(digitalPinToInterrupt(3), countPulseB, RISING);

  Serial.begin(9600);
}

void loop() {
  // Display the pulse count
  Serial.print("Pulse A: ");
  Serial.println(pulseCountA);
  Serial.print("Pulse B: ");
  Serial.println(pulseCountB);

  delay(500);
}

void countPulseA() {
  pulseCountA++;
}

void countPulseB() {
  pulseCountB++;
}
