#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

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
#define tolerance 5

volatile int32_t encoderPosition = 0;  // Position of the encoder (in counts)
volatile int32_t lastEncoded = 0;      // Last encoded value to detect direction

// Variables for target angle, target counts, and motor speed
int16_t targetAngle = 90;                // Target angle (set by Serial input)
int32_t targetCounts = 0;               // Target counts (calculated from target angle)
int pwmValue = 90;
bool reachedTarget = false;             // Flag to track if target is reached

// PID parameters 
// https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
// https://www.sciencedirect.com/topics/computer-science/ziegler-nichols-method
bool use_pid = true; // Set to true if you want to use this  
const float Ku = 1.8;
const float Tu = 0.25;
const float freq = 1/Tu;
const float kp = 0.5*Ku;//2.0*0.6; // Ku found to be at 2.0
// Tu Oscillations = 0.25
const float ki = 0.0;//2.0*freq;//0.4*Ku/Tu;
const float kd = 0.0; //0.125/freq; //0.066*Ku*Tu;//0.075*2.0*0.15;
int32_t actualAngle = 0.0;
long prevT = 0;
float error_prev = 0;
float error_int = 0;
unsigned long start_time = millis();
bool recievedInput = false;

const int maxPWM = 255;
const int minPWM = 50;

// Global variables
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
#define OUTPUT_READABLE_EULER
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() 
{
    mpuInterrupt = true;
}


void initMPU()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

    Serial.begin(57600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // // wait for ready
    // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    // while (Serial.available() && Serial.read()); // empty buffer
    // while (!Serial.available());                 // wait for data
    // while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void setup() {
  // Initialize serial communication for debugging and input
  initMPU();

  // Set encoder pins as input
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);

  // Set motor control pins as output
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  delay(1000);
  targetCounts =  static_cast<int32_t>((static_cast<int64_t>(CPR) * targetAngle) / 360);  // Recalculate target counts when target angle changes
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);

}

void serializeData()
{
  // Used to serialize and print the data
  // Print the encoderPosition Desired Position 
  // Serial.print(current_time_sec);
  // Serial.print(",");
  // Serial.print(encoderPosition);
  // Serial.print(",");
  // Serial.print(targetCounts);
  // Serial.println("");  
}

// Used to get the PID gains
int getPIDGains(int current_error)
{
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );  
    prevT = currT;
    
    // We get the errors for the derivative and integral gains
    // derivative
    float dedt = (current_error-error_prev)/(deltaT);
    // integral
    error_int = error_int + current_error*deltaT;
    // Constrain the gains
    int pid_pwm = kp*current_error + kd*dedt+ ki*error_int;

    // To be safe in real life we would constraint
    // our output
    if (abs(current_error) <= tolerance)
    {
      pid_pwm = 0;
    }
    else if (abs(pid_pwm) >= maxPWM)
    {
      pid_pwm = maxPWM;
    }
    else if (abs(pid_pwm) <= minPWM)
    {
      pid_pwm = minPWM;
    }  
    return abs(pid_pwm);

}

void loop() {

  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  { // Get the Latest packet 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    targetAngle = euler[2] * 180/M_PI;
    targetCounts =  static_cast<int32_t>((static_cast<int64_t>(CPR) * targetAngle) / 360);  // Recalculate target counts when target angle changes
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);
  }

  int error = targetCounts - encoderPosition;
  double current_time_sec = (millis() - start_time)/1000.0;
  // Print the encoderPosition Desired Position 
  // Serial.print(current_time_sec);
  // Serial.print(",");
  // Serial.print(encoderPosition);
  // Serial.print(",");
  // Serial.print(targetCounts);
  // Serial.println("");  
  Serial.print("Euler\t");
  Serial.print(euler[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(euler[1] * 180/M_PI);
  Serial.print("\t");
  Serial.print(euler[2] * 180/M_PI);
  Serial.println("");

  // Compute PID gains if we are using it
  if (use_pid == true)
  {
    pwmValue = getPIDGains(error);
  }
  if (abs(error) <= tolerance) 
  {
   stopMotor();
  }
  else {
    if (error <= tolerance) {  // Forward
      moveForward(pwmValue);
      // Serial.print("Moving Forward - Encoder Position (counts): ");
    } 
    else if (error > tolerance) {  // Reverse
      moveBackward(pwmValue);
    }
  }
}

void moveForward(int pwmValue) 
{
  analogWrite(MOTOR_IN1, pwmValue);
  digitalWrite(MOTOR_IN2, 0);
}

void moveBackward(int pwmValue)
{
  digitalWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, pwmValue);
}

void parseSerialInput(String input) 
{
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
void stopMotor() 
{
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, 0);
  detachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A));
  detachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B));
  // reachedTarget = true;  // Set flag to avoid restarting
}

// Interrupt service routine (ISR) to handle encoder changes
void updateEncoder() 
{
  bool MSB = digitalRead(ENCODER_PIN_A); // Most Significant Bit (Channel A)
  bool LSB = digitalRead(ENCODER_PIN_B); // Least Significant Bit (Channel B)

  uint8_t encoded = (MSB << 1) | LSB;         // Combine both channels to get a 2-bit value
  uint8_t sum = (lastEncoded << 2) | encoded; // Form a 4-bit combination of the last and current state

  // Use logic to update position
  if (sum == 0b0100 || sum == 0b0010 || sum == 0b1011 || sum == 0b1101) encoderPosition--;
  if (sum == 0b1110 || sum == 0b1000 || sum == 0b0001 || sum == 0b0111) encoderPosition++;

  lastEncoded = encoded;  // Save current state for the next interrupt
}
