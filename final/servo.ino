#include <Servo.h>

class ServoController 
{
  public:
    // Constructor to initialize pin and bias
    ServoController(int servo_pin, int bias_position)
      : pin(servo_pin), bias(bias_position), position(0) {}

    // Attach the servo to its pin and move to bias position
    void initialize() {
      servo.attach(pin);
      moveTo(bias); // Move to the bias position on initialization
    }

    // Move servo to specified position, including bias
    void moveTo(int pos) {
      position = pos; //+ bias;
      // position = constrain(position); // Ensure within bounds
      servo.write(position);
    }

    // Set a new bias dynamically if needed
    void setBias(int new_bias) {
      bias = new_bias;
    }

    // Get the current position (including bias)
    int getPosition() const {
      return position;
    }

  private:
    Servo servo;      // Servo object
    int pin;          // Pin connected to the servo
    int position;     // Current position of the servo (with bias applied)
    int bias;         // Bias offset for the servo position
};

// Global variables
const int servo_roll_pin = 5;
const int bias_servo_roll_position = 91;
ServoController servoRoll(servo_roll_pin, bias_servo_roll_position);

const int servo_pitch_pin = 6;
const int bias_servo_pitch_position = 90;
ServoController servoPitch(servo_pitch_pin, bias_servo_pitch_position);

void initServos() {
  servoRoll.initialize();
  servoPitch.initialize();
}

void setup() {
  initServos();
}

void loop() {
  // Example usage:
  // servoRoll.moveTo(90);
  // for (int pos = 0; pos <= 180; pos += 1) {
  //   servoRoll.moveTo(pos);
  //   delay(15);
  // }
  // for (int pos = 180; pos >= 0; pos -= 1) {
  //   servoRoll.moveTo(pos);
  //   delay(15);
  // }
}
