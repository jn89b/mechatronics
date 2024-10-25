// Define the control inputs
#define MOT_A1_PIN 10
#define MOT_A2_PIN 9

void setup(void)
{
  // Set all the motor control inputs to OUTPUT
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);

  // Initialize the serial UART at 9600 baud
  Serial.begin(9600);
}

void loop(void)
{
  // Generate a fixed motion sequence to demonstrate the motor modes.

  // Ramp speed up.
  for (int i = 0; i < 11; i++) {
    spin_and_wait(25*i, 25*i, 500);
  }
  // Full speed forward.
  spin_and_wait(255,255,2000);

  // Ramp speed into full reverse.
  for (int i = 0; i < 21 ; i++) {
    spin_and_wait(255 - 25*i, 255 - 25*i, 500);
  }

  // Full speed reverse.
  spin_and_wait(-255,-255,2000);

  // Stop.
  spin_and_wait(0,0,2000);

  // Full speed, forward, turn, reverse, and turn for a two-wheeled base.
  int duration = 4000;
  spin_and_wait(255, 255, duration);
  spin_and_wait(0, 0, 1000);
  spin_and_wait(-255, 255, duration);
  spin_and_wait(0, 0, 1000);
  spin_and_wait(-255, -255, duration);
  spin_and_wait(0, 0, 1000);
  spin_and_wait(255, -255, duration);
  spin_and_wait(0, 0, 1000);
}

/// Set the current on a motor channel using PWM and directional logic.
///
/// \param pwm    PWM duty cycle ranging from -255 full reverse to 255 full forward
/// \param IN1_PIN  pin number xIN1 for the given channel
/// \param IN2_PIN  pin number xIN2 for the given channel
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

/// Set the current on both motors.
///
/// \param pwm_A  motor A PWM, -255 to 255
/// \param pwm_B  motor B PWM, -255 to 255
void set_motor_currents(int pwm_A, int pwm_B)
{
  set_motor_pwm(pwm_A, MOT_A1_PIN, MOT_A2_PIN);
  // Print a status message to the console.
  Serial.print("Set motor A PWM = ");
  Serial.print(pwm_A);
  Serial.println();
}

/// Simple primitive for the motion sequence to set a speed and wait for an interval.
///
/// \param pwm_A  motor A PWM, -255 to 255
/// \param pwm_B  motor B PWM, -255 to 255
/// \param duration delay in milliseconds
void spin_and_wait(int pwm_A, int pwm_B, int duration)
{
  set_motor_currents(pwm_A, pwm_B);
  delay(duration);
}

