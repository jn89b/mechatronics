//Code for the QRE1113 Analog board
//Outputs via the serial terminal - Lower numbers mean more reflected
int QRE1113_Pin = 1; //connected to analog A1
const float MAX_ADC = 1023.0;
const float MAX_VOLTAGE = 5; // 5V
const bool use_op_amp = false;
float op_amp_gain = 1; // 1 means no gain

float computeOutputVoltage(int analog_value){
  return (analog_value * MAX_VOLTAGE) / MAX_ADC;
}

void setup(){
  Serial.begin(9600);

  // if we are using an op-amp we need to set the gain
    if(use_op_amp){
        op_amp_gain = 4.7; // refer to the datasheet for the gain
    }

}


void loop(){

    // Note this value is inversely proportional to the amount of light reflected
    // So higher values mean more light reflected meaning the sensor is further away from the object
    // Lower values mean less light reflected meaning the sensor is closer to the object
    int QRE_Value = analogRead(QRE1113_Pin);
    
    // This is the actual output voltage of the sensor
    float output_voltage = computeOutputVoltage(QRE_Value);

    // Print the inverted value with a label on the same line
    // The idea is that since the values are discrete you have really poor resolution
    // So you can't really use the values to determine distance
    // What we can do instead is use an op-amp to amplify the signal to scale up the resolution
    Serial.print("QRE Value: ");
    Serial.print(QRE_Value); 
    Serial.println();   
    Serial.print("Output Voltage: ");
    Serial.print(output_voltage);
    Serial.println(); // Add a blank line between each loop iteration
    Serial.println();
}