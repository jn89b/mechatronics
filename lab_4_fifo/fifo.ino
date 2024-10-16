#include <Wire.h>


#define MPU6050_ADDRESS 0x68
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define FIFO_EN 0x23
#define USER_CTRL 0x6A
#define FIFO_COUNTH 0x72
#define FIFO_R_W 0x74
#define ACCEL_SCALE_MODIFIER_16G 2048.0
#define ACCEL_SCALE_MODIFIER_8G 4096.0

int freq = 100; // Sampling rate of 30 Hz
int dt = 1000 / freq; // Time between samples in ms

void setup() {
    Wire.begin();
    Serial.begin(115200);
    // Initialize MPU6050
    mpu6050_init();
}

void loop() {
    // Read data from the FIFO
    read_fifo();
    delay(30); // delay for 30 Hz sampling rate
}

void mpu6050_init() {
    // Wake up the MPU6050
    writeRegister(PWR_MGMT_1, 0x00);
    delay(100);
    // Set sample rate to approximately 30 Hz
    writeRegister(SMPLRT_DIV, 0x1F);
    // Set DLPF (Digital Low Pass Filter) to 42 Hz
    writeRegister(CONFIG, 0x03);
    // Set gyro full scale range to +/- 2000 deg/sec
    // 250 deg/s --> 0x00, 500 deg/s --> 0x08, 1000 deg/s --> 0x10, 2000 deg/s --> 0x18
    writeRegister(GYRO_CONFIG, 0x08);
    // Wire.write; 2g --> 0x00, 4g --> 0x08, 8g --> 0x10, 16g --> 0x18
    writeRegister(ACCEL_CONFIG, 0x08);
    // Enable accelerometer for FIFO
    writeRegister(FIFO_EN, 0x08);
    // Reset and enable FIFO
    writeRegister(USER_CTRL, 0x44);
}

void read_fifo() {
    // Read FIFO count
    uint16_t fifo_count = readFIFOCount();
    if (fifo_count >= 6) 
    { // Check if FIFO has 6 bytes
        Wire.beginTransmission(MPU6050_ADDRESS);
        Wire.write(FIFO_R_W);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU6050_ADDRESS, 6);
        int16_t accelX = Wire.read() << 8 | Wire.read();
        int16_t accelY = Wire.read() << 8 | Wire.read();
        int16_t accelZ = Wire.read() << 8 | Wire.read();
        // Convert raw values to actual acceleration in g
        float ax = accelX / ACCEL_SCALE_MODIFIER_8G;
        float ay = accelY / ACCEL_SCALE_MODIFIER_8G;
        float az = accelZ / ACCEL_SCALE_MODIFIER_8G;
        // Get the current timestamp
        // unsigned  timestamp = millis();
        double time_sec = millis() / 1000.0;
        // Print data to Serial Monitor with timestamp
        Serial.print("Timestamp: "); Serial.print(time_sec,3);
        Serial.print(" ms | AccelX: "); Serial.print(ax, 3);
        Serial.print(" g | AccelY: "); Serial.print(ay, 3);
        Serial.print(" g | AccelZ: "); Serial.println(az, 3);
    }
}

// Getting the data from the FIFO Buffer
uint16_t readFIFOCount() {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(FIFO_COUNTH);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS, 2);
    uint16_t count = Wire.read() << 8 | Wire.read();
    return count;
}

void writeRegister(uint8_t reg, uint8_t value) 
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}
