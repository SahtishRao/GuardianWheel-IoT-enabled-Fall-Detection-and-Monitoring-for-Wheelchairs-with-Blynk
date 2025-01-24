#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
    Serial.begin(115200);             // Start the Serial Monitor
    Wire.begin(D2, D1);             // Start the I2C bus on NodeMCU's SDA and SCL pins
    mpu.initialize();               // Initialize the MPU6050 sensor
    if (mpu.testConnection()) {
        Serial.println("MPU6050 Ready!");
    } else {
        Serial.println("MPU6050 connection failed!");
    }
}

void loop() {
    // Variables to hold raw acceleration and gyroscope data
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    // Get acceleration values
    mpu.getAcceleration(&ax, &ay, &az);
    
    // Get gyroscope values
    mpu.getRotation(&gx, &gy, &gz);

    // Convert raw acceleration data to 'g' units
    float accelX = ax / 16384.0;
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;

    // Convert raw gyroscope data to degrees/second
    float gyroX = gx / 131.0;
    float gyroY = gy / 131.0;
    float gyroZ = gz / 131.0;

    // Print acceleration data
    Serial.print("Acc X: "); Serial.println(accelX);
    Serial.print("Acc Y: "); Serial.println(accelY);
    Serial.print("Acc Z: "); Serial.println(accelZ);

    // Print gyroscope data
    Serial.print("Gyro X: "); Serial.println(gyroX);
    Serial.print("Gyro Y: "); Serial.println(gyroY);
    Serial.print("Gyro Z: "); Serial.println(gyroZ);

    delay(1000);     // Wait for half a second before reading again
}
