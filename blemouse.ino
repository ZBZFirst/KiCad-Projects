#include <BleMouse.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// BLE Mouse
BleMouse bleMouse("ESP32 IMU Mouse", "ESP32", 100);

// Touch sensor pins for mouse buttons
const uint8_t touchPins[] = {12, 14, 27, 33, 32};
const int thresholds[] = {90, 90, 90, 90, 90};
bool buttonStates[] = {false, false, false, false, false};

// Mode toggle pin
const int modeTogglePin = 26; // Using digital pin 26 for mode toggle
bool lastToggleState = false;

// MPU6050
Adafruit_MPU6050 mpu;
#define BUFFER_SIZE 5  // Smaller buffer for more responsive mouse

// Arrays to store last N readings for averaging
float accelXBuffer[BUFFER_SIZE] = {0};
float accelYBuffer[BUFFER_SIZE] = {0};
float accelZBuffer[BUFFER_SIZE] = {0};
float gyroXBuffer[BUFFER_SIZE] = {0};
float gyroYBuffer[BUFFER_SIZE] = {0};
float gyroZBuffer[BUFFER_SIZE] = {0};
int currentIndex = 0;

// Mouse control variables
bool relativeMode = true; // Start with relative mode (traditional mouse)
int mouseSensitivity = 1; // Adjust this to change mouse speed
unsigned long lastMouseUpdate = 0;
const long mouseInterval = 15; // Update mouse every 15ms for responsiveness

void calculateAverages(float &avgAccelX, float &avgAccelY, float &avgAccelZ, 
                      float &avgGyroX, float &avgGyroY, float &avgGyroZ) {
    float sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;
    float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;

    for (int i = 0; i < BUFFER_SIZE; i++) {
        sumAccelX += accelXBuffer[i];
        sumAccelY += accelYBuffer[i];
        sumAccelZ += accelZBuffer[i];
        sumGyroX += gyroXBuffer[i];
        sumGyroY += gyroYBuffer[i];
        sumGyroZ += gyroZBuffer[i];
    }

    avgAccelX = sumAccelX / BUFFER_SIZE;
    avgAccelY = sumAccelY / BUFFER_SIZE;
    avgAccelZ = sumAccelZ / BUFFER_SIZE;
    avgGyroX = sumGyroX / BUFFER_SIZE;
    avgGyroY = sumGyroY / BUFFER_SIZE;
    avgGyroZ = sumGyroZ / BUFFER_SIZE;
}

void updateMPUBuffer() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    accelXBuffer[currentIndex] = a.acceleration.x;
    accelYBuffer[currentIndex] = a.acceleration.y;
    accelZBuffer[currentIndex] = a.acceleration.z;
    gyroXBuffer[currentIndex] = g.gyro.x;
    gyroYBuffer[currentIndex] = g.gyro.y;
    gyroZBuffer[currentIndex] = g.gyro.z;

    currentIndex = (currentIndex + 1) % BUFFER_SIZE;
}

void handleMouseButtons() {
    for (int i = 0; i < 5; i++) {
        int touchValue = touchRead(touchPins[i]);
        bool isTouched = touchValue < thresholds[i];

        if (isTouched && !buttonStates[i]) {
            buttonStates[i] = true;
            Serial.print("Button ");
            Serial.print(i);
            Serial.println(" PRESSED");
            
            if (i == 0) {
                bleMouse.press(MOUSE_LEFT);
            } else if (i == 1) {
                bleMouse.press(MOUSE_RIGHT);
            } else if (i == 2) {
                bleMouse.press(MOUSE_MIDDLE);
            } else if (i == 3) {
                bleMouse.press(MOUSE_BACK);
            } else if (i == 4) {
                bleMouse.press(MOUSE_FORWARD);
            }
        } else if (!isTouched && buttonStates[i]) {
            buttonStates[i] = false;
            Serial.print("Button ");
            Serial.print(i);
            Serial.println(" RELEASED");
            
            if (i == 0) {
                bleMouse.release(MOUSE_LEFT);
            } else if (i == 1) {
                bleMouse.release(MOUSE_RIGHT);
            } else if (i == 2) {
                bleMouse.release(MOUSE_MIDDLE);
            } else if (i == 3) {
                bleMouse.release(MOUSE_BACK);
            } else if (i == 4) {
                bleMouse.release(MOUSE_FORWARD);
            }
        }
    }
}

void handleMouseMovement() {
    if (millis() - lastMouseUpdate < mouseInterval) return;
    lastMouseUpdate = millis();
    
    float avgAccelX, avgAccelY, avgAccelZ, avgGyroX, avgGyroY, avgGyroZ;
    calculateAverages(avgAccelX, avgAccelY, avgAccelZ, avgGyroX, avgGyroY, avgGyroZ);
    
    if (relativeMode) {
        // Relative mode: Use gyro for mouse movement (like a traditional mouse)
        int deltaX = (int)(avgGyroY * 150 * mouseSensitivity);
        int deltaY = (int)(avgGyroX * 150 * mouseSensitivity);
        
        // Apply dead zone and constraints
        if (abs(deltaX) > 3 || abs(deltaY) > 3) {
            // Limit maximum movement to prevent jumping
            deltaX = constrain(deltaX, -30, 30);
            deltaY = constrain(deltaY, -30, 30);
            bleMouse.move(deltaX, deltaY);
        }
    } else {
        // Absolute mode: Use accelerometer for pointing
        // Map tilt to cursor movement
        int absX = (int)(avgAccelY * 20 * mouseSensitivity);
        int absY = (int)(avgAccelX * 20 * mouseSensitivity);
        
        // Constrain movement
        absX = constrain(absX, -30, 30);
        absY = constrain(absY, -30, 30);
        
        bleMouse.move(absX, absY);
    }
}

void handleModeToggle() {
    bool currentToggleState = digitalRead(modeTogglePin) == LOW; // Assuming pull-up, so LOW means pressed
    
    if (currentToggleState && !lastToggleState) {
        relativeMode = !relativeMode;
        Serial.print("Switched to ");
        Serial.println(relativeMode ? "RELATIVE mode" : "ABSOLUTE mode");
        delay(300); // Debounce
    }
    lastToggleState = currentToggleState;
}

void setup() {
    Serial.begin(115200);
    
    // Initialize BLE Mouse
    bleMouse.begin();
    
    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) delay(10);
    }
    
    // Set up MPU6050 for mouse control
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 initialized!");

    // Setup mode toggle pin
    pinMode(modeTogglePin, INPUT_PULLUP);

    // Initialize MPU buffer
    for (int i = 0; i < BUFFER_SIZE; i++) {
        updateMPUBuffer();
        delay(10);
    }
    
    Serial.println("ESP32 IMU Mouse Ready!");
    Serial.println("Touch Pins: 12=Left, 14=Right, 27=Middle, 33=Back, 32=Forward");
    Serial.println("Digital Pin 26: Toggle Relative/Absolute Mode");
    Serial.println("Waiting for BLE connection...");
}

void loop() {
    if (bleMouse.isConnected()) {
        updateMPUBuffer(); // Keep updating IMU data
        handleMouseButtons();
        handleModeToggle();
        handleMouseMovement();
    } else {
        // Reset button states when disconnected
        for (int i = 0; i < 5; i++) {
            buttonStates[i] = false;
        }
        Serial.println("Waiting for BLE connection...");
        delay(1000);
    }
    
    delay(10);
}
