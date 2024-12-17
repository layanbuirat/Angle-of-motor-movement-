#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050.h>

// Initialize LCD (I2C address, 16 columns, 2 rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Change address if needed

// Motor control pins
const int motorIn1 = 7;  // L298N IN1
const int motorIn2 = 8;  // L298N IN2
const int motorPWM = 9;  // L298N ENA (PWM)

// Encoder pins
const int encoderA = 3;  // Encoder A connected to pin 2 (interrupt pin)
const int encoderB = 2;  // Encoder B connected to pin 3

// Variables for motor control and position tracking
volatile int encoderCount = 0;  // Tracks encoder pulses
int motorSpeed = 200;           // Motor speed (PWM value for full 360° rotation)
String motorDirection = "STOP"; // Stores motor direction

// MPU-6050 setup
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float angleY = 0.0;  // Y-axis angle (pitch)
float initialAngleY = 0.0;  // Offset for zeroing the MPU on startup

// Constants for encoder calculations
const int pulsesPerRevolution = 140;  // Adjust based on your encoder's specification
const int degreesPerPulse = 360 / pulsesPerRevolution;  // Degrees moved per pulse

void setup() {
  // Serial communication for debugging
  Serial.begin(9600);

  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Set motor pins as output
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorPWM, OUTPUT);

  // Set encoder pins as input
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();

  // Attach interrupt to encoder pin
  attachInterrupt(digitalPinToInterrupt(encoderA), updateEncoder, CHANGE);

  // Capture the initial MPU angle for zeroing
  delay(1000);  // Allow MPU to stabilize
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  initialAngleY = atan2(ay, az) * 180 / PI;  // Calculate initial Y-axis angle

  // Initial motor setup
  motorSpeed = 140;  // Set default speed
}

void loop() {
  // Read MPU6050 data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate the Y-axis angle (pitch) and zero it using the initial offset
  angleY = atan2(ay, az) * 180 / PI - initialAngleY;

  // Print MPU data for debugging
  Serial.print("AngleY: ");
  Serial.println(angleY);

  // Display the angle on the LCD
  lcd.setCursor(0, 0);
  lcd.print("MPU Y-Angle: ");
  lcd.print(angleY, 2); // Print with two decimal places for precision
  lcd.print("°   ");

  // Calculate the motor position in degrees
  int position = (encoderCount * degreesPerPulse) % 360;
  if (position < 0) {
    position += 360;  // Ensure position is always positive
  }

  // Display motor position on the LCD
  lcd.setCursor(0, 1);
  lcd.print("Motor Pos: ");
  lcd.print(position);
  lcd.print("°   ");

  // Rotate motor based on MPU Y-axis angle
  rotateMotor(angleY);

  // Delay for LCD update
  delay(20);
}

void rotateMotor(float angle) {
  static float lastAngle = 0.0;

  // If the angle is close to 0°, stop motor but don't reset position immediately
  if (abs(angle) < 5) {  // Adjust this threshold as needed
    setMotorDirection("STOP");
    return;
  }

  // If the angle has changed significantly, rotate the motor
  if (abs(angle - lastAngle) > 1) {
    // Calculate target pulses based on MPU angle
    int targetPulses = abs(angle) * pulsesPerRevolution / 360;

    // Calculate the difference between current and target position
    int pulseDifference = targetPulses - abs(encoderCount);

    // Rotate motor if there is a significant difference
    if (angle > 0) {
      setMotorDirection("CW");
    } else {
      setMotorDirection("CCW");
    }

    // Rotate motor for the calculated pulse difference
    rotateForPulses(abs(pulseDifference));

    // Update the last angle
    lastAngle = angle;
  }
  // Stop the motor once the target position is reached
  setMotorDirection("STOP");
}

// Function to rotate the motor for a specific number of pulses
void rotateForPulses(int targetPosition) {
  int pulsesMoved = 0;
  
  while (pulsesMoved < targetPosition) {  // Allow small movements
    delay(10);  // Small delay to allow encoder to register pulses
    pulsesMoved = abs(encoderCount);  // Update the number of pulses moved
  }
}

// Function to set motor direction
void setMotorDirection(String direction) {
  motorDirection = direction;
  if (direction == "CW") {
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);
    analogWrite(motorPWM, motorSpeed);
  } else if (direction == "CCW") {
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, HIGH);
    analogWrite(motorPWM, motorSpeed);
  } else {
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, LOW);
    analogWrite(motorPWM, 0);
  }
}

// Interrupt service routine for encoder
void updateEncoder() {
  // Check motor direction and update encoder count
  if (digitalRead(motorIn1) == HIGH && digitalRead(motorIn2) == LOW) {
    encoderCount++;  // CW direction
  } else if (digitalRead(motorIn1) == LOW && digitalRead(motorIn2) == HIGH) {
    encoderCount--;  // CCW direction
  }
}

// Function to reset motor position
void resetMotorPosition() {
  encoderCount = 0;  // Reset the encoder count
  setMotorDirection("STOP");  // Stop the motor
  Serial.println("Motor position reset to 0°.");
}
