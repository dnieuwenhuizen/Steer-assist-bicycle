/*
    Main program steer assist made by Dick Nieuwenhuizen
    Running the system with the steer into the fall controller

    Feel free to use this code in any way you want, no copyright involved.
    Redled is for SD card
    Yellowled is for IMU

    SD_button is low when pressed. When pressed write to SD card.
    Board is mounted on the baggage carrier with the IMU in the middle.
    Writing to SD card lowers the sampling frequency from 1450 to 1300 Hz. Do not write to the
    SD card too often.

    MPU9250: orientation of the magnetometer is different with respect to the gyro and accelerometer.
    hx equals ay and gy, hy equals ax and gx, hz equals -az and -gz

    In the current setup -gx equals to our lean rate and -roll equals to lean angle.

    Check if motor zero is found by the flashing green light on the motor drive. The led starts red and starts
    flashing a green light when the zero is found. 
*/

#include <Encoder.h>
#include <MPU9250.h>
#include <MadgwickAHRS.h>
// #include <SD.h>
#include <SdFat.h>
#include <Filters.h>
#include <math.h>

void velocity_estimation(float* velocity);
void get_temp(float* temp);
void setTorque(float t);
float limit(float val, float min, float max);
void zPulse();
void enableServo();
void disableServo();
void dynamoPulse();

// Initialize Benchmark variables;
float K[4] = {0, 0, 0, 0}; // Feedback gain array K
float Ks = 5;
float Cs = 0.075;
const int controller_button = 10;
int controller_state = 0;

// SD card variables
SdFatSdio SD;
// const int chipSelect = BUILTIN_SDCARD;
File dataFile;
const int SD_button = 15;
int statesd_button_new = 0;
int statesd_button_old = 0;

// i2c parameters
const uint32_t bus = 0;    // Bus is zero when using pins 18 and 19
const uint32_t i2cRate = 400000; // 400kHz

// an MPU9250 object with its I2C address
// of 0x68 (ADDR to GRND) and on Teensy bus 0
// using pins 16 and 17 instead of 18 and 19.
MPU9250 IMU(0x68, bus, I2C_PINS_18_19);

// Madgwick filter
Madgwick Madgwick_filter;
// Timing for Madgwick filter
float filter_now;
float deltat;
float lastUpdate_filter;

// Specify variables
float ax, ay, az, gx, gy, gz, hx, hy, hz;
float lean_angle, lean_rate;
float pitch, yaw, roll;

// Magnetometer calibration values, A_inv soft iron scaled matrix and magbias hard iron effects
float A_inv[9] = {1.081523, -0.033576, 0.045224, -0.033576, 1.022031, 0.018054, -0.045224,  0.018054, 1.003392};
float magbias[3] = {1485.934993, 2832.129947, -633.979257};
float beta, zeta;
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
float yaw_declination = 0.9504; // Declination in degrees
int32_t beginStatus;
float printTimelast;
float printTimenew;

// Leds
const int teensyLed = 13;
const int whiteLed = 24;
const int yellowLed = 25;
const int redLed = 26;

// Dynamo input
const int dynamoPin = 2;

// Motor pins
Encoder myEnc(31, 32);
int zPulsePin = 30;
const int enablePin = 17;
const int servoPwmPin = 14;

// Zero pulse motor
bool foundZero = false;
long zeroPosition = 0;
void zPulse() {
  foundZero = true;
  zeroPosition = myEnc.read();
  detachInterrupt(digitalPinToInterrupt(zPulsePin));
}

// Velocity estimation from dynamo hub, works fine after check with with reed contact
uint8_t dynamoCount = 0;
unsigned long dynamoTimeLast = 0;
unsigned long *dynamo_ptr;
float velocity = 0;
void dynamoPulse() {
  dynamoCount++;
}

// Enable servo
bool isServoEnabled = false;
void enableServo() {
  digitalWrite(enablePin, LOW);
  isServoEnabled = true;
}

// Disable Servo
void disableServo() {
  digitalWrite(enablePin, HIGH);
  isServoEnabled = false;
}

// Limit function
float limit(float val, float min, float max) {
  if (val > max) {
    return max;
  }

  if (val < min) {
    return min;
  }

  return val;
}

void setTorque(float t) {
  // One wire 50% duty cycle PWM setting
  // torque constant is different than the datasheet, stating the torque constant for
  // stall torque. Using this torque constant we obtain a stall torque of 2.6 Nm at 10 Amp
  const float torquePerAmp = 0.26; // [Nm/Arms]
  const float fullOutputAmp = 10.0; // [A]
  const int maxPwmValue = 32767;
  const int pwmRange = maxPwmValue / 2;

  float amps = t / torquePerAmp;
  // Can't send 0 to the motor drive, so limit with 0.99
  float com = limit(amps / fullOutputAmp, -0.99, 0.99);
  analogWrite(servoPwmPin, pwmRange * (1 + com));
}

// Pins for temperature measurement using the PT100
const int Vcc_PT100 = 37;
const int PT100_Pin = 35;

// Estimate temperature based on theory
void get_temp(float* temp) {
  // Store variables inside a function to make them local
  // PT100 temperature estimation variables
  const float R0 = 100;
  const float R1 = 5104; // Ohm
  const float Vin = 3.305; // volt
  float Rt;
  float voltage_PT100;
  const float A_var = 3.9083 * pow(10, -3);
  const float B_var = -5.7705 * pow(10, -7);

  voltage_PT100 = analogRead(PT100_Pin) / ((float)1024 / 3.3); // 10 bit ADC
  voltage_PT100 /= 26.5207;
  Rt = R1 * (1 / (Vin / voltage_PT100 - 1));
  *temp = (-A_var * R0 + sqrt(pow(A_var * R0, 2) - 4 * R0 * B_var * (R0 - Rt))) / (2 * R0 * B_var);
}


//------------------------------------ Setup -------------------------------------//
void setup() {
  dynamo_ptr = &dynamoTimeLast;
  // Initialize motor settings
  pinMode(enablePin, OUTPUT);
  // analogWriteResolution 15bits
  // max is 32767
  // 0 torque is 16383
  analogWriteFrequency(servoPwmPin, 1800); // 1.8kHz PWM freq, must be between 1 and 100 kHz!
  analogWriteResolution(15);

  // Set the leds as output
  pinMode(teensyLed, OUTPUT);
  pinMode(whiteLed, OUTPUT);
  pinMode(yellowLed, OUTPUT);
  pinMode(redLed, OUTPUT);

  // Blink on start-up
  for (int i = 0; i < 3; i++) {
    digitalWrite(whiteLed, LOW);
    digitalWrite(yellowLed, LOW); // HIGH is led OFF
    digitalWrite(redLed, LOW); // HIGH is led OFF
    delay(250);
    digitalWrite(whiteLed, HIGH);
    digitalWrite(yellowLed, HIGH); // HIGH is led OFF
    digitalWrite(redLed, HIGH); // HIGH is led OFF
    delay(250);
  }

  // Initialize temperature sensor
  pinMode(Vcc_PT100, OUTPUT);
  delay(10);
  digitalWrite(Vcc_PT100, HIGH);

  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  // while (!Serial);
  delay(100);

  // Initializing SD card and corresponding button
  Serial.print("\nInitializing SD card...");
  if (SD.begin()) {         // (SD.begin(chipSelect)) {
    Serial.println("initialization done.");
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    dataFile.println("----------------------");
  } else {
    Serial.println("initialization failed!");
    digitalWrite(redLed, LOW);
  }
  pinMode(SD_button, INPUT_PULLUP);

  // Starting the I2C bus
  i2c_t3(bus).begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, i2cRate);
  delay(10);

  // Begin MPU9250
  // initMPU9250();
  digitalWrite(yellowLed, LOW);
  IMU.begin();
  IMU.configuration(ACCEL_RANGE_2G, GYRO_RANGE_250DPS);
  IMU.setFilt(DLPF_BANDWIDTH_5HZ, (uint8_t) 1); // Set internal lowpass filter at 184Hz and data ouput rate of 500 Hz
  // Sample rate = 1000/(1 + SRD)
  digitalWrite(yellowLed, HIGH);

  // Iniatialize global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
  float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
  // float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
  beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
  Serial.print("beta is: "); Serial.println(beta, 4);

  // Attach interrupts to the zero pulse inddex of the motor and to the dynamo
  attachInterrupt(digitalPinToInterrupt(zPulsePin), zPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(dynamoPin), dynamoPulse, RISING);

  pinMode(controller_button, INPUT_PULLUP);
  SD.remove("pp10_none.txt");
  SD.remove("pp10_Ks.txt");
}

unsigned long lastPrintTime = 0;
unsigned long writeStart = 0;
unsigned long writeTime = 0;
int loopCnt = 0;
int loopCntSD = 0;
float steer_angle;
float steer_rate = 0;
float newTimeSteer;
float oldTimeSteer;
float positionCounts = 0;
float oldPosition = 0;
float newPosition = 0;
float steerRateArray[10];
float torque = 0;
float loopTime = 0;
float sampling_frequency = 0;
float temp;
unsigned long timingMotor;
unsigned int timeOutMotor;
unsigned long button_debounce;
uint8_t first_none = 0; uint8_t first_Ks = 0; uint8_t first_LQR = 0;

unsigned long dynamoPrint;

#define buffsize 50000
char str[buffsize];

// -------------------------------------------- Loop ------------------------------------------- //
void loop() {
  dynamoPrint = 0;

  // Loop time of the system
  loopTime = micros();

  // Obtain velocity
  velocity_estimation(&velocity);

  if (digitalRead(controller_button) == LOW && millis() > button_debounce) {
    controller_state++;
    if (controller_state > 1) {
      controller_state = 0;
    }
    button_debounce = millis() + 1000;
  }

  switch (controller_state) {
    case 0:
      {
        // Obtain feedback gain for the controller
        K[0] = 0; K[1] = 0; K[2] = 0; K[3] = 0;
        if (first_none == LOW) {
          digitalWrite(whiteLed,HIGH);
          char sd_file[] = "pp10_none.txt";
          statesd_button_new = 0;
          dataFile = SD.open(sd_file, FILE_WRITE);
          dataFile.println("\n No controller \n");
          dataFile.close();
          first_none = HIGH;
          first_Ks = LOW;
        }
        break;
      }
    case 1:
      {
        // Obtain feedback gain for the controller
        K[0] = Ks * (5 - velocity); K[1] = Cs * (5 - velocity); K[2] = 0; K[3] = 0;
        if (velocity > 5 || velocity < 1) {
          K[0] = 0;
          K[1] = 0;
        }
        digitalWrite(whiteLed, LOW);
        if (first_Ks == LOW) {
          char sd_file[] = "pp10_Ks.txt";
          statesd_button_new = 0;
          dataFile = SD.open(sd_file, FILE_WRITE);
          dataFile.println("\n Ks controller \n");
          dataFile.close();
          first_Ks = HIGH;
          first_none = LOW;
        }
        break;
      }
  }

  // Obtain data of accelerometer, gyro and magnetometer at once
  IMU.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz);
  hx *= 10; hy *= 10; hz *= 10; // In nanoTesla

  // Calibrate magnetometer values, first remove offset from hard iron effects
  hx -= magbias[0]; hy -= magbias[1]; hz -= magbias[2];
  // Then scale for soft iron effects
  hx = hx * A_inv[0] + hy * A_inv[1] + hz * A_inv[2];
  hy = hx * A_inv[3] + hy * A_inv[4] + hz * A_inv[5];
  hz = hx * A_inv[6] + hy * A_inv[7] + hz * A_inv[8];

  // Madgwick filter implementation
  filter_now = micros();
  deltat = ((filter_now - lastUpdate_filter) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate_filter = filter_now;
  // Serial.print("Deltat is: "); Serial.println(deltat,6);
  Madgwick_filter.update(gx, gy, gz, ax, ay, az, hy, hx, -hz, deltat, beta);
  // beta = 0.4; Madgwick_filter.updateIMU(gx, gy, gz, ax, ay, az, deltat, beta);
  roll = Madgwick_filter.getRoll(); pitch = Madgwick_filter.getPitch(); yaw = Madgwick_filter.getYaw();
  yaw -= yaw_declination;
  roll *= PI / 180; pitch *= PI / 180; yaw *= PI / 180; // turn into radians
  lean_angle = -roll;
  lean_rate = -gx; // Do not filter for your control algorithm filterLowpassGyro.input(-gx); // gy instead of gx

  // 185,000 counts per revolution for the quadrature encoder. Encoder has been switched, so datasheet is wrong.
  // New position estimation and steer rate estimation
  newTimeSteer = micros();
  positionCounts = myEnc.read() - zeroPosition;
  steer_angle = positionCounts / 183500 * (2 * PI); // radians
  // Compute steer rate using moving average
  steerRateArray[loopCnt] = (steer_angle - oldPosition) / (newTimeSteer - oldTimeSteer) * (float) 1000000;
  oldPosition = steer_angle;
  oldTimeSteer = newTimeSteer;

  // Take the average of 10 samples to obtain the steer rate
  if (loopCnt % 10 == 0) {
    steer_rate = 0;
    for (int i = 0; i < loopCnt; i++) {
      steer_rate += steerRateArray[i];
    }
    steer_rate /= loopCnt;
    loopCnt = 0;
  }
  loopCnt++;

  get_temp(&temp);

  // Servo control loop
  if (foundZero) {
    if (!isServoEnabled) {
      enableServo();
    }
    if (millis() > timingMotor) {
      // Running at 100 hz
      timingMotor = millis() + 10;
      // When feedback gain on steer_rate is too high, causes oscillations 29-6
      torque = (K[0] * lean_rate + K[1] * steer_rate + K[2] * lean_angle + K[3] * steer_angle);
      torque = limit(torque, -2.6, 2.6);
      if ((temp < (float) 50.0) && (millis() > timeOutMotor)) { // Start with a low temperature to see if it works
        setTorque(torque);
        digitalWrite(redLed, HIGH);
      }
      else if (millis() < timeOutMotor) {
        torque /= 2;
        setTorque(torque);
        digitalWrite(redLed, LOW);
      }
      else {
        torque /= 2;
        setTorque(torque);
        timeOutMotor = millis() + 30000;
        digitalWrite(redLed, LOW);
      }
    }
  }

  // If loop for the SD button to know in which state you are.
  if ((digitalRead(SD_button) == LOW) && (statesd_button_new == statesd_button_old)) {
    statesd_button_new++;
  }
  if ((digitalRead(SD_button) == HIGH) && (statesd_button_new != statesd_button_old)) {
    statesd_button_old = statesd_button_new;
  }

  // Computing in string is slow and costs a lot of RAM. If you are using a slower
  // smaller microcontroller consider switching to char arrays
  String dataString = "";
  dataString += String(millis());
  dataString += " " + String(velocity, 2) + " " + String(torque, 2) + " " + String(lean_rate, 4);
  dataString += " " + String(steer_rate, 4) + " " + String(lean_angle, 4) + " " + String(steer_angle, 4);
  dataString += " " + String(yaw, 4) + " " + String(-ax, 4) + " " + String(temp, 1) + " " + String(statesd_button_new);

  // Writing data to Serial connection if available
  if (millis() > lastPrintTime) {
    Serial.print(dataString);
    // printDataEuler();
    Serial.print(" "); Serial.print(sampling_frequency);
    Serial.print("    "); printDataAccGyro();
    Serial.println(" ");

    lastPrintTime = millis() + 500;
  }

  // Writing data to the SD card
  if (digitalRead(SD_button) == LOW) {
    if (millis() > writeStart) {
      writeStart = millis() + 40;   // Sampling at 25 Hz
      switch (controller_state) {
        case 0:
          {
            char sd_file[] = "pp10_none.txt";
            dataFile = SD.open(sd_file, FILE_WRITE);
            break;
          }
        case 1:
          {
            char sd_file[] = "pp10_Ks.txt";
            dataFile = SD.open(sd_file, FILE_WRITE);
            break;
          }
      }
      // dataFile = SD.open(sd_file, FILE_WRITE);
      if (dataFile) {
        dataFile.println(dataString);
      }
      dataFile.close(); // dataFile.close() does the same as flush()
      loopCntSD++;
    }
  }

  // Calculate sampling frequency of the loop
  sampling_frequency = 1 / ((micros() - loopTime) / 1000000);
}

// Function for estimating the velocity based on the pulses coming from the dynamo hub  
void velocity_estimation(float* velocity) {
  if (dynamoCount > 1) { // 13 is full round
    // dynamoTimeNow = micros();
    // Used a pointer here since I was debugging
    unsigned long dynamoTimeDifference = micros() - *dynamo_ptr;
    // Serial.print(micros()); Serial.print(" "); Serial.print(dynamoTimeLast);
    // Serial.print(" "); Serial.print((long)dynamo_ptr, HEX); Serial.print(" "); Serial.println(dynamoTimeDifference);

    // velocity = 0.4488/((float)dynamoTimeDifference/1000000)*0.38; // Distance travelled for one pulse divided over time
    *velocity = 2 * PI / ((float)dynamoTimeDifference / 1000000) * 0.378 / 7;
    *dynamo_ptr = micros();
    // Serial.println(dynamoTimeLast);
    dynamoCount = 0;
  }
  else {
    // dynamoTimeNow = micros();
    if (micros() > (dynamoTimeLast + 1000000)) { // No movement for sometime then velocity is zero
      *velocity = 0;
    }
  }
}

// Some functions which make the main text a lot smaller
void printDataEuler() {
  Serial.print("Orientation: ");
  Serial.print(yaw * (180 / PI), 5);
  Serial.print("   ");
  Serial.print(pitch * (180 / PI), 5);
  Serial.print("   ");
  Serial.println(roll * (180 / PI), 5);
}

void printDataAccGyro() {

  Serial.print(ax, 6);
  Serial.print("   ");
  Serial.print(ay, 6);
  Serial.print("   ");
  Serial.print(az, 6);
  Serial.print("   ");

  Serial.print(gx, 6);
  Serial.print("   ");
  Serial.print(gy, 6);
  Serial.print("   ");
  Serial.print(gz, 6);
  Serial.print("   ");
}