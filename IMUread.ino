#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

unsigned long previousTime = 0;
const unsigned long interval = 1;  // Update interval in milliseconds

// Read accelerometer and gyroscope data
int16_t rawAx, rawAy, rawAz;
int16_t rawGx, rawGy, rawGz;

// Calibrated coefficients
float Cal_Accel_X;
float Cal_Accel_Y;
float Cal_Accel_Z;

float Cal_Gyro_Roll;
float Cal_Gyro_Pitch;
float Cal_Gyro_Yaw;

// Calibrated form raw data
double CalibratedAx;
double CalibratedAy;
double CalibratedAz;

double CalibratedGx;
double CalibratedGy;
double CalibratedGz;

// Raw Orientation form accelerometer
double Pitch_Accel;
double Roll_Accel;

double Roll_Gyro;
double Pitch_Gyro;

// Kalman
float KalmanAngleRoll = 0;
float KalmanUncertaintyAngleRoll = 4; // std dev = 2
float KalmanAnglePitch = 0;
float KalmanUncertaintyAnglePitch = 4; // std dev = 2
float KalmanOutput[] = {0,0}; // angle prediction, uncertainty of the prediction

void setup() {
  Serial.begin(9600);

  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();

  mpu.initialize();
  mpu.setFullScaleGyroRange(0x01);

  Serial.println("Testing connection to MPU6050...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  Calibrate_factor();
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= interval) {
    previousTime = currentTime;
    IMU_Calibration();

    // Orientation form Accelerometer
    Roll_Accel = atan(CalibratedAy/sqrt(CalibratedAx*CalibratedAx + CalibratedAz*CalibratedAz)) * 180.0/3.142;
    Pitch_Accel =  -atan(CalibratedAx/sqrt(CalibratedAy*CalibratedAy + CalibratedAz*CalibratedAz)) * 180.0/3.142;

    // Orientaation form Gyrometer
    float dt = interval / 1000.0;  // Convert interval to seconds
    Roll_Gyro += CalibratedGx * dt;
    Pitch_Gyro += CalibratedGy * dt;

    // Add complementary filter to combine accelerometer and gyroscope readings
    double alpha = 0.98;  // Complementary filter constant
    double Roll = alpha * Roll_Gyro + (1 - alpha) * Roll_Accel;
    double Pitch = alpha * Pitch_Gyro + (1 - alpha) * Pitch_Accel;

    DiscreteKalmanFilter(KalmanAngleRoll,CalibratedGx,Roll_Accel,KalmanUncertaintyAngleRoll);
    KalmanAngleRoll = KalmanOutput[0];
    KalmanUncertaintyAngleRoll = KalmanOutput[1];

    DiscreteKalmanFilter(KalmanAnglePitch,CalibratedGy,Pitch_Accel,KalmanUncertaintyAnglePitch);
    KalmanAnglePitch = KalmanOutput[0];
    KalmanUncertaintyAnglePitch = KalmanOutput[1];

    // Update Roll_Gyro and Pitch_Gyro with the filtered values
    Roll_Gyro = Roll;
    Pitch_Gyro = Pitch;

    Serial.print(Roll_Accel);
    Serial.print(" ");
    Serial.print(Pitch_Accel);
    Serial.print(" ");
    Serial.print(KalmanAngleRoll);
    Serial.print(" ");
    Serial.print(KalmanAnglePitch);
    Serial.print(" ");
    Serial.print(Roll);
    Serial.print(" ");
    Serial.print(Pitch);
    Serial.println();
  }
}

void IMU_Calibration(){
    mpu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);
    // Map raw accelerometer values to g
    CalibratedAx = (rawAx - Cal_Accel_X) / 16384.0;
    CalibratedAy = (rawAy - Cal_Accel_Y) / 16384.0;
    CalibratedAz = ((rawAz - Cal_Accel_Z) / 16384.0) + 0.981;
    
    // Convert raw gyroscope values
    CalibratedGx = (rawGx - Cal_Gyro_Roll) / 65.5;
    CalibratedGy = (rawGy - Cal_Gyro_Pitch) / 65.5;
    CalibratedGz = (rawGz - Cal_Gyro_Yaw) / 65.5;
}

void Calibrate_factor(){
    for(int i = 0; i < 5000;i++){
      mpu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);
      Cal_Accel_X += rawAx;
      Cal_Accel_Y += rawAy;
      Cal_Accel_Z += rawAz;

      Cal_Gyro_Roll += rawGx;
      Cal_Gyro_Pitch = rawGy;
      Cal_Gyro_Yaw = rawGz;
    }

    Cal_Accel_X /= 5000;
    Cal_Accel_Y /= 5000;
    Cal_Accel_Z /= 5000;

    Cal_Gyro_Roll /= 5000;
    Cal_Gyro_Pitch /= 5000;
    Cal_Gyro_Yaw /= 5000;
}

void DiscreteKalmanFilter(float KalmanState,float KalmanInput,float KalmanMeasurement,float KalmanUncertainty){ // KalmanInput = rotation rate, KalmanMeasurement = accelerometer angle 
  KalmanState = KalmanState + interval/1000.0 * KalmanInput; // Xk = Ax + Bu 
  KalmanUncertainty =  KalmanUncertainty + 1*1; // Pk = APkAT + Q  
  float KalmanGain = KalmanUncertainty/(KalmanUncertainty + 10*10); // Pk = PCT/CPCT + R
  KalmanState = KalmanState + KalmanGain*(KalmanMeasurement - KalmanState); // Xk = Xk + K(y - Cx)
  KalmanUncertainty = (1 - KalmanGain)*KalmanUncertainty; // Pk = (I - KC)Pk

  KalmanOutput[0] = KalmanState;
  KalmanOutput[1] = KalmanUncertainty;
}