// LSM6 library to read the LSMDS33 (Accelerometer and Gyro)
# include <Wire.h>
# include <LSM6.h>
# include <USBCore.h>

// Motor Pin definitions
#define L_PWM 10
#define L_DIR 16
#define R_PWM 9
#define R_DIR 15

// Define Results Dimension
#define RESULTS_DIM 324

// imu class
LSM6 imu;

// Variables to keep results
float results[RESULTS_DIM];
int currentExperiment = 0;
int resultIndex = 0;

// State machine
#define STATE_RUNNING_EXPERIMENT 0
#define STATE_FINISHED_EXPERIMENT 1
int state;

// Time stamp for data recording
unsigned long update_ts;
unsigned long dataRecord_ts;

// Intiate variables to store the filtered output
float filteredAccelX = 0;
float filteredAccelY = 0;
// Alpha value for the filter
const float alpha = 0.2; // Experiment with this value

void setup() {
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_DIR, OUTPUT);
  pinMode(R_DIR, OUTPUT);

  // Start the wire library for i2c.
  // Note: do not add this command into
  // a class constructor. It must occur
  // (or be called) from setup().
  Wire.begin(); 

  // Serial for debug output
  Serial.begin(9600);
  Serial.println("***RESET***");
  delay(1000);

  // Check the IMU initialised ok.
  if (!imu.init() ) {  // no..? :(
    // Since we failed to communicate with the
    // IMU, we put the robot into an infinite
    // while loop and report the error.
    while (1) {
      Serial.println("Failed to detect and initialize IMU!");
      delay(1000);
    }
  }

  // IMU initialise ok!
  // Set the IMU with default settings.
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL1_XL, 0b01011000); // 208 Hz, +/4 g

  state = STATE_RUNNING_EXPERIMENT;
  update_ts = millis();
  dataRecord_ts = millis();
} // End of setup()

void loop() {
  if (state == STATE_RUNNING_EXPERIMENT){
// Robot active, travel straight
    digitalWrite( L_DIR, LOW );
    digitalWrite( R_DIR, LOW );
    analogWrite( L_PWM, 60 );
    analogWrite( R_PWM, 60 );

// Record data every 100 ms (0.1 second)
   if (millis() - dataRecord_ts >= 100) {
      dataRecord_ts = millis();
      recordDataAcc();
    }

    // Transition to finished state after some condition
   if (millis() - update_ts > 10000) {  // Example: after 5 seconds
     state = STATE_FINISHED_EXPERIMENT;
    }
  } else if (state == STATE_FINISHED_EXPERIMENT) {
    analogWrite( L_PWM, 0 );
    analogWrite( R_PWM, 0 );
    reportResultsOverSerial();
    delay(1000);
  }}


    
void recordDataAcc() {
  imu.read();
  //float accelX_mg = imu.a.x * 0.122;
  float accelY_mg = imu.a.y * 0.122;

  //filteredAccelX = (alpha * accelX_mg) + ((1 - alpha) * filteredAccelX);
  filteredAccelY = (alpha * accelY_mg) + ((1 - alpha) * filteredAccelY);

  // Store data in results array
  if (resultIndex < RESULTS_DIM ) {
    //results[currentExperiment][resultIndex++] = filteredAccelX;
    results[resultIndex++] = filteredAccelY;
    Serial.print("Recording Data at Index: ");
    Serial.print(resultIndex - 1);
    Serial.print(", Value: ");
    Serial.println(filteredAccelY, 3);
  }
}

void reportResultsOverSerial() {
  for (int i = 0; i < resultIndex; i++) {
      Serial.print(results[i], 3);
      if (i< RESULTS_DIM - 1 ){
        Serial.print(", ");}
  }
  Serial.println();
  Serial.println("---End of Results ---\n\n");
}


 // delay(100);
 // End of loop()


    //float accelZ_mg = imu.a.z * 0.122;
    //float gyroX_mdps = imu.g.x * 8.75; // Convert to mdps
    //float gyroY_mdps = imu.g.y * 8.75;
    //float gyroZ_mdps = imu.g.z * 8.75;
    //Serial.print("Accel (mg): ");
    //Serial.print(accelX_mg);
    //Serial.print(" ");
    //Serial.println(accelY_mg);
    //Serial.print(" ");
    //Serial.print(accelZ_mg);
    //Serial.print("\t Gyro (mdps):  ");
    //Serial.print(gyroX_mdps);
    //Serial.print(" ");
    //Serial.print(gyroY_mdps);
    //Serial.print(" ");
    //Serial.println(gyroZ_mdps);
