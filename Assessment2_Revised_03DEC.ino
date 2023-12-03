// LSM6 library to read the LSMDS33 (Accelerometer and Gyro)
# include <Wire.h>
# include <LSM6.h>
# include <USBCore.h>
u8 USB_SendSpace(u8 ep);
# define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

// Motor Pin definitions
#define L_PWM 10
#define L_DIR 16
#define R_PWM 9
#define R_DIR 15
#define LEFT_BUMP 4
#define RIGHT_BUMP 5
#define EMIT_PIN 11

// Define Results Dimension
#define RESULTS_DIM 350

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
float filteredAccelZ = 0;
int flag = 0;

// Alpha value for the filter
const float alpha = 0.2; 

int Bump_pins[2] = {LEFT_BUMP, RIGHT_BUMP};

float readBumpSensor(int number, int Bump_pins[2]){
    // ensure sensor number is within the range, else return 0
    if(number < 0) {
        return 0;
    }
    if(number > 1) {
        return 0;
    }
    pinMode(EMIT_PIN, OUTPUT);
    digitalWrite(EMIT_PIN, LOW);
    pinMode(Bump_pins[number], OUTPUT);
    digitalWrite(Bump_pins[number], HIGH);
    delayMicroseconds(10);
    pinMode(Bump_pins[number], INPUT);
    
    unsigned long start_time = micros();
    while(digitalRead(Bump_pins[number]) == HIGH) {
        // Do nothing here (waiting).
    }
    unsigned long end_time = micros();
    pinMode(Bump_pins[number], INPUT);

    unsigned long elapsed_time = end_time - start_time;
    return (float)elapsed_time;}


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
  delay(1000);
  if( SERIAL_ACTIVE )Serial.println("***RESET***");


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
static unsigned long lastTime = 0; 
unsigned long currentTime = millis();

    if (currentTime - lastTime >= 10) { // Check if 10 milliseconds have passed
        lastTime = currentTime; // Update the last time

        float LeftBumper = readBumpSensor(0, Bump_pins);
        float RightBumper = readBumpSensor(1, Bump_pins);

        if (state == STATE_RUNNING_EXPERIMENT) {
            if (LeftBumper > 1000 || RightBumper > 1000 || flag == 1){
            //if (LeftBumper < 35000 || RightBumper < 35000 || flag == 1){
                analogWrite(L_PWM, 0);
                analogWrite(R_PWM, 0);
                flag = 1;
            } else {
                digitalWrite(L_DIR, LOW);
                digitalWrite(R_DIR, LOW);
                analogWrite(L_PWM, 100);
                analogWrite(R_PWM, 100);
            }

            recordDataAcc();

            // Increment the data point counter and check if we've reached 100 data points
            if (++resultIndex / 3 >= 110) { // Increment resultIndex, divide by 3 because we store X, Y, Z
                state = STATE_FINISHED_EXPERIMENT;
            }
        } 
        else if (state == STATE_FINISHED_EXPERIMENT) {
            analogWrite(L_PWM, 0);
            analogWrite(R_PWM, 0);
            reportResultsOverSerial();
            delay(1000); 
        }
    }
}


/*if (state == STATE_RUNNING_EXPERIMENT){
    if (LeftBumper > 2000 || RightBumper > 2000 || flag == 1){
    //if(LeftBumper < 35000 || RightBumper < 35000 || flag == 1){
      analogWrite ( L_PWM , 0);
      analogWrite (R_PWM, 0);
      flag = 1;} 

    else {
// Robot active, travel straight
    digitalWrite( L_DIR, LOW );
    digitalWrite( R_DIR, LOW );
    analogWrite( L_PWM, 100 );
    analogWrite( R_PWM, 100 );}

// Record data every 100 ms (0.1 second) >= 100, 50ms (0.05 second) >= 50
   if (millis() - dataRecord_ts >= 5) {
      dataRecord_ts = millis();
      recordDataAcc();
    }

    // Transition to finished state after some condition
   if (millis() - update_ts > 1000) {  // after 1 second
     state = STATE_FINISHED_EXPERIMENT;
     float mille = millis();
     Serial.println(mille);
    }} 
  else if (state == STATE_FINISHED_EXPERIMENT) {
    analogWrite( L_PWM, 0 );
    analogWrite( R_PWM, 0 );
    reportResultsOverSerial();
    delay(1000);
  }*/


    
void recordDataAcc() {
  imu.read();
  float accelX_mg = imu.a.x * 0.122; // Convert to mg
  float accelY_mg = imu.a.y * 0.122;
  float accelZ_mg = imu.a.z * 0.122;
  
  filteredAccelX = (alpha * accelX_mg) + ((1 - alpha) * filteredAccelX);
  filteredAccelY = (alpha * accelY_mg) + ((1 - alpha) * filteredAccelY);
  filteredAccelZ = (alpha * accelZ_mg) + ((1 - alpha) * filteredAccelZ);
  

  // Store data in results array
  if (resultIndex < RESULTS_DIM - 3 ) {
    results[resultIndex] = filteredAccelX;
    resultIndex++;
    Serial.println(filteredAccelX);
    results[resultIndex] = filteredAccelY;
    resultIndex++;
    Serial.println(filteredAccelY);
    results[resultIndex] = filteredAccelZ;
    resultIndex++;
    Serial.println(filteredAccelZ);
  }
}

void reportResultsOverSerial() {
  for (int i = 0; i < resultIndex; i += 3) { // Increment by 3 to handle x, y, z together
    // Check to avoid accessing out of bounds
    if (i + 2 < resultIndex && SERIAL_ACTIVE) {
      if( SERIAL_ACTIVE ) Serial.print("(");
      delay(1);
      if( SERIAL_ACTIVE ) Serial.print(results[i], 3); // x
      delay(1);
      if( SERIAL_ACTIVE ) Serial.print(", ");
      delay(1);
      if( SERIAL_ACTIVE ) Serial.print(results[i + 1], 3); // y
      delay(1);
      if( SERIAL_ACTIVE ) Serial.print(", ");
      delay(1);
      if( SERIAL_ACTIVE ) Serial.print(results[i + 2], 3); // z
      delay(1);
      if( SERIAL_ACTIVE ) Serial.print(")");
      delay(1);

      if (i + 3 < resultIndex) {
        if( SERIAL_ACTIVE ) Serial.print(", ");
        delay(1);

      }
    }
  }
  if( SERIAL_ACTIVE ) Serial.println();
     delay(1);
  if( SERIAL_ACTIVE ) Serial.println("---End of Results ---\n\n");
     delay(1);

}
