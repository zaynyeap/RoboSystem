#define LEFT_BUMP 4
#define RIGHT_BUMP 5
#define EMIT_PIN 11
#define L_PWM_PIN 10         //Left Motor PWM
#define L_DIR_PIN 16         //Left Motor Direction
#define R_PWM_PIN 9          //Right Motor PWM
#define R_DIR_PIN 15         //Right Motor Direction

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
  Serial.begin(9600);
  Serial.println("***RESET***");
}

void loop() {
  float LeftBumper = readBumpSensor(0,Bump_pins);
  float RightBumper = readBumpSensor(1,Bump_pins);
  //Serial.println(LeftBumper);
  Serial.println(RightBumper);

 /* if (LeftBumper > 2000 || RightBumper > 2000){
    digitalWrite (L_DIR_PIN, LOW);
    digitalWrite (R_DIR_PIN, LOW);
    analogWrite(L_PWM_PIN,0);
    analogWrite(R_PWM_PIN,0);
    delay(10000000);}

  else{
    digitalWrite (L_DIR_PIN, LOW);
    digitalWrite (R_DIR_PIN, LOW);
    analogWrite(L_PWM_PIN,100);
    analogWrite(R_PWM_PIN,100);}*/}
