//List of pins from top to bottom
const int ENB = 5;
const int phaB = 4;
const int ENA = 0;
const int phaA = 2;
const int END = 14;
const int phaD = 12;
const int ENC = 13;
const int phaC = 15;

// Direction pins for the 4 motors
// Select pins commented out in the case of using NOT gates

//Encoder pins for each motor
#define GEARING 50
#define ENCODERMULT 14
volatile float RPM = 0;
volatile uint32_t lastA = 0;

ICACHE_RAM_ATTR void interruptA(){
  uint32_t currA = micros();
  if (lastA < currA){
    float rev = currA - lastA;
    rev = 1.0 / rev;
    rev *= 1000000;
    rev *= 60;
    rev /= GEARING;
    rev /= ENCODERMULT;
    RPM = rev;
    //Serial.println(RPM);
  }
  lastA = currA;
}
void setup() {
  //Serial.begin(9600);
  // Setup pin modes motor driver
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ENC, OUTPUT);
  pinMode(END, OUTPUT);
  pinMode(phaA, OUTPUT);
  pinMode(phaB, OUTPUT);
  pinMode(phaC, OUTPUT);
  pinMode(phaD, OUTPUT);

  set_speed(255);
  // Setup pin modes for encoder inputs
  //pinMode(encA, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(encA), interruptA, RISING);
}

void loop() {
  forward_dir();
  delay(1000);
  right_dir();
  delay(1000);
  backward_dir();
  delay(1000);
  left_dir();
  delay(1000);
}

void set_speed(int x){
  analogWrite(ENA, x);
  analogWrite(ENB, x);
  analogWrite(ENC, x);
  analogWrite(END, x);
}

void forward_dir(){
  digitalWrite(phaA, LOW);
  digitalWrite(phaB, LOW);
  digitalWrite(phaC, LOW);
  digitalWrite(phaD, HIGH);
}
void back_dir(){
  digitalWrite(phaA, HIGH);
  digitalWrite(phaB, HIGH);
  digitalWrite(phaC, HIGH);
  digitalWrite(phaD, LOW);
}

void left_dir(){
  digitalWrite(phaA, LOW);
  digitalWrite(phaB, HIGH);
  digitalWrite(phaC, HIGH);
  digitalWrite(phaD, HIGH);
}
void right_dir(){
  digitalWrite(phaA, HIGH);
  digitalWrite(phaB, LOW);
  digitalWrite(phaC, LOW);
  digitalWrite(phaD, LOW);
}