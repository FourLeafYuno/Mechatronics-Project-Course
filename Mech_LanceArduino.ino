#define in A3
#define A 7
#define B 6
#define C 5
#define D 4
#define NUMBER_OF_STEPS_PER_REV 512
#define delayMS 2

boolean charge = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(D, OUTPUT);
  pinMode(in, INPUT);
  Serial.begin(115200);
}

void loop() {
  Serial.println(analogRead(in));
  if (digitalRead(in) == HIGH) {
    for (int i = 0; i < NUMBER_OF_STEPS_PER_REV; i++) {
      Serial.print("Rotate");
      onestep();
    }
    write(0, 0, 0, 0);
    delay(100);
  }
}

void write(int a, int b, int c, int d) {
  digitalWrite(A, a);
  digitalWrite(B, b);
  digitalWrite(C, c);
  digitalWrite(D, d);
}

void onestep() {
  write(1, 0, 0, 0);
  delay(delayMS);
  write(1, 1, 0, 0);
  delay(delayMS);
  write(0, 1, 0, 0);
  delay(delayMS);
  write(0, 1, 1, 0);
  delay(delayMS);
  write(0, 0, 1, 0);
  delay(delayMS);
  write(0, 0, 1, 1);
  delay(delayMS);
  write(0, 0, 0, 1);
  delay(delayMS);
  write(1, 0, 0, 1);
  delay(delayMS);
}
