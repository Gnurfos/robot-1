

const int controlPin1 = 3; // connected to pin 7 on the H-bridge
const int controlPin2 = 4; // connected to pin 2 on the H-bridge
const int controlPin3 = 5; // connected to pin 10 on the H-bridge
const int controlPin4 = 6; // connected to pin 15 on the H-bridge

const int enablePin = 10;

const int nbPins = 4;
int pins[nbPins];
int activePinIdx = 0;

int side = 0; // 0 none  1 left 2 right 3 both
int direction = 0; // 0 1

void setup(){
  pins[0] = controlPin1;
  pins[1] = controlPin2;
  pins[2] = controlPin3;
  pins[3] = controlPin4;
  pinMode(controlPin1, OUTPUT);
  pinMode(controlPin2, OUTPUT);
  pinMode(controlPin3, OUTPUT);
  pinMode(controlPin4, OUTPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(controlPin1, LOW);
  digitalWrite(controlPin2, LOW);
  digitalWrite(controlPin3, LOW);
  digitalWrite(controlPin4, LOW);
  analogWrite(enablePin, 0);
}

void loop(){
  analogWrite(enablePin, 255);
  for (int i = 0; i < nbPins; i++) {
    if (i == activePinIdx) {
      digitalWrite(pins[i], HIGH);
    } else {
      digitalWrite(pins[i], LOW);      
    }
  }
  
  activePinIdx = (activePinIdx + 1) % nbPins;
  
  delay(2000);
  
}

