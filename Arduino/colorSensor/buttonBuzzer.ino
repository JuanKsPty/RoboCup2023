const int buzzerPin = A1;
const int button = A0;


void setup()
{
  pinMode(buzzerPin, OUTPUT);
  pinMode(button, INPUT);
  Serial.begin(9600);
}

void loop()
{
  int buttonState = digitalRead(button);
  Serial.println(button);
  digitalWrite(buzzerPin, LOW);
  if (buttonState == 1){
    digitalWrite(buzzerPin, HIGH);
  }

  delay(1);
}
