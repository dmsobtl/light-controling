const int buttonPin = 12;
const int LEDPin = 13;
int LEDState = HIGH;
int buttonState;
int lastButtonState = LOW;
long lastDebounceTime = 0;
long debounceDelay = 50;
void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(LEDPin, OUTPUT);
  digitalWrite(LEDPin, LEDState);

}

void loop() {
  int reading = digitalRead(buttonPin);
  if(reading != lastButtonState){
    lastDebounceTime = millis();
  }
  if((millis() - lastDebounceTime) > debounceDelay){
    if(reading != buttonState){
      buttonState = reading;
      if(buttonState == HIGH){
        LEDState =! LEDState;
      }
    }
  }
  digitalWrite(LEDPin, LEDState);
  lastButtonState = reading;

}
