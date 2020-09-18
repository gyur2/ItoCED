#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  digitalWrite(PIN_LED, 0); // turn off LED.
  delay(1000);
}

void loop() {
  digitalWrite(PIN_LED, 1); // update LED status.
  delay(200); // wait for 200 milliseconds
  digitalWrite(PIN_LED, 0); // update LED status.
  delay(200); // wait for 200 milliseconds
  digitalWrite(PIN_LED, 1); // update LED status.
  delay(200); // wait for 200 milliseconds
  digitalWrite(PIN_LED, 0); // update LED status.
  delay(200); // wait for 200 milliseconds 
  digitalWrite(PIN_LED, 1); // update LED status.
  delay(200); // wait for 200 milliseconds
  digitalWrite(PIN_LED, 0); // update LED status.
  delay(200); // wait for 200 milliseconds 
  digitalWrite(PIN_LED, 1); // update LED status.
  delay(200); // wait for 200 milliseconds
  digitalWrite(PIN_LED, 0); // update LED status.
  delay(200); // wait for 200 milliseconds 
  digitalWrite(PIN_LED, 1); // update LED status.
  delay(200); // wait for 200 milliseconds
  digitalWrite(PIN_LED, 0); // update LED status.
  delay(200); // wait for 200 milliseconds 
  
  while(1){
    digitalWrite(PIN_LED, 1); //update LED status.
  }
}

int toggle_state(int toggle) {
  return toggle;
}
