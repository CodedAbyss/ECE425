void setup() {
  int baudrate = 9600;  //serial monitor baud rate'
  Serial.begin(baudrate);
}

void loop() {
  int right_photoresistor = analogRead(A1);
  int left_photoresistor = analogRead(A0);

  Serial.print("Left Photoresistor: ");
  Serial.print(left_photoresistor);
  Serial.print("   ");
  Serial.print("Right Photoresistor: ");
  Serial.println(right_photoresistor);
  

  delay(1000);
}