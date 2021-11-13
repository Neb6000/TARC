
float x;
void setup() {
 Serial.begin(115200);
 Serial.setTimeout(1);
}
void loop() {
 //while (!Serial.available());
 if(Serial.available()){
    x = Serial.readString().toFloat();
    Serial.print(x + 1.0);
 }
}