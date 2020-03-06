// Code to test the Tx/Rx config of the Teensy
// relative to the Raspberry Pi Zero wf

void setup(){
  Serial5.begin(9600);
}

void loop(){
  delay(5000);
  Serial5.println("Hello World!");
}
