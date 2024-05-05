const int ledPin=13;
const int switchPin=4;
void setup()
{
  pinMode(ledPin,OUTPUT);
  pinMode(switchPin,INPUT);

}
void loop()
{
  int switchState=digitalRead(switchPin);
  if(switchPin==1)
  {
    digitalWrite(ledPin,HIGH);
    delay(1000);
  }
  else
  {
    digitalWrite(ledPin,LOW);
    delay(1000);
  }
}