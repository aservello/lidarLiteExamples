// LidarLight PWM Test

unsigned long pulseWidth;

void setup()
{
  Serial.begin(9600); // Start serial communications

  pinMode(2, OUTPUT); // pin 2 will be triggdr pin
  digitalWrite(2, LOW); // low means continus read

  pinMode(3, INPUT); // pin 3 will be monitor pin
}

void loop()
{
  pulseWidth = pulseIn(3, HIGH); // Count how long the pulse is high in microseconds

  // if its not 0 print the distance 
  if(pulseWidth != 0)
  {
    pulseWidth = pulseWidth / 10; // 10  is 1 cm
    Serial.println(pulseWidth); // Print the distance
    Serial.println(" cm"); // printing cm after dist
  }
}
