int in1 = A0;
int in2 = A1;
int in3 = A2;

int v1 = 0;
int v2 = 0;
int v3 = 0;

void setup() {
  Serial.begin(9600); 
}

void loop() {
  v1 = analogRead(in1);
  v2 = analogRead(in2);
  v3 = analogRead(in3);
  Serial.println(v1);
  Serial.println(v2);
  Serial.println(v3);
  Serial.println(" ");
  delay(250);
}
