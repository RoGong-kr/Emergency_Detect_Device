#include <SoftwareSerial.h>

SoftwareSerial btSerial1(2, 3); // RX, TX
String result[6];

int in3 = 10;
int in4 = 11;
int input = 8;
int ledR = 7;
int m1 = 5;
int m2 = 6;

unsigned long previousMillis = 0;
const long delayTime = 1000;
bool isConnected = false;
void setup() {
  Serial.begin(9600);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(input, INPUT);
  pinMode(ledR, OUTPUT);
  pinMode(m1, OUTPUT); // Conveyor Motor
  pinMode(m2, OUTPUT); // hammer motor

  btSerial1.begin(9600);
}

bool stackMSG1 = true;

void loop() {

  if (btSerial1.available()) //when bluetooth data exist
  {
    char id1msg = btSerial1.parseInt();
    Serial.println(id1msg);

    analogWrite(in3, 0);
    analogWrite(in4, 150);
    //Serial.print("inputPin : "); Serial.println(digitalRead(input));

    if (digitalRead(input) == 0) // check emergency switch
    {
      digitalWrite(ledR, HIGH);
      tone(12, 262);
    }
    else {
      noTone(12);
      digitalWrite(ledR, LOW);
      analogWrite(m1, 255); // Run Conveyor Belt motor
      analogWrite(m2, 0);   // Stop Emergency Hammer motor
    }
    
    if ((id1msg == false) )//|| (id2msg == false))
    {
      analogWrite(m1, 0);   // Stop Conveyor Belt motor
      analogWrite(m2, 255); // Run Hammer motor for hitting Emergency Switch
    }

  }
  
  if (!isConnected) // when bluetooth is disconnected.
  {
    // send cmd to bluetooth module
    btSerial1.write("AT+INQ");
    btSerial1.write('\r');
    btSerial1.write('\n');
    
    while (!btSerial1.available()); // wait cmd response
    String msg1 = btSerial1.readString();
    Split(msg1, '\n');
    Serial.println("ID1:AT+INQ");
    String cmd = result[3];
    cmd = cmd.substring(0, 15);

    if (cmd.equals("Devices Found 1")) // Find device
    {
      // Pair found device
      btSerial1.write("AT+CONN1");
      btSerial1.write('\r');
      btSerial1.write('\n');
      Serial.println("ID1:AT+CONN1");
      isConnected = true;
    }
  }

}


void Split(String sData, char cSeparator)
{
  int nGetIndex = 0 ;
  int msgCount = 0;
  String sTemp = "";

  String sCopy = sData;
  while (true)
  {
    nGetIndex = sCopy.indexOf(cSeparator);
    if (-1 != nGetIndex)
    {
      sTemp = sCopy.substring(0, nGetIndex);

      //Serial.println( sTemp );
      result[msgCount] = sTemp;
      msgCount++;

      sCopy = sCopy.substring(nGetIndex + 1);
    }
    else
    {
      //Serial.println( sCopy );
      result[msgCount] = sCopy;
      break;
    }
  }
}
