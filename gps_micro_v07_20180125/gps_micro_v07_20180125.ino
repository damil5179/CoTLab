/*
  - 20180122    Implement SoftwareSerial (SungMin)
  - 20180122    Implement GPS (SungMin)
  - 20180123    Parse latitude&longitude (SungMin)
  - 20180125    Convert latitude&longitude format (SungMin)
  - 20180125    Definition of response method when GPS does not work (SungMin)
*/

#include <SoftwareSerial.h>
#include "tempo.h"

bool first_loop_exec;

#define GPS_INFO_BUFFER_SIZE 128
#define MESSAGE_ARRAY_BUFFER_SIZE 32
char GPS_info_char;
char GPS_info_buffer[GPS_INFO_BUFFER_SIZE];
unsigned int received_char;

int i; // counter
bool message_started;
int commaPosition = 0;
int messageArrayIndex = 0;
int gpsFlag = 0;
String message = "";
String messageArray[MESSAGE_ARRAY_BUFFER_SIZE];
String checkLatitude = "0000.00000";
String checkLongitude = "00000.00000";
float converseLatitudeType = 0;
float converseLongitudeType = 0;

String tempDegree = "";
String tempMinutes = "";
float tempDegreeFloat;
float tempMinutesFloat;
float latitudeDD; // Decimal Degree
float longitudeDD; // Decimal Degree

SoftwareSerial mySerial_GPS(8, 9); // 8=RX, 9=TX (needed to communicate with GPS)

// REAL TIME SCHEDULER PARAMETERS AND VARIABLES
#define SCHEDULER_TIME 10000 // scheduler interrupt runs every 20000us = 20ms
#define DIVIDER_STD 200 // logging message sent every 100 scheduler times (20ms) 1s
#define DIVIDER_DELAY 500 // delay after forwarding meggages is 3s
unsigned int divider=0;
unsigned int divider_max=DIVIDER_DELAY;

// SENDS THE POLLING MESSAGE TO GPS
void scheduled_interrupt() 
{
  divider++;
  if (divider==divider_max) {
    divider=0;
    divider_max=DIVIDER_STD;
    mySerial_GPS.println("$PUBX,00*33"); // data polling to the GPS
  }
}

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("Connected");
  mySerial_GPS.begin(9600);
  mySerial_GPS.println("Connected");

  pinMode(13, OUTPUT);

  first_loop_exec=true;
  i=0;
  message_started=false;
//  delay(30*1000); // Interrupt protection before boot
  Timer1.initialize(); // initialize 10ms scheduler timer
  Timer1.setPeriod(SCHEDULER_TIME); // sets the main scheduler time in microseconds (10ms in this case)
  Timer1.attachInterrupt(scheduled_interrupt); // attaches the interrupt
  Timer1.start(); // starts the timer

  digitalWrite(13, LOW);
}

void loop() { // run over and over
  
  if (first_loop_exec == true){
    delay(2000);
    mySerial_GPS.println(F("$PUBX,40,RMC,0,0,0,0*47")); //RMC OFF
    delay(100);
    mySerial_GPS.println(F("$PUBX,40,VTG,0,0,0,0*5E")); //VTG OFF
    delay(100);
    mySerial_GPS.println(F("$PUBX,40,GGA,0,0,0,0*5A")); //CGA OFF
    delay(100);
    mySerial_GPS.println(F("$PUBX,40,GSA,0,0,0,0*4E")); //GSA OFF
    delay(100);
    mySerial_GPS.println(F("$PUBX,40,GSV,0,0,0,0*59")); //GSV OFF
    delay(100);
    mySerial_GPS.println(F("$PUBX,40,GLL,0,0,0,0*5C")); //GLL OFF
    delay(1000);
    first_loop_exec = false;
  }

  // MANAGES THE CHARACTERS RECEIVED BY GPS
  while (mySerial_GPS.available()) {
    gpsFlag = 1;
    GPS_info_char=mySerial_GPS.read();
    
    if (GPS_info_char == '$'){ // start of message
      message_started=true;
      received_char=0;
    } else if (GPS_info_char == '*'){ // end of message
      messageArrayIndex = 0;
      message = String(GPS_info_buffer);
      Serial.println(message);
      do
      {
        commaPosition = message.indexOf(',');
        if (commaPosition != -1) {
          messageArray[messageArrayIndex] = message.substring(0, commaPosition);
          messageArrayIndex++;
          message = message.substring(commaPosition+1, message.length());
        }
      }
      while(commaPosition >= 0);

      if (messageArray[3].equals(checkLatitude) && messageArray[5].equals(checkLongitude)) {
        digitalWrite(13, HIGH);
        messageArray[3] = "3724.61482";
        messageArray[5] = "12708.50632";
      } else {
        digitalWrite(13, LOW);
      }
      
      // convert latitude format (decimal degree)
      tempDegree = messageArray[3].substring(0, 2);
      tempMinutes = messageArray[3].substring(2, 11);
      tempDegreeFloat = tempDegree.toFloat();
      tempMinutesFloat = tempMinutes.toFloat() / 60;
      latitudeDD = tempDegreeFloat + tempMinutesFloat;
      
      // convert longitude format (decimal degree)
      tempDegree = messageArray[5].substring(0, 3);
      tempMinutes = messageArray[5].substring(3, 12);
      tempDegreeFloat = tempDegree.toFloat();
      tempMinutesFloat = tempMinutes.toFloat() / 60;
      longitudeDD = tempDegreeFloat + tempMinutesFloat;
      
      Serial.print(">A");
      Serial.print(latitudeDD, 6);
      Serial.print("O");
      Serial.println(longitudeDD, 6);

      Serial1.print(">A");
      Serial1.print(latitudeDD, 6);
      Serial1.print("O");
      Serial1.println(longitudeDD, 6);
      
      message_started=false; // ready for the new message
    } else if (message_started==true){ // the message is already started and I got a new character
      if (received_char<=GPS_INFO_BUFFER_SIZE){ // to avoid buffer overflow
        GPS_info_buffer[received_char]=GPS_info_char;
        received_char++;
      } else { // resets everything (overflow happened)
        message_started=false;
        received_char=0;
      }
    }
  }

  if ((mySerial_GPS.available() == 0) && (gpsFlag != 1)) {
    Serial.println(">A37.410247O127.141772");
    Serial1.println(">A37.410247O127.141772");
    delay(5000);
  }

  while (Serial.available()) {
    mySerial_GPS.write(Serial.read());
  }
}
