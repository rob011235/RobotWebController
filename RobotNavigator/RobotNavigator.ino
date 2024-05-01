/*
RobotNavigator.ino
Date: 1 May 2024
Programmer: Rob Garner (rgarner011235@gmail.com)
Purpose: Controller code for a robot that gets commands from a website over wifi and executes them.

Based on code written by Arturo Guadalupi
last revision November 2015
*/

#include <SPI.h>
#include <WiFiNINA.h>
#include <TinyGPS++.h> //to process NMEA messages from gps module
#include "config.h" 

///////please enter your sensitive data in config.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;
// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
//IPAddress server(74,125,232,128);  // numeric IP for Google (no DNS)

#define SERVER "robnmhu.pythonanywhere.com"
#define PATH   "/api/commands"

// Initialize the SSL client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP, port 443 is default for HTTPS):
WiFiClient client;

//Set up variables for next waypoint
coord nextWayPoint;

// Create a TinyGPS++ object
TinyGPSPlus gps;


void setup() {
  // initialize our starting waypoint
  nextWayPoint.lat = 35.0;
  nextWayPoint.lon = -105.0;
  nextWayPoint.alt = 6000.0;

  //Initialize serial connections
  Serial.begin(9600);
  Serial1.begin(9600);

  // check for the WiFi module:
  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);
  while (WiFi.status() == WL_NO_MODULE) {
    Serial.println("\nCommunication with WiFi module failed!");
    delay(1000);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  Serial.print("\nAttempting to connect to SSID: ");
  Serial.println(ssid);
  // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
  do {
    status = WiFi.begin(ssid, pass);
    delay(100); // wait until connected
  } while (status != WL_CONNECTED);
  Serial.println("\nConnected to wifi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  if (client.connect(SERVER, 80)) {
    Serial.println("connected to server");
    // Make a HTTP request:
    client.println("GET " PATH " HTTP/1.1");
    client.println("Host: " SERVER);
    client.println("Connection: close");
    client.println();
  }
}

uint32_t bytes = 0;

void loop() 
{
  if(millis()%50 == 0) processGPSInfo(); //Check command stream every second
  if(millis()%5000 == 0) checkForIncommingCommands(); //Check command stream every second
}
char commandBuffer[100];
int commandIndex = 0;
void checkForIncommingCommands()
{
    // if there are incoming bytes available
  // from the server, read them and print them:
  if(client.connected())
  {
    while (client.available()) 
    {
      char c = client.read();
      if(c == '\n')
      {
        commandBuffer[commandIndex++]='\0'; //terminate string
        Serial.print("Command line received: ");
        Serial.println(commandBuffer);
        commandIndex = 0; //reset so we can start reading a new line.
        //TODO: process command
        break; //Read one line then return to main thread
      }
      else
      {
        commandBuffer[commandIndex++]=client.read();
      }
    }
  }
  else
  {
    //Try to reconnect and get more commands
    client.connect(SERVER, 80);
  }
}
void processGPSInfo()
{
 if (Serial1.available() > 0)
  {
    if (gps.encode(Serial1.read()))
    {
      coord location;
      if(getGPSLocation(location))
      {
        Serial.print("Location: ");
        Serial.print(location.lat, 6);
        Serial.print(F(","));
        Serial.println(location.lon, 6);

        double distance = CalcDistance(location, nextWayPoint);
        Serial.print("Distance to next waypoint: ");
        Serial.print(distance/1000);
        Serial.println(" km.");

        float desiredHeading = CalcHeading(location, nextWayPoint);
        Serial.print("Target heading: ");
        Serial.print(desiredHeading,2);
        Serial.println(" degrees.");

        //TODO: Add ability to get heading from GPS displacement and magnatometer.
      }
    }
  }
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

bool getGPSLocation(coord& location)
{
  if (gps.location.isValid())
  {
    location.lat = gps.location.lat();
    location.lon = gps.location.lng();
    location.alt = gps.altitude.meters();
    return true;
  }
  return false;
}

// Calculates the distance between two coordinates
// using the haversine formula located here: 
//https://community.esri.com/t5/coordinate-reference-systems-blog/distance-on-a-sphere-the-haversine-formula/ba-p/902128#:~:text=For%20example%2C%20haversine(%CE%B8),longitude%20of%20the%20two%20points.
double CalcDistance(coord pt1, coord pt2)
{
  double R = 6371000.0; // radius of earth in meters
  double lat1 = pt1.lat * DEG_TO_RAD;
  double lon1 = pt1.lon * DEG_TO_RAD;
  double lat2 = pt2.lat * DEG_TO_RAD;
  double lon2 = pt2.lon * DEG_TO_RAD;
  double a1 = sin((lat2-lat1) / 2.0);
  double a2 = sin((lon2-lon1) / 2.0);
  double a = a1*a1 + cos(lat1) * cos(lat2) * a2*a2;
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double meters = R * c;  // output distance in meters;
  return meters;
}

float CalcHeading(coord pt1, coord pt2)
{
  float y = cos(pt1.lat)*sin(pt2.lat)-sin(pt1.lat)*cos(pt2.lat)*cos(pt2.lon-pt1.lon);
  float x = sin(pt2.lon - pt1.lon)*cos(pt2.lat);
  float headingf = atan2(x,y) * RAD_TO_DEG;
  int headingi = headingf; //get the integer part
  float remainder = headingf-headingi;// get the decimal part
  int headingn =  (headingi + 360)% 360; //normalize to 0-359
  float heading = headingn + remainder;
  return heading;
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}