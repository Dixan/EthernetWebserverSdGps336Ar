/*
  MEGA2560 VERSION!

  Web Server

  A simple web server that shows the value of the analog input pins.
  using an Arduino Wiznet Ethernet shield.

  Circuit:
   Ethernet shield attached to pins 10, 11, 12, 13
   Analog inputs attached to pins A0 through A5 (optional)

  created 18 Dec 2009
  by David A. Mellis
  modified 9 Apr 2012
  by Tom Igoe
  modified 02 Sept 2015
  by Arturo Guadalupi

  GPS added 23oct2020           : Dick van Kalsbeek
  SD webserver added 24dec2020  : Dick van Kalsbeek
  Git reopo created 27dec2020   : Dick van Kalsbeek

*/
//libraries
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Ethernet.h>
#include <SD.h>

// Enter a MAC address and IP address for your controller below.; The IP address will be dependent on your local network:
byte mac[] =
{
  0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6
};
IPAddress ip(192, 168, 178, 60);
//IPAddress ip(169, 254, 178, 60);
IPAddress gateway(192, 168, 178, 1);
IPAddress dnServer(192, 168, 178, 1);
IPAddress subnet(255, 255, 255, 0);

// Initialize the Ethernet server library; with the IP address and port you want to use; (port 80 is default for HTTP):
EthernetServer server(80);

//instances
TinyGPSPlus objGps;
SoftwareSerial objSoftSer(4, 3); //pin 4 : GPS-Tx is Arduino Rx; pin 3: GPS-Rx is Arduino-Tx
File webFile;

//structs
struct strGpsData
{
  String fixAltitude = String(2);
  String fixLongitude = String(6);
  String fixLatitude = String(6);
  String fixTimeHhMmSsCc;
  String fixNrOfSatellites;
};

//globals
strGpsData gpsData;

//io
#define diUseSdCardWhenLow 8

void setup()
{
  // You can use Ethernet.init(pin) to configure the CS pin
  //Ethernet.init(10);  // Most Arduino shields
  //Ethernet.init(5);   // MKR ETH shield
  //Ethernet.init(0);   // Teensy 2.0
  //Ethernet.init(20);  // Teensy++ 2.0
  //Ethernet.init(15);  // ESP8266 weih Adafruit Featherwing Ethernet
  //Ethernet.init(33);  // ESP32 with Adafruit Featherwing Ethernet
  InitSerial();
  InitSketchInfo();
  InitEthernetShield();
  InitPins();
  InitSoftwareSerial();
  InitSdCard();
}

void InitSerial()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Ethernet WebServer Example");
  Serial.println("MEGA2560 VERSION!");
}

void InitSketchInfo()
{
  Serial.print("Sketch:   ");
  Serial.println(__FILE__);
  Serial.print("Uploaded: ");
  Serial.print(__DATE__);
  Serial.print(", ");
  Serial.println(__TIME__);
  Serial.println(" ");
  Serial.println("created by: Dick van Kalsbeek");
}

void InitPins()
{
  Serial.println("Pins: init");
  pinMode(diUseSdCardWhenLow, INPUT_PULLUP);
  Serial.println("Pins: ready");
}

void InitSoftwareSerial()
{
  Serial.println("Second serial: init");
  //objSoftSer.begin(9600); //default speed of ATGM336H GPS Module
  Serial1.begin(9600); //default speed of ATGM336H GPS Module
  Serial.println("Second serial: ready");
}

void InitEthernetShield()
{
  Serial.println("Ethernet shield: init");
  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  //Ethernet.begin(mac, ip, dnServer, gateway, subnet);

  // Check for Ethernet hardware present
  while (Ethernet.hardwareStatus() == EthernetNoHardware)
  {
    Serial.println("Ethernet shield was not found. Wait 200ms and re-init.");
    delay(500);
    Ethernet.begin(mac, ip);
    //    while (true)
    //    {
    //      delay(1); // do nothing, no point running without Ethernet hardware
    //    }
  }

  //  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  //  {
  //    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
  //    while (true) {
  //      delay(2); // do nothing, no point running without Ethernet hardware
  //    }
  //  }

  if (Ethernet.linkStatus() == LinkOFF)
  {
    Serial.println("Ethernet cable is not connected.");
  }

  // start the server
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

  Serial.println("Ethernet shield: ready");
}

void InitSdCard()
{
  // initialize SD card
  Serial.println(" SD card: init");

  if (!SD.begin(4))
  {
    Serial.println("ERROR - SD card initialization failed!");
    return;    // init failed
  }
  Serial.println("SUCCESS - SD card initialized.");

  // check for index.htm file
  if (!SD.exists("index.htm"))
  {
    Serial.println("ERROR - Can't find index.htm file!");
    return;  // can't find index file
  }
  Serial.println("SUCCESS - Found index.htm file.");

  Serial.println("SD card: ready");
}

void SerialEvent()
{

}

void loop()
{
  UpdateWebServer();
  RetrieveGpsData();
}

void UpdateWebServer()
{
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client)
  {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected())
    {
      if (client.available())
      {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 5");  // refresh the page automatically every 5 sec

          if (digitalRead(diUseSdCardWhenLow) == HIGH)
          {
            HardCodedWebSite(client);
          }
          else
          {
            SdCardWebSite(client);
          }

          break;
        }
        if (c == '\n')
        {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}

void RetrieveGpsData()
{
  static int m_fixNrOfSatellitesPrev = 0;

  //while (objSoftSer.available() > 0)
  while (Serial1.available() > 0)
  {
    //Serial.println("Read GPS data");
    //objGps.encode(objSoftSer.read());
    objGps.encode(Serial1.read());

    //    if (objGps.location.isUpdated())
    //    {
    //      Serial.println("---");
    //      Serial.print("Time (hhmmsscc): ");
    //      Serial.println(objGps.time.value());
    //      gpsData.fixTimeHhMmSsCc = objGps.time.value();
    //
    //      Serial.print("Latitude: ");
    //      Serial.println(objGps.location.lat(), 6);
    //      gpsData.fixLatitude = objGps.location.lat();
    //
    //      Serial.print("Longitude: ");
    //      Serial.println(objGps.location.lng(), 6);
    //      gpsData.fixLongitude = objGps.location.lng();
    //
    //      Serial.print("Altitude in meters: ");
    //      Serial.println(objGps.altitude.meters(), 2);
    //      gpsData.fixAltitude = objGps.altitude.meters();
    //
    //      Serial.print("Number of satellites in use: ");
    //      Serial.println(objGps.satellites.value());
    //      gpsData.fixNrOfSatellites = objGps.satellites.value();
    //    }

    //    gpsData.fixNrOfSatellites = objGps.satellites.value();
    //
    //    if (gpsData.fixNrOfSatellites.toInt() > m_fixNrOfSatellitesPrev)
    //    {
    //      m_fixNrOfSatellitesPrev = gpsData.fixNrOfSatellites.toInt();
    //      UpdateWebServer();
    //    }
  }
}

void HardCodedWebSite(EthernetClient client)
{
  Serial.println("Getting hardcoded website source..");

  client.println();
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
  // output the value of each analog input pin
  //          for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
  //            int sensorReading = analogRead(analogChannel);
  //            client.print("analog input ");
  //            client.print(analogChannel);
  //            client.print(" is ");
  //            client.print(sensorReading);
  //            client.println("<br />");
  //          }

  client.println("<br />");
  client.print("Arduino Ethernetserver with:");
  client.println("<br />");
  client.print("GPS data from ATGM336");
  client.println("<br />");
  client.print("Created by Dick van Kalsbeek 23oct2020");
  client.println("<br />");
  client.print("Auto refresh rate at 5 sec");
  client.println("<br />");

  client.println("<br />");
  client.print("time : ");
  client.print(objGps.time.value());
  client.println("<br />");

  client.print("Latitude : ");
  client.println(objGps.location.lat(), 6);
  client.println("<br />");
  client.print("breedtegraad - evenaar is 90 - noorderbreedte - t.o.v. NZ - noordelijk en zuidelijk halfrond");
  client.println("<br />");
  client.println("<br />");

  client.print("Longitude : ");
  client.println(objGps.location.lng(), 6);
  client.println("<br />");
  client.print("lengtegraad - Greenwich is 0 - oosterlengte - t.o.v. OW - westelijk en oostelijk halfrond");
  client.println("<br />");
  client.println("<br />");

  client.print("Altitude in meters: ");
  client.println(objGps.altitude.meters(), 2);
  client.println("<br />");

  client.print("Number of satellites in use: ");
  client.println(objGps.satellites.value());

  client.println("</html>");
}

void SdCardWebSite(EthernetClient client)
{
  Serial.println("Getting website source from microSD");

  webFile = SD.open("index.htm");        // open web page file
  String m_row = "";
  String m_actualValue = "";

  if (webFile)
  {
    client.println("<!DOCTYPE HTML>");
    Serial.println("Reading webfile from microSD");
    while (webFile.available())
    {
      m_row = webFile.readStringUntil('\r');
      //m_row = webFile.readString();
      //Serial.print(m_row);
      //
      //      if (m_row.indexOf("objGps.time.value()") > 0)
      //      {
      //        //Serial.println("Time tage found");
      //        m_actualValue = (String)objGps.time.value();
      //      }
      //
      //      if (m_row.indexOf("objGps.satellites.value()") > 0)
      //      {
      //        //Serial.println("Satellite tag found");
      //        m_actualValue = (String)objGps.satellites.value();
      //      }

      client.print(m_row + " " + m_actualValue); //beetje wazig.. ik krijg in de serial steeds een lege regel

      m_actualValue = "";
      //client.println(m_row); //beetje wazig.. ik krijg in de serial steeds een lege regel
    }

    client.print("End of page..");
    webFile.close();
    Serial.println("Closing webfile");
  }
}

void SdCardWebSiteOld(EthernetClient client)
{
  Serial.println("Getting website source from microSD");

  webFile = SD.open("index.htm");        // open web page file
  //char m_htmlData;

  if (webFile)
  {
    Serial.println("Reading webfile from microSD");
    while (webFile.available())
    {
      //m_htmlData = webFile.read();
      //Serial.println(m_htmlData);
      //client.write(m_htmlData); // send web page to client
      //client.write(webFile.read());
      client.print(webFile.read());
    }
    webFile.close();
    Serial.println("Closing webfile");
  }
}
