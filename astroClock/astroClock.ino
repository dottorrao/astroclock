//===============================================================================================
//INCLUDE LIBRARIES SECTION
//===============================================================================================
#include <TFT_eSPI.h>       // Graphics and font library for ST7735 driver chip
#include <SPI.h>            // To communicate with Serial Peripheral Interface
#include <WiFi.h>           // ESP32 Core WiFi Library 
#include <WebServer.h>      // Local DNS Server used for redirecting all requests to the configuration portal (  https://github.com/zhouhan0126/DNSServer---esp32  )
#include <WiFiManager.h>    // WiFi Configuration Magic (  https://github.com/zhouhan0126/DNSServer---esp32  ) >>  https://github.com/zhouhan0126/DNSServer---esp32  (ORIGINAL)
                            // https://diyprojects.io/wifimanager-library-easily-manage-wi-fi-connection-projects-esp8266/#.X3ZH3vIzYYs
                            // Esp32 and pushbutton: https://microcontrollerslab.com/push-button-esp32-gpio-digital-input/
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_ST7735.h>  // LCD driver
#include <Adafruit_GFX.h>     // LCD graphical driver
#include <HTTPClient.h>       // Http client to get data from API on line      
#include <ArduinoJson.h>      // Jeson library to manage JSON object (reading of data from API on line)
#include <time.h>             // Used to manage date and time 

//URL to get nea data from NASA: https://ssd-api.jpl.nasa.gov/cad.api?body=Earth&sort=date&nea=true&limit=10&fullname=true

//===============================================================================================
//DEFINE SECTION
//===============================================================================================
//LCD width & high
#define TFT_W 160         //LCD width
#define TFT_H 128         //LCD high

//The colors must be re-defined because the TST LCD that it's in used for this projects has a RGB codification inverted.
#define BLACK 0x0000
#define YELLOW 0x07FF 
#define WHITE 0xFFFF
#define BLUE 0xF800
#define GREEN 0x07E0
#define RED 0x001F
/*
#define CYAN 0xFFE0
#define MAGENTA 0xF81F
#define DGRAY 0x528A
#define LGRAY 0x2945
#define ORANGE 0xFC02
*/

//===============================================================================================
//VARIABLES DECLARATION SECTION
//===============================================================================================
int PIN_RESET_BUTTON = 17;  //This pin on ESP32 is used to reset the ESP32 it self and WIFI manager
int RESET = 0;              //This variable is used to get the status (HIGH or LOW) of PIN_RESET_BUTTON. If status is HIGH, reset is executed.
String formattedDate;       //Contains the date in formatted way as specified in string type

String dayStamp;            //Contains the current date in string type
String timeStamp;           //Contains the current time in string type

String NEO1name;            //Variable to define NEO1 name
String NEO1date;            //Variable to define NEO1 date of the near approach
String NEO1distance;        //Variable to define NEO1 near distance from Eart
float  NEO1distanceFLoat;

String NEO2name;            //Variable to define NEO2 name
String NEO2date;            //Variable to define NEO2 date of the near approach
String NEO2distance;        //Variable to define NEO2 near distance from Eart
float  NEO2distanceFLoat;

String timeStampOLD;          //Variable used to keep old timestamp value in order to synchronize periodically date and time
unsigned long startMillis;    //Variable used to manage the synchronisation of the date and time with NPD server
unsigned long currentMillis;  //Variable used to manage the synchronisation of the date and time with NPD server

const unsigned long period = 6 * 60 * 60 * 1000; // hh * mm * ss * millisecond  //Definition of period for any synchronisation (hours/minutes/seconds/milliseconds)
struct tm timeinfo;           //Structs to contains data from NPD server


//===============================================================================================
//OBJECTS DECLARATION SECTION
//===============================================================================================
WiFiManager wm;               // Object WIFIManager to manage the WIFI connection 
WiFiUDP ntpUDP;               // Used to connect to UDP server to get current date/time
NTPClient timeClient(ntpUDP); // Object used to connect to UDP server to get current date/time
TFT_eSPI tft = TFT_eSPI();    // Invoke library, pins defined in User_Setup.h

//===============================================================================================
//METHODS SECTION
//===============================================================================================

//Call back method in case of failure in WIFI connection
//@TODO: test if it's working correctly and better define
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

//---------------------
//WIFI Manager connection. WIFI Manager is setting UP an accesspoint if previous credential are null or
//no longer valid. Otherwise is using last WIFI credential to connect (they are saved into permanent memory)
//@TODO: display message during the connection
void wifiConnection (){
  wm.setAPCallback(configModeCallback);
  // try to connet, in case of failure, create the access point "ASTRO_CLOCK"
  wm.autoConnect("AstroClock");
  // if process arrive here, connection has been enabled
  Serial.print("ESP32 is connected to Wi-Fi network ");
  Serial.println (WiFi.SSID());
}


//---------------------
//Init TimeClient of ESP32 device.
void initTimeClient(){
  timeClient.begin();
  //@TODO set timeoff dinamically
  //timeClient.setTimeOffset(7200);
}

//---------------------
/*
This method is used to get current date and time from NTP server.
Current date/time is get from NTP server and local configTime is set.
*/
void initLocalTime(){  
  Serial.println("initLocalTime(): Initialization of Local Time...:");
  const char* ntpServer = "pool.ntp.org";   //ntp server
  const long  gmtOffset_sec = 7200;         //Greenwich offset in second (Italy)
  const int   daylightOffset_sec = 0;
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); // datetime inizialization
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.print("initLocalTime(): Inizialization time is:");
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.println("initLocalTime(): Initialization of Local Time executed!");
}

//---------------------
//Reset the device
void resetDevice(){
  Serial.println("Erase settings and restart ...");
  wm.resetSettings();  
  WiFi.disconnect(false,true);
  ESP.restart();
  delay(2000);  
}

//---------------------
//Init LCD display
void initLCD(){
  tft.init();           // Initialization of TFT display  
  tft.setRotation (7);  // Needed to keep the LCD display in landscape orientation.
  tft.fillRect(0, 0, TFT_W, TFT_H, TFT_BLACK);
}

//---------------------
//Video display of the data: Date, Hour, NEO1 and NEO2 details
void printDisplay(){
  static char outstr[15];
  getLocalTime(&timeinfo);
  tft.setCursor(1,1);
  tft.setTextSize(1);
  tft.setTextColor(GREEN,BLACK);
  tft.println(&timeinfo, "%A, %B %d %Y");
  tft.setTextSize(1);
  tft.println("");
  tft.setTextSize(3);
  tft.setTextColor(YELLOW,BLACK);
  tft.println(&timeinfo, "%H:%M:%S");
  tft.setTextSize(1);
  tft.setTextColor(GREEN,WHITE);
  tft.println("");
  
  tft.setTextColor(RED);
  tft.print ("Name: ");
  tft.setTextColor(WHITE);
  tft.println(NEO1name);
  tft.setTextColor(RED);
  tft.print ("Date: ");
  tft.setTextColor(WHITE);
  tft.println(NEO1date);
  tft.setTextColor (RED);
  tft.print("Dis:  ");
  tft.setTextColor(WHITE);
  dtostrf(NEO1distanceFLoat,9, 3, outstr);
  tft.print(outstr);
  tft.println(" Km");
  
  tft.println("");

  tft.setTextColor(RED);
  tft.print ("Name: ");
  tft.setTextColor(WHITE);
  tft.println(NEO2name);
  tft.setTextColor(RED);
  tft.print ("Date: ");
  tft.setTextColor(WHITE);
  tft.println(NEO2date);
  tft.setTextColor (RED);
  tft.print("Dis:  ");
  tft.setTextColor(WHITE);
  dtostrf(NEO2distanceFLoat,9, 3, outstr);
  tft.print(outstr);
  tft.println(" Km");
  
  timeStampOLD = timeStamp;
}

//---------------------
//Method to split a string by a given chars and going back the needed values.
//Input: String to split, separator, index of the element that has to return.
String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

//---------------------
//Getting data from API NASA 
//@TODO: optimaze the code creating the appropriate method to avoid to repeat the code.
void getNEOData(){
  Serial.println ( "Getting data from API NASA ssd-api.jpl.nasa.gov" );
  HTTPClient http;
  http.begin("https://ssd-api.jpl.nasa.gov/cad.api?body=Earth&sort=date&nea=true&limit=10&fullname=true");
  int httpCode = http.GET();
  if (httpCode > 0) {
    const size_t bufferSize = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(5) + JSON_OBJECT_SIZE(8) + 370;
    DynamicJsonBuffer jsonBuffer(bufferSize);
    
    JsonObject& root = jsonBuffer.parseObject(http.getString());
    root.printTo(Serial);
    Serial.println("");
    String NEO1 = root["data"][0];
    String NEO2 = root["data"][1];
    //-------
    NEO1.remove(0,1);
    NEO1.remove(NEO1.length()-1,NEO1.length());
    NEO1name = getValue(NEO1,',',0);
    NEO1date = getValue(NEO1,',',3);
    NEO1distance = getValue(NEO1,',',4);
    NEO1name.remove(0,1);
    NEO1name.remove(NEO1name.length()-1,NEO1name.length());
    NEO1date.remove(0,1);
    NEO1date.remove(NEO1date.length()-1,NEO1date.length()); 
    NEO1distance.remove(0,1);
    NEO1distance.remove(NEO1distance.length()-1,NEO1distance.length());
    NEO1distance.trim();
    NEO1distance = NEO1distance.substring(0,6);
    NEO1distanceFLoat = NEO1distance.toFloat();
    NEO1distanceFLoat = NEO1distanceFLoat * 149597870.7; // 1 au = 149597870.7 km
    //NEO1distance = String(NEO1distanceFLoat);
    //NEO1distance = NEO1distance.substring(0,10);
    
    //-------
    NEO2.remove(0,1);
    NEO2.remove(NEO2.length()-1,NEO2.length());
    NEO2name = getValue(NEO2,',',0);
    NEO2date = getValue(NEO2,',',3);
    NEO2distance = getValue(NEO2,',',4);
    NEO2name.remove(0,1);
    NEO2name.remove(NEO2name.length()-1,NEO2name.length());
    NEO2date.remove(0,1);
    NEO2date.remove(NEO2date.length()-1,NEO2date.length()); 
    NEO2distance.remove(0,1);
    NEO2distance.remove(NEO2distance.length()-1,NEO2distance.length());
    NEO2distance.trim();
    NEO2distance = NEO2distance.substring(0,6);
    NEO2distanceFLoat = NEO2distance.toFloat();
    NEO2distanceFLoat = NEO2distanceFLoat * 149597870.7; // 1 au = 149597870.7 km
    //NEO2distance = String(NEO2distanceFLoat);
    //NEO2distance = NEO2distance.substring(0,10);
  }
}

//===============================================================================================
//SETUP
//===============================================================================================
void setup() {
  Serial.begin(115200);
  startMillis = millis();
  pinMode(PIN_RESET_BUTTON, INPUT);
  // connection to WIFI
  wifiConnection();
  delay(100);
  // initialization of time client
  initLocalTime();
  initTimeClient();
  // initialization of LCD display
  initLCD();
  delay(100);
  getNEOData();
}

//===============================================================================================
//LOOP - MAIN PROGRAM
//===============================================================================================
void loop() {
  currentMillis = millis();
  if ( (currentMillis - startMillis >= period) ){
    startMillis = currentMillis;
    getNEOData();
    initLocalTime();
    initTimeClient();
  }
  //RESET = digitalRead(PIN_RESET_BUTTON);
  //if(RESET == HIGH) {                                 
  //  resetDevice();
  //}
  //getDateTimeNTP();
  delay(1000);
  printDisplay();
}
