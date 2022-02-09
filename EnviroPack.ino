//WIP, Base sur un exemple Tinygps++
//Modifications de l'exemple faites par GrimtekTachyon


#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h> 
#include <U8x8lib.h> 
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 2
#define DHTTYPE DHT11
#define RXpin A3                                  
#define TXpin A2                                   
#define GPSPOWER -1                                
#define GPSONSTATE HIGH                            
#define GPSOFFSTATE LOW   

TinyGPSPlus gps; 
SoftwareSerial GPSserial(RXpin, TXpin);

U8X8_SSD1306_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE);    


float GPSLat;                                      
float GPSLon;                                     
float GPSAlt;                                     
float GPSHead;
uint8_t GPSSats;                                   
uint32_t GPSHdop;                                  
uint8_t hours, mins, secs, day, month;
uint16_t year;
uint32_t startGetFixmS;
uint32_t endFixmS;
double ppm;
int smokeA0 = A0;

DHT_Unified dht(DHTPIN, DHTTYPE);



void loop()
{
  if (gpsWaitFix(5))
  {
    Serial.println();
    Serial.println();
    Serial.print(F("Fix time "));
    Serial.print(endFixmS - startGetFixmS);
    Serial.println(F("mS"));

    GPSLat = gps.location.lat();
    GPSLon = gps.location.lng();
    GPSAlt = gps.altitude.meters();
    GPSSats = gps.satellites.value();
    GPSHdop = gps.hdop.value();
    GPSHead = gps.course.deg();

    printGPSfix();
    //displayscreen1();
    //uncomment if a 128x64 Oled display is used
    
    startGetFixmS = millis();    //have a fix, next thing that happens is checking for a fix, so restart timer
  }
  else
  {
    //displayscreen2();
    //uncomment if a 128x64 Oled display is used
    
    disp.print( (millis() - startGetFixmS) / 1000 );
    Serial.println();
    Serial.println();
    Serial.print(F("Timeout - No GPS Fix "));
    Serial.print( (millis() - startGetFixmS) / 1000 );
    Serial.println(F("s"));
  }

//Qualite de l'air, Temperature et Humidite
  MQ2();
  dht11();
  
}


bool gpsWaitFix(uint16_t waitSecs)
{
  //waits a specified number of seconds for a fix, returns true for good fix

  uint32_t endwaitmS;
  uint8_t GPSchar;

  Serial.print(F("Wait GPS Fix "));
  Serial.print(waitSecs);
  Serial.println(F(" seconds"));

  endwaitmS = millis() + (waitSecs * 1000);

  while (millis() < endwaitmS)
  {
    if (GPSserial.available() > 0)
    {
      GPSchar = GPSserial.read();
      gps.encode(GPSchar);
      //Serial.write(GPSchar);
    }

    if (gps.location.isUpdated() && gps.altitude.isUpdated() && gps.date.isUpdated())
    {
      endFixmS = millis();                                
      return true;
    }
  }

  return false;
}


void printGPSfix()
{
  float tempfloat;

  Serial.print(F("New GPS Fix "));

  tempfloat = ( (float) GPSHdop / 100);

  Serial.println(F("Lat,"));
  Serial.print(GPSLat, 6);
  Serial.println(F(",Lon,"));
  Serial.print(GPSLon, 6);
  Serial.println(F(",Alt,"));
  Serial.print(GPSAlt, 1);
  Serial.println(F("m,Sats,"));
  Serial.print(GPSSats);
  Serial.println(F(",HDOP,"));
  Serial.print(tempfloat, 2);
  Serial.print(F(",Time,"));
  Serial.println();
  Serial.println();
}


void displayscreen1()
{
  sensors_event_t event;
 
  //show GPS data on display
  float tempfloat;
  tempfloat = ( (float) GPSHdop / 100);

  disp.clearLine(0);
  disp.clearLine(1);
  disp.setCursor(0, 0);
  disp.print("Lat:  ");
  disp.print(GPSLat, 6);
  
  disp.clearLine(2);
  disp.setCursor(0, 1);
  disp.print("Lon: ");
  disp.print(GPSLon, 6);
  
  disp.clearLine(3);
  disp.setCursor(0, 2);
  disp.print("Alt :");
  disp.print(GPSAlt, 1);
  
  disp.clearLine(4);
  disp.setCursor(0, 3);
  disp.print(F("Sats "));
  disp.print(GPSSats);
  
  disp.clearLine(5);
  disp.setCursor(0, 4);
  dht.temperature().getEvent(&event);
  disp.print("Temp:");
  disp.print(event.temperature, 2);
  disp.print(F("C"));

  disp.clearLine(6);
  disp.setCursor(0, 5);
  dht.humidity().getEvent(&event);
  disp.print("Hum: ");
  disp.print(event.relative_humidity, 0);
  disp.print(F("%"));
  
  disp.clearLine(7);
  disp.setCursor(0, 6);
  disp.print("MQ2: ");
  int analogSensor = analogRead(smokeA0);
  ppm = 200+(analogSensor*9.57);
  disp.print(ppm);
  disp.print("ppm");
  
  disp.clearLine(8);
  disp.setCursor(0, 7);
  disp.print("--");
  /*disp.print("Air quality: ");
  if(ppm<450){
  disp.print("Good");}
  if(450<ppm<1000){
    disp.print("Medium");}
  if(ppm>1000){
    disp.print("Get out");}*/

}

void displayscreen2()
{
  sensors_event_t event;
  
  //show GPS data on display
  float tempfloat;
  tempfloat = ( (float) GPSHdop / 100);

  disp.clearLine(0);
  disp.setCursor(0, 0);
  disp.print(F("No GPS Fix "));

  disp.clearLine(1);
  disp.clearLine(2);
  disp.clearLine(3);
  disp.clearLine(4);
  disp.clearLine(7);
  disp.clearLine(8);
  
  disp.clearLine(5);
  disp.setCursor(0, 4);
  dht.temperature().getEvent(&event);
  disp.print("Temp:");
  disp.print(event.temperature, 2);
  disp.print(F("C"));

  disp.clearLine(6);
  disp.setCursor(0, 5);
  dht.humidity().getEvent(&event);
  disp.print(F("Hum: "));
  disp.print(event.relative_humidity, 0);
  disp.print(F("%   "));

  disp.clearLine(7);
  disp.setCursor(0, 6);
  disp.print("MQ2: ");
  int analogSensor = analogRead(smokeA0);
  ppm = 200+(analogSensor*9.57);
  disp.print(ppm);
  disp.print("ppm");
  
  disp.clearLine(8);
  disp.setCursor(0, 7);
  disp.print("--");
  /*disp.print("Air quality: ");
  if(ppm<450){
    disp.print("Good");}
  if(450<ppm<1000){
    disp.print("Medium");}
  if(ppm>1000){
    disp.print("Get out");}*/

  disp.setCursor(11,0);
}

void GPSON()
{
  if (GPSPOWER)
  {
    digitalWrite(GPSPOWER, GPSONSTATE);                         
  }
}


void GPSOFF()
{
  if (GPSPOWER)
  {
    digitalWrite(GPSPOWER, GPSOFFSTATE);                        
  }
}

void dht11(){
  
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.println();
    Serial.println();
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
    Serial.println();
    Serial.println();
    }
  }

void MQ2(){
  
  int analogSensor = analogRead(smokeA0);
  
  ppm = 200+(analogSensor*9.57);
  
  Serial.print("Smoke ppm: ");
  Serial.println(ppm);
}

void setup()
{
  if (GPSPOWER >= 0)
  {
    pinMode(GPSPOWER, OUTPUT);
    GPSON();
  }

  GPSserial.begin(9600);

  Serial.begin(115200);
  Serial.println();
  Serial.print(F(__TIME__));
  Serial.print(F(" "));
  Serial.println(F(__DATE__));

  disp.begin();
  disp.setFont(u8x8_font_chroma48medium8_r);
  disp.clear();
  disp.setCursor(0, 0);
  disp.print(F("Display Ready"));

  Serial.println(F("29_GPS_Checker_Display Starting"));
  Serial.println();

  startGetFixmS = millis();

  //Setup Temperature
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);

  
  //Setup MQ-2
  pinMode(smokeA0, INPUT);
}
