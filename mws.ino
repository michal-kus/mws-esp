#define STRING(num) #num
 
//GSM
//Leave empty if not set
const char simPIN[] = "";
 
//APN setup - check on operator website
const char apn[]      = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";
 
//TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22
 
//Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
//Set serial for AT commands (to SIM800 module)
#define SerialAT Serial1
 
//Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb
 
#include <Wire.h>
#include <TinyGsmClient.h>
 
#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif
 
//I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);
 
//TinyGSM Client for Internet connection
TinyGsmClient client(modem);
 
#define uS_TO_S_FACTOR 1000000UL   /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  3600        /* Time ESP32 will go to sleep (in seconds) 3600 seconds = 1 hour */
 
#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00
 
//server
const char server[] = "mobilewheatherstation.herokuapp.com";
const char resource[] = "/measurements";         
const int  port = 80;
 
//Temp and humidity sensor
#include <dht11.h>
dht11 DHT11;
#define DHT11PIN 4
 
//Air qual sensor
#include <DFRobot_SGP40.h>
DFRobot_SGP40    mySgp40;
 
//Temp and pressure sensor
#include <BMP180I2C.h>
#define I2C_ADDRESS_BMP 0x77
BMP180I2C bmp180(I2C_ADDRESS_BMP);
 
//Other configuration
String stationId = "TEST";
const int INTERVAL = 10000;
//measurementId is incremented after every request
int measurementId = 0;
 
bool setPowerBoostKeepOn(int en){
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    I2CPower.write(0x35); // 0x37 is default reg value
  }
  return I2CPower.endTransmission() == 0;
}
 
void setup()
{
  Serial.begin(115200);
 
  // Start I2C communication with GSM module
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);
 
  // Keep power for GSM when running from battery
  bool isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));
 
  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
 
  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);
 
  // Restart SIM800 module, it takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // use modem.init() if you don't need the complete restart
 
  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    modem.simUnlock(simPIN);
  }
 
  Wire.begin(18,19);
 
  setupAirQualitySensor();
 
  setupBMPSensor();
 
  esp_sleep_enable_timer_wakeup(INTERVAL);
}
 
void setupBMPSensor() {
  //begin() initializes the interface, checks the sensor ID and reads the calibration parameters.  
  if (!bmp180.begin())
  {
    Serial.println("begin() failed. check your BMP180 Interface and I2C Address.");
    while (1);
  }
 
  //reset sensor to default parameters.
  bmp180.resetToDefaults();
 
  //enable ultra high resolution mode for pressure measurements
  bmp180.setSamplingMode(BMP180MI::MODE_UHR);
}
 
void readTempAndHumiditySensor(){
  int chk = DHT11.read(DHT11PIN);
  Serial.print("Stan sensora: ");
  switch (chk)
  {
    case DHTLIB_OK: 
    Serial.print("OK"); 
    break;
    case DHTLIB_ERROR_CHECKSUM: 
    Serial.println("Control sum error"); 
    break;
    case DHTLIB_ERROR_TIMEOUT: 
    Serial.println("Timeout - no reply from sensor"); 
    break;
    default: 
    Serial.println("Unknown error"); 
    break;
  }
}
 
float getPressure() {
  //start a pressure measurement. pressure measurements depend on temperature measurement, you should only start a pressure 
  //measurement immediately after a temperature measurement. 
  if (!bmp180.measurePressure())
  {
    Serial.println("could not start perssure measurement, is a measurement already running?");
    return 0.0;
  }
 
  //wait for the measurement to finish. proceed as soon as hasValue() returned true. 
  do
  {
    delay(100);
  } while (!bmp180.hasValue());
 
  return (bmp180.getPressure() / 100);
}
 
void sendMeasurementsToServerViaGSM() {
  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
  }
  else {
    delay(10);
    SerialMon.println(" OK");
    
    SerialMon.print("Connecting to ");
    SerialMon.print(server);
    if (!client.connect(server, port)) {
      SerialMon.println(" fail");
    }
    else {
      delay(10);
      SerialMon.println(" OK");
    
      // Making an HTTP POST request
      SerialMon.println("Performing HTTP POST request...");
 
      measurementId++;
      
      String httpRequestData = "{\"stationId\":\"" + stationId +
      "\",\"measurementId\":" + measurementId +
      ",\"temperature\":" + String((float)DHT11.temperature) +
      ",\"humidity\":" + String((float)DHT11.humidity) +
      ",\"pressure\":" + String(getPressure()) +
      ",\"airQuality\":" + mySgp40.getVoclndex() +
      "}"; 
          
      client.print(String("POST ") + resource + " HTTP/1.1\r\n");
      client.print(String("Host: ") + server + "\r\n");
      client.println("Connection: close");
      client.println("Content-Type: application/json");
      client.print("Content-Length: ");
      client.println(httpRequestData.length());
      client.println();
      client.println(httpRequestData);

      SerialMon.println(httpRequestData);
 
      unsigned long timeout = millis();
      while (client.connected() && millis() - timeout < 10000L) {
        // Print available data (HTTP response from server)
        while (client.available()) {
          char c = client.read();
          SerialMon.print(c);
          timeout = millis();
        }
      }
      SerialMon.println();
    
      // Close client and disconnect
      client.stop();
      delay(10);
      SerialMon.println(F("Server disconnected"));
      modem.gprsDisconnect();
      delay(10);
      SerialMon.println(F("GPRS disconnected"));
    }
  }
}
void setupAirQualitySensor(){
  while(mySgp40.begin(10000) !=true){
    Serial.println("failed to init chip, please check if the chip connection is fine");
    delay(1000);
  }
  Serial.println("sgp40 initialized successfully!");
}
 
void loop()
{
  readTempAndHumiditySensor();
  sendMeasurementsToServerViaGSM();
  esp_deep_sleep_start();
}
