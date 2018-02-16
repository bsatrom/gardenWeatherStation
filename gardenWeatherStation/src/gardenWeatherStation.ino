/*
 * Project gardenWeatherStation
 * Description: Particle Project for monitoring envionmental conditions in a
 * home garden.
 * Author: Brandon Satrom
 * Date: 02/15/2018
 */

 #include "HTU21D.h"
 #include "SparkFun_MPL3115A2.h"
 #include "SparkFunMAX17043.h"

 #define TEMPERATURE_PRECISION 11

 int WDIR = A0;
 int RAIN = D2;
 int WSPEED = D3;

 // Timing variables
 unsigned long previousSensorMillis = 0;
 unsigned long previousBatteryMillis = 0;
 unsigned long sensorInterval = 60000;
 unsigned long batteryInterval = 300000;

 /* Battery Variables */
 double voltage = 0;
 double charge = 0;
 bool alert;
 int threshold = 20;

 HTU21D htu = HTU21D();
 MPL3115A2 baro = MPL3115A2();

 void setup()
 {
 	 Serial.begin(9600);

   setupBatteryMonitor();
   setupHTU();
   setupBarometer();
   setupWindAndRainSensors();

   // Take initial battery reading
   getBatteryStatus();
 }

 void loop()
 {
   unsigned long currentMillis = millis();

   if(currentMillis - previousBatteryMillis > batteryInterval) {
     previousBatteryMillis = currentMillis;

     getBatteryStatus();
   }

   if(currentMillis - previousSensorMillis > sensorInterval) {
     previousSensorMillis = currentMillis;

     getStationReadings();
   }


 }

 void setupBatteryMonitor() {
   // Set up battery-related Spark variables
 	 Particle.variable("voltage", voltage);
 	 Particle.variable("charge", charge);
 	 Particle.variable("alert", alert);

   Serial.println("Setting up Battery monitor");

   lipo.begin();
 	 lipo.quickStart();
   lipo.setThreshold(threshold); // Set alert threshold to 20%.
 }

 void setupHTU() {
   // Setup temperature and humidity sensor
   Serial.println("HTU21D test");
   while(!htu.begin()){
 	  Serial.println("HTU21D not found");
 	  delay(1000);
 	}
   Serial.println("HTU21D OK");
 }

 void setupBarometer() {
   // Setup pressure and altitude sensor
   while(!baro.begin()) {
     Serial.println("MPL3115A2 not found");
     delay(1000);
   }
   Serial.println("MPL3115A2 OK");

   baro.setModeAltimeter();
   baro.setOversampleRate(7);
   baro.enableEventFlags();
 }

 void setupWindAndRainSensors() {
   pinMode(WSPEED, INPUT_PULLUP);
   pinMode(RAIN, INPUT_PULLUP);

   // attach external interrupt pins to IRQ functions
   //attachInterrupt(RAIN, rainIRQ, FALLING);
   //attachInterrupt(WSPEED, wspeedIRQ, FALLING);

   // turn on interrupts
   interrupts();
 }

 void getStationReadings() {
   String payload;
   String tempStr;
   String humidityStr;
   String pressureStr;
   String altStr;

   float tempC = 0;
   float tempF = 0;
   float humidity = 0;
   float pressure = 0;
   float alt = 0;

   tempC = htu.readTemperature();
   tempF = tempC * (9.0/5.0) + 32;

   humidity = htu.readHumidity();
   pressure = baro.readPressure();
   alt = baro.readAltitudeFt();

   tempStr = String(tempF, 2);
   humidityStr = String(humidity, 2);
   pressureStr = String(pressure, 2);
   altStr = String(alt, 2);

   payload = "{ \"at\":" + tempStr + ", \"h\":"
     + humidityStr + ", \"p\":" + pressureStr
     + ", \"a\":" + altStr + " }";

   Spark.publish("SensorReading", payload);
 }

 void getBatteryStatus() {
   String payload;
   String voltageStr;
   String chargeStr;
   String alertStr;

   voltage = lipo.getVoltage();
 	charge = lipo.getSOC();
 	alert = lipo.getAlert();

   if (alert && threshold > 20) {
     lipo.clearAlert();
   }

   voltageStr = String(voltage, 2);
   chargeStr = String(charge, 2);
   alertStr = String(alert);

   payload = "{ \"a\":" + alertStr + ", \"c\":"
     + chargeStr + ", \"v\":" + voltageStr + " }";

   Spark.publish("BatteryState", payload);
 }
