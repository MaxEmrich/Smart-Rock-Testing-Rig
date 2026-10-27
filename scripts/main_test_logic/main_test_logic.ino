/**
 * In lab use case example for the SmartRock project
 * IN DEVELOPMENT CODE - MAY BE UNSTABLE
 * This project uses a hypnos, an ADS1115 and a MS5803
 * 
 * MANAGER MUST BE INCLUDED FIRST IN ALL CODE
 */


// ---- INLCUDED LIBRARIES, MOSTLY OPENS/LOOM ----
//============================================================
#include <Loom_Manager.h>
#include <Hardware/Loom_Hypnos/Loom_Hypnos.h>
#include <Sensors/I2C/Loom_ADS1115/Loom_ADS1115.h>
#include <Sensors/I2C/Loom_MS5803/Loom_MS5803.h>
#include <Sensors/I2C/Loom_VCNL4030/Loom_VCNL4030.h>
#include <Sensors/Loom_Analog/Loom_Analog.h>
#include <Wire.h>
#include <Adafruit_SleepyDog.h>
#include <RTClib.h>
//============================================================



// ---- ERROR HANDLING ----
//============================================================
enum Status {
  OK,
  ERR_TIMEOUT,
  ERR_NO_SENSOR,
  ERR_CALIBRATION
};
//============================================================



// ---- SPECS, CONSTS, & REQUIRED MIN/MAX VALUES ----
//============================================================
/* Memory Addressing of I2C Devices: 
  MS5803: 
  - 0x77 if CSB pin is tied high 
  - 0x76 if CSB pin is tied low -- our CSB pin is pulled low on the schematic, use this add

  VCNL4030:
  - 0x60 for the 4030x01-GSO8 Model --- this is our current model
  - 0x41 for the 40301x01-GSO8 Model 
  - 0x40 for the 40302x01-GSO8 Model
  - 0x41 for the 40303x01-GSO8 Model

  DFROBOT O2 Sensor:
  - Set by dip switch on the board:
    - A1  A0
    - 0   0: 0x70
    - 0   1: 0x71
    - 1   0: 0x72
    - 1   1: 0x73 (default value)
*/

  // minimum acceptable battery voltage for the Smart Rock
float vmin_bat = 4.3; // this value should be updated -- ask Tyler 

int sensorInitCountdown = 5000; // countdown is in milliseconds
int powerInitCountdown = 1000; // countdown is in milliseconds

float EC_slope = 0.00;  // EC calibration slope 
float EC_intercept = 0.00;  // EC calibration y-intercept 
float Turbidity_slope = 0.1574;  // Turbidity calibration slope
float Turbidity_intercept = -496.06 - 40;  // Turbidity calibration y-intercept

const uint8_t ms5803_addr = 0x76;
const uint8_t vcnl4030_addr = 0x60;
const uint8_t dfrobotO2_addr = 0x73;

const uint8_t vcnl4030_ID_reg = 0x0E;
const uint8_t ms5803_PROM_reg = 0xAE;

RTC_DS3231 rtc;
//============================================================


// ---- LOOM MANAGER, HYPNOS, & SENSOR CONSTRUCTORS ----
//============================================================
  //OPEnS Loom Constructors
Manager manager("Data", 1);
Loom_Analog analog(manager);

  // OPEnS Hypnos Constructors
Loom_Hypnos hypnos(manager, HYPNOS_VERSION::V3_3, TIME_ZONE::PST);
TimeSpan sleepInterval;

  // Sensor Module Constructors from Loom
Loom_ADS1115 ads(manager);
Loom_MS5803 ms(manager, 118);
Loom_VCNL4030 vcnl(manager);
//============================================================


// ---- CHECKING THE STATE OF THE POWER RAIL ENABLE PINS ----
//====================================================================================
// Look at the state of pins 5 and 6 to determine if rails are enabled
bool is3VEnabled() { return digitalRead(5) == LOW; }
bool is5VEnabled() { return digitalRead(6) == HIGH; }

void printRailState() {
  Serial.print("3.3V rail: "); Serial.println(is3VEnabled() ? "ON" : "OFF");
  Serial.print("5V rail: ");   Serial.println(is5VEnabled()  ? "ON" : "OFF");
  if (is3VEnabled() && is5VEnabled()) {
    Serial.println("Digital pins 5 and 6 are set to ON...");
  }
}
//====================================================================================



// ---- TAKE DATA, SENSOR CALIBRATION & TESTING ----
//====================================================================================


// Create a 7-bit address buffer and ask for ACK from memory at addr
bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;   // 0 = ACK, 2 = no ACK on address, 4 = bus error
}

// 
bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;  // repeated start -- returning 0 means the sensor acknowledged 
  if (Wire.requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read(); // look at each byte on the current register 
  }

  return true;
}

void checkSensors() {
  uint8_t b[2]; // this buffer is for reading a single 16-bit register and splitting it into 2 registers -- bc i2c only moves 1 byte at a time

  // ADS1115: read config register (0x01)
  if (!i2cPing(0x48)) Serial.println("ADS1115: NOT FOUND");
  else if (readRegs(0x48, 0x01, b, 2)) {
    Serial.print("ADS1115 config: 0x"); Serial.println((b[0] << 8) | b[1], HEX);
    // Power-on default is 0x8583. Bit 15 = 1 means idle (not mid-conversion).
  }

  // VCNL4030x01: product ID register (0x = command, 0x = ID)
  if (!i2cPing(vcnl4030_addr)) {
    Serial.println("VCNL4030X01: NOT FOUND");
  } else if (readRegs(vcnl4030_ADDR, vcnl4030_ID_reg, b, 2)) {
    Serial.print("VCNL ID low: 0x");  Serial.println(b[0], HEX);  // expect 80
    Serial.print("VCNL ID high: 0x"); Serial.println(b[1], HEX);  // version code
  } else {
    Serial.println("VCNL4030X01: ID read failed");
  }

  // MS5803: no status or ID register, so ping and check the calibration PROM (NOTE: MS5803 uses PROM not EEPROM, can't change this value)
  if (!i2cPing(ms5803_addr)) {
    Serial.println("MS5803: NOT FOUND");
  } else {
    Wire.beginTransmission(ms5803_addr); Wire.write(0x1E); Wire.endTransmission();
    delay(3);
    uint8_t b[2];
    if (!readRegs(ms5803_addr, 0xA2, b, 2)) { // PROM coefficient C1, asks to get PROM ready for read
      Serial.println("MS5803: read failed");
    } else {
      uint16_t c1 = (b[0] << 8) | b[1]; // combine 2 bytes back into 1 
      Serial.println((c1 == 0 || c1 == 0xFFFF) ? "MS5803: PROM bad" : "MS5803: OK");
    }
  }

  uint8_t d[3];
  if (!i2cPing(dfrobotO2_addr)) {
    Serial.println("O2 sensor: NOT FOUND");
    // read directly on register BEFORE calling DFROBOT's code (note that I'm doing this bc DFROBOT's code is somewhat poorly documented, and DFROBOT has no ID register to check)
  } else if (readRegs(dfrobotO2_addr, 0x03, d, 3)) {
    // d[0] = integer part, d[1] = tenths, d[2] = hundredths (IMPORTANT NOTE: raw, NOT yet calibrated)
    Serial.print("O2 raw: ");
    Serial.print(d[0]); Serial.print(" ");
    Serial.print(d[1]); Serial.print(" ");
    Serial.println(d[2]);
  }
}  

  // Smart Rock function to measure, package, calibrate, and display data 
  // m: calibration slopes (EC and Turbidity)
  // b: calibration y-intercepts (EC and Turbidity)
void take_data(float ECm, float ECb, float Tm, float Tb, bool troubleshooting_mode){

    // Troubleshooting print 2
  if(troubleshooting_mode==1){Serial.println("Begin Taking Data");}

    // Measure and package the data from the Loom modules into JSON packet
  manager.measure();
  manager.package();

    // Troubleshooting print 3
  if(troubleshooting_mode==1){Serial.println("Measured Data");}
//===========================
  // Applying Calibration Equations
  
    // This equation converts the measured conductance into EC in uS/cm
  float EC = ((ads.getAnalog(2)/ads.getAnalog(1)) * ECm + ECb); // ECm and ECb are the passed in slope and y-intercept from the beginning of void loop()
    // This equation converts the measured infrared backscatter (proximity) into Turbidity in NTU
  float Turb = ((vcnl.readProximity()) * Tm + Tb); // Tm and Tb are the passed in slope and y-intercept from the beginning of void loop()
    // Troubleshooting print 4
  if(troubleshooting_mode==1){Serial.println("EC and Turbidity Values Calculated");}
 //===========================
  
    // Adds data under "Sensor Values" to the JSON packet
  manager.addData("MS5803", "Pressure", ms.getPressure());          // MS5803 Pressure
  manager.addData("MS5803", "Temperature", ms.getTemperature());    // MS5803 Temperature
  manager.addData("VCNL4030x01","Ambient Light", vcnl.readAmbient());  // VCNL4030x01 Light
  manager.addData("VCNL4030x01","Proximity", vcnl.readProximity());    // VCNL4030x01 Proximity
  manager.addData("Analog Values","Conductivity", EC);              // Calibrated EC in uS/cm
  manager.addData("Analog Values","Turbidity", Turb);              //Calibrated Turbidity
  
  

    // Troubleshooting print 5
  if(troubleshooting_mode==1){Serial.println("Data Added to Packet");}

    // Print the current JSON packet to Serial Monitor and log to SD card
  manager.display_data();                   
  hypnos.logToSD();
  
    // Troubleshooting print 6
  if(troubleshooting_mode==1){Serial.println("Packet Logged to SD Card");}
}
//====================================================================================




  // Called when the interrupt is triggered 
void isrTrigger();

  // Measures and packages sensor readings which are displayed to the serial monitor and sent to the SD card
void take_data(float, float, float, float, bool);

//============================================================


void setup() {
  
    // Wait 20 seconds for the serial console to open
  manager.beginSerial();

  Serial.println("---Begin Test Script---");

  /* 
    Enable & test the hypnos rails
    Pin 5 controls 3.3V rail
    Pin 6 controls 5V rail
  */


  hypnos.enable();
  hypnos.set_custom_time();
  printRailState();
  hypnos.checkVoltage(3.7);
  uint32_t readVoltageFeather3V = analogRead(A7) * 2.0 * 3.3 / 1024.0;
  uint32_t readVoltageFeather5V = analogRead(A7) * 2.0 * 3.3 / 1024.0;
  Serial.println("FeatherM0 reads 3.3V rail voltage from A7 pin as: "); Serial.print(readVoltageFeather3V); 
  Serial.println("FeatherM0 reads 5V rail voltage from A7 pin as: "); Serial.print(readVoltageFeather5V); 


  manager.initialize();



    // Gets sleep interval from SD card
  sleepInterval = hypnos.getConfigFromSD("SD_config.json");
    // Register the ISR and attach to the interrupt
  hypnos.registerInterrupt(isrTrigger);

}

void loop() {
  // Check VCNL4030x01:
  checkSensors();
  delay(3000);
}

void isrTrigger(){
  hypnos.wakeup();
}


