/*
 *  This code is working for the OTT and ALS sensors as modified and tested by Rob 25/02/2025
 *
 *  Main branch Definitely the main branch weirs
 *  Kai James kaicjames@outlook.com
 *
 *  TO ADD A NEW SENSOR
 *  - Modify updateConfig() function
 *  - Modify config file
 *  - Add library and necessary definitions
 *  - Add updateNewSensor function. Place in loop()
 *  - Add output to buildDataStrings()
 *  - Modify header file to include new sensor
 */


//General libraries
#include "RTClib.h"
#include "Wire.h"
#include "SdFat.h"
#include "RTCZero.h"
#include "RH_RF95.h"
//Sensor libraries
#include <SDI12.h>
#include "ADS1X15.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "SparkFunBME280.h"
#include "SparkFun_TMP117.h"
#include <Adafruit_MAX31865.h>
#include <QuickMedianLib.h>

// #include <FlashAsEEPROM.h> Not using this right now

#define TEST_LOOP             (false)     //Run test loop instead of actual loop
#define SLEEP_ENABLED         (false)    //Disable to keep serial coms alive for testing
#define FIRMWARE_VERSION      (107)
#define BV_OFFSET             (0.01)
#define OTT_OFFSET            (0.15)
#define RTC_OFFSET_S          (12)
#define WRAP_AROUND_S_LOWER   (11)
#define WRAP_AROUND_S_UPPER   (49)
#define MAX_LOG_SIZE_BYTES    (1800000000)  //1.8GB. Max for some versions of FAT16
#define ALS_AVE_COUNT         (11)  //Number of ADC reads for ALS averaging
#define ALS_AVE_DELAY         (0) //Period (mS) between ADC reads for ALS averaging. ALS takes around 10mS to poll regardless.
#define TEMP_INIT_TIME        (100)
#define MAX_ALS_CAL_POINTS    (12)    //We store the array length in the last value ie array[11]
#define MAX_REPEATED_NODES    (10)
#define USB_UPDATE_WAIT       (200)   //in mS
#define MINIMUM_POLL_PERIOD   (10)    //Given in seconds, probably needs to become a function off adc wake time
//Default turbidity sensor settings (overridable in CFG_<FW>.txt)
#define DEFAULT_PUMP_TIME 10        //forward pump time (s) before wet read, draws water into sensor
#define DEFAULT_BACKFLUSH 10        //reverse pump time (s) after wet read, backflushes sensor
#define DEFAULT_READ_COUNT 10       //turbidity samples per reading for median
#define DEFAULT_MIN_H2O 20          //min water level (mm) to allow a wet turbidity read
#define DEFAULT_TURB_READ_PERIOD 100 //period (ms) between turbidity samples
//Hardware pins
#define FET_POWER             (4)
#define ONE_WIRE_POWER        (2)        //One wire temp sensors
#define ONE_WIRE_BUS          (5)        //One wire temp sensors
#define SDI12_DATA_PIN              (9)          // The pin of the SDI-12 data bus
#define SD_SPI_CS             (A4)
#define USS_ECHO              (A2)
#define USS_TRIG              (A3)
#define USS_INIT_TIME         (50)
#define LED                   (13)
#define BATT                  (A5)
#define TURBIDITY_MOTOR_FORWARD (A0)    //Turbidity pump forward. NOTE: A0 is also the PT100 MAX31865 CS - turbidity and PT100 are mutually exclusive per node
#define TURBIDITY_MOTOR_REVERSE_PIN (A1) //Turbidity pump reverse
#define TURB_PUMP_SPEED           (3)   //Turbidity pump speed PWM pin
#define TURB_DRY_READ    "TURBDRY.csv" //8 char FAT limit
#define TURB_WET_READ   "TURBRAW.csv" //8 char FAT limit
//A1 and D2 are used for pulse counters 1 (32bit) and 2 (16bit) respectively

//I2C Addressing
// #define ADC_ADDR              (0x49) //WERG address is 0x48, easy to switch
#define ADC_ADDR              (0x48)
#define TMP_ADDR              (0x4A)
#define BME280_ADDR           (0x77)
#define RTC_ADDR              (0x68)  //Set automatically, here as a reminder
//Sensor wake times
#define SDI12_WAKE_TIME     (3000)//CHANGED FROM 1000
#define TMP117_WAKE         (200)
#define BME280_WAKE         (40)
//Other
#define WDT_TIMEOUT           (4)   //Watchdog timeout given by 2^value. Min = 0 (1sec), max = 8 (4min)
#define VEGE_SM_SCALE   (0.65)
//RTD
#define RREF      430.0
#define RNOMINAL  100.0
//Time
#define SECS_PER_DAY 86400
#define SECS_PER_HOUR 3600
#define SECS_PER_MIN 60
#define LEAP_YEAR(Y)     ( ((1970+(Y))>0) && !((1970+(Y))%4) && ( ((1970+(Y))%100) || !((1970+(Y))%400) ) )



uint8_t arrayRead(String valueString, String valueArray[]);
uint8_t arrayRead(String valueString, float valueArray[]);
uint8_t arrayRead(String valueString, int valueArray[]);
void pt100Cal(int adcChannel);

const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

typedef struct {
  uint8_t aveCount = 1;
  uint16_t aveDelay = 10;
  float calTemp[MAX_ALS_CAL_POINTS] = {};
  float mvMin[MAX_ALS_CAL_POINTS] = {0};
  float valMin[MAX_ALS_CAL_POINTS] = {0};
  float mvMax[MAX_ALS_CAL_POINTS] = {5000};
  float valMax[MAX_ALS_CAL_POINTS] = {100};
  double slope[MAX_ALS_CAL_POINTS] = {1};
  double offset[MAX_ALS_CAL_POINTS] = {0};
  uint8_t tempPoints = 0;
  uint8_t calPoints = 0;
  String id = "DF";
} ADC_t;

typedef struct {
  //System config________________
  String siteID = "AA";
  String NodeID = "DFLT";
  float battLowerLimit = 3.5;
  int pollOffset = 23;
  int LoRaBandwidth = 125000;
  float LoRaFrequency = 919.9;   //was 921.2;
  int pollPerLoRa = 6;
  bool LoRaEnabled = true;
  bool SDEnabled = true;
  int analogWakeTime = 5000; //ms was 1000
  int timezone = 10;
  //Special system functions____________
  int LoRaRepeater = 0;
  String Nodes_to_repeat[MAX_REPEATED_NODES] = {"NULL"};
  //Serial sensor config_________________________
  float pulse1_mult = 1;
  float pulse2_mult = 1;
  //Turbidity sensor config__________________
  int turbPumpTime = DEFAULT_PUMP_TIME;
  int turbBackflush = DEFAULT_BACKFLUSH;
  int turbDryReads = DEFAULT_READ_COUNT;
  int turbWetReads = DEFAULT_READ_COUNT;
  int minh2o = DEFAULT_MIN_H2O;
  int turbPeriod = DEFAULT_TURB_READ_PERIOD;
  //Analog sensor config__________________
  ADC_t adc[4];
} cfg_t;
cfg_t cfg;

typedef struct {
  uint8_t sensorCount = 0;
  double measure[4] = {0, 0, 0, 0};
  double measure_2[4] = {0, 0, 0, 0};
  DeviceAddress addr[4];
} value_t;
value_t value;

typedef struct {
  value_t pulse1;
  value_t pulse2;
  value_t temp;
  value_t RTCTemp;
  value_t ALS;
  value_t OTT;
  value_t USS;
  value_t TMP117;
  value_t BME280;
  value_t adc[4];
  value_t PT100;
  value_t turbidity;
} sensor_t;
sensor_t sensor;

typedef struct {
  String packet;
  int RSSI;
} LoRaReceive_t;
LoRaReceive_t LoRaReceive;

//Initialise board stuff
RH_RF95 rf95(12, 6);
uint8_t rfbuf[RH_RF95_MAX_MESSAGE_LEN];
RTC_DS3231 rtc;
RTCZero internalrtc;

//Initialise sensors
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);
DeviceAddress Thermometer;
DeviceAddress hello;
Adafruit_MAX31865 thermo = Adafruit_MAX31865(A0); //NOTE: A0 is also the Turb forward motor - turbidity and PT100 are mutually exclusive per node

BME280 bme280;
TMP117 tempSensor;
SDI12 mySDI12(SDI12_DATA_PIN);
ADS1115 ads(ADC_ADDR);
// ADS1015 ads(ADC_ADDR);

SdFs SD;
typedef FsFile file_t;
file_t file;
file_t debugFile;

uint8_t wdtTime1 = 11;    //Valid values: 0-11. 11 gives 16s timeout. 10 gives 8s timeout and so on
uint8_t wdtTime2 = 5;    //4=16s, gendiv 5=32s,6=1 min, 7=2min, 8= 4min
int pollPeriod = 60;
File logFile;
File configFile;
String UNIXtimestamp;
String normTimestamp;
String fileNameStr;
String LoRaDataString;
String CSVDataString;
int sleep_now_time;
int sleep_remaining_s = 0;
uint8_t tx_count = 0;
char addr[5];
char hex_chars[] = "0123456789ABCDEF";
int Year;
bool setupLoop = true;
int logFileSize = 0;
int loraPollCount = 999;  //Poll on first reading
String CSVHeader;
int logIncrement = 97;  //ASCII lowercase a
int lastUpTime = 0;
int WakeTime = 0;
int SensWakeTime = 0;
bool dailyReset = false;
bool dailyReset2 = false;
volatile uint16_t tip_count = 0;
uint8_t tempCalCount;
uint32_t unixtime = 0;      //32 bit will overflow in 2106. Not my problem.
uint32_t nextUnixWake = 0;
static  const uint8_t monthDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; // API starts months from 1, this array starts from 0
bool USBConnected = 0;
//Turbidity measurement buffers and results
float turbVoltageMeasurementArray[10];
float turbHousingMeasurementArray[10]; //housing temp sensor
float turbWaterMeasurementArray[10];   //water temp sensor
float turbVoltageMedian;
float turbHousingMedian;
float turbidity;          //wet median (mV)
float turbidity_air_avg;  //dry median (mV)

char SDbuf[125];

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////    SETUP           ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup () {
  //Output pin setup
  pinMode(FET_POWER, OUTPUT);
  digitalWrite(FET_POWER, HIGH);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  analogReference(AR_INTERNAL2V23); //For internal battery level calculation

  //Grab node ID
  hexstr(getChipId(), addr, sizeof(addr));  //Establish default node ID. Overwrite if specified in config
  cfg.NodeID = String(addr);

  setupWDT( wdtTime1 ); //Temporary watchdog pre-config read

  //Loose stuff
  sensor.RTCTemp.sensorCount = 1;
  Wire.begin();

  //Auto detect sensors
  OneWireTempSetup(); //Must auto detect sensors BEFORE configRead()
  TMP117Setup();  //Auto detect tmp117

  //SD card setup
  if (!SD.begin(SD_SPI_CS)) {
    crashNflash(5);  //10*10ms high period
  }
  disableWDT();

  configRead();
  updateADCEquations();

  //Turbidity pump pins - only claimed on nodes that actually have the turbidity rig.
  //A0/A1 are otherwise free (A0 is the PT100 CS when a PT100 is fitted), so we
  //only configure them when a turbidity node is configured to avoid breaking other sensors.
  if (sensor.turbidity.sensorCount > 0) {
    pinMode(TURBIDITY_MOTOR_FORWARD, OUTPUT);
    pinMode(TURBIDITY_MOTOR_REVERSE_PIN, OUTPUT);
    pinMode(TURB_PUMP_SPEED, OUTPUT);
    digitalWrite(TURBIDITY_MOTOR_FORWARD, LOW);
    digitalWrite(TURBIDITY_MOTOR_REVERSE_PIN, LOW);
    analogWrite(TURB_PUMP_SPEED, 0);
  }

  //Create debug file if doesn't exist
  if (!SD.exists("debug.txt")) {
    file.open("debug.txt", O_RDWR | O_CREAT);
    file.sync();
    file.close();
  }

  //LoRa setup
  if (rf95.init() == false) {
    while (1) {
      delay(50);
      if (rf95.init()) {
        break;
      }
    }
  } else {
    rf95.setTxPower(23, false);
    rf95.setFrequency(cfg.LoRaFrequency);
    rf95.setSignalBandwidth(cfg.LoRaBandwidth); //500kHz
  }
  debug("LoRa started");

  //Special system functions
  if (cfg.LoRaRepeater) {
    LoRaRepeaterLoop();
  }

  //Setup watchdog
  wdtTimeoutCalc();   //After SD setup and LoRaRepeater
  setupWDT( wdtTime1 );

  //Ultra sonic setup
  pinMode(USS_TRIG, OUTPUT);
  pinMode(USS_ECHO, INPUT);
  digitalWrite(USS_TRIG, LOW);

  //OTT probe setup
  if (sensor.OTT.sensorCount > 0) {
    mySDI12.begin();
  }

  if (sensor.PT100.sensorCount > 0) {
    thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  }

  //BME280 setup
  if (sensor.BME280.sensorCount != 0 ) {
    bme280Setup();
  }

  //ADC setup
  debug("About to start ADC");
  ads.begin();
  ads.setGain(0);  // 6.144 volt


  //CSV setup
  String currHeader = "blank";
  generateCSVHeader();
  fileNameStr = fileNameGen(logIncrement);

  while (SD.exists((char*)fileNameStr.c_str()) && logIncrement < 122) { //ASCII lowercase chars for logIncrement
    logIncrement++;
    fileNameStr = fileNameGen(logIncrement);
  }
  logIncrement--;
  if (logIncrement == 96) { //If creating the first file
    // currHeader = "notEqual";
  }
  else {  //If NOT creating the first file
    fileNameStr = fileNameGen(logIncrement);
    file.open((char*)fileNameStr.c_str(), O_RDWR | O_CREAT);
    currHeader = file.readStringUntil('\n');
    logFileSize = file.fileSize();
    file.sync();
    file.close();
  }

  currHeader.trim();
  CSVHeader.trim();

  if ((currHeader != CSVHeader) ) {
    CSVHeader += "\n";
    logIncrement++;
    fileNameStr = fileNameGen(logIncrement);
    file.open((char*)fileNameStr.c_str(), O_RDWR | O_CREAT | O_APPEND);
    file.write((char*)CSVHeader.c_str());
    logFile.sync();
    logFile.close();
  }

  //RTC
  debug("About to start RTC");
  if (! rtc.begin()) {
    crashNflash(2);
  }
  DateTime now = rtc.now();
  unixtime = now.unixtime();

  //Grab upload time
  int Hour;
  int Min;
  int Sec;
  int Day;
  char Month[12];
  uint8_t monthIndex;
  sscanf(__TIME__, "%d:%d:%d", &Hour, &Min, &Sec);
  sscanf(__DATE__, "%s %d %d", Month, &Day, &Year);
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  uint32_t uploadUnixtime = makeTime(Sec, Min, Hour, Day, monthIndex + 1, Year - 1970);
  uploadUnixtime -= 60 * 60 * cfg.timezone; //Set to universal time

  //Sync RTC on first code upload
  int RTCTime = now.unixtime();
  //if (uploadUnixtime > RTCTime) {
  // rtc.adjust(DateTime(Year, monthIndex + 1, Day, Hour, Min, Sec));
  rtc.adjust(uploadUnixtime);
  //}

  if (cfg.pollOffset > pollPeriod * cfg.pollPerLoRa) { //Protection for badly-set poll offsets
    cfg.pollOffset = 1;
  }

  internalrtc.begin(false); //RTC for
  internalrtc.attachInterrupt(wake_from_sleep);

  // TO BECOME USB UPDATE FUNCTION
  // ///////////////////////////////////////////////////
  delay(3000);
  disableWDT();
  int timeoutStart = millis();
  while (SerialUSB.available() || millis() - timeoutStart < USB_UPDATE_WAIT) {
    // SerialUSB.println("Hello");
    debug(String("Checking for USB connection"));
    delay(500);
    if (SerialUSB.available()) {
      debug(String("Connected for USB config"));
      debug(String(SerialUSB.readStringUntil('\n')));
    }
  }
  //  debug("Going to sleep to sync time");
  if (!TEST_LOOP) {
    sleepTillSynced();
  }
  else {
    disableWDT();
  }
  String tmpStr = String("Node ") + cfg.NodeID + " ready for work. Time offset " + String(cfg.pollOffset, DEC) + " seconds";
  debug(tmpStr); //Quick message to say we've woken up


  //Run after sleep sync to avoid bad interupts
  if (sensor.pulse1.sensorCount > 0) {
    pulse1Setup();
  }
  if (sensor.pulse2.sensorCount > 0) {
    pulse2Setup();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////      LOOP           ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop () {
  resetWDT();
  while (TEST_LOOP) {
  }

  while (battVoltUpdate() <= 3.45) { //Power protection set to work for primary and secondary cells
    debug("Battery voltage too low. Going back to sleep");
    sleep_remaining_s = 300; //Sleep for 5 minutes
    sleep();
  }
  wakeSensors();    // Turn on 12V supply and ready TMP117 and BME280

  ////////////////////////////////////////////////
  //Sensor updates- Must be in order of wake time (Least to most)
  ///////////////////////////////////////////////

  if (sensor.pulse1.sensorCount > 0) {
    if (!pulse1Update()) {
    }
  }
  if (sensor.pulse2.sensorCount > 0) {
    if (!pulse2Update()) {
    }
  }
  if (sensor.RTCTemp.sensorCount > 0) {
    if (!RTCTempUpdate()) {
    }
  }
  if (sensor.temp.sensorCount > 0) {
    if (!tempUpdate()) {
    }
  }
  if (sensor.BME280.sensorCount > 0) {
    if (!BME280Update()) {
    }
  }
  if (sensor.TMP117.sensorCount > 0) {
    if (!TMP117Update()) {
    }
  }
  if (sensor.PT100.sensorCount > 0) {
    if (!PT100Update()) {
    }
  }
  tempCalCount = 0; //tempCalCount resets once per poll to ensure order of multiple ALS probes
  if (sensor.adc[0].sensorCount > 0) {
    if (!ADCUpdate(0)) {
    }
  }
  if (sensor.adc[1].sensorCount > 0) {
    if (!ADCUpdate(1)) {
    }
  }
  if (sensor.adc[2].sensorCount > 0) {
    if (!ADCUpdate(2)) {
    }
  }
  if (sensor.adc[3].sensorCount > 0) {
    if (!ADCUpdate(3)) {
    }
  }
  if (sensor.OTT.sensorCount > 0) {
    if (!OTTUpdate()) {
    }
  }
  if (sensor.USS.sensorCount > 0) {
    if (!USSUpdate()) {
    }
  }
  if (sensor.turbidity.sensorCount > 0) {
    bool enoughWater = true;
    if (sensor.adc[0].sensorCount > 0) {
      enoughWater = (sensor.adc[j].measure_2[0] >= cfg.minh2o); //Check logic. Use OTT or ALS for this?
    }
    if (enoughWater) {
      analogWrite(TURB_PUMP_SPEED, 255); //max 255
      TurbMeasure(TURB_DRY_READ, cfg.turbDryReads);
      turbidity_air_avg = turbVoltageMedian; //raw voltage, not NTU
      delayUsingMillis(10);
      digitalWrite(TURBIDITY_MOTOR_FORWARD, HIGH); //run pump forward
      forwardPump();
      TurbMeasure(TURB_WET_READ, cfg.turbWetReads);
      turbidity = turbVoltageMedian; //raw voltage, not NTU
      digitalWrite(TURBIDITY_MOTOR_FORWARD, LOW);
      delayUsingMillis(500);
      digitalWrite(TURBIDITY_MOTOR_REVERSE_PIN, HIGH); //reverse backflush
      reversePump();
      digitalWrite(TURBIDITY_MOTOR_REVERSE_PIN, LOW);
      delayUsingMillis(10);
      analogWrite(TURB_PUMP_SPEED, 0);
    } else {
      resetTurbidityMedians();
      debug(normTimestamp + "Not enough water, skipping turbidity measure\n");
    }
  }
  turnOff12V();
  buildTimestamps();
  buildDataStrings();

  if (cfg.SDEnabled) {
    logDataToSD();
  }

  if (cfg.LoRaEnabled) {
    if (loraPollCount >= cfg.pollPerLoRa) {
      loraPollCount = 0;
      LoRaUpdate();
    }
    loraPollCount++;
  }

  rf95.sleep();
  delay(20);    //Delay 20ms to ensure the chips have gone to sleep before powering off the board
  disableWDT(); // disable watchdog=
  bool first_loop = true;
  DateTime now = rtc.now();
  lastUpTime = millis() - WakeTime;
  nextUnixWake += pollPeriod;
  sleep_remaining_s = nextUnixWake - now.unixtime();
  sleep();

  WakeTime = millis();  //For wake period calculation
  setupWDT( wdtTime1 ); // initialize and activate WDT with maximum period
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////    FUNCTIONS         ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

String fileNameGen(int increment) {
  // String fileName = String(project) + String(cfg.NodeID) + String(char(increment)) + ".csv";
  String fileName = String("/") + String(cfg.siteID) + "_" + String(cfg.NodeID) + "_" + String(char(increment)) + ".csv";
  return fileName;
}

void sleep() {
  sleep_now_time = internalrtc.getEpoch();
  if (sleep_remaining_s > 0) {
    if (SLEEP_ENABLED) {
      internalrtc.setAlarmEpoch(sleep_now_time + (sleep_remaining_s - 1));
      internalrtc.enableAlarm(internalrtc.MATCH_HHMMSS);
      SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
      SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
      __DSB();
      __WFI();    //Wait for interrupt
      SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    }
    else if (!SLEEP_ENABLED) {
      delay(sleep_remaining_s * 1000);
    }
    if (USBConnected) {
      USBUpdate();
    }
    // sleep_remaining_s = sleep_remaining_s - (internalrtc.getEpoch() - sleep_now_time); // Restarts clock in case of wake due to external interurpt
  }
}

uint16_t getChipId() {
  volatile uint32_t *ptr = (volatile uint32_t *)0x0080A048;
  return *ptr;
}

void hexstr(uint16_t v, char *buf, size_t Size) {
  uint8_t i;
  if (Size > 4) {
    for (i = 0; i < 4; i++) {
      buf[3 - i] = hex_chars[v >> (i * 4) & 0x0f];
    }
    buf[4] = '\0';
  }
}

static void   WDTsync() {
  while (WDT->STATUS.bit.SYNCBUSY == 1); //Just wait till WDT is free
}

void setupWDT( uint8_t period) {
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(5) | GCLK_GENDIV_DIV(wdtTime2);  //gendiv 4=16s, gendiv 5=32s,6=1 min, 7=2min, 8= 4min
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(5) |
  GCLK_GENCTRL_GENEN |
  GCLK_GENCTRL_SRC_OSCULP32K |
  GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.bit.SYNCBUSY);  // Syncronize write to GENCTRL reg.
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT |
  GCLK_CLKCTRL_CLKEN |
  GCLK_CLKCTRL_GEN_GCLK5;
  WDT->CTRL.reg = 0; // disable watchdog
  WDTsync(); // sync is required
  WDT->CONFIG.reg = period; // see Table 17-5 Timeout Period (valid values 0-11)
  WDT->CTRL.reg = WDT_CTRL_ENABLE; //enable watchdog
  WDTsync();
}

void systemReset() {  // use the WDT watchdog timer to force a system reset.
  WDT->CLEAR.reg = 0x00; // system reset via WDT
  WDTsync();
}

void resetWDT() {
  WDT->CLEAR.reg = 0xA5; // reset the WDT
  WDTsync();
}

void wake_from_sleep() {  //ISR runs whenever system wakes up from RTC
}

void LoRaUpdate() {
  char *pmsg;
  String LoRaString;
  if ((int) (cfg.LoRaFrequency * 10.0) == 9212) { //Can't compare floats
    LoRaString = "SEW:" + normTimestamp + LoRaDataString;
  }
  else {
    LoRaString = "PKT:" + normTimestamp + LoRaDataString;
  }
  pmsg = (char*)LoRaString.c_str();
  rf95.send((uint8_t *)pmsg, strlen(pmsg) + 1);
  rf95.waitPacketSent();    //This takes 189ms
}

void sendLoRaRaw(String msg) {
  char *ppmsg;
  ppmsg = (char*)msg.c_str();
  rf95.send((uint8_t *)ppmsg, strlen(ppmsg) + 1);
  rf95.waitPacketSent();
}

void debug(String msg) {
  buildTimestamps();

  char *ppmsg;
  msg = String(cfg.NodeID) + "_DBG: " + msg;

  if (cfg.LoRaEnabled) {
    ppmsg = (char*)msg.c_str();
    rf95.send((uint8_t *)ppmsg, strlen(ppmsg) + 1);
    rf95.waitPacketSent();
  }
  if (cfg.SDEnabled) {
    FsDateTime::setCallback(dateTime);
    msg = normTimestamp + msg + "\n";
    ppmsg = (char*)msg.c_str();
    file.open("debug.txt", O_RDWR | O_APPEND);
    file.write(ppmsg);
    file.sync();
    file.close();
  }
}

float battVoltUpdate() {
  float BATT_LVL = analogRead(BATT);
  BATT_LVL = BATT_LVL / 1024 * 2.23 * 2 + BV_OFFSET;
  return BATT_LVL;
}

bool tempUpdate() { //Make sure battery power is connected.
  tempSensors.requestTemperatures();
  for (int i = 0;  i < sensor.temp.sensorCount;  i++) {
    sensor.temp.measure[i] = tempSensors.getTempC(sensor.temp.addr[i]);
  }
  return true;
}

bool OTTUpdate() {
  String myComAdress = "?!";
  String address;
  String myComId = "0I!";
  String myComSend = "0D0!";
  String myComMeasure = "0M!";

  int wait = SDI12_WAKE_TIME - (millis() - SensWakeTime);
  if (wait > 0 ) {
    delay(wait);
  }


  //     //Warmup test, prints device details
  //   mySDI12.sendCommand("?!");
  //   delay(300);                     // wait a while for a response
  //   while(mySDI12.available()){    // write the response to the screen
  //   SerialUSB.write(mySDI12.read());
  //   }


  mySDI12.sendCommand(myComMeasure);
  // delay(400);
  String requestDetails = readSDI12();    //Gives <1 character address><3 character time seconds><1 character # of values>
  requestDetails.remove(0,1); //Trim down to measurement time
  requestDetails.remove(3,1); //Trim down to measurement time
  uint16_t measurementTime = requestDetails.toInt();
  delay(measurementTime*1000 + 1500); //Delay for measurement time + 1.5 seconds

  mySDI12.clearBuffer();
  delay(200);
  mySDI12.sendCommand(myComSend);
  delay(50);
  String rawdata = readSDI12();
  //  debug("Raw data = " + rawdata);
  mySDI12.clearBuffer();
  delay(50);
  //Decoding string sent from probe
  int p = 0;
  int pos[] = {0, 0, 0, 0, 0, 0};  //We might get more values, we will sort them but ignore
  for (int z = 0 ; z < rawdata.length() ; z++)  {
    char u = rawdata.charAt(z);
    if (u == '+' || u == '-') {
      pos[p] = z ;
      p++;
    }
    delay (50);
  }
  String level = rawdata.substring(pos[0], pos[1]);
  String temp = rawdata.substring(pos[1], rawdata.length());
  sensor.OTT.measure[0] = level.toDouble();
  sensor.OTT.measure[1] = temp.toDouble();
  return true;
}

String readSDI12() {
  String sdiResponse = "";
  delay(30);
  while (mySDI12.available()) {  // This sometimes times out while still writing
    char c = mySDI12.read();
    // debug("c=" + String(c));
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
    }
    delay(10);
  }
  return sdiResponse;
}

bool RTCTempUpdate() {
  sensor.RTCTemp.measure[0] = rtc.getTemperature();
  return true;
}

bool USSUpdate() {
  digitalWrite(FET_POWER, LOW);
  delay(USS_INIT_TIME);
  digitalWrite(USS_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(USS_TRIG, HIGH);
  delayMicroseconds(20);
  digitalWrite(USS_TRIG, LOW);
  double duration = pulseIn(USS_ECHO, HIGH);
  float dist = duration / 2 * 0.000343;
  delay(50);
  sensor.USS.measure[0] = dist;   //Should return duration and have temp compensation
  digitalWrite(FET_POWER, LOW);
  return true;
}

void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) {
  DateTime now = rtc.now();

  // Return date using FS_DATE macro to format fields.
  *date = FS_DATE(now.year(), now.month(), now.day());

  // Return time using FS_TIME macro to format fields.
  *time = FS_TIME(now.hour(), now.minute(), now.second());

  // Return low time bits in units of 10 ms, 0 <= ms10 <= 199.
  *ms10 = now.second() & 1 ? 100 : 0;
}

void logDataToSD() {
  FsDateTime::setCallback(dateTime);

  file.open((char*)fileNameStr.c_str(), O_RDWR | O_APPEND);
  int logFileLast = file.fileSize();
  file.write((char*)normTimestamp.c_str());
  file.write((char*)CSVDataString.c_str());
  logFileSize = file.fileSize();
  file.sync();
  file.close();
  if (logFileSize == 0 || logFileSize <= logFileLast) {
    debug("Failed to write to log, restarting system");
    systemReset();
  }
  if (logFileSize > MAX_LOG_SIZE_BYTES) {
    debug("Logfilesize over max");
    logIncrement++;
    fileNameStr = fileNameGen(logIncrement);
    // logFile = SD.open((char*)fileNameStr.c_str(), FILE_WRITE);
    file.open((char*)fileNameStr.c_str(), O_RDWR | O_CREAT );
    file.write((char*)CSVHeader.c_str());
    // logFile.println(CSVHeader);
    // logFile.close();

    file.sync();
    file.close();

  }
}

void crashNflash(int identifier) {
  for (int j = 0; j < 5; j++) { //Loops 5 times before resetting
    for (int k = 0; k < identifier; k++) {  //Set number of flashes
      digitalWrite(LED, HIGH);
      delay(50);
      digitalWrite(LED, LOW);
      delay(300);
    }
    delay(2000);
  }
  systemReset();
}

void buildTimestamps() {
  String day;
  String month;
  String hour;
  String minute;
  String second;
  DateTime now = DateTime(nextUnixWake);

  if (now.month() < 10) {
    month = "0" + String(now.month(), DEC);
  } else {
    month = String(now.month(), DEC);
  }
  if (now.day() < 10) {
    day = "0" + String(now.day(), DEC);
  } else {
    day = String(now.day(), DEC);
  }
  if (now.hour() < 10) {
    hour = "0" + String(now.hour(), DEC);
  } else {
    hour = String(now.hour(), DEC);
  }
  if (now.minute() < 10) {
    minute = "0" + String(now.minute(), DEC);
  } else {
    minute = String(now.minute(), DEC);
  }
  if (now.second() < 10) {
    second = "0" + String(now.second(), DEC);
  } else {
    second = String(now.second(), DEC);
  }
  normTimestamp = day + "/" + month + "/" + String(now.year(), DEC) + " " + hour + ":" + minute + ":" + second + ",";
  UNIXtimestamp = String(now.unixtime());
  // debug(String(UNIXtimestamp));
}

void buildDataStrings() {
  LoRaDataString = cfg.siteID + ",";   //First "SITE_ID" identifies packet to relevant gateway
  LoRaDataString += cfg.NodeID + ",";
  LoRaDataString += String(FIRMWARE_VERSION) + ",";
  LoRaDataString += String(tx_count++, DEC) + ",";
  LoRaDataString += String(lastUpTime) + ",";
  LoRaDataString += String(logFileSize) + ",";
  LoRaDataString += String(battVoltUpdate()) + ",";
  CSVDataString = LoRaDataString;

  if (sensor.pulse1.sensorCount > 0) {
    // LoRaDataString +=  "PA1=" + String(sensor.pulse1.measure[0], 2) + ",";    //Raw
    LoRaDataString +=  "PB1=" + String(sensor.pulse1.measure_2[0], 2) + ",";  //Calibrated
    LoRaDataString +=  "PC1=" + String(sensor.pulse1.measure_2[1], 2) + ",";  //Calibrated daily cumulative
    // CSVDataString += String(sensor.pulse1.measure[0], 2) + ",";      //Raw
    CSVDataString += String(sensor.pulse1.measure_2[0], 2) + ",";
    CSVDataString += String(sensor.pulse1.measure_2[1], 2) + ",";
  }
  if (sensor.pulse2.sensorCount > 0) {
    // LoRaDataString +=  "PA2=" + String(sensor.pulse2.measure[0], 2) + ",";   //Raw
    LoRaDataString +=  "PB2=" + String(sensor.pulse2.measure_2[0], 2) + ",";    //Calibrated
    LoRaDataString +=  "PC2=" + String(sensor.pulse2.measure_2[1], 2) + ",";    //Calibrated daily cumulative
    // CSVDataString += String(sensor.pulse2.measure[0], 2) + ",";
    CSVDataString += String(sensor.pulse2.measure_2[0], 2) + ",";
    CSVDataString += String(sensor.pulse2.measure_2[1], 2) + ",";
  }
  if (sensor.RTCTemp.sensorCount > 0) {
    for (int i = 0; i < sensor.RTCTemp.sensorCount; i++) {
      LoRaDataString += "" + String(sensor.RTCTemp.measure[i], 2) + ",";
      CSVDataString += String(sensor.RTCTemp.measure[i], 2) + ",";
    }
  }
  if (sensor.temp.sensorCount > 0) {
    for (int i = 0; i < sensor.temp.sensorCount; i++) {
      LoRaDataString += "TO" + String(i) + "=" + String(sensor.temp.measure[i], 2) + ",";
      CSVDataString += String(sensor.temp.measure[i], 2) + ",";
    }
  }
  if (sensor.TMP117.sensorCount > 0) {
    LoRaDataString += "TT1=" + String(sensor.TMP117.measure[0], 2) + ",";
    CSVDataString += String(sensor.TMP117.measure[0], 2) + ",";
  }
  if (sensor.OTT.sensorCount > 0) {
    for (int i = 0; i < sensor.OTT.sensorCount; i++) {
      LoRaDataString += "" + String(sensor.OTT.measure[0], 3) + ",";
      LoRaDataString += "" + String(sensor.OTT.measure[1], 2) + ",";
      CSVDataString += String(sensor.OTT.measure[0], 3) + ",";        //changed from CSVDataString += String(sensor.OTT.measure[0], 2) + ",";
      CSVDataString += String(sensor.OTT.measure[1], 2) + ",";
    }
  }
  if (sensor.USS.sensorCount > 0) {
    for (int i = 0; i < sensor.USS.sensorCount; i++) {
      LoRaDataString += "US1" + String(sensor.USS.measure[i], 2) + ",";
      CSVDataString += String(sensor.USS.measure[i], 2) + ",";
    }
  }
  if (sensor.BME280.sensorCount > 0) {
    LoRaDataString += "HB1=" + String(sensor.BME280.measure[0], 3) + ",";
    CSVDataString += String(sensor.BME280.measure[0], 3) + ",";
    LoRaDataString += "QB1=" + String(sensor.BME280.measure_2[0], 3) + ",";
    CSVDataString += String(sensor.BME280.measure_2[0], 3) + ",";
  }
  if (sensor.PT100.sensorCount > 0) {
    LoRaDataString += "TP1=" + String(sensor.PT100.measure[0], 3) + ",";
    CSVDataString += String(sensor.PT100.measure[0], 3) + ",";
  }
  for (int j = 0; j < 4 ; j++) {
    if (sensor.adc[j].sensorCount > 0) {
      // LoRaDataString += "AR" + String(j) + "=" + String(sensor.adc[j].measure[0], 2) + ",";
      LoRaDataString += "A_" + String(cfg.adc[j].id) + "=" + String(sensor.adc[j].measure_2[0], 2) + ",";
      // CSVDataString += String(sensor.adc[j].measure[0], 2) + ",";    //Raw
      CSVDataString += String(sensor.adc[j].measure_2[0], 2) + ",";     //Calibrated
    }
  }
  if (sensor.turbidity.sensorCount > 0) {
    LoRaDataString += "TURB_DRY=" + String(turbidity_air_avg, 2) + ",";
    LoRaDataString += "TURB_WET=" + String(turbidity, 2) + ",";
    CSVDataString += String(turbidity_air_avg, 2) + ",";
    CSVDataString += String(turbidity, 2) + ",";
    CSVDataString += String(turbHousingMedian, 2) + ",";
  }
  CSVDataString += "\n";
}

void configRead() {
  String configFileName = "CFG_" + String(FIRMWARE_VERSION) + ".txt";
  String key;
  String value;
  String line;
  char * pch;
  if (!SD.exists(configFileName)) {
    crashNflash(3);
  }
  configFile = SD.open((char*)configFileName.c_str());
  while (configFile.available()) {
    line = configFile.readStringUntil('\n');
    key = strtok((char*)line.c_str(), "=");
    value = strtok(NULL, ";");
    if (value != "DEFAULT" && value != NULL) {
      key.trim();
      value.trim();

      //System config______________________________
      if (key == "Site_ID") {
        cfg.siteID = value;
      }
      else if (key == "Node_ID") {
        cfg.NodeID = value;
      }
      else if (key == "pollOffset") {
        cfg.pollOffset = value.toInt();
      }
      else if (key == "LoRaBandwidth") {
        cfg.LoRaBandwidth = value.toInt();
      }
      else if (key == "LoRaFrequency") {
        cfg.LoRaFrequency = value.toFloat();
      }
      else if (key == "LoRaReportFreq") {
        cfg.pollPerLoRa = value.toInt();
      }
      else if (key == "pollPeriod") {
        pollPeriod = value.toInt();
        if (pollPeriod < MINIMUM_POLL_PERIOD) {
          pollPeriod = MINIMUM_POLL_PERIOD;
        }
      }
      else if (key == "LoRa_EN") {
        cfg.LoRaEnabled = value.toInt();
      }
      else if (key == "SD_EN") {
        cfg.SDEnabled = value.toInt();
      }
      else if (key == "Analog_Wake") {
        cfg.analogWakeTime = value.toInt();
      }
      else if (key == "Timezone") {
        cfg.timezone = value.toInt();
      }
      //Special system functions________________________
      else if (key == "LoRaRepeater") {
        cfg.LoRaRepeater = value.toInt();
      }
      else if (key == "Nodes_to_repeat") {
        arrayRead(value, cfg.Nodes_to_repeat);
      }
      //Serial sensor config____________________________
      else if (key == "Pulse1_Attached") {
        sensor.pulse1.sensorCount = value.toInt();
      }
      else if (key == "Pulse1_Multiplier") {
        cfg.pulse1_mult = value.toFloat();
      }
      else if (key == "Pulse2_Attached") {
        sensor.pulse2.sensorCount = value.toInt();
      }
      else if (key == "Pulse2_Multiplier") {
        cfg.pulse2_mult = value.toFloat();
      }
      else if (key == "RTC_Temp_Count") {
        sensor.RTCTemp.sensorCount = value.toInt();
      }
      else if (key == "OTT_Count") {
        sensor.OTT.sensorCount = value.toInt();
      }
      else if (key == "USS_Count") {
        sensor.USS.sensorCount = value.toInt();
      }
      else if (key == "BME280_Count") {
        sensor.BME280.sensorCount = value.toInt();
      }
      else if (key == "PT100_Count") {
        sensor.PT100.sensorCount = value.toInt();
      }
      //Analog sensor config______________________
      else if (key == "ADC0_Attached") {
        sensor.adc[0].sensorCount = value.toInt();
      }
      else if (key == "ADC0_AveCount") {
        cfg.adc[0].aveCount = value.toInt();
      }
      else if (key == "ADC0_AveDelay") {
        cfg.adc[0].aveDelay = value.toInt();
      }
      else if (key == "ADC0_ID") {
        cfg.adc[0].id = value;
      }
      else if (key == "ADC0_Cal_Temp") {
        cfg.adc[0].tempPoints = arrayRead(value, cfg.adc[0].calTemp);  //arrayRead returns number of values in array.
      }
      else if (key == "ADC0_Cal_mV_min") {
        arrayRead(value, cfg.adc[0].mvMin);
      }
      else if (key == "ADC0_Cal_Val_min") {
        arrayRead(value, cfg.adc[0].valMin);
      }
      else if (key == "ADC0_Cal_mV_max") {
        arrayRead(value, cfg.adc[0].mvMax);
      }
      else if (key == "ADC0_Cal_Val_max") {
        cfg.adc[0].calPoints = arrayRead(value, cfg.adc[0].valMax);
      }

      else if (key == "ADC1_Attached") {
        sensor.adc[1].sensorCount = value.toInt();
      }
      else if (key == "ADC1_AveCount") {
        cfg.adc[1].aveCount = value.toInt();
      }
      else if (key == "ADC1_AveDelay") {
        cfg.adc[1].aveDelay = value.toInt();
      }
      else if (key == "ADC1_ID") {
        cfg.adc[1].id = value;
      }
      else if (key == "ADC1_Cal_Temp") {
        cfg.adc[1].tempPoints = arrayRead(value, cfg.adc[1].calTemp);
      }
      else if (key == "ADC1_Cal_mV_min") {
        arrayRead(value, cfg.adc[1].mvMin);
      }
      else if (key == "ADC1_Cal_Val_min") {
        arrayRead(value, cfg.adc[1].valMin);
      }
      else if (key == "ADC1_Cal_mV_max") {
        arrayRead(value, cfg.adc[1].mvMax);
      }
      else if (key == "ADC1_Cal_Val_max") {
        cfg.adc[1].calPoints = arrayRead(value, cfg.adc[1].valMax);
      }

      else if (key == "ADC2_Attached") {
        sensor.adc[2].sensorCount = value.toInt();
      }
      else if (key == "ADC2_AveCount") {
        cfg.adc[2].aveCount = value.toInt();
      }
      else if (key == "ADC2_AveDelay") {
        cfg.adc[2].aveDelay = value.toInt();
      }
      else if (key == "ADC2_ID") {
        cfg.adc[2].id = value;
      }
      else if (key == "ADC2_Cal_Temp") {
        cfg.adc[2].tempPoints = arrayRead(value, cfg.adc[2].calTemp);
      }
      else if (key == "ADC2_Cal_mV_min") {
        arrayRead(value, cfg.adc[2].mvMin);
      }
      else if (key == "ADC2_Cal_Val_min") {
        arrayRead(value, cfg.adc[2].valMin);
      }
      else if (key == "ADC2_Cal_mV_max") {
        arrayRead(value, cfg.adc[2].mvMax);
      }
      else if (key == "ADC2_Cal_Val_max") {
        cfg.adc[2].calPoints = arrayRead(value, cfg.adc[2].valMax);
      }

      else if (key == "ADC3_Attached") {
        sensor.adc[3].sensorCount = value.toInt();
      }
      else if (key == "ADC3_AveCount") {
        cfg.adc[3].aveCount = value.toInt();
      }
      else if (key == "ADC3_AveDelay") {
        cfg.adc[3].aveDelay = value.toInt();
      }
      else if (key == "ADC3_ID") {
        cfg.adc[3].id = value;
      }
      else if (key == "ADC3_Cal_Temp") {
        cfg.adc[3].tempPoints = arrayRead(value, cfg.adc[3].calTemp);
      }
      else if (key == "ADC3_Cal_mV_min") {
        arrayRead(value, cfg.adc[3].mvMin);
      }
      else if (key == "ADC3_Cal_Val_min") {
        arrayRead(value, cfg.adc[3].valMin);
      }
      else if (key == "ADC3_Cal_mV_max") {
        arrayRead(value, cfg.adc[3].mvMax);
      }
      else if (key == "ADC3_Cal_Val_max") {
        cfg.adc[3].calPoints = arrayRead(value, cfg.adc[3].valMax);
      }
      //Turbidity sensor config__________________
      else if (key == "Turbidity_Count") {
        sensor.turbidity.sensorCount = value.toInt();
      }
      else if (key == "tpumptme") {
        cfg.turbPumpTime = value.toInt();
      }
      else if (key == "tbflshtm") {
        cfg.turbBackflush = value.toInt();
      }
      else if (key == "dryread") {
        cfg.turbDryReads = value.toInt();
      }
      else if (key == "wetread") {
        cfg.turbWetReads = value.toInt();
      }
      else if (key == "minh2o") {
        cfg.minh2o = value.toInt();
      }
      else if (key == "turbPd") {
        cfg.turbPeriod = value.toInt();
      }

    }
  }
  configFile.close();
}

void wakeSensors() {
  digitalWrite(FET_POWER, LOW);
  digitalWrite(ONE_WIRE_POWER, HIGH);
  SensWakeTime = millis();
  tempSensor.setOneShotMode();
  bme280.setMode(MODE_FORCED);
}

void turnOff12V() {
  digitalWrite(FET_POWER, HIGH);
  digitalWrite(ONE_WIRE_POWER, LOW);
}

void disableWDT() {
  WDT->CTRL.reg = 0;
}

void sleepTillSynced() { //Initial clock sync
  WDT->CTRL.reg = 0; // disable watchdog
  DateTime now = rtc.now();

  int relativeTimeS = (now.minute() * 60 + now.second()) % (pollPeriod * cfg.pollPerLoRa);  //current time into poll period (time into hour/poll period)
  if (relativeTimeS > cfg.pollOffset) {
    sleep_remaining_s = pollPeriod * cfg.pollPerLoRa - relativeTimeS + cfg.pollOffset;
    // debug("First case "+String(sleep_remaining_s));
  }
  else if (relativeTimeS < cfg.pollOffset) {
    sleep_remaining_s = cfg.pollOffset - relativeTimeS;
    // debug("Second case "+String(sleep_remaining_s));
  }
  nextUnixWake = now.unixtime() + sleep_remaining_s;
  debug("Syncing sleep cycle. Sleeping for " + String(sleep_remaining_s) + " seconds");
  rf95.sleep();
  delay(20);    //Delay 20ms to ensure the chips have gone to sleep before powering off the board
  sleep();
}

bool LoRaListen(int timeout_S) {
  // Example use
  // if(LoRaListen(10)){;   //Enter 999 for no timeout
  // SerialUSB.println(LoRaReceive.RSSI);
  // SerialUSB.println(LoRaReceive.packet);}

  uint8_t len;
  int startTime = millis();
  while ((millis() - startTime) < (timeout_S * 1000) || timeout_S == 999) {
    while (rf95.available()) {
      len = sizeof(rfbuf);
      if (rf95.recv(rfbuf, &len)) {
        LoRaReceive.packet = (char*)rfbuf;
        LoRaReceive.RSSI = rf95.lastRssi();
      }
      return true;
    }
  }
  return false;
}

void bubbleSort(int a[], int arrayIndex[], int size) {
  for (int i = 0; i < (size - 1); i++) {
    for (int o = 0; o < (size - (i + 1)); o++) {
      if (a[o] > a[o + 1]) {
        int t = a[o];
        a[o] = a[o + 1];
        a[o + 1] = t;

        int tmp = arrayIndex[o];
        arrayIndex[o] = arrayIndex[o + 1];
        arrayIndex[o + 1] = tmp;
      }
    }
  }
}

void OneWireTempSetup() {
  int humanVal[4];
  pinMode(ONE_WIRE_POWER, OUTPUT);
  digitalWrite(ONE_WIRE_POWER, HIGH); //Turn on sensor
  digitalWrite(FET_POWER, LOW); //Turn on sensor
  delay(TEMP_INIT_TIME);  //Wait till we are warmed up
  tempSensors.begin();  // Start up the library
  sensor.temp.sensorCount = tempSensors.getDeviceCount(); //Check how many devices are attached
  //  debug("Temp sens count: " + String(sensor.temp.sensorCount));
  tempSensors.requestTemperatures();
  for (int i = 0;  i < sensor.temp.sensorCount;  i++) { //For each sensor, grab the value we will label with (Nibble 1 and 7 of full device address)
    tempSensors.getAddress(Thermometer, i);
    humanVal[i] = 256 * Thermometer[1] + Thermometer[2];
    //  debug("Temp_Address_" + String(i) + " = " + String(humanVal[i])+". Temp = "+String(tempSensors.getTempCByIndex(i)));
  }
  int arrayIndex[4] = {0, 1, 2, 3}; //This is how we will keep track of the order of the devices after sorting
  bubbleSort(humanVal, arrayIndex, sensor.temp.sensorCount);  //Sorts from smallest to largest. arrayIndex tells us where each value moved.
  for (int i = 0;  i < sensor.temp.sensorCount;  i++) { //For each sensor
    // tempSensors.getAddress(Thermometer, i);   //Read address
    tempSensors.getAddress(sensor.temp.addr[i], arrayIndex[i]);   //Assign addresses so that temp_1 will be the smallest address. Associate with relevant ALS probe.
    //  debug("HumanValOrdered_" + String(i) + " = " + String(humanVal[i]));

  }
  digitalWrite(ONE_WIRE_POWER, LOW);  //Switch off temp sensor
  digitalWrite(FET_POWER, HIGH);  //Switch off temp sensor
}

void wdtTimeoutCalc() {
  if (WDT_TIMEOUT <= 4) {
    wdtTime1 = 7 + WDT_TIMEOUT;
    wdtTime2 = 4;
  }
  else if (WDT_TIMEOUT > 4) {
    wdtTime1 = 11;
    wdtTime2 = WDT_TIMEOUT;
  }
}

void TMP117Setup() {
  if (tempSensor.begin(TMP_ADDR, Wire)) {
    sensor.TMP117.sensorCount = 1;
    tempSensor.setOneShotMode();
  }
}

void BME280Setup() {
  bme280.settings.commInterface = I2C_MODE;
  bme280.settings.I2CAddress = BME280_ADDR;
  if (bme280.beginI2C()) {           //May crash here if no BME connected
    sensor.BME280.sensorCount = 1;
  }
  bme280.setMode(MODE_SLEEP);//Need to sleep the library even if the chip is disconnected
}

void LoRaRepeaterLoop() {
  LoRaListen(999);   //Enter 999 for no timeout
  String tmp = LoRaReceive.packet;
  int i = 0;
  char * pch = strtok((char*)tmp.c_str(), ",");
  bool repeatPacket = false;
  while (pch != NULL) {
    for (int j = 0 ; j < MAX_REPEATED_NODES ; j++) {
      if (String(pch) == cfg.Nodes_to_repeat[j]) {
        repeatPacket = true;
      }
    }
    pch = strtok (NULL, ",");
    i++;
  }
  if (repeatPacket == true) {
    delay(500); //Wait for existing signal to clear
    // sendLoRaRaw("Repeat packet:");
    sendLoRaRaw(LoRaReceive.packet);
    // LoRaReceive.RSSI;
  }
}

void generateCSVHeader() {
  CSVHeader = "TIMESTAMP,SITE_ID,NODE_ID,FW_VER,COUNT,UPTIME,FILE_SIZE,BATT_V,";
  if (sensor.pulse1.sensorCount > 0) {
    CSVHeader += "Pulse1,Pulse1_Daily,";
  }
  if (sensor.pulse2.sensorCount > 0) {
    CSVHeader += "Pulse2,Pulse2_Daily,";
  }
  if (sensor.RTCTemp.sensorCount > 0) {
    CSVHeader += "RTCTemp,";
  }
  for (int j = 0; j < sensor.temp.sensorCount ; j++) {
    CSVHeader +=  "1WTemp_" + String(j) + ",";
  }
  if (sensor.TMP117.sensorCount > 0) {
    CSVHeader += "TMPTemp,";
  }
  if (sensor.OTT.sensorCount > 0 ) {
    CSVHeader += "OTT_Level,OTT_Temp,";
  }
  if (sensor.USS.sensorCount > 0) {
    CSVHeader += "USS,";
  }
  if (sensor.BME280.sensorCount > 0) {
    CSVHeader += "BMEHum,BMEPressure,";
  }
  if (sensor.PT100.sensorCount > 0) {
    CSVHeader += "PT100Temp,";
  }
  for (int j = 0; j < 4 ; j++) {
    if (sensor.adc[j].sensorCount > 0) {
      CSVHeader += "A_" + String(cfg.adc[j].id) + ",";
    }
  }
  if (sensor.turbidity.sensorCount > 0) {
    CSVHeader += "TURB_DRY,TURB_WET,HOUSING_TEMP,";
  }
  // CSVHeader += "\n";    //Doesnt seem to be wokring
}

bool pulse1Update() {
  DateTime now;
  sensor.pulse1.measure[0] = TC4->COUNT32.COUNT.reg;
  sensor.pulse1.measure_2[0] = sensor.pulse1.measure[0] * cfg.pulse1_mult;
  sensor.pulse1.measure_2[1] += sensor.pulse1.measure_2[0] ;
  TC4->COUNT32.COUNT.reg = 0;
  now = rtc.now();
  if (now.hour() == 0) {
    if (dailyReset) {   //WILL NEED ANOTHER DAILY RESET
      sensor.pulse1.measure_2[1] = 0;
      dailyReset = false;
    }
  }
  else if (!dailyReset) {
    dailyReset = true;
  }
  return true;
}

bool pulse2Update() {
  DateTime now;
  sensor.pulse2.measure[0] = TC3->COUNT16.COUNT.reg;
  sensor.pulse2.measure_2[0] = sensor.pulse2.measure[0] * cfg.pulse2_mult;
  sensor.pulse2.measure_2[1] += sensor.pulse2.measure_2[0] ;
  TC3->COUNT16.COUNT.reg = 0;
  now = rtc.now();
  if (now.hour() == 0) {
    if (dailyReset2) {
      sensor.pulse2.measure_2[1] = 0;
      dailyReset2 = false;
    }
  }
  else if (!dailyReset2) {
    dailyReset2 = true;
  }
  return true;
}

bool BME280Update() {
  int wait = BME280_WAKE - (millis() - SensWakeTime);
  if (wait > 0 ) {
    delay(wait);
  }
  bme280.readTempC();  //This is used to calibrate pressure (maybe humidity)
  sensor.BME280.measure[0] = bme280.readFloatHumidity();
  sensor.BME280.measure_2[0] = bme280.readFloatPressure();
  return true;
}

bool TMP117Update() {
  int wait = TMP117_WAKE - (millis() - SensWakeTime);
  if (wait > 0 ) {
    delay(wait);
  }
  sensor.TMP117.measure[0] = tempSensor.readTempC();
  return true;
}

uint8_t arrayRead(String valueString, String valueArray[]) {
  int i = 0;
  char * pch;
  pch = strtok((char*)valueString.c_str(), "{},");
  while (pch != NULL) {
    valueArray[i] = pch;
    pch = strtok (NULL, ",{}");
    i++;
  }
  return i;
}

uint8_t arrayRead(String valueString, float valueArray[]) {
  int i = 0;
  char * pch;
  pch = strtok((char*)valueString.c_str(), "{},");
  while (pch != NULL) {
    valueArray[i] = atof(pch);
    pch = strtok (NULL, ",{}");
    i++;
  }
  return i;
}

uint8_t arrayRead(String valueString, int valueArray[]) {
  int i = 0;
  char * pch;
  pch = strtok((char*)valueString.c_str(), "{},");
  while (pch != NULL) {
    valueArray[i] = atoi(pch);
    pch = strtok (NULL, ",{}");
    i++;
  }
  return i;
}

bool ADCUpdate(int adcChannel) {
  double f = ads.toVoltage(1);
  int wait = cfg.analogWakeTime - (millis() - SensWakeTime);
  if (wait > 0 ) {
    delay(wait);
  }
  //   ads.readADC_SingleEnded(SM_1_PIN);     FOR ADS1105 sensors
  // tmp = ads.getLastConversionResults() * 2;
  // sensorData.soilMoisture1 = SM_conversion(tmp);

  //Read and average__________________
  sensor.adc[adcChannel].measure[0] = 0;
  sensor.adc[adcChannel].measure[0] += ads.readADC(adcChannel) * f * 1000;
  for (int k = 1; k < cfg.adc[adcChannel].aveCount; k++) {
    delay(cfg.adc[adcChannel].aveDelay);
    sensor.adc[adcChannel].measure[0] += ads.readADC(adcChannel) * f * 1000;
  }
  sensor.adc[adcChannel].measure[0] = sensor.adc[adcChannel].measure[0] / cfg.adc[adcChannel].aveCount; //true makes a call, only do it once per poll session
  //Calibration______________________
  if (cfg.adc[adcChannel].tempPoints < 2) { //No temperature calibration
    linearPiecewiseADCCal(adcChannel);
  }
  else {
    temperatureADCCal(adcChannel);
  }
  return true;
}

void pulse1Setup() {
  //  CLOCK /////////////////////////////////////////////////////
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |        // Enable the generic clock...
  GCLK_CLKCTRL_GEN_GCLK1 |    // On GCLK1
  GCLK_CLKCTRL_ID_TC4_TC5;    //Route to TC4
  GCLK->GENCTRL.bit.RUNSTDBY = 1;                 //GCLK1 run standby
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);  //Wait for sync

  // MUX  //////////////////////////////////////////////////////////
  PORT->Group[g_APinDescription[A1].ulPort].PINCFG[g_APinDescription[A1].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[A1].ulPort].PMUX[g_APinDescription[A1].ulPin >> 1].reg |= PORT_PMUX_PMUXO_A;    //See table 6-1 for MUXO_A MUXO_B etc

  // EXTERNAL INTERUPT CONTROLLER   //////////////////////////////////////
  EIC->EVCTRL.reg |= EIC_EVCTRL_EXTINTEO8;                                // Enable event output on external interrupt 8 (A1). Defined in table 6-1 PORT function multiplexing in the SAMD21 datasheet
  EIC->CONFIG[1].reg |= EIC_CONFIG_SENSE0_LOW;                           // Set event detecting a LOW level on interrupt 8. Defined in Table 20-3.
  EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT8;                               // Disable interrupts on interrupt 3
  EIC->CTRL.bit.ENABLE = 1;                                               // Enable the EIC peripheral
  while (EIC->STATUS.bit.SYNCBUSY);                                       // Wait for synchronization

  // EVENT SYSTEM SETUP  ////////////////////////////////////////////////////
  PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;                                  // Switch on the event system peripheral
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) |                               // Attach the event user (receiver) to channel 0 (n + 1)
  EVSYS_USER_USER(EVSYS_ID_USER_TC4_EVU);
  // Set the event user (receiver) as timer TC4. Find here: C:\Users\User\AppData\Local\Arduino15\packages\arduino\tools\CMSIS-Atmel\1.2.0\CMSIS\Device\ATMEL\samd21\include\instance\evsys.h

  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |               // No event edge detection
  EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                  // Set event path as asynchronous
  EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_8) |   // Set event generator (sender) as external interrupt 8
  EVSYS_CHANNEL_CHANNEL(0);                          // Attach the generator (sender) to channel 0

  // TIMER COUNTER SETUP  //////////////////////////////////////////////////
  TC4->COUNT32.EVCTRL.reg |= TC_EVCTRL_TCEI |              // Enable asynchronous events on the TC timer
  TC_EVCTRL_EVACT_COUNT;        // Increment the TC timer each time an event is received

  TC4->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32;
  // Configure TC4 together with TC5 to operate in 32-bit mode. C:\Users\User\AppData\Local\Arduino15\packages\arduino\tools\CMSIS-Atmel\1.2.0\CMSIS\Device\ATMEL\samd21\include\component\tc.h

  TC4->COUNT32.CTRLA.bit.ENABLE = 1;                       // Enable TC4
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);                // Wait for synchronization

  TC4->COUNT32.READREQ.reg = TC_READREQ_RCONT |            // Enable a continuous read request
  TC_READREQ_ADDR(0x10);        // Offset of the 32-bit COUNT register
  TC4->COUNT32.CTRLA.bit.RUNSTDBY = 1;
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);                // Wait for synchronization
}

void pulse2Setup() {
  //  CLOCK /////////////////////////////////////////////////////
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |        //Comments and instructions in pulse1Setup()
  GCLK_CLKCTRL_GEN_GCLK1 |
  GCLK_CLKCTRL_ID_TCC2_TC3;
  GCLK->GENCTRL.bit.RUNSTDBY = 1;
  while (GCLK->STATUS.bit.SYNCBUSY);

  // MUX  //////////////////////////////////////////////////////////
  PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg |= PORT_PMUX_PMUXO_A;

  // EXTERNAL INTERUPT CONTROLLER   //////////////////////////////////////
  EIC->EVCTRL.reg |= EIC_EVCTRL_EXTINTEO14;
  EIC->CONFIG[1].reg |= EIC_CONFIG_SENSE6_LOW;
  EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT14;
  EIC->CTRL.bit.ENABLE = 1;
  while (EIC->STATUS.bit.SYNCBUSY);

  // EVENT SYSTEM SETUP  ////////////////////////////////////////////////////
  PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(2) |
  EVSYS_USER_USER(EVSYS_ID_USER_TC3_EVU);

  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |
  EVSYS_CHANNEL_PATH_ASYNCHRONOUS |
  EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_14) |
  EVSYS_CHANNEL_CHANNEL(1);

  // TIMER COUNTER SETUP  //////////////////////////////////////////////////
  TC3->COUNT16.EVCTRL.reg |= TC_EVCTRL_TCEI |           //16 bit counter for TC3
  TC_EVCTRL_EVACT_COUNT;

  TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16;
  TC3->COUNT16.CTRLA.bit.ENABLE = 1;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

  TC3->COUNT16.READREQ.reg = TC_READREQ_RCONT |
  TC_READREQ_ADDR(0x10);
  TC3->COUNT16.CTRLA.bit.RUNSTDBY = 1;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

void updateADCEquations() {
  for (int i = 0; i < 4; i++) { //For every ADC channel
    for (int j = 0; j < cfg.adc[i].calPoints; j++) { //For every calibrated point
      cfg.adc[i].slope[j] = (cfg.adc[i].valMax[j] - cfg.adc[i].valMin[j]) / (cfg.adc[i].mvMax[j] - cfg.adc[i].mvMin[j]);
      cfg.adc[i].offset[j] = cfg.adc[i].valMin[j] - (cfg.adc[i].slope[j] * cfg.adc[i].mvMin[j]);
    }
  }
}

void linearPiecewiseADCCal(int adcChannel) { //This function uses linear interpolation to calibrate sensors with PIECEWISE LINEAR EQUATIONS
  double Slope, Offset;
  float linearInterpRatio;
  if (sensor.adc[adcChannel].measure[0] < cfg.adc[adcChannel].mvMin[0]) { //Voltage below minimum
    Slope = cfg.adc[adcChannel].slope[0];
    Offset = cfg.adc[adcChannel].offset[0];
    debug("ADC" + String (adcChannel) + " below min calibrated voltage");
  }
  else if (sensor.adc[adcChannel].measure[0] > cfg.adc[adcChannel].mvMax[cfg.adc[adcChannel].calPoints - 1]) { //Voltage above maximum
    Slope = cfg.adc[adcChannel].slope[cfg.adc[adcChannel].calPoints - 1];
    Offset = cfg.adc[adcChannel].offset[cfg.adc[adcChannel].calPoints - 1];
    debug("ADC" + String (adcChannel) + " above max calibrated voltage");
  }
  else {
    for (int j = 0; j < cfg.adc[adcChannel].calPoints; j++) {
      if (sensor.adc[adcChannel].measure[0] < cfg.adc[adcChannel].mvMax[j]) {
        sensor.adc[adcChannel].measure_2[0] = sensor.adc[adcChannel].measure[0] * cfg.adc[adcChannel].slope[j] + cfg.adc[adcChannel].offset[j];
        // linearInterpRatio = (sensor.adc[adcChannel].measure[0] - cfg.adc[adcChannel].mvMin[j]) / (cfg.adc[adcChannel].mvMax[j] - cfg.adc[adcChannel].mvMin[j]);  //this is old and wrong. I had it confused with the logic from temp cal. might have been skewing results slightly
        // Slope = linearInterpRatio * (cfg.adc[adcChannel].slope[j] - cfg.adc[adcChannel].slope[j-1]) + cfg.adc[adcChannel].slope[j-1];
        // Offset = linearInterpRatio * (cfg.adc[adcChannel].offset[j] - cfg.adc[adcChannel].offset[j-1]) + cfg.adc[adcChannel].offset[j-1];
        break;
      }
    }
  }

}


void temperatureADCCal(int adcChannel) { //This function uses linear interpolation to calibrate sensors based on a TEMPERATURE CHANGE
  double Slope, Offset; //tempCalCount resets once per poll to ensure order of multiple ALS probes
  float linearInterpRatio;
  if (sensor.temp.measure[tempCalCount] < cfg.adc[adcChannel].calTemp[0] ) { //Temperature below minimum
    Slope = cfg.adc[adcChannel].slope[0];
    Offset = cfg.adc[adcChannel].offset[0];
    debug("ADC" + String (adcChannel) + " below min calibrated temp");
  }
  else if (sensor.temp.measure[tempCalCount] > cfg.adc[adcChannel].calTemp[cfg.adc[adcChannel].tempPoints - 1]) {
    Slope = cfg.adc[adcChannel].slope[cfg.adc[adcChannel].tempPoints - 1];
    Offset = cfg.adc[adcChannel].offset[cfg.adc[adcChannel].tempPoints - 1];
    debug("ADC" + String (adcChannel) + " above max calibrated temp");
  }
  else {
    for (int j = 1; j < cfg.adc[adcChannel].tempPoints; j++) {
      if (sensor.temp.measure[tempCalCount] < cfg.adc[adcChannel].calTemp[j]) {
        linearInterpRatio = (sensor.temp.measure[tempCalCount] - cfg.adc[adcChannel].calTemp[j - 1]) / (cfg.adc[adcChannel].calTemp[j] - cfg.adc[adcChannel].calTemp[j - 1]);
        Slope = linearInterpRatio * (cfg.adc[adcChannel].slope[j] - cfg.adc[adcChannel].slope[j - 1]) + cfg.adc[adcChannel].slope[j - 1];
        Offset = linearInterpRatio * (cfg.adc[adcChannel].offset[j] - cfg.adc[adcChannel].offset[j - 1]) + cfg.adc[adcChannel].offset[j - 1];
        break;
      }
    }
  }
  sensor.adc[adcChannel].measure_2[0] = sensor.adc[adcChannel].measure[0] * Slope + Offset;
  tempCalCount++;
}

bool PT100Update(void) {
  sensor.PT100.measure[0] = thermo.temperature(RNOMINAL, RREF);
}

void bme280Setup() {
  // BME280Setup();
  bme280.settings.commInterface = I2C_MODE;
  bme280.settings.I2CAddress = BME280_ADDR;
  if (bme280.beginI2C()) {           //May crash here if no BME connected
    sensor.BME280.sensorCount = 1;
  }
  bme280.setMode(MODE_SLEEP);//Need to sleep the library even if the chip is disconnected
}


uint32_t makeTime(int Second, int Minute, int Hour, int Day, int Month, int Year) {
  int i;
  uint32_t seconds;

  // seconds from 1970 till 1 jan 00:00:00 of the given year
  seconds = Year * (SECS_PER_DAY * 365);
  for (i = 0; i < Year; i++) {
    if (LEAP_YEAR(i)) {
      seconds += SECS_PER_DAY;   // add extra days for leap years
    }
  }

  // add days for this year, months start from 1
  for (i = 1; i < Month; i++) {
    if ( (i == 2) && LEAP_YEAR(Year)) {
      seconds += SECS_PER_DAY * 29;
    } else {
      seconds += SECS_PER_DAY * monthDays[i - 1]; //monthDay array starts from 0
    }
  }
  seconds += (Day - 1) * SECS_PER_DAY;
  seconds += Hour * SECS_PER_HOUR;
  seconds += Minute * SECS_PER_MIN;
  seconds += Second;
  return (uint32_t)seconds;
}

//Turbidity routines (merged from MobileTurbidity/PauloPaperMobileSed_rj.ino)
void resetTurbidityMedians() {
  turbVoltageMedian = 0.0f;
  turbHousingMedian = 0.0f;
  turbidity = 0.0f;
  turbidity_air_avg = 0.0f;
}

void TurbMeasure(String nameOfFile, int howManyReads) {
  //Collect raw turbidity voltage and temperature arrays, compute medians
  if (howManyReads > 10) howManyReads = 10; //array size guard
  for (int i = 0; i < howManyReads; i++) {
    resetWDT();
    if (sensor.TMP117.sensorCount > 0) sensor.TMP117.measure[0] = tempSensor.readTempC();
    if (sensor.BME280.sensorCount > 0) sensor.BME280.measure[0] = bme280.readTempC();
    turbHousingMeasurementArray[i] = sensor.TMP117.measure[0];
    turbWaterMeasurementArray[i] = sensor.BME280.measure[0];
    turbVoltageMeasurementArray[i] = ads.readADC(1) * ads.toVoltage(1) * 1000; //ADS1115 channel 1, mV (same mV conversion as ADCUpdate)
    delayUsingMillis(cfg.turbPeriod);
  }
  turbVoltageMedian = QuickMedian<float>::GetMedian(turbVoltageMeasurementArray, howManyReads);
  turbHousingMedian = QuickMedian<float>::GetMedian(turbHousingMeasurementArray, howManyReads);
  SaveTurbData(nameOfFile, howManyReads);
}

void SaveTurbData(String nameOfFile, int howManyReads) { //Append raw turbidity readings to a dedicated CSV
  FsDateTime::setCallback(dateTime);
  String csvObject = "";
  if (!SD.exists((char*)nameOfFile.c_str())) {
    csvObject = "Site_name,DateTime,Unixtime[s],Measure_number[-],Turbidity voltage[mV],Housing Temp[c],Water Temp[c]\n";
  }
  file.open((char*)nameOfFile.c_str(), O_RDWR | O_CREAT | O_APPEND);
  if (!file) {
    debug("Failed to open " + nameOfFile + " for turbidity logging");
    return;
  }
  for (int i = 0; i < howManyReads; i++) {
    csvObject = cfg.siteID + "_" + cfg.NodeID + "," + normTimestamp + UNIXtimestamp + "," + String(tx_count, DEC) + "," + String(turbVoltageMeasurementArray[i], 4) + "," + String(turbHousingMeasurementArray[i], 2) + "," + String(turbWaterMeasurementArray[i], 2) + "\n";
    file.write((char*)csvObject.c_str());
  }
  file.sync();
  file.close();
}

void forwardPump() { //Run forward pump for configured seconds, servicing WDT
  for (int i = 0; i < cfg.turbPumpTime; i++) {
    delayUsingMillis(1000);
    resetWDT();
  }
}

void reversePump() { //Run reverse pump for configured seconds, servicing WDT
  for (int i = 0; i < cfg.turbBackflush; i++) {
    delayUsingMillis(1000);
    resetWDT();
  }
}

void delayUsingMillis(unsigned long period) {
  unsigned long start = millis();
  while (millis() - start < period) {
    resetWDT();
    yield();
  }
}

String USBUpdate() {
  int usbStart = millis();
  bool dataRec = 0;
  String USBData;
  while (dataRec == 0) {
    if (SerialUSB.available()) {
      USBData = SerialUSB.readStringUntil('\n');
      dataRec = 1;
    }
  }
  return USBData;
}
