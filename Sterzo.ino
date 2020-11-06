/* 
  a proof of concept STERZO imitation based on a nrf51822 smart watch
  to get going:
  - modify variant.h to suit the pin layout of your smartwatch,
    in particular I2C for the accelerometer, and Serial TX for the Serial.prints to work
  - install Arduino NRF51 board package & BLEPeripheral library
  - select the device settings corresponding with the smartwatch : ram/flash (xxaa), oscillator(RC)
  - hookup a ST-LINK for programming
  - flash a Nordic SoftDevice (S110 works for me)
  - program arduino sketch
  
*/

#include "nrf.h"

// ble
#include <SPI.h>
#include <BLEPeripheral.h>

// accelerometer
#include "KX022.h"

// display output
#include <Wire.h>
#include <U8g2lib.h> 

// define pins (varies per shield/board)
#define BLE_REQ   10
#define BLE_RDY   2
#define BLE_RST   9
#define BLE_DEVICE_NAME "stefaan-sterzo"

#define pinButton     (4) // P0.04
#define pinBattery    (5) // P0.05

#define DEVICE_SLEEP_TIMEOUT 60000 //ms
// nRF51822: -40, -30, -20, -16, -12, -8, -4, 0, 4
#define BLE_TX_POWER_IN_DB (-12)   // 0dB works ok, -40dB is gentler on the battery, but doesn't work properly

uint16_t myCrankRevs = 0;
uint32_t crankRevLastEventMillis = 0; // last crank event time in millis


#define pinAccelerometerINT1 (13) // PIN1 = P0.13, PIN2 = P0.12, ADDR = P0.15
// accelerometer
KX022 acc;
volatile bool kx022InterruptFlag = false;
bool isCrankUp = false; // basic debounce for the accelerometer tilt position
void KX022_IRQHandler(void);
void KX022_DoInterrupts(void);

bool bleConnected = false;
uint32_t bleNotifyMillis = 0;
uint32_t bleNotifyErrorsForDebugOnly = 0;

BLEPeripheral blePeripheral = BLEPeripheral(BLE_REQ, BLE_RDY, BLE_RST);
//BLEService svcSpeedCadence = BLEService("1816"); // assigned UUID!
BLEService svcSterzo = BLEService("347b0001-7635-408b-8918-8ff3949ce592");

// cadence sensor characteristics
//BLECharacteristic charSpeedCadenceMeasurement = BLECharacteristic("2a5b", BLENotify,5); // valueSize = 5, we gaan 5 bytes sturen
//BLECharacteristic charSpeedCadenceFeature = BLECharacteristic("2a5c", BLERead,2);

// sterzo characteristics
BLECharacteristic char12 = BLECharacteristic("347b0012-7635-408b-8918-8ff3949ce592", BLEWrite,4); // waarom lukt dat hier niet zonder valueSize??
BLECharacteristic char13 = BLECharacteristic("347b0013-7635-408b-8918-8ff3949ce592", BLERead,1); // read 1 byte 0xFF
BLECharacteristic char14 = BLECharacteristic("347b0014-7635-408b-8918-8ff3949ce592", BLENotify,1); // 1 byte 0xFF, maar doet niets
BLECharacteristic char19 = BLECharacteristic("347b0019-7635-408b-8918-8ff3949ce592", BLERead,1); // read 1 byte 0xFF
BLECharacteristic char30 = BLECharacteristic("347b0030-7635-408b-8918-8ff3949ce592", BLENotify,4); // 4 bytes stearing angle
BLECharacteristic char31 = BLECharacteristic("347b0031-7635-408b-8918-8ff3949ce592", BLEWrite,4); // zwift writes 4 bytes, says Keith
BLECharacteristic char32 = BLECharacteristic("347b0032-7635-408b-8918-8ff3949ce592", BLEIndicate,4); // 4 bytes challenge

unsigned char csc_manufacturerData[] = "stefaanLLC";

// the oled display
U8G2_SSD1306_64X32_1F_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 2, /* dc=*/ 0, /* reset=*/ 1); 
uint32_t displayMillis;
bool displayOn = false;
#define DISPLAY_MAXPAGES 2
uint8_t displayPageId = 0;

/*
static void bleNotifyCrankRevs ()
{
  // notify central about crankrevs every second
  if ((bleConnected) && (charSpeedCadenceMeasurement.subscribed())) {
    // eventueel canNotify of canIndicate gebruiken
    // deze gaven altijd 'true' van zodra een central connected was
    // maar in nRF51822 wordt gecheckt op buffer
    uint16_t crankEventTime;
    unsigned char cscMeasurementValue[5];
    crankEventTime = crankRevLastEventMillis % 64000;
    crankEventTime = (crankEventTime * 1024) / 1000;
    cscMeasurementValue[0] = 2; // flags
    cscMeasurementValue[1] = (unsigned char) (myCrankRevs & 0xFF);
    cscMeasurementValue[2] = (unsigned char) ((myCrankRevs>>8) & 0xFF);
    cscMeasurementValue[3] = (unsigned char) (crankEventTime & 0xFF);
    cscMeasurementValue[4] = (unsigned char) ((crankEventTime>>8) & 0xFF);
    bool retval;
    // dit stuurt data naar de central :
    retval = charSpeedCadenceMeasurement.setValue(cscMeasurementValue,5);
    if (!retval) {
      // too bad, hopefully better next time
      bleNotifyErrorsForDebugOnly++; // for debug, see if this causes missing data during ride
    }
  }
  
} // bleNotifyCrankRevs
*/
#define MAX_ANGLE 45
static void bleNotifySteeringAngle ()
{
  float xyz[3];
  Wire.begin();
  acc.getAccelXYZ(xyz);
  Wire.end();
  //Serial.print("X: ");Serial.print(xyz[0]);
  //Serial.print(" - Y: ");Serial.print(xyz[1]);
  //Serial.print(" - Z: ");Serial.println(xyz[2]);
  // link X acceleration to angle
  float steeringAngle = xyz[0]*MAX_ANGLE; // x acceleration between -1 & +1g -> convert to +- MAX_ANGLE
  Serial.print("sending angle ");Serial.println(steeringAngle);

  // notify central about crankrevs every second
  if ((bleConnected) && (char30.subscribed())) {
    // eventueel canNotify of canIndicate gebruiken
    // deze gaven altijd 'true' van zodra een central connected was
    // maar in nRF51822 wordt gecheckt op buffer
    bool retval;
    retval = char30.setValue((uint8_t *)&steeringAngle,4);
    if (!retval) {
      // too bad, hopefully better next time
      Serial.println("could not notify");
    }
  }
} // bleNotifySteeringAngle

static void displayPage (uint8_t pageId) {
  Wire.begin();
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setContrast(255); // 255 = highest possible 

  // page0 : batteryVoltage & crankRevs
  if (pageId == 0) {
    // batteryVoltage
    int batVoltage = analogRead(pinBattery);
    u8g2.setCursor(1,10);
    u8g2.print("bat:");
    u8g2.print((float)batVoltage*0.019266); // conversion todo
    u8g2.print("V");
    u8g2.setCursor(1,20);
    u8g2.print("revs:");
    u8g2.print(myCrankRevs);
    u8g2.setCursor(1,30);
    if (bleConnected) u8g2.print("conn");
    else u8g2.print("disconn");
    u8g2.sendBuffer();    
  }
  else if (pageId == 1) {
    // ble errors
    u8g2.setCursor(1,10);
    u8g2.print("err: ");
    u8g2.print(bleNotifyErrorsForDebugOnly);
    u8g2.sendBuffer();    
  }
  Wire.end(); // keep TWI peripheral off as much as possible
  
} // displayPage


// ble callbacks
void blePeripheralConnectHandler(BLECentral& central) {
  bleConnected = true;
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  bleConnected = false;
}

void charHeartRateControlPointEventHandler (BLECentral& central, BLECharacteristic& characteristic) {
}

// niet nodig, want Zwift schrijft hier niets naartoe?
void char12EventHandler (BLECentral& central, BLECharacteristic& characteristic) {
  Serial.print(F("char12EventHandler : "));
  Serial.print(characteristic.valueLength()); Serial.println(F(" bytes written!"));
  for (int i=0;i<characteristic.valueLength();i++) {
      Serial.print(characteristic[i]);Serial.print(" ");
  }
  Serial.println();
} // char12EventHandler

// if Zwift writes 0x310, we send a challenge on char32
void char31EventHandler (BLECentral& central, BLECharacteristic& characteristic) {
  Serial.print(F("char31EventHandler : "));
  Serial.print(characteristic.valueLength()); Serial.println(F(" bytes written!"));
  for (int i=0;i<characteristic.valueLength();i++) {
      Serial.print(characteristic[i]);Serial.print(" ");
  }
  Serial.println();

  if ((characteristic[0] == 0x03) && (characteristic[1] == 0x10)) {
    unsigned char challenge[] = {0x03,0x10,0x4a, 0x89};
    bool retval;
    Serial.println(F("char31EventHandler : got 0x310!"));
    retval = char32.setValue(challenge,4);
    if (!retval) {
      Serial.println(F("char32.setValue failed, no fallback for now"));
    }
  }
  else if ((characteristic[0] == 0x03) && (characteristic[1] == 0x11)) {
    unsigned char fakeData[] = {0x03,0x11,0xff, 0xff};
    bool retval;
    Serial.println(F("char31EventHandler : got 0x311!"));
    retval = char32.setValue(fakeData,4);
    if (!retval) {
      Serial.println(F("char32.setValue failed, no fallback for now"));
    }
  }
} // char31EventHandler

// if Zwift subscribes to the notifications, we send a challenge on char32
void char32EventHandler (BLECentral& central, BLECharacteristic& characteristic) {
  if ((bleConnected) && (char32.subscribed())) {
    unsigned char challenge[] = {0x03,0x10,0x4a, 0x89};
    bool retval;
    retval = char32.setValue(challenge,4);
    if (!retval) {
      Serial.println(F("char32.setValue failed, no fallback for now"));
    }
  }
} // char32EventHandler

void setup() {

  Serial.begin(115000);
  Serial.println("start");

  // 1. setup pins
  pinMode (pinButton, INPUT_PULLUP);

  // 2. BLE setup
  blePeripheral.setDeviceName(BLE_DEVICE_NAME);
  //blePeripheral.setAppearance(0x0); // wadisda?

  blePeripheral.setAdvertisingInterval(100); // 100ms is default - 500 for low power
  //can this also be used to save power ?
  // master will contact slave at most every 500ms and at least 1/s
  // that's enough because we want to notify only 1/s
  // tried in connect/disconnect handler -> works sometimes
  //blePeripheral.setConnectionInterval(400,800); // default : 40/80 = 50ms-100ms
  //blePeripheral.setConnectionInterval(120,240); // 400/800 didn't work reliably with nrfconnect, didn't check strava 
  blePeripheral.setConnectionInterval(40,80); // sterzo test 

  blePeripheral.setLocalName(BLE_DEVICE_NAME); // optional
  blePeripheral.setManufacturerData(csc_manufacturerData, sizeof(csc_manufacturerData));
  //blePeripheral.setAdvertisedServiceUuid(svcSpeedCadence.uuid());
  blePeripheral.setAdvertisedServiceUuid(svcSterzo.uuid());

  // add attributes (services, characteristics, descriptors) to peripheral
  /*
  blePeripheral.addAttribute(svcSpeedCadence);
  blePeripheral.addAttribute(charSpeedCadenceMeasurement);
  blePeripheral.addAttribute(charSpeedCadenceFeature);
*/
  blePeripheral.addAttribute(svcSterzo);
  blePeripheral.addAttribute(char12);
  blePeripheral.addAttribute(char13);
  blePeripheral.addAttribute(char14);
  blePeripheral.addAttribute(char19);
  blePeripheral.addAttribute(char30);
  blePeripheral.addAttribute(char31);
  blePeripheral.addAttribute(char32);

  // set initial value
  /*
  unsigned char cscMeasurementValue[] = {2,1,0,0,0}; // 2 = flags, 1,0 = cumulative crank revs; 0,0 = last crank event time
  unsigned char cscFeatureVal[] = {2,0}; // crank rev data supported
  charSpeedCadenceMeasurement.setValue(cscMeasurementValue,5); 
  charSpeedCadenceFeature.setValue(cscFeatureVal,2);
  */
  unsigned char defaultValue[4] = {0x0,0x0,0x0,0x0};
  char30.setValue(defaultValue,4); // default angle = 0
  defaultValue[0] = 0xFF; // fill other characteristics with a default 0xFF
  char12.setValue(defaultValue,1);
  char13.setValue(defaultValue,1);
  char14.setValue(defaultValue,1);
  char19.setValue(defaultValue,1);
  char31.setValue(defaultValue,1);
  char32.setValue(defaultValue,1);


  // event handler for the BLE device
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  
  // event handler for the characteristic
  char12.setEventHandler(BLEWritten, char12EventHandler);
  char31.setEventHandler(BLEWritten, char31EventHandler);
  char32.setEventHandler(BLESubscribed, char32EventHandler);

  // begin initialization
  blePeripheral.begin();
  blePeripheral.setTxPower(BLE_TX_POWER_IN_DB);

  // 3. setup accelerometer
  Wire.begin();
  // TODO : delay alleen nodig als er geen batterij aan hangt
  delay(500); // wait for the accelerometer to power up
  acc.init();
  // int active high geconfiged in de kx022 lib
  attachInterrupt(pinAccelerometerINT1,KX022_IRQHandler, RISING);
  // somehow I miss the first rising edge of the PIN1,
  // even if I attachInterrupt before acc.init()
  acc.readRegister(KX022_INT_REL);
  Wire.end(); // keep TWI peripheral off as much as possible

  // clear any port event from wakeup (because arduino framework doesn't do this)
  // otherwise next time port detect interrupt is activated we end up in an endless loop from GPIOTE_IRQHandler
  NRF_GPIOTE->EVENTS_PORT = 0;

  // dit gebeurt eigenlijk ook al door attachInterrupt via _initialize ..
  NVIC_SetPriority(GPIOTE_IRQn, 15);
  NVIC_ClearPendingIRQ(GPIOTE_IRQn);
  NVIC_EnableIRQ(GPIOTE_IRQn);

} // setup


/*
 * on active ble connection, sensor notifies central every second
 * without connection, sensor stays awake as long as crank revolutions are detected
 * without connection, sensor goes to sleep after 60s if no crank revolutions are detected
 */
void loop() {

  blePeripheral.poll();

  // dit is de acc.poll()
  if (kx022InterruptFlag)
    KX022_DoInterrupts();


  //TEMP if (bleConnected) {
  if (true) {
    // notify crank revs every second
    if (millis() - bleNotifyMillis > 1000) {
      //bleNotifyCrankRevs ();
      bleNotifySteeringAngle ();
      bleNotifyMillis = millis();
    }
  }
  else {
    // wait for ble connection
    // after 1 minute without pulses go to sleep
    if ((millis() - crankRevLastEventMillis) > DEVICE_SLEEP_TIMEOUT) {
      // setup wakeup from button
      NRF_GPIO->PIN_CNF[pinButton] &= ~GPIO_PIN_CNF_SENSE_Msk;
      NRF_GPIO->PIN_CNF[pinButton] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);

      // accelerometer in standby
      Wire.begin();
      acc.writeRegister(KX022_CNTL1,CNTL1_PC1_STANDBY);
      Wire.end(); // keep TWI peripheral off as much as possible
      // ALTERNATIVE : wake-up from accelerometer
      // we need to leave the accelerometer on for that -> battery life?
      // met ODR=0.781Hz is de current 1.8uA ipv 0.9uA standby current
      // TODO : use TPE or WUFE for wakeup? any difference in battery life?
      // setup wakeup from accelerometer
      //NRF_GPIO->PIN_CNF[pinAccelerometerINT1] &= ~GPIO_PIN_CNF_SENSE_Msk;
      //NRF_GPIO->PIN_CNF[pinAccelerometerINT1] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);

      NRF_GPIOTE->INTENSET = 0x80000000; /* activate PORT DETECT interrupt */
      NRF_POWER->SYSTEMOFF = 0x1UL; /* using system off -> wake up with a reset */
      while(1); 
      // won't come here
    }
  }

  // display : check side button and display pages
  if ((digitalRead(pinButton) == LOW) && ((millis() - displayMillis) > 500)) {
    displayMillis = millis();
    displayOn = true;
    displayPage(displayPageId);
    displayPageId++;
    if (displayPageId >= DISPLAY_MAXPAGES) displayPageId = 0;
  }
  if (displayOn && ((millis() - displayMillis) > 5000)) {
    Wire.begin();
    u8g2.clearDisplay();
    u8g2.setPowerSave(1); // DISPLAY_OFF
    Wire.end(); // keep TWI peripheral off as much as possible
    displayOn = false;
  }  

} // loop

void KX022_IRQHandler(void) {
  // ISR context -> no i2c access to device here!
  kx022InterruptFlag = true;
  // we can't eliminate the source of the external interrupt yet
  // so disable this irq until we handle the interrupt in KX022_DoInterrupts
  NVIC_DisableIRQ(GPIOTE_IRQn);
} // KX022_IRQHandler

void KX022_DoInterrupts(void) {
  uint8_t ins2, curTiltDirection;
  // read INS2
  Wire.begin();
   ins2 = acc.readRegister(KX022_INS2);

  if (ins2 & INS2_WUFS) {
    // TODO handle wakeup
  }

  if (ins2 & INS2_TPS) {
    // find tilt direction
    //uint8_t prevDirection = acc.readRegister(KX022_TSPP);
    curTiltDirection = acc.readRegister(KX022_TSCP);
    if ((curTiltDirection == TILTPOSITION_LEFT) || (curTiltDirection == TILTPOSITION_DOWN)) {
      isCrankUp = false;
    }
    else if ((curTiltDirection == TILTPOSITION_RIGHT) || (curTiltDirection == TILTPOSITION_UP)) {
      if (!isCrankUp) {
        myCrankRevs++;
        crankRevLastEventMillis = millis();
      }
      isCrankUp = true;
    }
  }

  // clear int
  acc.readRegister(KX022_INT_REL);
  Wire.end(); // keep TWI peripheral off as much as possible

  kx022InterruptFlag = false;
  NVIC_EnableIRQ(GPIOTE_IRQn);  

} // KX022_DoInterrupts

// not compatible with arduino framework WInterrupts has GPIOTE_IRQHandler
// not needed here
/*
extern "C"{
void GPIOTE_IRQHandler(void)
{
  // events wissen -> dit neemt de int source weg!
  // hoe weet je nu welk event actief is? -> ze 1 voor 1 afgaan
  // maar in dit voorbeeld worden enkel IN[0] en PORTS geactiveerd
  if (NRF_GPIOTE->EVENTS_IN[0]) {
    NRF_GPIOTE->EVENTS_IN[0] = 0;
  }
  if (NRF_GPIOTE->EVENTS_PORT) {
    NRF_GPIOTE->EVENTS_PORT = 0;
  }

} // GPIOTE_IRQHandler

} // extern "C"
*/
