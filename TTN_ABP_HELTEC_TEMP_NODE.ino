//------------------ Arduino LMIC -------------------------------
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_Sensor.h>
//---------------------------------------------------------------
//------------------ Heltec ESP32 LoraWAN -----------------------
#include "Arduino.h"

#include "board.h"
#include "LoRaMac.h"
#include <SPI.h>
#include <LoRa.h>
#include <Mcu.h>
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
//---------------------------------------------------------------
//-------------------CayenneLPP----------------------------------
#include <CayenneLPP.h>
#include <LPPDecode.h>


CayenneLPP lpp(51);
//---------------------------------------------------------------

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0xC7, 0x63, 0x7F, 0x16, 0xE4, 0x91, 0x8B, 0xE0, 0x59, 0xE2, 0xE3, 0xFB, 0xE0, 0x5B, 0x7D, 0x33 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x5F, 0x8B, 0x32, 0x60, 0xA2, 0x46, 0x73, 0x99, 0xEF, 0xBF, 0x2F, 0x26, 0xB0, 0x3E, 0xA2, 0x8D };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26001140; // <-- temp test

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 5 ;//1800;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 34, 35},
};
#define Vext  21
SSD1306  display(0x3c, SDA, SCL, RST_LED);

#define DHT_PIN 13
#define BUZZER_PIN 12

// DHT11 or DHT22
#define DHTTYPE DHT22

// Initialize dht
DHT_Unified dht(DHT_PIN, DHTTYPE);

uint32_t g_temperature = 0;    // global temperature

void LEDdisplayJOINING()
{
  digitalWrite(Vext,LOW);
  delay(50);
  display.wakeup();
  display.clear();
  display.drawString(58, 22, "JOINING...");
  display.display();
}
void LEDdisplayJOINED()
{
  digitalWrite(Vext,LOW);
  delay(50);
  display.wakeup();
  display.clear();
  display.drawString(64, 22, "JOINED");
  display.display();
  delay(1000);
  display.sleep();
  digitalWrite(Vext,HIGH);
}
void LEDdisplaySENDING()
{
  digitalWrite(Vext,LOW);
  delay(10);
  display.wakeup();
  display.init();
  delay(50);
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.clear();
  display.drawString(58, 22, "SENDING...");
  display.display();
}

void LEDdisplayTXCOMPLETE()
{
  digitalWrite(Vext,LOW);
  delay(10);
  display.wakeup();
  display.init();
  delay(50);
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.clear();
  display.drawString(58, 22, "TX Completed");
  display.display();
}

void LEDdisplaySTART()
{
  display.wakeup();
  display.init();
  delay(100);
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.clear();
  display.drawString(64, 11, "Leonard"); //"LORAWAN");
  display.drawString(64, 33, "Racal"); //"STARTING");
  display.display();
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            LEDdisplayJOINING();
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              LEDdisplayJOINED();
              
            }
            //OTAA Future implementation
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:

             Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.txrxFlags &(TXRX_DNW1 | TXRX_DNW2)) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
              if (LMIC.dataLen == 1) {
                uint8_t result = LMIC.frame[LMIC.dataBeg + 0];
                if (result == 0)  {
                  Serial.println("RELAIS 0");
                  digitalWrite(BUZZER_PIN, LOW); // la led est sur la pin 5 de l'arduino
                  
                }              
                if (result == 1)  {
                  Serial.println("RELAIS 1");
                  digitalWrite(BUZZER_PIN, HIGH);
                                  
                } 
              }
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            LEDdisplayTXCOMPLETE();
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE")); 
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        LEDdisplaySENDING();
        
        //---------------------Cayenne LPP-----------------------------
        // Prepare upstream data transmission at the next possible time.
        getTempHumidity();
        
        
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    pinMode(BUZZER_PIN, OUTPUT); // Configure as Output for Downlink, you can replace it with LED or a Relay
    pinMode(Vext,OUTPUT);
    digitalWrite(Vext, LOW);    // OLED USE Vext as power supply, must turn ON Vext before OLED init
    
    Serial.begin(115200);
    Serial.println(F("Starting"));
    
    
    dht.begin();

    // Print temperature sensor details.
    sensor_t sensor;
    
    dht.temperature().getSensor(&sensor);
    Serial.println(F("------------------------------------"));
    Serial.println(F("Temperature Sensor"));
    Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
    Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
    Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
    Serial.println(F("------------------------------------"));
    
    

    // Print humidity sensor details.
    dht.humidity().getSensor(&sensor);
    Serial.println(F("Humidity Sensor"));
    Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
    Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
    Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
    Serial.println(F("------------------------------------"));
    
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
 
   

    // Start OLED
    LEDdisplaySTART();
    
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14); //DR_SF7

    LMIC_setClockError (MAX_CLOCK_ERROR * 10/100);  

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}

void getTempHumidity() {
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  g_temperature = event.temperature;

  if (isnan(g_temperature)) {
    Serial.println(F("Error reading temperature!"));
  }

  else {
    Serial.print(F("Temperature: "));
    Serial.print(g_temperature);
    Serial.println(F("°C"));
  }

  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  uint32_t humidity = event.relative_humidity;
  if (isnan(humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(humidity);
    Serial.println(F("%"));
  }
  
  // reset LPP
  lpp.reset();

  lpp.addTemperature(1, g_temperature);
  
  lpp.addRelativeHumidity(2, humidity);

  lpp.addDigitalOutput(3, 0); //channel 3 , pin 12 of Heltec Lora32

  
}
