# HeltecLora32_ABP_LoRaWAN

This code is used for Bitstoc LoRaWAN Kit.
As of the moment the kit is using ABP (Activation-By-Personalisation) to connect to TTN or Bitstoc.IO

Replace the following NWKSKEY, APPSKEY and DEVADDR

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

# Cayenne LPP Protocol

This End Node code uses Cayenne LPP (Low Power Payload) to send Sensor signal to TTN or Bitstoc.IO

Bitstoc.IO is currently in development upon making this repository.

Make sure you configure your TTN to have Cayenne LPP payload and Cayenne Integration in Application.
