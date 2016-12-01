/*******************************************************************************
  Created by Andres Sabas @ Electronic Cats 2016
  Based on Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

  LoRaCat: Track

  Example
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <SoftwareSerial.h>
#include <TinyGPS++.h>
TinyGPSPlus gps;
SoftwareSerial ss(5, 4);
int32_t lat;
int32_t lon;
bool join_flag=false;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0xD4, 0x0D, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
//static const u1_t PROGMEM APPKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
static const u1_t PROGMEM APPKEY[16] = { 0xC4, 0x90, 0x52, 0x2D, 0x08, 0xD3, 0x82, 0x9D, 0xC0, 0x38, 0x8D, 0x73, 0xEA, 0xCD, 0x13, 0x0F };
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

static char mydata[] = "ping";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;
// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 3, 4},
};

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
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
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));

      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      join_flag=true;
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
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
        Serial.print(F("Data: "));
        Serial.write(LMIC.frame, LMIC.dataLen);
        Serial.println();
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
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

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    if (gps.location.isValid()) {
      // Pack GPS coordinates according to https://www.thethingsnetwork.org/forum/t/best-practices-when-sending-gps-location-data/1242/13
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.println(gps.location.lng(), 6);
      uint8_t coords[6]; // 6*8 bits = 48
      lat = gps.location.lat() * 10000;
      lon = (-1) * (gps.location.lng()) * 10000;

      // Pad 2 int32_t to 6 8uint_t, skipping the last byte (x >> 24)
      coords[0] = lat;
      coords[1] = lat >> 8;
      coords[2] = lat >> 16;

      coords[3] = lon;
      coords[4] = lon >> 8;
      coords[5] = lon >> 16;

      LMIC_setTxData2(1, coords, sizeof(coords), 0);
      Serial.println(F("Packet queued"));
    } else {
      Serial.println(F("INVALID"));
      strcpy(mydata, "NOPE");
      LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
    }
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("Starting Plus"));
  ss.begin(9600);

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  //Configuration for Multitech Conduit SubBand 7
  for (int channel=0; channel<72; ++channel) {
      LMIC_disableChannel(channel);
    }

      LMIC_enableChannel(48);
      LMIC_enableChannel(49);
      LMIC_enableChannel(50);
      LMIC_enableChannel(51);
      LMIC_enableChannel(52);
      LMIC_enableChannel(53);
      LMIC_enableChannel(54);
      LMIC_enableChannel(55);
      LMIC_enableChannel(70);

      // TTN uses SF9 for its RX2 window. This is configured in the
      // join accept message, but the LMIC library does not currently
      // process this part of the join accept yet (see Arduino-LMIC issue #20).
      LMIC.dn2Dr = DR_SF9;

      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);

      // Use a fixed data rate of SF9 (not sure if tx power is
      // actually used). SF9 is the lowest datarate that (withing the
      // TTN fair-usage-policy of 30 seconds of airtime per day)
      // allows us to send at least 4 packets every hour.
      LMIC_setDrTxpow(DR_SF7, 14);

      // Let LMIC compensate for +/- 1% clock error
      LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
  if (join_flag==true) {
    if (ss.available()) {
      //int c = ss.read();
      gps.encode(ss.read());
    }
  }
  os_runloop_once();
}
