/* -*- mode: c++; c-basic-offset: 2; -*- */

#include <Arduino.h>

#include <Dns.h>
#include <ethernet_comp.h>
#include <UIPClient.h>
#include <UIPEthernet.h>
#include <UIPServer.h>
#include <UIPUdp.h>

#include "ntp.h"

#include <Wire.h>
#include <AltSoftSerial.h>

#include <avr/pgmspace.h>

#include "credentials.h"

#define RTC_ADDR 0xe8

#define INVERTER_MONITOR_VERSION "0.1"
#define MAX_NUM_INVERTERS 2
#define MAX_DATA_FIELDS 32
#define MAX_SERIALNUM_CHARS 24

#define INVERTER_POLL_MILLIS 60000
#define INVERTER_POLL_NEW_INVERTERS_MILLIS 300000

#define DESTINATION_ALL 0x00

uint8_t mac[] = { 0xde, 0xad, 0xbe, 0xef, 0xfe, 0xed};
uint8_t myIP[] = {192, 168, 0, 50};

uint8_t myAddress;

uint8_t rcvState;
uint8_t rcvData[256];
uint8_t rcvHeader[7];
uint16_t rcvChksum;
uint8_t rcvPtr;

IPAddress lastRequestor;
uint16_t lastPort;

const char tag_na[] PROGMEM = "n/a";
const char tag_0[] PROGMEM = "temp";
const char tag_1[] PROGMEM = "vpv1";
const char tag_2[] PROGMEM = "vpv2";
const char tag_4[] PROGMEM = "ipv1";
const char tag_5[] PROGMEM = "ipv2";
const char tag_7[] PROGMEM = "Hetotal";
const char tag_8[] PROGMEM = "Letotal";
const char tag_9[] PROGMEM = "Hhtotal";
const char tag_10[] PROGMEM = "Lhtotal";
const char tag_11[] PROGMEM = "pac";
const char tag_12[] PROGMEM = "mode";
const char tag_13[] PROGMEM = "etoday";
const char tag_32[] PROGMEM = "surTemp";
const char tag_33[] PROGMEM = "bdTemp";
const char tag_34[] PROGMEM = "irr";
const char tag_35[] PROGMEM = "windSpeed";
const char tag_56[] PROGMEM = "waitTime";
const char tag_57[] PROGMEM = "tempFaultValue";
const char tag_58[] PROGMEM = "pv1FaultValue";
const char tag_59[] PROGMEM = "pv2FaultValue";
const char tag_61[] PROGMEM = "gfciFaultValue";
const char tag_62[] PROGMEM = "HerrorMessage";
const char tag_63[] PROGMEM = "LerrorMessage";
const char tag_64[] PROGMEM = "vpv";
const char tag_65[] PROGMEM = "iac";
const char tag_66[] PROGMEM = "vac";
const char tag_67[] PROGMEM = "fac";
const char tag_68[] PROGMEM = "pac";
const char tag_69[] PROGMEM = "zac";
const char tag_70[] PROGMEM = "ipv";
const char tag_71[] PROGMEM = "Hetotal_R";
const char tag_72[] PROGMEM = "Letotal_R";
const char tag_73[] PROGMEM = "Hhtotal";
const char tag_74[] PROGMEM = "Lhtotal";
const char tag_75[] PROGMEM = "poweron";
const char tag_76[] PROGMEM = "mode";
const char tag_120[] PROGMEM = "gvFaultValue";
const char tag_121[] PROGMEM = "gfFaultValue";
const char tag_122[] PROGMEM = "gzFaultValue";
const char tag_123[] PROGMEM = "tempFaultValue";
const char tag_124[] PROGMEM = "pv1FaultValue";
const char tag_125[] PROGMEM = "gfciFaultValue";
const char tag_126[] PROGMEM = "HerrorMessage";
const char tag_127[] PROGMEM = "LerrorMessage";

PGM_P const tags[] PROGMEM = {
  // 0x00
  tag_0, tag_1, tag_2, tag_na, tag_4, tag_5, tag_na, tag_7,
  tag_8, tag_9, tag_10, tag_11, tag_12, tag_13, tag_na, tag_na,
  // 0x10
  tag_na, tag_na, tag_na, tag_na, tag_na, tag_na, tag_na, tag_na,
  tag_na, tag_na, tag_na, tag_na, tag_na, tag_na, tag_na, tag_na,
  // 0x20
  tag_32, tag_33, tag_34, tag_35, tag_na, tag_na, tag_na, tag_na,
  tag_na, tag_na, tag_na, tag_na, tag_na, tag_na, tag_na, tag_na,
  // 0x30
  tag_na, tag_na, tag_na, tag_na, tag_na, tag_na, tag_na, tag_na,
  tag_56, tag_57, tag_58, tag_59, tag_na, tag_61, tag_62, tag_63,
  // 0x40
  tag_64, tag_65, tag_66, tag_67, tag_68, tag_69, tag_70, tag_71,
  tag_72, tag_73, tag_74, tag_75, tag_76, tag_na, tag_na, tag_na,
  // 0x50
  tag_na, tag_na, tag_na, tag_na, tag_na, tag_na, tag_na, tag_na,
  tag_na, tag_na, tag_na, tag_na, tag_na, tag_na, tag_na, tag_na,
  // 0x60
  tag_na, tag_na, tag_na, tag_na, tag_na, tag_na, tag_na, tag_na,
  tag_na, tag_na, tag_na, tag_na, tag_na, tag_na, tag_na, tag_na,
  // 0x70
  tag_na, tag_na, tag_na, tag_na, tag_na, tag_na, tag_na, tag_na,
  tag_120, tag_121, tag_122, tag_123, tag_124, tag_125, tag_126, tag_127,
};

const float scale_factors[] PROGMEM = {
  // 0x00
  0.1, 0.1, 0.1, 0.0, 0.1, 0.1, 0.0, 0.1,
  0.1, 1.0, 1.0, 1.0, 0.0, 0.01, 1.0, 1.0,
  // 0x10
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  // 0x20
  0.1, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  // 0x30
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  1.0, 0.1, 0.1, 0.1, 0.0, 0.001, 0.0, 0.0,
  // 0x40
  0.1, 0.1, 0.1, 0.01, 1.0, 0.0001, 0.1, 0.1,
  0.1, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  // 0x50
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  // 0x60
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  // 0x70
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
  0.1, 0.01, 0.0001, 0.1, 0.1, 0.001, 0.0, 0.0,
};

#define MAX_TAG 127

typedef struct inverter_t {
  // If not yet registered, id == 0
  uint8_t id;
  long last_poll;
  uint8_t serial_len;
  uint8_t serial[MAX_SERIALNUM_CHARS];
  uint8_t num_data_fields;
  uint8_t data_fields[MAX_DATA_FIELDS];
} inverter_t;

inverter_t inverters[MAX_NUM_INVERTERS];
uint8_t nextInverterId;

long last_new_inverter_check;
uint8_t waiting_for_new_inverter;

#define INVERTER_VALID(x) (inverters[ x ].id != 0)

EthernetClient client;
EthernetUDP udp;
EthernetUDP udpNtp;

// So long as debug doesn't start with 0xaa 0x55 should be right...
AltSoftSerial debugSerial;

void send_request(uint8_t destAddress, uint8_t ctlCode, uint8_t funcCode, uint8_t len, uint8_t *data);

void handle_good_packet();

static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }

////////////////////////////////////////////////////////////////////////
// This part copied from
// https://github.com/adafruit/RTClib/blob/master/RTClib.cpp
////////////////////////////////////////////////////////////////////////

#define SECONDS_FROM_1970_TO_2000 946684800
const uint8_t daysInMonth [] PROGMEM = { 31,28,31,30,31,30,31,31,30,31,30,31 };

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
    if (y >= 2000)
        y -= 2000;
    uint16_t days = d;
    for (uint8_t i = 1; i < m; ++i)
        days += pgm_read_byte(daysInMonth + i - 1);
    if (m > 2 && y % 4 == 0)
        ++days;
    return days + 365 * y + (y + 3) / 4 - 1;
}

static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
    return ((days * 24L + h) * 60 + m) * 60 + s;
}

////////////////////////////////////////////////////////////////////////
// End part copied from ladyada
////////////////////////////////////////////////////////////////////////

uint32_t read_current_time_unix_epoch() {
  uint32_t time;
  uint8_t hh, mm, ss;
  uint8_t y, m, d, b;
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(0xe8, 7);

  // read seconds
  b = Wire.read();
  ss = bcd2bin(b);

  // read minutes
  b = Wire.read();
  mm = bcd2bin(b);

  // read hours
  b = Wire.read();
  hh = bcd2bin(b);

  // ignore daynumber
  b = Wire.read();
  
  // read day-of-month
  b = Wire.read();
  d = bcd2bin(b);

  // read month+century
  b = Wire.read();
  m = bcd2bin(b & 0x7f);
  
  // read year
  b = Wire.read();
  y = bcd2bin(b);

  time = time2long(date2days(y, m, d), hh, mm, ss) + SECONDS_FROM_1970_TO_2000;

  return time;
}

void sync_rtc() {
  uint32_t time = ntpUnixTime(udpNtp);

  uint8_t hh, mm, ss;
  uint8_t y, m, d;

  time -= SECONDS_FROM_1970_TO_2000;    // bring to 2000 timestamp from 1970

  ss = time % 60;
  time /= 60;
  mm = time % 60;
  time /= 60;
  hh = time % 24;
  uint16_t days = time / 24;
  uint8_t leap;
  for (y = 0; ; ++y) {
    leap = y % 4 == 0;
    if (days < 365 + leap)
      break;
    days -= 365 + leap;
  }
  for (m = 1; ; ++m) {
    uint8_t daysPerMonth = pgm_read_byte(daysInMonth + m - 1);
    if (leap && m == 2)
      ++daysPerMonth;
    if (days < daysPerMonth)
      break;
    days -= daysPerMonth;
  }
  d = days + 1;

  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0);
  Wire.write(bin2bcd(ss));
  Wire.write(bin2bcd(mm));
  Wire.write(bin2bcd(hh));
  Wire.write(0); // Don't care about day-of-week
  Wire.write(bin2bcd(d));
  Wire.write(bin2bcd(m));
  Wire.write(bin2bcd(y));
  Wire.endTransmission();
}

void start_rtc() {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x0e); // Start of control regs
  Wire.write(0x04); /* Oscillator on, SQW off when on battery,
		     * SQW 1Hz, SQW/INT pin is INT, both alarms disabled */
  Wire.endTransmission();
}

#define RCV_SOURCE rcvHeader[1]
#define RCV_DEST   rcvHeader[2]
#define RCV_CODE   rcvHeader[4]
#define RCV_FUNC   rcvHeader[5]
#define RCV_DATALEN rcvHeader[6]

void process_serial_input() {
  uint8_t c = Serial.read();

  switch (rcvState) {
  case 0:
    // Awaiting start byte 1
    if (c == 0xaa) {
      rcvChksum += c;
      rcvState = 1;
    }
    break;
  case 1:
    // Awaiting start byte 2
    if (c == 0x55) {
      rcvChksum += c;
      rcvState = 2;
      rcvPtr = 0;
    } else {
      rcvState = 0;
      rcvChksum = 0;
    }
    break;
  case 2:
    // read header
    rcvHeader[rcvPtr++] = c;
    rcvChksum += c;
    if (rcvPtr == 7) {
      rcvPtr = 0;
      rcvState = 3;
      if (RCV_DATALEN == 0) {
	rcvState = 4;
      }
    }
    break;
  case 3:
    // read data
    rcvData[rcvPtr++] = c;
    rcvChksum += c;
    if (rcvPtr == RCV_DATALEN) {
      rcvState = 4;
    }
    break;
  case 4:
    // read chksum byte 1
    if (c == (rcvChksum >> 8)) {
      // Checksum byte 1 OK
      rcvState = 5;
    } else {
      // Checksum byte 1 bad
      rcvState = 0;
      rcvChksum = 0;
    }
    break;
  case 5:
    // read chksum byte 2
    if (c == (rcvChksum & 0xff)) {
      handle_good_packet();
    }
    rcvState = 0;
    rcvChksum = 0;
    break;
  default:
    rcvState = 0;
    rcvChksum = 0;
  }

}

#define SEND_PART(x) Serial.write(x); cksum += x;

void send_request(uint8_t destAddress, uint8_t ctlCode, uint8_t funcCode, uint8_t len, uint8_t *data) {
  uint16_t cksum = 0;

  SEND_PART(0xaa);
  SEND_PART(0x55);
  SEND_PART(myAddress);
  SEND_PART(0x00);
  SEND_PART(0x00);
  SEND_PART(destAddress);
  SEND_PART(ctlCode);
  SEND_PART(funcCode);
  SEND_PART(len);
  for (uint8_t j = 0; j < len; j++) {
    SEND_PART(data[j]);
  }

  Serial.write((cksum >> 8) & 0xff);
  Serial.write(cksum & 0xff);
}

//void writeInverterSerialToDebug(uint8_t i) {
//  for (uint8_t j = 0; j < inverters[i].serial_len; j++) {
//    debugSerial.write(inverters[i].serial[j]);
//  }
//}

uint8_t inverter_id_for_source() {
  for (uint8_t j = 0; j < MAX_NUM_INVERTERS; j++) {
    if (inverters[j].id == RCV_SOURCE) {
      return j;
    }
  }
  return -1;
}

void allocate_id_to_inverter() {
  uint8_t i = 0;
  while (i < MAX_NUM_INVERTERS &&
	 inverters[i].id != 0)
    i++;
  if (i == MAX_NUM_INVERTERS) {
    //debugSerial.println(F("Too many inverters, cannot register"));
    return;
  }
  inverters[i].id = nextInverterId++;

  uint8_t s_len = RCV_DATALEN;
  if (s_len > MAX_SERIALNUM_CHARS) {
    //debugSerial.println(F("WARNING: inverter serial number is too long"));
  }
  inverters[i].serial_len = s_len;
  
  //debugSerial.print(F("Allocating ID "));
  //debugSerial.print(inverters[i].id);
  //debugSerial.print(F(" to inverter "));

  uint8_t j;
  for (j = 0; j < RCV_DATALEN; j++) {
    if (j < s_len) {
      inverters[i].serial[j] = rcvData[j];
    }
    //debugSerial.write(rcvData[j]);
  }

  // Bit ugly to add stuff to the end of the received packet here,
  // but we do it so that we can still allocate an ID to inverters
  // whose serial number is bigger than we can store.
  rcvData[j] = inverters[i].id;

  //debugSerial.println();
  send_request(DESTINATION_ALL, 0x10, 0x01, RCV_DATALEN + 1, rcvData);
}

void request_data_map() {
  send_request(RCV_SOURCE, 0x11, 0x00, 0, NULL);
}

void save_inverter_data_map() {
  uint8_t i = inverter_id_for_source();
  uint8_t n = RCV_DATALEN;
  if (n > MAX_DATA_FIELDS) {
    n = MAX_DATA_FIELDS;
    //debugSerial.print(F("WARNING: inverter "));
    //writeInverterSerialToDebug(i);
    //debugSerial.println(F(" has too many data fields."));
  }
  inverters[i].num_data_fields = n;
  for (uint8_t j = 0; j < n; j++) {
    inverters[i].data_fields[j] = rcvData[j];
  }
}

char prvTagHalf[15];
uint32_t prvTagVal;

uint32_t sample_timestamp;
uint8_t sample_inverter;

void handle_data_start(const char *name) {
  client.print(F("pvmonitor."));
  client.print(name);
  client.write(' ');
  client.print(sample_timestamp);
  client.write(' ');
}

void handle_data_end() {
  client.print(F(" inverter="));
  for (uint8_t i = 0; i < inverters[i].serial_len; i++) {
    uint8_t c = inverters[sample_inverter].serial[i];
    if (c >= 0x20) { // only send printable characters
      client.write(c);
    }
  }
  client.write('\n');
}

void handle_float_data(const char *name, float value) {
  handle_data_start(name);
  client.print(value);
  handle_data_end();
}

void handle_integer_data(const char *name, uint16_t value) {
  handle_data_start(name);
  client.print(value);
  handle_data_end();
}

void handle_double_width_integer_data(const char *name, uint32_t value) {
  handle_data_start(name);
  client.print(value);
  handle_data_end();
}

void parse_double_width_data_half(const char *tag, uint16_t val, float scaleFactor) {
  if (strncmp(prvTagHalf, &(tag[1]), 15) == 0) {
    uint32_t tmp = val;
    if (tag[0] == 'H') {
      tmp <<= 16;
    }
    prvTagVal += tmp;
    if (scaleFactor == 0.0) {
      handle_double_width_integer_data(prvTagHalf, prvTagVal);
    } else {
      float tmp = scaleFactor * prvTagVal;
      handle_float_data(prvTagHalf, tmp);
    }
  } else {
    strncpy(prvTagHalf, &(tag[1]), 15);
    prvTagVal = val;
    if (tag[0] == 'H') {
      prvTagVal <<= 16;
    }
  }
}
void parse_data_bytes(uint8_t tag, uint8_t v1, uint8_t v2) {
  char tmp[16];
  uint16_t v = (v1 << 8) | v2;

  float scaleFactor = 0.0;

  if (tag <= MAX_TAG) {
    // This is a tag we know about
    const char *tag_addr = (PGM_P)pgm_read_word(&(tags[tag]));
    strncpy_P(tmp, tag_addr, 16);
    tmp[15] = 0;
    scaleFactor = pgm_read_float(&(scale_factors[tag]));

    if (tmp[0] == 'H' || tmp[0] == 'L') {
      parse_double_width_data_half(tmp, v, scaleFactor);
    }
  } else {
    // Not a tag we know about; make a name (tag_%d) without
    // using sprintf() which takes up too much flash space
    tmp[0] = 't'; tmp[1] = 'a'; tmp[2] = 'g'; tmp[3] = '_';
    tmp[4] = (tag >> 4) + 0x30;
    if (tmp[4] > 0x39) tmp[4] += 7;
    tmp[5] = (tag & 0xf) + 0x30;
    if (tmp[5] > 0x39) tmp[5] += 7;
    tmp[6] = 0;
  }
  
  if (scaleFactor == 0.0) {
    handle_integer_data(tmp, v);
  } else {
    float fv = scaleFactor * v;
    handle_float_data(tmp, fv);
  }
}

void process_inverter_data() {
  uint8_t i = inverter_id_for_source();
  //debugSerial.print(F("Data packet from "));
  //writeInverterSerialToDebug(i);
  //debugSerial.println(":");
  sample_timestamp = read_current_time_unix_epoch();
  sample_inverter = i;
  client.connect(LOG_HOST, LOG_PORT);
  for (uint8_t d = 0;
       (d < inverters[i].num_data_fields
	&&
	(2*d) < RCV_DATALEN);
       d++) {
    parse_data_bytes(inverters[i].data_fields[d], rcvData[2*d], rcvData[2*d+1]);
  }
  client.stop();
}

void handle_good_packet() {
#if 0
  debugSerial.print(F("HANDLE: "));
  debugSerial.print(F("S="));
  debugSerial.print(RCV_SOURCE);
  debugSerial.print(F(", D="));
  debugSerial.print(RCV_DEST);
  debugSerial.print(F(", C="));
  debugSerial.print(RCV_CODE);
  debugSerial.print(F(", F="));
  debugSerial.print(RCV_FUNC);
  debugSerial.print(F(", Sz="));
  debugSerial.print(RCV_DATALEN);
  debugSerial.print(F(", Da="));
  for (uint8_t i = 0; i < RCV_DATALEN; i++) {
    debugSerial.print(rcvData[i]);
    debugSerial.print(F(" "));
  }
  debugSerial.println();
#endif

  udp.beginPacket(lastRequestor, lastPort);
  udp.write(0xaa);
  udp.write(0x55);
  for (uint8_t i = 0; i < 7; i++)
    udp.write(rcvHeader[i]);
  for (uint8_t i = 0; i < RCV_DATALEN; i++)
    udp.write(rcvData[i]);
  udp.endPacket();

  switch(RCV_CODE) {
  case 0x10: // Register
    switch (RCV_FUNC) {
    case 0x00:
      // Another bus master is looking for unregistered inverters, but
      // is sending it on the wrong signal pair. Uh-oh...
      break;
    case 0x01:
      // Another bus master is allocating an inverter an ID, but is
      // sending it on the wrong signal pair. Uh-oh...
      break;
    case 0x80:
      // An inverter is responding to a query for unregistered inverters.
      // Allocate it an ID
      waiting_for_new_inverter = 0;
      //debugSerial.println(F("ALLOCATE_ID_TO_INVERTER"));
      allocate_id_to_inverter();
      break;
    case 0x81:
      // Inverter acknowledging it's new ID
      //debugSerial.println(F("ALLOCATE_ID_TO_INVERTER_ACK"));
      request_data_map();
      break;
    }
    break;
  case 0x11: // Read
    switch (RCV_FUNC) {
    case 0x00:
      // Another bus master wants stuff
    case 0x01:
      // Another bus master wants stuff
    case 0x02:
      // Another bus master wants stuff
      break;
    case 0x80:
      // Inverter is telling us what it's read-only data map is
      save_inverter_data_map();
      break;
    case 0x81:
      // Inverter is telling us what it's read-write data map is
      break;
    case 0x82:
      // Inverter is telling us what it's data registers contain
      // ACTUAL DATA PACKET!
      process_inverter_data();
      break;
    }
    break;
  case 0x12: // Write
    // We don't mess with inverter settings
    break;
  case 0x13: // Execute
    // We don't mess with inverter settings
    break;
  }
}

#define MAX_UDP_SEND_BUFFER 64
uint8_t udpData[MAX_UDP_SEND_BUFFER];
void process_ethernet_input_if_available() {
  int packetSize = udp.parsePacket();

  //if (packetSize > 0) {
    //debugSerial.print(F("PEIIA: packetSize="));
    //debugSerial.println(packetSize);
  //}

  if (packetSize > 0 && packetSize <= MAX_UDP_SEND_BUFFER) {
    int readLen = min(packetSize, MAX_UDP_SEND_BUFFER);
    udp.read(udpData, readLen);
    lastRequestor = udp.remoteIP();
    lastPort = udp.remotePort();
    for (int i = 0; i < readLen; i++) {
      //debugSerial.print(F("d["));
      //debugSerial.print(i);
      //debugSerial.print(F("]="));
      //debugSerial.println(udpData[i]);
      Serial.write(udpData[i]);
    }
  }
}

void start_check_for_new_inverters() {
  last_new_inverter_check = millis();
  waiting_for_new_inverter = 1;
  send_request(DESTINATION_ALL, 0x10, 0x00, 0, NULL);
}

uint8_t is_time_to_check_inverter(uint8_t inverterNum) {
  long now = millis();
  return (now - inverters[inverterNum].last_poll > INVERTER_POLL_MILLIS);
}

uint8_t is_time_to_check_for_new_inverters() {
  long now = millis();
  long time_to_wait = INVERTER_POLL_NEW_INVERTERS_MILLIS;
  if (!waiting_for_new_inverter) {
    // If we have received a response from our last new inverter poll...
    time_to_wait = 5000;
    // keep looking for new inverters until we're full
  }
  return ((now - last_new_inverter_check) > time_to_wait);
}

void start_check_inverter(uint8_t inverterNum) {
  send_request(inverters[inverterNum].id, 0x11, 0x02, 0, NULL);
  inverters[inverterNum].last_poll = millis();
}

void setup() {
  // RS-485 connection to inverter - Pins 1&2 (D0/D1) / PD0&P1D
  Serial.begin(9600); 

  // Debug connection to computer - Pins 14&15 (RX D8/TX D9) / PB0&PB1
  debugSerial.begin(9600); 
  debugSerial.print(F("Inverter Monitor v"));
  debugSerial.println(INVERTER_MONITOR_VERSION);

  UIPEthernet.begin(mac, myIP);

  delay(1000);
  debugSerial.println(F("start_rtc"));
  start_rtc();
  delay(1000);

  debugSerial.println(F("start_rtc"));
  delay(1000);
  sync_rtc();
  delay(1000);
  
  debugSerial.println(F("Testing data write..."));
  //sample_timestamp = read_current_time_unix_epoch();

  sample_inverter = 0;
  inverters[0].serial_len=1;
  inverters[0].serial[0] = 'T';
  client.connect(LOG_HOST, LOG_PORT);
  parse_data_bytes(0, 0x00, 0xd7);
  parse_data_bytes(13, 0x03, 0xdb);
  parse_data_bytes(1, 0x08, 0xa1);
  parse_data_bytes(2, 0x08, 0xba);
  parse_data_bytes(4, 0x00, 0x05);
  parse_data_bytes(5, 0x00, 0x05);
  parse_data_bytes(65, 0x00, 0x08);
  parse_data_bytes(66, 0x09, 0xb1);
  parse_data_bytes(67, 0x13, 0x86);
  parse_data_bytes(68, 0x00, 0xc2);
  parse_data_bytes(69, 0xff, 0xff);
  parse_data_bytes(71, 0x00, 0x00);
  parse_data_bytes(72, 0x08, 0x22);
  parse_data_bytes(73, 0x00, 0x00);
  parse_data_bytes(74, 0x00, 0xdb);
  client.stop();
  debugSerial.println(F("done."));

  last_new_inverter_check = 0;
  for (int i = 0; i < MAX_NUM_INVERTERS; i++) {
    inverters[i].id = 0;
    inverters[i].last_poll = 0;
  }

  rcvState = 0;
  rcvChksum = 0;

  myAddress = 0x01;
  nextInverterId = 0x10;

  
  // Tell all inverters to deregister and respond to reregistration requests
  for (uint8_t i = 0; i < 4; i++) {
    send_request(DESTINATION_ALL, 0x10, 0x04, 0, NULL);
    delay(1000);
  }
  sync_rtc();

  start_check_for_new_inverters();
}

void loop() {
  //debugSerial.print(F("loop: t="));
  //debugSerial.println(millis());
  
  UIPEthernet.maintain();

  while (Serial.available()) {
    process_serial_input();
  }

  process_ethernet_input_if_available();
  
  if (is_time_to_check_for_new_inverters()) {
    start_check_for_new_inverters();
  }

  for (int i = 0; i < MAX_NUM_INVERTERS; i++) {
    if (INVERTER_VALID(i) && is_time_to_check_inverter(i)) {
      start_check_inverter(i);
    }
  }

  delay(250);
}
