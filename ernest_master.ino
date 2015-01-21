#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>

// SPI pins for 2.4GHz transceiver
#define PIN_R_CE 8
#define PIN_R_CSN 7

// Output pins for client LED counters
#define PIN_CNT_1 6
#define PIN_CNT_2 5
#define PIN_CNT_3 4
#define PIN_CNT_4 3

//// Hardware abstractions

// nRF24L01 radio
RF24 radio(PIN_R_CSN, PIN_R_CE);
// Pipe address to communicate on
uint64_t rf_pipe = 0xF0F0F0F0E1LL;

// delay between updates, millis
const unsigned long update_interval = 30*1000;
// Last time we sent temp data, millis
unsigned long last_update_time = 0;

struct datagram {
    float temp;
    float pressure;
    uint8_t node_id;
};

void setup() {
    // Setup serial
    Serial.begin(9600);

    // Set output pins for the counters
    pinMode(PIN_CNT_1, OUTPUT);
    pinMode(PIN_CNT_2, OUTPUT);
    pinMode(PIN_CNT_3, OUTPUT);
    pinMode(PIN_CNT_4, OUTPUT);

    // Setup RF
    radio.begin();
    radio.enableAckPayload();
    //radio.setRetries(15, 15);
    //radio.setPayloadSize(8);
    radio.openReadingPipe(1, rf_pipe);

    // Dump details for debugging
    radio.printDetails();
}

/*
 * Main loop:
 */
void loop() {
    if(millis() - last_update_time > update_interval) {
        last_update_time = millis();
    }
    delay(1000);
}
