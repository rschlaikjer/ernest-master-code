#include <SPI.h>
#include <Wire.h>
#include <Ethernet.h>
#include <SFE_BMP180.h>
#include <LiquidCrystal_I2C.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "printf.h"

#define MAX_CLIENTS 16

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
RF24 radio(PIN_R_CE, PIN_R_CSN);
// Pipe address to communicate on
uint64_t rf_pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

// delay between updates, millis
const unsigned long update_interval = 30*1000;
// Last time we sent temp data, millis
unsigned long last_update_time = 0;

struct datagram {
    double temp;
    double pressure;
    uint8_t node_id;
};

// State
static uint32_t readings_handled = 0;
float node_temps[MAX_CLIENTS];
float node_pressures[MAX_CLIENTS];
short node_updated[MAX_CLIENTS];

// Function prototypes
void handlePendingData();
void postTempData(short node_id, float temp, float pressure){
    printf("Node: %d, temp: ", node_id);
    Serial.print(temp);
    Serial.print(", pressure ");
    Serial.print(pressure);
    Serial.println(".");
}

void setup() {
    // Setup serial
    Serial.begin(9600);
    printf_begin();

    // Set output pins for the counters
    pinMode(PIN_CNT_1, OUTPUT);
    pinMode(PIN_CNT_2, OUTPUT);
    pinMode(PIN_CNT_3, OUTPUT);
    pinMode(PIN_CNT_4, OUTPUT);

    // Setup RF
    radio.begin();
    radio.enableAckPayload();
    radio.setRetries(15, 15);
    radio.setPayloadSize(16);
    radio.openWritingPipe(rf_pipes[1]);
    radio.openReadingPipe(1, rf_pipes[0]);
    radio.startListening();

    // Dump details for debugging
    radio.printDetails();

    // Initialize the node data table
    for (int i = 0; i < MAX_CLIENTS; i++){
        node_updated[i] = 0;
    }
}

/*
 * Main loop:
 * - Check for pending node transmissions
 * - Iterate over the node data we have stored, and see if any of it is dirty
 *     - If so, post it to the server
 * - Adjust relay based on response
 */
void loop() {
    // Grab new node data
    handlePendingData();

    // Check for changed data
    for (int i = 0; i < MAX_CLIENTS; i++){
        // For changed node data, POST it
        if (node_updated[i]) {
            postTempData(i, node_temps[i], node_pressures[i]);
        }
        node_updated[i] = 0;
    }
    delay(1000);
}

// Check for pending data, and mark readings as to-be-updated.
void handlePendingData(){
    if (radio.available()){
        // Dump the payloads until we've gotten everything
        static struct datagram node_data;
        bool done = false;
        while (!done){
            done = radio.read(&node_data, sizeof(struct datagram));
            // Update the node data array and flag the data as changed
            readings_handled++;
            node_temps[node_data.node_id] = node_data.temp;
            node_pressures[node_data.node_id] = node_data.pressure;
            node_updated[node_data.node_id] = 1;
        }

        // Ack with the number of node broadcasts we have handled
        radio.writeAckPayload( 1, &readings_handled, sizeof(readings_handled) );
    }
}
