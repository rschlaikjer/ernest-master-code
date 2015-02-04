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

//// Hardware abstractions

// nRF24L01 radio
RF24 radio(PIN_R_CE, PIN_R_CSN);
// Pipe address to communicate on
uint64_t rf_pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

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

// Relay
#define RELAY_PIN 6
bool G_FURNACE_ON = false;

// LCD
LiquidCrystal_I2C lcd(0x3F,20,4); //Addr: 0x3F, 20 chars & 4 lines

// Ethernet
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
EthernetClient client;
char server[] = "nest.rhye.org";
unsigned long lastConnectionTime = 0;           // last time we connected to the server, in millis
boolean lastConnected = false;

// Optional on-board sensor
#define ALTITUDE 4.0
SFE_BMP180 sensor;
unsigned long last_temp_conn = 0;
unsigned temp_poll_time = 15 * 1000;
bool G_HAS_OWN_SENSOR = true;

// Function prototypes
void handlePendingData();
void postTempData(short node_id, float temp, float pressure);
void makeHTTPRequest(short node_id, float temp, float pressure);
void parseHTTPResponse();
void updateLocalTemps();

void setup() {
    // Setup serial
    Serial.begin(9600);
    printf_begin();

    // LCD display
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Booting...");

    // Ethernet
    Ethernet.begin(mac);

    // Relay
    pinMode(RELAY_PIN, OUTPUT);

    // Temp sensor (if present)
    if (!sensor.begin()){
        G_HAS_OWN_SENSOR = false;
    }

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

    // If we have our own sensor, get data from it too
    if (G_HAS_OWN_SENSOR){
        if (millis() - last_temp_conn > temp_poll_time){
            updateLocalTemps();
        }
    }

    delay(1000);
}

void update_lcd(){
    lcd.setCursor(0, 0);
    lcd.print("Temp ");
    //lcd.print(temp);
    lcd.print("C ");
    //lcd.print((temp * 9.0 / 5.0) + 32);
    lcd.print("F");

    lcd.setCursor(0, 1);
    lcd.print("Furnace ");
    if (G_FURNACE_ON){
        lcd.print("on ");
    } else {
        lcd.print("off");
    }
}

void postTempData(short node_id, float temp, float pressure){
    printf("Node: %d, temp: ", node_id);
    Serial.print(temp);
    Serial.print(", pressure ");
    Serial.print(pressure);
    Serial.println(".");

    // Post the data to the webserver
    makeHTTPRequest(node_id, temp, pressure);

    // Wait for the response to come back
    while (!client.available()){}

    // Parse & potentially to update the relay
    parseHTTPResponse();
}

void makeHTTPRequest(short node_id, float temp, float pressure){
    if (client.connect(server, 80)){
        client.print("GET /control?");
        client.print("node_id=");
        client.print(node_id, 2);
        client.print("&temp=");
        client.print(temp, 2);
        client.print("&pressure=");
        client.print(pressure, 2);
        client.println(" HTTP/1.1");
        client.println("Host: nest.rhye.org");
        client.println("User-Agent: arduino-ethernet");
        client.println("Connection: close");
        client.println();
    } else {
        // Connection failed
        client.stop();
    }
}

void parseHTTPResponse(){
    // Really cheap and dirty, pretty much just strcmp for burn-y or burn-n
    // in the response body. Not guaranteed to work with other people's httpds
    char msg_payload_len = 6;
    unsigned char post_nl_read = 0;
    char cmd[msg_payload_len];
    while (client.available()){
        char c = client.read();
        if (c == '\n' || c == '\r'){
space:
            post_nl_read = 0;
            while (client.available() && post_nl_read < msg_payload_len){
                cmd[post_nl_read] = client.read();
                if (cmd[post_nl_read] == '\n' || cmd[post_nl_read] == '\r'){
                    goto space;
                }
                post_nl_read++;
            }
            if (post_nl_read == 6){
                if (cmd[0] == 'b'
                        && cmd[1] == 'u'
                        && cmd[2] == 'r'
                        && cmd[3] == 'n'
                        && cmd[4] == '-'){
                    if (cmd[5] == 'y'){
                        // Turn on the heat
                        digitalWrite(RELAY_PIN, HIGH);
                        G_FURNACE_ON = 1;
                    } else if (cmd[5] == 'n'){
                        // Turn off the heat
                        digitalWrite(RELAY_PIN, LOW);
                        G_FURNACE_ON = 0;
                    }
                }
            }
        }
    }
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
void updateLocalTemps(){
    char status;
    double temp, pressure, pressure_abs;
    status = sensor.startTemperature();
    if (status != 0){
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed temperature measurement:
        // Note that the measurement is stored in the variable T.
        // Function returns 1 if successful, 0 if failure.
        status = sensor.getTemperature(temp);
        if (status != 0){
            // Start a pressure measurement:
            // The parameter is the oversampling setting, from 0 to 3
            // If request is successful, the number of ms to wait is returned.
            // If request is unsuccessful, 0 is returned.
            status = sensor.startPressure(3);
            if (status != 0){
                // Wait for the measurement to complete:
                delay(status);

                // Get the pressure (dependent on temperature)
                status = sensor.getPressure(pressure_abs, temp);
                if (status != 0){
                    // The pressure sensor returns abolute pressure, which varies with altitude.
                    // To remove the effects of altitude, use the sealevel function and your current altitude.
                    pressure = sensor.sealevel(pressure_abs, ALTITUDE);
                    postTempData(255, temp, pressure);
                }
            }
        }
    }
}
