#include <SPI.h>
#include <Wire.h>
#include <SFE_BMP180.h>
#include <LiquidCrystal_I2C.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "printf.h"

// Ethernet client library
#include <Ethernet.h>

// Wifi client libs
#include <Adafruit_CC3000.h>
#include <ccspi.h>

#define MAX_CLIENTS 16

//// Hardware abstractions

// nRF24L01 radio
#define PIN_R_CE 8
#define PIN_R_CSN 7
RF24 radio(PIN_R_CE, PIN_R_CSN);
uint64_t rf_pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

struct datagram {
    double temp;
    double pressure;
    double humidity;
    uint8_t node_id;
    uint64_t parity;
};

// State
static uint32_t readings_handled = 0;
double node_temps[MAX_CLIENTS];
double node_pressures[MAX_CLIENTS];
short node_updated[MAX_CLIENTS];
uint8_t G_LAST_UPDATED_NODE;
double G_LAST_UPDATED_TEMP, G_LAST_UPDATED_PRESSURE;

// Relay
#define RELAY_PIN 6
bool G_FURNACE_ON = false;

// LCD
LiquidCrystal_I2C lcd(0x3F,20,4); //Addr: 0x3F, 20 chars & 4 lines
unsigned long lcd_last_node_update = 0;
unsigned long lcd_last_node_reset = 0;
int lcd_current_node = 0;
short lcd_node_active[MAX_CLIENTS];

// IP General
char server[] = "nest.rhye.org";
#define PIN_IP_SELECTOR 0
enum IPDEV { IPDEV_WIFI, IPDEV_ETHERNET };
int IP_DEVICE = IPDEV_ETHERNET;

// Ethernet
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
EthernetClient client_eth;
unsigned long lastConnectionTime = 0;           // last time we connected to the server, in millis
boolean lastConnected = false;


// Wifi
#define PIN_WIFI_IRQ 2
#define PIN_WIFI_VBAT 5
#define PIN_WIFI_CS 10

#define WIFI_SSID "BarackaFlockaFlames"
#define WIFI_KEY "my_wifi_password_here"
#define WIFI_CIPHER WLAN_SEC_WPA2

Adafruit_CC3000 cc3000 = Adafruit_CC3000(PIN_WIFI_CS, PIN_WIFI_IRQ, PIN_WIFI_VBAT, SPI_CLOCK_DIVIDER);

Adafruit_CC3000_Client client_wifi;

const unsigned long
dhcpTimeout     = 60L * 1000L, // Max time to wait for address from DHCP
                connectTimeout  = 15L * 1000L, // Max time to wait for server connection
                responseTimeout = 15L * 1000L; // Max time to wait for data from server

// Optional on-board sensor
#define ALTITUDE 4.0
SFE_BMP180 sensor;
unsigned long last_temp_conn = 0;
unsigned temp_poll_time = 15 * 1000;
bool G_HAS_OWN_SENSOR = true;
double G_LOCAL_TEMP, G_LOCAL_PRESSURE;

// Function prototypes
void handlePendingData();
void postTempData(short node_id, float temp, float pressure);
void makeHTTPRequest(short node_id, float temp, float pressure);
void parseHTTPResponse();
void updateLocalTemps();
void updateLCD();
void setLCDDebugLine(char* msg);

uint16_t checkFirmwareVersion(void) {
    uint8_t  major, minor;
    uint16_t version = 0;

#ifndef CC3000_TINY_DRIVER
    if(!cc3000.getFirmwareVersion(&major, &minor)) {
        Serial.println(F("Unable to retrieve the firmware version!\r\n"));
    } else {
        Serial.print(F("Firmware V. : "));
        Serial.print(major); Serial.print(F(".")); Serial.println(minor);
        version = ((uint16_t)major << 8) | minor;
    }
#endif
    return version;
}

bool displayConnectionDetails(void) {
    uint32_t addr, netmask, gateway, dhcpserv, dnsserv;

    if(!cc3000.getIPAddress(&addr, &netmask, &gateway, &dhcpserv, &dnsserv))
        return false;

    Serial.print(F("IP Addr: ")); cc3000.printIPdotsRev(addr);
    Serial.print(F("\r\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\r\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\r\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\r\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
}


void setup() {
    // Setup serial
    Serial.begin(9600);
    printf_begin();

    // Wait for debugger, pins to stabilize
    delay(3000);

    // LCD display
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Booting...");

    // Read the jumper to determine if we should use wifi or ethernet
    pinMode(PIN_IP_SELECTOR, INPUT);
    delay(1000);
    if (digitalRead(PIN_IP_SELECTOR)){
        IP_DEVICE = IPDEV_WIFI;
    } else {
        IP_DEVICE = IPDEV_ETHERNET;
    }

    if (IP_DEVICE == IPDEV_ETHERNET){
        // Ethernet
        Ethernet.begin(mac);
    } else {
        if (!cc3000.begin()){
            setLCDDebugLine((char *)"Failed to init cc3000");
            while (1);
        }

        uint16_t firmware = checkFirmwareVersion();
        if ((firmware != 0x113) && (firmware != 0x118)) {
            setLCDDebugLine((char *)"Bad CC3000 firmware");
            while (1);
        }

        if (!cc3000.deleteProfiles()){
            setLCDDebugLine((char *)"CC3000 profiles exn");
            while (1);
        }

        if (!cc3000.connectToAP(WIFI_SSID, WIFI_KEY, WIFI_CIPHER)){
            setLCDDebugLine((char *)"Wifi connect failed");
            while (1);
        }

        uint32_t time = millis();
        for (; !cc3000.checkDHCP() && ((millis() - time) < dhcpTimeout); delay(100));

        if (!cc3000.checkDHCP()){
            setLCDDebugLine((char *)"Failed to DHCP");
            while (1);
        }

        while(!displayConnectionDetails());
    }

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
    radio.setPayloadSize(22);
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
    Serial.print(".");
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
            last_temp_conn = millis();
        }
    }

    setLCDDebugLine((char*)"");
    updateLCD();

    delay(1000);
}

void setLCDDebugLine(char* msg){
    lcd.setCursor(0, 3);
    lcd.print(msg);

    // Whitespace the rest of the line
    int i = 20;
    for (; i > 0 && msg[20-i] != '\0'; i--){
    }
    for (;i > 0; i--){
        lcd.print(" ");
    }
}

void setLCDNodeTemp(int row){
    int i;
    // If it's been > 5 seconds, move to the next active node
    if (millis() - lcd_last_node_update > 5000){
        // Clear the row
        lcd.setCursor(0, row);
        for (i=0; i < 20; i++){
            lcd.print(" ");
        }

        lcd_last_node_update = millis();
        for (i=0; i < 16; i++){
            lcd_current_node++;
            if (lcd_current_node > 15) {
                lcd_current_node = 0;
            }
            if (lcd_node_active[lcd_current_node]){
                break;
            }
        }
        lcd.setCursor(0, row);
        lcd.print("N");
        lcd.print(lcd_current_node);
        lcd.print(" ");
        lcd.print(node_temps[lcd_current_node]);
        lcd.print("C ");
        lcd.print((node_temps[lcd_current_node] * 9.0 / 5.0) + 32);
        lcd.print("F");
    }

    // If it's been > 60 seconds, reset the 'active' bit on all nodes
    if (millis() - lcd_last_node_reset > 60000){
        lcd_last_node_reset = millis();
        for (i=0; i < MAX_CLIENTS; i++){
            lcd_node_active[i] = 0;
        }
    }
}

void updateLCD(){
    lcd.setCursor(0, 0);
    if (G_HAS_OWN_SENSOR){
        lcd.print("Temp ");
        lcd.print(G_LOCAL_TEMP);
        lcd.print("C ");
        lcd.print((G_LOCAL_TEMP * 9.0 / 5.0) + 32);
        lcd.print("F");
    } else {
        setLCDNodeTemp(0);
    }

    lcd.setCursor(0, 1);
    lcd.print("Furnace ");
    if (G_FURNACE_ON){
        lcd.print("on ");
    } else {
        lcd.print("off");
    }

    if (G_HAS_OWN_SENSOR){
        setLCDNodeTemp(2);
    }
}

void postTempData(short node_id, float temp, float pressure){
    setLCDDebugLine((char*)"Posting reading...");
    Serial.println("");
    printf("Node: %d, temp: ", node_id);
    Serial.print(temp);
    Serial.print(", pressure ");
    Serial.print(pressure);
    Serial.println(".");

    // Post the data to the webserver
    //Serial.println("Making HTTP request...");
    makeHTTPRequest(node_id, temp, pressure);


    if (IP_DEVICE == IPDEV_WIFI){
        client_wifi.close();
    } else {
        client_eth.stop();
    }
}

void writeHTTPRequest(short node_id, float temp, float pressure){
    if (IP_DEVICE == IPDEV_WIFI){
        client_wifi.print("GET /control?");
        client_wifi.print("node_id=");
        client_wifi.print(node_id);
        client_wifi.print("&temp=");
        client_wifi.print(temp, 2);
        client_wifi.print("&pressure=");
        client_wifi.print(pressure, 2);
        client_wifi.println(" HTTP/1.1");
        client_wifi.println("Host: nest.rhye.org");
        client_wifi.println("User-Agent: arduino-ethernet");
        client_wifi.println("Connection: close");
        client_wifi.println();
    } else {
        client_eth.print("GET /control?");
        client_eth.print("node_id=");
        client_eth.print(node_id);
        client_eth.print("&temp=");
        client_eth.print(temp, 2);
        client_eth.print("&pressure=");
        client_eth.print(pressure, 2);
        client_eth.println(" HTTP/1.1");
        client_eth.println("Host: nest.rhye.org");
        client_eth.println("User-Agent: arduino-ethernet");
        client_eth.println("Connection: close");
        client_eth.println();
    }
}

void makeHTTPRequest(short node_id, float temp, float pressure){
    if (IP_DEVICE == IPDEV_WIFI){
        uint32_t ip;
        cc3000.getHostByName((char *)server, &ip);
        uint32_t time = millis();
        do {
            client_wifi = cc3000.connectTCP(ip, 80);
        } while(!client_wifi.connected() && ((millis() - time) < connectTimeout));

        if(!client_wifi.connected()) {
            setLCDDebugLine((char *)"Connection failed");
            return;
        }

        writeHTTPRequest(node_id, temp, pressure);

        parseHTTPResponse();
    } else {
        if (client_eth.connect(server, 80)){
            writeHTTPRequest(node_id, temp, pressure);
        } else {
            // Connection failed
            //Serial.println("Failed to make an HTTP connection!");
            client_eth.stop();
        }

        // Wait for the response to come back
        //Serial.println("Waiting for client to be available...");
        int retries;
        for (retries = 0; retries < 10; retries++){
            if (client_eth.available()){
                // Parse & potentially to update the relay
                //Serial.println("Parsing resp...");
                parseHTTPResponse();
                client_eth.stop();
                return;
            }
            delay(100);
        }
    }
}

void parseHTTPResponse(){
    // Really cheap and dirty, pretty much just strcmp for burn-y or burn-n
    // in the response body. Not guaranteed to work with other people's httpds
    char msg_payload_len = 6;
    unsigned char post_nl_read = 0;
    char cmd[msg_payload_len];
    char c;
    while ((IP_DEVICE == IPDEV_WIFI) ? client_wifi.available() : client_eth.available()){
        c = (IP_DEVICE == IPDEV_WIFI) ? client_wifi.read() : client_eth.read();
        if (c == '\n' || c == '\r'){
space:
            post_nl_read = 0;
            while (
                ((IP_DEVICE == IPDEV_WIFI) ? client_wifi.available() : client_eth.available())
             && (post_nl_read < msg_payload_len)){
                if (IP_DEVICE == IPDEV_WIFI){
                    cmd[post_nl_read] = client_wifi.read();
                } else {
                    cmd[post_nl_read] = client_eth.read();
                }
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

void print_uint64_bin(uint64_t u){
    uint64_t b = 1;
    for (uint64_t i = 0; i < 64; i++){
        if (u & b){
            Serial.print("1");
        } else {
            Serial.print("0");
        }
        b = b << 1;
    }
}

uint64_t dec_of_float(double d){
    uint64_t ret = 0;
    unsigned char *ds = (unsigned char *) &d;
    for (unsigned i = 0; i < sizeof (double) && i < 8; i++){
        ret = ret | (ds[i] << (i * 8));
    }
    return ret;
}

uint64_t parity(double t, double p, double h, uint64_t node_id){
    uint64_t ret = node_id;
    ret = ret ^ dec_of_float(t);
    ret = ret ^ dec_of_float(p);
    ret = ret ^ dec_of_float(h);
    return ret;
}

// Check for pending data, and mark readings as to-be-updated.
void handlePendingData(){
    setLCDDebugLine((char*)"Listen to radio...");

    // Don't get caught in an infinite radio loop - break out after handling
    // more than a reasonable number of packets.
    int packets_read = 0;
    if (radio.available() && packets_read < 25){
        packets_read++;

        // Dump the payloads until we've gotten everything
        static struct datagram node_data;
        bool done = false;
        while (!done){
            done = radio.read(&node_data, sizeof(struct datagram));
            // Ack with the number of node broadcasts we have handled
            radio.writeAckPayload( 1, &readings_handled, sizeof(readings_handled) );

            // Check the parity
            uint64_t local_parity = parity(
                    node_data.temp,
                    node_data.pressure,
                    node_data.humidity,
                    node_data.node_id
                    );

            // If the parity is wrong, don't use the reading
            if (node_data.parity == 0 || local_parity != node_data.parity){
                Serial.println("Parity mismatch for node ");
                Serial.println(node_data.node_id);
                print_uint64_bin(local_parity);
                Serial.println("");
                print_uint64_bin(node_data.parity);
                Serial.println("");
                continue;
            }

            // Update the node data array and flag the data as changed
            readings_handled++;
            node_temps[node_data.node_id] = node_data.temp;
            node_pressures[node_data.node_id] = node_data.pressure;
            node_updated[node_data.node_id] = 1;
            lcd_node_active[node_data.node_id] = 1;
        }

    }
}
void updateLocalTemps(){
    setLCDDebugLine((char*)"Taking a reading...");
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
                    G_LOCAL_TEMP = temp;
                    G_LOCAL_PRESSURE = pressure;
                }
            }
        }
    }
}
