/*
 * --------------------------------------------------------------------------------------------------------------------
 * Example sketch/program showing how to read data from a PICC to serial.
 * --------------------------------------------------------------------------------------------------------------------
 * This is a MFRC522 library example; for further details and other examples see: https://github.com/miguelbalboa/rfid
 * 
 * Example sketch/program showing how to read data from a PICC (that is: a RFID Tag or Card) using a MFRC522 based RFID
 * Reader on the Arduino SPI interface.
 * 
 * When the Arduino and the MFRC522 module are connected (see the pin layout below), load this sketch into Arduino IDE
 * then verify/compile and upload it. To see the output: use Tools, Serial Monitor of the IDE (hit Ctrl+Shft+M). When
 * you present a PICC (that is: a RFID Tag or Card) at reading distance of the MFRC522 Reader/PCD, the serial output
 * will show the ID/UID, type and any data blocks it can read. Note: you may see "Timeout in communication" messages
 * when removing the PICC from reading distance too early.
 * 
 * If your reader supports it, this sketch/program will read all the PICCs presented (that is: multiple tag reading).
 * So if you stack two or more PICCs on top of each other and present them to the reader, it will first output all
 * details of the first and then the next PICC. Note that this may take some time as all data blocks are dumped, so
 * keep the PICCs at reading distance until complete.
 * 
 * @license Released into the public domain.
 * 
 * Typical pin layout used:
 * -----------------------------------------------------------------------------------------
 *             MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
 *             Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
 * Signal      Pin          Pin           Pin       Pin        Pin              Pin
 * -----------------------------------------------------------------------------------------
 * RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
 * SPI SS      SDA(SS)      10            53        D10        10               10
 * SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
 * SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
 * SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
 */

#include <SPI.h>
#include <MFRC522.h>
#include <TM1638plus.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <time.h>

#define RST_PIN         0          // Configurable, see typical pin layout above
#define SS_PIN          15         // Configurable, see typical pin layout above

// GPIO I/O pins on the Arduino connected to strobe, clock, data,
//pick on any I/O you want.
#define  STROBE_TM 16
#define  CLOCK_TM 5
#define  DIO_TM 4


// parameter addresses
# define ID_START 0
# define ID_END 3
# define ENDPOINT_START 4
# define ENDPOINT_END 67
# define ENDPOINT_PWD_START 68
# define ENDPOINT_PWD_END 99
# define WIFI_START 100
# define WIFI_END 131
# define WIFI_PWD_START 132
# define WIFI_PWD_END 163
# define AUDITORIUM_START 164
# define AUDITORIUM_END 165
# define DEBUG_START 166
# define PASSWORD_START 167
# define PASSWORD_END 198
# define CONF_START 199
# define CONF_END 202

// parameter lengths
# define ID_LEN ID_END - ID_START + 1
# define ENDPOINT_LEN ENDPOINT_END - ENDPOINT_START + 1
# define ENDPOINT_PWD_LEN ENDPOINT_PWD_END - ENDPOINT_PWD_START + 1
# define WIFI_LEN WIFI_END - WIFI_START + 1
# define WIFI_PWD_LEN WIFI_PWD_END - WIFI_PWD_START + 1
# define AUDITORIUM_LEN AUDITORIUM_END - AUDITORIUM_START + 1
# define DEBUG_LEN 1
# define PASSWORD_LEN PASSWORD_END - PASSWORD_START + 1
# define CONF_LEN CONF_END - CONF_START + 1

# define MAX_PARAMETER_NAME_LEN 64
# define MAX_PARAMETER_VALUE_LEN 64

// commands
# define ID "id"
# define ENDPOINT "endpoint_url"
# define ENDPOINT_PWD "endpoint_password"
# define WIFI "wifi"
# define WIFI_PWD "wifi_password"
# define AUDITORIUM "auditorium"
# define DEBUG "debug"
# define PASSWORD "password"

# define SHOW_ALL "show"
# define CONFIG "cfg"

/* Parameters are stored in EEPROM in the following order
 * -----------------------------------------------------------------------------------------------------------------------------------------------------
 * | Parameter                                                               | Address                                      | Type   | Length in bytes |
 * | ID: Unique(!) ID of the reader                                          | from ID_START to ID_END                      | int32  | 4               |
 * | ENDPOINT: reader will send data to this endpoint URL                    | from ENDPOINT_START to ENDPOINT_END          | str    | 64              |
 * | ENDPOINT_PWD: password for the endpoint                                 | from ENDPOINT_PWD_START to ENDPOINT_PWD_END  | str    | 32              |
 * | WIFI: reader will connect to this WiFi network                          | from WIFI_START to WIFI_END                  | str    | 32              |
 * | WIFI_PWD: with this password                                            | from WIFI_PWD_START to WIFI_PWD_END          | str    | 32              |
 * | AUDITORIUM: number of the auditorim where the reader is located         | from AUDITORIUM_START to AUDITORIUM_END      | uint16 | 2               |
 * | DEBUG: whether the scanner will send debug info to the serial           | DEBUG                                        | uint8  | 1               |
 * |                                                                         |                                              |        |                 |
 * | PASSWORD: Allows entering the configuration mode. Keep it safe          | from PASSWORD_START to PASSWORD_END          | str    | 32              |
 * | IS_FIRST_CONF: Determines whether this device was previously configured | from CONF_START to CONF_END                  | str    | 4               |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------
 */

# define EEPROM_SIZE 512

# define BUFFER_SIZE 200


 /* SETUP SEQUENCE
  * Make sure you are using serial port with 9600 baud rate
  * Connect reader to PC, install driver for CH34x chip
  * Here is the sample setup sequence:
  * -> means that you are sending the message
  * <- is the response from the reader
  * 
  * SEQUENCE
  * -> cfg                                             # Initiates a configuration session
  * <- password is required                            # Reader asks for a password
  *                                                    # On the first configuration run though the message will be different:
  *                                                    # -> your temporary password is *temp_password*, make sure you change it to something more secure
  *                                                    # -> cfg on
  *
  * -> *my_secret_password*                            # Send the password
  * <- cfg on                                          # After this message received, you are good to go and configure some parameters
  * -> PARAMETER_NAME PARAMETER_VALUE                  # Send a name and a value of the parameter, separated with the space.
  * <- Parameter set: PARAMETER_NAME PARAMETER_VALUE   # If you send correct request, reader will respond
  *                                                    # Make sure that values does not include any spaces!
  *                                   
  *                                                    # Possible parameters are :
  *                                                    # ID
  *                                                    # ENDPOINT
  *                                                    # ENDPOINT_PWD
  *                                                    # WIFI
  *                                                    # WIFI_PWD
  *                                                    # AUDITORIUM
  *                                                    # DEBUG
  *                                                    # PASSWORD
  *                                       
  * -> cfg                                             # when you have finished, send cfg once more time
  * <- cfg off                                         # reader will terminate the configuration session, save all changed parameters to EEPROM                                        
  */
// 1 is bad pwd
// 2 is bad formatting o comand
// 3 is bad param name
// 4 is bad param

// Errors

# define PWD 1
# define BAD_PARAM 2
# define BAD_FORMAT 3
# define BAD_ARG 4

# define TRY_COUNT 10

typedef struct{
    long student_id;
    long timestamp;
}log_entry;
//Constructor object
TM1638plus tm(STROBE_TM, CLOCK_TM , DIO_TM);
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance

long id;
//+1 for null-terminator
char endpoint_url[ENDPOINT_LEN + 1];
char endpoind_pwd[ENDPOINT_PWD_LEN + 1];
char wifi[WIFI_LEN + 1];
char wifi_pwd[WIFI_PWD_LEN + 1];
unsigned short auditorium;
char is_debug;
char password[PASSWORD_LEN + 1];
char is_first_conf[CONF_LEN + 1];

char error_reason = 0;

char buffer_pointer;
log_entry buffer[BUFFER_SIZE];


void send_error_msg() {
    Serial.print("Incorrect command, try again: ");
    Serial.println((int)error_reason);
    error_reason = 0;
}

char read_until_char(short term, char * buffer, short len) {
    while (true) {
        if (Serial.available()) {
            delay(100);
            char current = (char) Serial.peek();
            char counter = 0;
            char i = 0;
            while (Serial.available() && counter < len && current != '\n'
                   && (term != -1 ? current != term : true) || (i == 0)) {
                buffer[i] = (char) Serial.read();
                current = buffer[i];
                i++;
            }
            // null-terminate
            buffer[i - 1] = '\0';
            return current;
        }
    }
}

void read_parameter(char * tgt, short start, short len) {
    short counter = 0;
    for (short i = start; i < start + len; i++) {
        tgt[counter] = EEPROM.read(i);
        counter ++;
    }
}

void save_parameter(char * src, short start, short len) {
    short counter = 0;
    for (short i = start; i < start + len; i++) {
        EEPROM.write(i, src[counter]);
        counter ++;
    }
}

void read_flash_data() {
  // reads parameters stored in EEPROM
  	read_parameter((char *)&id, ID_START, ID_LEN);
  	read_parameter(endpoint_url, ENDPOINT_START, ENDPOINT_LEN);
  	read_parameter(endpoind_pwd, ENDPOINT_PWD_START, ENDPOINT_PWD_LEN);
  	read_parameter(wifi, WIFI_START, WIFI_LEN);
  	read_parameter(wifi_pwd, WIFI_PWD_START, WIFI_PWD_LEN);
  	read_parameter((char *)&auditorium, AUDITORIUM_START, AUDITORIUM_LEN);
  	read_parameter((char *)&is_debug, DEBUG_START, DEBUG_LEN);
  	read_parameter(password, PASSWORD_START, PASSWORD_LEN);
  	read_parameter(is_first_conf, CONF_START, CONF_LEN);
}

void save_flash_data() {
    save_parameter((char *)&id, ID_START, ID_LEN);
    save_parameter(endpoint_url, ENDPOINT_START, ENDPOINT_LEN);
    save_parameter(endpoind_pwd, ENDPOINT_PWD_START, ENDPOINT_PWD_LEN);
    save_parameter(wifi, WIFI_START, WIFI_LEN);
    save_parameter(wifi_pwd, WIFI_PWD_START, WIFI_PWD_LEN);
    save_parameter((char *)&auditorium, AUDITORIUM_START, AUDITORIUM_LEN);
    save_parameter((char *)&is_debug, DEBUG_START, DEBUG_LEN);
    save_parameter(password, PASSWORD_START, PASSWORD_LEN);
    save_parameter(is_first_conf, CONF_START, CONF_LEN);
    EEPROM.end();
    delay(100);
    EEPROM.begin(EEPROM_SIZE);
}

void print_flash_data() {
	Serial.print("id: ");
	Serial.println(id);
	Serial.print("endpoint_url: ");
	Serial.println(endpoint_url);
	Serial.print("endpoint_password: ");
	Serial.println(endpoind_pwd);
	Serial.print("wifi: ");
	Serial.println(wifi);
	Serial.print("wifi_pwd: ");
	Serial.println(wifi_pwd);
	Serial.print("auditorium: ");
	Serial.println(auditorium);
	Serial.print("is debug: ");
	Serial.println(is_debug);
	Serial.print("password: ");
	Serial.println(password);
	Serial.print("is first conf: ");
	Serial.println(is_first_conf);
}


void debug(char * str) {
  	if (is_debug) {
    	Serial.println(str);
  	}
}

void configure() {
	read_flash_data();
	print_flash_data();
  	if (strcmp(is_first_conf, "no") != 0) {
    	randomSeed(analogRead(0));
    // generate random password
    	for (short i = 0; i < 8; i++) 
      		password[i] = (char)random(32, 127);
      	password[33] = '\0';
    	Serial.println(F("cfg on"));
    	Serial.print(F("Your temporary password is "));
    	Serial.println(password);
    	Serial.println(F("You can change it if you want"));
    	strcpy(is_first_conf, "no");
  	} else {
  	    while (true) {
            Serial.println("Provide password");
            char pwd[MAX_PARAMETER_VALUE_LEN] = {0};

            read_until_char(-1, pwd, MAX_PARAMETER_VALUE_LEN);

            if (strcmp(password, pwd) == 0) {
                Serial.println(F("cfg on"));
                break;
            } else {
                error_reason = PWD;
            }
            if (error_reason)
                send_error_msg();
        }
  	}
  	while (true) {
        char parameter[MAX_PARAMETER_NAME_LEN] = {0};
        char value[MAX_PARAMETER_VALUE_LEN] = {0};

        char current = read_until_char(' ', parameter, MAX_PARAMETER_NAME_LEN);

        if (current == ' ' || current == '\n') {
            if (current == ' ') {
                read_until_char(-1, value, MAX_PARAMETER_VALUE_LEN);
            }
            char is_cmd = 0;

            if (strcmp(parameter, ID) == 0 && parameter) {
                long myid = atoi(value);
                if (myid == 0 && value[0] != '0') {
                    error_reason = BAD_ARG;
                } else {
                    id = myid;
                }
            } else if (strcmp(parameter, ENDPOINT) == 0) {
                strcpy(endpoint_url, value);
            } else if (strcmp(parameter, ENDPOINT_PWD) == 0) {
                strcpy(endpoind_pwd, value);
            } else if (strcmp(parameter, WIFI) == 0) {
                strcpy(wifi, value);
            } else if (strcmp(parameter, WIFI_PWD) == 0) {
                strcpy(wifi_pwd, value);
            } else if (strcmp(parameter, AUDITORIUM) == 0) {
                long aud = atoi(value);
                if (aud == 0 && value[0] != '0') {
                    error_reason = BAD_PARAM;
                } else {
                    auditorium = aud;
                }
            } else if (strcmp(parameter, DEBUG) == 0) {
                char db = (char) atoi(value);
                if (db == 0 && value[0] != '0') {
                    error_reason = BAD_ARG;
                } else {
                    is_debug = db;
                }
            } else if (strcmp(parameter, PASSWORD) == 0) {
                strcpy(password, value);
            } else if (strcmp(parameter, SHOW_ALL) == 0) {
                print_flash_data();
                is_cmd = 1;
            } else if (strcmp(parameter, CONFIG) == 0) {
                save_flash_data();
                Serial.println(F("cfg off"));
                break;
            } else {
                error_reason = BAD_PARAM;
            }
            if (!error_reason && !is_cmd) {
                Serial.print("Parameter set:  ");
                Serial.print(parameter);
                Serial.print(": ");
                Serial.println(value);
            }
        } else {
            error_reason = BAD_FORMAT;
        }
        if (error_reason) {
            send_error_msg();
        }
    }
    ESP.restart();
}

void try_configure() {
    if (Serial.available()) {
        char cmd[MAX_PARAMETER_VALUE_LEN] = {0};
        read_until_char(-1, cmd, MAX_PARAMETER_VALUE_LEN);
        if (strcmp(cmd, CONFIG) == 0) {
            configure();
        }
    }
}

void setup() {
  	EEPROM.begin(EEPROM_SIZE);
	Serial.begin(9600);		// Initialize serial communications with the PC
  	while (!Serial);   // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
  	Serial.println(F("Device is online, reading flash"));
  	read_flash_data();
  	Serial.print(F("FLASH read success, debug is "));
  	Serial.println(is_debug ? "1" : "0");

    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi, wifi_pwd);
    Serial.println("\nConnecting to WiFi");

    char i = 0;
    while (WiFi.status() != WL_CONNECTED && i < TRY_COUNT) {
        Serial.print(".");
        delay(1000);
        try_configure();
        i++;
    }

    if (i == TRY_COUNT) {
        error_reason = 5;
    } else {
        Serial.println("\nConnected to WiFi");
    }

    i = 0;
    configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    Serial.println("\nWaiting for time");
    while (time(nullptr) < 100000 && i < TRY_COUNT) {
        Serial.print(".");
        delay(200);
        try_configure();
        i++;
    }

    if (i == TRY_COUNT) {
        error_reason = 6;
    }

  	tm.displayBegin();
	SPI.begin();			// Init SPI bus
	mfrc522.PCD_Init();		// Init MFRC522
	delay(4);				// Optional delay. Some board do need more time after init to be ready, see Readme

  	Serial.println(F("Device is armed"));
	Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
    if (error_reason) {
        send_error_msg();
        ESP.restart();
    }
}

String printHex(byte *buffer, byte bufferSize) {
    String str = "";
    for (byte i = 0; i < bufferSize; i++) {
        str += (buffer[i] < 0x10 ? "0" : "");
        str += String(buffer[i], HEX);
    }
    str.toUpperCase();
    return str;
}

void loop() {
    tm.displayText("--------");

    if (Serial.available()) {
        char cmd[MAX_PARAMETER_VALUE_LEN] = {0};
        read_until_char(-1, cmd, MAX_PARAMETER_VALUE_LEN);
        if (strcmp(cmd, CONFIG) == 0) {
            configure();
        }
    }

    // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
    if (mfrc522.PICC_IsNewCardPresent()) {
        // Select one of the cards
        if (mfrc522.PICC_ReadCardSerial()) {
            String data = printHex(mfrc522.uid.uidByte, mfrc522.uid.size);
            time_t now = time(nullptr);
            long student_id;
            memcpy(&student_id, mfrc522.uid.uidByte, mfrc522.uid.size);

            buffer[buffer_pointer].student_id = student_id;
            buffer[buffer_pointer].timestamp = long(now);
            buffer_pointer = (char)((buffer_pointer + 1) % BUFFER_SIZE);

            Serial.println("Buffer content");
            for (int i = 0; i < buffer_pointer; i ++) {
                Serial.print("Student_id: ");
                Serial.print(buffer[i].student_id);
                Serial.print(" Timestamp: ");
                Serial.println(buffer[i].timestamp);
            }

            tm.displayText(data.c_str());
            mfrc522.PICC_HaltA();
            mfrc522.PCD_StopCrypto1();

            delay(1000);
        }
    }

}

