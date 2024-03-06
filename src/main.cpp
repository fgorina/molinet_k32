/*
  Controlador per molinet amb sensor tipus "interruptor".

  La entrada es per D5. En principimels pulsos son negatius (es pot canviar es clar). Es un interruptor que creua a terra!!!
  Cada puls ha de tenir una mida mínima de debouncing en milisegons. Això s'ha de medir.

  Les entrades D8 i D7 son entrades de 12v per connectar al mando del relé de mando e indiquen Up and Down respectivament.

  Un botó de reset a D2 serveix per indicar la cadena està recollida. 
  01/12/23. Ara el botó està anul·lat doncs es pot fer millor per Bluetooth
    El LED ara es el LED intern amb lo que queda lliure el pin per futures extensions

  Un valor mm_pulse serveix per convertir les mesures a m. S'ha de calibrar i potser podriem fer-ho 
    gràcies al botó.

  Un LED ens indica coses. Si es intermitent vol dir que no hem aconseguit conexió al servidor
  Encés vol dir que tenim conexió

Per tant tenim les següents conexions:

    - Alimentació (12v i gnd)
    - Sensor
    - Mando Up indica si el relè del molinet està en up
    - Mando Down indica si el relè del molinet està en down
    - ReléUp serveix per activar el relè del molimet en up
    - ReléDown serveix per activar el relè del molinet en down

  Es conecta al servidor signalk i actualitza amb

    windlass.chain : Metres de cadena deixats anar
    windlass.state : Estat del relé delmonitor (down = 1, off = 0, up = -1, error = 2)

    Ens connectem des de l'aplicació amb BLE per lo que si mno disposem de servidor també funciona. Les comandes son

    D - Fer baixar la cadena
    U - Fer pujar la cadena
    S - Parar el molinet
    G<l> - Ajustar la cadena a l metres (pujant o baixant segons sigui convenient)
    L<l> - Deixar anar (+) o recollir (-) l metres de cadena

    Hi han una sèrie de comandes de configuració

    C<n> - Ajustr el calibrat. Som els mm per puls del sensor (en el nostre cas 48mm * 7)
    M<l> - Màxima longitut de la cadena
    R - Posa el comptador a 0 (marca cadena recollida)

 
    Quan deixem anar o recollim el molinet sempre es para al valor màxim de la cadena i a 0.

    Quan fem un G o L es para al valor desitjat o a 0/màxim cadena

    Passar de U < D o de D > U rpara el molinet per si de cas el moviment es causat per un dispositiu extern.

    G i L necessiten que estigui parat.

    01/12/23. S'afegeixen instruccions per configurar el dispositiu :

    N<String> defineix la SSID de la xarxa. Si es null (buit) aleshores no s'intenta connectar al servidor de SignalK (modo standalone)

    P<String> Es el Password.

    J<String> Defineix el nom Bluetooth del dispositiu. Potser també podriem
      fr que fos el del signalk

    KI<IPAddress> defineix la IP Address del servidor SignalK
    KP<port> defineix el port del servidor SignalK
    KU<String> Es el path de signalk per web sockets

    B rebota el dispositiu
    I Envia un missatge amb l'estat que comença per I

  El BLE defineix 2 caracteristiques:

    command
    windlass

  Command es la que es fa servir per enviar les comandes
  windlass es per on es reb l'estat com [U,D,S]<chain length>
  o es reb el estat com a I<un valor per linia>

  20/12/23 Reestructuració de la part de connexió a SignalK per suportar autentificació per Token i mDNS
    
*/

#include <Arduino.h>
#include <EEPROM.h>

#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <HTTPClient.h>
#include <ESPmDNS.h>

#define TEST 0
#define DEBUG true


#define TOKEN "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJkZXZpY2UiOiJ3aW5kbGFzcy4xIiwiaWF0IjoxNzAyNDgzMTA1LCJleHAiOjE3MzQwNDA3MDV9.INsE_XEL92hc-gzQQGs-T4a2rpNcbomCCgpqDVai1hc"

#define login "{\"requestId\": \"f0312f2b-baa2-482f-833a-64c0dd1435d4\", \"login\": { \"username\": \"pi\", \"password\": \"um23zap\" }}"
//#define pwd = "axz93klm"


#define update1 "{ \"context\": \""
#define update2 "\", \"updates\": [ {  \"source\": {\"label\": \"windlass\" }, \"values\": [ { \"path\": \"navigation.anchor.rodeLength\",\"value\":"
#define update3 "}, { \"path\": \"windlass.state\", \"value\":"
#define update4 " } ] } ]}"

#define subscribe "\", \"subscribe\": [{\"path\": \"windlass.command\", \"policy\": \"instant\"},{\"path\": \"navigation.anchor.rodeLength\", \"policy\": \"instant\"}]}"

#define metaUpdate "{\"updates\": [{\"meta\":[{\"path\":\"navigation.anchor.rodeLength\", \"value\": {\"units\": \"m\"}}]}]}"

#define windlassStat "\", \"updates\": [ {  \"values\": [ { \"path\": \"windlass.state\", \"value\":"

#define resetWindlassCommand "\", \"updates\": [ {  \"values\": [ { \"path\": \"navigation.anchor.rodeLength\", \"value\": \"S\"} ] } ]}"

// BLE

#define SERVICE_UUID "c7622328-de83-4c4d-957e-b8d51309194b"
#define WINDLASS_CHARACTERISTIC_UUID "f5c63a4c-553a-4b9f-8ece-3ebcd4da2f07"
#define COMMAND_CHARACTERISTIC_UUID "7aa080fe-0ef8-4f0c-987f-74a636dd3a77"
#define ONBOARD_LED 2  //2 in heltec is 25

#define EEPROM_SIZE 512  // 256 bytes for EEPROM

// Configuration saved in EEPROM

char ssid[20] = "Yamato";
char password[20] = "ailataN1991";
char device_name[20] = "windlass";
char skserver[20] = "";
int skport = 0;  // It is 4 bytes
char skpath[100] = "/signalk/v1/stream?subscribe=none";

float mm_pulse = 48.0 * 7;
float max_len = 60.0;


// GPIOs
const int led = 26;       // was 16 // D0, abans era 12 D6
const int sensor = 18;    // 14 //D5
const int up = 5;         // 15 //D8
const int down = 23;      // 13 //D7
const int button = 21;    // 4 // D2 ok
const int upRele = 22;    // 5 // D1
const int downRele = 19;  // 12 // D6, abans era 16  D0


//using namespace websockets;

using namespace websockets;

int state = 0;        // It is for debouncing. State of sensor
int debouncing = 20;  // Speed = 40m/min =
int ledState = 0;
int sensorState = 0;
int timer1;               // For debouncing sensor
int timer2;               // For button. Must be pressed some time to activate the reset
int counter = 0;          // Comptador de pulsos del molinet
float target_len = -1.0;  // Target len of chain to achieve

// WiFi & WebSockets

bool mdnsDone = false;  // Will be true when we have a server

bool wifi_connect = false;
int enabled = 0;  // 0 Deshabilita les accions fins que s'ha rebut un command
WebsocketsClient client;
int socketState = -4;  // -5 does not use WiFi, -4 -> Before connecting to WiFi, -3, -2.Connectingauthorized, 2-> Connected and authorized

String me = "vessels.self";
char token[256] = "";
char bigBuffer[1024] = "";

// BLE
bool deviceConnected = false;
BLECharacteristic *commandCharacteristic;
BLECharacteristic *windlassCharacteristic;

// LED

int ledOn = 0;
int ledOff = 100;

// Tasks

TaskHandle_t taskGpio;
TaskHandle_t taskNetwork;
TaskHandle_t taskLed;

void clearLed() {
  ledState = 0;
  digitalWrite(led, ledState);
  digitalWrite(ONBOARD_LED, ledState);
}

void setLed() {
  ledState = 1;
  digitalWrite(led, ledState);
  digitalWrite(ONBOARD_LED, ledState);
}

void toggleLed() {
  if (ledState == 0) {
    ledState = 1;
  } else {
    ledState = 0;
  }
  digitalWrite(led, ledState);
  digitalWrite(ONBOARD_LED, ledState);
}

void print_info() {
  Serial.println("===================== Print Info =======================");
  Serial.print("mm_pulse: ");
  Serial.println(mm_pulse);
  Serial.print("max_len: ");
  Serial.println(max_len);
  Serial.print("ssid: ");
  Serial.println(ssid);
  Serial.print("password: ");
  Serial.println(password);
  Serial.print("dev_name: ");
  Serial.println(device_name);

  Serial.print("server: ws://");
  Serial.print(skserver);
  Serial.print(":");
  Serial.print(skport);
  Serial.print("/");
  Serial.println(skpath);
  Serial.print("token: ");
  Serial.println(token);
  //
  int isUp = digitalRead(up);
  int isDown = digitalRead(down);
  Serial.print("Up ");
  Serial.print(isUp);
  Serial.print(" Down ");
  Serial.println(isDown);
  Serial.println("========================================================");

  char buffer[250];
  sprintf(buffer, "I%f\n%f\n%s\n%s\n%s\n%s\n%d\n%s\n%d\n%d\n%d", mm_pulse, max_len, ssid, password, device_name, skserver, skport, skpath, isUp, isDown, socketState);
  uint8_t *ptr = (uint8_t *)buffer;
  if (DEBUG) {
    Serial.println("Setting char value to ");
    Serial.println(buffer);
  }

  windlassCharacteristic->setValue(ptr, strlen(buffer));
  windlassCharacteristic->notify();
}

float meters(int pulses) {
  return pulses * mm_pulse / 1000.0;
}


// Sends login data. TODO : Change to tokens
void sendLogin() {
  client.send(login);
}

void sendMeta() {
  client.send(metaUpdate);
}
// Sends data to SignalK Server (State + chain length)

void sendData(int isUp, int isDown, int counter) {

  if (TEST == 1) {
    return;
  }

  if (socketState != 2) {
    Serial.println("sendData called when not connected to server");
    return;
  }
  float m = meters(counter);
  String st = "0";

  if (isUp == 1 && isDown == 0) {
    st = "-1";
  } else if (isUp == 0 && isDown == 1) {
    st = "1";
  } else if (isUp == 0 && isDown == 0) {
    st = "0";
  } else {
    st = "-2";
  }
  String s = update1 + me + update2 + String(m) + update3 + st + update4;
  client.send(s);
  if (DEBUG) { Serial.print("Sent: "); }
  if (DEBUG) { Serial.println(s); }
}

// Sets State (U,D,S) and chain length as value of windlassCharacteristic.

void setWindlassCharacteristic(int isUp, int isDown, int counter) {

  float m = meters(counter);
  String st = "S";
  if (isUp) {
    st = "U";
  } else if (isDown) {
    st = "D";
  }

  String msg = st + String(m);
  uint8_t *ptr = (uint8_t *)msg.c_str();
  if (DEBUG) {
    Serial.print("Setting char value to ");
    Serial.println(msg);
  }
  windlassCharacteristic->setValue(ptr, msg.length());
  windlassCharacteristic->notify();
}
// BLE callbacks and functions

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    Serial.println("Connected to BLE central");
    deviceConnected = true;
    BLEDevice::stopAdvertising();
  };

  void onDisconnect(BLEServer *pServer) {
    Serial.println("Disconnected from BLE central");
    deviceConnected = false;
    BLEDevice::startAdvertising();
  }
};

// Processes values written to commandCharacteristic.
// There are action commands and configurtation commands

class MyCallbacks : public BLECharacteristicCallbacks {

  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0) {
      char op = value[0];
      char op1;
      float param = 0.0;

      if (value.length() > 1) {
        param = strtof(value.substr(1).c_str(), 0);
        op1 = value[1];
      }

      if (DEBUG) {
        Serial.println("*********");
        Serial.print("New value: ");
        Serial.print(op);
        Serial.print(" : ");
        Serial.println(param);

        Serial.println("*********");
      }

      if (enabled == 0 && op == 'S') {
        enabled = 1;
        if (DEBUG) { Serial.println("Enabled"); }
      } else {
        switch (op) {

          case 'D':  // Go down, let chain
            //target_len = -1.0;
            if (digitalRead(up) == 0) {  // Up
              digitalWrite(upRele, 0);
              digitalWrite(downRele, 1);
              if (DEBUG) { Serial.println("Baixant cadena"); }
            } else {
              digitalWrite(upRele, 0);
              digitalWrite(downRele, 0);
            }
            break;

          case 'U':  // Go Up, recover Chain

            if (digitalRead(down) == 0) {  // Up
              digitalWrite(downRele, 0);
              digitalWrite(upRele, 1);
              if (DEBUG) { Serial.println("Pujant cadena"); }
            } else {
              digitalWrite(upRele, 0);
              digitalWrite(downRele, 0);
              if (DEBUG) { Serial.println("Stopping cadena when up"); }
            }
            break;

          case 'S':  // Stop
            digitalWrite(upRele, 0);
            digitalWrite(downRele, 0);
            if (DEBUG) { Serial.println("Stop"); }
            break;

          case 'G':  // Go to up or down to param meters

            if (digitalRead(up) == 0 && digitalRead(down) == 0) {
              if (DEBUG) { Serial.print("Go To  "); }
              if (DEBUG) { Serial.println(param); }
              target_len = param;
              if (target_len > meters(counter)) {
                digitalWrite(upRele, 0);
                digitalWrite(downRele, 1);
              } else if (target_len < meters(counter)) {
                digitalWrite(downRele, 0);
                digitalWrite(upRele, 1);
              } else {
                digitalWrite(downRele, 0);
                digitalWrite(upRele, 0);
              }
            }
            break;

          case 'L':  // Go up or down m meters
            if (digitalRead(up) == 0 && digitalRead(down) == 0) {
              if (DEBUG) { Serial.print("Give "); }
              if (DEBUG) { Serial.println("param"); }

              target_len = meters(counter) + param;
              if (target_len > meters(counter)) {
                digitalWrite(upRele, 0);
                digitalWrite(downRele, 1);
              } else if (target_len < meters(counter)) {
                digitalWrite(downRele, 0);
                digitalWrite(upRele, 1);
              } else {
                digitalWrite(downRele, 0);
                digitalWrite(upRele, 0);
              }
            }
            break;

            // Configuration

          case 'R':  // Reset position to 0
            digitalWrite(upRele, 0);
            digitalWrite(downRele, 0);
            counter = 0;
            if (DEBUG) { Serial.println("Resetting top chain position"); }
            break;

          case 'C':  // Calibrate every pulse (turn) to parm mm
            digitalWrite(upRele, 0);
            digitalWrite(downRele, 0);
            if (DEBUG) { Serial.print("Calibrating "); }
            mm_pulse = param;
            EEPROM.put(0, mm_pulse);
            EEPROM.commit();
            if (DEBUG) { Serial.println(mm_pulse); }
            break;

          case 'M':  // Set maximum length to param m

            digitalWrite(upRele, 0);
            digitalWrite(downRele, 0);
            if (DEBUG) { Serial.print("Max Chain length  "); }
            max_len = param;
            EEPROM.put(4, max_len);
            EEPROM.commit();
            if (DEBUG) { Serial.println(max_len); }
            break;

          case 'N':  // Set Signal K network ssid  to sparam

            strcpy(ssid, value.substr(1).c_str());
            if (DEBUG) {
              Serial.print("SSID  ");
              Serial.println(ssid);
            }

            EEPROM.put(8, ssid);
            token[0] = 0;
            EEPROM.put(192, token);
            EEPROM.commit();

            wifi_connect = false;
            if (WiFi.status() == WL_CONNECTED) {
              WiFi.disconnect();
            }


            break;

          case 'P':  // Set Signal K network password  to sparam

            strcpy(password, value.substr(1).c_str());
            if (DEBUG) {
              Serial.print("Password  ");
              Serial.println(password);
            }

            EEPROM.put(28, password);
            EEPROM.commit();
            wifi_connect = false;
            if (WiFi.status() == WL_CONNECTED) {

              WiFi.disconnect();
            }

            break;

          case 'J':  // Set device name (for signalK and BLE)

            strcpy(device_name, value.substr(1).c_str());
            if (DEBUG) {
              Serial.print("Device Name  ");
              Serial.println(device_name);
            }

            EEPROM.put(48, device_name);
            EEPROM.commit();
            // ESP.restart();   // Restart device so we see with the new name (ex. windlass.1)

            break;

          case 'K':  // Set SignalK server URL

            switch (op1) {

              case 'I':
                strcpy(skserver, value.substr(2).c_str());
                if (DEBUG) {
                  Serial.print("SignalK Host  ");
                  Serial.println(skserver);
                }
                EEPROM.put(68, skserver);
                token[0] = 0;
                EEPROM.put(192, token);
                EEPROM.commit();

                if (socketState >= 0) {
                  client.close();
                }
                break;

              case 'P':

                if (value.length() > 2) {
                  skport = atoi(value.substr(2).c_str());

                  if (DEBUG) {
                    Serial.print("SignalK port  ");
                    Serial.println(skport);
                  }
                  EEPROM.put(88, skport);
                  EEPROM.commit();
                  if (socketState >= 0) {
                    client.close();
                  }
                }
                break;

              case 'U':
                strcpy(skpath, value.substr(2).c_str());
                if (DEBUG) {
                  Serial.print("SignalK Path  ");
                  Serial.println(skpath);
                }
                EEPROM.put(92, skpath);
                EEPROM.commit();
                if (socketState >= 0) {
                  client.close();
                }

                break;
            }

            break;

          case 'B':
            ESP.restart();
            break;

          case 'I':  // Set SignalK server URL
            print_info();
            break;

          default:
            digitalWrite(upRele, 0);
            digitalWrite(downRele, 0);
            if (DEBUG) { Serial.print("Unknown command"); }
            if (DEBUG) { Serial.println(value.c_str()); }
            break;
        }

        delay(100);
        int isUp = digitalRead(up);
        int isDown = digitalRead(down);
        setWindlassCharacteristic(isUp, isDown, counter);
        sendData(isUp, isDown, counter);
      }
    }
  }
};



// Creates de BLE peripheral with its services and characteristics

void setup_ble() {
  Serial.println("Starting BLE work!");

  BLEDevice::init(device_name);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  commandCharacteristic = pService->createCharacteristic(
    COMMAND_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE);
  windlassCharacteristic = pService->createCharacteristic(
    WINDLASS_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  commandCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE Initialized!");
}

// Signal K callbacks and functions

void onWsEventsCallback(WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionOpened) {
    socketState = 1;  // Just so it does not try to reconnect
    ledOn = 100;
    ledOff = 100;
    if (DEBUG) { Serial.println("Wss Connnection Opened"); }
  } else if (event == WebsocketsEvent::ConnectionClosed) {
    ledOn = 100;
    ledOff = 500;
    if (strlen(skserver) == 0) {
      mdnsDone = false;
    }
    socketState = -2;
    if (DEBUG) { Serial.println("Wss Connnection Closed"); }

  } else if (event == WebsocketsEvent::GotPing) {
    if (DEBUG) { Serial.println("Wss Got a Ping!"); }
  } else if (event == WebsocketsEvent::GotPong) {
    if (DEBUG) { Serial.println("Wss Got a Pong!"); }
  } else {
    if (DEBUG) { Serial.println("Received unindentified WebSockets event"); }
  }
}

void onWsMessageCallback(WebsocketsMessage message) {
  if (DEBUG) {
    Serial.print("Got Message: ");
    Serial.println(message.data());
  }


  if (socketState >= 0) {

    ledOn = 1000;
    ledOff = 0;
    /*
    if (DEBUG) { Serial.println("Sending login"); }
    En principi això ja no es necessari 
    sendLogin();
    */

    socketState = 2;

    if (DEBUG) {
      Serial.println("Sending meta :");
      Serial.println(metaUpdate);
    }
    sendMeta();

  } else if (socketState == 1) {

    // Ideally check if "statusCode":200 is in message

    socketState = 2;
    setLed();
    if (DEBUG) {
      Serial.println("Sending meta :");
      Serial.println(metaUpdate);
    }
    sendMeta();
  }
}




// Connects to WiFi and SignalK Server

bool start_wifi() {

  ledOn = 100;
  ledOff = 900;

  // We start by connecting to a WiFi network

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int retries = 0;

  while (WiFi.status() != WL_CONNECTED) {
    retries++;
    if (retries > 10) {
      return false;
    }
    vTaskDelay(200);
  }

  if (DEBUG) { Serial.println(""); }
  if (DEBUG) { Serial.print("WiFi Connected to "); }
  if (DEBUG) { Serial.println(ssid); }
  if (DEBUG) { Serial.print("IP address: "); }
  if (DEBUG) { Serial.println(WiFi.localIP()); }

  ledOn = 100;
  ledOff = 800;

  return true;
}

// Processes one round of GPIO read and updates BLE and SignalK if necessary

void processGPIO() {

  int b = digitalRead(sensor);

  int isUp = digitalRead(up);
  int isDown = digitalRead(down);
  int isReset = digitalRead(button);


  if (isReset == 1 && counter != 0 && false) {  // Long press. Change to new button.
    if (DEBUG) { Serial.println("Comptador set to 0"); }
    counter = 0;
    setWindlassCharacteristic(isUp, isDown, counter);
    sendData(isUp, isDown, counter);
  }

  if (sensorState == 0) {  // Init state, all restful
    if (b == 1) {          // Sensor down Move to state i. Begin of debouncing
      timer1 = millis();
      sensorState = 1;
    }
  } else if (sensorState == 1) {  // Sensor was down () waiting for timer)
    if (b == 0) {                 // sensor up befor time debouncing, go back to state p
      timer1 = 0;
      sensorState = 0;
    } else {
      if ((millis() - timer1) > debouncing) {  // Time enough with sensor down. Trigger action

        sensorState = 2;
        timer1 = millis();
        timer2 = 0;

        // Chcek if uo or down

        if (isUp == 1 && isDown == 0) {

          counter = counter - 1;
          // Stop everything ic counter <= 0
          if (counter <= 0 || (target_len > 0 && meters(counter) <= target_len)) {
            if (counter < 0) {
              counter = 0;
            }
            digitalWrite(upRele, 0);
            digitalWrite(downRele, 0);
            target_len = -1.0;
            delay(100);
            isUp = digitalRead(up);  // Check new states
            isDown = digitalRead(down);
          }
          if (DEBUG) { Serial.print("Comptador : "); }
          if (DEBUG) { Serial.println(counter); }
          setWindlassCharacteristic(isUp, isDown, counter);
          sendData(isUp, isDown, counter);
        } else if (isUp == 0 && isDown == 1) {
          counter = counter + 1;
          if (meters(counter) >= max_len || (target_len > 0 && meters(counter) >= target_len)) {
            digitalWrite(upRele, 0);
            digitalWrite(downRele, 0);
            target_len = -1.0;
            delay(100);
            isUp = digitalRead(up);  // Check new states
            isDown = digitalRead(down);
          }
          if (DEBUG) { Serial.print("Comptador : "); }
          if (DEBUG) { Serial.println(counter); }
          setWindlassCharacteristic(isUp, isDown, counter);
          sendData(isUp, isDown, counter);
        } else if (isUp == 1 && isDown == 1) {
          if (DEBUG) { Serial.println("Error, Up and Down activats simultàneament"); }
        } else if (isUp == 0 && isDown == 0) {
          setWindlassCharacteristic(isUp, isDown, counter);
          sendData(isUp, isDown, counter);
          if (DEBUG) { Serial.println("Pau tranquilitat i una senyal espuria"); }
        }
      }
    }
  } else if (sensorState == 2) {                              // Waiting for sensor up
    if (b == 0) {                                             // Sensor up
      if (timer2 != 0 && (timer2 - millis()) > debouncing) {  // Time enough up. Go to state 0
        sensorState = 0;
        timer2 = 0;
      } else if (timer2 == 0) {  // Waiting some tome
        timer2 = millis();
      }
    }
  };
}

// TASKS :

// Connects to WiFi if ssid > 0

String requestAuth(char *skserver, int skport, char *skpath) {

  HTTPClient http;

  sprintf(bigBuffer, "http://%s:%d%s", skserver, skport, "/signalk/v1/access/requests");

  String url(bigBuffer);
  Serial.print("URL Request ");
  Serial.println(url);
  http.begin(url);

  String body = "{ \"clientId\": \"" + String(device_name) + "\", \"description\": \"Windlass controller\",\"permissions\": \"readwrite\"}";

  http.addHeader("Content-Type", "application/json");
  int responseCode = http.POST(body);

  Serial.print("Made POST with responseCode ");
  Serial.println(responseCode);

  if (responseCode == -1) {
    return "";
  }

  String payload = http.getString();
  http.end();

  Serial.print("Answer : ");
  Serial.println(payload);

  DynamicJsonDocument doc(1024);
  deserializeJson(doc, payload.c_str());

  if (responseCode == 400) {

  } else {
  }
  //Serial.print("Object "); Serial.println(doc);
  String href = String((const char *)doc["href"]);
  Serial.print("HRef ");
  Serial.println(href);

  return href;
}

bool checkAuth(char *skserver, int skport, String path) {
  HTTPClient http;

  sprintf(bigBuffer, "http://%s:%d%s", skserver, skport, (char *)path.c_str());
  String url(bigBuffer);
  Serial.println("URL Request  for checking token: ");
  Serial.println(url);

  http.begin(url);
  int responseCode = http.GET();

  String payload = http.getString();
  http.end();
  Serial.print("Answer : ");
  Serial.println(payload);
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, payload.c_str());

  if (String((const char *)doc["accessRequest"]["permission"]) == "APPROVED") {
    sprintf(token, "Bearer %s", (const char *)doc["accessRequest"]["token"]);

    Serial.println("Token:");
    Serial.println(token);
    client.addHeader("Authorization", token);
    EEPROM.put(192, token);
    EEPROM.commit();
    return true;
  } else {
    return false;
  }
}
bool validateToken() {
  HTTPClient http;

  if (strlen(token) == 0) {
    return false;
  }
  while (true) {
    sprintf(bigBuffer, "http://%s:%d%s", skserver, skport, "/signalk/v1/stream");
    String url(bigBuffer);
    Serial.println("URL Request  for checking validity of token: ");
    Serial.println(url);

    http.begin(url);
    http.addHeader("Authorization", token);
    int responseCode = http.GET();

    if (responseCode == 426) {
      Serial.println("Token Valid");
      return true;
    } else if (responseCode == 401) {
      token[0] = 0;
      Serial.println("Token Invalid");
      return false;
    } 
    else{
      vTaskDelay(1000);   // Server not avalilable
    }
  }
}
bool connectWs(char *skserver, int skport, char *skpath) {

  if (!validateToken()) {
    if (strlen(token) == 0) {
      String href = requestAuth(skserver, skport, skpath);
      if (href == "") {
        return false;
      }
      while (!checkAuth(skserver, skport, href)) {
        ledOn = 100;
        ledOff = 400;
        vTaskDelay(5000);
      }
    }
  }
  if (strlen(token) > 0) {
    client.onMessage(onWsMessageCallback);
    client.onEvent(onWsEventsCallback);
    client.addHeader("Authorization", token);
    bool connected = client.connect(skserver, skport, skpath);
    if (connected) {
      if (DEBUG) { Serial.println("Connected to ws"); }
      ledOn = 100;
      ledOff = 200;
      return true;
    } else {
      if (DEBUG) { Serial.println("Unable to connect to ws"); }
      token[0] = 0;
      return false;
    }
  } else {
    socketState = -5;  // No way!!!
    Serial.println("********** NO WAY TO GET IDENTIFIED ***********");
    return false;
  }
}

void browseService(const char *service, const char *proto) {
  Serial.printf("Browsing for service _%s._%s.local. ... ", service, proto);
  int n = MDNS.queryService(service, proto);
  if (n == 0) {
    Serial.println("no services found");
  } else {
    Serial.print(n);
    Serial.println(" service(s) found");
    for (int i = 0; i < n; ++i) {
      if (MDNS.hostname(i) == "signalk") {
        sprintf(bigBuffer, "%d.%d.%d.%d", MDNS.IP(i)[0], MDNS.IP(i)[01], MDNS.IP(i)[02], MDNS.IP(i)[3]);

        if (strcmp(skserver, bigBuffer) != 0) {

          // Reset the connection. Mujst gtet a new Token

          strcpy(skserver, bigBuffer);
          skport = MDNS.port(i);

          token[0] = 0;
          EEPROM.put(192, token);
          EEPROM.put(68, skserver);
          EEPROM.put(88, skport);
          EEPROM.commit();
        }
      }
      // Print details for each service found
      Serial.print("  ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(MDNS.hostname(i));
      Serial.print(" (");
      Serial.print(MDNS.IP(i));
      Serial.print(":");
      Serial.print(MDNS.port(i));
      Serial.println(")");
    }
  }
  Serial.println();
}

void startMdns() {
  Serial.println("Starting MDNS");
  vTaskDelay(10);
  if (!MDNS.begin(device_name)) {  // Set the hostname to "esp32.local"
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      vTaskDelay(1000);
    }
  }
  Serial.println("MDNS Started");

  browseService("_http", "_tcp");
  vTaskDelay(10000);
  mdnsDone = true;
}

void ledTask(void *parameter) {

  for (;;) {
    if (ledOn > 0) {
      setLed();
      vTaskDelay(ledOn);
    }

    if (ledOff > 0) {
      clearLed();
      vTaskDelay(ledOff);
    }
  }
}


/* socketState es l'indicador de l'estat mde la connexió de xarxa a SignalK

  Els valors poden ser :

    -5 : Quan no fem res de la xarxaperquè ha fallat la conexió, etc.
    -4 : Valor Inicial. NO estem connectats a la WiFI ni res i emprincipi podem no tenir SSID. Acció depèn de : strlen(ssid >0) -A Pasa a -3 else delay 100
    -3 : Ja tenim una SSID i per tant intentem fer una connexió. Si connected passa a -2, si no espera 1ms


    */

void networkTask(void *parameter) {
  int points = 0;
  int oldState = -10;
  for (;;) {
    if (DEBUG) {
      if (oldState != socketState) {
        Serial.print("Network Task state changed from ");
        Serial.print(oldState);
        Serial.print(" to ");
        Serial.println(socketState);
        oldState = socketState;
      }
    }

    switch (socketState) {

      case -5:  // Network Disabled

        vTaskDelay(1000);
        break;

      case -4:  // Check ssid

        if (WiFi.status() == WL_CONNECTED) {  // Should not happen!!!
          Serial.println("Perqué em vull connectar a la xarxa si ja estic connectat?????");
          socketState = -2;
        }

        if (strlen(ssid) > 0) {
          socketState = -3;
        } else {
          vTaskDelay(1000);
        }
        break;

      case -3:

        if (strlen(ssid) == 0) {
          socketState = -4;
          vTaskDelay(1000);
        }
        if (strlen(ssid) != 0) {
          if (start_wifi()) {
            if (WiFi.status() == WL_CONNECTED) {
              socketState = -2;
            } else {
            }
            vTaskDelay(100);  // Esperem a que canviin la ssid o el password
          }
        } else {
          vTaskDelay(1000);
        }

        break;

      case -2:  // Make connection to server
        if (!mdnsDone) {
          startMdns();
        }
        if (connectWs(skserver, skport, skpath)) {
          Serial.println("Connected to server????");
          print_info();
          socketState = 0;
        }
        break;
    }
    client.poll();
    vTaskDelay(1);
  }
}

void gpioTask(void *parameter) {
  while (true) {
    processGPIO();
    vTaskDelay(1);
  }
}

// General Setup


void setup() {
  // put your setup code here, to run once:

  EEPROM.begin(EEPROM_SIZE);
  pinMode(upRele, OUTPUT);
  pinMode(downRele, OUTPUT);
  digitalWrite(upRele, 0);
  digitalWrite(downRele, 0);

  digitalWrite(upRele, 0);
  Serial.begin(115200);

  pinMode(led, OUTPUT);
  pinMode(sensor, INPUT);
  pinMode(up, INPUT);
  pinMode(down, INPUT);
  pinMode(button, INPUT);


  Serial.print("setup() running on core ");
  Serial.println(xPortGetCoreID());
  pinMode(ONBOARD_LED, OUTPUT);

  // Load Data from EEPROM

  float f1;
  float f2;
  char s20[20];

  EEPROM.get(0, f1);
  EEPROM.get(4, f2);

  if (isnan(f1) || isnan(f2) || f1 == 0.0 || f2 == 0.0) {

    // Init EEPROM Area
    EEPROM.put(0, mm_pulse);
    EEPROM.put(4, max_len);
    EEPROM.put(8, ssid);
    EEPROM.put(28, password);
    EEPROM.put(48, device_name);
    EEPROM.put(68, skserver);
    EEPROM.put(88, skport);
    EEPROM.put(92, skpath);
    EEPROM.put(192, token);

    EEPROM.commit();
    if (DEBUG) { Serial.println(); }
    if (DEBUG) { Serial.println("Written default data to EEPROM"); }
  } else {
    mm_pulse = f1;
    max_len = f2;
    if (DEBUG) { Serial.println(); }
    if (DEBUG) { Serial.print("Config data mm_pulse "); }
    if (DEBUG) { Serial.print(mm_pulse); }
    if (DEBUG) { Serial.print(" max len "); }
    if (DEBUG) { Serial.println(max_len); }
  }

  EEPROM.get(8, ssid);
  EEPROM.get(28, password);
  EEPROM.get(48, s20);

  if (strlen(s20) != 0) {
    strcpy(device_name, s20);
  }

  EEPROM.get(68, skserver);
  EEPROM.get(88, skport);
  EEPROM.get(92, skpath);
  EEPROM.get(192, token);

  if (strlen(skserver) > 0) {
    Serial.println("Alreaddy have a server, no need to lookup by mDns");
    mdnsDone = true;
  }

  setup_ble();
  print_info();

  // Create WiFi and gpio tasks

  ledOn = 0;
  ledOff = 100;

  xTaskCreatePinnedToCore(gpioTask, "TaskGpio", 2000, NULL, 1, &taskGpio, 1);
  xTaskCreatePinnedToCore(networkTask, "TaskNetwork", 4000, NULL, 1, &taskNetwork, 0);
  xTaskCreatePinnedToCore(ledTask, "TaskLed", 1000, NULL, 1, &taskLed, 0);

  int isUp = digitalRead(up);
  int isDown = digitalRead(down);
  setWindlassCharacteristic(isUp, isDown, counter);
  sendData(isUp, isDown, counter);

  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {
  vTaskDelay(10);
}
