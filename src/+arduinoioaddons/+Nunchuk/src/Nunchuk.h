#include "LibraryBase.h"
#include <ArduinoNunchuk.h>

// Nunchuk Methods
const char MSG_CREATE_NUNCHUCK[]            PROGMEM = "Arduino::nunchuk = new ArduinoNunchuk();";
const char MSG_UPDATE_NUNCHUCK[]            PROGMEM = "Arduino::nunchuk->update();";
const char MSG_INIT_NUNCHUCK[]            PROGMEM = "Arduino::nunchuk->init();";

// TODO delete object

// Errors
const char MSG_CREATE_ERROR_NUNCHUCK[]            PROGMEM = "Could not create Nunchuk \n";

#define NUNCHUK_CREATE            0x00
#define NUNCHUK_UPDATE            0x01
#define NUNCHUK_INIT              0x02

union {
    float fval;
    byte bval[4];
} floatAsBytes;

class Nunchuk: public LibraryBase {
public:
  ArduinoNunchuk *nunchuk;
  uint16_t analogX;
  uint16_t analogY;
  uint16_t accelX;
  uint16_t accelY;
  uint16_t accelZ;
  char zButton;
  char cButton;
public:
  Nunchuk(MWArduinoClass& a) {
    libName = "Nunchuk/Nunchuk";
    a.registerLibrary(this);
  }

public:
  void commandHandler(byte cmdID, byte* dataIn, unsigned int payloadSize) {
    switch(cmdID) {
      case NUNCHUK_CREATE:
      {
        createArduinoNunchuk();
        sendResponseMsg(cmdID, 0, 0);
        break;
      }
      case NUNCHUK_INIT:
      {
        nunchukInit();
        sendResponseMsg(cmdID, 0, 0);
        break;
      }
      case NUNCHUK_UPDATE:
      {
        nunchukUpdate();
        byte results[12] = {highByte(analogX), lowByte(analogX), highByte(analogY), lowByte(analogY), highByte(accelX), lowByte(accelX), highByte(accelY), lowByte(accelY), highByte(accelZ), lowByte(accelZ), zButton, cButton};
        sendResponseMsg(cmdID, results, 12);
        break;
      }
      default:
      {
        // do nothing?
        break;
      }
    }
  }

public:
  void createArduinoNunchuk() {
    nunchuk = new ArduinoNunchuk();
    debugPrint(MSG_CREATE_NUNCHUCK);
  }

  void nunchukUpdate() {
    nunchuk -> update();
    analogX = nunchuk->analogX;
    analogY = nunchuk->analogY;
    accelX = nunchuk->accelX;
    accelY = nunchuk->accelY;
    accelZ = nunchuk->accelZ;
    zButton = nunchuk->zButton;
    cButton = nunchuk->cButton;
    debugPrint(MSG_UPDATE_NUNCHUCK);
  }

  void nunchukInit() {
    nunchuk -> init();
    debugPrint(MSG_INIT_NUNCHUCK);
  }
};
