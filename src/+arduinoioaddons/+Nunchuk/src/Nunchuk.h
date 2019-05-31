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
class Nunchuk: public LibraryBase {
public:
  ArduinoNunchuk *nunchuk;

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
        sendResponseMsg(cmdID, 0, 0);
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
    debugPrint(MSG_UPDATE_NUNCHUCK);
  }

  void nunchukInit() {
    nunchuk -> init();
    debugPrint(MSG_INIT_NUNCHUCK);
  }
};
