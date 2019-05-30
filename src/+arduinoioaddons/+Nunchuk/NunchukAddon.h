#include "ArduinoNunchuk.h";
#include "LibraryBase.h"

// Nunchuk Methods
const char MSG_CREATE_NUNCHUCK[]            PROGMEM = "Arduino::nunchuk = ArduinoNunchuk();";
const char MSG_UPDATE_NUNCHUCK[]            PROGMEM = "Arduino::nunchuk->update();";
const char MSG_SENDBYTE_NUNCHUCK[]          PROGMEM = "Arduino::nunchuk->_sendByte(%hhx, %hhx);"

// Errors
const char MSG_CREATE_ERROR_NUNCHUCK[]            PROGMEM = "Could not create Nunchuk \n";


#define NUNCHUK_CREATE            0x00;
#define NUNCHUK_UPDATE            0x01;
#define NUNCHUK_SENDBYTE          0x02;

class ArduinoNunchuk: public LibraryBase {
public:
  ArduinoNunchuk *nunchuk;

public:
  ArduinoNunchuk(MWArduinoClass& a) {
    libName = "NunchukAddon";
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
      case NUNCHUK_UPDATE:
      {
        nunchukUpdate();
        sendResponseMsg(cmdID, 0, 0);
        break;
      }
      case NUNCHUK_SENDBYTE:
      {
        nunchukSendByte(dataIn[0], dataIn[1]);
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

  void nunchukSendByte(byte data, byte location) {
    nunchuk -> _sendByte(data, location);
    debugPrint(MSG_SENDBYTE_NUNCHUCK);
  }
}
