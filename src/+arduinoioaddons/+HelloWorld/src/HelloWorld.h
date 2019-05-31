#include "LibraryBase.h"

class HelloWorld : public LibraryBase {
public:
  HelloWorld(MWArduinoClass& a) {
    libName = "HelloWorld/HelloWorld";
    a.registerLibrary(this);
  }

public:
  void commandHandler(byte cmdID, byte* inputs, unsigned int payload_size) {
    switch (cmdID) {
      case 0x01: {
        byte val [13] = "Hello, World!";
        sendResponseMsg(cmdID, val, 13);
        break;
      }
      default: {
        // Do nothing
      }
    }
  }
};
