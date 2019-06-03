// Nunchuk.h
// Created by: Floris van Rossum, 5/30/2019
// COgnitive MEdical Technology and Robotics Laboratory (COMET Robotics Lab)
// Worcester Polytechnic Institute

#include "LibraryBase.h"
#include <ArduinoNunchuk.h>

// Debug print statements for use with debugPrint
const char MSG_CREATE_NUNCHUCK[]            PROGMEM = "Arduino::nunchuk = new ArduinoNunchuk();";
const char MSG_UPDATE_NUNCHUCK[]            PROGMEM = "Arduino::nunchuk->update();";
const char MSG_INIT_NUNCHUCK[]              PROGMEM = "Arduino::nunchuk->init();";
const char MSG_DELETE_NUNCHUK[]             PROGMEM = "Arduino::nunchuk->delete();";

// MATLAB hex CMD_IDs
#define NUNCHUK_CREATE            0x00
#define NUNCHUK_UPDATE            0x01
#define NUNCHUK_INIT              0x02

// Nunchuk Wrapper class
// Extension of the MATLAB LibraryBase, MATLAB Custom Add-on
// Purpose: Allow the ArduinoNunchuk Library to interface with MATLAB
class Nunchuk: public LibraryBase {
public:
  ArduinoNunchuk *nunchuk;

  // Store nunchuk values
  uint16_t analogX;
  uint16_t analogY;
  uint16_t accelX;
  uint16_t accelY;
  uint16_t accelZ;
  char zButton;
  char cButton;
public:

  // Constructor method
  Nunchuk(MWArduinoClass& a) {
    libName = "Nunchuk/Nunchuk"; // MATLAB Custom Add-on name
    a.registerLibrary(this);
  }

public:
  // LibraryBase method to communicate between MATLAB and Arduino
  void commandHandler(byte cmdID, byte* dataIn, unsigned int payloadSize) {

    // Handle the different cmdIDs as defined in the hex CMD_IDs
    switch(cmdID) {
      case NUNCHUK_CREATE:
      {
        // Create an instantiation of ArduinoNunchuk
        createArduinoNunchuk();
        sendResponseMsg(cmdID, 0, 0); // send reply of success
        break;
      }
      case NUNCHUK_INIT:
      {
        // Initalize ArduinoNunchuk
        nunchukInit();
        sendResponseMsg(cmdID, 0, 0);
        break;
      }
      case NUNCHUK_UPDATE:
      {
        // Update the Nunchuk wrapper values
        nunchukUpdate();

        // Create the return byte array, must be bytes to allow sendResponseMsg to work
        // Convert all 16 bit ints to single bytes
        byte results[12] = {highByte(analogX), lowByte(analogX), highByte(analogY), lowByte(analogY), highByte(accelX), lowByte(accelX), highByte(accelY), lowByte(accelY), highByte(accelZ), lowByte(accelZ), zButton, cButton};
        sendResponseMsg(cmdID, results, 12); // Send results
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
    // Create an instantiation of ArduinoNunchuk
    nunchuk = new ArduinoNunchuk();
    debugPrint(MSG_CREATE_NUNCHUCK); // Send message of success to MATLAB
  }

  void nunchukUpdate() {
    // Call ArduinoNunchuk.h update() method
    nunchuk -> update();

    // Update local class variables
    analogX = nunchuk->analogX;
    analogY = nunchuk->analogY;
    accelX = nunchuk->accelX;
    accelY = nunchuk->accelY;
    accelZ = nunchuk->accelZ;
    zButton = nunchuk->zButton;
    cButton = nunchuk->cButton;
    // debugPrint(MSG_UPDATE_NUNCHUCK); // Send update message
  }

  void nunchukInit() {
    // Initialize the nunchuk object
    nunchuk -> init();
    // debugPrint(MSG_INIT_NUNCHUCK);
  }
};
