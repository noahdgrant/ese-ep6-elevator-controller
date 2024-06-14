/*!
 * @file CANModule.cpp
 * @brief Michael Galle's Elevator Controller API
 * @copyright Michael Galle
 *
 * @author [Michael Galle]
 * @version V1.0
 * REQUIRED: LIBRARY FROM ARDUINO LIBRARY MANAGER: mcp_can by coryjfowler (You can see examples in File > Examples > mcp_can)
 */

#include "CANModule.h"

CANModule::CANModule() : mcp2515(SPI_CS_PIN)                // Constructor  IMPORTANT: The 'new' keyword does not exist in Arduino so to instantiate an object within an class you must use an 'initializer list' (see: http://arduinoetcetera.blogspot.com/2011/01/classes-within-classes-initialiser.html)
{}

CANModule::~CANModule() 									                 // Destructor
{}

void CANModule::setup()                                    // Set up CAN communications
{
    initializeCAN();                                       
}

void CANModule::loop() {}                                 // Not used but kept for future


uint16_t CANModule::getSetpoint()                         // Return setpoint private variable
{
    return setpoint;
}

void CANModule::setSetpoint(uint16_t sp) {
  setpoint = sp;
}

byte CANModule::getTxdata()                               // Get the first byte of data from txdata array (per our protocol) - modify if you want to use more than first byte
{
  return txdata[0];
}  
                       
void CANModule::setTxdata(byte data)                                      // Set the first byte of data from txdata array (per our protocol) - modify if you want to use more than first byte  
{
  txdata[0] = data;
}                     

// Transmit CAN message
void CANModule::transmitCAN() {
    byte sndStat = mcp2515.sendMsgBuf(TxID, 0, DLC, txdata);
    if (sndStat == CAN_OK) {
        sprintf(msgString, "[CAN] TX: ID: 0x%X Data: 0x%X", TxID, txdata[0]);
    }
    else {
        sprintf(msgString, "[CAN] TX: Error Sending Message...");
    }
    Serial.println(msgString);
}

// Receive CAN message (based on sample code in library)
void CANModule::receiveCAN(LCD lcd) {
    mcp2515.readMsgBuf(&RxID, &len, rxdata);                    // Read data: len = data length, rxdata = data byte(s)

    if ((RxID & 0x80000000) == 0x80000000)                      // Determine if ID is standard (11 bits) or extended (29 bits)    - Note: This library has the IDE bit in the first nibble, this is different than the order in an extended CAN frame
        sprintf(msgString, "[CAN] RX: Extended ID: 0x%.8lX DLC: %1d Data:", (RxID & 0x1FFFFFFF), len);   // If extended ID is used then all bits are ID (uses last 29 of the 32 possible bits in the 4 byte ID) 
    else
        sprintf(msgString, "[CAN] RX: Standard ID: 0x%.3lX DLC: %1d Data:", RxID, len);

    Serial.print(msgString);

    if ((RxID & 0x40000000) == 0x40000000) {                    // Determine if message is a remote request frame.
        sprintf(msgString, " REMOTE REQUEST FRAME");
        Serial.print(msgString);
    }
    else {
        for (byte i = 0; i < len; i++) {
            sprintf(msgString, " 0x%.2X", rxdata[i]);
            Serial.print(msgString);
        }
    }
    Serial.println();                  

    // Change setpoint and output new destination floor
    lcd.lcdObj.setCursor(0, 0);                                   // Set cursor to column 0, line 0  (line 1 is second row since counting starts at 0)
    if (rxdata[0] == FLOOR1) {
        setpoint = FLOOR1_SP;
        lcd.lcdObj.print("Floor 1");
    }
    else if (rxdata[0] == FLOOR2) {
        setpoint = FLOOR2_SP;
        lcd.lcdObj.print("Floor 2");
    }
    else if (rxdata[0] == FLOOR3) {
        setpoint = FLOOR3_SP;
        lcd.lcdObj.print("Floor 3");
    }
    else {
      // Do nothing on unknown command
    }
}

// Set up CAN communications
void CANModule::initializeCAN() {
    Serial.println("Starting CAN init");
    // Initialize MCP2515 running at 8MHz with a baudrate of 125kb/s and the masks and filters enabled in standard mode. (USE MCP_ANY to disable masks and filters and MCP_STD to only check the ID bytes)
    if (mcp2515.begin(MCP_STD, CAN_125KBPS, MCP_8MHZ) == CAN_OK) {              // Change to MCP_ANY TO DISABLE MASK AND FILTERS, MCP_STD to use CAN Standard mode IDs
        Serial.println("MCP2515 Initialized Successfully!");   
    }
    else {
        Serial.println("Error Initializing MCP2515...");
    }
    Serial.println("Finished CAN init");

    #ifndef MASK
      mcp2515.init_Mask(0, 0, MASK);
      mcp2515.init_Mask(1, 0, MASK);
    #endif

    #ifndef FILTER_SC
      mcp2515.init_Filt(0, 0, FILTER_SC);                     // init_Filt(filter number, 0 for standard mode, filter used to accept a matching ID) - Filter 1
    #endif

    mcp2515.setMode(MCP_NORMAL);                              // Change to normal mode to allow messages to be transmitted
    pinMode(INT_PIN, INPUT);                                  // Interrupt pin triggered by SLAVE (CAN Adapter) to ask MASTER to initiate SPI communication
    pinMode(SPI_CS_PIN, OUTPUT);                              // Chip select pin for CAN module
}
