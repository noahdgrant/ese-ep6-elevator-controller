/*!
 * @file ElevatorController.cpp
 * @brief Michael Galle's Elevator Controller API
 * @copyright Michael Galle
 *
 * @author [Michael Galle]
 * @version V1.0
 *
 */

#include "ElevatorController.h"

ElevatorController::ElevatorController()                    // Constructor - No code
{}	

ElevatorController::~ElevatorController()                   // Destructor - No code
{}	

void ElevatorController::setup() {
    Serial.begin(115200);                                   // initialize serial communication at 115200 bits per second:
    while (!Serial);                                        // Wait until serial port connects. (Only needed for native USB port)
    Wire.begin();                                           // join i2c bus (address optional for master) - Arduino is master
    SPI.begin();                                            // initialize the SPI library and set the MOSI, and CS pin modes to output mode. Also sets MOSI and SCLK to LOW and CS to HIGH.
    
    // Setup of sub-modules of the ElevatorController
    initializeTimer();                                      // Set up interrupt timing for transmit interval (EC sends current floor every 2 seconds) 
    CM.setup();                                             // Setup CAN module object
    DSM.setup();                                            // Setup Distance Sensor module object
    DM.setup();                                             // Setup DAC module object
    LCDM.setup();                                           // Setup CAN module object

    // Initial settings (default floor)
    CM.setTxdata(FLOOR1);                                  // FLOOR1, FLOOR2, FLOOR3    
    LCDM.lcdObj.setCursor(0, 0);
    LCDM.lcdObj.print("Floor 1");
    CM.setSetpoint(FLOOR1_SP);                             // Initialize default setpoint to that of FLOOR1

    m_currentFloor = 0; // Unknown

    // Initialize flags
    flagRecv = false;
    flagTx = false;
}

void ElevatorController::loop() {
    // Receive CAN message for which floor to go to
    if (flagRecv) {                                         // Receive message (INT_PIN triggers interrupt that sets flagRecv true to indicate that a new message has been received)
        flagRecv = false;                                   // Reset the flag as we will use it again if another request is received
        CM.receiveCAN(LCDM);                                // Receive the message 
    }

    // Transmit CAN message to tell everyone the current elevator floor every few seconds
    if (flagTx) {                                           // Use a timer interrupt (that sets flagTx true) to transmit the current floor every few seconds
        flagTx = false;
        CM.setTxdata(m_currentFloor);
        CM.transmitCAN();                                   // Send the current floor via CAN 
    }

    Move(CM.getSetpoint());
    checkCurrentFloor();
}
 
	
// Set up interrupt timing for transmit interval (EC sends current floor every 2 seconds) 
void ElevatorController::initializeTimer() {                     
    cli();                                                              // stop interrupts
    // Set timer1 interrupt by setting the registers --> See register map and tutorial at:  https://www.instructables.com/Arduino-Timer-Interrupts/
    TCCR1A = 0;                                                         // Set TCCR1A register to 0 (clear existing control values so we can set functionality below)
    TCCR1B = 0;                                                         // Set TCCR1B register to 0 (clear existing control valuesso we can set functionality below)
    TCNT1 = 0;                                                          // Initialize the counter value for timer1 to 0

    // Set the compare match register (binary value) so we get our desired Hz increments (as described above with prescaler of 1024)
    // [(16*10^6) / (1/T *1024)] - 1     --> Control Register for timer1 must be below 65536 since it is a 16-bit register (you can modify the prescaler value if too large)
    // 15624 -> T=1 second
    // 32767 -> T=2 seconds
    OCR1A = 15624;

    // Turn on CTC Mode for timer1
    TCCR1B |= (1 << WGM12);                                             // WGM12 == 3 (turn on CTC Mode for timer1)
    // Set the CS10 and CS12 bits in TCCR1B to give a prescaler = 1024
    TCCR1B |= (1 << CS12) | (1 << CS10);                                // CS12 == 2, (to get value of 1024), CS10 == 0  (Use external 16MHz clock (external to microprocessor but onboard the UNO) source on T1 pin of arduino) - See register map at:  https://www.instructables.com/Arduino-Timer-Interrupts/
    // Enable the timer compare interrupt for timer1
    TIMSK1 |= (1 << OCIE1A);                                            // TIMSK1 is a register, OCIE1A = 1  (set this bit to 1) - this enables the interrupt vector (TIMER1_COMPA_vect)
    sei();                                                              // enable/allow interrupts
    // Set timer-based interrupt flag
    flagTx = false;
}

// Move to setpoint distance (floor)
void ElevatorController::Move(uint16_t setpoint) {
  	int difference = 0; // Difference in mm from setpoint (floor). A positive value is above the setpoint distance (floor) and a negative value is below.
    
    DSM.sensor.start();
    m_dist = DSM.sensor.getDistance();
    delay(100);
    DSM.sensor.stop();

    if (m_dist > MINHEIGHT && m_dist < MAXHEIGHT) {
        // Output the distance to the LCD
        LCDM.loop(m_dist);

        //Output the difference between setpoint and distance
        difference = m_dist - setpoint;        // positive value means above setpoint (later take the negative of this value to indicate direction to move - i.e. down)
        //Serial.print("Distance ");           // Testing
        //Serial.println(m_dist);              // Testing
        if (abs(difference) <= SETPOINT_TOLERANCE) {
            difference = 0;
        }

        // Set dynamic parameters to smooth out motion: difference = difference * A e^(-a * difference) - Graph this to see what the motion will look like
        a = (float)DAMPENER / (float)diffMax;

        difference = difference * A * exp((-1) * a * abs(difference));   
        
        // Need to make sure this is below 1023 since this is the max value that can be sent to the DAC
        if(difference > 1023) {
          difference = 1023; 
        } 

        //Serial.println(difference);                          // Testing
        DM.transferDAC(difference);                            // Set values on DAC to control motor speed
    }
    else {
        DM.transferDAC(0);                                     // Stop the elevator when get an out of range measurement - make sure Floor 1 is above MINHEIGHT and Floor 3 is below MAXHEIGHT
    }
}

void ElevatorController::checkCurrentFloor() {

    // Check current floor
    if ((m_dist >= FLOOR1_SP - (SETPOINT_TOLERANCE + 10)) && (m_dist <= FLOOR1_SP + (SETPOINT_TOLERANCE + 10))) {
        m_currentFloor = FLOOR1;
    } else if ((m_dist >= FLOOR2_SP - (SETPOINT_TOLERANCE + 10)) && (m_dist <= FLOOR2_SP + (SETPOINT_TOLERANCE + 10))) {
        m_currentFloor = FLOOR2;
    } else if ((m_dist >= FLOOR3_SP - (SETPOINT_TOLERANCE + 10)) && (m_dist <= FLOOR3_SP + (SETPOINT_TOLERANCE + 10))) {
        m_currentFloor = FLOOR3;
    } else {
      // If the car is between floors, keep repeating the last known floor
    }
}