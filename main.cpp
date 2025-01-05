/* TODO
 * Fix SD card issues
 * Add Settings Page with options (reset spool, );
 * Add "Finished!" on BLE control pad when done
 * Add PWD protection on BLE
 * Add pause/emergency button
 */

#include "mbed.h"
#include "rtos.h"
#include "SDFileSystem.h"
#include "uLCD_4DGL.h"
#include "params.h"
#include "Stepper.h"
#include "Servo.h"
#include "PinDetect.h"
#include "Motor.h"

/*** Devices and Pins ***/
// Debugging : LEDs, PC
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
RawSerial pc(USBTX, USBRX);

// Peripherals
SDFileSystem sd(p5, p6, p7, p8, "sd");
DigitalIn cd(p15, PullUp);

uLCD_4DGL lcd(p9, p10, p30);
RawSerial ble(p13, p14);

// Motors
Stepper wireFeeder(p16, p17, p18, p19, p20, p21);
Motor wireCutter(p23, p24, p25);
Servo wireGuide(p22);

PinDetect feederHallSensor(p11, PullUp);
PinDetect cutterUpperLimitSwitch(p27, PullUp);
PinDetect cutterLowerLimitSwitch(p28, PullUp);

// Global Variables
volatile double wireLeft = 1000.0; //ft, current wire on spool
volatile double wireLength = 0.0; // Length of Wire, in
volatile double leftIncisionDist=0.0; // Distance from left end to incision, in
volatile double rightIncisionDist=0.0; // Distance from right end to incision, in
volatile int optionSelected = 1;
volatile bool refreshScreen = true;
volatile int guideAngle = POS_CUT;

volatile int feederEncoderCount = 0;

volatile int numWires = 1;
volatile int numWiresLeft = 1;

volatile ButtonState currentButton=INVALID;
volatile State currentState = MENU;
volatile bool stateChange = 1;

volatile BleState currentBleState = IDLE;
volatile char bhit;
volatile char bnum;
volatile bool buttonReady = true;

volatile float startTime = 0.0;

Thread heartbeatThread;
Thread updateWireLeftThread;
Thread saveWireLeftThread;
Thread waitForButtonThread;
Thread updateBottomScreenThread;
Timeout bleTimeout;

Timer cutterTimer;

Mutex lcdLock;

void validateWireParams() {
    
    // If incision distance past midpoint, round down to nearest increment
    if(leftIncisionDist>= wireLength/2) {
        leftIncisionDist = WIRE_INCREMENT*((int)(wireLength/2/WIRE_INCREMENT))-MIN_DIST_FROM_MIDPOINT;
    }
    if(rightIncisionDist>= wireLength/2) {
        rightIncisionDist = WIRE_INCREMENT*((int)(wireLength/2/WIRE_INCREMENT))-MIN_DIST_FROM_MIDPOINT;
    }
    
    if(wireLength < 0) { wireLength = 0; }
    if(leftIncisionDist < 0) {leftIncisionDist = 0.0;}
    if(rightIncisionDist < 0) {rightIncisionDist = 0.0;}
    if(numWires < 1) { numWires = 1; }
    
    if (numWires*wireLength > wireLeft*12.0) {numWires=wireLeft*12.0/wireLength;}
}

void updateWireLeft() {
    int percentLeft = 100;
    while(1) {
        percentLeft = 100*wireLeft/MAX_SPOOL_LENGTH;
        lcdLock.lock();
        
        lcd.filled_rectangle(0,0,127,16, BLACK);
        
        lcd.locate(0,0);
        lcd.printf("Wire Left: \n\r");
        lcd.printf("%3.1f ft", wireLeft);
        
        lcd.rectangle(127-52,0,127,12, WHITE);
        if(percentLeft >= WIRE_LEVEL_MED) {
            lcd.filled_rectangle(127-51,1,127-51+percentLeft/2,11, GREEN);
        } else if(percentLeft >= WIRE_LEVEL_LOW) {
            lcd.filled_rectangle(127-51,1,127-51+percentLeft/2,11, RED+GREEN);
        } else {
            lcd.filled_rectangle(127-51,1,127-51+percentLeft/2,11, RED);
        }
        
        lcd.locate(14,1);
        lcd.printf("%3i%%", percentLeft);
        lcdLock.unlock();
        Thread::wait(1000);
    }

}

void saveWireLeft() {
    char * str;
    while(1) {
        FILE * fp = fopen("sd/params.txt", "W");
        if(fp == NULL) {
            error ("Unable to open file");
        }
        sprintf(str, "%f", wireLeft);
        fprintf(fp, str);
        fclose(fp);
        Thread::wait(60*1000);
    }
}

// Heartbeat Thread. Make sure RTOS still running
void heartbeat() {
    while(1){
        led4 = !led4;
        Thread::wait(1000);
    }
}

void bleIRQ(){
    switch(currentBleState) {
        case IDLE: {
            if(ble.getc()=='!') { currentBleState = GOT_EXCLAM; }
            else { currentBleState = IDLE; }
            break;
        }
        case GOT_EXCLAM: {
            if(ble.getc()=='B') { currentBleState = GOT_B; }
            else { currentBleState = IDLE; }
            break;
        }
        case GOT_B: {
            bnum = ble.getc();
            currentBleState = GOT_BTN;
            break;
        }
        case GOT_BTN: {
            bhit = ble.getc();
            currentBleState = GOT_HIT;
            break;
        }
        case GOT_HIT: {
            int btn = 0xFF;
            if (ble.getc()==char(~('!' + 'B' + bnum + bhit))) { //checksum OK?
                buttonReady = 1;
                btn=(((bnum-'0')<<4) | (bhit-'0'));
                currentButton = (ButtonState)btn;
            }
            currentBleState = IDLE;
            break;
        }
        default: {
            ble.getc();
            currentBleState=IDLE;
            break;
        }
        
    }
    
}

void updateBottomScreen(){
    while(true){
        if(currentState==MENU&&stateChange){
            lcdLock.lock();
            lcd.filled_rectangle(0,16,127,127, BLACK); // Clear screen
            lcd.locate(0,3);
            lcd.printf("Select Option\n\r");
            lcd.printf("[1]New Operation \n\r");
            lcd.printf("[2]Settings \n\r");
            lcd.printf("[3]About \n\r");
            lcdLock.unlock();
            stateChange = 0;
        }
        if(currentState==CUTTING_ONE&&stateChange){
            lcdLock.lock();
            lcd.filled_rectangle(0,16,127,127, BLACK); // Clear screen
            
            lcd.filled_rectangle(0,32,127,32+4, RED); // Wire
            lcd.filled_rectangle(16,32,18,32+4, BLACK); // Incision Left
            lcd.filled_rectangle(16,33,18,35, 0xB87333);
            
            lcd.filled_rectangle(127-16,32,127-18,32+4, BLACK); // Incision Right
            lcd.filled_rectangle(127-16,33,127-18,35, 0xB87333);
            
            lcd.filled_rectangle(0,40,1,48, RED); // Length visual
            lcd.filled_rectangle(126,40,127,48, RED);
            lcd.filled_rectangle(0,43,127,44, RED);
            lcd.locate(3,5);
            lcd.printf("Length (in.)");
            
            lcd.filled_rectangle(0,48,1,56, RED); // Left Incision Visual
            lcd.filled_rectangle(17,48,18,56, RED);
            lcd.filled_rectangle(0,52,18,53, RED);
            lcd.locate(1,6);
            lcd.printf("L");
            
            lcd.filled_rectangle(127,48,126,56, RED); // Right Incision Visual
            lcd.filled_rectangle(127-17,48,127-18,56, RED);
            lcd.filled_rectangle(127,52,127-18,53, RED);
            lcd.locate(17,6);
            lcd.printf("R");
            
            lcd.locate(0,15);
            lcd.printf("[L]Back    [R]Next");
            lcdLock.unlock();
            stateChange = 0;
        }
        if(currentState == CUTTING_TWO && stateChange) {
            lcdLock.lock();
            lcd.filled_rectangle(0,16,127,127, BLACK); // Clear screen
            lcd.locate(0,15);
            lcd.printf("[L]Abort");
            lcdLock.unlock();
            stateChange = 0;
        }
        if(currentState == CUTTING_ONE && refreshScreen) {
            lcdLock.lock();
            lcd.filled_rectangle(0,80,127,112, BLACK);
            
            lcd.locate(0,10);
            validateWireParams();
            lcd.printf("%s[1]Length:%4.1fin\n\r", (optionSelected%5==1)?">":" ",wireLength);
            lcd.printf("%s[2]L_Cut: %4.1fin\n\r",(optionSelected%5==2)?">":" ",leftIncisionDist);
            lcd.printf("%s[3]R_Cut: %4.1fin\n\r",(optionSelected%5==3)?">":" ",rightIncisionDist);
            lcd.printf("%s[4]Num Wires: %3i\n\r",(optionSelected%5==4)?">":" ",numWires);
            lcdLock.unlock();
            refreshScreen = false;
        }
        if(currentState == CUTTING_TWO) {
            lcdLock.lock();
            lcd.filled_rectangle(0,40,127,60, BLACK);
            lcd.locate(0,4);
            lcd.printf("Progress:\n\n\n");
            lcd.printf("Wires Made:%3i/%i",numWires-numWiresLeft, numWires);
            lcd.rectangle(14,40,127-14,52, WHITE);
            
            lcd.filled_rectangle(14,41,14+(int)(100*(1-(float)numWiresLeft/numWires)),51, GREEN);
            lcd.locate(12,6);
            lcd.printf("%3i%%", (int)(100*(1-(float)numWiresLeft/numWires)));
            
            if(numWiresLeft==0){
                lcd.locate(9,15);
                lcd.printf("[R]Finish");
            }
            lcdLock.unlock();
        }
        if(currentState==SETTINGS_ONE&&stateChange){
            lcdLock.lock();
            lcd.filled_rectangle(0,16,127,127, BLACK); // Clear screen
            lcd.locate(0,3);
            lcd.printf("Select Setting\n\r");
            lcd.printf("[1]Reset Spool \n\r");
            lcd.printf("[2]Feed Wire \n\r");
            lcd.printf("[3]Move Cutter \n\r");
            lcd.printf("[4]Move GuideMotor \n\r");
            
            lcd.locate(0,15);
            lcd.printf("[L]Back");
            lcdLock.unlock();
            stateChange = 0;
        }
        if(currentState==SETTINGS_FEED&&stateChange){
            lcdLock.lock();
            lcd.filled_rectangle(0,16,127,127, BLACK); // Clear screen
            lcd.locate(0,3);
            lcd.printf("[U]Feed FWD\n\r");
            lcd.printf("[D]Feed REV\n\r");
            
            lcd.locate(0,15);
            lcd.printf("[L]Back");
            lcdLock.unlock();
            stateChange = 0;
        }
        if(currentState==SETTINGS_CUTTER&&stateChange){
            lcdLock.lock();
            lcd.filled_rectangle(0,16,127,127, BLACK); // Clear screen
            lcd.locate(0,3);
            lcd.printf("[U]Cutter CW\n\r");
            lcd.printf("[D]Cutter CCW\n\r");
            
            lcd.locate(0,15);
            lcd.printf("[L]Back");
            lcdLock.unlock();
            stateChange = 0;
        }
        if(currentState==SETTINGS_GUIDE&&stateChange){
            lcdLock.lock();
            lcd.filled_rectangle(0,16,127,127, BLACK); // Clear screen
            lcd.locate(0,3);
            lcd.printf("[U]Inc. Angle\n\r");
            lcd.printf("[D]Dec. Angle\n\r");
            
            lcd.locate(0,15);
            lcd.printf("[L]Back");
            lcdLock.unlock();
            stateChange = 0;
        }
        if(currentState==SETTINGS_GUIDE&&refreshScreen){
            lcdLock.lock();
            lcd.filled_rectangle(0,48,127,56, BLACK); // Clear screen
            lcd.locate(0,6);
            lcd.printf("Angle: %3i", guideAngle);
            lcdLock.unlock();
            refreshScreen=false;
        }
        Thread::wait(100);
    }
}

void updateFeederEncoderCount() {
    Thread::yield();
    feederEncoderCount++;
}

void feedWireUntilCount(int counts) {
    feederEncoderCount = 0;
    wireFeeder.enable();
    while (feederEncoderCount < counts) {
        wireFeeder.step(FEEDER_MOTOR_STEPS,STEPPER_REV,FEEDER_MOTOR_SPEED);
        Thread::wait(5);
    }
    wireFeeder.disable();
}

/*
void feedWireUntilLength(float length) {
    int steps = (int)(length*FEEDER_STEP_PER_INCH);
    pc.printf("%f\n\r", FEEDER_STEP_PER_INCH);
    wireFeeder.enable();
    for(int step = 0; step < steps; step++){
        wireFeeder.step(FULL_STEP,STEPPER_REV,FEEDER_MOTOR_SPEED);
        Thread::wait(5);
    }
    wireFeeder.disable();
}
*/

void cut() {
    wireCutter.speed(-CUTTER_MOTOR_SPEED);    
    while(!cutterLowerLimitSwitch) { Thread::wait(10); }
    wireCutter.speed(CUTTER_MOTOR_SPEED);
    while(!cutterUpperLimitSwitch) { Thread::wait(10); }
    wireCutter.speed(0.0);
}

void cutWires() {
    wireCutter.speed(CUTTER_MOTOR_SPEED);
    while(!cutterUpperLimitSwitch) { Thread::wait(10); }
    wireCutter.speed(0.0);
    for(int i = numWiresLeft; i > 0; i--) {
        // Feed wire until left incision
        feedWireUntilLength(leftIncisionDist);
        //feedWireUntilCount(leftIncisionDist/WIRE_INCREMENT);
        // Switch servo to stripper
        wireGuide.position(POS_STRIP);
        // Make left incision & open back up
        cut();
        
        // Feed wire until right incision
        //feedWireUntilLength(wireLength-leftIncisionDist-rightIncisionDist);
        feedWireUntilCount((wireLength-leftIncisionDist-rightIncisionDist)/WIRE_INCREMENT);
        // Make right incision & open back up
        cut();
        
        // Feed wire until length
        //feedWireUntilLength(rightIncisionDist);
        feedWireUntilCount(rightIncisionDist/WIRE_INCREMENT);
        // Switch servo to cutter
        wireGuide.position(POS_CUT);
        // Make cut & open back up
        cut();
        
        wireLeft -= feederEncoderCount/8*FEEDER_WHEEL_CIRCUMFERENCE;
        //wireLeft -= wireLength;
        numWiresLeft--;
    }
    
}

int main() {
    // Initialize uLCD with Splash Screen
    lcd.baudrate(3000000);
    lcd.printf(" WireFactory v1.0\n\r");
    lcd.printf("\n\r\n\r\n\r");
    lcd.printf("  Made for HKN by\n\r");
    lcd.printf("\n\r");
    lcd.printf("   Thomas Contis\n\r");
    lcd.printf("   Daniel Jacobs\n\r");
    lcd.printf("Christopher Saetia\n\r");
    
    Thread::wait(SPLASH_SCREEN_LOAD_TIME);
    
    // Run Threads
    lcd.cls();
    
    // If card is inserted for at least 3 seconds, assume it is in
    /*
    bool cardInserted = false;
    while(!cardInserted){
        if(!cd) {
            lcdLock.lock();
            lcd.locate(0, 0);
            lcd.color(RED);
            lcd.printf("Insert uSD card to\nstart!");
            lcd.color(GREEN);
            lcdLock.unlock();
        }
        while(!cd);
        Thread::wait(3000);
        if(cd) {cardInserted = true;}
    }
    */
    
    updateWireLeftThread.start(&updateWireLeft);
    heartbeatThread.start(&heartbeat);    
    //saveWireLeftThread.start(&saveWireLeft);
    updateBottomScreenThread.start(&updateBottomScreen);
    
    ble.attach(&bleIRQ,RawSerial::RxIrq);
    
    feederHallSensor.attach_asserted(&updateFeederEncoderCount);
    feederHallSensor.attach_deasserted(&updateFeederEncoderCount);
    feederHallSensor.setSampleFrequency(5000);
    
    cutterUpperLimitSwitch.setSampleFrequency(5000);
    cutterLowerLimitSwitch.setSampleFrequency(5000);
    
    // Initialize SD Card
    /*
    FILE * fp = fopen("/sd/params.txt", "r");
    if(fp != NULL) {
        char * temp;
        fgets(temp, 100, fp);
        pc.printf("%s\n", temp);
        fclose(fp);
    } else { led3=1; }
    */
    //wireLength = strtod(temp, NULL);
    
    // Initialize Motors
    wireGuide.calibrate(0.0015,0.0009,180);
    wireGuide.position(POS_STRIP);
    
    wireCutter.speed(CUTTER_MOTOR_SPEED);
    while(!cutterUpperLimitSwitch) { Thread::wait(10); }
    wireCutter.speed(0.0);
    // Use main thread for operation
    while(1) {
        switch(currentState) {
            case MENU : {
                if(buttonReady && currentButton == ONE_RELEASED) {
                    currentState = CUTTING_ONE;
                    stateChange = 1;
                }
                if(buttonReady && currentButton == TWO_RELEASED) {
                    currentState = SETTINGS_ONE;
                    stateChange = 1;
                }
                break; 
            }
            case CUTTING_ONE : {
                if (buttonReady) {
                    switch(currentButton) {
                        case UP_PRESSED:
                            if(optionSelected==1)
                                wireLength+=WIRE_INCREMENT;
                            if(optionSelected==2)
                                leftIncisionDist+=WIRE_INCREMENT;
                            if(optionSelected==3)
                                rightIncisionDist+=WIRE_INCREMENT;
                            if(optionSelected==4)
                                numWires++;
                            refreshScreen = true;
                            break;
                        case DOWN_PRESSED:
                            if(optionSelected==1)
                                wireLength-=WIRE_INCREMENT;
                            if(optionSelected==2)
                                leftIncisionDist-=WIRE_INCREMENT;
                            if(optionSelected==3)
                                rightIncisionDist-=WIRE_INCREMENT;
                            if(optionSelected==4)
                                numWires--;
                            refreshScreen = true;
                            break;
                        case ONE_RELEASED:
                            optionSelected=1;
                            refreshScreen = true;
                            break;
                        case TWO_RELEASED:
                            optionSelected=2;
                            refreshScreen = true;
                            break;
                        case THREE_RELEASED:
                            optionSelected=3;
                            refreshScreen = true;
                            break;
                        case FOUR_RELEASED:
                            optionSelected=4;
                            refreshScreen = true;
                            break;
                        case LEFT_RELEASED:
                            currentState = MENU;
                            stateChange = 1;
                            refreshScreen = true;
                            break;
                        case RIGHT_RELEASED:
                            currentState = CUTTING_TWO;
                            numWiresLeft = numWires;
                            stateChange = 1;
                            refreshScreen = true;
                            break;
                        default:
                            refreshScreen = false;
                            break;
                    }
                }
                break;
            }
            case CUTTING_TWO : {
                cutWires();
                if(buttonReady && numWiresLeft==0 && currentButton==RIGHT_RELEASED) {
                    currentState = MENU;
                    stateChange = 1;
                }
                break;
            }
            case SETTINGS_ONE : {
                if (buttonReady) {
                    switch(currentButton) {
                        case ONE_RELEASED:
                                wireLeft = MAX_SPOOL_LENGTH;
                                break;
                        case TWO_RELEASED:
                                currentState = SETTINGS_FEED;
                                wireFeeder.enable();
                                stateChange = 1;
                                refreshScreen = true;
                                break;
                        case THREE_RELEASED:
                                currentState = SETTINGS_CUTTER;
                                stateChange = 1;
                                refreshScreen = true;
                                break;
                        case FOUR_RELEASED:
                                currentState = SETTINGS_GUIDE;
                                stateChange = 1;
                                refreshScreen = true;
                                break;
                        case LEFT_RELEASED:
                                currentState = MENU;
                                stateChange = 1;
                                refreshScreen = true;
                                break;
                        default : 
                            break;
                    }
                }
                break;
            }
            case SETTINGS_FEED : {
                if (buttonReady) {
                    switch(currentButton) {
                        case UP_PRESSED:
                            while(!(currentButton==UP_RELEASED)){
                                wireFeeder.step(FEEDER_MOTOR_STEPS,STEPPER_REV,FEEDER_MOTOR_SPEED);
                                Thread::wait(5);
                            }
                            break;
                        case DOWN_PRESSED:
                            while(!(currentButton==DOWN_RELEASED)){
                                wireFeeder.step(FEEDER_MOTOR_STEPS,STEPPER_FWD,FEEDER_MOTOR_SPEED);
                                Thread::wait(5);
                            }
                            break;
                        case LEFT_RELEASED:
                                currentState = SETTINGS_ONE;
                                wireFeeder.disable();
                                stateChange = 1;
                                refreshScreen = true;
                                break;
                        default : 
                            break;
                    }
                }
                break;
            }
            case SETTINGS_CUTTER : {
                if (buttonReady) {
                    switch(currentButton) {
                        case UP_PRESSED:
                            wireCutter.speed(CUTTER_MOTOR_SPEED);
                            while(!(currentButton==UP_RELEASED)){
                                Thread::wait(50);
                            }
                            wireCutter.speed(0.0);
                            break;
                        case DOWN_PRESSED:
                            wireCutter.speed(-CUTTER_MOTOR_SPEED);
                            while(!(currentButton==DOWN_RELEASED)){
                                Thread::wait(50);
                            }
                            wireCutter.speed(0.0);
                            break;
                        case LEFT_RELEASED:
                                currentState = SETTINGS_ONE;
                                wireFeeder.disable();
                                stateChange = 1;
                                refreshScreen = true;
                                break;
                        default : 
                            break;
                    }
                }
                break;
            }
            case SETTINGS_GUIDE : {
                if (buttonReady) {
                    switch(currentButton) {
                        case UP_RELEASED:
                            wireGuide.position(++guideAngle);
                            refreshScreen = true;
                            break;
                        case DOWN_RELEASED:
                            wireGuide.position(--guideAngle);
                            refreshScreen = true;
                            break;
                        case LEFT_RELEASED:
                                currentState = SETTINGS_ONE;
                                stateChange = 1;
                                refreshScreen = true;
                                break;
                        default : 
                            break;
                    }
                }
                break;
            }
            default :
                break;
        }
        buttonReady = 0;
        Thread::wait(50);  
    }
}