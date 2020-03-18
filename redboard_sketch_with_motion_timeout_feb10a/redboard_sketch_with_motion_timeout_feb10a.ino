/* Sweep
by RAWSINGULARITY/PRATIKPANCHAL22 <http://pratikpanchal.com>
This example code is in the public domain.

modified 22 Oct 2018
by Pratik Panchal
*/

#include <Servo.h>


typedef enum {
 unknownMode = 0,
 automaticMode,
 manualMode,
 sweepMode,
 panSweepMode,
 tiltSweepMode,
 sweep1Mode,
 noMotionMode,
 dcMotorMode,
 helpMode,
 //
 stepperMode,
} Modes_t;

typedef enum {
  unknownDevice = 0,
  leftMotor,
  rightMotor,
  verticalMotor,
  panServo,
  tiltServo,
  gripperServo,
  dispatchServo,
} Devices_t;

typedef enum {
 parmUnknown = 0,
 parmTargetValue,
 parmMinLimitValue,
 parmMaxLimitValue,
 parmJumpOffsetValue,
 parmTimeDelayValue,
 parmCurrentValue,
 parmLastChangeTimeStamp,
} DeviceParm_t;

typedef struct {
  Devices_t     device;
  int           currentValue;
  unsigned long lastChangeTimeStamp;  // ms
  int           targetValue;
  int           minLimitVal;
  int           maxLimitVal;
  int           jumpOffsetValue;      // By how much should the value change from current to target (min=1; max=255 (direct change, for example for servos))
  int           timeDelay;            // What should be the timeDelay in between the changes (ms)
} DeviceRow_t;

typedef struct {
  Devices_t     device;
  int           triggerValue;
  unsigned long startTime;
  int           timeout;
  int           targetValue;
} TimeoutMatrixRow_t;

// On powerup, targetValues are defaultValues that devices need to be set on
// To make sure, initialize deviceMatrix currentValue with a outOfBound invalid value ...
//    so that the inertial engine changes it to targetValue (i.e. defaultValue on startup)
// !!! Do not do this for DC motors  for currentValue != 1 !!!
DeviceRow_t deviceMatrix[] = {
  /* device     currentValue,  lastChangeTimeStamp, targetValue, minLimitVal, maxLimitVal, jumpOffsetValue, timeDelay */
  {leftMotor,              1,                    0,           0,        -255,         255,               1,         5     },
  {rightMotor,             1,                    0,           0,        -255,         255,               1,         5     },
  {verticalMotor,          1,                    0,           0,        -255,         255,               1,         5     },
  {panServo,               0,                    0,           0,           0,         180,             255,         5     },
  {tiltServo,              0,                    0,           0,           0,         180,             255,         5     },
  {gripperServo,           0,                    0,          40,          35,          72,             255,         0     },
  {dispatchServo,          0,                    0,           0,           0,         180,             255,         5     },
};

TimeoutMatrixRow_t timeoutMatrix[] = {
  /* device     triggerValue,            startTime,     timeout, targetValue*/     
  {verticalMotor,        100,                    0,        3050,          75,   },
  //{leftMotor,              1,                    0,        5000,           0,   },
  //{rightMotor,             1,                    0,        5000,           0,   },
};

//Servo panServo;  // create servo object to control a servo
//Servo tiltServo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
long randPanAngle;
long randTiltAngle;

int ByteReceived;
int angle;

int mode = unknownMode;
int panAngleInput;
int tiltAngleInput;


// PIN assignments
// Left Motor (lm)
#define LM_EN    6      //ENA (~)
#define LM_DIR1  7      //IN1
#define LM_DIR2  8      //IN2

// Right Motor (rm)
#define RM_EN   11      //ENB (~)
#define RM_DIR1 12      //IN3
#define RM_DIR2 13      //IN4

// Vertical lift Motor (vm)
#define VM_EN    3      //EN (~)
#define VM_DIR1  2      //IN
#define VM_DIR2  4      //IN

// Gripper Motor (gm)
#define GS_EN    5
Servo gripperServoObj;



//Declare variables for functions
char user_input;
int x;
int y;
int state;

// stepper
int stepperInput=0;
int delayParm = 1;

// Prototypes
boolean setDeviceParameter(Devices_t device, DeviceParm_t deviceParm, int value);
void getSerialDataAndParse(void);
void   inertialEngine(void);
void motionTimeoutEngine(void);
unsigned long getDeviceParameter(Devices_t device, DeviceParm_t deviceParm);

void setup() {

 /*
 panServo.attach(7);  // attaches the servo on pin 7 to the servo object
 tiltServo.attach(6);  // attaches the servo on pin 6 to the servo object
 */

 // Begin Serial
 Serial.begin(500000);


 // Set Pin Modes
 // LM
 pinMode(LM_EN,   OUTPUT);
 pinMode(LM_DIR1, OUTPUT);
 pinMode(LM_DIR2, OUTPUT);

 // RM
 pinMode(RM_EN,   OUTPUT);
 pinMode(RM_DIR1, OUTPUT);
 pinMode(RM_DIR2, OUTPUT);

 // VM
 pinMode(VM_EN,   OUTPUT);
 pinMode(VM_DIR1, OUTPUT);
 pinMode(VM_DIR2, OUTPUT);

 // GM
 //pinMode(GS_EN,   OUTPUT);
 gripperServoObj.attach(GS_EN);

 /*
 resetEDPins(); //Set step, direction, microstep and enable pins to default states
 */

 printHelp();

}

void printHelp() {
 Serial.println("~Hardware Interface~");
 Serial.println(" Modes: ");
 Serial.println(" Left Motor: lm=200, lm=-150");
 Serial.println(" Right Motor: rm=200, rm=-150");
 Serial.println(" Both Motors: bm=200, bm=-150");
 Serial.println(" Vertical lift Motor: vm=200, vm=-150");
 Serial.println();
}

void loop() {

 /*
  * lm = left motor
  * rm = right motor
  * vm = vertical lift motor
  * ps = pan servo
  * ts = tilt servo
  * gs = gripper servo
  * ds = dispatch servo
  */

  // PSEUDO-THREADS
  /*
     Pseudo-threads are function calls which do a discrete task before
     giving up the processor for other tasks
     Rules:
     1. There should be no delay() in a pseudo-thread, or any blocking
        call like while(1)
     2. The pseudo-thread should maintain its own private data to clock
        through its internal state machine.
  */

  // Get serial data and parse it
  getSerialDataAndParse();

  // Inertial Engine - loop
  inertialEngine();

  // Motion timeout engine - loop
  motionTimeoutEngine();



}


boolean setDeviceToValue(Devices_t targetDevice, int targetValue) {

  boolean retState = false;

  switch (targetDevice) {
     case leftMotor:
        // Set DIR pin
        if(targetValue > 0) {
           digitalWrite(LM_DIR1, HIGH);
           digitalWrite(LM_DIR2, LOW);
        }
        else if (targetValue < 0) {
           digitalWrite(LM_DIR1, LOW);
           digitalWrite(LM_DIR2, HIGH);
           targetValue *= (-1);
        }
        else {
           digitalWrite(LM_DIR1, LOW);
           digitalWrite(LM_DIR2, LOW);
        }

        // Set EN pin
        analogWrite(LM_EN, targetValue);
        retState = true;
        break;

     case rightMotor:
        // Set DIR pin
        if(targetValue > 0) {
           digitalWrite(RM_DIR1, LOW);
           digitalWrite(RM_DIR2, HIGH);
        }
        else if (targetValue < 0) {
           digitalWrite(RM_DIR1, HIGH);
           digitalWrite(RM_DIR2, LOW);
           targetValue *= (-1);
        }
        else {
           digitalWrite(RM_DIR1, LOW);
           digitalWrite(RM_DIR2, LOW);
        }

        // Set EN pin
        analogWrite(RM_EN, targetValue);
        retState = true;
        break;

     case verticalMotor:
        // Set DIR pin
        if(targetValue > 0) {
           digitalWrite(VM_DIR1, HIGH);
           digitalWrite(VM_DIR2, LOW);
        }
        else if (targetValue < 0) {
           digitalWrite(VM_DIR1, LOW);
           digitalWrite(VM_DIR2, HIGH);
           targetValue *= (-1);
        }
        else {
           digitalWrite(VM_DIR1, LOW);
           digitalWrite(VM_DIR2, LOW);
        }

        // Set EN pin
        analogWrite(VM_EN, targetValue);
        retState = true;
        break;


     case gripperServo:
        gripperServoObj.write(targetValue);
        retState = true;
        break;


     default:
        Serial.print("Unknown targetDevice: ");
        Serial.println(targetDevice);
        retState = false;
        break;
  }

  return retState;
}


unsigned long getDeviceParameter(Devices_t device, DeviceParm_t deviceParm) {
 int idx=0;
 unsigned long retVal;

 for(idx=0; idx<sizeof(deviceMatrix)/sizeof(DeviceRow_t); idx++) {
   if(deviceMatrix[idx].device == device) {
     switch(deviceParm) {
       case parmTargetValue:
          return (deviceMatrix[idx].targetValue);
          break;
          
       case parmCurrentValue:
          return (deviceMatrix[idx].currentValue);
          break;

       case parmLastChangeTimeStamp:
          return (deviceMatrix[idx].lastChangeTimeStamp);
          break;

       default:
          Serial.print("Unknown parm: ");
          Serial.println(deviceParm);
          break;
      }
    }
  }
}

boolean setDeviceParameter(Devices_t device, DeviceParm_t deviceParm, int value) {
 int idx=0;
 boolean retVal = false;

 for(idx=0; idx<sizeof(deviceMatrix)/sizeof(DeviceRow_t); idx++) {
   if(deviceMatrix[idx].device == device) {
     switch(deviceParm) {
       case parmTargetValue:
          deviceMatrix[idx].targetValue = value;
          retVal = true;
       break;

       default:
          Serial.print("Unknown parm: ");
          Serial.println(deviceParm);
       break;
     }
     break;
   }
 }

 if(retVal) {
   Serial.print(" >> Success setting device=");
   Serial.print(device);
   Serial.print(" deviceParm=");
   Serial.print(deviceParm);
   Serial.print(" value=");
   Serial.println(value);
   yield();
 }
 else {
   Serial.println(" >> Failed setting inertial engine parm ");
   Serial.print(deviceParm);
   Serial.print(" for device ");
   Serial.println(device);
 }
 return retVal;
}

void getSerialDataAndParse() {

  String cmdStr;

  if (Serial.available() > 3) {

    //ByteReceived = Serial.read();
    cmdStr = Serial.readStringUntil(';');

    cmdStr.trim();

    Serial.print("rcvd: ");
    Serial.println(cmdStr);
    Serial.print("Len: ");
    Serial.print(cmdStr.length());

    /*
    int semiColonIdx = cmdStr.indexOf(';');
    Serial.print(" semiColonIdx: ");
    Serial.println(semiColonIdx);
    */

    //cmdStr = cmdStr.substring(0,cmdStr.length()-2);

    Serial.print("Parsed cmd: ");
    Serial.println(cmdStr);

    if(cmdStr.substring(0,3) == "lm=") {
      int val = cmdStr.substring(3).toInt();
      //Serial.print("device: lm. Value: ");
      //Serial.println(val);
      setDeviceParameter(leftMotor, parmTargetValue, val);
    }

    else if(cmdStr.substring(0,3) == "rm=") {
      setDeviceParameter(rightMotor, parmTargetValue, cmdStr.substring(3).toInt());
    }

    else if(cmdStr.substring(0,3) == "bm=") {
      setDeviceParameter(rightMotor, parmTargetValue, cmdStr.substring(3).toInt());
      setDeviceParameter(leftMotor,  parmTargetValue, cmdStr.substring(3).toInt());
    }

    else if(cmdStr.substring(0,3) == "gs=") {
      setDeviceParameter(gripperServo, parmTargetValue, cmdStr.substring(3).toInt());
    }

    else if(cmdStr.substring(0,3) == "vm=") {
      setDeviceParameter(verticalMotor, parmTargetValue, cmdStr.substring(3).toInt());
    }

    else if(cmdStr.substring(0,3) == "es=") {
      if(cmdStr.substring(3) == "all") {
         setDeviceParameter(verticalMotor, parmTargetValue, 0);
         setDeviceParameter(leftMotor, parmTargetValue, 0);
         setDeviceParameter(rightMotor, parmTargetValue, 0);
      }
    }

    // Other modes
    else if(cmdStr == "stop") {
      mode = noMotionMode;
      Serial.print("~no motion mode~");
    }

    else if(cmdStr == "help") {
      mode = helpMode;
      printHelp();
    }

    else {
      //mode = unknownMode;
      //printHelp();
      Serial.println("~unknown mode. skipping...~");
    }

  // END Serial Available
  }
}



void inertialEngine() {
  int idx=0;
  int newValue = 0;

  for(idx=0; idx<sizeof(deviceMatrix)/sizeof(DeviceRow_t); idx++) {

     // if the currentValue = targetValue, skip this
     if(deviceMatrix[idx].currentValue == deviceMatrix[idx].targetValue) {
        continue;
     }

     // if here, it means currentValue != targetValue
     // in that case see if sufficient time has passed to change currentValue
     unsigned long currentTimeStamp = millis();  //returns number of milliseconds passed since powerup
     if(currentTimeStamp - deviceMatrix[idx].lastChangeTimeStamp < deviceMatrix[idx].timeDelay) {
        continue;
     }

     // if here, it means that currentValue != targetValue. Make sure that targetValue in in valid range
     // if not, adjust it to fall in range.
     if(deviceMatrix[idx].targetValue < deviceMatrix[idx].minLimitVal) {
        deviceMatrix[idx].targetValue = deviceMatrix[idx].minLimitVal;
     }
     if(deviceMatrix[idx].targetValue > deviceMatrix[idx].maxLimitVal) {
        deviceMatrix[idx].targetValue = deviceMatrix[idx].maxLimitVal;
     }

     // if here, it means that sufficient time has passed. Compute the newValue
     newValue = deviceMatrix[idx].currentValue;
     if(deviceMatrix[idx].targetValue > deviceMatrix[idx].currentValue) {
        newValue += deviceMatrix[idx].jumpOffsetValue;

      if(newValue > deviceMatrix[idx].targetValue) {
         newValue = deviceMatrix[idx].targetValue;
      }
     }
     else if(deviceMatrix[idx].targetValue < deviceMatrix[idx].currentValue) {
        newValue -= deviceMatrix[idx].jumpOffsetValue;

      if(newValue < deviceMatrix[idx].targetValue) {
         newValue = deviceMatrix[idx].targetValue;
      }
     }

     // if here, it means that newValue has been computed. Time to apply it to hardware
     if(setDeviceToValue(deviceMatrix[idx].device, newValue) == true) {
        // newValue has been applied to hardware. Update the table
        deviceMatrix[idx].currentValue = newValue;
        // Update the timeStamp
        deviceMatrix[idx].lastChangeTimeStamp = currentTimeStamp;
        //Serial.print("# Setting Device; ");
        //Serial.print(deviceMatrix[idx].device);
        //Serial.print(" To value: ");
        //Serial.println(newValue);
     }
     else {
        // Applying changes to hardware failed.
        Serial.println("Error! Setting hardware state failed");
     }

     yield();
  }
}

void motionTimeoutEngine() {
  int idx;
  int currentVal;
  int multiplier;
  
  for(idx=0; idx<sizeof(timeoutMatrix)/sizeof(TimeoutMatrixRow_t); idx++) {
    currentVal = (int)(getDeviceParameter(timeoutMatrix[idx].device, parmCurrentValue));
    if(abs(currentVal) >= timeoutMatrix[idx].triggerValue) {
      if(timeoutMatrix[idx].startTime == 0) {
         timeoutMatrix[idx].startTime = millis();
      }

      multiplier = (int)(getDeviceParameter(timeoutMatrix[idx].device, parmCurrentValue)) / 
                    abs((int)(getDeviceParameter(timeoutMatrix[idx].device, parmCurrentValue)));
      
      if(millis() - timeoutMatrix[idx].startTime >= timeoutMatrix[idx].timeout) {
        if(getDeviceParameter(timeoutMatrix[idx].device, parmTargetValue) != multiplier*timeoutMatrix[idx].targetValue) {
            setDeviceParameter(timeoutMatrix[idx].device, parmTargetValue, (multiplier*timeoutMatrix[idx].targetValue));  
        }
      }
    }
    else {
      timeoutMatrix[idx].startTime = 0;
    }
  }
  
}

