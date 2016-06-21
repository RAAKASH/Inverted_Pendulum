#include "I2Cdev.h"
#include<math.h>
#include <LMotorController.h>

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };





volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// Varable declaration


 
 
 #define enablea 8  // Enable Pin for motor 1
#define enableb 7  // Enable Pin for motor 2
 
#define motorf 5  // Control pin 1 for motor 1
#define motorb 9  // Control pin 2 for motor 1
#define motorf 5  // Control pin 1 for motor 2
#define motorb 9  // Control pin 2 for motor 2

LMotorController motorController(enablea, motorf, motorb, enableb, motorf, motorb, 1, 1);



void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz).
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

     Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println("\nSend any character to begin DMP programming and demo: ");
    while (Serial.available() && Serial.read()); // empty buffer
  //  while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println("Initializing DMP...");
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(54);
    mpu.setYGyroOffset(-27);
    mpu.setZGyroOffset(-5);
    mpu.setZAccelOffset(1851); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
    
    /****************** Main project**************************/
   
    
    pinMode(motorf, OUTPUT);  // These are PWM pins output to motor
    pinMode(motorb, OUTPUT);  // These are PWM pins output to motor
    pinMode(enablea, OUTPUT);
    pinMode(enableb, OUTPUT);
    digitalWrite(enablea,HIGH);
    digitalWrite(enableb,HIGH);
    
    /********************************************************/
    
    
    
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

     if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
     // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

        } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

          mpu.getFIFOBytes(fifoBuffer, packetSize);
        
          fifoCount -= packetSize;


        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            //Serial.print(ypr[0] * 180/M_PI);
            //Serial.print("\t");
            //Serial.print(ypr[1] * 180/M_PI);
            //Serial.print("\t");
            //Serial.println(ypr[2] * 180/M_PI);
           
            
 /* **************************************************************************************** */
 /* ****************************  Main Project code  *************************************** */
 /* **************************************************************************************** */
           
           
 float y,p,r,setp,Actual,Error,Last,Drive,Integral;
 float P,I,D;
  //Proportional control , Derivative control , Integral control constants
           
    
           // Need to tinker with these
           Integral = 0;
           float kP = 4, kI = 2, kD = 1;
          
           
           int IntThresh = 30; //30 degree difference
           float ScaleFactor = 1 ;  // arbitrary must check with model
           
           Last = 0 ;  // used for derivative control.
           
           
           
           //defining variables for yaw ,pitch and roll.
           y = ypr[0] * 180/M_PI;  // Yaw
           p = ypr[1] * 180/M_PI;  // Pitch
           r = ypr[2] * 180/M_PI;  // Roll
            
           setp  = 0.53 ;  // Setpoint , to check 
           
           
           Actual =  p ;   // may be ,p or r also  Hence check with the model that we made

           Error = setp - Actual; 
          // Serial.print("Error: ");  
           //Serial.println(Error);

         if (abs(Error) < IntThresh)
          { // prevent integral 'windup' 

          Integral = Integral + Error; // accumulate the error integral 

           } 

         else { 

           Integral=0; // zero it if out of bounds 

               } 

           P = Error*kP; // calc proportional term 

           I = Integral*kI; // integral term 

           D = (Last-Actual)*kD; // derivative term 

           Drive = P + I + D; // Total drive = P+I+D 
          
           Drive = Drive*ScaleFactor; // scale Drive to be in the range 0-255 
         Serial.print("Drive:  ");
           Serial.println(Drive);    
           
           if (abs(Drive)>255) { 

              Drive=255; 

               } 

           
           if (Drive < 0)
           { 
             // Check which direction to go.
             // In our case ive assumed it to be backwards  do check it.            
             
            analogWrite(8,floor(-Drive));
            analogWrite(7,floor(-Drive));
            
            
            
            
            digitalWrite (motorf,LOW); // change direction as needed
             
            //analogWrite (motorb,floor(-Drive)); // send PWM command to motor board 
            digitalWrite (motorb,HIGH);
              } 

           else { // depending on the sign of Error 
                  // Again need to be checked
            
            analogWrite(8,floor(Drive));
            analogWrite(7,floor(Drive));
                         
             digitalWrite (motorf,HIGH);     
             digitalWrite (motorb,LOW);// change direction as needed 
           
           
                } 

           
            

           Last = Actual; // save current value for next time 
            
            
                  
            
            
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
