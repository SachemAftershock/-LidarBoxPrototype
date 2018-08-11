// LidarBoxPrototype
//
// Lidar Lite 3 (am-3829)
// http://www.andymark.com/product-p/am-3829.htm
// http://files.andymark.com/PDFs/pli-06-instruction.pdf
// 
// 9-Axis Adafruit BNO055 Absolute Orientation Sensor
// https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code


// The following discussion is to help provide context and definitions to
// understand the lidar box design and application to the robot.
// The lidar box will rotate a mirror to provide 360 degree rotations of the
// lidar beam to provide distance measurements.  As many measurements per rotation
// shall be commanded and processed as is acheivable, limited by the sensors, mechanics, 
// and electronics speed capabilties.  The processing requires knowing what angle
// each measurement is taken at. Thus each measure forms a polar coordinate.
// This design defines the scan trigger as the up-edge transition of measures from below 
// the minimum distance threshold- representing the rotation exiting the 1st quadrant of 
// the a 360 degree mirror rotation.  The scan end trigger is when the down-edge transition
// to below the minimum distance threshold.  The lidar box is a 4 sided box with one 
// wall being solid (blocking the lidar beam resulting in very short distances 
// measured, and the other three walls open.  Thus the angle for each measurement is 
// implicit as 270 degress divided by the number of measures in the scan.  The Lidar box 
// is mounted on the aft of the robot with the opaque quadrant of the box facing 
// the interior of the robot.  Thus, quadrant 1 creates the 90deg blanking pulse, 
// quadrant 2 (90deg) faces robot's starboard, Q3 faces aft, Q4 faces port.
// This definition has angle Naught1 logically at robot relative front left corner. 
// This in contrast to robot heading, defining Naught2 at center of Q1, 
// a.k.a. Naught2=Naught1+45deg.  Lidar box heading is Naught3=Naught2+180deg.
// The compass (currently a mangetometer) will have some angular value, and at 
// autonomous start, its measured value will be remembered as Naught4, so that for the 
// duration of autonomous mode the present angular change from Naught4 may be subtracted from 
// each measure's implicit angle. This restores the orientation of the collected 
// data to be field relative for the following wall finding algorithm. Range to the three 
// field walls will be calculated from the measurements of a single scan, by analyzing 
// each of its meaningful three quadrants (Q2-Q4) separately.  For each quadrant, the 
// cosine of the measurement's angle relative to the center angle of that quadrant as its 
// naught will be multiplied with its corresponding meaure (a.k.a. hypotneus), resulting 
// in the cartesian distance to that field wall.  The maximum of all that quadrant's 
// distances will then be the selected distance to that wall.  This will compute the 
// correct distance as long as at least one measure was un-occluded by other robots, field 
// elements, etc.  The robot start position is likely to allow this to work-out for all 
// three walls.  The algorithm will reconcile port and starboard distances to constrain the 
// robot position to within the known field width.  The algorithm will employ a 
// self-tracker to estimate robot position when occlusions cause results to not resolve 
// well.  


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>

// We'll use the default serial port as a monitor as is typically done. 
// Identify the Arduiino pins that the 2nd serial port will run using.
// https://www.arduino.cc/en/Tutorial/SoftwareSerialExample
SoftwareSerial RoboRioSerial(10, 11); // RX, TX

// Define array to hold each measure of a single 
// revolution of the Lidar mirror box.
const int scan_buffer_size = 350;
int measure_counter = 0;
struct scan_record {
  unsigned long measure_timestamp;
  int measured_distance;
  float robot_azimuth_referenced_to_start_position;    
} scan[scan_buffer_size];

// Define global constants and variables. 
unsigned long ScanCollectionTimerStartInMs = 0;
const unsigned long ScanCollectionTimeInMs = 1000;
const unsigned long ScanCollectionTimeoutInMs = ScanCollectionTimeInMs + 50;

// Define overall state machine modes for this program.
enum mode { collectMode, computeMode, haltMode } theMode = collectMode;

enum RoboRioCommandEnum { noCmd, markAzimuthAtAutonomousStartCmd, getPositionCmd } RoboRioCommand = noCmd;

const int commandReceivedLedOnboardArduino = 13; 

class CRobotPositionAndAttitude {
  public:
    float x;
    float y;
    float az;
    bool GoodQuality;    
} theRobotPositionAndAttitude;

void toggleCommandArrivedLed(int LightDuration = 200)
{
   digitalWrite(commandReceivedLedOnboardArduino, HIGH); 
   delay(LightDuration);
   digitalWrite(commandReceivedLedOnboardArduino, LOW); 
   delay(LightDuration);
}

class CLidar {
  private:
    const byte LIDAR_ADDRESS = 0x62;
    const byte COMMAND_REG = 0x00;
    const byte ReadNoBias = 0x03;
    const byte ReadBias = 0x04;
    const byte STATUS_REG = 0x01;
    const byte DIST_REG = 0x8f;

  public:
    static const int triggerDistanceInCm = 10;
    int measureDistance();
    void waitForScanTrigger();
} theLidar;

int CLidar::measureDistance(){  
  static int bias_commmand_counter = 0;  //TBD may want this to timeout if not called in a while??
  bool busy = true;
  int distanceInCm = 0;
  
  // Set the command to read with bias every 100 iterations,
  // and read without bias most of the time.
  Wire.beginTransmission(LIDAR_ADDRESS);
  Wire.write(COMMAND_REG);
  if (bias_commmand_counter % 100) {
    Wire.write(ReadNoBias);
  } else {
    Wire.write(ReadBias);
  }
  Wire.endTransmission();
  // Wait until Lidar has completed the measure, then get it. 
  // (Check if the device is still reading. If LSB = 0, device is ready.)
  busy = true;
  while (busy) {
    Wire.beginTransmission(LIDAR_ADDRESS);
    Wire.write(STATUS_REG);
    Wire.endTransmission();
    Wire.requestFrom(LIDAR_ADDRESS, 1);
    busy = Wire.read() & 0x01;
  }
  // Read the measured distance.
  Wire.beginTransmission(LIDAR_ADDRESS);
  Wire.write(DIST_REG);
  Wire.endTransmission();
  Wire.requestFrom(LIDAR_ADDRESS, 2);
  distanceInCm = (Wire.read() << 8) | Wire.read();
  
  // Increment bias command counter
  bias_commmand_counter++;
  
  return distanceInCm; 
}

void CLidar::waitForScanTrigger(){
  while (triggerDistanceInCm > measureDistance());
  
  // Begin the scan timer for the scan.
  ScanCollectionTimerStartInMs = millis();
}

class CCompass {
  private:
  // Compass defines
    Adafruit_BNO055 bno;
    float azimuthOffset;
  public: 
    Initialize();
    float measureCompass();
    void snapAzimuthOffset();
    void initialCalibrateCompass();
} theCompass;

CCompass::Initialize(){
  bno = Adafruit_BNO055(55);
  azimuthOffset = 0.0;

  /* Initialise the orientation sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

float CCompass::measureCompass(){
  sensors_event_t event;
  float az = event.orientation.x; // TBD check this, I think this is value that goes beyond plus/minus 360deg
  az = fmod(az,360.0); // remap to [-360..360)
  //az = (az >= 0.0) ? az-180.0 : 360.0-az;  // remap [-180..180]
  return az;
}

void CCompass::snapAzimuthOffset() {
  azimuthOffset = measureCompass();
}

void CCompass::initialCalibrateCompass(){
  /* Any sensor data reporting 0 should be ignored, */ 
  /* 3 means 'fully calibrated" */  
  uint8_t system, gyro, accel, mag;  
  system = gyro = accel = mag = 0;  
  Serial.print("Waiting for gyro to finish calibration...");
  while (gyro < 3) {
    bno.getCalibration(&system, &gyro, &accel, &mag);
  }
  Serial.print("Gyro calibration complete.");
}

//TBD consider make this method of an RoboRioPort class. 
void processCmd(RoboRioCommandEnum theRoboRioCommand) {
  switch (theRoboRioCommand) {
    case noCmd:
      break;
    case markAzimuthAtAutonomousStartCmd:
      toggleCommandArrivedLed();
      theCompass.snapAzimuthOffset();
      toggleCommandArrivedLed();
      toggleCommandArrivedLed();
      break;
    case getPositionCmd:
      toggleCommandArrivedLed();

      RoboRioSerial.print(theRobotPositionAndAttitude.x);
      RoboRioSerial.print(theRobotPositionAndAttitude.y);
      RoboRioSerial.print(theRobotPositionAndAttitude.az);
      RoboRioSerial.print(theRobotPositionAndAttitude.GoodQuality);
      
      toggleCommandArrivedLed();
      toggleCommandArrivedLed();
      break;
  }
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("LidarBoxPrototype");
  RoboRioSerial.begin(115200);

  theCompass.initialCalibrateCompass();  
}

void loop() {

  const float widthOfFieldInCm = 27.0 * 12.0 * 2.54; 
  float x1 = 0.0;
  float x2 = 0.0;
  float scan_angle = 0.0;
  float robot_relative_scan_angle = 0.0;

  switch (theMode) {

    case collectMode:

      if (RoboRioSerial.available()) {
        RoboRioCommand = RoboRioSerial.read();
        Serial.print("RoboRioCmd: ");
        Serial.println((int)RoboRioCommand);
        processCmd(RoboRioCommand);
      }
      else
      {
    
        // Wait for the scan trigger 
        theLidar.waitForScanTrigger();
        
        // Take a distance measurement at the current mirror angle and robot orientation.
        scan[measure_counter].measured_distance = theLidar.measureDistance();
        
        scan[measure_counter].measure_timestamp = micros();
  
        // Retreive the compass corresponds to robot's azimuth.
        scan[measure_counter].robot_azimuth_referenced_to_start_position = theCompass.measureCompass();

        measure_counter++;
  
        // Detect end of scan, or scan timeout.
        if ((scan[measure_counter].measured_distance < CLidar::triggerDistanceInCm) ||
            (millis()-ScanCollectionTimerStartInMs) > ScanCollectionTimeoutInMs) {
          theMode = computeMode;  
        }
   
        break;
        
      case computeMode:

        x1 = 0.0; 
        x2 = 0.0;
        scan_angle = 0.0;
        // cartisean coordinate relative to driver station wall, left side.
        theRobotPositionAndAttitude.x = 0.0;
        theRobotPositionAndAttitude.y = 0.0;

        for (int i=0; i<measure_counter; i++){
          scan_angle = 270.0 * 
                         (scan[i].measure_timestamp-ScanCollectionTimerStartInMs) /
                         (scan[measure_counter-1].measure_timestamp-ScanCollectionTimerStartInMs);
          robot_relative_scan_angle = scan_angle - theCompass.measureCompass();
          if (robot_relative_scan_angle > 180.0) {
            robot_relative_scan_angle -= 180.0;
            // TBD fix y too
          }
          if ((scan_angle >= 0.0) && (scan_angle < 90.0)) { // Quadrant 1 immediately after blanking quadrant.
            x1 = max(x1, scan[i].measured_distance * cos(scan_angle - scan[i].robot_azimuth_referenced_to_start_position - 45.0));
          } else if ((scan_angle >= 90.0) && (scan_angle < 180.0)) { // Quadrant 2 facing the rear wall at start position.
            theRobotPositionAndAttitude.y = max(theRobotPositionAndAttitude.y, (-1.0) * scan[i].measured_distance * cos(scan_angle - scan[i].robot_azimuth_referenced_to_start_position + 45.0));
          } else if ((scan_angle >= 180.0) && (scan_angle < 270.0)) { // Quadrant 3 immediately preceding next blanking period.
            x2 = max(x2, (-1.0) * scan[i].measured_distance * cos(scan_angle - scan[i].robot_azimuth_referenced_to_start_position - 45.0));
          }
        }
        theRobotPositionAndAttitude.x = x1;
        theRobotPositionAndAttitude.az = theCompass.measureCompass(); // get current robot azimuth relative to robot start orientation.
        theRobotPositionAndAttitude.GoodQuality = abs(widthOfFieldInCm-(x1+x2)) < 6.0*2.54; //TBD can add in a tilt detector to also declare bad.
  
        theMode = collectMode;
        
        break;
  
      case haltMode:
        if (RoboRioSerial.available()) {
          RoboRioCommand = RoboRioSerial.read();
          Serial.print("RoboRioCmd: ");
          Serial.println((int)RoboRioCommand);
          processCmd(RoboRioCommand);
        }
        break;
  
    } //switch

  } // RoboRioCmd

  //Serial.println("exit loop");

} //loop()



