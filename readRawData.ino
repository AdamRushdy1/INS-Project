#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);

Adafruit_MPU6050 imu;
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);

// Assisted with Claude for mag calibration

// Calibration offsets (fill in after running calibration)
float magOffsetX = 0, magOffsetY = 0, magOffsetZ = 0;  // hard iron
float magScaleX  = 1, magScaleY  = 1, magScaleZ  = 1;  // soft iron

bool calibrated = false;

// Hard/soft iron calibration
void calibrateMag() {
  Serial.println("CALIBRATING: rotate sensor in all directions for 15 seconds...");

  float minX =  1e9, minY =  1e9, minZ =  1e9;
  float maxX = -1e9, maxY = -1e9, maxZ = -1e9;

  unsigned long start = millis();
  while (millis() - start < 15000) {
    sensors_event_t event;
    compass.getEvent(&event);
    float x = event.magnetic.x;
    float y = event.magnetic.y ;
    float z = event.magnetic.z;

    minX = min(minX, x); maxX = max(maxX, x);
    minY = min(minY, y); maxY = max(maxY, y);
    minZ = min(minZ, z); maxZ = max(maxZ, z);

    delay(20);
  }

  // Hard iron: shift centre to origin
  magOffsetX = (maxX + minX) / 2.0;
  magOffsetY = (maxY + minY) / 2.0;
  magOffsetZ = (maxZ + minZ) / 2.0;

  // Soft iron: normalise axis ranges to equal length
  float rangeX = (maxX - minX) / 2.0;
  float rangeY = (maxY - minY) / 2.0;
  float rangeZ = (maxZ - minZ) / 2.0;
  float avgRange = (rangeX + rangeY + rangeZ) / 3.0;

  magScaleX = avgRange / rangeX;
  magScaleY = avgRange / rangeY;
  magScaleZ = avgRange / rangeZ;

  calibrated = true;
  Serial.println("CALIBRATION DONE");
}

void setup(void) {
  Serial.begin(115200);
  Wire.begin();
  GPSSerial.begin(9600, SERIAL_8N1, 16, 17);

  if (!imu.begin()) {
    Serial.println("Failed to find IMU");
    while (1) delay(10);
  }
 
  if (!compass.begin()) {
    Serial.println("Failed to find HMC5883L");
    while (1) delay(10);  
  }

  // Send 'C' over serial to trigger calibration, or skip with any other key
  Serial.println("Send 'C' to calibrate compass, any other key to skip...");
  unsigned long wait = millis();
  while (millis() - wait < 5000) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'C' || c == 'c') { calibrateMag(); }
      break;
    }
  }
}

void loop(void) {
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }

  // Read IMU
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  // Output
  // Format: ax,ay,az,gx,gy,gz,magx,magy,magz,lat,lon,alt,speed,course,sats,hdop

  Serial.print(a.acceleration.x); Serial.print(",");
  Serial.print(a.acceleration.y); Serial.print(",");
  Serial.print(a.acceleration.z); Serial.print(",");
  Serial.print(g.gyro.x);         Serial.print(",");
  Serial.print(g.gyro.y);         Serial.print(",");
  Serial.print(g.gyro.z);         Serial.print(",");

  sensors_event_t event;
  compass.getEvent(&event);

  float magx = (event.magnetic.x - magOffsetX) * magScaleX;
  float magy = (event.magnetic.y - magOffsetY) * magScaleY;
  float magz = (event.magnetic.z - magOffsetZ) * magScaleZ;

  Serial.print(magx, 2);       Serial.print(",");
  Serial.print(magy, 2);       Serial.print(",");
  Serial.print(magz, 2);       Serial.print(",");

  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6); Serial.print(",");
    Serial.print(gps.location.lng(), 6); Serial.print(",");
    Serial.print(gps.altitude.meters());  Serial.print(",");
    Serial.print(gps.speed.mps());        Serial.print(",");
    Serial.print(gps.course.deg());       Serial.print(",");
    Serial.print(gps.satellites.value()); Serial.print(",");
    Serial.println(gps.hdop.hdop());
  } else {
    Serial.println("0,0,0,0,0,0,0");
  }

  delay(20);
}
