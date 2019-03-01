#include <Wire.h>

#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
SFEVL53L1X distanceSensor;

// The i2c address of the TCA i2c mux
#define TCAADDR 0x74

 // the sensor MUX indices from left to right 
int offsets[] = {3,2,1,0,7,6,5,4}; 

// Select one of the devices on the i2c mux: 0-7
void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// 
void setup(void)
{
  Wire.begin();

  Serial.begin(115200);
  Serial.println("Starting Lidar Array");

  delay(2000);

  // Set each lidar's configuration
  for(int i=0;i<8;i++){
    tcaselect(i);  // Select the last port in the chain
    distanceSensor.begin();
    distanceSensor.setIntermeasurementPeriod(35);
  }

}

// The output range data array
uint16_t ranges[] = {0,0,0,0,0,0,0,0};

void loop(void)
{
  // process even and odd lidars in batches (to maximize speed and minimize interference)
  for(int j=0;j<2;j++) {
    // start current batch rangefinding
    for(int i=0;i<4;i++) {
      tcaselect(offsets[2*i+j]);  // Select mux device
      distanceSensor.startRanging();
    }
  
    // Wait for completion of measurement. Takes 40-50ms.
    delay(50);

    // read the distances into the ranges array 
    for(int i=0;i<4;i++) {
      tcaselect(offsets[2*i+j]);  // Select mux device
      ranges[i*2+j] = distanceSensor.getDistance();  // returns mm
      distanceSensor.stopRanging();
    }
    // Sleep some more to avoid lidars drawing too much current
    delay(40);
  }

  // Emit the ranges as ascii digits, comma seperated with a newline and the end
  for (int i=0;i<8;i++) {
    Serial.print(ranges[i]);
    if(i<7) Serial.print(',');
  }
  Serial.println();
}
