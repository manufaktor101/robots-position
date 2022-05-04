#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <arduino.h>
#include <math.h>

////////////////// PARAMETERS //////////////////
//position
int32_t coordinates[3];
uint8_t dimension = POZYX_3D;  // positioning dimension
int32_t height = 1000;     // height of device, required in 2.5D positioning
uint8_t algorithm = POZYX_POS_ALG_UWB_ONLY;   // positioning algorithm to use. try POZYX_POS_ALG_TRACKING for fast moving objects.
//
//anchors
const uint8_t num_anchors = 4;                                    // the number of anchors
uint16_t anchors[num_anchors] = {0x6f4a, 0x6f22, 0x6f77, 0x1134};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x[num_anchors] = {0, 5932, 4480, 788};               // anchor x-coordinates in mm
int32_t anchors_y[num_anchors] = {0, 0, 3927, 1935};                  // anchor y-coordinates in mm
int32_t heights[num_anchors] = {457, 456, 443, 2036};              // anchor z-coordinates in mm
//
//data for go to coordinates
float angleChangeRadian; //angle to the target point
float angleChangeRadian2; //angle to the target point
float angleChange; //angle to the target point
float xTarget = 0 ; //target coordinate
float yTarget = 0 ; //target coordinate
int16_t heading_raw;
float CurrentAngle;
float Vx;
float Vy;
float Vtx;
float Vty;
//

////////////////////////////////////////////////


void setup(){
Serial.begin(115200);

  //Initialize Pozyx device
  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }

   setAnchorsManual();
}

void loop() {

  //Position
  int status;
  coordinates_t position;

  status = Pozyx.doPositioning(&position, dimension, height, algorithm);//start positioning

  Pozyx.regRead(POZYX_POS_X, (uint8_t*)&coordinates, 3*sizeof(int32_t)); //read position
  Pozyx.regRead(POZYX_EUL_HEADING, (uint8_t*)&heading_raw, sizeof(int16_t));

  //angle to target
 CurrentAngle = heading_raw/ POZYX_EULER_DIV_DEG;
 Vx = cos(CurrentAngle*(PI/180));//vector from heading
 Vy = sin(CurrentAngle*(PI/180));//vector from heading
 Vtx = xTarget-coordinates[0];//vector to target
 Vty = yTarget-coordinates[1]; //vector to target
 
  angleChangeRadian2 = atan2(Vx*Vty-Vy*Vtx,Vx*Vtx+Vy*Vty); //calculate angle to target vector with minus

  //angleChangeRadian = acos(((Vtx*Vx)+(Vty*Vy))/((sqrt(sq(Vtx)+sq(Vty))*sqrt(sq(Vx)+sq(Vy))))); //calculate angle to target vector

  angleChange = angleChangeRadian2 * (180/PI);
  
//  Serial.print(coordinates[0]);
//  Serial.print('-');
//  Serial.println(coordinates[1]);
  Serial.print(angleChange);
  Serial.print('\\');
  //Serial.println(angleChangeRadian);
  Serial.println(angleChangeRadian2);
 // Serial.println(CurrentAngle);

}






void setAnchorsManual(){
  for(int i = 0; i < num_anchors; i++){
    device_coordinates_t anchor;
    anchor.network_id = anchors[i];
    anchor.flag = 0x1;
    anchor.pos.x = anchors_x[i];
    anchor.pos.y = anchors_y[i];
    anchor.pos.z = heights[i];
  }
}
