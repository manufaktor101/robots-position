#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <arduino.h>
#include <math.h>

////////////////// PARAMETERS //////////////////
//position
int32_t coordinates[3];
uint8_t dimension = POZYX_3D;  // positioning dimension
int32_t height = 0;     // height of device, required in 2.5D positioning
uint8_t algorithm = POZYX_POS_ALG_TRACKING;   // positioning algorithm to use. try POZYX_POS_ALG_TRACKING for fast moving objects. other POZYX_POS_ALG_UWB_ONLY
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
float xTarget = 3000 ; //target coordinate
float yTarget = 1500 ; //target coordinate
int16_t heading_raw;
float CurrentAngle;
float Vx;
float Vy;
float Vtx;
float Vty;
bool rightPosition = false;

//
//string communication to second arduino
String inputString = "";
boolean stringComplete = false;
String center = "$0,0,0,0,0,0,0,0,0,0,0,0#";
String up = "$1,0,0,0,0,0,0,0,0,0,0,0#";
String down = "$2,0,0,0,0,0,0,0,0,0,0,0#";
String left = "$3,0,0,0,0,0,0,0,0,0,0,0#";
String right = "$4,0,0,0,0,0,0,0,0,0,0,0#";
String panLeft = "$0,1,0,0,0,0,0,0,0,0#";
String panRight = "$0,2,0,0,0,0,0,0,0,0#";

////////////////////////////////////////////////


void setup() {
  Serial.begin(115200);

  //Initialize Pozyx device
  if (Pozyx.begin(false, MODE_INTERRUPT, POZYX_INT_MASK_RX_DATA, 0) == POZYX_FAILURE) {
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }

  setAnchorsManual();

  // reserve 100 bytes for the inputString:
  inputString.reserve(100);

  Serial.println(panLeft.length());
}

void loop() {

  //Position
  int status;
  coordinates_t position;

  status = Pozyx.doPositioning(&position, dimension, height, algorithm);//start positioning

  Pozyx.regRead(POZYX_POS_X, (uint8_t*)&coordinates, 3 * sizeof(int32_t)); //read position
  Pozyx.regRead(POZYX_EUL_HEADING, (uint8_t*)&heading_raw, sizeof(int16_t));

  //angle to target
  CurrentAngle = heading_raw / POZYX_EULER_DIV_DEG;
  Vx = cos(CurrentAngle * (PI / 180)); //vector from heading
  Vy = sin(CurrentAngle * (PI / 180)); //vector from heading
  Vtx = xTarget - coordinates[0]; //vector to target
  Vty = yTarget - coordinates[1]; //vector to target

  angleChangeRadian2 = atan2(Vx * Vty - Vy * Vtx, Vx * Vtx + Vy * Vty); //calculate angle to target vector with minus

  //angleChangeRadian = acos(((Vtx*Vx)+(Vty*Vy))/((sqrt(sq(Vtx)+sq(Vty))*sqrt(sq(Vx)+sq(Vy))))); //calculate angle to target vector

  angleChange = angleChangeRadian2 * (180 / PI);
  //
  //  Serial.print(coordinates[0]);
  //  Serial.print('\\');
  //  Serial.println(coordinates[1]);
  //  Serial.println(angleChange);
  //  Serial.print('\\');
  //Serial.println(angleChangeRadian);
  //Serial.println(angleChangeRadian2);
  // Serial.println(CurrentAngle);
  //Serial.print(Vtx);
  //Serial.print('\\');
  //Serial.println(Vty);

  if (-200<Vtx<200 && -200<Vty<200){

    rightPosition = true;
    
          int len = center.length();
  
        for (int i=0; i<len; i++)
        {
          Serial.print(center.charAt(i));
        }
      }
  
  else if (Vtx>200 && Vtx<-200 && Vty>200 && Vty<-200){
    rightPosition = false;
                            
                          
                          
                            
                            /* I suppose I need to be able to run the serial print just one time
                                because it keep shooting over the point, maybe a bool?
                                maybe a do while?
                                state machine? (library)
                          
                                Also how to stop the robot when it's at the coordinates?
                                can I have the angle if statement into a if statement of the Vtz/Vty?
                          
                          
                            */
                            if (-20 <= angleChange && angleChange <= 20) {
                              int len = up.length();
                          
                              for (int i = 0; i < len; i++)
                              {
                                Serial.print(up.charAt(i));
                              }
                            }
                          
                            else if (20 < angleChange && angleChange < 180) {
                          
                              int len = panRight.length();
                          
                              for (int i = 0; i < len; i++)
                              {
                                Serial.print(panRight.charAt(i)); // turn
                              }
                            }
                            else if (-20 > angleChange && angleChange > -180) {
                          
                              int len = panLeft.length();
                          
                              for (int i = 0; i < len; i++)
                              {
                                Serial.print(panLeft.charAt(i));
                              }
                            }
  }
  //
  //else if (Vtx<20 && Vtx>-20 && Vty<20 && Vty>-20){
  //
  //
  //      int len = center.length();
  //
  //      for (int i = 0; i < len; i++)
  //      {
  //        Serial.print(center.charAt(i));
  //      }
  //
  //    }



  //angle to motors communication

  // we wait up to 50ms to see if we have received an incoming message (if so we receive an RX_DATA interrupt)
  if (Pozyx.waitForFlag(POZYX_INT_STATUS_RX_DATA, 50))
  {
    // we have received a message!
    uint8_t length = 0;
    uint16_t messenger = 0x00;
    delay(1);
    // Let's read out some information about the message (i.e., how many bytes did we receive and who sent the message)
    Pozyx.getLastDataLength(&length);
    Pozyx.getLastNetworkId(&messenger);

    char data[length];

    // read the contents of the receive (RX) buffer into a character array called data with the
    //same length as the contents of the buffer, this is the message that was sent to this device
    Pozyx.readRXBufferData((uint8_t *) data, length);

    //convert char array data into a string

    Serial.println(data);

    inputString = String(data);


    if (inputString == "panLeft")
    {

      int len = panLeft.length();

      for (int i = 0; i < len; i++)
      {
        Serial.print(panLeft.charAt(i));
      }
    }


    if (inputString == "panRight")
    {

      int len = panRight.length();

      for (int i = 0; i < len; i++)
      {
        Serial.print(panRight.charAt(i));
      }
    }


    //compare the string literal with one of the string variables


    //refactor for loop with foreach loop if the resulting effect is desirable

    if (inputString == "up")
    {

      int len = up.length();

      for (int i = 0; i < len; i++)
      {
        Serial.print(up.charAt(i));
      }

    }
    else if
    (inputString == "down")
    {

      int len = down.length();

      for (int i = 0; i < len; i++)
      {
        Serial.print(down.charAt(i));
      }


    }
    else if (inputString == "center")
    {

      int len = center.length();

      for (int i = 0; i < len; i++)
      {
        Serial.print(center.charAt(i));
      }

    }
    else if (inputString == "left")
    {

      int len = left.length();

      for (int i = 0; i < len; i++)
      {
        Serial.print(left.charAt(i));
      }

    }
    else if (inputString == "right")
    {

      int len = right.length();

      for (int i = 0; i < len; i++)
      {
        Serial.print(right.charAt(i));
      }

    }

  }

}





void setAnchorsManual() {
  for (int i = 0; i < num_anchors; i++) {
    device_coordinates_t anchor;
    anchor.network_id = anchors[i];
    anchor.flag = 0x1;
    anchor.pos.x = anchors_x[i];
    anchor.pos.y = anchors_y[i];
    anchor.pos.z = heights[i];
  }
}
