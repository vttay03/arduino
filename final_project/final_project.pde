/*
    Embedded Development Systems Laboratory
 Fall 2011 - Final Project
 Taylor Green
 Description : OBDII Interface to CAN-BUS; retrieves health and sensor information from
 automobile and sends it via RF link to Ground Station
 
 10/19/11 - Started new program structure with both 11-bit and 29-bit capabilities,
 Should include auto-detect capability to determine what type of CAN-Bus
 is implemented on the vehicle (standard vs. extended)
 
 10/30/11 - Close to final version - still need to reduce size of telemetry packet but
 auto-detection of CAN-bus addressing scheme is included, just need to make
 sure vehicle is turned on before plugging the 'CAN-duino' in
 
 11/03/11 - Updated to include PIDs supported in CAN-duino telemetry
 
 */

#include <MCP2515.h>
#include <SPI.h>

const int ledPin = 8; // led pin on CAN-Bus shield
int ledState = 1;

unsigned long MET;
byte tlm[60]; // ground station packet
CANMSG obdquery;
CANMSG obdresponse;

void setup()
{
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, ledState);

  obdquery.extendedAdrsValue = EXT_OBDII_REQUEST; // always assign both addresses; boolean within CANMSG
  obdquery.adrsValue = STD_OBDII_REQUEST;         // structure will decide which one to use

  Serial.begin(9600);

  // Setup sync bytes
  tlm[0] = 0xFE;
  tlm[1] = 0xFA;
  tlm[2] = 0x30;
  tlm[3] = 0xC8;

  tlm[4] = 0x01; // packet type
  tlm[5] = 0x30; // payload length (48 bytes)

  //Set CAN to 500K
  if (!MCP2515::initCAN(CAN_BAUD_500K))
    abort();

  //Detect CAN-Bus addressing scheme and update telemetry point
  switch (detectCANversion())
  {
  case 0:
    tlm[10] = 0x00;
    break;

  case 1:
    tlm[10] = 0x01;
    break;

  case 2:
    tlm[10] = 0x02;
    break;

  default:
    tlm[10] = 0x00;
    break;
  }

}

void loop()
{

  /****DEPLOYMENT CODE****/

  byte header; // should always be 0x47
  byte cmd[3]; // command contents
  int timeout_flag;
  int cnt;  

  while (Serial.available() == 0); // wait for incoming command byte(s)
  header = Serial.read();

  timeout_flag = 0;
  cnt = 0;

  switch (header) // test for correct header
  {
  case 0x47:
    while (Serial.available() != 3) // validate command length
    {
      cnt++;
      if (cnt == 5)
      {
        timeout_flag = 1;
        break; 
      }
      delay(10);  
    }

    if (timeout_flag == 0)
    {
      cmd[0] = Serial.read();
      cmd[1] = Serial.read();
      cmd[2] = Serial.read();
      if(!getECUdata(cmd))
        Serial.write(0xFF);

      // Send telemetry packet
      Serial.write(tlm, PKT_LEN);
    }
    else
    {
      Serial.flush(); 
    }
    break;

  default:
    Serial.flush();
    break; 

  }

}

boolean getECUdata( byte request[3] )
{

  // Break command bits down
  if ( bitRead(request[0], 7) ) // get Arduino MET
  {
    MET = millis();  
    ledState ^= 1;
    digitalWrite(ledPin, ledState); // flip LED to show heartbeart
    tlm[6] = (MET & 0xFF);
    tlm[7] = (MET & 0xFF00) >> 8;
    tlm[8] = (MET & 0xFF0000) >> 16;
    tlm[9] = (MET & 0xFF000000) >> 24;
  }

  if ( bitRead(request[0], 5) ) // request number of DTCs available
  {
    obdquery.dataLength = 8;
    obdquery.data[0] = 0x02;
    obdquery.data[1] = 0x01;
    obdquery.data[2] = NUM_ENGINE_DTC;
    obdquery.data[3] = 0;
    obdquery.data[4] = 0;
    obdquery.data[5] = 0;
    obdquery.data[6] = 0;
    obdquery.data[7] = 0;
    MCP2515::transmitCANMessage(obdquery, 100); // send request and timeout after 100 milliseconds
    if(!MCP2515::receiveCANMessage(&obdresponse, 100))
    {
      tlm[11] = 0x00;
    }
    else
    {
      tlm[11] = obdresponse.data[3];
    }

  }

  if ( bitRead(request[0], 4) ) // update engine coolant temperature
  {
    obdquery.dataLength = 8;
    obdquery.data[0] = 0x02;
    obdquery.data[1] = 0x01;
    obdquery.data[2] = ENGINE_COOLANT_TEMP;
    obdquery.data[3] = 0;
    obdquery.data[4] = 0;
    obdquery.data[5] = 0;
    obdquery.data[6] = 0;
    obdquery.data[7] = 0;
    MCP2515::transmitCANMessage(obdquery, 100); // send request and timeout after 100 milliseconds
    if(!MCP2515::receiveCANMessage(&obdresponse, 100))
    {
      tlm[40] = 0x00;
      tlm[41] = 0x00;
      tlm[42] = 0x00;
      tlm[43] = 0x00;  
    }
    else
    {
      tlm[40] = obdresponse.data[3];
      tlm[41] = 0x00;
      tlm[42] = 0x00;
      tlm[43] = 0x00;
    }

  }

  if ( bitRead(request[0], 3) ) // update relative throttle position
  {
    obdquery.dataLength = 8;
    obdquery.data[0] = 0x02;
    obdquery.data[1] = 0x01;
    obdquery.data[2] = THROTTLE;
    obdquery.data[3] = 0;
    obdquery.data[4] = 0;
    obdquery.data[5] = 0;
    obdquery.data[6] = 0;
    obdquery.data[7] = 0;
    MCP2515::transmitCANMessage(obdquery, 100); // send request and timeout after 100 milliseconds
    if(!MCP2515::receiveCANMessage(&obdresponse, 100))
    {
      tlm[36] = 0x00;
      tlm[37] = 0x00;
      tlm[38] = 0x00;
      tlm[39] = 0x00;  
    }
    else
    {
      tlm[36] = obdresponse.data[3];
      tlm[37] = 0x00;
      tlm[38] = 0x00;
      tlm[39] = 0x00;
    }

  }

  if ( bitRead(request[0], 2) ) // update vehicle run time
  {            
    obdquery.dataLength = 8;
    obdquery.data[0] = 0x02;
    obdquery.data[1] = 0x01;
    obdquery.data[2] = ENGINE_RUN_TIME;
    obdquery.data[3] = 0;
    obdquery.data[4] = 0;
    obdquery.data[5] = 0;
    obdquery.data[6] = 0;
    obdquery.data[7] = 0;
    MCP2515::transmitCANMessage(obdquery, 100); // send request and timeout after 100 milliseconds
    if(!MCP2515::receiveCANMessage(&obdresponse, 100))
    {
      tlm[32] = 0x00;
      tlm[33] = 0x00;
      tlm[34] = 0x00;
      tlm[35] = 0x00;  
    }
    else
    {
      tlm[32] = obdresponse.data[3];
      tlm[33] = obdresponse.data[4];
      tlm[34] = 0x00;
      tlm[35] = 0x00;
    }

  }

  if ( bitRead(request[0], 1) ) // update vehicle speed
  {
    obdquery.dataLength = 8;
    obdquery.data[0] = 0x02;
    obdquery.data[1] = 0x01;
    obdquery.data[2] = VEHICLE_SPEED;
    obdquery.data[3] = 0;
    obdquery.data[4] = 0;
    obdquery.data[5] = 0;
    obdquery.data[6] = 0;
    obdquery.data[7] = 0;
    MCP2515::transmitCANMessage(obdquery, 100); // send request and timeout after 100 milliseconds
    if(!MCP2515::receiveCANMessage(&obdresponse, 100))
    {
      tlm[28] = 0x00;
      tlm[29] = 0x00;
      tlm[30] = 0x00;
      tlm[31] = 0x00;  
    }
    else
    {
      tlm[28] = obdresponse.data[3];
      tlm[29] = 0x00;
      tlm[30] = 0x00;
      tlm[31] = 0x00;
    }

  }

  if ( bitRead(request[0], 0) ) // update engine RPM
  {            
    obdquery.dataLength = 8;
    obdquery.data[0] = 0x02;
    obdquery.data[1] = 0x01;
    obdquery.data[2] = ENGINE_RPM;
    obdquery.data[3] = 0;
    obdquery.data[4] = 0;
    obdquery.data[5] = 0;
    obdquery.data[6] = 0;
    obdquery.data[7] = 0;
    MCP2515::transmitCANMessage(obdquery, 100); // send request and timeout after 100 milliseconds
    if(!MCP2515::receiveCANMessage(&obdresponse, 100))
    {
      tlm[24] = 0x00;
      tlm[25] = 0x00;
      tlm[26] = 0x00;
      tlm[27] = 0x00;  
    }
    else
    {
      tlm[24] = obdresponse.data[3];
      tlm[25] = obdresponse.data[4];
      tlm[26] = 0x00;
      tlm[27] = 0x00;
    }

  }

  if ( bitRead(request[1], 2) ) // get Mode 01 data
  {
    obdquery.dataLength = 8;
    obdquery.data[0] = 0x02;
    obdquery.data[1] = 0x01;
    obdquery.data[2] = request[2];
    obdquery.data[3] = 0;
    obdquery.data[4] = 0;
    obdquery.data[5] = 0;
    obdquery.data[6] = 0;
    obdquery.data[7] = 0;
    MCP2515::transmitCANMessage(obdquery, 100); // send request and timeout after 100 milliseconds
    if(!MCP2515::receiveCANMessage(&obdresponse, 100))
    {
      tlm[44] = 0x00;
      tlm[45] = 0x00;
      tlm[46] = 0x00;
      tlm[47] = 0x00;  
    }
    else
    {
      tlm[44] = obdresponse.data[3];
      tlm[45] = obdresponse.data[4];
      tlm[46] = obdresponse.data[5];
      tlm[47] = obdresponse.data[6];
    }

  }

  if ( bitRead(request[1], 1) ) // request engine DTCs
  {
    obdquery.dataLength = 8;
    obdquery.data[0] = 0x01;
    obdquery.data[1] = 0x03;
    obdquery.data[2] = 0;
    obdquery.data[3] = 0;
    obdquery.data[4] = 0;
    obdquery.data[5] = 0;
    obdquery.data[6] = 0;
    obdquery.data[7] = 0;
    MCP2515::transmitCANMessage(obdquery, 100); // send request and timeout after 100 milliseconds
    if(!MCP2515::receiveCANMessage(&obdresponse, 100))
    {
      tlm[48] = 0x00;
      tlm[49] = 0x00;
      tlm[50] = 0x00;
      tlm[51] = 0x00;  
      tlm[52] = 0x00;
      tlm[53] = 0x00;  
    }
    else
    {
      tlm[48] = obdresponse.data[2];
      tlm[49] = obdresponse.data[3];
      tlm[50] = obdresponse.data[4];
      tlm[51] = obdresponse.data[5];  
      tlm[52] = obdresponse.data[6];
      tlm[53] = obdresponse.data[7];  
      /* NEED TO CHECK WHETHER BUFFER SIZE IN STRUCTURE IS SUFFICIENT ENOUGH FOR ANTICIPATED RESPONSE */
    }

  }

  if ( bitRead(request[1], 0) ) // clear engine DTCs
  {
    obdquery.dataLength = 8;
    obdquery.data[0] = 0x01;
    obdquery.data[1] = 0x04;
    obdquery.data[2] = 0;
    obdquery.data[3] = 0;
    obdquery.data[4] = 0;
    obdquery.data[5] = 0;
    obdquery.data[6] = 0;
    obdquery.data[7] = 0;
    MCP2515::transmitCANMessage(obdquery, 100); // send request and timeout after 100 milliseconds
    //MCP2515::receiveCANMessage(&obdresponse, 100);

    // No reply needed for this mode
  }

  compute_checksum(tlm, PKT_LEN); // compute checksum bytes

  return true;

}

void compute_checksum(byte data[], int length)
{
  int i;
  byte checksum0 = 0x00;
  byte checksum1 = 0x00;

  for (i = 4; i < (length - 2); i+=2) // skip sync bytes
  {
    checksum0 ^= data[i];
    checksum1 ^= data[i+1];
  }

  data[length - 2] = checksum0;
  data[length - 1] = checksum1;

}

int detectCANversion()
{

  /* Test for 11-bit Addressing Scheme */
  if ( !MCP2515::setCANConfigMode() )
    abort();

  if ( !MCP2515::setCANFilters(false) )
    abort();

  if ( !MCP2515::setCANNormalMode() )
    abort();

  obdquery.isExtendedAdrs = false; 
  obdquery.rtr = false;

  obdquery.dataLength = 8;
  obdquery.data[0] = 0x02;
  obdquery.data[1] = 0x01;
  obdquery.data[2] = 0;
  obdquery.data[3] = 0;
  obdquery.data[4] = 0;
  obdquery.data[5] = 0;
  obdquery.data[6] = 0;
  obdquery.data[7] = 0;
  MCP2515::transmitCANMessage(obdquery, 100); // send request and timeout after 100 milliseconds
  if(MCP2515::receiveCANMessage(&obdresponse, 100))
  {
    /* Assign telemetry for PIDs supported 0x01 - 0x1F */
    tlm[12] = obdresponse.data[3];
    tlm[13] = obdresponse.data[4];
    tlm[14] = obdresponse.data[5];
    tlm[15] = obdresponse.data[6];
    
    /* Assign telemetry for PIDs supported 0x21 - 0x3F */
    obdquery.data[2] = 0x20;
    MCP2515::transmitCANMessage(obdquery, 100); // send request and timeout after 100 milliseconds
    if(MCP2515::receiveCANMessage(&obdresponse, 100))
    {
      tlm[16] = obdresponse.data[3];
      tlm[17] = obdresponse.data[4];
      tlm[18] = obdresponse.data[5];
      tlm[19] = obdresponse.data[6];
    }
    else
    {
      tlm[16] = 0x00;
      tlm[17] = 0x00;
      tlm[18] = 0x00;
      tlm[19] = 0x00;
    }
    

    /* Assign telemetry for PIDs supported 0x41 - 0x5F */
    obdquery.data[2] = 0x40;
    MCP2515::transmitCANMessage(obdquery, 100); // send request and timeout after 100 milliseconds
    if(MCP2515::receiveCANMessage(&obdresponse, 100))
    {
      tlm[20] = obdresponse.data[3];
      tlm[21] = obdresponse.data[4];
      tlm[22] = obdresponse.data[5];
      tlm[23] = obdresponse.data[6];
    }
    else
    {
      tlm[20] = 0x00;
      tlm[21] = 0x00;
      tlm[22] = 0x00;
      tlm[23] = 0x00;
    }

    return 1; // '1' indicates vehicle uses standard addressing scheme
  }

  /* Test for 29-bit Addressing Scheme */
  if ( !MCP2515::setCANConfigMode() )
    abort();

  if ( !MCP2515::setCANFilters(true) )
    abort();

  if ( !MCP2515::setCANNormalMode() )
    abort();

  obdquery.isExtendedAdrs = true;
  
  MCP2515::transmitCANMessage(obdquery, 100); // send request and timeout after 100 milliseconds
  if(MCP2515::receiveCANMessage(&obdresponse, 100))
  {
    /* Assign telemetry for PIDs supported 0x01 - 0x1F */
    tlm[12] = obdresponse.data[3];
    tlm[13] = obdresponse.data[4];
    tlm[14] = obdresponse.data[5];
    tlm[15] = obdresponse.data[6];
    
    /* Assign telemetry for PIDs supported 0x21 - 0x3F */
    obdquery.data[2] = 0x20;
    MCP2515::transmitCANMessage(obdquery, 100); // send request and timeout after 100 milliseconds
    if(MCP2515::receiveCANMessage(&obdresponse, 100))
    {
      tlm[16] = obdresponse.data[3];
      tlm[17] = obdresponse.data[4];
      tlm[18] = obdresponse.data[5];
      tlm[19] = obdresponse.data[6];
    }
    else
    {
      tlm[16] = 0x00;
      tlm[17] = 0x00;
      tlm[18] = 0x00;
      tlm[19] = 0x00;
    }
    

    /* Assign telemetry for PIDs supported 0x41 - 0x5F */
    obdquery.data[2] = 0x40;
    MCP2515::transmitCANMessage(obdquery, 100); // send request and timeout after 100 milliseconds
    if(MCP2515::receiveCANMessage(&obdresponse, 100))
    {
      tlm[20] = obdresponse.data[3];
      tlm[21] = obdresponse.data[4];
      tlm[22] = obdresponse.data[5];
      tlm[23] = obdresponse.data[6];
    }
    else
    {
      tlm[20] = 0x00;
      tlm[21] = 0x00;
      tlm[22] = 0x00;
      tlm[23] = 0x00;
    }
    
    return 2; // '2' indicates vehicle uses extended addressing scheme
  }

  /* Assign 0x00 to PIDs supported telemetry since valid response was not received for either addressing scheme */
  tlm[12] = 0x00;   
  tlm[13] = 0x00;   
  tlm[14] = 0x00;   
  tlm[15] = 0x00;   
  tlm[16] = 0x00;   
  tlm[17] = 0x00;   
  tlm[18] = 0x00;   
  tlm[19] = 0x00;   
  tlm[20] = 0x00;   
  tlm[21] = 0x00;   
  tlm[22] = 0x00;   
  tlm[23] = 0x00;

  return 0;  
}

void abort()
{
  while(1)
  {
    Serial.write(0x55);
    Serial.write(0xAA);
    delay(5000);
  }  
}

