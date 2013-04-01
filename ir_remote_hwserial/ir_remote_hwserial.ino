/*
 * ir_remote
 *
 * Description: Receive commands on RF link (XBee) to a BeagleBone running a 
 * a webserver to change the channels on Verizon set-top box
 *
 * 10-BYTE PACKET
 * 
 * OFFSET          VALUE          DESCRIPTION
 * 0               0x1A
 * 1               0xCF
 * 2               0xFC
 * 3               0x1D
 * 4               PKT TYPE       0xA1 = Verizon FIOS, 0xA2 = Yamaha A/V Receiver
 * 5               DATA BYTE      Hundreds Channel if PKT TYPE = 0xA1, Command Index if PKT TYPE = 0xA2
 * 6               DATA BYTE	  Tens Channel if PKT TYPE = 0xA1, 0xAA if PKT TYPE = 0xA2
 * 7               DATA BYTE      Ones Channel if PKT TYPE = 0xA1, 0x55 if PKT TYPE = 0xA2
 * 8               DATA BYTE      Command Count (i.e. number of commands transmitted, rollover occurs at 255)
 * 9               CHECKSUM       XOR of byte offsets 4 - 8
 *
 * Command Index for Yamaha A/V Receiver
 * 0x1 = Power Off
 * 0x2 = Power On
 * 0x3 = Volume Down
 * 0x4 = Volume Up
 * 0x5 = Change Input DVD
 * 0x6 = Change Input Cable
 * 0x7 = Change Input CD
 *
 * Version 1.1 January, 2013
 *
 * Taylor A. Green
 */

#include <IRremote.h>
#include "ircodes.h"

const int ledPin = 13;
const int RECV_PIN = 11; // raw decoded output from 38 kHz IR receiver
const int SEND_PIN = 3; // PWM output to IR emitter
int hb = 0; // heartbeat pulse for LED

byte tlm[10]; // telemetry packet, NOT USED
byte cmd[10]; // command packet

IRrecv irrecv(RECV_PIN);
IRsend irsend;
decode_results results;

void setup()
{
  // Initialize hardware
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, hb);
  irrecv.enableIRIn(); // Start the IR receiver
  Serial.begin(9600); // XBee Link to BeagleBone
  Serial.setTimeout(100); // set default timeout on serial port read to 100 ms

}

// Waits in an infinite loop for commands from BeagleBone XBee Link
// Once data is received, sync, packet length, and checksum are validated 
// Finally command is processed
void loop()
{
  int i;
  byte ser;
  char pkt[6];
  char sync[3];

  while (Serial.available() == 0); // wait for incoming command byte(s)
  ser = Serial.read();

  if ( ser == 0x1A ) // first sync byte found, get remaining 3
  {
    if ( Serial.readBytes(sync, 3) == 3) // make sure timeout didn't occur
    {
      if ( validSync((byte*)sync) == true ) // valid sync, get remaining packet
      {
        if ( Serial.readBytes(pkt, 6) == 6 ) // make sure timeout didn't occur
        {      
          if ( validChecksum((byte*)pkt, 6) == true ) // good checksum, act on command
            processCmd((byte*)pkt);
          else
          {
            Serial.println("NAK - BAD CHKSUM");
            //displayPacket((byte*)pkt, 6);
          }
        }
        else
          Serial.println("NAK - PKT TIMEOUT");
      }   
      else
      {
        Serial.println("NAK - BAD SYNC");
        //displayPacket((byte*)sync, 3);
      }
    }
    else
      Serial.println("NAK - SYNC TIMEOUT");
  }


  Serial.flush(); // flush buffer and start over


}

void processCmd(byte cmd[])
{
  int i;
  byte type;
  
  // flip LED
  hb ^= 1;
  digitalWrite(ledPin, hb);
  
  // ack command
  Serial.print("ACK - ");
  Serial.print(cmd[0], HEX); Serial.print(" ");
  Serial.print(cmd[1], HEX); Serial.print(" ");
  Serial.print(cmd[2], HEX); Serial.print(" ");
  Serial.print(cmd[3], HEX); Serial.print(" ");
  Serial.print(cmd[4], HEX); Serial.print(" ");
  Serial.print(cmd[5], HEX); Serial.println();
  
  type = cmd[0]; // command type is at index 0 (sync is stripped off)
  
  switch(type)
  {
    case 0xA1: // FIOS Remote
      irsend.sendRaw(verizonCH[int(cmd[1])], 35, 38); // length = 35, frequency = 38 kHz
      delay(200); // delay 200 ms
      irsend.sendRaw(verizonCH[int(cmd[2])], 35, 38); // length = 35, frequency = 38 kHz
      delay(200); // delay 200 ms
      irsend.sendRaw(verizonCH[int(cmd[3])], 35, 38); // length = 35, frequency = 38 kHz
      break;
    
    case 0xA2: // Yamaha A/V Receiver
      if ( cmd[1] == 0x1 ) // Power Off
      {
        irsend.sendNEC(AVOff, 32); // length = 32
        delay(1000); 
        irsend.sendNEC(AVOff, 32); // length = 32, send twice for reliability
      }
      else if ( cmd[1] == 0x2 ) // Power On
      {
        irsend.sendNEC(AVOn, 32); // length = 32
        delay(1000);
        irsend.sendNEC(AVOn, 32); // length = 32, send twice for reliability
      }
      else if ( cmd[1] == 0x3 ) // Volume Down
      {
        irsend.sendNEC(AVVolDown, 32); // length = 32
        delay(500);
        irsend.sendNEC(AVVolDown, 32); // must send Vol Up/Down commands four times in a row with 500 ms delay in between
        delay(500);
        irsend.sendNEC(AVVolDown, 32); // must send Vol Up/Down commands four times in a row with 500 ms delay in between
        delay(500);
        irsend.sendNEC(AVVolDown, 32); // must send Vol Up/Down commands four times in a row with 500 ms delay in between
      }
      else if ( cmd[1] == 0x4 ) // Volume Up
      {
        irsend.sendNEC(AVVolUp, 32); // length = 32
        delay(500);
        irsend.sendNEC(AVVolUp, 32); // must send Vol Up/Down commands four times in a row with 500 ms delay in between
        delay(500);
        irsend.sendNEC(AVVolUp, 32); // must send Vol Up/Down commands four times in a row with 500 ms delay in between
        delay(500);
        irsend.sendNEC(AVVolUp, 32); // must send Vol Up/Down commands four times in a row with 500 ms delay in between
      }
      else if ( cmd[1] == 0x5 ) // Change Input DVD
      {
        irsend.sendNEC(AVDvd, 32); // length = 32
        delay(1000);
        irsend.sendNEC(AVDvd, 32); // length = 32, send twice for reliability
      }
      else if ( cmd[1] == 0x6 ) // Change Input Cable
      {
        irsend.sendNEC(AVCbl, 32); // length = 32
        delay(1000);
        irsend.sendNEC(AVCbl, 32); // length = 32, send twice for reliability
      }
      else if ( cmd[1] == 0x7 ) // Change Input CD
      {
        irsend.sendNEC(AVCd, 32); // length = 32
        delay(1000);
        irsend.sendNEC(AVCd, 32); // length = 32, send twice for reliability
      }
      break;
     
    default: // unknown packet
      break; 
  }
}

boolean validSync(byte data[])
{
  if ( data[0] != 0xCF ) 
    return false;
  else if ( data[1] != 0xFC )
    return false;
  else if ( data[2] != 0x1D ) 
    return false;
  else
    return true; 
}

boolean validChecksum(byte data[], int length)
{
  int i;
  byte checksum = 0x00;

  for (i = 0; i < length; i++)
  {
    checksum ^= data[i];
  }

  if ( checksum == 0x00 ) // valid checksum would cancel itself out
    return true;
  else
    return false;
}

void displayPacket(byte data[], int length)
{
  int i;
  
  for (i = 0; i < length; i++)
  {
    Serial.print("Byte "); 
    Serial.print(i, DEC); 
    Serial.print(": "); 
    Serial.println(data[i], HEX);   
  } 
}

