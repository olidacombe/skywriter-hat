#include "skywriter.h"
#include <Arduino.h>

void _SkyWriter::begin(unsigned char pin_xfer, unsigned char pin_reset){
  this->xfer = pin_xfer;
  this->rst  = pin_reset;
  this->addr = SW_ADDR;
  this->init_ready = false;

  Wire.begin();
  
  pinMode(this->xfer, INPUT_PULLUP);
  pinMode(this->rst,  OUTPUT);
  digitalWrite(this->rst, LOW);
  delay(10);
  digitalWrite(this->rst, HIGH);
  delay(50);
}

void _SkyWriter::write_init_data() {
  if(!digitalRead(this->xfer)) {
    pinMode(this->xfer, OUTPUT);
    digitalWrite(this->xfer, LOW);

    Wire.beginTransmission(this->addr);
    Wire.write(this->init_data, this->init_data[0]);
    Wire.endTransmission(WIRE_STOP);

    digitalWrite(this->xfer, HIGH);
    pinMode(this->xfer, INPUT);
  }
}

unsigned char _SkyWriter::req_approach_detection() {
  unsigned char ret=0x00;
  if(!digitalRead(this->xfer)) {
    pinMode(this->xfer, OUTPUT);
    digitalWrite(this->xfer, LOW);
    Wire.beginTransmission(this->addr);
    Wire.write(this->req_app, this->req_app[0]);
    ret=Wire.endTransmission(WIRE_STOP);
    digitalWrite(this->xfer, HIGH);
    pinMode(this->xfer, INPUT);
  }
  return ret;
}

unsigned char _SkyWriter::wake() {
  return this->req_approach_detection();
}

/*
void _SkyWriter::wake() {
  if(!digitalRead(this->xfer)) {
    pinMode(this->xfer, OUTPUT);
    digitalWrite(this->xfer, LOW);

    Wire.requestFrom(this->addr, (unsigned char)32);

    digitalWrite(this->xfer, HIGH);
    pinMode(this->xfer, INPUT);
  }
}
*/

unsigned char _SkyWriter::poll(){
  if(!this->init_ready) {
    this->write_init_data();
  }
  if (!digitalRead(this->xfer)){
    pinMode(this->xfer, OUTPUT);
    digitalWrite(this->xfer, LOW);
    
    Wire.requestFrom(this->addr, (unsigned char)32);
    //Wire.requestFrom(this->addr, (unsigned char)32);
    
    unsigned char d_size,d_flags,d_seq,d_ident;
    
    if( Wire.available() >= 4 ){
      d_size  = Wire.read();
      this->header[0]=d_size;
      d_flags = Wire.read();
      this->header[1]=d_flags;
      d_seq   = Wire.read();
      this->header[2]=d_seq;
      d_ident = Wire.read();
      this->header[3]=d_ident;
    }
    else{
      // could this be the problem?  Not setting back to INPUT?
      digitalWrite(this->xfer, HIGH);
      pinMode(this->xfer, INPUT);
      return 0x01;
    }
    
    this->command_buffer[0] = '\0';
    
    unsigned char i = 0;
    while(Wire.available()){
      this->command_buffer[i] = Wire.read();
      i++;
    }
    this->command_buffer[i] = '\0';
  
    switch(d_ident){
      case 0x91: // 145
        this->handle_sensor_data(this->command_buffer);
        break;
      case 0x15: // 21
        if(command_buffer[0]==0xa2 && !command_buffer[2]) {
          this->init_ready=true;
        }
        // Status info - Unimplemented
        if(this->handle_status) this->handle_status(this->header, this->command_buffer);
        break;
      case 0x83: // 131
        // Firmware data - Unimplemented
        break;
    }
    
    
    digitalWrite(this->xfer, HIGH);
    pinMode(this->xfer, INPUT);

    return d_ident;
  }
  return 0x00;
}

void _SkyWriter::onTouch(    void (*function)(unsigned char) ){this->handle_touch    = function;}
void _SkyWriter::onAirwheel( void (*function)(int)           ){this->handle_airwheel = function;}
void _SkyWriter::onGesture(  void (*function)(unsigned char) ){this->handle_gesture  = function;}
void _SkyWriter::onXYZ(      void (*function)(unsigned int,unsigned int,unsigned int) ){this->handle_xyz  = function;}
void _SkyWriter::onStatus(  unsigned char (*function)(unsigned char *, unsigned char *) ){this->handle_status = function;}

void _SkyWriter::handle_sensor_data(unsigned char* data){
/*
  | HEADER | PAYLOAD
  |        |  DataOutputConfigMask 2 | TimeStamp 1 | SystemInfo 1 | Content |
  
  DataOutputConfigMask
  Bit 0 - DSPStatus
  Bit 1 - GestureInfo
  Bit 2 - TouchInfo
  Bit 3 - AirWheelInfo
  Bit 4 - XYZ Position
  Bit 5 - NoisePower
  Bit 6-7 - Reserved
  Bit 8-10 - ElectrodeConfiguration
  Bit 11 - CICData
  Bit 12 - SDData
  Bit 13-15 - Reserved
  
  SystemInfo
  Bit 0 - PositionValid, indicates xyz pos data is valid
  Bit 1 - AirWheelValid, indicates AirWheel is active and AirWheelInfo is vaid
  Bit 2 - RawDataValid, indicates CICData and SDData fields are valid
  Bit 3 - NoisePowerValid, indicates NoisePower field is valid
  Bit 4 - EnvironmentalNoise, indicates that environmental noise has been detected
  Bit 5 - Clipping, indicates that the ADCs are clipping
  Bit 6 - Reserved
  Bit 7 - DSPRunning, indicates the system is currently running
  DSPStatus - 2 bytes -
  GestureInfo - 4 bytes
  TouchInfo - 4 bytes
  AirWheelInfo - 2 bytes - first byte indicates rotation, ++ = clockwise, 32 = 1 rotation
  xyzPosition - 6 bytes - 1+2 = x, 3-4 = y, 5-6 = z
  NoisePower
  CICData
  SDData
*/  
      
      if( data[SW_PAYLOAD_HDR_CONFIGMASK] & SW_DATA_XYZ && data[SW_PAYLOAD_HDR_SYSINFO] & SW_SYS_POSITION ){
        // Valid XYZ position
        this->x = data[SW_PAYLOAD_X+1] << 8 | data[SW_PAYLOAD_X];
        this->y = data[SW_PAYLOAD_Y+1] << 8 | data[SW_PAYLOAD_Y];
        this->z = data[SW_PAYLOAD_Z+1] << 8 | data[SW_PAYLOAD_Z];
        
        if( handle_xyz != NULL ) handle_xyz(this->x,this->y,this->z);
      }
      
      if( data[SW_PAYLOAD_HDR_CONFIGMASK] & SW_DATA_GESTURE && data[SW_PAYLOAD_GESTURE] > 0){
        // Valid gesture
        this->last_gesture = data[SW_PAYLOAD_GESTURE];
        if( this->handle_gesture != NULL ) this->handle_gesture(this->last_gesture);
      }
      
      if ( data[SW_PAYLOAD_HDR_CONFIGMASK] & SW_DATA_TOUCH ){
        // Valid touch
        uint16_t touch_action = data[SW_PAYLOAD_TOUCH+1] << 8 | data[SW_PAYLOAD_TOUCH];
        uint16_t comp = 1 << 14;
        uint8_t x;
        for(x = 0; x < 16; x++){
          if( touch_action & comp ){
            this->last_touch = x;
            if( this->handle_touch != NULL ) this->handle_touch(this->last_touch);
            return;
          }
          comp = comp >> 1;
        }
      }
      
      if( data[SW_PAYLOAD_HDR_CONFIGMASK] & SW_DATA_AIRWHEEL && data[SW_PAYLOAD_HDR_SYSINFO] & SW_SYS_AIRWHEEL ){
        
        double delta = (data[SW_PAYLOAD_AIRWHEEL] - lastrotation) / 32.0;
        
        if( delta != 0 && delta > -0.5 and delta < 0.5 ){
          if( this->handle_airwheel != NULL ) this->handle_airwheel(delta * 360.0);  
        }
        
        this->rotation += delta * 360.0;
        this->rotation %= 360;
        
        this->lastrotation = data[SW_PAYLOAD_AIRWHEEL];
      }
}
