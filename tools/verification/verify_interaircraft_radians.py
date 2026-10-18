"""Exercise the actual interaircraft implementation with 0.001-radian wire data."""
from pathlib import Path
import re
import shutil

import verify_si_units as common

ROOT=common.ROOT
OUT=ROOT/'.cache/interaircraft-radians'

HARNESS=r'''
#include "Arduino.h"
#include "interaircraft_comm/interaircraft_comm.h"
#include "config/aircraft_config.h"
#include <limits>
#include <cstdlib>
float att[3]={},ang_vel[3]={};
int identity=2;
int aircraftId(){return identity;}
bool aircraftIsLeft(){return identity%2==0;}
bool aircraftIsRight(){return identity%2==1;}
unsigned checks=0;
void check(bool ok,const char* what){++checks;if(!ok){std::fprintf(stderr,"FAIL %s (%u)\n",what,checks);std::exit(1);}}
int16_t word(const std::vector<uint8_t>& b,unsigned offset){return static_cast<int16_t>(b[offset]|(b[offset+1]<<8));}
void put(std::vector<uint8_t>& b,unsigned offset,int16_t n){b[offset]=n&255;b[offset+1]=(n>>8)&255;}
void checksum(std::vector<uint8_t>& b){uint8_t sum=0;for(size_t i=0;i<b.size()-1;i++)sum+=b[i];b.back()=sum;}
void frame(const std::vector<uint8_t>& b,size_t size,uint8_t kind){
 check(b.size()==size,"frame size");check(b[0]==0x55&&b[1]==kind,"header");
 uint8_t sum=0;for(size_t i=0;i<b.size()-1;i++)sum+=b[i];check(sum==b.back(),"checksum");}
float* attitudes[]={att_B,att_C,att_D,att_E,att_F,att_G};
float* rates[]={ang_vel_B,ang_vel_C,ang_vel_D,ang_vel_E,ang_vel_F,ang_vel_G};
int quantize(float x){return static_cast<int>(std::lround(static_cast<double>(x)*1000));}
int main(){
 commandLinkDebugSetting()=0;sensorLinkDebugSetting()=0;
 Stream tx;AircraftCommandLink sender;sender.configure(&tx,nullptr);
 SlaveCommand c;c.mode=STABILIZE_MODE;c.masterContribution={.25f,-.125f,.125f,-.5f,.5f};
 // Every int16 value, every command slot: encoding and decoding must recover the exact code.
 for(unsigned slot=0;slot<3;slot++)for(int raw=-32768;raw<=32767;raw++){
  c.target.pitch=raw/1000.f;check(sender.setCommandLink(slot,c),"command accepted");
  ticks+=10000;tx.output.clear();sender.sendPreparedChildCommands();frame(tx.output,40,0x60);
  check(word(tx.output,33+2*slot)==raw,"command wire is milliradians");
  const int expected[]={1024,-512,-2048,2048,512};
  for(unsigned i=0;i<5;i++)check(word(tx.output,3+slot*10+i*2)==expected[i],"control scale/layout retained");
  Stream rx;rx.input=tx.output;AircraftCommandLink receiver;receiver.configure(nullptr,&rx,slot+2);
  receiver.receiveCommandData();const auto& decoded=receiver.getCommandLink();
  check(decoded.target.pitch==raw/1000.f,"command decode radians");
  check(decoded.masterContribution.elevator==.125f,"logical output decode retained");
 }
 for(auto p:std::vector<std::pair<float,int>>{{.00049f,0},{.0005f,1},{.00051f,1},
    {-.00049f,0},{-.0005f,-1},{-.00051f,-1},{.0015f,2},{-.0015f,-2}}){
  c.target.pitch=p.first;check(sender.setCommandLink(0,c),"fractional pitch accepted");
  ticks+=10000;tx.output.clear();sender.sendPreparedChildCommands();check(word(tx.output,33)==p.second,"symmetric nearest rounding");
 }
 c.target.pitch=1;check(sender.setCommandLink(0,c),"valid cache seed");
 ticks+=10000;tx.output.clear();sender.sendPreparedChildCommands();auto saved=tx.output;
 for(float x:{32.768f,-32.769f,std::numeric_limits<float>::quiet_NaN(),
              std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()}){
  c.target.pitch=x;check(!sender.setCommandLink(0,c),"invalid command rejected");
  ticks+=10000;tx.output.clear();sender.sendPreparedChildCommands();check(tx.output==saved,"rejection keeps cache");
 }
 check(!sender.setCommandLink(3,c),"invalid slot rejected");
 // Raw sensor decode: all int16 codes, both sides, every transmitted physical field.
 for(int raw=-32768;raw<=32767;raw++)for(unsigned side=0;side<2;side++){
  std::vector<uint8_t> bytes(34);bytes[0]=0x55;bytes[1]=0x71;bytes[2]=side;
  for(unsigned offset=3;offset<33;offset+=2)put(bytes,offset,raw);checksum(bytes);
  Stream rx;rx.input=bytes;AircraftSensorLink receiver;receiver.configure(nullptr,&rx,side);
  receiver.receiveSensorData();check(receiver.hasReceivedData(),"sensor frame accepted");
  for(unsigned slot=0;slot<3;slot++){
   unsigned node=slot*2+(side?0:1);
   for(unsigned axis=0;axis<3;axis++)check(attitudes[node][axis]==raw/1000.f,"sensor attitude decode radians");
   check(rates[node][0]==raw/1000.f,"sensor rate decode radians per second");
   check(rates[node][1]==0&&rates[node][2]==0,"untransmitted axes unchanged");
  }
 }
 // Actual sensor sender: local and downstream slots, both sides, all three chain levels.
 for(identity=2;identity<=7;identity++){
  att[0]=.1234f;att[1]=-.4567f;att[2]=1.2344f;ang_vel[0]=-2.3456f;
  for(unsigned i=0;i<6;i++){attitudes[i][0]=.25f*(i+1);attitudes[i][1]=-.125f*(i+1);
   attitudes[i][2]=.0625f*(i+1);rates[i][0]=.5f*(i+1);}
  Stream up,down;AircraftSensorLink sender;sender.configure(&up,&down,aircraftIsLeft());
  ticks+=5000;sender.forwardReceivedChildSensors();frame(up.output,34,0x71);
  const unsigned local=(identity-2)/2;
  check(up.output[2]==(aircraftIsLeft()?1:0),"sensor side flag");
  for(unsigned slot=0;slot<3;slot++){
   float roll=0,pitch=0,yaw=0,rate=0,relative=0;unsigned node=slot*2+(aircraftIsLeft()?0:1);
   if(slot==local){roll=att[0];pitch=att[1];yaw=att[2];rate=ang_vel[0];}
   if(slot>local){roll=attitudes[node][0];pitch=attitudes[node][1];yaw=attitudes[node][2];rate=rates[node][0];
    relative=roll-(slot-1==local?att[0]:attitudes[node-2][0]);}
   const float values[]={relative,rate,roll,pitch,yaw};
   for(unsigned field=0;field<5;field++)check(word(up.output,3+field*6+slot*2)==quantize(values[field]),"sensor slot encoding");
  }
 }
 identity=2;att[0]=100;att[1]=-100;att[2]=std::numeric_limits<float>::quiet_NaN();
 ang_vel[0]=std::numeric_limits<float>::infinity();
 Stream up,down;AircraftSensorLink saturated;saturated.configure(&up,&down,true);ticks+=5000;saturated.forwardReceivedChildSensors();
 check(word(up.output,15)==32767&&word(up.output,21)==-32768,"sensor saturation");
 check(word(up.output,9)==0&&word(up.output,27)==0,"nonfinite state encodes zero");
 // Real receive/forward path must copy the complete command frame without quantizing again.
 Stream parent,child;parent.input=saved;AircraftCommandLink relay;relay.configure(&child,&parent,2);
 relay.receiveCommandData();ticks+=10000;relay.forwardReceivedChildCommands();check(child.output==saved,"command cascade byte preservation");
 // Bad checksums and partial frames must preserve the last parsed target.
 auto damaged=saved;damaged.back()^=1;parent.input.insert(parent.input.end(),damaged.begin(),damaged.end());relay.receiveCommandData();
 check(relay.getCommandLink().target.pitch==1.f,"bad checksum preserves command");
 parent.input.insert(parent.input.end(),saved.begin(),saved.begin()+15);relay.receiveCommandData();
 check(relay.getCommandLink().target.pitch==1.f,"partial frame preserves command");
 // Capture actual printReceivedData via the public receive methods.
 commandLinkDebugSetting()=1;Serial.printed.clear();Stream printRx;printRx.input=saved;
 AircraftCommandLink commandPrinter;commandPrinter.configure(nullptr,&printRx,2);commandPrinter.receiveCommandData();
 check(Serial.printed.find("pitchAngles (deg): 57.296")!=std::string::npos,"command print degrees");
 sensorLinkDebugSetting()=1;Serial.printed.clear();std::vector<uint8_t> sensor(34);
 sensor[0]=0x55;sensor[1]=0x71;sensor[2]=1;
 const int16_t fields[]={1000,-500,1571,-1000,0};
 for(unsigned field=0;field<5;field++)for(unsigned slot=0;slot<3;slot++)put(sensor,3+field*6+slot*2,fields[field]);checksum(sensor);
 Stream sensorRx;sensorRx.input=sensor;AircraftSensorLink sensorPrinter;sensorPrinter.configure(nullptr,&sensorRx,true);sensorPrinter.receiveSensorData();
 check(Serial.printed.find("relativeAngles (deg): 57.296")!=std::string::npos,"relative print degrees");
 check(Serial.printed.find("rollRates (deg/s): -28.648")!=std::string::npos,"rate print degrees per second");
 check(Serial.printed.find("rollAngles (deg): 90.012")!=std::string::npos,"roll print degrees");
 check(Serial.printed.find("pitchAngles (deg): -57.296")!=std::string::npos,"pitch print degrees");
 check(Serial.printed.find("yawAngles (deg): 0.000")!=std::string::npos,"yaw print degrees");
 std::printf("PASS: %u assertions; exhaustive command/sensor int16 ranges, slots, rounding, limits, cascade and degree prints\n",checks);
}
'''


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    stubs=OUT/'stubs'
    arduino=common.ARDUINO.replace(
        'template<class...T> void print(T...){} template<class...T> void println(T...){}\n template<class...T> void printf(T...){}',
        '''std::string printed;
 void print(const char* x){printed+=x;} void println(const char* x){printed+=x;printed+='\\n';}
 template<class...T> void printf(const char* format,T... args){char buf[512];std::snprintf(buf,sizeof(buf),format,args...);printed+=buf;}''')
    for name,content in {**common.STUBS,'Arduino.h':arduino}.items():
        p=stubs/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(content,encoding='utf-8')
    cpp=OUT/'protocol.cpp';cpp.write_text(HARNESS,encoding='utf-8')
    results=[]
    for label,flags in [('normal',[]),('fast-math',['-ffast-math'])]:
        exe=OUT/f'protocol-{label}.exe'
        common.run([shutil.which('g++'),'-std=c++17','-O2','-DCOFLY_AIRCRAFT_COUNT=7',
                    '-I'+str(stubs),'-I'+str(ROOT/'src'),*flags,str(cpp),
                    str(ROOT/'src/interaircraft_comm/interaircraft_comm.cpp'),'-o',str(exe)])
        results.append(label+': '+common.run([str(exe)]).strip())
    print('\n'.join(results));(OUT/'results.txt').write_text('\n'.join(results)+'\n')


if __name__=='__main__':main()
