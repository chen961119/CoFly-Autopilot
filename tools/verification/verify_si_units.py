"""Host regression for the SI boundary migration (Python 3 + g++).

Compiles actual controller, command, filter and link sources twice:
the supplied Git baseline and the working tree. Hardware-only sensor functions
are extracted verbatim and exercised against deterministic register/serial data.
Build products stay in .cache/si-verification. No board is flashed.
"""
from pathlib import Path
import argparse
import math
import shutil
import subprocess
import re

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / '.cache/si-verification'

ARDUINO = r'''
#pragma once
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <vector>
#include <string>
#include <cstdio>
#define PI 3.14159265358979323846f
#define F(x) x
#define LOW 0
#define HIGH 1
#define HEX 16
template<class T, class U, class V> auto constrain(T x,U lo,V hi)->decltype(x+lo+hi) {
  return x<lo?lo:x>hi?hi:x;
}
using String=std::string;
inline uint32_t ticks=0;
inline uint32_t micros(){return ticks;}
inline uint32_t millis(){return ticks/1000;}
inline void digitalWrite(int,int){}
inline void delay(int){}
inline void delayMicroseconds(int){}
struct Stream {
 std::vector<uint8_t> input,output; size_t pos=0;
 void begin(int){} int available(){return int(input.size()-pos);}
 int read(){return available()?input[pos++]:-1;} int peek(){return available()?input[pos]:-1;}
 size_t readBytes(uint8_t* p,size_t n){size_t i=0;while(i<n&&available())p[i++]=read();return i;}
 size_t write(const uint8_t* p,size_t n){output.insert(output.end(),p,p+n);return n;}
 size_t write(uint8_t b){output.push_back(b);return 1;}
 template<class...T> void print(T...){} template<class...T> void println(T...){}
 template<class...T> void printf(T...){}
 int availableForWrite(){return 8192;} explicit operator bool()const{return true;}
};
inline Stream Serial,Serial1,Serial2,Serial3,Serial5,Serial6,Serial7,Serial8;
'''

STUBS = {
 'Arduino.h': ARDUINO,
 'SPI.h': r'''#pragma once
#include "Arduino.h"
struct SPISettings {};
struct Spi {std::vector<uint8_t> data;size_t pos=0;
 void beginTransaction(SPISettings){} void endTransaction(){}
 uint8_t transfer(int){return pos<data.size()?data[pos++]:0;}};
inline Spi SPI;
''',
 'radio_comm/radio_comm.h': '''#pragma once
inline float radio_channel1_normalized=0,radio_channel2_normalized=0,radio_channel3_normalized=0,
 radio_channel4_normalized=0,radio_channel5_normalized=0,radio_channel6_normalized=0,
 radio_channel7_normalized=0,radio_channel8_normalized=0;
''',
 'interface/human_interface.h': '#pragma once\ninline void displayFlightModeIndicators(FlightMode){}\n',
 'math/math_utils.h': '''#pragma once
#include <cmath>
inline float invSqrt(float x){return 1.0f/std::sqrt(x);}
inline float keeppositive(float x){return x>0?x:0;}
''',
}

SOURCES = [
 'parameters/control_parameters.cpp', 'control/control_output.cpp',
 'control/control_modes.cpp', 'control/command_generation.cpp',
 'control/roll_pid_controller.cpp', 'control/pitch_pid_controller.cpp',
 'control/yaw_rate_pid_controller.cpp', 'control/roll_damping_controller.cpp',
 'control/pitch_indi_controller.cpp', 'control/configuration_controller.cpp',
 'sensors/filters.cpp', 'interaircraft_comm/interaircraft_comm.cpp',
 'telemetry/telemetry.cpp',
 'control/controller_instances.cpp','control/master_controller.cpp','control/slave_controller.cpp',
 'control/manual_controller.cpp','control/throttle_controller.cpp','allocation/master_mixer.cpp',
]

HARNESS = r'''
#include "Arduino.h"
#include "config/aircraft_config.h"
#include "sensors/sensor_processing.h"
#include "sensors/imu_internal.h"
#include "control/control_output.h"
#include "control/command_generation.h"
#include "control/control_modes.h"
#include "control/roll_pid_controller.h"
#include "control/pitch_pid_controller.h"
#include "control/pitch_indi_controller.h"
#include "control/yaw_rate_pid_controller.h"
#include "control/roll_damping_controller.h"
#include "control/configuration_controller.h"
#include "interaircraft_comm/interaircraft_comm.h"
#include "parameters/control_parameters.h"
#include "telemetry/telemetry.h"
#include "control/control_pipeline.h"
#include "control/master_controller.h"
#include "control/slave_controller.h"
#include "allocation/master_mixer.h"
#ifndef LEGACY
#include "math/units.h"
static_assert(Units::toRadians(180.f)>3.1415925f && Units::toRadians(180.f)<3.141593f,"180 degrees must be pi radians");
static_assert(Units::acceleration(1.f)>9.80664f && Units::acceleration(1.f)<9.80666f,"standard gravity must be SI");
#endif
using namespace ControlParameters;
int identity=1;int aircraftId(){return identity;}
bool aircraftIsLeft(){return identity%2==0;}
bool aircraftIsRight(){return identity>1&&identity%2==1;}
bool aircraftConfigurationValid(){return true;}
bool validAircraftConfiguration(int i){return i>=1&&i<=aircraftCount();}
namespace CommandLinks {AircraftCommandLink *left=nullptr,*right=nullptr;}
void receiveCommandData(){}
ControlCycleOutput commands;
float dt=0.002f;unsigned long current_time=0;
float AccX_6050,AccY_6050,AccZ_6050,GyroX_6050,GyroY_6050,GyroZ_6050;
float Acc_X_EXT,Acc_Y_EXT,Acc_Z_EXT,Gyro_X_EXT,Gyro_Y_EXT,Gyro_Z_EXT;
float ang_vel[3],ang_vel_filtered[3],ang_acc[3],acc[3];
void initializeAngularAccelerationFilters(){}
float in(float x){
#ifdef LEGACY
return x;
#else
return Units::toRadians(x);
#endif
}
float out(float x){
#ifdef LEGACY
return x;
#else
return Units::toDegrees(x);
#endif
}
void emit(float x){std::printf("%.9g ",x);}
void measurements(int step){
 for(int i=0;i<3;i++){
  att[i]=in(8*std::sin(step*.013f+i*.2f));
  ang_vel[i]=in(20*std::cos(step*.017f+i*.3f));
  ang_acc[i]=in(140*std::sin(step*.009f+i));
 }
 updateAngularVelocityFilters();
 float* as[]={att_B,att_C,att_D,att_E,att_F,att_G};
 float* rs[]={ang_vel_B,ang_vel_C,ang_vel_D,ang_vel_E,ang_vel_F,ang_vel_G};
 for(int j=0;j<6;j++)for(int i=0;i<3;i++){
  as[j][i]=in(out(att[i])+(j+1)*1.25f);rs[j][i]=in(8*std::cos(step*.011f+j));
 }
}
int main(){
 initializeControlFilters();
 RollPidController roll; PitchPidController pitch; PitchIndiController indi;
 YawRatePidController yaw; RollDampingController damping;ConfigurationController config;
 indi.reset(3.0f);
 for(int step=0;step<900;step++){
  ticks+=2000;measurements(step);currentFlightMode=step%160<20?MANUAL_MODE:STABILIZE_MODE;
  const bool enabled=isStabilizedMode(currentFlightMode),integral=(step/90)%2;
  roll.prepare(integral,step%160==20);pitch.prepare(integral,step%160==20);
  yaw.prepare(integral,step%160==20);config.prepare(integral);
  roll.update(12*std::sin(step*.01f),enabled);
  pitch.update(6*std::sin(step*.012f),enabled);
  indi.update(6*std::sin(step*.012f),enabled);
  yaw.update(15*std::sin(step*.02f),7.0f,0.0f,enabled);damping.update();
  float cfg[6];for(int i=0;i<6;i++)cfg[i]=in(i%2?-10:10);config.update(cfg);
  emit(rollControlOutput.pidOutput);emit(rollControlOutput.filteredPidOutput);
  emit(pitchPidControlOutput.pidOutput);emit(yawRateControlOutput.pidOutput);
  emit(rollDampingControlOutput.dampingCommand);emit(pitchIndiControlOutput.indiOutput);
#ifdef LEGACY
  emit(pitchIndiControlOutput.q_dot_ref_deg_s2);emit(pitchIndiControlOutput.q_ref_deg_s);
  emit(pitchIndiControlOutput.q_f_deg_s);emit(pitchIndiControlOutput.q_dot_meas_deg_s2);
  emit(pitchIndiControlOutput.delta_e_cmd_deg);emit(pitchIndiControlOutput.delta_e_hat_deg);
  emit(rollControlOutput.equivalentRollAngleDeg);
  for(float x:configurationControlOutput.angleErrorsDeg)emit(x);
  for(float x:configurationControlOutput.measuredAnglesDeg)emit(x);
  for(float x:configurationControlOutput.measuredRatesDegPerSec)emit(x);
#else
  emit(out(pitchIndiControlOutput.q_dot_ref));emit(out(pitchIndiControlOutput.q_ref));
  emit(out(pitchIndiControlOutput.q_f));emit(out(pitchIndiControlOutput.q_dot_meas));
  emit(out(pitchIndiControlOutput.delta_e_cmd));emit(out(pitchIndiControlOutput.delta_e_hat));
  emit(out(rollControlOutput.equivalentRollAngle));
  for(float x:configurationControlOutput.angleErrors)emit(out(x));
  for(float x:configurationControlOutput.measuredAngles)emit(out(x));
  for(float x:configurationControlOutput.measuredRates)emit(out(x));
#endif
  for(float x:configurationControlOutput.pidOutputs)emit(x);
  targets.roll=enabled?in(12):.3f;targets.pitch=enabled?in(6):-.2f;
  targets.yawRate=enabled?in(15):.1f;
  emit(targetRollDegrees());emit(targetPitchDegrees());emit(targetYawRateDegreesPerSec());
  mixerStrategyCommand=(step%3-1)*.8f;generateSlaveCommand();
  for(auto x:slaveTargets)emit(out(x.pitch));
  activePitchController=step%2?PitchController::PID:PitchController::INDI;
  runMasterController();mixMasterCommands();generateSlaveCommand();
  const ActuatorCommand* outputs[]={&commands.local,&commands.slaves[0],&commands.slaves[1],
    &commands.slaves[2],&commands.slaves[3],&commands.slaves[4],&commands.slaves[5]};
  for(auto p:outputs){emit(p->aileron1);emit(p->aileron2);emit(p->elevator);emit(p->throttle);emit(p->rudder);}
  emit(manualControlOutput.rollCommand);emit(manualControlOutput.pitchCommand);
  emit(manualControlOutput.yawCommand);emit(manualControlOutput.throttleCommand);
  identity=2;targets.pitch=in(6);runSlaveController();
  emit(pitchPidControlOutput.pidOutput);emit(pitchIndiControlOutput.indiOutput);identity=1;
  std::puts("");
 }
 // RC target generation: manual/stabilized, CH7 thresholds, clipping.
 for(int mode=0;mode<2;mode++)for(float ch7:{-.5f,-.2f,0.f,.2f,.5f}){
  radio_channel1_normalized=.4f;radio_channel2_normalized=-.3f;
  radio_channel4_normalized=.2f;radio_channel5_normalized=mode?-.5f:.5f;
  radio_channel7_normalized=ch7;prepareControlMode();
  emit(mode?out(targets.roll):targets.roll);emit(mode?out(targets.pitch):targets.pitch);
  emit(mode?out(targets.yawRate):targets.yawRate);
  for(float x:targets.configuration)emit(out(x));std::puts("");
 }
 // SI attitude from the real quaternion estimator; compare after converting back.
 quat[0]=1;quat[1]=quat[2]=quat[3]=0;
 for(int i=0;i<200;i++){
  Madgwick6DOF(2.f,4.f,-3.f,.03f,.02f,1.0f,.002f);
  for(float x:quat)emit(x);for(float x:att)emit(out(x));std::puts("");
 }
 // Real command frame encoder/decoder; exercise both signs and every slot.
 identity=2;Stream tx;AircraftCommandLink sender;sender.configure(&tx,nullptr);
 for(unsigned i=0;i<3;i++){SlaveCommand c;c.mode=STABILIZE_MODE;c.target.pitch=in((int(i)-1)*12.35f);
  c.masterContribution={.25f,-.2f,.1f,-.4f,.3f};if(!sender.setCommandLink(i,c))return 2;}
 ticks+=10000;sender.sendPreparedChildCommands();
 for(unsigned slot=0;slot<3;slot++){
  Stream rx;rx.input=tx.output;AircraftCommandLink receiver;receiver.configure(nullptr,&rx,slot+2);
  receiver.receiveCommandData();const auto c=receiver.getCommandLink();
  emit(out(c.target.pitch));emit(c.masterContribution.elevator);std::puts("");
 }
 // Real sensor frame encode/decode, all left/right slots and chain levels.
 for(int id=2;id<=7;id++){
  identity=id;measurements(21);Stream up,down;AircraftSensorLink sensor;
  sensor.configure(&up,&down,aircraftIsLeft());ticks+=10000;sensor.forwardReceivedChildSensors();
  Stream rx;rx.input=up.output;AircraftSensorLink receiver;receiver.configure(nullptr,&rx,aircraftIsLeft());receiver.receiveSensorData();
  const float* as[]={att_B,att_C,att_D,att_E,att_F,att_G};
  const float* rs[]={ang_vel_B,ang_vel_C,ang_vel_D,ang_vel_E,ang_vel_F,ang_vel_G};
  for(int i=0;i<6;i++){for(int j=0;j<3;j++)emit(out(as[i][j]));emit(out(rs[i][0]));}std::puts("");
 }
 ticks+=250000;Serial8.output.clear();telemetry();
 if(Serial8.output.size()!=19)return 3;
 for(size_t i=2;i<18;i+=2){int16_t x=Serial8.output[i]|(Serial8.output[i+1]<<8);emit(x/10.f);}std::puts("");
}
'''


def run(cmd):
    result = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(f'{cmd}\n{result.stdout}\n{result.stderr}')
    return result.stdout


def snapshot(path, ref):
    path.mkdir(parents=True, exist_ok=True)
    revision=run(['git','rev-parse',ref]).strip()
    marker=path/'.revision'
    if marker.exists() and marker.read_text()==revision:return
    for name in run(['git', 'ls-tree', '-r', '--name-only', ref, 'src']).splitlines():
        target = path / Path(name).relative_to('src')
        target.parent.mkdir(parents=True, exist_ok=True)
        data = subprocess.check_output(['git', 'show', f'{ref}:{name}'], cwd=ROOT)
        target.write_bytes(data)
    marker.write_text(revision)


def function(source, name):
    """Extract a whole source function, including its unchanged preprocessor branches."""
    import re
    match=re.search(r'^void '+name+r'\([^\n]*\)\s*\{',source,re.M)
    if not match:raise ValueError(name)
    start=match.start();pos=source.index('{',start);depth=1;pos+=1
    while depth:
        depth+=(source[pos]=='{')-(source[pos]=='}');pos+=1
    return source[start:pos]+'\n'


SENSOR_HARNESS = r'''
#include "Arduino.h"
#include "config/flight_config.h"
#include "config/serial_config.h"
#include "SPI.h"
#include "sensors/sensor_processing.h"
#include "sensors/imu_internal.h"
#ifndef LEGACY
#include "math/units.h"
#endif
float dt=.002f;
SPISettings bmiSettings;
float AccX_6050,AccY_6050,AccZ_6050,GyroX_6050,GyroY_6050,GyroZ_6050;
float Acc_X_EXT,Acc_Y_EXT,Acc_Z_EXT,Gyro_X_EXT,Gyro_Y_EXT,Gyro_Z_EXT;
float AccX_prev_6050,AccY_prev_6050,AccZ_prev_6050,GyroX_prev_6050,GyroY_prev_6050,GyroZ_prev_6050;
float ang_vel[3],ang_vel_filtered[3],ang_acc[3],acc[3];
float B_accel_6050=.6f,B_gyro_6050=.47f;
CalibrationAccGyroData appliedImuCalibration={.01f,-.02f,.03f,.1f,-.2f,.3f};
FC_Binary_Packet airdata;
float temp,pressure,filteredAltitude;
struct Barometer {float readTemperature(){return 20.f;}float readPressure(){return 101325.f;}
 float readAltitude(float){return 123.f;}}bmp;
#define USE_BAROMETER
float relativeAngle_raw,relativeAngle_ready,relativeAngle_offset=350;
bool angleReadValid=false,angleReadAttempted=false;
unsigned long angleReadAttemptTime=0;
constexpr unsigned long ANGLE_RETRY_INTERVAL_US=20000;
struct Rotate {float angle=0;bool valid=true;bool readAngle(float& x){x=angle;return valid;}
 static float relativeAngle(float a,float b,bool reverse){float d=a-b;if(d>180)d-=360;if(d<-180)d+=360;return reverse?-d:d;}};
using MT6701=Rotate;Rotate angleSensor;
bool aircraftIsRight(){return false;}
struct Bus {std::vector<uint8_t> data;size_t pos=0;
 void beginTransmission(int){}void write(int){}int endTransmission(bool=true){return 0;}
 int requestFrom(uint8_t,uint8_t n){pos=0;return n;}
 int read(){return pos<data.size()?data[pos++]:0;}}Wire;
struct Mpu {void getMotion6(int16_t*a,int16_t*b,int16_t*c,int16_t*d,int16_t*e,int16_t*f){
 *a=1000;*b=-2000;*c=16000;*d=100;*e=-200;*f=300;}}mpu6050;
constexpr uint8_t ICM_ADDR=0x69,REG_ICM_BANK_SEL=0x76,REG_ICM_ACC_DATA=0x1f,SPI_READ=0x80;
#define ACCEL_SCALE_FACTOR 16384.0
#define GYRO_SCALE_FACTOR 131.0
PX4LowPassFilter2p gyroDerivFiltX,gyroDerivFiltY,gyroDerivFiltZ,angularAccFiltX,angularAccFiltY,angularAccFiltZ;
float gyroX_filt_prev=0,gyroY_filt_prev=0,gyroZ_filt_prev=0;
unsigned long prev_time_gyro_deriv=0;
float out(float x){
#ifdef LEGACY
return x;
#else
return Units::toDegrees(x);
#endif
}
float accout(float x){
#ifdef LEGACY
return x;
#else
return Units::gravityUnits(x);
#endif
}
void emit(float x){std::printf("%.9g ",x);}
void reading(){emit(accout(AccX_6050));emit(accout(AccY_6050));emit(accout(AccZ_6050));
 emit(out(GyroX_6050));emit(out(GyroY_6050));emit(out(GyroZ_6050));std::puts("");}
void word(std::vector<uint8_t>& v,int16_t x,bool little){
 if(little){v.push_back(x&255);v.push_back((x>>8)&255);}else{v.push_back((x>>8)&255);v.push_back(x&255);}}
void extFrame(uint8_t type,int16_t x,int16_t y,int16_t z,int16_t w){
 Serial1.input={0x55,type};Serial1.pos=0;
 for(auto n:{x,y,z,w})word(Serial1.input,n,true);
 uint8_t sum=0;for(auto n:Serial1.input)sum+=n;Serial1.input.push_back(sum);getIMUdata_EXT();}
// EXTRACTED_FUNCTIONS
int main(){
 for(int j=0;j<120;j++){
  SPI.data={0,0};SPI.pos=0;for(int n:{1000+j,-2000+j,5400-j})word(SPI.data,n,true);
  SPI.data.push_back(0);for(int n:{120+j,-240+j,360-j})word(SPI.data,n,true);
  getBMI088data();reading();
  Wire.data.clear();for(int n:{1000+j,-2000+j,16000-j,100+j,-200+j,300-j})word(Wire.data,n,false);
  getICM42688data();reading();getIMUdata();reading();
 }
 // Parsed external readings: accel was already m/s² and must not get a second g conversion.
 extFrame(0x52,16384,-8192,4096,0);emit(out(Gyro_X_EXT));emit(out(Gyro_Y_EXT));emit(out(Gyro_Z_EXT));
 extFrame(0x51,16384,-8192,4096,0);emit(Acc_X_EXT);emit(Acc_Y_EXT);emit(Acc_Z_EXT);std::puts("");
 extFrame(0x59,30000,3000,-4000,5000);for(float x:quat)emit(x);for(float x:att)emit(out(x));std::puts("");
 initializeAngularAccelerationFilters();initializeControlFilters();
 for(int j=0;j<400;j++){
  ticks+=j==200?20000:2000;
  // Get globals from the register path, then select/publish/filter/differentiate.
  GyroX_6050=Gyro_X_EXT;GyroY_6050=Gyro_Y_EXT;GyroZ_6050=Gyro_Z_EXT;
  updateSensorMeasurements();getAngularACC();
  for(float x:ang_vel)emit(out(x));for(float x:ang_acc)emit(out(x));
#if defined EXTIMU
  for(float x:acc)emit(x);
#else
  for(float x:acc)emit(accout(x));
#endif
  std::puts("");
 }
 for(float a:{0.f,5.f,180.f,350.f,359.9f}){
  angleSensor.angle=a;getRotateSensor1();emit(out(relativeAngle_raw));emit(out(relativeAngle_ready));std::puts("");
 }
 FC_Binary_Packet packet={0xaa,0x55,12.5f,-4.5f,23.0f,1,2,0};
 uint8_t* p=reinterpret_cast<uint8_t*>(&packet);for(size_t i=2;i<sizeof(packet)-1;i++)packet.checksum+=p[i];
 Serial8.input.assign(p,p+sizeof(packet));getairdata();
 emit(out(airdata.aoa));emit(out(airdata.aos));emit(airdata.tas);std::puts("");
 getbarodata();
#ifdef LEGACY
 emit(temp);
#else
 emit(temp-273.15f);
#endif
 emit(pressure);emit(filteredAltitude);std::puts("");
 // Exercise the real SI-to-legacy Madgwick wrapper as well as its core estimator.
 for(int i=0;i<50;i++){Madgwick();for(float x:quat)emit(x);for(float x:att)emit(out(x));std::puts("");}
}
'''


def sensors(compiler, stubs, legacy):
    results=[]
    for label,src in [('legacy',legacy),('si',ROOT/'src')]:
        source=(src/'sensors/sensor_processing.cpp').read_text(encoding='utf-8')
        names=['getBMI088data','getICM42688data','getIMUdata','getIMUdata_EXT',
               'initializeAngularAccelerationFilters','getAngularACC','updateSensorMeasurements',
               'getRotateSensor1','getairdata','getbarodata']
        code=SENSOR_HARNESS.replace('// EXTRACTED_FUNCTIONS', '\n'.join(function(source,n) for n in names))
        # extFrame calls the extracted function defined below it.
        code=code.replace('void extFrame(', 'void getIMUdata_EXT();\nvoid extFrame(')
        harness=OUT/f'sensors-{label}.cpp';harness.write_text(code,encoding='utf-8')
        for external in (False,True):
            exe=OUT/f'sensors-{label}-{external}.exe'
            cmd=[compiler,'-std=c++17','-O2','-I'+str(stubs),'-I'+str(src),str(harness),
                 str(src/'sensors/filters.cpp'),'-o',str(exe)]
            if label=='legacy':cmd+=['-DLEGACY']
            if external:cmd+=['-DEXTIMU']
            run(cmd);results.append([float(x) for x in run([str(exe)]).split()])
    reports=[]
    for i,external in enumerate((False,True)):
        a,b=results[i],results[i+2]
        assert len(a)==len(b)
        for j,(x,y) in enumerate(zip(a,b)):
            derivative=2173<=j<5773 and (j-2173)%9 in (3,4,5)
            tolerance=(.1 if derivative else .0003)+abs(x)*.0002
            if not math.isfinite(x+y) or abs(x-y)>tolerance:
                raise AssertionError(f'sensor external={external} index={j}: {x} != {y}')
        worst=max(abs(x-y) for x,y in zip(a,b))
        reports.append(f'sensor external={external}: PASS, {len(a)} values, max absolute difference={worst:.8g}')
    return reports


def audit(legacy):
    """Check the user's explicit exclusions and publication/display contracts."""
    def text(src,name):return (src/name).read_text(encoding='utf-8')
    preserved=['system/flight_clock.cpp','system/flight_clock.h',
               'actuators/actuator_output.cpp','actuators/actuator_output.h',
               'radio_comm/radio_comm.cpp','radio_comm/radio_comm.h']
    preserved += [str(p.relative_to(legacy)) for p in (legacy/'parameters').glob('*') if p.suffix in ('.cpp','.h')]
    preserved += [str(p.relative_to(legacy)) for p in (legacy/'control').glob('*_controller.h')]
    for name in preserved:
        assert text(legacy,name)==text(ROOT/'src',name), f'excluded file changed: {name}'
    for p in (legacy/'control').glob('*_controller.cpp'):
        old=text(legacy,str(p.relative_to(legacy)))
        new=text(ROOT/'src',str(p.relative_to(legacy)))
        strip=lambda s:re.sub(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"','',s,flags=re.S)
        numbers=lambda s:re.findall(r'(?<![\w.])(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?[fFuUlL]*',strip(s))
        assert numbers(old)==numbers(new), f'controller numeric literal changed: {p.name}'
        names=lambda s:re.findall(r'\b(?:float|double|bool|int|auto)\s+(\w+)',strip(s))
        assert names(old)==names(new), f'controller variable or argument renamed: {p.name}'
    types=text(ROOT/'src','control/control_types.h')
    assert not re.search(r'\b\w*(?:_deg|Deg)\w*\b',types)
    log=text(ROOT/'src','logging/logger.cpp')
    for name in ['q_dot_ref','q_ref','q_f','q_dot_meas','delta_e_cmd','delta_e_hat']:
        assert f'LOG_FIELD(pitchIndiControlOutput.{name});' in log
    assert 'writer.field("si_units_version", 1);' in log
    assert 'LOG_FIELD(current_time);' in log and 'LOG_ARRAY(ActuatorOutput);' in log
    oled=text(ROOT/'src','interface/human_interface.cpp')
    for name in ['att[0]','att[1]','att[2]','relativeRoll']:
        assert f'display.print(Units::toDegrees({name}), 1)' in oled
    ground=text(ROOT/'src','ground_station_service/ground_station_service.cpp')
    for name in ['att[0]','att[1]','att[2]','ang_vel[0]','ang_vel[1]','ang_vel[2]']:
        assert f'Units::toDegrees({name})' in ground
    assert 'groundAcceleration(acc[0]), groundAcceleration(acc[1]), groundAcceleration(acc[2])' in ground
    source=text(ROOT/'src','sensors/sensor_processing.cpp')
    assert 'ParameterKey::Mt6701Zero, Units::toDegrees(relativeAngle_raw)' in source
    assert 'float filteredPressure = 101325.0f;' in source
    assert 'temp = bmp.readTemperature() + 273.15f;' in source
    for axis,value in [('X','-0.02'),('Y','-0.01'),('Z','0.03')]:
        assert f'float AccError{axis}_6050 = Units::acceleration({value}f);' in source
    for axis,value in [('X','-13.83'),('Y','-9.41'),('Z','1.48')]:
        assert f'float GyroError{axis}_6050 = Units::toRadians({value}f);' in source
    allocation=text(ROOT/'src','control/control_allocation.cpp')
    unwrapped=re.sub(r'Units::toDegrees\((att(?:_[B-G])?\[\d\])\)',r'\1',allocation)
    unwrapped=unwrapped.replace('#include "math/units.h"\n','')
    assert unwrapped==text(legacy,'control/control_allocation.cpp')
    return [f'exclusion audit: PASS, {len(preserved)} files unchanged',
            'controller declarations/numeric literals: PASS',
            'SI fields, CSV schema, GCS/OLED and calibration boundary audit: PASS']


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', default='6a5ddeea176a38b4e865b11b9f0f4c60f3fc727b')
    parser.add_argument('--sensors-only', action='store_true')
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    legacy = OUT / 'baseline'
    snapshot(legacy, args.baseline)
    stubs = OUT / 'stubs'
    for name, content in STUBS.items():
        p=stubs/name; p.parent.mkdir(parents=True,exist_ok=True);p.write_text(content,encoding='utf-8')
    harness = OUT/'regression.cpp'
    # Radio variables are stubs shared across translation units.
    harness.write_text('#include "radio_comm/radio_comm.h"\n'+HARNESS,encoding='utf-8')
    compiler=shutil.which('g++')
    if not compiler: raise RuntimeError('g++ is required')
    if args.sensors_only:
        print('\n'.join(sensors(compiler,stubs,legacy)))
        return
    summary=[]
    for count in (1,3,4,5,7):
        results=[]
        for label,src in [('legacy',legacy),('si',ROOT/'src')]:
            exe=OUT/f'{label}-{count}.exe'
            cmd=[compiler,'-std=c++17','-O2','-ffunction-sections','-fdata-sections',
                 '-Wl,--gc-sections',f'-DCOFLY_AIRCRAFT_COUNT={count}',
                 '-I'+str(stubs),'-I'+str(src),str(harness)]
            if label=='legacy':cmd+=['-DLEGACY']
            cmd += [str(src/name) for name in SOURCES]+['-o',str(exe)]
            run(cmd)
            output=run([str(exe)])
            (OUT/f'{label}-{count}.txt').write_text(output)
            results.append([float(x) for x in output.split()])
        a,b=results
        if len(a)!=len(b):raise AssertionError((len(a),len(b)))
        worst=0.0
        for i,(x,y) in enumerate(zip(a,b)):
            # Old wire: 0.1 degree truncation; new wire: nearest 0.001 radian.
            # Combined angular error <= 0.1 + degrees(0.0005), plus float slack.
            protocol = i >= (900*87 + 10*9 + 200*7)
            tolerance = .129 if protocol else 0.002+abs(x)*.0001
            if not math.isfinite(x+y) or abs(x-y)>tolerance:
                raise AssertionError(f'count={count}, value={i}: {x} != {y}, tolerance={tolerance}')
            worst=max(worst,abs(x-y))
        summary.append(f'count={count}: PASS, {len(a)} values, max absolute difference={worst:.8g}')
    summary+=sensors(compiler,stubs,legacy)
    summary+=audit(legacy)
    print('\n'.join(summary))
    (OUT/'results.txt').write_text('\n'.join(summary)+'\n')


if __name__=='__main__':main()
