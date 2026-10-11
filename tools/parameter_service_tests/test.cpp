#include <cstdio>
#include <cstdlib>
#include <vector>
#include "Arduino.h"
#include "SD.h"
#include "parameters/parameter_service.h"
#include "parameters/parameter_eeprom.h"
#include "parameters/parameter_text.h"
#include "parameters/imu_calibration_integrity.h"

#define CHECK(value) do { if (!(value)) { fprintf(stderr,"Failed line %d: %s\n",__LINE__,#value); std::exit(1); } } while (0)
uint32_t testTime=0;
MockSerial Serial;
MockSD SD;
std::map<std::string,std::vector<uint8_t>> files;
bool sdFails=false, eeFails=false, locked=true;
unsigned sdWrites=0, eeWrites=0;
float gain=0, other=0, offsets[12]={}, zero=0;
int32_t id=0, planes=3, valid=1, model=1, fingerprint=0;
std::map<std::string,double> eeValues;

const FlightParameter *flightParameterTable(size_t &count) {
  static std::vector<FlightParameter> params=[] {
    fingerprint=imuCalibrationFingerprint(offsets,1);
    std::vector<FlightParameter> p={
      {1,"gain",&gain,0,10,"T","T"}, {2,"other",&other,0,10,"T","T"},
      {1001,"aircraft_id",&id,0,7,"T","T",ParameterStorage::EEPROM,ParameterEffect::Reboot},
      {1002,"aircraft_count",&planes,1,7,"T","T",ParameterStorage::EEPROM,ParameterEffect::Reboot}};
    const char *names[]={"offset0","offset1","offset2","offset3","offset4","offset5","offset6","offset7","offset8","offset9","offset10","offset11"};
    for (unsigned i=0;i<12;++i) p.emplace_back(1101+i,names[i],offsets+i,-10,10,"T","T",ParameterStorage::EEPROM);
    p.emplace_back(1113,"valid",&valid,0,1,"T","T",ParameterStorage::EEPROM,ParameterEffect::Immediate,true);
    p.emplace_back(1114,"model",&model,1,1,"T","T",ParameterStorage::EEPROM,ParameterEffect::Immediate,true);
    p.emplace_back(1115,"fingerprint",&fingerprint,INT32_MIN,INT32_MAX,"T","T",ParameterStorage::EEPROM,ParameterEffect::Immediate,true);
    p.emplace_back(1201,"zero",&zero,0,360,"T","T",ParameterStorage::EEPROM);
    return p;
  }();
  count=params.size();return params.data();
}
bool initializeEEPROMParameters() { return true; }
bool readEEPROMParameter(const FlightParameter &p,double &value) {
  if (p.storage!=ParameterStorage::EEPROM) return false;
  value=p.defaultValue;return true;
}
bool hasEEPROMMigrationSource(const FlightParameter &) { return false; }
bool saveEEPROMParameters(const FlightParameter *p,size_t n,const double *v,bool) {
  ++eeWrites;if (eeFails) return false;
  for (size_t i=0;i<n;++i) if (p[i].storage==ParameterStorage::EEPROM) eeValues[p[i].name]=v[i];
  return true;
}
const char *eepromParameterState() { return "TEST"; }
bool isFlightLocked() { return locked; }
void setFlightLocked(bool value) { locked=value; }
bool validAircraftConfiguration(int value,int total) { return value>=0 && value<=total; }
void activateAircraftConfiguration() {}
const char *aircraftName() { return "TEST"; }
int aircraftCount() { return planes; }
void loadImuCalibration() {}
void handleGroundStationCommand(char *,char *reply,size_t) { *reply=0; }
void pollDeviceReboot(bool) {}
bool deviceRebootPending() { return false; }

void at(uint32_t time) { testTime=time;pollParameterService(); }
void reset(bool sd=true) {
  locked=true;sdFails=eeFails=false;testTime=0;Serial={};files.clear();eeValues.clear();
  initializeParameterService(sd);sdWrites=eeWrites=0;
}
double sdValue(const char *name) {
  const auto &data=files.at("params.cfg");ParameterText::View view;
  CHECK(ParameterText::valid(data.data(),data.size(),view));
  size_t offset=view.records;
  for (size_t i=0;i<view.rows;++i) {
    char record[PARAMETER_RECORD_BUFFER_SIZE], *key, *type;double value;
    CHECK(ParameterText::line(data.data(),data.size(),offset,record,sizeof(record)));
    CHECK(ParameterText::decode(record,key,type,value));
    if (!strcmp(name,key)) return value;
  }
  CHECK(false);return 0;
}
int main() {
  // The separated SD module must preserve primary/backup and schema restoration.
  reset();CHECK(saveParameterValue(1,1));CHECK(flushParameterWrites());
  CHECK(saveParameterValue(1,2));CHECK(flushParameterWrites());
  const auto backup=files.at("params_backup.cfg");
  files["params.cfg"][0]^=1;
  gain=7;sdWrites=0;initializeParameterService(true);
  CHECK(gain==1 && sdWrites==0); // Recover the old validated backup.
  CHECK(saveParameterValue(1,3));CHECK(flushParameterWrites());
  CHECK(sdValue("gain")==3 && files.at("params_backup.cfg")==backup);
  files["params.cfg"][0]^=1;files["params_backup.cfg"][0]^=1;
  gain=7;initializeParameterService(true);CHECK(gain==0);

  reset();size_t registryCount;const auto *registry=flightParameterTable(registryCount);
  uint8_t migrated[256];size_t body=ParameterText::begin(migrated,sizeof(migrated),2);
  FlightParameter obsolete(99,"obsolete",&other,0,10,"T","T");
  body+=ParameterRecord::encode(registry[0],5,reinterpret_cast<char *>(migrated+body),sizeof(migrated)-body);
  body+=ParameterRecord::encode(obsolete,4,reinterpret_cast<char *>(migrated+body),sizeof(migrated)-body);
  const size_t used=ParameterText::finish(migrated,sizeof(migrated),body);CHECK(used);
  files["params.cfg"]={migrated,migrated+used};const auto original=files.at("params.cfg");
  initializeParameterService(true);
  CHECK(gain==5 && other==0 && sdValue("gain")==5);
  CHECK(files.at("params_backup.cfg")==original); // Keep the exact pre-migration file.

  reset();
  CHECK(saveParameterValue(1,1));CHECK(gain==1);CHECK(sdWrites==0);
  at(1000);CHECK(saveParameterValue(2,2));
  at(2999);CHECK(sdWrites==0);at(3000);CHECK(sdWrites==1);
  CHECK(sdValue("gain")==1 && sdValue("other")==2);CHECK(eeWrites==0);
  at(4000);CHECK(saveParameterValue(1,3));
  CHECK(!saveParameterValue(1,20)); // Invalid edits also do not reset the timer.
  at(5500);CHECK(saveParameterValue(1,3)); // Same value must not reset timer.
  at(6000);CHECK(sdWrites==3); // Exactly one old backup and one new primary.
  at(6100);CHECK(saveParameterValue(1,4));at(6200);CHECK(saveParameterValue(1,3));
  at(8199);CHECK(sdWrites==3);at(8200);CHECK(sdWrites==5);
  CHECK(sdValue("gain")==3); // Reverting is still a RAM change: save the final snapshot.

  reset();CHECK(saveParameterValue(1,1));at(500);CHECK(saveParameterValue(1201,12));
  at(1500);CHECK(saveParameterValue(2,2));at(2500);CHECK(eeWrites==1 && sdWrites==0);
  CHECK(eeValues["zero"]==12);at(3500);CHECK(sdWrites==1 && eeWrites==1);

  reset();CHECK(saveParameterValue(1,1));CHECK(saveParameterValue(1201,12));eeFails=true;
  at(2000);CHECK(eeWrites==1 && sdWrites==1);CHECK(gain==1 && zero==12);
  at(2001);CHECK(eeWrites==1);eeFails=false;at(3999);CHECK(eeWrites==1);
  at(4000);CHECK(eeWrites==2 && sdWrites==1 && eeValues["zero"]==12);

  reset();CHECK(saveParameterValue(1,1));sdFails=true;at(2000);CHECK(files.empty());
  CHECK(gain==1);sdFails=false;at(4000);CHECK(sdValue("gain")==1);
  reset(false);CHECK(saveParameterValue(1,1));at(2000);CHECK(gain==1 && sdWrites==0);
  CHECK(!flushParameterWrites()); // Cannot safely reboot with unsaved SD changes.

  reset();CHECK(saveParameterValue(1001,1));CHECK(parameterRebootRequired());CHECK(locked);
  CHECK(eeWrites==0);CHECK(flushParameterWrites());CHECK(eeWrites==1 && eeValues["aircraft_id"]==1);
  locked=false;CHECK(!saveParameterValue(1001,2));CHECK(!saveParameterValue(1201,2));

  reset();CHECK(saveParameterValue(1001,1));CHECK(saveParameterValue(1001,0));
  at(2000);CHECK(eeWrites==1 && parameterRebootRequired());
  CHECK(saveParameterValue(1201,2));eeFails=true;CHECK(!flushParameterWrites());
  eeFails=false;CHECK(flushParameterWrites());CHECK(eeValues["zero"]==2);

  reset();CHECK(saveParameterValue(1,2));float newOffsets[12]={1};
  CHECK(saveImuCalibrationParameters(newOffsets));
  CHECK(sdWrites==0 && eeWrites==0 && valid==1 && offsets[0]==1);
  CHECK(fingerprint==imuCalibrationFingerprint(newOffsets,1));
  at(1000);CHECK(saveImuCalibrationParameters(newOffsets)); // Identical group does not reset timer.
  at(1999);CHECK(sdWrites==0 && eeWrites==0);at(2000);
  CHECK(sdWrites==1 && sdValue("gain")==2 && eeWrites==1 && eeValues["offset0"]==1);
  at(5000);CHECK(eeWrites==1 && sdWrites==1); // One snapshot, not three calibration writes.

  reset();CHECK(saveParameterValue(1201,2));at(1000);CHECK(saveImuCalibrationParameters(newOffsets));
  at(2000);CHECK(eeWrites==0);eeFails=true;at(3000);CHECK(eeWrites==1 && valid==1 && offsets[0]==1);
  eeFails=false;at(5000);CHECK(eeWrites==2 && eeValues["valid"]==1 && eeValues["offset0"]==1);

  reset();CHECK(saveParameterValue(1201,2));float invalidOffsets[12]={20};
  CHECK(!saveImuCalibrationParameters(invalidOffsets));CHECK(offsets[0]==0 && valid==1 && eeWrites==0);
  CHECK(!saveImuCalibrationParameters(nullptr));locked=false;
  CHECK(!saveImuCalibrationParameters(newOffsets));locked=true;at(2000);CHECK(eeWrites==1);

  reset();CHECK(saveParameterValue(1101,2));CHECK(offsets[0]==2 && valid==0);
  at(1999);CHECK(eeWrites==0);at(2000);CHECK(eeWrites==1 && eeValues["valid"]==0);

  // Splitting a calibration group is rejected; moving the whole group to SD
  // still uses the same ordinary delayed snapshot logic.
  reset();size_t parameterCount;auto *p=const_cast<FlightParameter *>(flightParameterTable(parameterCount));
  size_t marker=0;while (p[marker].id!=1113) ++marker;
  p[marker].storage=ParameterStorage::SD;
  initializeParameterService(true);CHECK(!saveImuCalibrationParameters(newOffsets));
  CHECK(!saveParameterValue(1101,2));
  for (size_t i=0;i<parameterCount;++i)
    if (p[i].id>=1101 && p[i].id<=1115) p[i].storage=ParameterStorage::SD;
  reset();CHECK(saveImuCalibrationParameters(newOffsets));CHECK(sdWrites==0 && eeWrites==0);
  at(2000);CHECK(sdWrites==1 && eeWrites==0 && sdValue("offset0")==1 && sdValue("valid")==1);
  for (size_t i=0;i<parameterCount;++i)
    if (p[i].id>=1101 && p[i].id<=1115) p[i].storage=ParameterStorage::EEPROM;

  reset();at(UINT32_MAX-500);CHECK(saveParameterValue(1,1));at(1498);CHECK(sdWrites==0);
  at(1499);CHECK(sdWrites==1); // Timer survives millis wraparound.

  reset();Serial.connected=true;Serial.input="PARAM SET 9 gain 2\n";
  at(0);at(1);CHECK(Serial.output.find("OK\tgain\tfloat\t2\tPENDING")!=std::string::npos);
  at(2000);at(2001);CHECK(Serial.output.find("STORAGE\tSD\tSAVED")!=std::string::npos);
  CHECK(sdWrites==1);Serial.connected=false;at(2100);CHECK(saveParameterValue(1,3));
  at(4100);CHECK(sdWrites==3); // USB disconnect does not stop persistence.
  puts("Parameter service debounce integration tests passed");
}
