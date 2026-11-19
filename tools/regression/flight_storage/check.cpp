// Run with the adjacent run.py. Exercise production producers, serializers, SD
// session and logger, using only deterministic replacements for hardware/RTOS.
#include "logging/logger.cpp"
#include "ground_station_service/sd_file_service.cpp"
#include "parameters/parameter_sd.h"

unsigned long current_time; float dt, quat[4], att[3], ang_vel[3], ang_acc[3], acc[3], ang_vel_filtered[3];
float airspeed, relativeAngle_raw, relativeAngle_ready, ActuatorOutput[5], manual_input[4];
int32_t strain_value[5];
float att_B[3],att_C[3],att_D[3],att_E[3],att_F[3],att_G[3];
float ang_vel_B[3],ang_vel_C[3],ang_vel_D[3],ang_vel_E[3],ang_vel_F[3],ang_vel_G[3];
#define CHANNEL(n) int radio_channel##n##_raw; float radio_channel##n##_normalized;
CHANNEL(1) CHANNEL(2) CHANNEL(3) CHANNEL(4) CHANNEL(5) CHANNEL(6) CHANNEL(7) CHANNEL(8)
FlightMode currentFlightMode,previousFlightMode;
PitchController activePitchController,previousPitchController;
float mixerStrategyCommand,configurationAngle; bool brakeEnabled,controlModeChanged;
namespace ControlParameters { int32_t integralControlEnabled; float configurationControlScale; }
ControlTargets targets; ControlCycleOutput commands;
RollControlOutput rollControlOutput; PitchPidControlOutput pitchPidControlOutput;
IndiPitchControlOutput pitchIndiControlOutput; RollDampingControlOutput rollDampingControlOutput;
YawRateControlOutput yawRateControlOutput; ThrottleControlOutput throttleControlOutput;
ConfigurationControlOutput configurationControlOutput;
bool locked=true, reboot=false;
bool isFlightLocked(){return locked;} bool deviceRebootPending(){return reboot;}
bool rotateSensorValid(){return true;} bool radioInputValid(){return true;}
bool isSensorDataFresh(bool left){return left;} AirspeedDiagnostics airspeedDiagnostics(){return {};}

void pumpStorage() {
  assert(currentTask==taskIdentity);
  currentTask=taskIdentity+1; serviceFlightLogger(); currentTask=taskIdentity;
}
int sequence=0;
template<class T> void seed(T &v) { v=static_cast<T>(++sequence); }
void seed(bool &v) { v=(++sequence%2)!=0; }
void seed(bool &&) {}
void seed(FlightMode &v) { v=static_cast<FlightMode>(++sequence%2); }
void seed(PitchController &v) { v=static_cast<PitchController>(++sequence%2); }
template<size_t N> void seedArray(float (&values)[N]) { for(auto &v:values) seed(v); }
template<size_t N> void seedArray(int32_t (&values)[N]) { for(auto &v:values) seed(v); }
void seedCycle() {
#define FIELD(v) seed(v);
#define ARRAY(v) seedArray(v);
#include "legacy_fields.inc"
#undef FIELD
#undef ARRAY
  nowUs += 20000; current_time=nowUs;
}
std::string oldCsv(bool header) {
  CsvLogWriter writer(header);
#define FIELD(v) writer.field(#v,v);
#define ARRAY(v) writer.array(#v,v);
#include "legacy_fields.inc"
#undef FIELD
#undef ARRAY
  return std::string(dataString)+"\r\n";
}

std::string command(const std::string &text) {
  char line[768], reply[SD_FILE_RESPONSE_SIZE]; strcpy(line,text.c_str());
  handleSdFileCommand(line,reply,sizeof(reply)); return reply;
}
void checkParameterRecovery() {
  int32_t id=1;
  float gain=0;
  const FlightParameter fields[]={{"aircraft_id",&id,0,3,"",""},{"gain",&gain,0,10,"",""}};
  const double defaults[]={1,0}, first[]={1,2}, next[]={1,4};
  const auto valid=[](size_t, double){return true;};
  // In the sync/close cases write() returns its full byte count and the bytes
  // are even readable from the simulated cache. A commit must still fail.
  for (auto *fault : {&shortWritePath, &syncErrorPath, &closeErrorPath}) {
    ParameterSD::initialize(true,fields,2,defaults,valid);
    assert(ParameterSD::save(first));
    const auto saved=files.at("/params.cfg");
    *fault="/params.cfg";
    assert(!ParameterSD::save(next));
    assert(!files.count("/params.cfg") && files.at("/params_backup.cfg")==saved);
    ParameterSD::initialize(true,fields,2,defaults,valid);
    bool migrate=false; double recovered[2]={};
    ParameterSD::MigrationStats stats{};
    assert(ParameterSD::load(recovered,migrate,stats));
    assert(recovered[0]==1 && recovered[1]==2 && !migrate);
    fault->clear();
    assert(ParameterSD::save(next));
    assert(ParameterSD::load(recovered,migrate,stats) && recovered[1]==4);
  }
  // Both the ordinary and exact-copy migration backup must reach the card
  // before the verified primary is replaced.
  const auto priorPrimary=files.at("/params.cfg");
  syncErrorPath="/params_backup.cfg";
  for (bool migration : {false,true}) {
    assert(!ParameterSD::save(first,migration));
    assert(files.at("/params.cfg")==priorPrimary);
  }
  syncErrorPath.clear();
}
int main() {
  initializeLogger(); startFlightStorage();
  seedCycle();
  std::string expected=oldCsv(true);
  for(int i=0;i<9;++i) {
    seedCycle(); if(i<8) expected+=oldCsv(false);
    captureFlightServices(true);
  }
  assert(droppedFlightLogSamples()==1);
  FlightServiceSample latest; copyFlightServiceSample(latest);
  assert(latest.current_time==nowUs && latest.dt==dt);
  // Mutate every live input before any row is formatted. Older queued rows
  // must match the exact bytes that the single-loop formatter produced.
  seedCycle();
  for(int i=0;i<8;++i) pumpStorage();
  const std::string path="/datalog001.csv";
  assert(files.at(path)==expected);

  seedCycle(); expected+=oldCsv(false); captureFlightServices(true);
  const auto opened=command("SD OPEN 1 2f646174616c6f673030312e637376");
  assert(opened=="@COFLY\t1\tSD_OPEN\t"+std::to_string(expected.size())+"\n");
  assert(sdFileSessionActive() && files.at(path)==expected);
  seedCycle(); captureFlightServices(true);
  auto first=command("SD READ 2 0 32");
  assert(first.find("SD_DATA\t0\t")!=std::string::npos && command("SD READ 2 0 32")==first);
  assert(command("SD READ 3 0 32").find("OFFSET")!=std::string::npos);
  assert(files.at(path)==expected);
  locked=false; pollSdFileService(true);
  assert(!sdFileSessionActive() && command("SD LIST 4 2f").find("LOCK_REQUIRED")!=std::string::npos);
  locked=true;
  assert(command("SD LIST 5 2f").find("SD_BEGIN")!=std::string::npos);
  nowUs+=15001000; pollSdFileService(true); assert(!sdFileSessionActive());
  assert(command("SD LIST 6 2f").find("SD_BEGIN")!=std::string::npos);
  pollSdFileService(false); assert(!sdFileSessionActive());

  // Media failure differs from an overloaded producer queue. A partially
  // written row is rolled back and later samples cannot corrupt that file.
  failWrite=true; seedCycle(); captureFlightServices(true); pumpStorage();
  assert(flightLogWriteFailures()==1 && droppedFlightLogSamples()==1 && files.at(path)==expected);
  failWrite=false; seedCycle(); captureFlightServices(true);
  assert(!uxQueueMessagesWaiting(logQueue));
  checkParameterRecovery();
}
