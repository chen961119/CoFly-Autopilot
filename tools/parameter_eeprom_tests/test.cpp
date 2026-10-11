#include "parameters/parameter_eeprom.h"
#include "EEPROM.h"
#include "parameters/parameter_text.h"
#include <assert.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <fstream>
#undef assert
#define assert(test) do { if (!(test)) { std::cerr << "Failed at line " << __LINE__ << ": " << #test << '\n'; std::exit(1); } } while (0)

MockEEPROM EEPROM;
float gain=.25f;
int32_t identity=0;
FlightParameter table[] = {
  {1,"gain",&gain,0,10,"T","T",ParameterStorage::EEPROM},
  {2,"aircraft_id",&identity,0,7,"T","T",ParameterStorage::EEPROM},
};
constexpr int PRIMARY=128, BACKUP=2176, SLOT=2048;
const double oldValues[]={.25,2}, newValues[]={.75,3};

bool matches(const double *expected) {
  for (unsigned i=0;i<2;++i) {
    double value=-1;
    if (!readEEPROMParameter(table[i],value) || value!=expected[i]) return false;
  }
  return true;
}
void empty() {
  EEPROM.bytes.fill(0xff);EEPROM.resetFaults();
  assert(!initializeEEPROMParameters());
}
int main() {
  empty();
  assert(saveEEPROMParameters(table,2,oldValues));
  const auto first=EEPROM.bytes;
  ParameterText::View firstView;assert(ParameterText::header(first.data()+PRIMARY,SLOT,firstView));
  const size_t firstUsed=firstView.used;
  std::ofstream fixture("tools/ground_station/build/parameter_text_fixture.cfg",std::ios::binary);
  fixture.write(reinterpret_cast<const char *>(first.data()+PRIMARY),firstUsed);fixture.close();
  // Text names have actual variable lengths; shortest decimals preserve -0.
  for (size_t width : {1u,27u,48u,128u}) {
    const std::string name(width,'x');
    uint8_t buffer[240];size_t body=ParameterText::begin(buffer,sizeof(buffer),1);
    const int n=snprintf(reinterpret_cast<char *>(buffer+body),sizeof(buffer)-body,"%s:float=-0\n",name.c_str());
    assert(n>0);body+=n;
    const size_t used=ParameterText::finish(buffer,sizeof(buffer),body);ParameterText::View view;
    assert(used && ParameterText::valid(buffer,used,view));
    char record[PARAMETER_RECORD_BUFFER_SIZE],*decodedName,*type;double value;size_t offset=view.records;
    assert(ParameterText::line(buffer,used,offset,record,sizeof(record)));
    assert(ParameterText::decode(record,decodedName,type,value));assert(name==decodedName);
    const float floating=value;uint32_t bits;memcpy(&bits,&floating,4);assert(bits==0x80000000u);
    buffer[view.records+width+1]='x';ParameterText::finish(buffer,sizeof(buffer),body);
    assert(!ParameterText::valid(buffer,used,view));
  }
  assert(initializeEEPROMParameters() && matches(oldValues));
  // The restored text formatter must retain the exact float32 bit pattern,
  // including subnormals, negative zero and the largest finite magnitudes.
  for (uint32_t bits : {0u,0x80000000u,1u,0x80000001u,0x00800000u,0x7f7fffffu,0xff7fffffu,0x3eaaaaabu}) {
    float number;memcpy(&number,&bits,4);
    FlightParameter p(9,"value",&number,-3.4028234663852886e38,3.4028234663852886e38,"T","T");
    char record[PARAMETER_RECORD_BUFFER_SIZE];assert(ParameterRecord::encode(p,number,record,sizeof(record))>0);
    char *newline=strchr(record,'\n');assert(newline);*newline=0;
    char *name,*type;double value;assert(ParameterText::decode(record,name,type,value));
    const float restored=value;uint32_t restoredBits;memcpy(&restoredBits,&restored,4);assert(bits==restoredBits);
  }
  EEPROM.resetFaults();
  assert(saveEEPROMParameters(table,2,oldValues));
  assert(EEPROM.writes==0); // unchanged primary is not rewritten
  assert(saveEEPROMParameters(table,2,newValues));
  const auto both=EEPROM.bytes;
  assert(initializeEEPROMParameters() && matches(newValues));
  uint8_t longer[2048];size_t body=ParameterText::begin(longer,sizeof(longer),2);
  const char *records="gain:float=0.2500\naircraft_id:int=2\n";
  memcpy(longer+body,records,strlen(records));body+=strlen(records);
  const size_t longerUsed=ParameterText::finish(longer,sizeof(longer),body);
  auto differentText=first;
  std::fill_n(differentText.begin()+PRIMARY,SLOT,0xff);
  std::copy_n(longer,longerUsed,differentText.begin()+PRIMARY);

  // Primary is preferred even when backup holds a different valid snapshot.
  std::copy_n(both.begin()+BACKUP,SLOT,EEPROM.bytes.begin()+PRIMARY);
  std::copy_n(both.begin()+PRIMARY,SLOT,EEPROM.bytes.begin()+BACKUP);
  assert(initializeEEPROMParameters() && matches(oldValues));

  EEPROM.bytes=both;EEPROM.bytes[PRIMARY+14]^=1;
  assert(initializeEEPROMParameters() && matches(oldValues));
  const auto recovered=EEPROM.bytes;
  const auto backup=std::vector<uint8_t>(EEPROM.bytes.begin()+BACKUP,EEPROM.bytes.begin()+BACKUP+SLOT);
  assert(saveEEPROMParameters(table,2,oldValues)); // same values must repair primary
  assert(std::equal(backup.begin(),backup.end(),EEPROM.bytes.begin()+BACKUP));
  assert(initializeEEPROMParameters() && matches(oldValues));

  // Every write interruption during backup creation and primary update.
  int cutCases=0;
  for (const auto &baseline : {first,both,recovered,differentText}) {
    EEPROM.bytes=baseline;EEPROM.resetFaults();
    assert(initializeEEPROMParameters());
    const double expected[]={baseline==both ? .75 : .25,baseline==both ? 3. : 2.};
    const double target[]={.5,4};
    assert(saveEEPROMParameters(table,2,target));
    const int writes=EEPROM.writes;
    for (int cut=0;cut<=writes;++cut) {
      EEPROM.bytes=baseline;EEPROM.resetFaults();
      assert(initializeEEPROMParameters());
      EEPROM.cut=cut;
      try { saveEEPROMParameters(table,2,target); } catch (const PowerCut &) {}
      EEPROM.resetFaults();
      assert(initializeEEPROMParameters());
      if (!matches(expected) && !matches(target)) {
        std::cerr << "Cut " << cut << "/" << writes << " expected " << expected[0] << "," << expected[1] << '\n';
        for (const auto &p : table) { double v=-1;std::cerr << p.name << " " << readEEPROMParameter(p,v) << " " << v << '\n'; }
      }
      assert(matches(expected) || matches(target));
      ++cutCases;
    }
    for (int fail=0;fail<writes;++fail) {
      EEPROM.bytes=baseline;EEPROM.resetFaults();
      assert(initializeEEPROMParameters());
      EEPROM.fail=fail;
      const bool saved=saveEEPROMParameters(table,2,target);
      EEPROM.resetFaults();
      // A failed write may be retried in the same boot, using the durable backup.
      assert(saveEEPROMParameters(table,2,target));
      assert(initializeEEPROMParameters() && matches(target));
      (void)saved; // A skipped update of an already matching byte is harmless.
    }
  }
  // Adding a record moves the footer; every interruption must retain the old
  // two-record snapshot or publish the complete new three-record snapshot.
  FlightParameter expanded[]={table[0],table[1],
      {3,"extra",&gain,0,10,"T","T",ParameterStorage::EEPROM}};
  const double expandedValues[]={.5,4,.125};
  EEPROM.bytes=first;EEPROM.resetFaults();assert(initializeEEPROMParameters());
  assert(saveEEPROMParameters(expanded,3,expandedValues));
  const int expandedWrites=EEPROM.writes;
  for (int cut=0;cut<=expandedWrites;++cut) {
    EEPROM.bytes=first;EEPROM.resetFaults();assert(initializeEEPROMParameters());EEPROM.cut=cut;
    try { saveEEPROMParameters(expanded,3,expandedValues); } catch (const PowerCut &) {}
    EEPROM.resetFaults();assert(initializeEEPROMParameters());
    double extra=-1;const bool found=readEEPROMParameter(expanded[2],extra);
    assert((matches(oldValues) && !found) || (matches(expandedValues) && found && extra==.125));
    ++cutCases;
  }
  // Silent write failures must never be reported as saved.
  EEPROM.bytes=first;EEPROM.resetFaults();assert(initializeEEPROMParameters());
  EEPROM.fail=0;
  assert(!saveEEPROMParameters(table,2,newValues));
  EEPROM.resetFaults();assert(initializeEEPROMParameters() && matches(oldValues));

  // Both snapshots invalid: no partial parameter recovery.
  EEPROM.bytes=both;EEPROM.bytes[PRIMARY+14]^=1;EEPROM.bytes[BACKUP+14]^=1;
  assert(!initializeEEPROMParameters());
  for (const auto &p : table) { double v=-1;assert(!readEEPROMParameter(p,v)); }
  assert(std::string(eepromParameterState())=="CORRUPT");
  // Previous layout and insufficient EEPROM capacity are rejected.
  EEPROM.bytes=both;EEPROM.bytes[PRIMARY+4]=3;EEPROM.bytes[BACKUP+4]=3;
  assert(!initializeEEPROMParameters());
  EEPROM.capacity=4223;assert(!initializeEEPROMParameters());
  assert(!saveEEPROMParameters(table,2,oldValues));EEPROM.capacity=4284;
  // Variable text capacity is checked before touching either slot.
  // 52 records of 27-byte names and value 0.5 fit; the 53rd exceeds 2048.
  empty();std::vector<std::string> names;names.reserve(53);
  for (int i=0;i<53;++i) {
    char suffix[4];snprintf(suffix,sizeof(suffix),"%03d",i);
    names.push_back("p"+std::string(23,'x')+suffix);
  }
  std::vector<FlightParameter> many;std::vector<double> manyValues(53,.5);
  for (int i=0;i<53;++i) many.emplace_back(i+1,names[i].c_str(),&gain,0,10,"T","T",ParameterStorage::EEPROM);
  assert(saveEEPROMParameters(many.data(),52,manyValues.data()));
  const auto full=EEPROM.bytes;EEPROM.resetFaults();
  assert(!saveEEPROMParameters(many.data(),53,manyValues.data()));
  assert(EEPROM.bytes==full && EEPROM.writes==0);
  std::cout << "EEPROM primary/backup tests passed; " << cutCases << " power-cut cases\n";
}
