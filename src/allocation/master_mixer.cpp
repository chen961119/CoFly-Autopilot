#include "math/units.h"
#include "control/control_pipeline.h"
#include "control/control_output.h"
#include "allocation/master_mixer.h"
#include "config/aircraft_config.h"
#include "math/math_utils.h"
#include "control/control_modes.h"

static void mixLegacyMaster()
{

  const bool manual = currentFlightMode == MANUAL_MODE;
  const float roll_PID = manual ? manualControlOutput.rollCommand : rollControlOutput.pidOutput;
  // 手动/PID为抬头正的控制意图；INDI已在出口转换为下偏正的混控逻辑量。
  const float elevatorCommand = manual ? -manualControlOutput.pitchCommand :
      (activePitchController == PitchController::INDI ?
       pitchIndiControlOutput.indiOutput :
       - pitchPidControlOutput.pidOutput);
  const float yawCommand = manual ? manualControlOutput.yawCommand : yawRateControlOutput.pidOutput;
  const float throttleCommand = manual ? manualControlOutput.throttleCommand : throttleControlOutput.throttleCommand;
  const float roll_PID_lpf = rollControlOutput.filteredPidOutput;
  const float roll_PID_dot_lpf = rollControlOutput.filteredPidOutputDerivative;
  const float Phiab_PID = configurationControlOutput.pidOutputs[0], Phiac_PID = configurationControlOutput.pidOutputs[1];
  const float Phibd_PID = configurationControlOutput.pidOutputs[2], Phice_PID = configurationControlOutput.pidOutputs[3];
  const float Phidf_PID = configurationControlOutput.pidOutputs[4], Phieg_PID = configurationControlOutput.pidOutputs[5];
  const float error_Phiab = Units::toDegrees(configurationControlOutput.angleErrors[0]), error_Phiac = Units::toDegrees(configurationControlOutput.angleErrors[1]);
  const float error_Phibd = Units::toDegrees(configurationControlOutput.angleErrors[2]), error_Phice = Units::toDegrees(configurationControlOutput.angleErrors[3]);
  const float error_Phidf = Units::toDegrees(configurationControlOutput.angleErrors[4]), error_Phieg = Units::toDegrees(configurationControlOutput.angleErrors[5]);
  // 按单机或编队构型，将姿态控制量分配到各机舵面和油门。
  // 手动模式直接使用归一化遥控指令；舵面归一化中位为 0，油门范围为 0～1。



  if (aircraftIsSingle())
  {
    // 自己飞
    commands.local.aileron1 = roll_PID; // 下偏为正
    commands.local.aileron2 = -roll_PID; // 下偏为正
    commands.local.elevator = elevatorCommand;  // 下偏为正
    commands.local.throttle = throttleCommand; // 油门增大为正
    commands.local.rudder = - yawCommand; // 左偏为正
  }
  else
  {

    /*
    //2机一起飞 BA
    //策略1

    float coeroll=1.0;
    if (mixerStrategyCommand > 0.2f)
    {
    commands.slaves[0].aileron1 = 0.81*roll_PID+0.5*Phiab_PID;
    commands.slaves[0].aileron2 = 0.41*roll_PID-0.5*Phiab_PID;
    commands.slaves[0].throttle = throttleCommand+0.8*yawCommand;
    commands.slaves[0].rudder = - yawCommand;
    //2机、3机


    commands.local.aileron1 = -0.41*roll_PID-0.5*Phiab_PID;
    commands.local.aileron2 = -0.81*roll_PID+0.5*Phiab_PID;
    commands.local.elevator = elevatorCommand;
    commands.local.throttle = throttleCommand-0.8*yawCommand;
    commands.local.rudder = - yawCommand;
    }
    else if(mixerStrategyCommand < 0.2f&&mixerStrategyCommand > -0.2f)
    {

    //策略2
    commands.slaves[0].aileron1 = 0.5*0.81*roll_PID+0.5*Phiab_PID;
    commands.slaves[0].aileron2 = 0.5*0.41*roll_PID-0.5*Phiab_PID;
    commands.slaves[0].throttle = throttleCommand+0.8*yawCommand;
    commands.slaves[0].rudder = - yawCommand;
    //2机、3机


    commands.local.aileron1 = 0.5*-0.41*roll_PID-0.5*Phiab_PID;
    commands.local.aileron2 = 0.5*-0.81*roll_PID+0.5*Phiab_PID;
    commands.local.elevator = elevatorCommand;
    commands.local.throttle = throttleCommand-0.8*yawCommand;
    commands.local.rudder = - yawCommand;

    }
    else
    {
    //策略3
    commands.slaves[0].aileron1 = 0.1*0.81*roll_PID+0.5*Phiab_PID;
    commands.slaves[0].aileron2 = 0.1*0.41*roll_PID-0.5*Phiab_PID;
    commands.slaves[0].throttle = throttleCommand+keeppositive(+1.2*yawCommand)+keeppositive(0.4*roll_PID);
    commands.slaves[0].rudder = - yawCommand;
    //2机、3机


    commands.local.aileron1 = 0.1*-0.41*roll_PID-0.5*Phiab_PID;
    commands.local.aileron2 = 0.1*-0.81*roll_PID+0.5*Phiab_PID;
    commands.local.elevator = elevatorCommand;
    commands.local.throttle = throttleCommand+keeppositive(-1.2*yawCommand)+keeppositive(-0.4*roll_PID);
    commands.local.rudder = - yawCommand;


    }

    */
    // USBSerial.println(commands.local.aileron1);

    // 3机一起飞 BAC
    if ((aircraftCount() == 3))
    {

      float coeab, coeac, coeroll;
      if (abs(error_Phiab) > 20)
      {
        coeab = abs(error_Phiab) / 20.0;
      }
      else
      {
        coeab = 0.8;
      }
      if (abs(error_Phiac) > 20)
      {
        coeac = abs(error_Phiac) / 20.0;
      }
      else
      {
        coeac = 0.8;
      }

      // coeac = 1.0;

      // 策略1
      if (mixerStrategyCommand > 0.2f)
      {
        commands.slaves[0].aileron1 =
            1.016 * roll_PID + coeab * 1 * Phiab_PID - coeac * 0.68 * Phiac_PID;
        commands.slaves[0].aileron2 = 
            0.593 * roll_PID - coeac * 0.12 * Phiac_PID - coeab * 0.34 * Phiab_PID;
        commands.slaves[0].throttle = throttleCommand + 0.8 * yawCommand;
        commands.slaves[0].rudder = - yawCommand;
        
        // 2机、3机
        commands.slaves[1].aileron1 =
            -0.593 * roll_PID + coeab * 0.12 * Phiab_PID + coeac * 0.34 * Phiac_PID;
        commands.slaves[1].aileron2 =
            -1.016 * roll_PID - coeac * 1 * Phiac_PID + coeab * 0.68 * Phiab_PID ;
        commands.slaves[1].throttle = throttleCommand - 0.8 * yawCommand;
        commands.slaves[1].rudder = - yawCommand;
        
        commands.local.aileron1 =
            0.1 * roll_PID - coeab * 1.186 * Phiab_PID + coeac * 0.71 * Phiac_PID;
        commands.local.aileron2 =
            -0.1 * roll_PID + coeac * 1.186 * Phiac_PID- coeab * 0.71 * Phiab_PID ;
        commands.local.elevator = elevatorCommand;
        commands.local.throttle = throttleCommand;
        commands.local.rudder = - yawCommand;

        
         // 2机、3机
      }
      // 策略2

      else if (mixerStrategyCommand < 0.2f && mixerStrategyCommand > -0.2f)
      {
        coeroll = 0.2;
        commands.slaves[0].aileron1 = 1.016 * coeroll * roll_PID + coeab * 0.864 * Phiab_PID -
                        coeac * 0.62 * Phiac_PID;
        commands.slaves[0].aileron2 = 0.593 * coeroll * roll_PID - coeab * 0.27 * Phiab_PID -
                        coeac * 0.08 * Phiac_PID;
        commands.slaves[0].throttle =
            throttleCommand + keeppositive(+1.2 * yawCommand) + keeppositive(0.4 * roll_PID);
        commands.slaves[0].rudder = - yawCommand;
         // 2机、3机
        commands.slaves[0].elevator = 0.0;

        commands.local.aileron1 =
            0.1 * roll_PID - coeab * 1.0 * Phiab_PID + coeac * 0.65 * Phiac_PID;
        commands.local.aileron2 =
            -0.1 * roll_PID - coeab * 0.65 * Phiab_PID + coeac * 1.0 * Phiac_PID;
        commands.local.elevator = elevatorCommand;
        commands.local.throttle = throttleCommand;
        commands.local.rudder = - yawCommand;

        commands.slaves[1].aileron1 = -0.593 * coeroll * roll_PID + coeab * 0.08 * Phiab_PID +
                        coeac * 0.27 * Phiac_PID;
        commands.slaves[1].aileron2 = -1.016 * coeroll * roll_PID + coeab * 0.62 * Phiab_PID -
                        coeac * 0.864 * Phiac_PID;
        commands.slaves[1].throttle =
            throttleCommand + keeppositive(-1.2 * yawCommand) + keeppositive(-0.4 * roll_PID);
        commands.slaves[1].rudder = - yawCommand;
         // 2机、3机
        commands.slaves[1].elevator = 0.0;
      }
      else
      {
        coeroll = 0.2;

        commands.slaves[0].aileron1 = 1.016 * coeroll * roll_PID + coeab * 0.864 * Phiab_PID -
                        coeac * 0.62 * Phiac_PID;
        commands.slaves[0].aileron2 = 0.593 * coeroll * roll_PID - coeab * 0.27 * Phiab_PID -
                        coeac * 0.08 * Phiac_PID;
        commands.slaves[0].throttle =
            throttleCommand + keeppositive(+1.2 * yawCommand) + keeppositive(0.4 * roll_PID);
        commands.slaves[0].rudder = - yawCommand;
         // 2机、3机
        // 升降舵下偏为正：正滚转控制量时 B 上偏、C 下偏，不含安装方向。
        commands.slaves[0].elevator =
            (-10.0 * 20.0 * roll_PID_lpf - 10.0 * roll_PID_dot_lpf) / 1000.0f;

        commands.local.aileron1 =
            0.1 * roll_PID - coeab * 1.0 * Phiab_PID + coeac * 0.65 * Phiac_PID;
        commands.local.aileron2 =
            -0.1 * roll_PID - coeab * 0.65 * Phiab_PID + coeac * 1.0 * Phiac_PID;
        commands.local.elevator = elevatorCommand;
        commands.local.throttle = throttleCommand;
        commands.local.rudder = - yawCommand;

        commands.slaves[1].aileron1 = -0.593 * coeroll * roll_PID + coeab * 0.08 * Phiab_PID +
                        coeac * 0.27 * Phiac_PID;
        commands.slaves[1].aileron2 = -1.016 * coeroll * roll_PID + coeab * 0.62 * Phiab_PID -
                        coeac * 0.864 * Phiac_PID;
        commands.slaves[1].throttle =
            throttleCommand + keeppositive(-1.2 * yawCommand) + keeppositive(-0.4 * roll_PID);
        commands.slaves[1].rudder = - yawCommand;
         // 2机、3机 增加了相对滚转到升降舵
        commands.slaves[1].elevator =
            (10.0 * 20.0 * roll_PID_lpf + 10.0 * roll_PID_dot_lpf) / 1000.0f;
      }
    }
    else if ((aircraftCount() == 4))
    {

      // 4机一起飞 DBAC
      float coeab, coeac, coebd,
          coeroll; // bc的系数和de的系数不同 de的系数是ab的0.66倍

      coeroll = 1.5;

      // USBSerial.println(coeroll);
      if (abs(error_Phiab) > 20)
      {
        coeab = 1.2 * abs(error_Phiab) / 20.0;
      }
      else
      {
        coeab = 1.2;
      }

      if (abs(error_Phiac) > 20)
      {
        coeac = 1.0 * abs(error_Phiac) / 20.0;
      }
      else
      {
        coeac = 1.0;
      }

      if (abs(error_Phibd) > 20)
      {
        coebd = 1.0 * abs(error_Phibd) / 20.0;
      }
      else
      {
        coebd = 1.0;
      }

      // 策略1
      if (mixerStrategyCommand > 0.2f)
      {
        commands.slaves[2].aileron1 = 1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                        0.53 * coeac * Phiac_PID + 1.0 * coebd * Phibd_PID;
        commands.slaves[2].aileron2 = 0.74 * coeroll * roll_PID + 0.325 * coeab * Phiab_PID -
                        0.26 * coeac * Phiac_PID - 0.1 * coebd * Phibd_PID;
        commands.slaves[2].throttle = throttleCommand + 1.2 * yawCommand;
        commands.slaves[2].rudder = - yawCommand;


        commands.slaves[0].aileron1 = 0.28 * coeroll * roll_PID - 0.667 * coeab * Phiab_PID +
                        0.19 * coeac * Phiac_PID - 1.0 * coebd * Phibd_PID;
        commands.slaves[0].aileron2 = 0.13 * coeroll * roll_PID - 1.0 * coeab * Phiab_PID +
                        0.39 * coeac * Phiac_PID - 0.736 * coebd * Phibd_PID;
        commands.slaves[0].throttle = throttleCommand + 0.6 * yawCommand;
        commands.slaves[0].rudder = - yawCommand;


        commands.local.aileron1 = -0.13 * coeroll * roll_PID - 1.0 * coeab * Phiab_PID +
                        0.736 * coeac * Phiac_PID - 0.39 * coebd * Phibd_PID;
        commands.local.aileron2 = -0.28 * coeroll * roll_PID - 0.667 * coeab * Phiab_PID +
                        1.0 * coeac * Phiac_PID - 0.19 * coebd * Phibd_PID;
        commands.local.elevator = elevatorCommand;
        commands.local.throttle = throttleCommand - 0.6 * yawCommand;
        ;
        commands.local.rudder = - yawCommand;

        commands.slaves[1].aileron1 = -0.74 * coeroll * roll_PID + 0.325 * coeab * Phiab_PID +
                        0.1 * coeac * Phiac_PID + 0.26 * coebd * Phibd_PID;
        commands.slaves[1].aileron2 = -1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                        1.0 * coeac * Phiac_PID + 0.53 * coebd * Phibd_PID;
        commands.slaves[1].throttle = throttleCommand - 1.2 * yawCommand;
        commands.slaves[1].rudder = - yawCommand;

      }

      // 策略2

      else if (mixerStrategyCommand < 0.2f && mixerStrategyCommand > -0.2f)
      {
        coeroll = 0.3;
        commands.slaves[2].aileron1 = 1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                        0.53 * coeac * Phiac_PID + 1.0 * coebd * Phibd_PID;
        commands.slaves[2].aileron2 = 0.74 * coeroll * roll_PID + 0.325 * coeab * Phiab_PID -
                        0.26 * coeac * Phiac_PID - 0.1 * coebd * Phibd_PID;
        commands.slaves[2].throttle =
            throttleCommand + keeppositive(+1.5 * yawCommand) + keeppositive(0.5 * roll_PID);
        commands.slaves[2].rudder = - yawCommand;


        commands.slaves[0].aileron1 = 0.28 * coeroll * roll_PID - 0.667 * coeab * Phiab_PID +
                        0.19 * coeac * Phiac_PID - 1.0 * coebd * Phibd_PID;
        commands.slaves[0].aileron2 = 0.1 * coeroll * roll_PID - 1.0 * coeab * Phiab_PID +
                        0.39 * coeac * Phiac_PID - 0.736 * coebd * Phibd_PID;
        commands.slaves[0].throttle =
            throttleCommand + keeppositive(+1.0 * yawCommand) + keeppositive(0.3 * roll_PID);
        commands.slaves[0].rudder = - yawCommand;


        commands.local.aileron1 = -0.1 * coeroll * roll_PID - 1.0 * coeab * Phiab_PID +
                        0.736 * coeac * Phiac_PID - 0.39 * coebd * Phibd_PID;
        commands.local.aileron2 = -0.28 * coeroll * roll_PID - 0.667 * coeab * Phiab_PID +
                        1.0 * coeac * Phiac_PID - 0.19 * coebd * Phibd_PID;
        commands.local.elevator = elevatorCommand;
        commands.local.throttle =
            throttleCommand + keeppositive(-1.0 * yawCommand) + keeppositive(-0.3 * roll_PID);
        commands.local.rudder = - yawCommand;

        commands.slaves[1].aileron1 = -0.74 * coeroll * roll_PID + 0.325 * coeab * Phiab_PID +
                        0.1 * coeac * Phiac_PID + 0.26 * coebd * Phibd_PID;
        commands.slaves[1].aileron2 = -1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                        1.0 * coeac * Phiac_PID + 0.53 * coebd * Phibd_PID;
        commands.slaves[1].throttle =
            throttleCommand + keeppositive(-1.5 * yawCommand) + keeppositive(-0.5 * roll_PID);
        commands.slaves[1].rudder = - yawCommand;

      }
      else
      {

        coeroll = 0.2;
        commands.slaves[2].aileron1 = 1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                        0.53 * coeac * Phiac_PID + 1.0 * coebd * Phibd_PID;
        commands.slaves[2].aileron2 = 0.74 * coeroll * roll_PID + 0.325 * coeab * Phiab_PID -
                        0.26 * coeac * Phiac_PID - 0.1 * coebd * Phibd_PID;
        commands.slaves[2].throttle =
            throttleCommand + keeppositive(+1.5 * yawCommand) + keeppositive(0.5 * roll_PID);
        commands.slaves[2].rudder = - yawCommand;


        commands.slaves[0].aileron1 = 0.28 * coeroll * roll_PID - 0.667 * coeab * Phiab_PID +
                        0.19 * coeac * Phiac_PID - 1.0 * coebd * Phibd_PID;
        commands.slaves[0].aileron2 = 0.1 * coeroll * roll_PID - 1.0 * coeab * Phiab_PID +
                        0.39 * coeac * Phiac_PID - 0.736 * coebd * Phibd_PID;
        commands.slaves[0].throttle =
            throttleCommand + keeppositive(+1.0 * yawCommand) + keeppositive(0.3 * roll_PID);
        commands.slaves[0].rudder = - yawCommand;


        commands.local.aileron1 = -0.1 * coeroll * roll_PID - 1.0 * coeab * Phiab_PID +
                        0.736 * coeac * Phiac_PID - 0.39 * coebd * Phibd_PID;
        commands.local.aileron2 = -0.28 * coeroll * roll_PID - 0.667 * coeab * Phiab_PID +
                        1.0 * coeac * Phiac_PID - 0.19 * coebd * Phibd_PID;
        commands.local.elevator = elevatorCommand;
        commands.local.throttle =
            throttleCommand + keeppositive(-1.0 * yawCommand) + keeppositive(-0.3 * roll_PID);
        commands.local.rudder = - yawCommand;

        commands.slaves[1].aileron1 = -0.74 * coeroll * roll_PID + 0.325 * coeab * Phiab_PID +
                        0.1 * coeac * Phiac_PID + 0.26 * coebd * Phibd_PID;
        commands.slaves[1].aileron2 = -1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                        1.0 * coeac * Phiac_PID + 0.53 * coebd * Phibd_PID;
        commands.slaves[1].throttle =
            throttleCommand + keeppositive(-1.5 * yawCommand) + keeppositive(-0.5 * roll_PID);
        commands.slaves[1].rudder = - yawCommand;

      }
    }
    else if ((aircraftCount() == 5))
    {

      // 5机一起飞DBACE
      float coeab, coeac, coebd, coece,
          coeroll; // bc的系数和de的系数不同 de的系数是ab的0.66倍
      if (abs(error_Phiab) > 20)
      {
        coeab = abs(error_Phiab) / 20.0;
      }
      else
      {
        coeab = 1.0;
      }

      if (abs(error_Phiac) > 20)
      {
        coeac = abs(error_Phiac) / 20.0;
      }
      else
      {
        coeac = 1.0;
      }

      if (abs(error_Phibd) > 20)
      {
        coebd = abs(error_Phibd) / 20.0;
      }
      else
      {
        coebd = 0.9;
      }

      if (abs(error_Phice) > 20)
      {
        coece = abs(error_Phice) / 20.0;
      }
      else
      {
        coece = 0.9;
      }

      // 策略1
      coeroll = 1.2;
      commands.slaves[2].aileron1 = 1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                      0.72 * coeac * Phiac_PID + 1.0 * coebd * Phibd_PID -
                      0.38 * coece * Phice_PID;
      commands.slaves[2].aileron2 = 0.75 * coeroll * roll_PID + 0.44 * coeab * Phiab_PID -
                      0.42 * coeac * Phiac_PID + 0.06 * coebd * Phibd_PID -
                      0.23 * coece * Phice_PID;
      commands.slaves[2].throttle = throttleCommand + 1.0 * yawCommand;
      commands.slaves[2].rudder = - yawCommand;


      commands.slaves[0].aileron1 = 0.417 * coeroll * roll_PID - 0.36 * coeab * Phiab_PID +
                      0.05 * coeac * Phiac_PID - 0.7 * coebd * Phibd_PID -
                      0.01 * coece * Phice_PID;
      commands.slaves[0].aileron2 = 0.33 * coeroll * roll_PID - 0.76 * coeab * Phiab_PID +
                      0.31 * coeac * Phiac_PID - 0.6 * coebd * Phibd_PID +
                      0.08 * coece * Phice_PID;
      commands.slaves[0].throttle = throttleCommand + 0.8 * yawCommand;
      commands.slaves[0].rudder = - yawCommand;


      commands.local.aileron1 = 0.15 * coeroll * roll_PID - 0.88 * coeab * Phiab_PID +
                      0.66 * coeac * Phiac_PID - 0.42 * coebd * Phibd_PID +
                      0.26 * coece * Phice_PID;
      commands.local.aileron2 = -0.15 * coeroll * roll_PID - 0.66 * coeab * Phiab_PID +
                      0.88 * coeac * Phiac_PID - 0.27 * coebd * Phibd_PID +
                      0.38 * coece * Phice_PID;
      commands.local.elevator = elevatorCommand;
      commands.local.throttle = throttleCommand;
      commands.local.rudder = - yawCommand;

      commands.slaves[1].aileron1 = -0.33 * coeroll * roll_PID - 0.31 * coeab * Phiab_PID +
                      0.76 * coeac * Phiac_PID - 0.08 * coebd * Phibd_PID +
                      0.6 * coece * Phice_PID;
      commands.slaves[1].aileron2 = -0.417 * coeroll * roll_PID - 0.05 * coeab * Phiab_PID +
                      0.36 * coeac * Phiac_PID - 0.01 * coebd * Phibd_PID +
                      0.7 * coece * Phice_PID;
      commands.slaves[1].throttle = throttleCommand - 0.8 * yawCommand;
      commands.slaves[1].rudder = - yawCommand;


      commands.slaves[3].aileron1 = -0.75 * coeroll * roll_PID + 0.42 * coeab * Phiab_PID -
                      coeac * 0.44 * Phiac_PID + 0.23 * coebd * Phibd_PID -
                      0.06 * coece * Phice_PID;
      commands.slaves[3].aileron2 = -1.0 * coeroll * roll_PID + 0.72 * coeab * Phiab_PID -
                      1.0 * coeac * Phiac_PID + 0.38 * coebd * Phibd_PID -
                      1.0 * coece * Phice_PID;
      commands.slaves[3].throttle = throttleCommand - 1.0 * yawCommand;
      commands.slaves[3].rudder = - yawCommand;


      /*
       else if (mixerStrategyCommand < 0.2f && mixerStrategyCommand > -0.2f) {
         //策略2
         coeroll = 0.5;
         commands.slaves[2].aileron1 = 1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID - 0.72 *
       coeac * Phiac_PID + 1.0 * coebd * Phibd_PID - 0.38 * coece * Phice_PID;
         commands.slaves[2].aileron2 = 0.75 * coeroll * roll_PID + 0.44 * coeab * Phiab_PID - 0.42
       * coeac * Phiac_PID + 0.06 * coebd * Phibd_PID - 0.23 * coece * Phice_PID;
         commands.slaves[2].throttle = throttleCommand + keeppositive(+1.8 * yawCommand) + keeppositive(0.7 *
       roll_PID); commands.slaves[2].rudder = - yawCommand;

         commands.slaves[0].aileron1 = 0.417 * coeroll * roll_PID - 0.36 * coeab * Phiab_PID + 0.05
       * coeac * Phiac_PID - 0.7 * coebd * Phibd_PID - 0.01 * coece * Phice_PID;
         commands.slaves[0].aileron2 = 0.33 * coeroll * roll_PID - 0.76 * coeab * Phiab_PID + 0.31
       * coeac * Phiac_PID - 0.6 * coebd * Phibd_PID + 0.08 * coece * Phice_PID;
         commands.slaves[0].throttle = throttleCommand + keeppositive(+1.0 * yawCommand) + keeppositive(0.4 *
       roll_PID); commands.slaves[0].rudder = - yawCommand;

         commands.local.aileron1 = 0.15 * coeroll * roll_PID - 0.88 * coeab * Phiab_PID + 0.66
       * coeac * Phiac_PID - 0.42 * coebd * Phibd_PID + 0.27 * coece * Phice_PID;
         commands.local.aileron2 = -0.15 * coeroll * roll_PID - 0.66 * coeab * Phiab_PID + 0.88
       * coeac * Phiac_PID - 0.27 * coebd * Phibd_PID + 0.42 * coece * Phice_PID;
         commands.local.elevator = elevatorCommand;
         commands.local.throttle = throttleCommand;
         commands.local.rudder = - yawCommand;

         commands.slaves[1].aileron1 = -0.33 * coeroll * roll_PID - 0.31 * coeab * Phiab_PID + 0.76
       * coeac * Phiac_PID - 0.08 * coebd * Phibd_PID + 0.6 * coece * Phice_PID;
         commands.slaves[1].aileron2 = -0.417 * coeroll * roll_PID - 0.05 * coeab * Phiab_PID +
       0.36 * coeac * Phiac_PID - 0.01 * coebd * Phibd_PID + 0.7 * coece *
       Phice_PID; commands.slaves[1].throttle = throttleCommand + keeppositive(-1.0 * yawCommand) +
       keeppositive(-0.4 * roll_PID); commands.slaves[1].rudder = - yawCommand;

         commands.slaves[3].aileron1 = -0.75 * coeroll * roll_PID + 0.42 * coeab * Phiab_PID -
       coeac * 0.44 * Phiac_PID + 0.23 * coebd * Phibd_PID - 0.06 * coece *
       Phice_PID; commands.slaves[3].aileron2 = -1.0 * coeroll * roll_PID + 0.72 * coeab *
       Phiab_PID - 1.0 * coeac * Phiac_PID + 0.38 * coebd * Phibd_PID - 1.0 * coece
       * Phice_PID; commands.slaves[3].throttle = throttleCommand + keeppositive(-1.8 * yawCommand) +
       keeppositive(-0.7 * roll_PID); commands.slaves[3].rudder = - yawCommand;
       }

       else {
         //USBSerial.println("ss");
         //策略3 实时控制分配
         //期望虚拟控制量
         dw4 = roll_PID * 40.0;
         dw6 = yawCommand * 20.0;
         dw7 = Phiab_PID * 130.0;
         dw10 = Phiac_PID * 130.0;
         dw13 = Phibd_PID * 110.0;
         dw16 = Phice_PID * 110.0;


         dw_att << dw4,
           dw6,
           0.0,
           0.0,
           0.0,
           0.0;

         dw_config << dw7,
           dw10,
           dw13,
           dw16;

         de_config = Bplusminismall_pinv * dw_config;
         de_att = Bplusfullsmall_pinv * dw_att;
         //printFullMatrix(de_config);
         commands.slaves[2].aileron1 = de_att(0) + de_config(0);
         commands.slaves[2].aileron2 = de_att(1) + de_config(1);
         commands.slaves[2].throttle = throttleCommand + keeppositive(+1.4 * yawCommand) + keeppositive(0.5 *
       roll_PID); commands.slaves[2].rudder = - yawCommand;

         commands.slaves[0].aileron1 = de_att(2) + de_config(2);
         commands.slaves[0].aileron2 = de_att(3) + de_config(3);
         commands.slaves[0].throttle = throttleCommand + keeppositive(+0.8 * yawCommand) + keeppositive(0.3 *
       roll_PID); commands.slaves[0].rudder = - yawCommand;

         commands.local.aileron1 = de_att(4) + de_config(4);
         commands.local.aileron2 = de_att(5) + de_config(5);
         commands.local.elevator = elevatorCommand;
         commands.local.throttle = throttleCommand;
         commands.local.rudder = - yawCommand;

         commands.slaves[1].aileron1 = de_att(6) + de_config(6);
         commands.slaves[1].aileron2 = de_att(7) + de_config(7);
         commands.slaves[1].throttle = throttleCommand + keeppositive(-0.8 * yawCommand) + keeppositive(-0.3 *
       roll_PID); commands.slaves[1].rudder = - yawCommand;

         commands.slaves[3].aileron1 = de_att(8) + de_config(8);
         commands.slaves[3].aileron2 = de_att(9) + de_config(9);
         commands.slaves[3].throttle = throttleCommand + keeppositive(-1.4 * yawCommand) + keeppositive(-0.5 *
       roll_PID); commands.slaves[3].rudder = - yawCommand;
       }
         */
    }
    else if ((aircraftCount() == 7))
    {

      // 7机一起飞 FDBACEG
      float coeab, coeac, coebd, coece, coedf, coeeg,
          coeroll; // bc的系数和de的系数不同 de的系数是ab的0.66倍
      if (abs(error_Phiab) > 20)
      {
        coeab = abs(error_Phiab) / 20.0;
      }
      else
      {
        coeab = 1.0;
      }

      if (abs(error_Phiac) > 20)
      {
        coeac = abs(error_Phiac) / 20.0;
      }
      else
      {
        coeac = 1.0;
      }

      if (abs(error_Phibd) > 20)
      {
        coebd = abs(error_Phibd) / 20.0;
      }
      else
      {
        coebd = 0.9;
      }

      if (abs(error_Phice) > 20)
      {
        coece = abs(error_Phice) / 20.0;
      }
      else
      {
        coece = 0.9;
      }

      if (abs(error_Phidf) > 20)
      {
        coedf = abs(error_Phidf) / 20.0;
      }
      else
      {
        coedf = 0.6;
      }

      if (abs(error_Phieg) > 20)
      {
        coeeg = abs(error_Phieg) / 20.0;
      }
      else
      {
        coeeg = 0.6;
      }

      // 策略1
      coeroll = 1.8;

      commands.slaves[4].aileron1 = 1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                      0.77 * coeac * Phiac_PID + 1.0 * coebd * Phibd_PID -
                      0.44 * coece * Phice_PID + 1.0 * coedf * Phidf_PID -
                      0.24 * coeeg * Phieg_PID;
      commands.slaves[4].aileron2 = 0.87 * coeroll * roll_PID + 0.69 * coeab * Phiab_PID -
                      0.56 * coeac * Phiac_PID + 0.54 * coebd * Phibd_PID -
                      0.33 * coece * Phice_PID + 0.19 * coedf * Phidf_PID -
                      0.18 * coeeg * Phieg_PID;
      commands.slaves[4].throttle = 0.85 * throttleCommand + 1.5 * yawCommand;
      commands.slaves[4].rudder = - yawCommand;


      commands.slaves[2].aileron1 = 0.72 * coeroll * roll_PID + 0.31 * coeab * Phiab_PID -
                      0.32 * coeac * Phiac_PID + 0.06 * coebd * Phibd_PID -
                      0.21 * coece * Phice_PID - 0.27 * coedf * Phidf_PID -
                      0.12 * coeeg * Phieg_PID;
      commands.slaves[2].aileron2 = 0.55 * coeroll * roll_PID - 0.12 * coeab * Phiab_PID -
                      0.06 * coeac * Phiac_PID - 0.44 * coebd * Phibd_PID -
                      0.07 * coece * Phice_PID - 0.41 * coedf * Phidf_PID -
                      0.05 * coeeg * Phieg_PID;
      commands.slaves[2].throttle = 0.92 * throttleCommand + 1.0 * yawCommand;
      commands.slaves[2].rudder = - yawCommand;


      commands.slaves[0].aileron1 = 0.4 * coeroll * roll_PID - 0.50 * coeab * Phiab_PID +
                      0.2 * coeac * Phiac_PID - 0.65 * coebd * Phibd_PID +
                      0.05 * coece * Phice_PID - 0.41 * coedf * Phidf_PID +
                      0.01 * coeeg * Phieg_PID;
      commands.slaves[0].aileron2 = 0.23 * coeroll * roll_PID - 0.83 * coeab * Phiab_PID +
                      0.47 * coeac * Phiac_PID - 0.57 * coebd * Phibd_PID +
                      0.17 * coece * Phice_PID - 0.27 * coedf * Phidf_PID +
                      0.08 * coeeg * Phieg_PID;
      commands.slaves[0].throttle = throttleCommand + 0.8 * yawCommand;
      commands.slaves[0].rudder = - yawCommand;


      commands.local.aileron1 = 0.1 * coeroll * roll_PID - 0.89 * coeab * Phiab_PID +
                      0.70 * coeac * Phiac_PID - 0.46 * coebd * Phibd_PID +
                      0.31 * coece * Phice_PID - 0.18 * coedf * Phidf_PID +
                      0.14 * coeeg * Phieg_PID;
      commands.local.aileron2 = -0.1 * coeroll * roll_PID - 0.70 * coeab * Phiab_PID +
                      0.89 * coeac * Phiac_PID - 0.31 * coebd * Phibd_PID +
                      0.46 * coece * Phice_PID - 0.14 * coedf * Phidf_PID +
                      0.18 * coeeg * Phieg_PID;
      commands.local.elevator = elevatorCommand;
      commands.local.throttle = throttleCommand;
      commands.local.rudder = - yawCommand;

      commands.slaves[1].aileron1 = -0.23 * coeroll * roll_PID - 0.47 * coeab * Phiab_PID +
                      0.83 * coeac * Phiac_PID - 0.17 * coebd * Phibd_PID +
                      0.57 * coece * Phice_PID - 0.08 * coedf * Phidf_PID +
                      0.27 * coeeg * Phieg_PID;
      commands.slaves[1].aileron2 = -0.4 * coeroll * roll_PID - 0.2 * coeab * Phiab_PID +
                      0.50 * coeac * Phiac_PID - 0.05 * coebd * Phibd_PID +
                      0.65 * coece * Phice_PID - 0.01 * coedf * Phidf_PID +
                      0.41 * coeeg * Phieg_PID;
      commands.slaves[1].throttle = throttleCommand - 0.8 * yawCommand;
      commands.slaves[1].rudder = - yawCommand;


      commands.slaves[3].aileron1 = -0.55 * coeroll * roll_PID + 0.06 * coeab * Phiab_PID +
                      coeac * 0.12 * Phiac_PID + 0.07 * coebd * Phibd_PID +
                      0.44 * coece * Phice_PID + 0.05 * coedf * Phidf_PID +
                      0.41 * coeeg * Phieg_PID;
      commands.slaves[3].aileron2 = -0.72 * coeroll * roll_PID + 0.32 * coeab * Phiab_PID -
                      0.31 * coeac * Phiac_PID + 0.21 * coebd * Phibd_PID -
                      0.06 * coece * Phice_PID + 0.12 * coedf * Phidf_PID +
                      0.27 * coeeg * Phieg_PID;
      commands.slaves[3].throttle = 0.92 * throttleCommand - 1.0 * yawCommand;
      commands.slaves[3].rudder = - yawCommand;


      commands.slaves[5].aileron1 = -0.87 * coeroll * roll_PID + 0.56 * coeab * Phiab_PID -
                      coeac * 0.69 * Phiac_PID + 0.33 * coebd * Phibd_PID -
                      0.54 * coece * Phice_PID + 0.18 * coedf * Phidf_PID -
                      0.19 * coeeg * Phieg_PID;
      commands.slaves[5].aileron2 = -1.0 * coeroll * roll_PID + 0.77 * coeab * Phiab_PID -
                      1.0 * coeac * Phiac_PID + 0.44 * coebd * Phibd_PID -
                      1.0 * coece * Phice_PID + 0.24 * coedf * Phidf_PID -
                      1.0 * coeeg * Phieg_PID;
      commands.slaves[5].throttle = 0.85 * throttleCommand - 1.5 * yawCommand;
      commands.slaves[5].rudder = - yawCommand;

    }
  }

}

void mixMasterCommands() {
  ActuatorCommand result;

  mixLegacyMaster();
  const float brakeOffset = ::brakeEnabled ? 0.1f : 0.0f;
  result.aileron1 = commands.local.aileron1 * 0.4f + brakeOffset;
  result.aileron2 = commands.local.aileron2 * 0.4f + brakeOffset;
  result.throttle = commands.local.throttle;
  result.rudder = commands.local.rudder;
  result.elevator = commands.local.elevator;
  // 主机交联属于主分配；输出转换层不再叠加。
  const float compensation = -result.elevator / 15.0f;
  result.aileron1 += compensation + rollDampingControlOutput.dampingCommand;
  result.aileron2 += compensation - rollDampingControlOutput.dampingCommand;

  commands.local =  result;
}

