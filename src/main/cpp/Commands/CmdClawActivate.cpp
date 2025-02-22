#include <iostream>

#include "commands/CmdClawActivate.h"
#include "Robot.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>  


CmdClawActivate::CmdClawActivate(double power) 
{
  m_power = power;
  AddRequirements(&robotcontainer.m_claw);
}


void CmdClawActivate::Initialize() 
{
  std::cout << "Initalize CmdClawActivated" << std::endl;

  currentState = ClawState::RunClawFull;
}


void CmdClawActivate::Execute() 
{
  switch(currentState)
  {
    case ClawState::RunClawFull:
      robotcontainer.m_claw.SetClawPower(m_power);
      currentState = ClawState::Sensor1;
      break;

    case ClawState::Sensor1:
      if(robotcontainer.m_claw.GetClawPhotoEyeFirst())
      {
        currentState = ClawState::RunClawCreep;
      }
      break;

    case ClawState::RunClawCreep:
      robotcontainer.m_claw.SetClawPower(0.2);
      currentState = ClawState::NotDetected;
      break;

    case ClawState::NotDetected:
      if(!robotcontainer.m_claw.GetClawPhotoEyeFirst())
      {
        currentState = ClawState::StopMotor;
      }
      break;

    case ClawState::StopMotor:
      robotcontainer.m_claw.StopClawPower(0);
      robotcontainer.m_claw.isCoralReady = true;
      currentState = ClawState::EndState;
      break;

    case ClawState::EndState:
      break;
  }

  // robotcontainer.m_claw.SetClawPower(m_power);
}

void CmdClawActivate::End(bool interrupted) 
{
  std::cout << "End CmdClawActivated" << std::endl;
  
  robotcontainer.m_claw.StopClawPower(0);
}

bool CmdClawActivate::IsFinished() 
{
  return false;
}
