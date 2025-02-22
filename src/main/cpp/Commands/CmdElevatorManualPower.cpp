#include "Commands/CmdElevatorManualPower.h"
#include "Robot.h"
#include <iostream>
#include "constants/Constants.h"

#define ELEVATOR_DEADBAND_CONSTANT 0.9

CmdElevatorManualPower::CmdElevatorManualPower(float power) 
{
  AddRequirements(&robotcontainer.m_elevator);
  m_power = power;
  m_manualElevatorEnabled = false;
}

void CmdElevatorManualPower::Initialize() 
{
  std::cout << "Starting Elevator Manual Power" << std::endl;
}

void CmdElevatorManualPower::Execute() 
{
  if((robotcontainer.m_topDriver.GetLeftY() > ELEVATOR_DEADBAND_CONSTANT && !m_manualElevatorEnabled))
  {
    robotcontainer.m_elevator.SetElevatorPower(-ELEV_MANUAL_SLOW_POWER);
    m_manualElevatorEnabled = true;
  }
  else if(robotcontainer.m_topDriver.GetLeftY() < -ELEVATOR_DEADBAND_CONSTANT)
  {
    robotcontainer.m_elevator.SetElevatorPower(ELEV_MANUAL_SLOW_POWER);
    m_manualElevatorEnabled = true;
  }
  else if(m_manualElevatorEnabled)
  {
    robotcontainer.m_elevator.SetElevatorPower(0);
    m_manualElevatorEnabled = false;
  }
}

void CmdElevatorManualPower::End(bool interrupted) 
{
  std::cout << "Ending Elevator Manual Power" << std::endl;
}

bool CmdElevatorManualPower::IsFinished()
{
  return false;
}

