#include "Commands/CmdElevatorPosition.h"
#include "Robot.h"
#include <iostream>

CmdElevatorPosition::CmdElevatorPosition(float position) 
{
  AddRequirements(&robotcontainer.m_elevator);
  m_position = position; 
}

void CmdElevatorPosition::Initialize() 
{
  std::cout << "Starting Elevator Position" << std::endl; 
}

void CmdElevatorPosition::Execute() 
{
  robotcontainer.m_elevator.SetTargetPosition(m_position);
  std::cout << "Executing Elevator Position" << std::endl;
}

void CmdElevatorPosition::End(bool interrupted) 
{
  std::cout << "Ending Elevator Position" << std::endl;
}


bool CmdElevatorPosition::IsFinished()
{
  if((robotcontainer.m_elevator.GetElevatorPosition() + ELEV_TOLERANCE) > m_position && (robotcontainer.m_elevator.GetElevatorPosition() - ELEV_TOLERANCE) < m_position)
  {
    return true;
  }
  else
  {
    return false;
  }
}


