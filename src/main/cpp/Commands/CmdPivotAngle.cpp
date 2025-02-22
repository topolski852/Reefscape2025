#include "commands/CmdPivotAngle.h"

#include <iostream>
#include "Robot.h"

CmdPivotAngle::CmdPivotAngle(double power , float angle) 
{
  m_power = power;
  m_angle = angle;
  AddRequirements(&robotcontainer.m_claw);
}

void CmdPivotAngle::Initialize() 
{
  std::cout << "Initalize CmdPivotAngle" << std::endl;
}

void CmdPivotAngle::Execute() 
{
  robotcontainer.m_claw.SetPivotPower(m_power);
  robotcontainer.m_claw.SetPivotHold(m_angle); 
}

void CmdPivotAngle::End(bool interrupted) 
{
  std::cout << "End CmdPivotAngle" << std::endl;
}

// Returns true when the command should end.
bool CmdPivotAngle::IsFinished() 
{
  robotcontainer.m_claw.SetPivotAngle(0);
  if((robotcontainer.m_claw.GetPivotAngle() + PIVOT_TOLERANCE) > m_angle && (robotcontainer.m_claw.GetPivotAngle() - PIVOT_TOLERANCE) < m_angle)
  {
    return true;
  }
  else
  {
    return false;
  }
 
}
