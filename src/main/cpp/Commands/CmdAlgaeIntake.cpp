#include <iostream>
#include "commands/CmdAlgaeIntake.h"
#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>  
#include "Robot.h"

CmdAlgaeIntake::CmdAlgaeIntake(double power) 
{
  m_power = power;
  AddRequirements(&robotcontainer.m_claw);
  
}

void CmdAlgaeIntake::Initialize() 
{
  std::cout << "Initalize CmdAlgaeIntake" << std::endl;
  m_timer.Reset(); //Resets the timer
  m_timer.Start(); //Starts the timer
}

void CmdAlgaeIntake::Execute() 
{
  if(robotcontainer.m_claw.GetAlgaePhotoEye())
  {
    robotcontainer.m_claw.SetAlgaePower(-0.2);
  }
  else
  {
    robotcontainer.m_claw.SetAlgaePower(-0.9);
  }
}

void CmdAlgaeIntake::End(bool interrupted) 
{
  std::cout << "End CmdAlgaeIntake" << std::endl;
  m_timer.Stop();
  robotcontainer.m_claw.SetAlgaePower(0.0);
}


bool CmdAlgaeIntake::IsFinished() 
{
  const units::second_t timeout = units::second_t(2.0);
  if(robotcontainer.m_claw.GetAlgaePhotoEye())
  {
    //robotcontainer.m_claw.SetAlgaePower(-0.2);
    std::cout << "Creep Power" << std::endl;
    return true;
  }
  // else if(m_timer.Get() >= timeout && !robotcontainer.m_claw.GetAlgaePhotoEye())
  // {
  //   std::cout << "End Command" << std::endl;
  //   return true;
  // }
  else 
  {
    return false;
  }
}
