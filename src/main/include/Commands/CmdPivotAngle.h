#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class CmdPivotAngle
    : public frc2::CommandHelper<frc2::Command, CmdPivotAngle> {
 public:
 
  CmdPivotAngle(double power , float angle) ;

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  double m_power;
  double m_angle;
};
