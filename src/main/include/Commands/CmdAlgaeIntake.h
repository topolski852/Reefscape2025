#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

class CmdAlgaeIntake
    : public frc2::CommandHelper<frc2::Command, CmdAlgaeIntake> {
 public:
  
  CmdAlgaeIntake(double power);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  double m_power;

  frc::Timer m_timer;
  
};
