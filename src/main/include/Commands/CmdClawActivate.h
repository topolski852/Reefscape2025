#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class CmdClawActivate
    : public frc2::CommandHelper<frc2::Command, CmdClawActivate> {
 public:
  
  CmdClawActivate(double power);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  enum class ClawState
  {
    RunClawFull,
    Sensor1,
    RunClawCreep,
    NotDetected,
    StopMotor,
    EndState
  };

  ClawState currentState = ClawState::RunClawFull;

  private:
  double m_power;
};
