#pragma once

#include "constants/Constants.h"

#include <frc/DigitalOutput.h>
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkMax.h>

#include <ctre/phoenix6/CANcoder.hpp>


using namespace rev::spark;

class Claw : public frc2::SubsystemBase {
 public:
  Claw();
  
  void Periodic() override;

/// --- CLAW ---
  void SetClawPower(double power);
  void StopClawPower(double power);

  bool GetClawPhotoEyeFirst(void);

  bool IsCoralReady();

/// --- ALGAE INTAKE ---
  void SetAlgaePower(double power);

  double GetAlgaePower();

  float GetAlgaeCurrent();
  float GetAlgaeTemp();

  bool GetAlgaePhotoEye();
  void SetPivotPower(double power);
  double GetPivotPower();

  void SetPivotAngle(double angle);
  double GetPivotAngle();

  void ZeroEncoder();

  float GetPivotCurrent();
  float GetPivotTemp();

  void SetPivotHold(float position);

  bool isCoralReady;
  
 private:
  SparkMax  m_claw{CLAW_CAN_ID, SparkMax::MotorType::kBrushed};

  SparkMax  m_pivot{PIVOT_CAN_ID, SparkMax::MotorType::kBrushed};

  rev::spark::SparkClosedLoopController m_pivotController = m_pivot.GetClosedLoopController();
  rev::spark::SparkRelativeEncoder      m_algaePivotEncoder = m_pivot.GetEncoder();

  rev::spark::SparkClosedLoopController m_pivotPID = m_pivot.GetClosedLoopController();

  frc::DigitalInput         m_armPhotoeyeFirst  {CLAW_PHOTO_EYE_FIRST};
  frc::DigitalInput         m_algaePhotoEye     {ALGAE_PHOTO_EYE};


};
