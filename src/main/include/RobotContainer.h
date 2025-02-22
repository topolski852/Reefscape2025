#pragma once

#include <frc2/command/CommandPtr.h>

#include <frc2/command/CommandPtr.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/NetworkButton.h>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>


// --- SUBSYSTEMS ---
#include "subsystems/Claw.h"
#include "subsystems/Climber.h"
#include "Subsystems/Elevator.h"
#include "subsystems/Drive.h"
#include "str/vision/VisionSystem.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  Climber m_climber;
  Claw    m_claw;
  Elevator m_elevator;

  // Drive& GetDrive();
  // str::vision::VisionSystem& GetVision();


  frc2::CommandXboxController m_topDriver{0};

 private:
  void ConfigureBindings();
  
  //frc2::CommandXboxController driverJoystick{0};

  // Drive driveSub{};

  bool SmartDashHoming;

    // str::vision::VisionSystem vision{
    //   [this](const frc::Pose2d& pose, units::second_t time,
    //          const Eigen::Vector3d& stdDevs) {
    //     driveSub.AddVisionMeasurement(pose, time, stdDevs);
    //   },
    //   [this](const frc::Pose2d& pose, units::second_t time,
    //          const Eigen::Vector3d& stdDevs) {
    //     driveSub.AddSingleTagVisionMeasurement(pose, time, stdDevs);
    //   }};

  // std::shared_ptr<nt::NetworkTable> tuningTable{
  //     nt::NetworkTableInstance::GetDefault().GetTable("Tuning")};
  // frc2::NetworkButton steerTuneBtn{tuningTable, "SteerPidTuning"};
  // frc2::NetworkButton driveTuneBtn{tuningTable, "DrivePidTuning"};
  // frc2::NetworkButton steerSysIdVoltsBtn{tuningTable, "SteerSysIdVolts"};
  // frc2::NetworkButton steerSysIdTorqueCurrentBtn{tuningTable,
  //                                                "SteerSysIdTorqueCurrent"};
  // frc2::NetworkButton driveSysIdBtn{tuningTable, "DriveSysId"};
  // frc2::NetworkButton wheelRadiusBtn{tuningTable, "WheelRadius"};



};
