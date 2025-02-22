#include "Subsystems/Claw.h"
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <frc/smartdashboard/SmartDashboard.h>
using namespace rev::spark;

Claw::Claw()
{
    SparkMaxConfig clawconfig{};

    clawconfig
        .Inverted(false)
        .SetIdleMode(SparkMaxConfig::IdleMode::kCoast);
    clawconfig.encoder
        .PositionConversionFactor(1000)
        .VelocityConversionFactor(1000);
    clawconfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .Pid(1.0, 0.0, 0.0); 
        
    m_claw.Configure(clawconfig,
     SparkMax::ResetMode::kResetSafeParameters,
     SparkMax::PersistMode::kPersistParameters);

    SparkMaxConfig pivotconfig{};
    pivotconfig
        .Inverted(false)
        .SetIdleMode(SparkMaxConfig::IdleMode::kCoast);
    pivotconfig.encoder
        .PositionConversionFactor(1000)
        .VelocityConversionFactor(1000);
    pivotconfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .Pid(1.0, 0.0, 0.0); //To Be Tuned      

    m_pivot.Configure(pivotconfig,
     SparkMax::ResetMode::kResetSafeParameters,
     SparkMax::PersistMode::kPersistParameters);

    frc::SmartDashboard::PutNumber("Claw Current", GetAlgaeCurrent());
    frc::SmartDashboard::PutNumber("Claw Temp", GetAlgaeTemp());

    frc::SmartDashboard::PutNumber("Pivot Current", GetPivotCurrent());
    frc::SmartDashboard::PutNumber("Pivot Temp", GetPivotTemp());


}

// This method will be called once per scheduler run
void Claw::Periodic() 
{
    frc::SmartDashboard::PutBoolean("Claw Photo Eye", GetClawPhotoEyeFirst());

    frc::SmartDashboard::PutBoolean("Algae Photo Eye", GetAlgaePhotoEye()); 
    
    frc::SmartDashboard::PutBoolean("Coral", IsCoralReady()); 
}

// --- CLAW ---

void Claw::SetClawPower(double power)
{
    m_claw.Set(power);
}

void Claw::StopClawPower(double power)
{
    m_claw.Set(0);
}

bool Claw::GetClawPhotoEyeFirst(void)
{
    return m_armPhotoeyeFirst.Get();
}

bool Claw::IsCoralReady()
{
    return isCoralReady;
}

// --- ALGAE INTAKE ---

float Claw::GetAlgaeCurrent()
{
    return m_claw.GetOutputCurrent();
}
bool Claw::GetAlgaePhotoEye()
{
    return m_algaePhotoEye.Get();
}
float Claw::GetAlgaeTemp()
{
    return m_claw.GetMotorTemperature();
}
double Claw::GetAlgaePower()
{
    return m_claw.Get();
}

void Claw::SetAlgaePower(double power)
{
    m_claw.Set(power);
}

// --- ALGAE PIVOT ---

void Claw::SetPivotPower(double power)
{
    m_pivot.Set(power);
}

double Claw::GetPivotPower()
{
    return m_pivot.Get();
}

void Claw::SetPivotAngle(double angle)
{
    m_pivot.Set(angle);
}

double Claw::GetPivotAngle()
{
    return m_pivot.Get();
}

float Claw::GetPivotCurrent()
{
    return m_pivot.GetOutputCurrent();
}

float Claw::GetPivotTemp()
{
    return m_pivot.GetMotorTemperature();
}

void Claw::SetPivotHold(float position)
{
     m_pivotController.SetReference(position, rev::spark::SparkLowLevel::ControlType::kPosition);
}