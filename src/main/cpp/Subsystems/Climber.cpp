#include "Subsystems/Climber.h"
#include "constants/Constants.h"
#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

Climber::Climber()
{
    frc::SmartDashboard::PutBoolean("Climber Activated Status", IsClimberActivated());
    frc::SmartDashboard::PutBoolean("Climber Done", GetClimberBeamBreak());
    m_isClimberActivated = false;
}

void Climber::SetClimbPower(double power)
{
    m_climber.Set(power);
}

double Climber::GetClimbPower(void)
{
    return m_climber.Get();
}

bool Climber::IsClimberActivated(void)
{
    return m_isClimberActivated;
}

bool Climber::GetClimberBeamBreak()
{
    return m_climberBeamBreak.Get();
}
