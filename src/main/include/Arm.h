#pragma once

#include <frc/Timer.h>

#include "RobotIO.h"

namespace arm
{
    enum eState
    {
        STATE_START = 0,
        STATE_IDLE = 1,
        STATE_HOMING = 2,
        STATE_MANUAL_RAISE = 3,
        STATE_MANUAL_LOWER = 4,
        STATE_AUTO_RAISE = 5,
        STATE_ERROR = 99
    };

    enum eCommand
    {
        COMMAND_NONE,
        COMMAND_HOME,
        COMMAND_MANUAL_RAISE,
        COMMAND_MANUAL_LOWER,
        COMMAND_AUTO_RAISE,
        COMMAND_STOP
    };

    // Speed Setpoints
    static constexpr double dHomeSpeed = -.3;
    static constexpr double dManualRaiseSpeed = .3;
    static constexpr double dManualLowerSpeed = -.2;
    static constexpr double dAutoRaiseSpeed = .3;
    static constexpr double dStopSpeed = 0;

    // Timer Setpoints
    static constexpr double dHomeTimeout = 10.0;
    static constexpr double dManualTimeout = 15.0;
    static constexpr double dAutoRaiseTimeout = 15.0;
}

class Arm
{
public:

    // Constructor/Destructor
    Arm();
    ~Arm()
        {  }

    // Accessor Methods
    inline void Home()
        {  m_eCommand = arm::COMMAND_HOME;  }

    inline void ManualRaise()
        {  m_eCommand = arm::COMMAND_MANUAL_RAISE;  }

    inline void ManualLower()
        {  m_eCommand = arm::COMMAND_MANUAL_LOWER;  }

    inline void AutoRaise()
        {  m_eCommand = arm::COMMAND_AUTO_RAISE;  }
    
    inline void Stop()
        {  m_eCommand = arm::COMMAND_STOP;  }


    inline bool IsIdle()
        {  return(m_eState == arm::eState::STATE_IDLE);  }

    inline bool IsHoming()
        {  return(m_eState == arm::eState::STATE_HOMING);  }

    inline bool IsManualRaising()
        {  return(m_eState == arm::eState::STATE_MANUAL_RAISE);  }

    inline bool IsManualLowering()
        {  return(m_eState == arm::eState::STATE_MANUAL_LOWER);  }

    inline bool IsAutoRaising()
        {  return(m_eState == arm::eState::STATE_AUTO_RAISE);  } 
        
    
    void Initialize( RobotIO *p_pRobotIO );
    void Execute();
    void UpdateInputStatus();

private:
    arm::eState m_eState;
    RobotIO *m_pRobotIO;
    arm::eCommand m_eCommand;

    frc::Timer *m_pTimeoutTimer;

    //Add Motor Configs Variable
};