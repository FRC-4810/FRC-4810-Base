#include "Arm.h"

Arm::Arm()
{
    m_pRobotIO = nullptr;
    m_eState = arm::eState::STATE_START;
    m_eCommand = arm::COMMAND_NONE;
}

void Arm::Initialize(RobotIO *p_pRobotIO)
{
    m_pRobotIO = p_pRobotIO;
    m_pTimeoutTimer = new frc::Timer();
    m_pTimeoutTimer->Reset();

    //Refresh Motor Configs
}

void Arm::UpdateInputStatus()
{

}

void Arm::Execute()
{
    if(m_pRobotIO != nullptr)
    {
        if(m_eState == arm::eState::STATE_START)
        {
            m_eState = arm::eState::STATE_IDLE;
        }


        if(m_eState == arm::eState::STATE_IDLE)
        {
            if(m_eCommand == arm::COMMAND_HOME)
            {
                if(/* Lower Limit Triggered */)
                {
                    m_eCommand = arm::COMMAND_NONE;
                    return;
                }
                //Apply motor configs
                
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set Motor Speeds

                m_eState = arm::eState::STATE_HOMING;
            }
            else if(m_eCommand == arm::COMMAND_MANUAL_RAISE)
            {
                //Apply motor configs
                
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set Motor Speeds

                m_eState = arm::eState::STATE_MANUAL_RAISE;
            }
            else if(m_eCommand == arm::COMMAND_MANUAL_LOWER)
            {
                if(/* Lower Limit Triggered */)
                {
                    m_eCommand = arm::COMMAND_NONE;
                    return;
                }

                //Apply motor configs
                
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set Motor Speeds

                m_eState = arm::eState::STATE_MANUAL_LOWER;
            }
            else if(m_eCommand == arm::COMMAND_AUTO_RAISE)
            {
                if(/*MOTOR IS PAST TARGET*/)
                {
                    m_eCommand = arm::COMMAND_NONE;
                    return;
                }

                //Apply motor configs
                
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set Motor Speeds

                m_eState = arm::eState::STATE_AUTO_RAISE;
            }

            else if(m_eCommand == arm::COMMAND_STOP)
            {
                m_eCommand = arm::COMMAND_NONE;
            }
        }



        else if(m_eState == arm::eState::STATE_HOMING)
        {
            bool bTimeoutReached = false;
            if(m_pTimeoutTimer->Get() >= units::time::second_t{arm::dHomeTimeout})
            {
                bTimeoutReached = true;
            }
            
            if(/*CHECK IF LOWER LIMIT REACHED*/ || bTimeoutReached)
            {
                //Stop Motors
                m_eCommand = arm::COMMAND_NONE;
                m_eState = arm::eState::STATE_IDLE;
            }
        }
        else if(m_eState == arm::eState::STATE_MANUAL_RAISE)
        {
            bool bTimeoutReached = false;
            if(m_pTimeoutTimer->Get() >= units::time::second_t{arm::dManualTimeout})
            {
                bTimeoutReached = true;
            }
            
            if(m_eCommand == arm::COMMAND_STOP || bTimeoutReached)
            {
                //Stop Motors
                m_eCommand = arm::COMMAND_NONE;
                m_eState = arm::eState::STATE_IDLE;
            }
        }
        else if(m_eState == arm::eState::STATE_MANUAL_LOWER)
        {
            bool bTimeoutReached = false;
            if(m_pTimeoutTimer->Get() >= units::time::second_t{arm::dManualTimeout})
            {
                bTimeoutReached = true;
            }
            
            if(m_eCommand == arm::COMMAND_STOP || /*If limit switch triggered*/ || bTimeoutReached)
            {
                //Stop Motors
                m_eCommand = arm::COMMAND_NONE;
                m_eState = arm::eState::STATE_IDLE;
            }
        }
        else if(m_eState == arm::eState::STATE_AUTO_RAISE)
        {
            bool bTimeoutReached = false;
            if(m_pTimeoutTimer->Get() >= units::time::second_t{arm::dAutoRaiseTimeout})
            {
                bTimeoutReached = true;
            }
            
            if(/*CHECK IF ENCODER POSITION REACHED*/ || bTimeoutReached)
            {
                //Stop Motors
                m_eCommand = arm::COMMAND_NONE;
                m_eState = arm::eState::STATE_IDLE;
            }
        }

        else
        {
            printf("Unknown or Error State Encountered\n");
        }
    }
    else
    {
        printf("Arm - Null RobotIO Pointer Encountered\n");
    }
}