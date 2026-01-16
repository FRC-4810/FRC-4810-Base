#include "Drivetrain.h"
#include "LimelightHelpers.h"

#include <frc/StateSpaceUtil.h>
#include <frc/DriverStation.h>

#include <networktables/NetworkTableInstance.h>

Drivetrain::Drivetrain()
{
    m_bIsFieldRelative = true;

    m_bLockOnStop = false;
    m_dGyroOffset = 180.0;

    m_bUseCameraMeasurements = false;
}

void Drivetrain::Initialize ( RobotIO *p_pRobotIO )
{
    m_pRobotIO = p_pRobotIO;

    m_frontLeft.ConfigModule();
    m_frontRight.ConfigModule();
    m_backLeft.ConfigModule();
    m_backRight.ConfigModule();

//-GMS - Odometry
    m_DrivetrainTimer = new frc::Timer{};
    m_DrivetrainTimer->Reset();
    m_DrivetrainTimer->Start();

    // Set gyrp offset at start (change m_dGyroOffset depending on what direction the bot should be facing at start)
    m_gyro.SetYaw(units::degree_t{m_dGyroOffset});
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("setIMUMode", 1);
}

void Drivetrain::Execute (  double leftYJoystickValue, double leftXJoystickValue,
                double rightXJoystickValue  )
{
    frc::SmartDashboard::PutNumber("Gyro Angle", (double)GetGyroRotation2d().Degrees());
    frc::Pose2d botPose = GetBotPose();
    frc::SmartDashboard::PutNumber("Odometry/Odometry X", (double)botPose.X());
    frc::SmartDashboard::PutNumber("Odometry/Odometry Y", (double)botPose.Y());
    frc::SmartDashboard::PutNumber("Odometry/Odometry Rot", (double)botPose.Rotation().Degrees());

    frc::SmartDashboard::PutBoolean("Drive Bot Relative", m_bIsFieldRelative);

    // Left Y value is Chassie X value (forward/backward)
    // Left X value is Chassie Y value (left/right)
    // Right X value is Chassie Rotation value (Clockwise/Counterclockwise)

    //-GMS - clamp values so chassis speed never above max speed
    if(pow(leftYJoystickValue, 2) + pow(leftXJoystickValue, 2) > 1 )
    {
        leftYJoystickValue /= sqrt(pow(leftYJoystickValue, 2) + pow(leftXJoystickValue, 2));
        leftXJoystickValue /= sqrt(pow(leftYJoystickValue, 2) + pow(leftXJoystickValue, 2));
    }

    // All speeds need to be inverted - Correct for joystick inversion.
    const auto xSpeed = m_xSpeedLimiter.Calculate(-frc::ApplyDeadband(leftYJoystickValue, 0.12)) * drivetrain::kMaxSpeed;
    const auto ySpeed = m_ySpeedLimiter.Calculate(frc::ApplyDeadband(leftXJoystickValue, 0.12)) * drivetrain::kMaxSpeed;
    const auto rot =  m_rotSpeedLimiter.Calculate(frc::ApplyDeadband(rightXJoystickValue, 0.12)) * drivetrain::kMaxAngularSpeed;

    //If all speeds are 0 and lock on stop is true, set wheels to opposing 45s
    if((double)xSpeed == 0 && (double)ySpeed == 0 && (double)rot == 0 && m_bLockOnStop)
    {
        m_frontLeft.SetDesiredState(frc::SwerveModuleState{units::meters_per_second_t{0}, units::radian_t{std::numbers::pi / 4}});      //Set Speed to 0, angle to 45
        m_frontRight.SetDesiredState(frc::SwerveModuleState{units::meters_per_second_t{0}, units::radian_t{-std::numbers::pi / 4}});    //Set Speed to 0, angle to -45
        m_backLeft.SetDesiredState(frc::SwerveModuleState{units::meters_per_second_t{0}, units::radian_t{-std::numbers::pi / 4}});      //Set Speed to 0, angle to -45
        m_backRight.SetDesiredState(frc::SwerveModuleState{units::meters_per_second_t{0}, units::radian_t{std::numbers::pi / 4}});      //Set Speed to 0, angle to 45
        
        return;
    }

    if(m_bIsFieldRelative)  //Field Centric Drive
    {
        DriveFieldRelative((double)xSpeed, (double)ySpeed, (double)rot);
    }
    else    //Bot centric drive
    {
        DriveBotRelative((double)xSpeed, (double)ySpeed, (double)rot);
    }
}

void Drivetrain::DriveFieldRelative( double xSpeed, double ySpeed, double rotSpeed )
{
    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::meters_per_second_t{xSpeed}, units::meters_per_second_t{ySpeed}, units::radians_per_second_t{rotSpeed}, GetGyroRotation2d());

    auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds);  //Go from speeds to swerve module states using kinematics objects

    m_frontLeft.SetDesiredState(fl);    //Set Front Left State to target
    m_frontRight.SetDesiredState(fr);   //Set Front Right State to target
    m_backLeft.SetDesiredState(bl);     //Set Back Left State to target
    m_backRight.SetDesiredState(br);    //Set Back Right State to target

    UpdateOdometry();
}

void Drivetrain::DriveBotRelative( double xSpeed, double ySpeed, double rotSpeed )
{
    frc::ChassisSpeeds speeds = frc::ChassisSpeeds{units::meters_per_second_t{ xSpeed }, units::meters_per_second_t{ ySpeed }, units::radians_per_second_t{ rotSpeed }};

    auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds);  //Go from speeds to swerve module states using kinematics objects

    m_frontLeft.SetDesiredState(fl);    //Set Front Left State to target
    m_frontRight.SetDesiredState(fr);   //Set Front Right State to target
    m_backLeft.SetDesiredState(bl);     //Set Back Left State to target
    m_backRight.SetDesiredState(br);    //Set Back Right State to target

    UpdateOdometry();
}

void Drivetrain::Stop()
{
    m_frontLeft.Stop();
    m_frontRight.Stop();
    m_backLeft.Stop();
    m_backRight.Stop();
}


// ************
// * Odometry *
// ************

void Drivetrain::UpdateOdometry()
{
    //-GMS - Try base odometry code to start - power on in a corner, drive around a room, and return to the 
    // corner. Values for x and y should be near zero. Also check left is positive y, forward is positive x.
    // Driver station should be set to blue alliance, though I don't think it will impact this.
    // Odometry values are posted to shuffleboard under "Odometry" folder.
    m_poseEstimator.UpdateWithTime(
        m_DrivetrainTimer->Get(),
        GetGyroRotation2d(),
        {
            m_frontLeft.GetPosition(),
            m_frontRight.GetPosition(),
            m_backLeft.GetPosition(),
            m_backRight.GetPosition()
        }
    );
}

void Drivetrain::ResetOdometry( frc::Pose2d pose )
{
    m_poseEstimator.ResetPose( pose );
}

frc::Pose2d Drivetrain::GetBotPose()
{
    return {
        m_poseEstimator.GetEstimatedPosition()
    };
}

void Drivetrain::TryAddVisionMeasurement(double correctedGyroDegrees)
{
    LimelightHelpers::SetRobotOrientation("limelight", (double)GetBotPose().Rotation().Degrees(), 0, 0, 0, 0, 0);  //Other 5 values are optional, ommitted for simplicity sake

    // ******************
    // * Megatag 1 code *
    // ******************

    LimelightHelpers::PoseEstimate mt1_pose = LimelightHelpers::getBotPoseEstimate_wpiBlue();

    bool bDoRejectUpdate = false;

    if(mt1_pose.tagCount == 1 && mt1_pose.rawFiducials.size() == 1)
    {
        if(mt1_pose.rawFiducials[0].ambiguity > 0.7)
        {
            bDoRejectUpdate = true;
        }

        if(mt1_pose.rawFiducials[0].distToCamera > 3)
        {
            bDoRejectUpdate = true;
        }
    }
    if(mt1_pose.tagCount == 0)
    {
        bDoRejectUpdate = true;
    }

    if(!bDoRejectUpdate)
    {
        m_poseEstimator.SetVisionMeasurementStdDevs({0.5, 0.5, 99999999});
        m_poseEstimator.AddVisionMeasurement(
            mt1_pose.pose,
            mt1_pose.timestampSeconds
        );
    }

    // ******************
    // * Megatag 2 code *
    // ******************

    /*LimelightHelpers::PoseEstimate mt2_pose = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    bool bDoRejectUpdate = false;

    if( fabs(m_gyro.GetAngularVelocityZWorld().GetValueAsDouble()) > 360)
    {
        bDoRejectUpdate = true;
    }
    if(mt2_pose.tagCount == 0)
    {
        bDoRejectUpdate = true;
    }

    if(!bDoRejectUpdate)
    {
        m_poseEstimator.SetVisionMeasurementStdDevs({.7,.7,9999999});
        m_poseEstimator.AddVisionMeasurement(
            mt2_pose.pose,
            mt2_pose.timestampSeconds
        );

        printf("Mt2 Pose Added | X: [%f], Y: [%f], Rot: [%f]", (double)mt2_pose.pose.X(), (double)mt2_pose.pose.Y(), (double)mt2_pose.pose.Rotation().Degrees());
    }*/
}
