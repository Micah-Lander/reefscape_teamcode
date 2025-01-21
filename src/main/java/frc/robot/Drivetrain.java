package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Drivetrain {

// Add port numbers into parentheses
    public final SwerveModule frontLeftSwerveModule  = new SwerveModule((int)RobotConstants.motorConfigs[0][0],
                                                                        (int)RobotConstants.motorConfigs[0][1],
                                                                        (int)RobotConstants.motorConfigs[0][2], 0);

    public final SwerveModule frontRightSwerveModule = new SwerveModule((int)RobotConstants.motorConfigs[1][0],
                                                                        (int)RobotConstants.motorConfigs[1][1],
                                                                        (int)RobotConstants.motorConfigs[1][2], 1);

    public final SwerveModule backLeftSwerveModule   = new SwerveModule((int)RobotConstants.motorConfigs[2][0],
                                                                        (int)RobotConstants.motorConfigs[2][1],
                                                                        (int)RobotConstants.motorConfigs[2][2], 2);

    public final SwerveModule backRightSwerveModule  = new SwerveModule((int)RobotConstants.motorConfigs[3][0],
                                                                        (int)RobotConstants.motorConfigs[3][1],
                                                                        (int)RobotConstants.motorConfigs[3][2], 3);

    // Add position from each wheel to middle of robot in parentheses in x and y
    private final Translation2d frontLeftPosition     = new Translation2d(0.3, 0.3);
    private final Translation2d frontRightPosition    = new Translation2d(-0.3, 0.3);
    private final Translation2d backLeftPosition      = new Translation2d(0.3, -0.3);
    private final Translation2d backRightPosition     = new Translation2d(-0.3, -0.3);
    
    
    // Add port numbers into parentheses

    private final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(frontLeftPosition,
            frontRightPosition, backLeftPosition, backRightPosition);

    public void drive(double calculatedLeftX, double calculatedLeftY, double calculatedRightY, double period) {
        var swerveDriveStates = swerveDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(new ChassisSpeeds(calculatedLeftX, calculatedLeftY, calculatedRightY), period));
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveDriveStates, RobotConstants.MaxSpeed);

        // we need to create setDesiredSate
        frontLeftSwerveModule.setDesiredState(swerveDriveStates[0]);
        frontRightSwerveModule.setDesiredState(swerveDriveStates[1]);
        backLeftSwerveModule.setDesiredState(swerveDriveStates[2]);
        backRightSwerveModule.setDesiredState(swerveDriveStates[3]);
    }

    public void drivetrain() {
    }
}