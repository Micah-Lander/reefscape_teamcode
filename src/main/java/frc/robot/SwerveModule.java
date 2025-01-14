package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Preferences;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {

    public TalonFX driveMotor, turnMotor;
    private final CANcoder turnEncoder;
    // in m/s

    private PIDController drivePIDController = new PIDController(RobotConstants.drivePIDTuning[0], RobotConstants.drivePIDTuning[1],
            RobotConstants.drivePIDTuning[2]);
    private SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(RobotConstants.driveFeedTuning[0], 
            RobotConstants.driveFeedTuning[1],  RobotConstants.driveFeedTuning[2]);

    private PIDController turnPIDController = new PIDController(RobotConstants.turnPIDTuning[0], RobotConstants.turnPIDTuning[1],
            RobotConstants.turnPIDTuning[2]);
    private SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(RobotConstants.turnFeedTuning[0], 
            RobotConstants.turnFeedTuning[1], RobotConstants.turnFeedTuning[2]);


    // constructor for swervemodule where you set the driving motor and the turning
    // motor
    public SwerveModule(int driveMotorChannel, int turnMotorChannel, int turnMotorEncoderChannel, int motorIndex) {

        this.driveMotor  = new TalonFX(driveMotorChannel);
        this.turnMotor   = new TalonFX(turnMotorChannel);
        this.turnEncoder = new CANcoder(turnMotorEncoderChannel);

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset        = RobotConstants.motorConfigs[motorIndex][3];
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        config.MagnetSensor.SensorDirection     = SensorDirectionValue.Clockwise_Positive;

        turnEncoder.getConfigurator().apply(config);
        // applying encoder configs

        drivePIDController.setIntegratorRange(-RobotConstants.integratorRange,RobotConstants.integratorRange);


    }

    public void setDesiredState(SwerveModuleState moduleState){

        double currentAngle = turnEncoder.getAbsolutePosition().getValue() * 2 * Math.PI;

        SwerveModuleState.optimize(moduleState, new Rotation2d(currentAngle));
        double desiredAngle = moduleState.angle.getRadians();
        double currentDriveVelocity = driveMotor.getVelocity().getValueAsDouble();
        double desiredDriveVelocity = moduleState.speedMetersPerSecond * Math.cos(desiredAngle-currentAngle);
        double currentAngularVelocity = turnEncoder.getVelocity().getValueAsDouble();
        double desiredAngularVelocity = moduleState.speedMetersPerSecond / (RobotConstants.wheelRadius * 0.0254);
        //in order to get the desired angular velocity we take the desired linear velocity and divide it by the wheel's radius
        //the 0.0254 is because 2.54 is the amount of centimeters in an inch and then divide that by 100 for the meters in an inch


        System.out.println("Desired Angle: " + (desiredAngle - currentAngle));
        System.out.println("Desired Velocity: " + desiredDriveVelocity);
        driveMotor.set((desiredDriveVelocity / RobotConstants.MaxSpeed) / 8);
        turnMotor.set((desiredAngle-currentAngle / RobotConstants.MaxAngularSpeed) / 8);
        //control the speed of the drive motor
        //driveMotor.setVoltage(drivePIDController.calculate(currentDriveVelocity, desiredDriveVelocity) +
                              //driveFeedForward.calculate(currentDriveVelocity, desiredDriveVelocity));

        //control the angle of the turn motor
        //turnMotor.setVoltage(turnPIDController.calculate(currentAngle, desiredAngle) +
                             //turnFeedForward.calculate(currentAngularVelocity, desiredAngularVelocity));
        

        //not exactly sure where the right joystick (turning joystick) plays into this
        //we reasoned that turning the actual robot would be done by turning all the wheels by a specific angle
        //if all the wheels turn a certain angle then the robot would turn by that angle
        
    }
    public void refreshTuning(){
        drivePIDController.setP(Preferences.getDouble("driveKP",0));
        drivePIDController.setI(Preferences.getDouble("driveKI",0));
        drivePIDController.setD(Preferences.getDouble("driveKD",0));
        turnPIDController.setP(Preferences.getDouble("turnKP",0));
        turnPIDController.setI(Preferences.getDouble("turnKI",0));
        turnPIDController.setD(Preferences.getDouble("turnKD",0));

        driveFeedForward = new SimpleMotorFeedforward(Preferences.getDouble("driveKS",0),
                Preferences.getDouble("driveKV",0), Preferences.getDouble("driveKA",0));
        turnFeedForward = new SimpleMotorFeedforward(Preferences.getDouble("turnKS",0),
                Preferences.getDouble("turnKV",0), Preferences.getDouble("turnKA",0));
    }

}
