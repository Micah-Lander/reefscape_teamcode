package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {
        final VelocityVoltage driveRequest;
        final PositionVoltage turnRequest;
        public TalonFX driveMotor, turnMotor;
        private final CANcoder turnEncoder;

        // private PIDController drivePIDController = new PIDController(RobotConstants.drivePIDTuning[0],
        //                 RobotConstants.drivePIDTuning[1], RobotConstants.drivePIDTuning[2]);
        private SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(RobotConstants.driveFeedTuning[0],
                        RobotConstants.driveFeedTuning[1], RobotConstants.driveFeedTuning[2]);

        private PIDController turnPIDController = new PIDController(RobotConstants.turnPIDTuning[0],
                        RobotConstants.turnPIDTuning[1], RobotConstants.turnPIDTuning[2]);
        private SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(RobotConstants.turnFeedTuning[0],
                        RobotConstants.turnFeedTuning[1], RobotConstants.turnFeedTuning[2]);

        private int index;
        // THIS IS FOR TROUBLESHOOTING GET RID OF LATER

        // constructor for swervemodule where objects are initialized
        public SwerveModule(int driveMotorChannel, int turnMotorChannel, int turnMotorEncoderChannel, int motorIndex) {

                this.driveMotor = new TalonFX(driveMotorChannel);
                this.turnMotor = new TalonFX(turnMotorChannel);
                this.turnEncoder = new CANcoder(turnMotorEncoderChannel);

                CANcoderConfiguration config = new CANcoderConfiguration();
                config.MagnetSensor.MagnetOffset = RobotConstants.motorConfigs[motorIndex][3];
                config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
                config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

                turnEncoder.getConfigurator().apply(config);

                
                var driveConfigs = new Slot0Configs();
                driveConfigs.kP = RobotConstants.drivePIDTuning[0];
                driveConfigs.kI = RobotConstants.drivePIDTuning[1]; 
                driveConfigs.kD = RobotConstants.drivePIDTuning[2];
                driveConfigs.kS = 0.1; //tuning to overcome static friction
                driveConfigs.kV = 0.12; //tuning for feedforward

                var turnConfigs = new Slot1Configs();
                turnConfigs.kP = RobotConstants.turnPIDTuning[0]; 
                turnConfigs.kI = RobotConstants.turnPIDTuning[1]; 
                turnConfigs.kD = RobotConstants.turnPIDTuning[2]; 


                driveMotor.getConfigurator().apply(driveConfigs);
                turnMotor.getConfigurator().apply(turnConfigs);

                driveRequest = new VelocityVoltage(0).withSlot(0);
                turnRequest = new PositionVoltage(0).withSlot(1);

                // drivePIDController.setIntegratorRange(-RobotConstants.integratorRange, RobotConstants.integratorRange);
                // turnMotor.continuous

                // get rid of motor index later
                this.index = motorIndex;
        }

        public void setDesiredState(SwerveModuleState moduleState) {
                
                double currentAngle = turnEncoder.getAbsolutePosition().getValue();

                SwerveModuleState.optimize(moduleState, new Rotation2d(currentAngle));
                double desiredAngle = moduleState.angle.getRotations();
                double currentDriveVelocity = driveMotor.getVelocity().getValue();

                if (index == 1) {
                        // System.out.println("Value:
                        // "+MathUtil.clamp(turnPIDController.calculate(currentAngle, desiredAngle),
                        // -0.5, 0.5)+" P: "+turnPIDController.getP()+" I: "+turnPIDController.getI()+"
                        // D: "+turnPIDController.getD());
                        //System.out.println("current: " + currentDriveVelocity);
                        System.out.println(currentAngle);
                        // System.out.println("calc: " + drivePIDController.calculate(currentDriveVelocity, 0.25));

                }

                driveMotor.setControl(driveRequest.withVelocity(1));
                turnMotor.setControl(turnRequest.withPosition(0));


                // driveMotor.set(drivePIDController.calculate(currentDriveVelocity, 0.25));
                // turnMotor.set(turnPIDController.calculate(currentAngle, desiredAngle));

                // not exactly sure where the right joystick (turning joystick) plays into this
                // we reasoned that turning the actual robot would be done by turning all the
                // wheels by a specific angle
                // if all the wheels turn a certain angle then the robot would turn by that
                // angle

        }

}
