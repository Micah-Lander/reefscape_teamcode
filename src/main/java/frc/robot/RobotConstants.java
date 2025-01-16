package frc.robot;

public class RobotConstants {
    public static float MaxSpeed = 3;
    //in m/s
    public static double MaxAngularSpeed = Math.PI/2;
    //half a rotation a second

    public static double wheelRadius = 2;
    //this is in INCHES


    public static double[] drivePIDTuning  = new double[]{1.0,0.0,0.0};
    public static double[] driveFeedTuning = new double[]{0.0,0.0,0.0};

    public static double[] turnPIDTuning   = new double[]{0.0,0.0,0.0};
    public static double[] turnFeedTuning  = new double[]{0.0,0.0,0.0};

    public static double integratorRange   = 0.5;

    //you need to change all these pid tuning values later when you can test

    public static double[][] motorConfigs  = new double[][]{{7, 8, 8, -0.37817}, 
                                                            {5, 6, 6, -0.62768}, 
                                                            {1, 2, 2, -0.07}, 
                                                            {3, 4, 4, -0.24759}};
    //first is frontLeft, second is frontRight, third is backLeft, fourth is backRight
    //first is driveChannel, second is turnChannel, third is CANCoder, fourth is magnetOffset

}
