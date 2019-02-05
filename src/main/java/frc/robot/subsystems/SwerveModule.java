package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.PIDController;
import frc.robot.utilities.Vector;

/**
 * A swerve module.
 */
public class SwerveModule {
    private static final double TICKS_PER_REVOLUTION = 360.0;
    private static final double DISTANCE_PER_PULSE = 1.0;//0.91;

    private Vector position;
    private double speedCommand;
    private double angleCommand;
    private double distance;
    private double previousDistance = 0;
    private double previousRobotDistance = 0.0;
    private double trueError;
    private double delta = 0.0;
    private double rightDelta = 0.0;
    private double forwardDelta = 0.0;
    private double rightSum = 0;
    private double forwardSum = 0;
    private SwerveMotor swerveMotor;
    private PIDController anglePID;
    private double keepPIDAngle;
    private double angle;
    private boolean wheelReversed;
    double joerot = 400.0;

    double kP = 9.4e-4;
    double kI = 0.0;
    double kD = 0.0;

    private double maxRPM = 5676;

//    private double omega_dRPM = 138.4615/400.0; //mechanical
    private double Max_dRPM = 400.0;
//    private double max_omega = omega_dRPM * Max_dRPM;
//    private double alpha_er = 0.0;
//    private double raw_turn_time = 0.0;
//    private double dT = 0.02;
//    private double turn_command_counts = 0.0;
//    private double omega_command = 0.0;
//    private double dRPM_command = 0.0;

    private double rotationCommand = 0.0;



    double spin = 0;

    private int id;

    /**
     * @param canPortC Port of the motor that moves the wheel Clockwise
     * @param canPortCC Port of the motor that moves the wheel CounterClockwise
     */
    SwerveModule(int canPortC, int canPortCC) {
        super();

        swerveMotor = new SwerveMotor(canPortC, canPortCC);
//TODO, Kp was 0.0012
        anglePID = new PIDController(0.0, 0.0, 0.0);
        anglePID.setContinuous();
        anglePID.setInputRange(0.0, 359.0);
        anglePID.setOutputRange(-1.0, 1.0);
        anglePID.enable();

        SmartDashboard.putNumber("kP", kP);
        SmartDashboard.putNumber("kI", kI);
        SmartDashboard.putNumber("kD", kD);
    }


    void drive() {
        distance = swerveMotor.getDistance() * DISTANCE_PER_PULSE;

        angle = swerveMotor.getAngle();

        trueError = MathUtils.calculateContinuousError(angleCommand, angle, 360.0, 0.0);

        delta = distance - previousDistance;

        if (id == 3) {
//            System.out.println(angle + " | | " + Time.get());
//            System.out.println(dRPM_command + " | | " + alpha_er + " | | " + raw_turn_time + " | | " + turn_command_counts + " | | " + omega_command);
        }

//        if (wheelReversed) {
//            delta = previousDistance - distance;
//        } else {
//            delta = distance - previousDistance;
//        }

        /*
         * If the wheel has to move over 90 degrees
         * go opposite to command and reverse translation
         */

         if (Math.abs(trueError) > TICKS_PER_REVOLUTION/4.0) {
            angleCommand = MathUtils.reverseWheelDirection(angleCommand);
            speedCommand *= -1;

            wheelReversed = true;
        } else {
            wheelReversed = false;
       }

        trueError = MathUtils.calculateContinuousError(angleCommand, angle, 360.0, 0.0);


        //Relatively stable value of kP is 4e-4
        //Good ratio; 30:1:1


        anglePID.setPID(SmartDashboard.getNumber("kP", kP/*0.9e-3, 1e-3*/), SmartDashboard.getNumber("kI", kI/*6.8e-6, 1e-5*/), SmartDashboard.getNumber("kD", kD/*1.9e-4, 2e-4*/));

//        kP = SmartDashboard.getNumber("kP", kP);
//        kI = SmartDashboard.getNumber("kI", kI);
//        kD = SmartDashboard.getNumber("kD", kD);

/* TODO Old offsets:
front_right_drift=-0.0051,-0.009
front_left_drift=0.00236,-0.004
back_left_drift= -0.0025,-0.0085
back_right_drift=0.0059,0.0027
 */
        anglePID.setInput(angle);

        anglePID.setSetpoint(angleCommand);
//        anglePID.setSetpoint(90*speedCommand);


        //Todo

//        if (id == 3) {
//            System.out.println(angleCommand + " :Command");
//        }


//        alpha_er = trueError;

        /*
         * If wheel is not translating, keep the wheel turned where it is
         */

        if (Math.abs(speedCommand) <= 0.1) {
//            alpha_er = 0.0;
        }

        /*
         * If wheel direction has to change more than 22.5 degrees
         * then set wheel speed command to 0 while wheel is turning.
         */

//        if (Math.abs(anglePID.getError()) > TICKS_PER_REVOLUTION/16) {
//            speedCommand = 0.0;
//        }

//        raw_turn_time = Math.abs(alpha_er/max_omega);
//        turn_command_counts = Math.ceil(raw_turn_time/dT);
//        omega_command = alpha_er/(turn_command_counts*dT);
//        dRPM_command = omega_command/omega_dRPM;
//        rotationCommand = dRPM_command/maxRPM;
//

//        rotationCommand = Math.signum(trueError)*Math.pow(Math.signum(trueError)*trueError, 0.99)/540.0;
//        if (Math.abs(rotationCommand) >= 0.7) {
//            rotationCommand = Math.signum(rotationCommand)*0.7;
//        }
//
//        if (Math.abs(trueError) <= 2.0) {
//            rotationCommand = 0.0;
//        }

        if (Math.abs(trueError) > 10) {
            rotationCommand = trueError/1100.0;
        } else {
            rotationCommand = 0.0;
        }
//        System.out.println(rotationCommand);
//        if ((id == 4) || (id == 4)) {
//        rotationCommand = joerot/maxRPM;
//        joerot = joerot + 0.0;
        swerveMotor.set(speedCommand, rotationCommand);
//        swerveMotor.set(speedCommand, anglePID.performPID());

//        } else {
//            swerveMotor.set(0.0, 0.0);
//        }
/*
front_right_drift=-0.0017,0.0
front_left_drift=0.001,0.0
back_left_drift=-0.001,0.0
back_right_drift=0.003,0.0
 */

        rightDelta = delta * Math.sin(Math.toRadians(angle));
        forwardDelta = delta * Math.cos(Math.toRadians(angle));




        if (id == 3) {
//            SmartDashboard.putNumber("BL Angle Command", anglePID.performPID());
            SmartDashboard.putNumber("BL Angle", angle);
            SmartDashboard.putNumber("BL Error", trueError);

        }

        if (id == 2) {
            SmartDashboard.putNumber("FL Angle", angle);
            SmartDashboard.putNumber("FL Error", trueError);
        }


        previousDistance = distance;
        previousRobotDistance = SwerveDrivetrain.getRobotDistance();

    }

    public void setID(int id) {
        this.id = id;
        swerveMotor.setID(id);
    }

    public void setDrift(String[] drift) {
        swerveMotor.setDrift(drift);
    }

    public double getAngle() {
        return angle;
    }

    void setSpeedCommand(double speedCommand) {
        this.speedCommand = speedCommand;
    }

    void setAngleCommand(double angleCommand) {
        this.angleCommand = angleCommand;
    }

    double getSpeedCommand() {
        return speedCommand;
    }

    double getAngleCommand() {
        return angleCommand;
    }

    Vector getPosition() {
        return position;
    }

    public void setPosition(Vector position) {
        this.position = position;
    }

    public double getAngleError() {
        return trueError;
    }

    public double getDistance() {
        return distance;
    }

    public void resetDistance() {
        swerveMotor.reset();
    }

    public boolean getReversed() {
        return this.wheelReversed;
    }

    public double getRightSum() {
        if (wheelReversed) {
            return -rightSum;
        } else {
            return rightSum;
        }
    }

    public double getForwardSum() {
        if (wheelReversed) {
            return -forwardSum;
        } else {
            return forwardSum;
        }
    }

	public void resetDelta() {
		rightSum = 0.0;
		forwardSum = 0.0;
	}

	public double[] getXYDist() {
		if (wheelReversed) {
			forwardSum *= -1.0;
			rightSum *= -1.0;
		}
		double[] i = {rightDelta, forwardDelta};
		return i;
	}
}
