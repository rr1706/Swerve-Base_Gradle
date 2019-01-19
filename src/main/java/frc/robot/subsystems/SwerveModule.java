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

    private Vector position;
    private double speedCommand;
    private double angleCommand;
    private double distance;
    private double previousDistance = 0;
    private double previousRobotDistance = 0.0;
    private double angleError;
    private double rightSum = 0;
    private double forwardSum = 0;
    private SwerveMotor swerveMotor;
    private PIDController anglePID;
    private double keepPIDAngle;
    private double angle;
    private double rightDelta;
    private double forwardDelta;
    private boolean wheelReversed;

    double spin = 0;

    private int id;

    /**
     * @param canPortC Port of the motor that moves the wheel Clockwise
     * @param canPortCC Port of the motor that moves the wheel CounterClockwise
     */
    SwerveModule(int canPortC, int canPortCC) {
        super();

        swerveMotor = new SwerveMotor(canPortC, canPortCC);

        anglePID = new PIDController(0.002, 0.0, 0.0);
        anglePID.setContinuous();
        anglePID.setInputRange(0.0, 360.0);
        anglePID.setOutputRange(-1.0, 1.0);
        anglePID.enable();

    }

    void drive() {
        double delta;
        double rightDelta;
        double forwardDelta;

        distance = swerveMotor.getDistance();

        angle = swerveMotor.getAngle();

        angleError = MathUtils.calculateError(angleCommand, angle);

        if (wheelReversed) {
            delta = previousDistance - distance;
        } else {
            delta = distance - previousDistance;
        }

        /*
         * If the wheel has to move over 90 degrees
         * go opposite to command and reverse translation
         */
//        if (Math.abs(angleError) > TICKS_PER_REVOLUTION/4.0) {
//            angleCommand = MathUtils.reverseWheelDirection(angleCommand);
//            speedCommand *= -1;
//            angleError = MathUtils.reverseErrorDirection(angleError);
//
//            wheelReversed = true;
//        } else {
//            wheelReversed = false;
//        }

//        anglePID.setPID(SmartDashboard.getNumber("kP", 0.0), SmartDashboard.getNumber("kI", 0.0), SmartDashboard.getNumber("kD", 0.0));
        anglePID.setInput(angle);
        anglePID.setSetpoint(angleCommand);

        /*
         * If wheel is not translating, keep the wheel turned where it is
         */
        if (Math.abs(speedCommand) <= 0.1) {
            anglePID.setSetpoint(keepPIDAngle);
        } else {
            keepPIDAngle = angleCommand;
        }

        /*
         * If wheel direction has to change more than 22.5 degrees
         * then set wheel speed command to 0 while wheel is turning.
         */
//        if (Math.abs(anglePID.getError()) > TICKS_PER_REVOLUTION/16) {
//            speedCommand = 0.0;
//        }


//        anglePID.setSetpoint(spin+=7.0);
//        spin%=360;


        SmartDashboard.putNumber("Angle", Math.abs(angle));
        SmartDashboard.putNumber("Angle Command", Math.abs(angleCommand));
        SmartDashboard.putNumber("Angle Error", Math.abs(anglePID.getError()));

        swerveMotor.set(speedCommand, anglePID.performPID());


        SmartDashboard.putNumber("Angle PID", anglePID.performPID());

        previousDistance = distance;
        previousRobotDistance = SwerveDrivetrain.getRobotDistance();

    }

    public void setID(int id) {
        this.id = id;
        swerveMotor.setID(id);
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
        return angleError;
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
