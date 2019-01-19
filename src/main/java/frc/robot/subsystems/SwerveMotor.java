package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.MathUtils;

class SwerveMotor {
    private static final int CAN_TIMEOUT = 20;
    private static final double SMALL_NUMBER = 0.05;

    private CANSparkMax clockwiseMotor;
    private CANSparkMax counterMotor;

    private CANEncoder clockwiseEncoder;
    private CANEncoder counterEncoder;

    private CANPIDController clockwisePID;
    private CANPIDController counterPID;

    private double motorP = 4e-4;
    private double motorI = 0.0;
    private double motorD = 2e-5;
    private double motorF = 0.0;
    private double kMaxOutput = 1;
    private double kMinOutput = -1;
    private double maxRPM = 5676;

    private double lastValidVelocity1 = 0.0;
    private double lastValidVelocity2 = 0.0;

    private double lastValidDistance1 = 0.0;
    private double lastValidDistance2 = 0.0;

    int zeros = 0;
    int goods = 0;

    private int id;

    /**
     *
     * @param canPortC Port of the motor that moves the wheel Clockwise
     * @param canPortCC Port of the motor that moves the wheel CounterClockwise
     */
    SwerveMotor(int canPortC, int canPortCC) {
        clockwiseMotor = new CANSparkMax(canPortC, CANSparkMaxLowLevel.MotorType.kBrushless);
        counterMotor = new CANSparkMax(canPortCC, CANSparkMaxLowLevel.MotorType.kBrushless);

        clockwiseMotor.setCANTimeout(CAN_TIMEOUT);
        counterMotor.setCANTimeout(CAN_TIMEOUT);

        clockwiseEncoder = new CANEncoder(clockwiseMotor);
        counterEncoder = new CANEncoder(counterMotor);

        clockwiseMotor.setInverted(false);
        counterMotor.setInverted(false);

        clockwisePID = clockwiseMotor.getPIDController();
        clockwisePID.setP(motorP);
        clockwisePID.setI(motorI);
        clockwisePID.setD(motorD);
        clockwisePID.setIZone(0.0);
        clockwisePID.setFF(motorF);
        clockwisePID.setOutputRange(kMinOutput, kMaxOutput);

        counterPID = counterMotor.getPIDController();
        counterPID.setP(motorP);
        counterPID.setI(motorI);
        counterPID.setD(motorD);
        counterPID.setIZone(0.0);
        counterPID.setFF(motorF);
        counterPID.setOutputRange(kMinOutput, kMaxOutput);
    }

    /**
     *
     * @param speedCommand Moves wheel forward
     * @param rotationCommand Moves wheel clockwise
     */
    void set(double speedCommand, double rotationCommand) {
//        speedCommand*=0.8;
//        rotationCommand*=0.8;
        double clockwiseCommand = speedCommand + rotationCommand;
        double counterCommand = speedCommand - rotationCommand;
        double clockwiseOvershoot = Math.abs(clockwiseCommand) - 1;
        double counterOvershoot = Math.abs(counterCommand) - 1;

        if (clockwiseOvershoot > 0) {
            clockwiseCommand -= clockwiseOvershoot * Math.signum(clockwiseCommand);
            counterCommand -= clockwiseOvershoot * Math.signum(clockwiseCommand);
        }

        if (counterOvershoot > 0) {
            counterCommand -= counterOvershoot * Math.signum(counterCommand);
            clockwiseCommand -= counterOvershoot * Math.signum(counterCommand);
        }

        if (clockwiseCommand != 0.0 && clockwiseEncoder.getVelocity() != 0.0 || clockwiseCommand == 0.0) {
            lastValidVelocity1 = clockwiseEncoder.getVelocity();
        }

        if (counterCommand != 0.0 && counterEncoder.getVelocity() != 0.0 || counterCommand == 0.0) {
            lastValidVelocity2 = counterEncoder.getVelocity();
        }

//        if (id == 1) {
//            System.out.println("C: " + speedCommand + " | CC: " + rotationCommand);
//        }

        if (Math.abs(clockwiseCommand) > SMALL_NUMBER) {
            clockwiseCommand*=maxRPM;
            clockwisePID.setReference(clockwiseCommand, ControlType.kVelocity);

        } else {
            clockwiseMotor.stopMotor();
        }

        if (Math.abs(counterCommand) > SMALL_NUMBER) {
            counterCommand*=maxRPM;
            counterPID.setReference(-counterCommand, ControlType.kVelocity);

        } else {
            counterMotor.stopMotor();
        }

        SmartDashboard.putNumber("Clockwise Command", clockwiseCommand);
        SmartDashboard.putNumber("Counter Command", counterCommand);

//        if (clockwiseEncoder.getVelocity() == 0.0 || counterEncoder.getVelocity() == 0.0) {
//            System.out.println("Bad: " + zeros++);
//        } else {
//            System.out.println("Good: " + goods++);
//        }


        SmartDashboard.putNumber("Motor1 Velocity", clockwiseEncoder.getVelocity());
        SmartDashboard.putNumber("Motor2 Velocity", -counterEncoder.getVelocity());

        }


    /**
     * @return sum of both encoders, wrapped from 0 to 360
     */
    double getAngle() {
        return MathUtils.resolveDeg((lastValidDistance1 + lastValidDistance2)*36.0);
    }

    /**
     * @return Distance the module has translated
     */
    double getDistance() {
        if (clockwiseEncoder.getPosition() != 0.0) {
            lastValidDistance1 = clockwiseEncoder.getPosition();
        }

        if (counterEncoder.getPosition() != 0.0) {
            lastValidDistance2 = counterEncoder.getPosition();
        }
        return lastValidDistance1 - lastValidDistance2;
    }

    void reset() {

    }

    void stop() {
        clockwiseMotor.stopMotor();
        counterMotor.stopMotor();
    }

    void setID(int id) {
        this.id = id;
    }
}
