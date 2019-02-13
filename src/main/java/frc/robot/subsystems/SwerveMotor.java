package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ds;
import frc.robot.RRLogger;
import frc.robot.utilities.MathUtils;

class SwerveMotor {
    private static final int CAN_TIMEOUT = 20;
    private static final double SMALL_NUMBER = 0.0075; //Was 0.05

    private double[] moduleDrift;

    private CANSparkMax clockwiseMotor;
    private CANSparkMax counterMotor;

    private CANEncoder clockwiseEncoder;
    private CANEncoder counterEncoder;

    private CANPIDController clockwisePID;
    private CANPIDController counterPID;

    private final double THEORETICAL_MAX = 5676;
    //4900 works at 12.7 Volts
    private double kMaxOutput = 0.9;
    private double kMinOutput = -0.9;
//Ian Idea for future: set maxRPM based off battery voltage, so battery doesn't inhibit drive train

    private double motorP = 1.651e-4;
    private double motorI = 0.9e-6;
    private double motorD = 0.9e-6;
    private double motorF = 0.93/THEORETICAL_MAX;
//Best value so far
//    private double motorP = 1.651e-4;
//    private double motorI = 0.9e-6;
//    private double motorD = 0.9e-6;
//    private double motorF = 0.93/maxRPM;


    private double lastValidVelocity1 = 0.0;
    private double lastValidVelocity2 = 0.0;

    private double lastValidDistanceClockwise = 0.0;
    private double lastValidDistanceCounter = 0.0;

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

        moduleDrift = new double[2];

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

        clockwiseMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 5);
        counterMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 5);
//        System.out.println(clockwiseMotor.getIdleMode() + "||" + counterMotor.getIdleMode());
//        counterMotor.setClosedLoopRampRate(30);
//        System.out.println(clockwiseMotor.getClosedLoopRampRate() + "||" + counterMotor.getClosedLoopRampRate() + "||" + clockwiseMotor.getOpenLoopRampRate() + "||" + counterMotor.getOpenLoopRampRate());
        //Look into: counterPID.getIMaxAccum() and counterPID.setIAccum()
//        Putting them here breaks it maybe
//        counterPID.setIAccum(0.0);
//        clockwisePID.setIAccum(0.0);
//        clockwiseEncoder.setPosition(0.0);
//        counterEncoder.setPosition(0.0);
        //If change in encoder readings is not proportional to the change in command, omit the encoder value

    }

    /**
     *
     * @param speedCommand Moves wheel forward
     * @param rotationCommand Moves wheel clockwise
     */
    void set(double speedCommand, double rotationCommand) {
          double MAX_RPM = 4000 /*80*(Ds.getBatteryVoltage()-8)*/; //Todo: Bad equation, fix later
//        speedCommand*=0.8;
//        rotationCommand*=0.8;

//        if (Math.abs(speedCommand) + Math.abs(rotationCommand) > 0.8/*0.3*/) {
//            clockwisePID.setFF(0.8/maxRPM);
//            counterPID.setFF(0.8/maxRPM);
//            counterPID.setI(1e-6);
//            clockwisePID.setI(1e-6);
//
//            clockwisePID.setP(1.4e-4);
//            counterPID.setP(1.4e-4);
//        } else {
//            clockwisePID.setP(motorP);
//            counterPID.setP(motorP);
//        }

//        if (Math.abs(rotationCommand) < SMALL_NUMBER) {
//            System.out.println(rotationCommand);
//
//            rotationCommand = 0.0;
//        }

//        if (speedCommand > SMALL_NUMBER) {
//            rotationCommand += speedCommand * moduleDrift[0];
//        } else if (speedCommand < -SMALL_NUMBER) {
//            rotationCommand += speedCommand * moduleDrift[1];
//        }
        double clockwiseCommand = speedCommand + rotationCommand;
        double counterCommand = speedCommand - rotationCommand;
//        double clockwiseOvershoot = Math.abs(clockwiseCommand) - 1;
//        double counterOvershoot = Math.abs(counterCommand) - 1;
//
//        if (clockwiseOvershoot > 0) {
//            clockwiseCommand -= clockwiseOvershoot * Math.signum(clockwiseCommand);
//            counterCommand -= clockwiseOvershoot * Math.signum(clockwiseCommand);
//        }
//
//        if (counterOvershoot > 0) {
//            counterCommand -= counterOvershoot * Math.signum(counterCommand);
//            clockwiseCommand -= counterOvershoot * Math.signum(counterCommand);
//        }


//        if (clockwiseCommand != 0.0 && clockwiseEncoder.getVelocity() != 0.0 || clockwiseCommand == 0.0) {
//            lastValidVelocity1 = clockwiseEncoder.getVelocity();
//        }
//
//        if (counterCommand != 0.0 && counterEncoder.getVelocity() != 0.0 || counterCommand == 0.0) {
//            lastValidVelocity2 = counterEncoder.getVelocity();
//        }

        if (id == 1) {
            SmartDashboard.putNumber("Front Right Ticks", lastValidDistanceClockwise);
            SmartDashboard.putNumber("Front Right Velocity", clockwiseEncoder.getVelocity());
        } else if (id == 2) {
            SmartDashboard.putNumber("Front Left Ticks", lastValidDistanceClockwise);
            SmartDashboard.putNumber("Front Left Velocity", clockwiseEncoder.getVelocity());
        } else if (id == 3) {
            SmartDashboard.putNumber("Back Right Ticks", lastValidDistanceClockwise);
            SmartDashboard.putNumber("Back Right Velocity", clockwiseEncoder.getVelocity());
        } else {
            SmartDashboard.putNumber("Back Left Ticks", lastValidDistanceClockwise);
            SmartDashboard.putNumber("Back Left Velocity", clockwiseEncoder.getVelocity());
//            System.out.println((lastValidDistanceCounter+lastValidDistanceClockwise)*36.0  + "| |" + lastValidDistanceClockwise + "||" + lastValidDistanceCounter + "||" + clockwiseMotor.getAppliedOutput() + "||" + counterMotor.getAppliedOutput());
//            System.out.println((lastValidDistanceCounter+lastValidDistanceClockwise)*36.0 + "| |" + (clockwiseEncoder.getVelocity()+counterEncoder.getVelocity()) + "| |" + clockwiseEncoder.getVelocity() + "| | " + counterEncoder.getVelocity());
//            System.out.println(clockwiseCommand*MAX_RPM + " | | " + clockwiseEncoder.getVelocity());

            RRLogger.addData("Clockwise Motor Command", clockwiseCommand*MAX_RPM);
            RRLogger.addData("Counter Motor Command", counterCommand*MAX_RPM);
            RRLogger.addData("Clockwise Encoder Position", lastValidDistanceClockwise);
            RRLogger.addData("Counter Encoder Position", lastValidDistanceCounter);
            RRLogger.addData("Clockwise Encoder Velocity", clockwiseEncoder.getVelocity());
            RRLogger.addData("Counter Encoder Velocity", counterEncoder.getVelocity());
        }

//        if ((rotationCommand == 0.0) && (clockwiseEncoder.getVelocity() + counterEncoder.getVelocity() > 15.0)) {
//            System.out.println("id:" + id + " | " + clockwiseEncoder.getVelocity() + "| | " + counterEncoder.getVelocity() + " | | " + (clockwiseEncoder.getVelocity()+counterEncoder.getVelocity()));
//        }

        if (Math.abs(clockwiseCommand) > SMALL_NUMBER) {
            clockwiseCommand*=MAX_RPM;
            clockwisePID.setReference(clockwiseCommand, ControlType.kVelocity);
//            clockwisePID.setReference(clockwiseCommand, ControlType.kVoltage);
//            clockwiseMotor.set(clockwiseCommand);

        } else {
            clockwiseMotor.stopMotor();
        }

        if (Math.abs(counterCommand) > SMALL_NUMBER) {
            counterCommand*=MAX_RPM;
            counterPID.setReference(-counterCommand, ControlType.kVelocity);
//            counterPID.setReference(-counterCommand, ControlType.kVoltage);
//            counterMotor.set(-counterCommand);
        } else {
            counterMotor.stopMotor();
        }
    }


    /**
     * @return sum of both encoders, wrapped from 0 to 360
     */
    double getAngle() {
        return MathUtils.resolveDeg((lastValidDistanceClockwise + lastValidDistanceCounter)*36.0);
    }

    /**
     * @return Distance the module has translated
     */
    double getDistance() {

//        if (lastValidDistanceClockwise != clockwiseEncoder.getPosition() && lastValidDistanceCounter != counterEncoder.getPosition()) {
//            if (clockwiseEncoder.getPosition() != 0.0 && counterEncoder.getPosition() != 0.0) {
                lastValidDistanceClockwise = clockwiseEncoder.getPosition();
                lastValidDistanceCounter = counterEncoder.getPosition();
//            }
//        }

        return lastValidDistanceClockwise - lastValidDistanceCounter;
    }

    void reset() {
        clockwiseEncoder.setPosition(0);
        counterEncoder.setPosition(0);
    }

    void stop() {
        clockwiseMotor.stopMotor();
        counterMotor.stopMotor();
    }

    void setID(int id) {
        this.id = id;
    }

    void setDrift(String[] drift) {
        moduleDrift[0] = Double.parseDouble(drift[0]);
        moduleDrift[1] = Double.parseDouble(drift[1]);
    }
}
