package frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;

class SwerveMotor {
    private Talon clockwiseMotor;
    private Talon counterMotor;

    /**
     *
     * @param pwmPortC Port of the motor that moves the wheel Clockwise
     * @param pwmPortCC Port of the motor that moves the wheel CounterClockwise
     */
    SwerveMotor(int pwmPortC, int pwmPortCC){
        clockwiseMotor = new Talon(pwmPortC);
        counterMotor = new Talon(pwmPortCC);
    }

    /**
     *
     * @param speedCommand Moves wheel forward
     * @param rotationCommand Moves wheel clockwise
     */
    void set(double speedCommand, double rotationCommand){
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

        clockwiseMotor.set(clockwiseCommand);
        counterMotor.set(counterCommand);
    }
}
