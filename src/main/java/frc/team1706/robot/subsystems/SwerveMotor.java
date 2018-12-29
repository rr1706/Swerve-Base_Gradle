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
        // TODO adjust for when commands go above 1 or below -1
        double clockwiseCommand = speedCommand + rotationCommand;
        double counterCommand = speedCommand - rotationCommand;

        clockwiseMotor.set(clockwiseCommand);
        counterMotor.set(counterCommand);
    }
}
