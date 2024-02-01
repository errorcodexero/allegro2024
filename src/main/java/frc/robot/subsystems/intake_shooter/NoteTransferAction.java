package frc.robot.subsystems.intake_shooter;

public class NoteTransferAction extends Action{
    private IntakeShooterSubsystem sub_;
    private MotorEncoderPowerAction feeder_;

    public NoteTransferAction(IntakeShooterSubsystem sub) throws Exception{
        super(sub.getRobot().getMessageLogger());

        sub = sub_;
        feeder = new MotorEncoderPowerAction(sub.getFeeder(), 2);
    }
    
}
