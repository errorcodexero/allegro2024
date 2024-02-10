package frc.robot.subsystems.intake_shooter;
import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

public class NoteTransferAction extends Action {
    private IntakeShooterSubsystem sub_;
    private MotorEncoderPowerAction feeder_;

    public NoteTransferAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());
        sub = sub_;

        feeder_ = new MotorEncoderPowerAction(sub_.getFeeder(), "targets:note_transfer");
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "NoteTransferAction";
    }
}