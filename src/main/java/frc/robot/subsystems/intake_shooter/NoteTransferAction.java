package frc.robot.subsystems.intake_shooter;
import org.xero1425.base.actions.Action;
//import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
//import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

public class NoteTransferAction extends Action {
    private IntakeShooterSubsystem sub_;
    //private MotorEncoderPowerAction feeder_;
    //private MotorEncoderPowerAction shooter1_;
    //private MotorEncoderPowerAction shooter2;
    //private MCMotionMagicAction updown_;
    //private MCMotionMagicAction tilt_;

    public NoteTransferAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());
        sub = sub_;
        // after the feeder wheels are prepared, then we make sure the tilt is in the right degrees
        //tilt_ = new MCMotionMagicAction(sub.getFeeder(), "pids:position", "target:shoot", 1, 1);
    }
    @Override
    public String toString(int indent) {
        return prefix(indent) + "NoteTransferAction";
    }
}