package frc.robot.subsystems.intake_shooter;
import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

public class IntakeShooterXferAction extends Action {
    private IntakeShooterSubsystem sub_;
    private MotorEncoderPowerAction feeder_;
    private MCVelocityAction shooter1_;
    private MCVelocityAction shooter2_;        

    public IntakeShooterXferAction(IntakeShooterSubsystem sub, double feeder, double shooter) throws Exception {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub;
        feeder_ = new MotorEncoderPowerAction(sub_.getFeeder(), feeder) ;
        shooter1_ = new MCVelocityAction(sub_.getShooter1(), "pids:velocity", shooter, 10.0, false) ;
        shooter2_ = new MCVelocityAction(sub_.getShooter2(), "pids:velocity", shooter, 10.0, false) ;
    }

    @Override
    public void start() throws Exception {
        super.start();
        sub_.getFeeder().setAction(feeder_, true);
        sub_.getShooter1().setAction(shooter1_, true);
        sub_.getShooter1().setAction(shooter2_, true);
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "NoteTransferAction";
    }
}