package frc.robot.subsystems.intake_shooter;
import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

public class IntakeShooterXferAction extends Action {
    private IntakeShooterSubsystem sub_;
    private MotorEncoderPowerAction feeder_;
    private MCVelocityAction shooter1_;
    private MCVelocityAction shooter2_;   
    private double start_ ;     
    private boolean feeders_running_ ;

    public IntakeShooterXferAction(IntakeShooterSubsystem sub, double feeder, double shooter) throws Exception {
        super(sub.getRobot());

        sub_ = sub;
        feeder_ = new MotorEncoderPowerAction(sub_.getFeeder(), feeder) ;
        shooter1_ = new MCVelocityAction(sub_.getShooter1(), "pids:velocity", shooter, 10.0, false) ;
        shooter2_ = new MCVelocityAction(sub_.getShooter2(), "pids:velocity", shooter, 10.0, false) ;
    }

    @Override
    public void start() throws Exception {
        super.start();
        start_ = sub_.getRobot().getTime() ;
        feeders_running_ = false ;

        sub_.getShooter1().setAction(shooter1_, true);
        sub_.getShooter2().setAction(shooter2_, true);
    }

    @Override
    public void run() throws Exception {
        if (!feeders_running_ && sub_.getRobot().getTime() - start_ > 0.4) {
            sub_.getFeeder().setAction(feeder_, true);
            feeders_running_ = true ;
        }
    }

    @Override
    public void cancel() {
        feeder_.cancel();
        shooter1_.cancel();
        shooter2_.cancel();
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "NoteTransferAction";
    }
}