package frc.robot.subsystems.intake_shooter;

// Runs the eject action for 1.5 seconds after the updown and tilt motors reach
// their stowed positions
import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

//
// TODO: This will not work like this.  The issue is that we are assinging a new action to
//       the intake shooter subsystem.  This will cancel this action so once you assign the
//       stow action, this action will be cancelled and run() will never be called again.  We
//       need to rework this so that the stow happens as part of this action, which by the way
//       it already does given the starts for eject_updown and eject_tilt are all stow targets.
//

public class IntakeShooterEjectAction extends Action{
    
    private enum State {
        Idle,
        ReadyToEject,
        Eject,
        Stow
    }

    private IntakeShooterSubsystem sub_;
    private MCMotionMagicAction eject_updown_;
    private MCMotionMagicAction eject_tilt_;
    private MotorEncoderPowerAction eject_feeder_;
    private MCVelocityAction eject_shooter1_;
    private MCVelocityAction eject_shooter2_;
    private XeroTimer timer_;
    private State state_;
    private IntakeShooterStowAction stow_intake_shooter_;

    public IntakeShooterEjectAction(IntakeShooterSubsystem sub) throws Exception {

        super(sub.getRobot().getMessageLogger());

        state_ = State.Idle;

        sub_ = sub;
        eject_updown_ = new MCMotionMagicAction(sub_.getUpDown(), "pids:position" , "targets:stow" , 0.5 , 1);
        eject_tilt_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position" , "targets:stow" , 0.5 , 1);
        eject_feeder_ = new MotorEncoderPowerAction(sub_.getFeeder(), "targets:eject");

        //
        // TODO: these should be not targets:eject but actions:eject:shooter-velocity
        //
        eject_shooter1_ = new MCVelocityAction(sub_.getShooter1(), "pids:position", "targets:eject", 1.0, false);
        eject_shooter2_ = new MCVelocityAction(sub_.getShooter2(), "pids:position", "targets:eject", 1.0, false);
        stow_intake_shooter_ = new IntakeShooterStowAction(sub_);
        timer_ = new XeroTimer(sub.getRobot(), "Eject", 1.5);
    }

    @Override 
    public void start() throws Exception{
        super.start();

        state_ = State.ReadyToEject;
        sub_.getUpDown().setAction(eject_updown_, true);
        sub_.getTilt().setAction(eject_tilt_, true);
    }

    @Override
    public void run() throws Exception {
        super.run();

        switch(state_) {
            case Idle:
                break;
            case ReadyToEject:
                stateReadyToEject();
                break;
            case Eject:
                stateEject();
                break;
            case Stow:
                stateStow();
                break;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "IntakeShooterEjectAction";
    }

    private void stateReadyToEject(){
        if(eject_updown_.isDone() && eject_tilt_.isDone()) {
            state_ = State.Eject;
            sub_.getFeeder().setAction(eject_feeder_, true);
            sub_.getShooter1().setAction(eject_shooter1_, true);
            sub_.getShooter2().setAction(eject_shooter2_, true);
            timer_.start();
        }
    }

    private void stateEject() {
        if(timer_.isExpired()) {
            state_ = State.Stow;
            sub_.getFeeder().cancelAction();
            sub_.getShooter1().cancelAction();
            sub_.getShooter2().cancelAction();
            sub_.setAction(stow_intake_shooter_, true);
        }
    }

    private void stateStow(){
        if(stow_intake_shooter_.isDone()){
            state_ = State.Idle;
            setDone();
        }
    }
}
