package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;

public class IntakeShooterEjectAction extends Action{
    
    private IntakeShooterSubsystem sub_;
    private MCMotionMagicAction eject_updown_;
    private MCMotionMagicAction eject_tilt_;
    private MCVelocityAction eject_feeder_;
    private MCVelocityAction eject_shooter1_;
    private MCVelocityAction eject_shooter2_;
    private MCVelocityAction stop_shooter1_;
    private MCVelocityAction stop_shooter2_;
    private XeroTimer timer_;

    public IntakeShooterEjectAction(IntakeShooterSubsystem sub) throws Exception {

        super(sub.getRobot().getMessageLogger());


        sub_ = sub;
        eject_updown_ = new MCMotionMagicAction(sub_.getUpDown(), "pids:position" , "targets:stow" , 0.5 , 1);
        eject_tilt_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position" , "targets:stow" , 0.5 , 1);
        eject_feeder_ = new MCVelocityAction(sub_.getFeeder(), "pids:position" , "targets:eject");
        eject_shooter1_ = new MCVelocityAction(sub_.getShooter1(), "pids:position", "targets:eject");
        eject_shooter2_ = new MCVelocityAction(sub_.getShooter2(), "pids:position", "targets:eject");
        stop_shooter1_ = new MCVelocityAction(sub_.getShooter1(), "pids:position", "targets:stop");
        stop_shooter2_ = new MCVelocityAction(sub_.getShooter2(), "pids:position", "targets:stop");
        timer_ = new XeroTimer(sub.getRobot(), "Eject", 1.5);
    }

    @Override 
    public void start() throws Exception{
        super.start();

        sub_.getShooter1().setAction(eject_shooter1_, true);
        sub_.getShooter2().setAction(eject_shooter2_, true);
        timer_.start();
    }

    @Override
    public void run() throws Exception {
        super.run() ;
        boolean step1= false;
        if(timer_.isExpired() && !step1){
            sub_.getUpDown().setAction(eject_updown_, true);
            sub_.getTilt().setAction(eject_tilt_, true);
            sub_.getFeeder().setAction(eject_feeder_, true);
            sub_.getShooter1().setAction(stop_shooter1_, true);
            sub_.getShooter2().setAction(stop_shooter2_, true);
            step1 = true;
        }
        if (eject_updown_.isDone() && eject_tilt_.isDone() && eject_feeder_.isDone()) {
            setDone();
        }

    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "IntakeShooterEjectAction";
    }
}
