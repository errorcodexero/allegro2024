package frc.robot.subsystems.amp_trap;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;

public class PlaceAction extends Action{
    private AmpTrapSubsystem sub_;
    
    private MCVelocityAction place_;

    private MCVelocityAction stop_;

    private XeroTimer timer_;

    public PlaceAction (AmpTrapSubsystem sub) throws Exception{
        super(sub.getRobot().getMessageLogger());

        sub_ = sub;

        place_ = new MCVelocityAction(sub.getManipulator(), "pids:position", "targets:place", 1.0, false);
        stop_ = new MCVelocityAction(sub.getManipulator(), "pids:position", "targets:stop", 1.0, false);

        timer_ = new XeroTimer(sub_.getRobot(), "Place", 1);
    }

    public void start() throws Exception{
        super.start();

        sub_.getManipulator().setAction(place_, true);

        timer_.start();
    }

    public void run() throws Exception{
        super.run();

        if(timer_.isExpired()){
            sub_.getManipulator().setAction(stop_);
            setDone();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "PlaceAction";
    }
}
