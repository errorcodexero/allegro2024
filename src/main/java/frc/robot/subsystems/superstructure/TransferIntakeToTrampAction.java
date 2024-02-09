package frc.robot.subsystems.superstructure;

import org.xero1425.base.actions.Action;

public class TransferIntakeToTrampAction extends Action {
    SuperStructureSubsystem sub_ ;

    public TransferIntakeToTrampAction(SuperStructureSubsystem sub) {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;
    }
    @Override
    public void start() throws Exception {
    }

    @Override
    public void run() throws Exception {
    }

    @Override
    public void cancel() {
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "TransferIntakeToTrampAction";
    }
}
