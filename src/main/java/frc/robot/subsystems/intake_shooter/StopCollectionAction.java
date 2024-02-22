package frc.robot.subsystems.intake_shooter;

public class StopCollectionAction extends CollectBaseAction {

    public StopCollectionAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        startStow();
    }

    @Override
    public void run() throws Exception {
        super.run() ;
        runStow();
    }

    @Override
    public void cancel() {
        super.cancel() ;
    }

    public String toString(int indent) {
        return spaces(indent) + "ButchStopCollectAction" ;
    }       
}