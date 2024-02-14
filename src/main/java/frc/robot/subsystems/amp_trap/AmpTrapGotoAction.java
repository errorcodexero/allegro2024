package frc.robot.subsystems.amp_trap;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;



public class AmpTrapGotoAction extends Action {
    private MCMotionMagicAction elevatorThreshAction;
    private MCMotionMagicAction elevatorGotoAction;
    private MCMotionMagicAction armAction;
    private AmpTrapSubsystem subsystem;
    private double elevatorThreshold;
    private double armNoGoZoneLower;
    private double armNoGoZoneUpper;
    private double armTarget;
    private double elevatorTarget;
    private enum State {
        Goto,
        ThreshGoto,
        ArmGoto,
        Thresh
    };

    private State state;

    public AmpTrapGotoAction(AmpTrapSubsystem subsystem, String elevatorTarget, String armTarget) throws Exception{
        this(subsystem, subsystem.getSettingsValue(elevatorTarget).getDouble(), subsystem.getSettingsValue(armTarget).getDouble());
    }
    public AmpTrapGotoAction(AmpTrapSubsystem subsystem, double elevatorTarget, double armTarget) throws Exception{
        super(subsystem.getRobot().getMessageLogger());
        this.subsystem = subsystem;

        // Get the elevator threshold and the no-go zone limits for the arm from the params file
        this.elevatorThreshold = subsystem.getElevator().getSettingsValue("threshold").getDouble();
        this.armNoGoZoneLower = subsystem.getArm().getSettingsValue("no-go-zone:lower").getDouble();
        this.armNoGoZoneUpper = subsystem.getArm().getSettingsValue("no-go-zone:upper").getDouble();

        this.armTarget = armTarget;
        this.elevatorTarget = elevatorTarget;

        this.elevatorThreshAction = new MCMotionMagicAction(subsystem.getElevator(), "ElevatorThresh", elevatorThreshold, 0.01, 0.02);
        this.elevatorGotoAction = new MCMotionMagicAction(subsystem.getArm(), "ElevatorThresh", elevatorTarget, 0.01, 0.02);

        // Create the MCMotionMagicAction for the arm motor
        this.armAction = new MCMotionMagicAction(subsystem.getArm(), "ArmGoto", armTarget, 3, 10);

        state = State.Thresh;
    }

    @Override
    public void start() throws Exception{
        super.start();
        subsystem.getElevator().setAction(elevatorGotoAction);
        subsystem.getElevator().setAction(elevatorThreshAction);
        subsystem.getArm().setAction(armAction);

        // Start the MCMotionMagicActions
        // If arm target isn't in no-go-zone
        // and the arm target and arm actual aren't on opposite sides
        // then they can be run in paralell
        if(((armTarget < armNoGoZoneLower || armTarget > armNoGoZoneUpper) && (!(armTarget > armNoGoZoneUpper && subsystem.getArm().getPosition() < armNoGoZoneLower) || !(armTarget < armNoGoZoneLower && subsystem.getArm().getPosition() > armNoGoZoneUpper)))){
            subsystem.getArm().setAction(armAction);
            subsystem.getElevator().setAction(elevatorGotoAction);
            state = State.Goto;
        }else if (elevatorTarget > elevatorThreshold){
        // Otherwise, start the 
            subsystem.getElevator().setAction(elevatorGotoAction);
            state = State.ThreshGoto;
        }else{
            subsystem.getElevator().setAction(elevatorThreshAction);
            state = State.Thresh;
        }
    }

    @Override
    public void run() throws Exception{
        super.run();
        // Update the MCMotionMagicActions
        if(state == State.Goto || state == State.ThreshGoto){
            elevatorGotoAction.run(); 
        }
        if(state == State.Goto || state == State.ArmGoto){
            armAction.run();
        }
        if(state == State.Thresh){
            elevatorThreshAction.run();
        }

        if(state == State.ThreshGoto && subsystem.getElevator().getPosition() > elevatorThreshold){
            subsystem.getArm().setAction(armAction);
            state = State.Goto;
        }
        // If the elevator has moved up past the elevator threshold, move the arm to its target position
        if (elevatorThreshAction.isDone() && state == State.Thresh){
            subsystem.getArm().setAction(armAction);
            state =  State.ArmGoto;
        }

        if(armAction.isDone()){
            if(elevatorGotoAction.isDone() && state == State.Goto){
                setDone();
                return;
            }
            if(state == State.ArmGoto){
                subsystem.getElevator().setAction(elevatorGotoAction);
                state = State.Goto;
            }
        }
    }

    @Override
    public void cancel() {
        super.cancel();
        // Cancel the MCMotionMagicActions
        elevatorGotoAction.cancel();
        elevatorThreshAction.cancel();
        armAction.cancel();
    }

   @Override
    public String toString(int indent) {
        return prefix(indent) + "AmpTrapGotoAction";
    }
}
