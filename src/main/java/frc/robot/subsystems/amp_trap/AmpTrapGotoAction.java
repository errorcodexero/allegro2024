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
    private String state;

    public AmpTrapGotoAction(AmpTrapSubsystem subsystem, String elevatorTarget, String armTarget) throws Exception{
        this(subsystem, subsystem.getSettingsValue(elevatorTarget).getDouble(), subsystem.getSettingsValue(armTarget).getDouble());
    }
    public AmpTrapGotoAction(AmpTrapSubsystem subsystem, double elevatorTarget, double armTarget) throws Exception{
        super(subsystem.getRobot().getMessageLogger());
        this.subsystem = subsystem;

        // Get the elevator threshold and the no-go zone limits for the arm from the params file
        this.elevatorThreshold = subsystem.getElevator().getSettingsValue("threshold").getDouble();
        this.armNoGoZoneLower = subsystem.getArm().getSettingsValue("no-go-zone:lower").getDouble();
        this.armNoGoZoneUpper = subsystem.getSettingsValue("no-go-zone:upper").getDouble();

        this.armTarget = armTarget;

        this.elevatorThreshAction = new MCMotionMagicAction(subsystem.getElevator(), "ElevatorThresh", elevatorThreshold, 0.01, 0.02);
        this.elevatorGotoAction = new MCMotionMagicAction(subsystem.getArm(), "ElevatorThresh", elevatorTarget, 0.01, 0.02);

        // Create the MCMotionMagicAction for the arm motor
        this.armAction = new MCMotionMagicAction(subsystem.getArm(), "ArmGoto", armTarget, 3, 10);

        state = "Thresh";
    }

    @Override
    public void start() throws Exception{
        super.start();
        subsystem.getElevator().setAction(elevatorGotoAction);
        subsystem.getElevator().setAction(elevatorThreshAction);
        subsystem.getArm().setAction(armAction);

        // Start the MCMotionMagicActions
        if(armTarget < armNoGoZoneLower && armTarget > armNoGoZoneUpper){
            subsystem.getArm().setAction(armAction);
            subsystem.getElevator().setAction(elevatorGotoAction);
            state = "Goto";
        }else{
            subsystem.getElevator().setAction(elevatorThreshAction);
            state = "Thresh";
        }
    }

    @Override
    public void run() throws Exception{
        super.run();
        // Update the MCMotionMagicActions
        if(state.equals("Goto")){
            elevatorGotoAction.run(); 
        }
        if(state.equals("Goto") || state.equals("ArmGoto")){
            armAction.run();
        }
        if(state.equals("Thresh")){
            elevatorThreshAction.run();
        }
        // If the elevator has moved up past the elevator threshold, move the arm to its target position
        if (elevatorThreshAction.isDone() && state.equals("Thresh")){
            subsystem.getArm().setAction(armAction);
            state = "ArmGoto";
        }

        if(armAction.isDone()){
            if(elevatorGotoAction.isDone() && state.equals("Goto")){
                setDone();
                return;
            }
            if(state.equals("ArmGoto")){
                subsystem.getElevator().setAction(elevatorGotoAction);
                state = "Goto";
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
