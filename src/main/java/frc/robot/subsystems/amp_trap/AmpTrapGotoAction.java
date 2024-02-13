package frc.robot.subsystems.amp_trap;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;

public class AmpTrapGotoAction extends Action {
    private MCMotionMagicAction elevatorThreshAction;
    private MCMotionMagicAction elevatorGotoAction;
    private MCMotionMagicAction wristAction;
    private AmpTrapSubsystem subsystem;
    private double elevatorThreshold;
    private double wristNoGoZoneLower;
    private double wristNoGoZoneUpper;
    private double wristTarget;
    private String state;

    public AmpTrapGotoAction(AmpTrapSubsystem subsystem, String elevatorTarget, String wristTarget) throws Exception{
        this(subsystem, subsystem.getSettingsValue(elevatorTarget).getDouble(), subsystem.getSettingsValue(wristTarget).getDouble());
    }
    public AmpTrapGotoAction(AmpTrapSubsystem subsystem, double elevatorTarget, double wristTarget) throws Exception{
        super(subsystem.getRobot().getMessageLogger());
        this.subsystem = subsystem;

        // Get the elevator threshold and the no-go zone limits for the wrist from the params file
        this.elevatorThreshold = subsystem.getElevator().getSettingsValue("threshold").getDouble();
        this.wristNoGoZoneLower = subsystem.getArm().getSettingsValue("no-go-zone:lower").getDouble();
        this.wristNoGoZoneUpper = subsystem.getSettingsValue("no-go-zone:upper").getDouble();

        this.wristTarget = wristTarget;

        this.elevatorThreshAction = new MCMotionMagicAction(subsystem.getElevator(), "ElevatorThresh", elevatorThreshold, 0.01, 0.02);
        this.elevatorGotoAction = new MCMotionMagicAction(subsystem.getWrist(), "ElevatorThresh", elevatorTarget, 0.01, 0.02);

        // Create the MCMotionMagicAction for the wrist motor
        this.wristAction = new MCMotionMagicAction(subsystem.getWrist(), "WristGoto", wristTarget, 3, 10);

        state = "Thresh";
    }

    @Override
    public void start() throws Exception{
        super.start();
        subsystem.getElevator().setAction(elevatorGotoAction);
        subsystem.getElevator().setAction(elevatorThreshAction);
        subsystem.getWrist().setAction(wristAction);

        // Start the MCMotionMagicActions
        if(wristTarget < wristNoGoZoneLower && wristTarget > wristNoGoZoneUpper){
            subsystem.getWrist().setAction(wristAction);
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
        if(state.equals("Goto") || state.equals("WristGoto")){
            wristAction.run();
        }
        if(state.equals("Thresh")){
            elevatorThreshAction.run();
        }
        // If the elevator has moved up past the elevator threshold, move the wrist to its target position
        if (elevatorThreshAction.isDone() && state.equals("Thresh")){
            subsystem.getWrist().setAction(wristAction);
            state = "WristGoto";
        }

        if(wristAction.isDone()){
            if(elevatorGotoAction.isDone() && state.equals("Goto")){
                setDone();
                return;
            }
            if(state.equals("WristGoto")){
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
        wristAction.cancel();
    }

   @Override
    public String toString(int indent) {
        return prefix(indent) + "AmpTrapGotoAction";
    }
}
