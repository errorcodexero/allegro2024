package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;

import frc.robot.subsystems.intake_shooter.IntakeGotoNamedPositionAction;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Start2Shoot2AutoMode extends AllegroGameAutoMode {

    public Start2Shoot2AutoMode(AutoController ctrl, boolean mirror, double mvalue) throws Exception {
        super(ctrl, "Start2-Shoot2", mirror, mvalue) ;
        
        AllegroRobot2024 robot = (AllegroRobot2024)ctrl.getRobot().getRobotSubsystem() ;
        double v1 = robot.getIntakeShooter().getUpDown().getSettingsValue("targets:stow").getDouble() ;
        double v2 = robot.getIntakeShooter().getTilt().getSettingsValue("targets:stow").getDouble() ; 
        IntakeGotoNamedPositionAction stow = new IntakeGotoNamedPositionAction(robot.getIntakeShooter(), v1, v2) ;

        shootFirstNote() ;  
        driveAndCollectClose("S2S2-P1", true, 2.0) ;
        shootFirstNote();
        addSubActionPair(robot.getIntakeShooter(), stow, true);
    }
}
