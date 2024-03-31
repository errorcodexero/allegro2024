package frc.robot.automodes.back;

import org.xero1425.base.controllers.AutoController;

import frc.robot.automodes.AllegroGameAutoMode;
import frc.robot.automodes.actions.Start2Shoot2Action;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Start2Shoot2AutoMode extends AllegroGameAutoMode {

    public Start2Shoot2AutoMode(AutoController ctrl, boolean mirror, double mvalue) throws Exception {
        super(ctrl, "2 Note: Center: All Close") ;
        
        AllegroRobot2024 robot = (AllegroRobot2024)ctrl.getRobot().getRobotSubsystem() ;
        Start2Shoot2Action action = new Start2Shoot2Action(robot, mirror, mvalue) ;
        addSubActionPair(robot, action, true);        
    }
}
