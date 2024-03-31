package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;

import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Start2Shoot2AutoMode extends AllegroGameAutoMode {

    public Start2Shoot2AutoMode(AutoController ctrl, boolean mirror, double mvalue) throws Exception {
        super(ctrl, "2 Note: Center: All Close") ;
        
        AllegroRobot2024 robot = (AllegroRobot2024)ctrl.getRobot().getRobotSubsystem() ;
        Start2Shoot2Action action = new Start2Shoot2Action(robot, mirror, mvalue) ;
        addSubActionPair(robot, action, true);        
    }
}
