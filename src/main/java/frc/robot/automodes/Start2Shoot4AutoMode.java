package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;

import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Start2Shoot4AutoMode extends AllegroGameAutoMode {

    public Start2Shoot4AutoMode(AutoController ctrl, boolean mirror, double mvalue) throws Exception {
        super(ctrl, "4 Note: Center: All Close") ;

        AllegroRobot2024 robot = (AllegroRobot2024)ctrl.getRobot().getRobotSubsystem() ;
        Start2Shoot4Action action = new Start2Shoot4Action(robot, mirror, mvalue) ;
        addSubActionPair(robot, action, true);
    }    
}
