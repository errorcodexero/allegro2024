package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;

import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Start3Shoot3AutoMode extends AllegroGameAutoMode {
    public Start3Shoot3AutoMode(AutoController ctrl, boolean mirror, double mvalue) throws Exception {
        super(ctrl, "Start3-Shoot3", mirror, mvalue) ;

        AllegroRobot2024 robot = (AllegroRobot2024)ctrl.getRobot().getRobotSubsystem() ;
        Start3Shoot3Action action = new Start3Shoot3Action(robot, mirror, mvalue) ;
        addSubActionPair(robot, action, true);        
    }
}
