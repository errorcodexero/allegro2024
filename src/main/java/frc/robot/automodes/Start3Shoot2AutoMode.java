package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;

import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Start3Shoot2AutoMode extends AllegroGameAutoMode {
    public Start3Shoot2AutoMode(AutoController ctrl, boolean mirror, double mvalue) throws Exception {
        super(ctrl, "Start3-Shoot2", mirror, mvalue) ;

        AllegroRobot2024 robot = (AllegroRobot2024)ctrl.getRobot().getRobotSubsystem() ;
        Start3Shoot2Action action = new Start3Shoot2Action(robot, mirror, mvalue) ;
        addSubActionPair(robot, action, true);
    }
}
