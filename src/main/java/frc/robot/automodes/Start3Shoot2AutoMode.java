package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;

import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Start3Shoot2AutoMode extends AllegroGameAutoMode {
    static final String[] titles = new String[] { 
        "Source Side Shoot Two Edge Note",
        "Source Side Shoot Two Inner Note"
    } ;

    public Start3Shoot2AutoMode(AutoController ctrl, int which, boolean mirror, double mvalue) throws Exception {
        super(ctrl, titles[which]) ;

        AllegroRobot2024 robot = (AllegroRobot2024)ctrl.getRobot().getRobotSubsystem() ;
        Start3Shoot2Action action = new Start3Shoot2Action(robot, which, mirror, mvalue) ;
        addSubActionPair(robot, action, true);        
    }
}
