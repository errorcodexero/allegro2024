package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.AutoMode;

import frc.robot.subsystems.toplevel.AllegroRobot2024;

//
// Baseline for the automode
//
// Shoot	    0.5
// P1	        3.16
// P2	        2.22
// Shoot        0.5
// P3	        2.28
// P4	        3.18
// Shoot	    0.5
// P5	        1.58
// Shoot	    0.5
// Total       14.42

public class Start1Shoot3AutoMode extends AutoMode {

    public Start1Shoot3AutoMode(AutoController ctrl, boolean mirror, double mvalue) throws Exception {
        super(ctrl, "Start1-Shoot3") ;

        AllegroRobot2024 robot = (AllegroRobot2024)ctrl.getRobot().getRobotSubsystem() ;
        Start1Shoot3Action action = new Start1Shoot3Action(robot, mirror, mvalue) ;
        addSubActionPair(robot, action, true);
    }
}
