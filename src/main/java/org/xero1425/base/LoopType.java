package org.xero1425.base  ;

/// \file

/// \brief This type gives the loop type for th the robot
public enum LoopType
{
    Initialization,           ///< No loop type
    Teleop,                   ///< Operator Control mode
    Autonomous,               ///< Auto mode
    Test,                     ///< Test mode, not really supported by the framework
    Disabled                  ///< Disable mode
} ;
