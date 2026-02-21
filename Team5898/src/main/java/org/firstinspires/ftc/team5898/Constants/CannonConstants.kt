package org.firstinspires.ftc.team5898.Constants

import com.bylazar.configurables.annotations.Configurable

@Configurable
object CannonConstants {
    @JvmField
    public var IntakePower: Double = 0.3;
    @JvmField
    public var FRONT_LAUNCH_VELOCITY = 1150.0;
    @JvmField
    public var BACK_LAUNCH_VELOCITY = 1515.0;
    @JvmField
    public var kP: Double = 300.0;
    @JvmField
    public var kI: Double = 0.0;
    @JvmField
    public var kD:Double = 0.2;
    @JvmField
    public var kF: Double = 15.0;
    @JvmField
    public var stopperClosePosition: Double = 0.5;
    @JvmField
    public var stopperOpenPosition: Double = 0.8;

}
