package org.firstinspires.ftc.team5898.Constants

import com.bylazar.configurables.annotations.Configurable

@Configurable
object CannonConstants {
    @JvmField
    public var IntakePower: Double = 1.0;
    @JvmField
    public var FRONT_LAUNCH_VELOCITY = 1100.0;
    @JvmField
    public var BACK_LAUNCH_VELOCITY = 1515.0;
    @JvmField
    public var kP: Int = 300;
    @JvmField
    public var kI: Double = 0.0;
    @JvmField
    public var kD:Double = 0.2;
    @JvmField
    public var kF: Int = 15;
    @JvmField
    public var stopperPosition: Double = 0.51;
}
