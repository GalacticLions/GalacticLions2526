package org.firstinspires.ftc.team5898.Constants

import com.bylazar.configurables.annotations.Configurable

@Configurable
object CannonConstants {
    @JvmField
    public var IntakePower: Double = 1.0;
    @JvmField
    public var LAUNCH_VELOCITY = 1100.0;
    @JvmField
    public var P: Int = 300;
    @JvmField
    public var I: Double = 0.0;
    @JvmField
    public var D:Double = 0.0;
    @JvmField
    public var F: Int = 10;
}
