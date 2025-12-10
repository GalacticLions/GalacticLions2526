package org.firstinspires.ftc.team5898.Constants

import com.bylazar.configurables.annotations.Configurable

@Configurable
object CannonConstants {
    @JvmField
    public var IntakePower: Double = 1.0;
    @JvmField
    public var LaunchPower: Double = 0.9;
    @JvmField
    public var LaunchPower_Reduced: Double = 0.7;
    @JvmField
    public var LAUNCH_VELOCITY = 1500.0;
    @JvmField
    public var LAUNCH_VELOCITY_ALT = 1950.0;

}
