package org.firstinspires.ftc.team5898.Constants

import androidx.core.graphics.component3
import com.bylazar.configurables.annotations.Configurable

@Configurable
object SlideConstants {
    @JvmField
    public var ErrorThreshold: Int = 5
//    @JvmField
//    public var maxheightL: Int = 4000
//    @JvmField
//    public var maxheightR: Int = 4000
    @JvmField
    public var movePower: Double = 0.7
    @JvmField
    public var holdPower: Double = 0.3
    @JvmField
    //Offset in Encoder Ticks
    public var Offset:Int = 0
}
