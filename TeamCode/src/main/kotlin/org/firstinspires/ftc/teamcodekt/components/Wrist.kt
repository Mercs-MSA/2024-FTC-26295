package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.hardware.SimpleServo
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import org.firstinspires.ftc.teamcodekt.components.meta.DeviceNames

@Config
object WristConfig {
    @JvmField var FORWARDS  = 0.8275
    @JvmField var BACKWARDS = 0.171
    @JvmField var REST      = 0.5
}

class Wrist {
    private val wristServo = SimpleServo(hwMap, DeviceNames.WRIST_SERVO, 0.0, 180.0)

    var wristPosition = 0.0

    fun setToBackwardsPos() {
        wristPosition = WristConfig.BACKWARDS
    }

    fun setToForwardsPos() {
        wristPosition = WristConfig.FORWARDS
    }

    fun setToRestingPos() {
        wristPosition = WristConfig.REST
    }

    fun update() {
        wristServo.position = wristPosition
    }
}