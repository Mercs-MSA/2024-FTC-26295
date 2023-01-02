package org.firstinspires.ftc.teamcode.koawalib.subsystems

import com.asiankoala.koawalib.hardware.motor.KMotor
import com.asiankoala.koawalib.subsystem.Subsystem

class Arm(val motor: KMotor) : Subsystem() {
    fun setPos(pos: Double) {
        motor.setPositionTarget(pos)
    }
}