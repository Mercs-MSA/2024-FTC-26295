package teamcode.v1.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.Cmd
import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.commands.LoopCmd
import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.logger.LoggerConfig
import com.asiankoala.koawalib.math.NVector
import com.asiankoala.koawalib.subsystem.odometry.Odometry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import teamcode.v1.Robot
import teamcode.v1.commands.sequences.DepositSequence
import teamcode.v1.commands.sequences.HomeSequence
import teamcode.v1.commands.subsystems.ClawCmds
import teamcode.v1.constants.ArmConstants
import teamcode.v1.constants.ClawConstants
import teamcode.v1.constants.LiftConstants
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
open class KTeleOp() : KOpMode(photonEnabled = true) {
    private val robot by lazy { Robot(Odometry.lastPose) }
    private var slowMode = false

    override fun mInit() {
        Logger.config = LoggerConfig.DASHBOARD_CONFIG
        scheduleDrive()
        scheduleCycling()
//        scheduleTest()
    }

    private fun scheduleDrive() {
        robot.drive.defaultCommand = object : Cmd() {
            val fastScalars = NVector(0.9, 0.9, 0.75)
            val slowScalars = NVector(0.4, 0.4, 0.4)
            val scalars get() = if(slowMode) slowScalars else fastScalars

            private fun joystickFunction(s: Double, k: Double, x: Double): Double {
                return max(0.0, s * x * (k * x.pow(3) - k + 1)) * x.sign
            }

            override fun execute() {
                val raws = NVector(
                    driver.leftStick.xSupplier.invoke(),
                    -driver.leftStick.ySupplier.invoke(),
                    -driver.rightStick.xSupplier.invoke()
                )

                robot.drive.powers = raws
                    .mapIndexed { i, d -> joystickFunction(scalars[i], 1.0, d) }
                    .asPose
            }

            init {
                addRequirements(robot.drive)
            }
        }

        driver.a.onPress(InstantCmd({ slowMode = !slowMode }))
    }

    private fun scheduleCycling() {
        + object : Cmd() {
            private var armPos = ArmConstants.homePos
                override fun execute() {
                    if(driver.dpadUp.isJustPressed) {
                        armPos += 1.0
                        + LoopCmd({robot.arm.setPos(armPos)})
                    } else if(driver.dpadDown.isJustPressed) {
                        armPos +- 1.0
                        + LoopCmd({robot.arm.setPos(armPos)})
                    } else if(driver.dpadUp.isJustReleased || driver.dpadDown.isJustReleased) {
                        armPos += 0.0
                        + InstantCmd({robot.arm.setPos(armPos)})
                    }
                    }
                    }

        driver.rightBumper.onPress(HomeSequence(robot.lift, robot.claw, robot.arm, ArmConstants.intervalPos, ArmConstants.groundPos))
        driver.leftBumper.onPress(DepositSequence(robot.lift, robot.arm, robot.claw, ArmConstants.highPos, LiftConstants.highPos))
        driver.leftTrigger.onPress(ClawCmds.ClawCloseCmd(robot.claw))
        driver.y.onPress(DepositSequence(robot.lift, robot.arm, robot.claw, ArmConstants.midPos, LiftConstants.midPos))
        driver.x.onPress(DepositSequence(robot.lift, robot.arm, robot.claw, ArmConstants.lowPos, LiftConstants.lowPos))
        driver.rightTrigger.onPress(ClawCmds.ClawOpenCmd(robot.claw))
        gunner.leftBumper.onPress(InstantCmd({robot.lift.setPos(-15.5)}))
        gunner.rightBumper.onPress(InstantCmd({robot.arm.setPos(-200.0)}))
    }

    private fun scheduleTest() {
        driver.leftBumper.onPress(InstantCmd({robot.arm.setPos(ArmConstants.highPos)}, robot.arm))
        driver.rightBumper.onPress(InstantCmd({robot.lift.setPos(LiftConstants.highPos)}, robot.lift))
//        driver.leftBumper.onPress(InstantCmd({robot.claw.setPos(ClawConstants.openPos)}))
//        driver.rightBumper.onPress(InstantCmd({robot.claw.setPos(ClawConstants.closePos)}))
        driver.a.onPress(InstantCmd({robot.arm.setPos(-10.0)}, robot.arm))
        driver.b.onPress(InstantCmd({robot.lift.setPos(0.0)}, robot.lift))
    }

    override fun mLoop() {
        Logger.addTelemetryData("arm pos", robot.hardware.armMotor.pos)
        Logger.addTelemetryData("lift pos", robot.hardware.liftLeadMotor.pos)
        Logger.addTelemetryData("arm power", robot.arm.motor.power)
        Logger.addTelemetryData("lift power", robot.hardware.liftLeadMotor.power)
    }
}