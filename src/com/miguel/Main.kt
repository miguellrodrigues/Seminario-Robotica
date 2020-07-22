package com.miguel

import com.miguel.control.Pid
import com.miguel.sensor.ProximitySensor
import com.miguel.util.Angle
import com.miguel.util.Vector
import coppelia.*
import java.util.*
import kotlin.collections.ArrayList
import kotlin.math.PI
import kotlin.properties.Delegates

object Main {

    private const val robot = "ePuck"

    private var state = "maze"
    private var action = "catch"

    private val sim = remoteApi()

    private var clientId by Delegates.notNull<Int>()

    private fun sendCommand(command: String) {
        val options = StringWA(1)
        options.array[0] = command

        sim.simxCallScriptFunction(
                clientId,
                "ePuck",
                1,
                "processCommand",
                IntWA(0),
                FloatWA(0),
                options,
                CharWA(0),
                IntWA(0),
                FloatWA(0),
                StringWA(0),
                CharWA(0),
                remoteApi.simx_opmode_blocking
        )
    }

    private fun getSimulationData(parameter: String): Array<Float> {
        val options = StringWA(1)
        options.array[0] = parameter

        val out: FloatWA = if (parameter == "time") {
            FloatWA(1)
        } else {
            FloatWA(3)
        }

        sim.simxCallScriptFunction(
                clientId,
                "ePuck",
                1,
                "getSimulationData",
                IntWA(0),
                FloatWA(0),
                options,
                CharWA(0),
                IntWA(0),
                out,
                StringWA(0),
                CharWA(0),
                remoteApi.simx_opmode_blocking
        )

        return out.array.toTypedArray()
    }

    data class Victim(
            val position: Vector,
            val handle: Int
    )

    @JvmStatic
    fun main(args: Array<String>) {

        sim.simxFinish(-1)

        println("Conectando...")
        val clientId = sim.simxStart("127.0.0.1", 19999, true, true, 5000, 5)

        this.clientId = clientId

        if (clientId != -1) {
            println("Conectado com sucesso")

            val robotHandle = IntW(1)
            sim.simxGetObjectHandle(clientId, "${robot}_bodyElements", robotHandle, remoteApi.simx_opmode_blocking)

            val rightMotorHandle = IntW(1)
            sim.simxGetObjectHandle(clientId, "${robot}_rightJoint", rightMotorHandle, remoteApi.simx_opmode_blocking)

            val leftMotorHandle = IntW(1)
            sim.simxGetObjectHandle(clientId, "${robot}_leftJoint", leftMotorHandle, remoteApi.simx_opmode_blocking)

            val robotPos = FloatWA(3)
            sim.simxGetObjectPosition(clientId, robotHandle.value, -1, robotPos, remoteApi.simx_opmode_streaming)

            val robotOrientation = FloatWA(3)
            sim.simxGetObjectOrientation(clientId, robotHandle.value, -1, robotOrientation, remoteApi.simx_opmode_streaming)

            val proximitySensors = ArrayList<ProximitySensor>()

            for (i in 1..8) {
                val handle = IntW(1)

                sim.simxGetObjectHandle(clientId, "${robot}_proxSensor$i", handle, remoteApi.simx_opmode_blocking)

                proximitySensors.add(ProximitySensor(handle.value))
            }

            proximitySensors.forEach {
                sim.simxReadProximitySensor(clientId, it.handle, it.detectionState, it.detectedPoint, null, null, remoteApi.simx_opmode_streaming)
            }

            val victimVectors = LinkedList<Victim>()
            val victims = LinkedList<Victim>()

            for (i in 0..10) {
                val handle = IntW(1)

                sim.simxGetObjectHandle(clientId, "Disc$i", handle, remoteApi.simx_opmode_blocking)

                val position = FloatWA(3)
                sim.simxGetObjectPosition(clientId, handle.value, -1, position, remoteApi.simx_opmode_blocking)

                victimVectors.add(
                        Victim(
                                Vector(
                                        position.array[0].toDouble(),
                                        position.array[1].toDouble(),
                                        position.array[2].toDouble()
                                ),
                                handle.value))
            }

            val rescueArea = Vector(-5.0, 1.5, 0.0168)

            synchronized(this) {
                val sort = ArrayList<Victim>()

                victimVectors.forEach {
                    sort.add(it)
                }

                sort.asReversed().sortWith(Comparator.comparingDouble { it.position.distance(rescueArea) })

                var actual = sort.first()

                sort.forEach { _ ->
                    victimVectors.sortWith(Comparator.comparingDouble { it.position.distance(actual.position) })

                    actual = victimVectors.removeFirst()

                    victims.add(actual)
                }
            }

            Thread.sleep(10)

            val running = true

            val vRef = 3.0

            var rightVelocity = vRef
            var leftVelocity = vRef

            val white = 0.8
            val black = 0.5

            val linePID = Pid(1.5, .0, .0, 3.0, 0.0)

            val distancePID = Pid(3.0, 0.5, .0, 5.0, 0.85)
            val anglePID = Pid(5.0, .0, .0, 8.0, 0.0)

            val finish = Vector(-2.5, -1.75, 0.02)

            val rescueAreaPosition = FloatWA(3)
            rescueAreaPosition.array[2] = 0.0333.toFloat()

            var actualVictim = victims.removeFirst()

            val points = Angle.getCircumferencePoints(rescueArea, 0.85)

            var lastPoint = points[(0 until points.size).random()]

            points.shuffle()

            sim.simxStartSimulation(clientId, remoteApi.simx_opmode_oneshot)

            loop@ while (running) {
                sim.simxGetObjectPosition(clientId, robotHandle.value, -1, robotPos, remoteApi.simx_opmode_buffer)

                sim.simxGetObjectOrientation(clientId, robotHandle.value, -1, robotOrientation, remoteApi.simx_opmode_buffer)

                val robotVector = Vector(robotPos.array[0].toDouble(), robotPos.array[1].toDouble(), robotPos.array[2].toDouble())

                when (state) {
                    "maze" -> {
                        val distance = robotVector.distance(finish)

                        if (distance <= 0.4) {
                            state = "rescue"
                            continue@loop
                        }

                        rightVelocity = vRef
                        leftVelocity = vRef

                        val sensors = getSimulationData("lightSensors")

                        var inLine = false

                        sensors.forEach {
                            if (it < 0.5) {
                                inLine = true
                            }
                        }

                        if (inLine) {
                            val normalizedLeft = (sensors[0] - white) / (black - white)
                            val normalizedRight = (sensors[2] - white) / (black - white)

                            val out = linePID.update(-(normalizedLeft - normalizedRight), 0.05)

                            rightVelocity = vRef - out
                            leftVelocity = vRef + out
                        }
                    }

                    "rescue" -> {
                        val distance = if (action == "catch") {
                            robotVector.distance(actualVictim.position)
                        } else {
                            robotVector.distance(rescueArea)
                        }

                        val theta = if (action == "catch") {
                            robotVector.differenceAngle(actualVictim.position)
                        } else {
                            robotVector.differenceAngle(rescueArea)
                        }

                        if (action == "carry") {
                            if (distance <= 0.01) {
                                points.remove(lastPoint)

                                var point = points[(0 until points.size).random()]

                                while (lastPoint.distance(point) <= 0.5)
                                    point = points[(0 until points.size).random()]

                                lastPoint = point

                                rescueAreaPosition.array[0] = point.x.toFloat()
                                rescueAreaPosition.array[1] = point.y.toFloat()

                                sim.simxSetObjectPosition(clientId, actualVictim.handle, -1, rescueAreaPosition, remoteApi.simx_opmode_oneshot)

                                if (victims.isEmpty()) {
                                    sim.simxPauseSimulation(clientId, remoteApi.simx_opmode_blocking)
                                    break@loop
                                }

                                actualVictim = victims.removeFirst()

                                action = "catch"

                                continue@loop
                            } else {
                                robotPos.array[2] = 0.002.toFloat()

                                sim.simxSetObjectPosition(clientId, actualVictim.handle, -1, robotPos, remoteApi.simx_opmode_oneshot)
                            }
                        }

                        if (action == "catch" && distance <= 0.01) {
                            sendCommand("color:${actualVictim.handle}:custom")
                            action = "carry"
                        }

                        val angleOUT = anglePID.update(Angle.normalizeRadian((robotOrientation.array[2] + PI / 2) - theta), 0.05)
                        val distanceOUT = distancePID.update(distance, 0.05)

                        rightVelocity = -angleOUT + distanceOUT
                        leftVelocity = angleOUT + distanceOUT
                    }
                }

                proximitySensors.forEach {
                    sim.simxReadProximitySensor(clientId, it.handle, it.detectionState, it.detectedPoint, null, null, remoteApi.simx_opmode_buffer)

                    if (it.getState()) {
                        when (proximitySensors.indexOf(it)) {
                            0 -> {
                                leftVelocity = vRef
                                rightVelocity = 0.0
                            }

                            1 -> {
                                leftVelocity = vRef
                                rightVelocity = 0.0
                            }

                            2 -> {
                                leftVelocity = vRef
                                rightVelocity = 0.0
                            }

                            3 -> {
                                leftVelocity = 0.0
                                rightVelocity = vRef
                            }

                            4 -> {
                                leftVelocity = 0.0
                                rightVelocity = vRef
                            }

                            5 -> {
                                leftVelocity = 0.0
                                rightVelocity = vRef
                            }

                            6 -> {

                            }

                            7 -> {

                            }

                            else -> {
                            }
                        }
                    }
                }

                sim.simxPauseCommunication(clientId, true)
                sim.simxSetJointTargetVelocity(clientId, rightMotorHandle.value, rightVelocity.toFloat(), remoteApi.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(clientId, leftMotorHandle.value, leftVelocity.toFloat(), remoteApi.simx_opmode_oneshot)
                sim.simxPauseCommunication(clientId, false)
            }

            sim.simxFinish(clientId)

        } else {
            println("Ocorreu um erro ao se conectar")
        }
    }
}
