package com.miguel

import com.miguel.util.Angle
import com.miguel.util.Pid
import com.miguel.util.ProximitySensor
import com.miguel.util.Vector
import coppelia.*
import java.util.*
import kotlin.collections.ArrayList
import kotlin.properties.Delegates

object Main {

    private const val robot = "ePuck"

    private var state = "labirinto"

    private val sim = remoteApi()

    private var clientId by Delegates.notNull<Int>()

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

            val victimVectors = LinkedList<Vector>()
            val victims = LinkedList<Vector>()

            for (i in 0..10) {
                val handle = IntW(1)

                sim.simxGetObjectHandle(clientId, "Disc$i", handle, remoteApi.simx_opmode_blocking)

                val position = FloatWA(3)
                sim.simxGetObjectPosition(clientId, handle.value, -1, position, remoteApi.simx_opmode_blocking)

                victimVectors.add(Vector(
                        position.array[0].toDouble(),
                        position.array[1].toDouble(),
                        position.array[2].toDouble()
                ))
            }

            synchronized(this) {
                val sort = ArrayList<Vector>()

                victimVectors.asReversed().forEach {
                    sort.add(it)
                }

                sort.sortWith(Comparator.comparingDouble { it.distance(victimVectors[3]) })

                var actual = sort.first()

                sort.forEach { _ ->
                    victimVectors.sortWith(Comparator.comparingDouble { value -> value.distance(actual) })

                    actual = victimVectors.removeFirst()

                    victims.add(actual)
                }
            }

            Thread.sleep(10)

            val running = true

            val vRef = 4.0

            var rightVelocity = vRef
            var leftVelocity = vRef

            val white = 0.8
            val black = 0.5

            sim.simxStartSimulation(clientId, remoteApi.simx_opmode_oneshot)

            val linePID = Pid(1.35, 0.09, 0.0, 4.0)

            val distancePID = Pid(1.0, .1, .1, 8.0)
            val anglePID = Pid(5.0, 0.0, 0.0, 8.0)

            val finish = Vector(-2.5, -1.75, 0.02)
            var actualVictim = victims.first()

            val rescueArea = Vector(-3.7250, -3.5750, 0.01)

            var action = "pegar"

            loop@ while (running) {
                sim.simxGetObjectPosition(clientId, robotHandle.value, -1, robotPos, remoteApi.simx_opmode_buffer)

                sim.simxGetObjectOrientation(clientId, robotHandle.value, -1, robotOrientation, remoteApi.simx_opmode_buffer)

                val robotVector = Vector(robotPos.array[0].toDouble(), robotPos.array[1].toDouble(), robotPos.array[2].toDouble())

                when (state) {
                    "labirinto" -> {
                        val distance = robotVector.distance(finish)

                        if (distance <= 0.4) {
                            state = "resgate"
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

                            val out = linePID.update(0 - (normalizedLeft - normalizedRight), 0.05)

                            rightVelocity = vRef - out
                            leftVelocity = vRef + out
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
                    }

                    "resgate" -> {
                        val distance = robotVector.distance(rescueArea)

                        val theta = robotVector.differenceAngle(rescueArea)

                        val angleOUT = anglePID.update(Angle.normalizeRadian(robotOrientation.array[2] - theta), 0.05)
                        val distanceOUT = distancePID.update(distance - 0.03, 0.05)

                        rightVelocity = -angleOUT + distanceOUT
                        leftVelocity = angleOUT + distanceOUT
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
