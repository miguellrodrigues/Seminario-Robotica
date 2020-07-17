package com.miguel

import com.miguel.util.ProximitySensor
import com.miguel.util.Vector
import coppelia.*
import java.util.*
import kotlin.math.abs
import kotlin.properties.Delegates

object Main {

    private const val robot = "ePuck"

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

    private fun getSimulationTime() : Float {
        return getSimulationData("time")[0]
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
            sim.simxGetObjectHandle(clientId, "robot", robotHandle, remoteApi.simx_opmode_blocking)

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

            Thread.sleep(10)

            val running = true

            var startTime: Float
            var endTime = 0F

            var rightVelocity = 3.0
            var leftVelocity = 3.0

            var vref = 3.0

            sim.simxStartSimulation(clientId, remoteApi.simx_opmode_oneshot)

            while (running) {
                startTime = getSimulationTime()

                sim.simxGetObjectPosition(clientId, robotHandle.value, -1, robotPos, remoteApi.simx_opmode_buffer)

                sim.simxGetObjectOrientation(clientId, robotHandle.value, -1, robotOrientation, remoteApi.simx_opmode_buffer)

                val robotVector = Vector(robotPos.array[0].toDouble(), robotPos.array[1].toDouble(), robotPos.array[2].toDouble())

                val elapsedTime = abs(endTime - startTime)

                val sensors = getSimulationData("lightSensors")

                if (sensors[0] < 0.5) {
                    leftVelocity = 0.0
                    rightVelocity = vref + 1
                }

                if (sensors[1] < 0.5) {
                    rightVelocity = vref
                    leftVelocity = vref
                }

                if (sensors[2] < 0.5) {
                    rightVelocity = 0.0
                    leftVelocity = vref + 1
                }

                sim.simxPauseCommunication(clientId, true)
                sim.simxSetJointTargetVelocity(clientId, rightMotorHandle.value, rightVelocity.toFloat(), remoteApi.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(clientId, leftMotorHandle.value, leftVelocity.toFloat(), remoteApi.simx_opmode_oneshot)
                sim.simxPauseCommunication(clientId, false)


                proximitySensors.forEach {
                    sim.simxReadProximitySensor(clientId, it.handle, it.detectionState, it.detectedPoint, null, null, remoteApi.simx_opmode_buffer)

                    /*if (it.getState()) {
                        println(it.detectedPoint.array!!.contentToString())
                    }*/
                }

                endTime = getSimulationTime()
            }

            sim.simxFinish(clientId)

        } else {
            println("Ocorreu um erro ao se conectar")
        }
    }
}
