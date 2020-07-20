package com.miguel.sensor

import com.miguel.control.Pid
import com.miguel.util.Vector
import coppelia.BoolW
import coppelia.FloatWA

class ProximitySensor(val handle: Int) {

    var detectionState = BoolW(false)

    var detectedPoint = FloatWA(3)

    private val pid = Pid(5.0, .0, .0, 3.0)

    fun getState() : Boolean {
        return this.detectionState.value
    }

    private fun getDetectedPoint() : Array<Float> {
        return detectedPoint.array.toTypedArray()
    }

    fun update(robot: Vector, time: Float) : Double {
        val detectedPoint = getDetectedPoint()

        val detectedPointVector = Vector(
                detectedPoint[0].toDouble(),
                detectedPoint[1].toDouble(),
                detectedPoint[2].toDouble()
        )

        return pid.update(robot.distance(detectedPointVector) , time.toDouble())
    }
}