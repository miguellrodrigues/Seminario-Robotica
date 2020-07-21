package com.miguel.control

class Pid(private val kp: Double, private val ki: Double, private val kd: Double, private val saturation: Double) {

    private var error = 0.0

    private var oldError = 0.0
    private var accumulator = 0.0
    private var out = 0.0

    fun update(error: Double, time: Double): Double {
        oldError = this.error
        this.error = error

        val proportional = kp * error
        val integral = ki * accumulator
        val derivative = kd * (error - oldError) / time

        val out = proportional + integral + derivative

        if (saturation > out && out > -saturation) {
            accumulator += (error + oldError) / 2 * time
        }

        this.out = out

        return out
    }
}