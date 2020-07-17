package com.miguel.util

class Pid(private val kp: Double, private val ki: Double, private val kd: Double, private val saturation: Double) {

    private var error = 0.0

    private var oldError = 0.0
    private var accumulator = 0.0
    private var out = 0.0

    fun update(error: Double, time: Double): Double {
        oldError = this.error
        this.error = error

        if (saturation > out) {
            accumulator += (error + oldError) / 2 * time
        }

        val proportional = kp * error
        val integral = ki * accumulator
        val derivative = kd * (error - oldError) / time

        val out = proportional + integral + derivative

        this.out = out

        return out
    }
}