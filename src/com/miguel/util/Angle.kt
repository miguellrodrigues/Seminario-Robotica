package com.miguel.util

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

object Angle {

    fun normalizeRadian(radian: Double) : Double {
        return atan2(sin(radian), cos(radian))
    }
}