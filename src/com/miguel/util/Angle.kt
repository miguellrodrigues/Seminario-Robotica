package com.miguel.util

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

object Angle {

    fun normalizeRadian(radian: Double): Double {
        return atan2(sin(radian), cos(radian))
    }

    fun getCircumferencePoints(center: Vector, radius: Double): ArrayList<Vector> {
        val points = ArrayList<Vector>()

        // val radiusConstant = radius / 360

        for (i in 0..360) {
            val radians = Math.toRadians(i.toDouble())

            // val r = radiusConstant * i

            val x = (radius * cos(radians))
            val y = (radius * sin(radians))

            val point = Vector(x, y, 0.0)

            center.add(point)

            points.add(center.clone())

            center.subtract(point)
        }

        return points
    }
}