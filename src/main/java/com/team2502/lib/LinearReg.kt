package com.team2502.lib

import edu.wpi.first.math.geometry.Pose2d
import kotlin.math.sqrt

private class PoseTime(val pose: Pose2d = Pose2d(), val time: Long = 0)

private class LinearRegression() {
    var n = 0.0
    var sx = 0.0
    var sy = 0.0

    var sxx = 0.0
    var sxy = 0.0
    var syy = 0.0

    var vx = 0.0
    var vy = 0.0

    public fun add(x: Double, y: Double) {
        sx += x
        sy += y

        sxx += x*x
        sxy += x*y
        syy += y*y
    }

    public fun fit() {
        val invN = 1.0 / n
        val fx = sx*invN
        val fy = sy*invN
        val fxx = sxx * invN - fx * fx
        val fxy = sxy * invN - fx * fy
        val fyy = syy * invN - fy * fy
        val d = fxx - fyy
        val r = sqrt(d*d + 4.0*fxy*fxy)
        if (d >= 0.0) {
            vx = d+r
            vy = 2.0*fxy
        }
        else {
            vx = 2.0*fxy
            vy = r-d
        }
    }
}
