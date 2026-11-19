package com.team2502.robot2026.subsystems

import com.team2502.robot2026.control.autonomous.Nodes
import com.team2502.robot2026.control.autonomous.Nodes.*
import com.team2502.robot2026.control.autonomous.Nodes.Companion.findPath
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.assertEquals

class NodeTests {
    @Test
    fun `findPath test`() {
        val results = findPath(
            Translation2d(1.0, 1.0),
            Rotation2d(0.0),
            Translation2d(3.0, 7.0),
            Rotation2d(0.0)
        )
        val expected = listOf(A, B, H)
        println("[TESTS]: expected: $expected, actual: $results")
        assertEquals(expected, results)
    }

    @Test
    fun `closestSquaredDistToLine test`() {
        val inputs = listOf(
            Triple(
                Translation2d(0.0, 0.0),
                Translation2d(0.0, 1.0),
                Translation2d(1.0, 2.0)
            ),
            Triple(
                Translation2d(0.0, 0.0),
                Translation2d(-1.0, 2.0),
                Translation2d(0.0, 1.0)
            ),
            Triple(
                Translation2d(0.0, 0.0),
                Translation2d(-1.0, 2.0),
                Translation2d(1.0, 2.0)
            ),
            Triple(
                Translation2d(0.0, 0.0),
                Translation2d(-1.0, 0.0),
                Translation2d(1.0, 0.0)
            ),
            Triple(
                Translation2d(1.0, 1.0),
                Translation2d(-1.0, 1.0),
                Translation2d(1.0, -1.0)
            ),
        )
        val results = mutableListOf<Double>()
        for (input in inputs) {
            results.add(Nodes.closestSquaredDistToLine(input.first, input.second, input.third))
        }

        val expected = listOf(
            1.0,
            1.0,
            4.0,
            0.0,
            2.0,
        )

        for (pair in expected.zip(results)) {
            println("[TESTS]: expected: ${pair.first}, actual: ${pair.second}")
            assertEquals(pair.first, pair.second, 0.05)
        }
    }
//
//    @Test
//    fun `intersectsWall test`() {
//        val inputs = listOf(
//            doubleArrayOf(1.0,1.0,1.0,1.0),
//
//        )
//        val results = mutableListOf<Double>()
//        for (input in inputs) {
//            results.add(Nodes.intersectsWall(input.first, input.second, input.third))
//        }
//
//        val expected = listOf(
//
//        )
//
//        for (pair in expected.zip(results)) {
//            println("[TESTS]: expected: ${pair.first}, actual: ${pair.second}")
//            assertEquals(pair.first, pair.second, 0.05)
//        }
//    }
}