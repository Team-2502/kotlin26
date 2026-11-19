package com.team2502.robot2026.subsystems

import com.team2502.robot2026.control.autonomous.Nodes
import com.team2502.robot2026.control.autonomous.Nodes.*
import com.team2502.robot2026.control.autonomous.Nodes.Companion.findPath
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import org.junit.jupiter.api.Assertions.assertArrayEquals
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.assertEquals

class NodeTests {
    fun testPath(start: Pair<Translation2d, Rotation2d>, goal: Pair<Translation2d, Rotation2d>, nodeList: List<Nodes>) {
        val results = findPath(
            start.first, start.second,
            goal.first, goal.second,
        )
        val expected = nodeList.map {
            Pair(it.translation, it.rotation)
        }.toMutableList()
        expected.add(0, start)
        expected.add(goal)
        assertArrayEquals(expected.toTypedArray(), results.toTypedArray()) {
            val expectedOut = expected.map {"(${it.first.x}, ${it.first.y}, ${it.second.degrees})"}
            val resultsOut = results.map {"(${it.first.x}, ${it.first.y}, ${it.second.degrees})"}
            "expected: $expectedOut, results: $resultsOut"
        }
    }

    @Test
    fun `findPath test`() {
        testPath(
            Pair(Translation2d(2.0, 1.0), Rotation2d()),
            Pair(Translation2d(7.0, 7.0), Rotation2d()),
            listOf(E, G)
        )
        testPath(
            Pair(Translation2d(1.0, 1.0), Rotation2d()),
            Pair(Translation2d(2.0, 1.0), Rotation2d()),
            listOf()
        )
        testPath(
            Pair(Translation2d(2.0, 1.0), Rotation2d()),
            Pair(Translation2d(7.0, 7.5), Rotation2d()),
            listOf(E)
        )
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
}