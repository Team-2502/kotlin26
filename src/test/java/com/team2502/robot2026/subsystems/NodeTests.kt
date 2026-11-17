package com.team2502.robot2026.subsystems

import com.team2502.robot2026.control.autonomous.Nodes.*
import com.team2502.robot2026.control.autonomous.Nodes.Companion.findPath
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.assertEquals

class NodeTests {
    @Test
    fun `pathfinder test`() {
        val actual = findPath(A,D)
        val expected = listOf(A, C, D)
        println("[TESTS]: expected: $expected, actual: $actual")
        assertEquals(expected,actual)
    }
}