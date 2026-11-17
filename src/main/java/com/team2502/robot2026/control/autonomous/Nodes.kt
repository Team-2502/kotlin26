package com.team2502.robot2026.control.autonomous

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import java.util.EnumMap
import java.util.PriorityQueue
import javax.swing.UIManager.put
import kotlin.math.PI

/// THESE IDS NEED TO BE UNIQUE - THEY ARE THE IDs USED IN THE DISTANCE ARRAY FOR DJIKSTRA'S
// TODO: make rotation intelligent: can be optional, whitelist, or blacklist
enum class Nodes(val translation: Translation2d, val rotation: Rotation2d) {
    A(Translation2d(0.0, 0.0), Rotation2d(PI)),
    B(Translation2d(1.0, 1.0), Rotation2d(PI)),
    C(Translation2d(1.0, 0.0), Rotation2d(PI)),
    D(Translation2d(3.0, 0.0), Rotation2d(PI));

    fun cost(target: Nodes): Double {
        val distanceDelta = target.translation.getDistance(this.translation)
        val angleDelta = target.rotation - this.rotation

        return distanceDelta
    }

    private val _connections = mutableListOf<Nodes>()
    private fun connectTo(vararg targets: Nodes) {
        for (target in targets) {
            if (target != this && target !in _connections) {
                _connections.add(target)
            }
            if (this != target && this !in target._connections) {
                target._connections.add(this)
            }
        }
    }

    companion object {
        init {
            A.connectTo(B, C)
            A.connectTo(B)
        }

        fun findPath(start: Nodes, goal: Nodes): List<Nodes> {
            val distances = EnumMap<Nodes, Double>(Nodes::class.java).apply {
                entries.forEach { put(it, Double.POSITIVE_INFINITY) }
                put(start, 0.0)
            }
            val cameFrom = EnumMap<Nodes, Nodes>(Nodes::class.java)
            val queue = PriorityQueue<Nodes>(compareBy { distances[it] })
            queue.add(start)

            while (queue.isNotEmpty()) {
                val current = queue.poll() ?: run {
                    println("Queue empty on start")
                    break
                }

                if (current == goal) {
                    return reconstructPath(cameFrom, goal)
                }

                val currentDist = distances[current] ?: Double.POSITIVE_INFINITY

                for (neighbor in current.connections) {
                    val newDist = currentDist + current.cost(neighbor)
                    val currentBestDist = distances[neighbor] ?: Double.POSITIVE_INFINITY

                    if (newDist < currentBestDist) {
                        distances[neighbor] = newDist
                        cameFrom[neighbor] = current

                        // update queue
                        queue.remove(neighbor)
                        queue.add(neighbor)
                    }
                }
            }

            return emptyList()
        }

        private fun reconstructPath(cameFrom: Map<Nodes, Nodes>, goal: Nodes): List<Nodes> {
            val path = mutableListOf(goal)
            var current = goal

            while (cameFrom.containsKey(current)) {
                current = cameFrom.getValue(current)
                path.add(current)
            }

            return path.reversed()
        }
    }

    val connections: List<Nodes>
        get() = _connections
}


