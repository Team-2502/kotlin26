package com.team2502.robot2026.control.autonomous

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import java.util.EnumMap
import java.util.PriorityQueue
import javax.swing.UIManager.put
import kotlin.math.PI

// TODO: make rotation intelligent: can be optional, whitelist, or blacklist
enum class Nodes(val translation: Translation2d, val rotation: Rotation2d) {
    A(Translation2d(1.673, 2.582), Rotation2d(PI)),
    B(Translation2d(3.462, 0.644), Rotation2d(PI)),
    C(Translation2d(1.673, 4.909), Rotation2d(PI)),
    D(Translation2d(1.253, 7.063), Rotation2d(PI)),
    E(Translation2d(3.462, 7.426), Rotation2d(PI)),
    F(Translation2d(3.462, 4.035), Rotation2d(PI)),
    G(Translation2d(5.789, 7.426), Rotation2d(PI)),
    H(Translation2d(5.789, 0.644), Rotation2d(PI)),
    I(Translation2d(10.752, 0.644), Rotation2d(PI)),
    J(Translation2d(10.752, 7.426), Rotation2d(PI)),
    K(Translation2d(13.079, 7.426), Rotation2d(PI)),
    L(Translation2d(13.079, 4.035), Rotation2d(PI)),
    M(Translation2d(13.079, 0.644), Rotation2d(PI)),
    N(Translation2d(14.868, 5.487), Rotation2d(PI)),
    O(Translation2d(14.868, 3.16), Rotation2d(PI)),
    P(Translation2d(15.288, 1.006), Rotation2d(PI));

    fun cost(target: Nodes): Double {
        val distanceDelta = target.translation.getDistance(this.translation)
        val angleDelta = target.rotation - this.rotation

        return distanceDelta
    }

    private val _connections = mutableSetOf<Nodes>()
    val connections: Set<Nodes>
        get() = _connections

    private fun connectTo(vararg targets: Nodes) {
        for (target in targets) {
            if (this != target) {
                if (!this._connections.add(target))
                    println("[NODES WARNING]: connectTo: this already in target.connections: this: $this target: $target")
                target._connections.add(this)
            } else {
                println("[NODES WARNING]: connectTo: node told connect to itself: this: $this, target: $target")
            }
        }
    }

    companion object {
        init {
            A.connectTo(B, F, C, E)
            C.connectTo(B, E)
            D.connectTo(B, C, E, F)
            F.connectTo(B, C, E)
            G.connectTo(E, J, I)
            H.connectTo(B, G, J, I)
            I.connectTo(J, M)
            J.connectTo(K)
            K.connectTo(N, L, O, P)
            L.connectTo(N, P)
            M.connectTo(L, P, N)
            O.connectTo(K, L, M, N, P)
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
                    println("[NODE WARNING]: findPath: Queue empty on start")
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
}


