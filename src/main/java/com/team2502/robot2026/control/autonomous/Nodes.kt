package com.team2502.robot2026.control.autonomous

import com.team2502.robot2026.Constants.Auto.PATHFINDING_MIN_DISTANCE_TOLERANCE_METERS
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import org.jetbrains.annotations.VisibleForTesting
import java.util.EnumMap
import java.util.PriorityQueue
import kotlin.math.PI
import kotlin.math.max
import kotlin.math.min

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

    fun cost(target: Nodes, goal: Translation2d, goalYaw: Rotation2d): Double {
        val distanceDelta = target.translation.getDistance(this.translation)
        val yawDelta = target.rotation - this.rotation

        val goalDistanceDelta = target.translation.getDistance(goal)
        val goalYawDelta = target.rotation - this.rotation

        return distanceDelta + goalDistanceDelta
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
        val objectEdges = listOf(
            Pair(Translation2d(4.029, 6.782), Translation2d(5.222, 6.782)),
            Pair(Translation2d(5.222, 6.782), Translation2d(5.222, 1.287)),
            Pair(Translation2d(5.222, 1.287), Translation2d(4.029, 1.287)),
            Pair(Translation2d(4.029, 1.287), Translation2d(4.029, 6.782)),
            Pair(Translation2d(0.0, 6.497), Translation2d(0.0, 5.43)),
            Pair(Translation2d(0.0, 5.43), Translation2d(0.686, 5.43)),
            Pair(Translation2d(0.686, 5.43), Translation2d(0.686, 6.497)),
            Pair(Translation2d(0.686, 6.497), Translation2d(0.0, 6.497)),
            Pair(Translation2d(0.0, 4.343), Translation2d(0.0, 3.149)),
            Pair(Translation2d(0.0, 3.149), Translation2d(1.106, 3.149)),
            Pair(Translation2d(1.106, 3.149), Translation2d(1.106, 4.343)),
            Pair(Translation2d(1.106, 4.343), Translation2d(0.0, 4.343)),
            Pair(Translation2d(11.318, 6.782), Translation2d(12.512, 6.782)),
            Pair(Translation2d(12.512, 6.782), Translation2d(12.512, 1.287)),
            Pair(Translation2d(12.512, 1.287), Translation2d(11.318, 1.287)),
            Pair(Translation2d(11.318, 1.287), Translation2d(11.318, 6.782)),
            Pair(Translation2d(15.435, 4.921), Translation2d(15.435, 3.727)),
            Pair(Translation2d(15.435, 3.727), Translation2d(16.541, 3.727)),
            Pair(Translation2d(16.541, 3.727), Translation2d(16.541, 4.921)),
            Pair(Translation2d(16.541, 4.921), Translation2d(15.435, 4.921)),
            Pair(Translation2d(15.855, 2.639), Translation2d(15.855, 1.573)),
            Pair(Translation2d(15.855, 1.573), Translation2d(16.541, 1.573)),
            Pair(Translation2d(16.541, 1.573), Translation2d(16.541, 2.639)),
            Pair(Translation2d(16.541, 2.639), Translation2d(15.855, 2.639)),
        )

        init {
            A.connectTo(B, C, E)
            C.connectTo(B, E)
            D.connectTo(B, C, E)
            E.connectTo(B)
            G.connectTo(E, J, I)
            H.connectTo(B, G, J, I)
            I.connectTo(J, M)
            J.connectTo(K)
            K.connectTo(N, P)
            M.connectTo(P, N, K)
            O.connectTo(K, M, N, P)
        }

        @VisibleForTesting
        internal fun closestSquaredDistToLine(
            point: Translation2d,
            lineStart: Translation2d,
            lineEnd: Translation2d
        ): Double {
            val lineVector = lineEnd - lineStart
            val lengthSquared = lineVector.squaredNorm
            if (lengthSquared == 0.0) {
                return (point - lineStart).squaredNorm // line is actually a point -> shortest distance is a line
            }
            val projected =
                lineStart + lineVector * min(1.0, max(0.0, (point - lineStart).dot(lineVector) / lengthSquared))
            return (point - projected).squaredNorm
        }

        @VisibleForTesting
        internal fun intersectsWall(
            pathStart: Translation2d,
            pathEnd: Translation2d,
            wallStart: Translation2d,
            wallEnd: Translation2d,
            minDistance: Double
        ): Boolean {
            val minDistanceSquared = minDistance * minDistance
            if (closestSquaredDistToLine(pathStart, wallStart, wallEnd) <= minDistanceSquared) return true
            if (closestSquaredDistToLine(pathEnd, wallStart, wallEnd) <= minDistanceSquared) return true
            if (closestSquaredDistToLine(wallStart, pathStart, pathEnd) <= minDistanceSquared) return true
            if (closestSquaredDistToLine(wallEnd, pathStart, pathEnd) <= minDistanceSquared) return true

            val pathVec = pathEnd - pathStart
            val objectVec = wallEnd - wallStart

            val o1 = pathVec.cross(wallStart - pathStart)
            val o2 = pathVec.cross(wallEnd - pathStart)
            val o3 = objectVec.cross(pathStart - wallStart)
            val o4 = objectVec.cross(pathEnd - wallStart)

            return (((o1 > 0) and (o2 < 0)) or ((o1 < 0) and (o2 > 0))) and (((o3 > 0) and (o4 < 0)) or ((o3 < 0) and (o4 > 0)))
        }

        fun isStraightLinePossible(
            start: Translation2d,
            end: Translation2d,
            minDistance: Double = PATHFINDING_MIN_DISTANCE_TOLERANCE_METERS
        ): Boolean {
            for (edge in objectEdges) {
                if (intersectsWall(start, end, edge.first, edge.second, minDistance)) return false
            }
            return true
        }

        fun findPath(
            start: Translation2d,
            startYaw: Rotation2d,
            goal: Translation2d,
            goalYaw: Rotation2d
        ): List<Pair<Translation2d, Rotation2d>> {
            if (isStraightLinePossible(start, goal)) {
                return listOf(Pair(start, startYaw), Pair(goal, goalYaw))
            }

            val distances = EnumMap<Nodes, Double>(Nodes::class.java).apply {
                Nodes.entries.forEach { put(it, Double.POSITIVE_INFINITY) }
            }
            val cameFrom = EnumMap<Nodes, Nodes>(Nodes::class.java)
            val queue = PriorityQueue<Nodes>(compareBy { distances[it] })
            for (node in Nodes.entries) {
                if (isStraightLinePossible(start, node.translation)) {
                    distances[node] = start.getDistance(node.translation)
                    queue.add(node)
                }
            }

            println("queue: $queue")

            while (queue.isNotEmpty()) {
                val current = queue.poll() ?: run {
                    println("[NODE WARNING]: findPath: Queue empty on start")
                    break
                }

                if (isStraightLinePossible(current.translation, goal)) {
                    return reconstructPath(cameFrom, current, start, startYaw, goal, goalYaw)
                }

                val currentDist = distances[current] ?: Double.POSITIVE_INFINITY

                for (neighbor in current.connections) {
                    val newDist = currentDist + current.cost(neighbor, goal, goalYaw)
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

        private fun reconstructPath(
            cameFrom: Map<Nodes, Nodes>,
            lastNode: Nodes,
            start: Translation2d,
            startYaw: Rotation2d,
            goal: Translation2d,
            goalYaw: Rotation2d
        ): List<Pair<Translation2d, Rotation2d>> {
            val path = mutableListOf(Pair(goal, goalYaw), Pair(lastNode.translation, lastNode.rotation))
            var current = lastNode

            while (cameFrom.containsKey(current)) {
                current = cameFrom.getValue(current)
                path.add(Pair(current.translation, current.rotation))
            }

            path.add(Pair(start, startYaw))
            return path.reversed()
        }
    }
}


