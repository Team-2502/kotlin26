package com.team2502.lib;

import java.io.File
import java.io.IOException

fun FileZero(
    filePath: File,
    createOperation: () -> Double) : Double {
    val zeroFromFile = filePath.run {
        if (!exists())
            return@run null
        val content = try {
            readText()
        } catch (e: IOException) {
            return@run null
        }
        return@run content.toDouble()
    }

    val result = zeroFromFile ?: createOperation()

    if (zeroFromFile == null) {
        filePath.writeText(result.toString())
    }

    return result
}