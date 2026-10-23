# Coroutine Overview

**Write actions in order. Give other tasks time to run. Stop what you start.**

A coroutine is a task that can pause and resume. Calls run in order; a suspending wait pauses this task so others can
continue. Waiting does not stop a motor: it keeps its last command until you change it.

## Waiting

Wait for what the action needs. The `await...` calls below are robot-framework helpers.

| Wait for | Example |
| --- | --- |
| Elapsed time | `delay(100.milliseconds)` |
| New controller data | `awaitDriverStationData()` |
| A button to be held | `awaitDriverStationCondition { joystick.trigger }` |
| The next button press | `awaitDriverStationRisingEdge { joystick.trigger }` |
| The next button release | `awaitDriverStationFallingEdge { joystick.trigger }` |
| A new pose, or return from the task if the wait times out | `val pose = awaitNewPose() ?: return` |

These wait functions will wait for the next event. If you want to handle multiple data sources, consider writing two
loops.

```kt
while (true) {
	awaitDriverStationData() // waits for next driver station packet
	val pose = awaitNewPose() // waits even longer for a new pose
}
```

For a sensor without a wait helper, check it repeatedly, suspending between checks:

```kotlin
// Should almost always have a timeout while waiting for a physical system
withTimeout(2.seconds) {
    while (!shooter.atSpeed()) {
        delay(20.milliseconds)
    }
}
```

When `awaitNewPose()` returns null after its default 100ms timeout, you have several choices for how to handle it:

1. `return`: returns from the containing function, and the function won't be restarted automatically.
2. `if (pose == null) { stopMotors(); continue; }`: disable motors and try again.
3. Briefly disable whatever functionality relies on pose.

`withTimeout` throws a cancellation exception if time runs out; code after it runs only if the wait completes.
A delay alone does not prove a mechanism is ready.

Every long-running loop must give other tasks a chance to run. On a shared robot thread, a loop that never suspends can
prevent other robot tasks from running. A function marked `suspend` does not necessarily pause every time it is called.

## Cancellation and Ownership

Changing modes (auto/teleop/test/disabled) cancels the old mode's task and its children. The background tasks continue
to run regardless of current mode. Cancellation is normal, let it propagate. Unexpected failures are errors, not
ordinary cancellation.

**One active task controls each mechanism**, even when tasks share a file. Coordinate requests through that owner
instead of having several tasks write motor outputs.

Cancellation ends the task, but does not automatically stop its motors. Put cleanup in `finally`: it runs on normal
return, failure, and cooperative cancellation. Keep cleanup short and non-suspending.

## Example: One Action

```kotlin
suspend fun runIntake() {
    try {
        intake.start()
        delay(2.seconds)
    } finally {
        intake.stop()
    }
}
```

The intake runs while this task waits. Cancellation during the delay still calls `finally`.

## Example: Two Independent Actions

Use `launch` when activities need to progress together. Keep child tasks inside `coroutineScope`; it waits for them all
to finish. An infinite child needs cancellation before the scope can finish normally.

```kotlin
suspend fun multitask() {
	coroutineScope { // Waits for all tasks inside of it to finish
		// Both launched tasks share the same robot thread
		launch { // Concurrently: run alongside the driver station loop
			while (true) {
				awaitDriverStationData()
				// Do joystick things
			}
		}
		launch { // Concurrently: run alongside the odometry/pose loop
			try {
				while (true) {
					val pose = awaitNewPose()
					if (pose == null) {
						robotContainer.shooter.stop()
						continue // Try again
					}
					robotContainer.shooter.aimTurret()
				}
			} finally {
				robotContainer.shooter.stop()
			}
		}
		// Waits indefinitely for both launched tasks to finish
	}
}
```

Each child should own and clean up its mechanism. Cancelling `multitask()` cancels both launched children.

## Common Pitfalls

1. `launch` may return before its child starts, so wait for initialization before relying on the launched child's actions.
2. `withTimeout` throws a cancellation exception on timeout, which can end a child task without failing its parent.
3. `cancel()` does not wait for cleanup; `join()` waits for the task to complete, and `cancelAndJoin()` does both.
4. If catching CancellationException, always re-throw it to continue cancelling the coroutine.
5. An uncaught non-CancellationException is designed to shut down the robot code. Catch any exceptions that should be handled.
	* To handle exceptions in a coroutineScope and its children, wrap `coroutineScope` in a try-catch-finally instead of `launch`.
    * `launch` only launches the task, a try-catch around it won't have the intended effect.
