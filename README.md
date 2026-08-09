# Robot Guidance Challenge

An HCS12-based `eebot` mobile robot, programmed in assembly, that learns and solves an unknown line-tracked maze — then retraces its path back to the start. Built for TMU's COE538 Microprocessor Systems.

**Result: solved.** The robot successfully completed the full round trip, storing the correct direction at each intersection in an array and retracing the maze error-free on the return leg — the only student in the class to get the robot all the way back to the starting point.

## The Challenge

- The robot starts at the maze entry point and follows a guidance line.
- At every intersection (only L/T junctions — max 2 choices), it picks a branch.
- If a branch dead-ends (detected via the front bumper), the robot executes a 180° turn, retraces to the last intersection, and tries the other branch — noting that the first choice was wrong.
- This continues until the robot reaches the forward destination (signaled by the operator tapping the rear bumper).
- The robot then retraces the *entire* maze back to the start, using what it learned, taking the correct branch at every intersection with no further errors.

Full rules and background: [`project.pdf`](./project.pdf) (original assignment brief, Ryerson/TMU COE538).

## Approach

The maze solution is stored as an array — one entry per intersection, holding the correct direction to take there. On the outbound run, a wrong branch gets corrected and overwritten in the array the moment the robot hits a dead end and backtracks; a correct branch is left as-is. By the time the robot reaches the forward destination, the array holds a complete, verified path, which the return leg just walks in order.

## Repository Layout

Built incrementally, one testable subsystem at a time, rather than as a single program written start to finish:

```
Sources/
├── main.asm                        — top-level program, integrates all subsystems
├── intersectionDetectionTest.asm    — standalone test: detecting an intersection
├── testTurnCalculation.asm           — standalone test: computing turn direction from current heading
├── testLeftTurn.asm / testRightTurn.asm  — individual turn execution routines
├── testIntersectionRemeberance.asm    — standalone test: recording/recalling intersection decisions
├── navigationStorageMethod.asm         — the maze-solution array data structure
├── testGetExits.asm                    — standalone test: reading available exits at a junction
└── projectTurnLeftOnly.asm              — reduced-scope variant for isolated bench testing
```

Each `test*.asm` file is a standalone, independently runnable/debuggable version of one subsystem — built and verified in isolation before being integrated into `main.asm`. This follows the assignment's own recommended strategy: decompose a large assembly project into small, independently testable pieces rather than attempting it as one continuous build.

## Notes

- Speed required tuning — line-tracking responsiveness had to be balanced against reliable, error-free navigation.
- Target hardware: HCS12 (Freescale/NXP), built in CodeWarrior.
