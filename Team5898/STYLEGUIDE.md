# Team 5898 Code Style Guide

The style guide for the `Team5898` module. If something isn't covered here,
match the code around it.

**Before every push:** `./gradlew :Team5898:assembleDebug` must pass. GitHub
also builds every push to `5898` (the Actions tab). If it goes red, fix it first.

---

## 1. Where code goes

```
team5898/
├── Constants.java        tunable numbers and hardware config names
├── *TeleOp*.java, *Auto*.java   OpModes
├── pedro/                Pedro Pathing config and tuners
└── Archive/              old code, not compiled
```

- Look up hardware with the names in `Constants.Hardware`. Don't type the
  config strings directly in OpModes.
- Don't use `Archive/` from live code. To retire an OpMode, move it there.
- Write new code in Java.
- Pedro Pathing API reference: <https://javadoc.io/doc/com.pedropathing/core/latest/index.html>. Look up
  methods there, not in old tutorials, since many of them are for Pedro 2.x.
- Don't edit `FtcRobotController/`, `build.gradle`, `build.common.gradle`, or
  `build.dependencies.gradle` on your own. These change when we update the FTC
  SDK. Check with the team first, and have one person make the change.

## 2. Formatting

- 4-space indent. Opening braces go on the same line.
- Always use braces. The one exception is a one-line early return:
  `if (isStopRequested()) return;`
- Delete unused imports and commented-out code. Git keeps the history.
- Write comments, docstrings, and commit messages in English.

## 3. Naming

- Classes: `PascalCase`. Methods and variables: `camelCase`.
- Constants: `UPPER_SNAKE`. PID gains are written `kP`, `kI`, `kD`, `kF`.
- Put the unit in the name when it isn't inches or degrees: `getHeadingRad()`,
  `timeoutSec`.
- **Exception: Pedro `Pose` headings are in radians.** Always write them with
  `Math.toRadians(...)` so it's obvious: `new Pose(24, 48, Math.toRadians(90))`.
  Writing `new Pose(24, 48, 90)` turns the robot to 90 *radians*, not 90°.
- OpMode `name` in `@TeleOp` / `@Autonomous` must be unique.

## 4. Constants

- No magic numbers in OpModes. Anything you might tune goes in `Constants.java`.
- Don't make tunables `final`, so Panels can change them live.
- Hardware config names (`"fl"`, `"imu"`) only live in `Constants.Hardware`.
  That class is the master list. It must match the Driver Station robot
  configuration exactly, including upper/lower case. A wrong name crashes the
  OpMode at init. If you rename a device in the configuration, update
  `Constants.Hardware` in the same commit.

## 5. Docstrings and comments

Every public class and method gets a short docstring. Write it yourself in
plain words. Don't use AI to generate it.

```java
/**
 * Turns the robot to face the given heading.
 *
 * @param target heading to turn to, in degrees
 * @return how far off we still are, in degrees
 * @author Eli
 */
```

- **Description:** one or two sentences on what it does.
- **`@param`:** one per parameter, with the unit.
- **`@return`:** only if it returns something.
- **`@author`:** optional.
- No HTML, `@see`, or code examples.

**TeleOp classes list their controls in the docstring.** That way the drive
team can learn the controls without reading the code. If you change a button,
update the list in the same commit.

```java
/**
 * Main TeleOp for league matches.
 *
 * gamepad1 (driver):
 *   left stick   drive / strafe
 *   right stick  turn
 *   guide        reset heading
 *
 * gamepad2 (operator):
 *   A            open claw
 *   B            close claw
 *   LB / RB      lift down / up
 *
 * @author Eli
 */
```

Line comments explain *why*: `// Reversed because the left side is mirrored`.
Step comments are the exception; see below.

## 6. Autonomous: one block per step

Put the start pose and every key field position at the top of the auto file as
named `Pose` constants. Build paths from those names; don't write numbers inside
the path code. When you need to adjust a position, you only change one line.

```java
// Key field positions (blue side; mirrored for red)
private static final Pose START    = new Pose(9, 60, Math.toRadians(0));
private static final Pose BASKET   = new Pose(18, 126, Math.toRadians(135));
private static final Pose SAMPLE_2 = new Pose(36, 120, Math.toRadians(0));
```

Split the routine into steps. Put a blank line between steps and a one-line
comment on top of each step saying what it does. Someone reading only the
comments should be able to follow the whole auto.

```java
// Drive up to the basket and score the preloaded sample
followAndWait(toBasket);
lift.setTargetPosition(LIFT_HIGH);
claw.setPosition(CLAW_OPEN);

// Back away and lower the lift so we don't tip
followAndWait(backAway);
lift.setTargetPosition(LIFT_DOWN);

// Drive to the second sample and grab it
followAndWait(toSecondSample);
claw.setPosition(CLAW_CLOSED);
```

`followAndWait` is a small helper in the OpMode:

```java
private void followAndWait(Path path) {
    follower.follow(path);
    while (opModeIsActive() && !follower.atParametricEnd()) {
        follower.update();
    }
}
```

- **Every wait loop must check `opModeIsActive()`.** Otherwise the robot keeps
  moving after STOP is pressed. Don't use `sleep()` in a loop either.
- **One file per auto, not one per alliance.** Write the paths once for blue and
  mirror them for red with an `isRed` flag. Don't copy the file into
  `RedSide` / `BlueSide` / `_Fixed` versions.

- Don't put blank lines inside a step. A blank line always means a new step.
- If a step needs more than one line of comment, split it into smaller steps.
- Pedro paths: name each path for where it goes (`toBasket`) and put a one-line
  comment above it.
- Long TeleOp loops use the same pattern: one commented block per control.

## 7. Artificial Intelligence

You're allowed to use AI, with these rules:

- **Use tools that can see the repo.** Coding assistants that read this project
  (in Android Studio or the terminal) are fine. Don't use web chatbots like
  ChatGPT in the browser. They can't see our code or the library versions we
  use, so they often write code for old APIs (for example, Pedro 2 instead of
  Pedro 3).
- **Review every line.** You're responsible for AI code the same as code you
  typed yourself.
- **Before you push, make sure it works and you understand it.** Build it, run
  it on the robot if it touches hardware, and be able to explain what each part
  does. If you can't explain it, don't push it.
- Docstrings are still written by you (see section 5).

## 8. Git

- Work on branch `5898`.
- Before you delete or rename a class, field, or constant, search the whole
  project for it (**Edit → Find → Find in Files** in Android Studio). Fix or
  archive everything that uses it, then build.

### Competition code freeze

- **Stop changing code the day before a competition.** After that, only fix
  bugs that stop the robot from working.
- **Tag the version on the robot.** Tag the commit that's installed on the robot
  and push the tag:
  ```
  git tag league-2
  git push origin league-2
  ```
  If something breaks at the event, `git checkout league-2` gets you back to the
  last version that worked.
- Name tags after the event: `league-1`, `league-2`, `qualifier`, `state`.

### Commit messages

```
Add second sample pickup to blue auto

The first path ended too close to the wall, so the claw missed.
Moved the end pose 2 inches out.
```

- **First line:** say what the commit does, written as a command:
  `Add`, `Fix`, `Tune`, `Remove`, `Move`. Keep it under about 60 characters,
  and don't end it with a period.
- **Body (optional):** leave a blank line after the first line, then explain
  *why*, what was wrong, or what you tested. Add a body for tuning changes and
  bug fixes.
- **One change per commit.** If the message needs "and" or `;` to list
  unrelated things, split it into separate commits.
- For tuning commits, put the new numbers in the message:
  `Tune flywheel kP 0.02 → 0.03 (less overshoot)`.

| ❌ Don't | ✅ Do |
|---|---|
| `...`, `Syncing`, `fix` | `Fix claw not closing at end of auto` |
| `2/19` (Git already records the date) | `Add far-side red auto` |
| `Update Constants.java` | `Raise lift high position to clear basket` |
| `Updated X; Updated Y; Added Z` | three separate commits |
| `Updated Poses (Not done, just syncing)` | Don't push unfinished work to `5898`. Commit it on your own branch first. |
