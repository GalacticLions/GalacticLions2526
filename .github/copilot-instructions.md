# Copilot instructions

This repo holds several FTC teams' modules. The rules below apply only to code
in `Team5898/`. For the other modules, match the surrounding code.

## Team5898

Follow `Team5898/STYLEGUIDE.md`. If you can read files, read it before editing
code in `Team5898/`. The most important rules:

- Don't write docstrings. The team writes them by hand. Leave `// TODO docstring`
  where one is needed.
- Use Pedro Pathing 3.0 APIs (`com.pedropathing:revhub:3.0.1`), not 2.x.
  Javadoc: https://javadoc.io/doc/com.pedropathing/core/latest/index.html
- Pedro `Pose` headings are radians. Always write `Math.toRadians(deg)`, never a
  bare number like `new Pose(24, 48, 90)`.
- Every wait loop in an OpMode must check `opModeIsActive()`. No `sleep()` in loops.
- No magic numbers in OpModes. Put tunables in `Constants.java` as non-final
  `public static` fields. Hardware config names only live in `Constants.Hardware`.
- In autonomous code, split each step into its own block: blank line between
  blocks, one-line comment on top saying what the step does.
- Write one auto per routine and mirror it for red with an `isRed` flag. Don't
  make separate red and blue copies.
- Don't import from `Archive/`. Don't edit `FtcRobotController/` or gradle files
  unless asked to.
- Java only, 4-space indent, always use braces.
