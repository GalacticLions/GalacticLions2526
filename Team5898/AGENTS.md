# Team 5898 — instructions for AI agents

Before writing or editing any code in `Team5898/`, read `STYLEGUIDE.md` in this
folder and follow it. Pay extra attention to these rules:

- Don't write docstrings for the user. Leave them for the person to write
  (STYLEGUIDE §5), or mark the spot with `// TODO docstring`.
- Use Pedro Pathing 3.0 APIs (`com.pedropathing:revhub:3.0.1`), not 2.x.
  Check method names against the Javadoc: https://javadoc.io/doc/com.pedropathing/core/latest/index.html
- Pedro `Pose` headings are radians. Always write `Math.toRadians(deg)`.
- Every wait loop in an OpMode must check `opModeIsActive()`.
- Don't edit `FtcRobotController/` or any gradle file unless asked to.
- `./gradlew :Team5898:assembleDebug` must pass before you say you're done.
