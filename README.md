---
title: "Robot Go"
subtitle: "How to Make a Robot Go — FRC BIOCORE"
author: "FRC 8592"
date: "Preseason 2026 → 2027 Season"
theme: blue
highlightjs: true
slideNumber: true
hash: true
transition: slide
width: 1280
height: 720
revealjs-url: https://unpkg.com/reveal.js@4.6.0
header-includes: |
  <style>
    /* reveal.css caps code at 400px and then lets it spill past the panel,
       which clipped the longer samples. !important is needed to win over it. */
    .reveal pre { width: 100%; box-shadow: none; margin: 0.4em 0; }
    .reveal pre code {
      max-height: 560px !important;
      font-size: 0.78em !important;
      line-height: 1.3 !important;
      overflow: auto !important;
      padding: 0.6em 0.8em;
    }
    /* The longest tables (schedule, SystemCore removals) overran the slide;
       the default cell padding was the main culprit. */
    .reveal table { font-size: 0.80em !important; }
    .reveal table td, .reveal table th { padding: 0.15em 0.45em !important; }
  </style>
---

# Welcome

## Robot Go

**How to Make a Robot Go!**

By the end you will have written, reviewed, merged, and deployed real code that moves real hardware.

::: incremental
Nobody starts knowing this. You don't learn it by reading - you learn it by putting code on a real robot, watching it fail, and figuring out why.
You will deploy broken code. You will break the practice robot. 
That's the job, and that's what the practice robot is for.
:::

## The schedule

| # | Session                                          |
|---|--------------------------------------------------|
| 1 | GitHub, PRs & Naming Conventions                 |
| 2 | Make a Motor Move + Logging                      |
| ↻ | *Workshop: Motor Helpers & Cleanup*              |
| 3 | How to Create a Command                          |
| 4 | PID and How It Works                             |
| ↻ | *Workshop: Cleanup #2*                           |
| 5 | Architecture: Command vs State Machine vs Hybrid |
| ↻ | *Workshop: Cleanup #3*                           |
| — | **Buffer meeting** - catch up, ask anything      |

## Two symbols you'll see everywhere

**🟦 roboRIO** — the control system we're using *right now*, WPILib 2026.
Everything you type and deploy this preseason is this.

**🟩 SystemCore** — the new control system for the **2027 season**.
Marked slides tell you what changes and why.

## Ground rules for the shop

::: incremental
- TODO - do we have these documented somewhere?
:::

## Setting up your laptop

1. **Install WPILib** -
   [docs.wpilib.org · WPILib installation](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
2. **Create a project** -
   [docs.wpilib.org · Creating a robot program](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-4/creating-robot-program.html)
3. **Build a drivetrain program** -
   [docs.wpilib.org · Test drivetrain program](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-4/creating-test-drivetrain-program-cpp-java-python.html)

Then clone this repo:

```bash
git clone git@github.com:FRC8592/Robot-Go.git
cd Robot-Go
./gradlew build
```

::: notes
use the biocore-preseason branch
::: 

## Your first deploy

```bash
./gradlew deploy
```

TODO - test this on the DIY robots

::: incremental
1. Connect to the robot's WiFi (or tether USB).
2. Open the Driver Station.
3. Confirm **Communications**, **Robot Code**, and **Joysticks** are green.
4. Call "ENABLING!"
5. Enable in Teleop.
:::

# The Control System

## 🟦 What's on the robot today

- **roboRIO** - the computer. Runs *your* code.
- **PDP/PDH** - power distribution. Every motor gets a breaker.
- **CAN bus** - one daisy-chained wire pair that talks to every motor controller.
- **Radio** - how the Driver Station reaches the robot.

## 🟩 SystemCore — new for 2027

The biggest control system change since the cRIO.

::: incremental
- **Multiple CAN buses** — no more one-bus bottleneck
- **Smart IO** — smarter, simpler sensor ports
- **Onboard IMU** — a gyro is built in, no separate board
- **Expansion Hub** — shared hardware with FTC
- Cheaper than the roboRIO
:::

## 🟩 What SystemCore removes

| Gone | Use instead |
|---|---|
| Relays | A motor controller, or Smart IO |
| Analog output | — (rarely used) |
| SPI devices | CAN or Smart IO sensors |
| Servos | CAN-based actuators |
| Ultrasonics, counters | Smart IO / CAN sensors |
| Analog triggers, DMA, interrupts | Polling in `periodic()` |
| SPI IMUs (ADIS16448/16470, ADXRS450) | The onboard IMU |
| LabVIEW | Java, C++, or Python |

## 🟩 Everything gets renamed

:::: {.columns}
::: {.column width="50%"}
**🟦 2026 · roboRIO**

```java
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
```
:::
::: {.column width="50%"}
**🟩 2027 · SystemCore**

```java
import org.wpilib.wpilibj.TimedRobot;
import org.wpilib.commands3.Command;
```
:::
::::

- Java: `edu.wpi.first` → **`org.wpilib`**
- The VS Code importer converts most of this automatically.

**The 2027 library is not finalized - exact names are still moving.**
Learn the *shape* of the change, not the spelling.

## 🟩 Also new for 2027

::: incremental
- **A new Driver Station** — shared between FRC and FTC, no longer LabVIEW
- **Java 25 / C++23** — much newer language features available
- `robotInit()` is deleted — the `Robot()` constructor, which 2026 already uses
- `MotorController.set()` → `setThrottle()`, `stopMotor()` → `disable()`
- All the gamepad classes collapse into one `Gamepad`
- **Commands v3** — a new command framework built on coroutines
:::

::: notes
Commands v3 is genuinely interesting — it uses Java 21+ virtual threads so a
command can be written as straight-line imperative code that pauses and
resumes, instead of being split across initialize/execute/isFinished. It could
not be built for the roboRIO because the JVM there didn't support it. Don't go
deep here; there's a backlog slide for it at the end.
:::

## Why we're learning on the roboRIO anyway

::: incremental
- It's what's bolted to our existing robots **today**.
- The 2027 library is alpha; the class names still change between builds.
- **The concepts transfer completely.** Subsystems, commands, PID, CAN,
  current limits - none of that changes.
- The rename is a find-and-replace. The thinking is the hard part.
:::

# Robot Go 1
GitHub, PRs & Naming

## Why version control

::: incremental
- **Undo, but for the whole project, forever.**
- Twenty people editing the same code without overwriting each other.
- "It worked yesterday" becomes a question you can actually answer.
- At competition: know exactly which commit is on the robot.
:::

::: notes
The competition argument is the one that lands with veterans. Ask anyone who's
been to a comp whether they've ever had "wait, is this the code that worked in
match 12?" That's a version control problem.
:::

## The four commands you'll use daily

```bash
git clone git@github.com:FRC8592/Robot-Go.git   # once, ever
git checkout -b yourname/what-you-did           # start work
git add -A && git commit -m "Add intake subsystem"
git push -u origin yourname/what-you-did        # share it
```

. . .

That's it. That's the whole loop.


## Branch naming

**`yourname/ticket-summary`**

```
rdmarsh2/612-curvature-drive
bsneade/89-intake-current-limit
alex/123-fix-shooter-pid
```

::: incremental
- Your name means everyone knows who to ask.
- The ticket number references the full description of the work
- The summary is a quick reference for what it is.
- Lowercase, hyphens, no spaces.
- **Never commit directly to `main`.**
- **TODO - document branching strategy**
:::


## Anatomy of a good pull request

::: incremental
- **Title**: what changed, in plain English
- **Body**: *why* it changed, and how you tested it
- **Small.** One idea per PR. A 40-line PR gets reviewed; a 900-line PR gets
  rubber-stamped, which is the same as not reviewed.
- **Tested on the robot?** Say so. Say what you saw.
:::

```markdown
## What
Adds a 40A smart current limit to the intake motor.

## Why
The intake was browning out the robot when a game piece jammed.

## Testing
Deployed to the practice bot, deliberately jammed the intake.
Voltage held above 10V; previously dropped to 7V and rebooted the RIO.
```

## Reviewing someone else's PR

You will review as much code as you write. It's not optional and it's not rude.

::: incremental
- **Read it to understand, not to approve.** If you can't explain what it does,
  say that - that's a finding.
- Ask questions instead of issuing orders: "what happens if the sensor is
  disconnected here?" beats "this is wrong."
- Check the numbers. Magic constants, wrong units, and copy-pasted CAN IDs are
  where the real bugs hide.
- **Approving means you believe it works.** Your name is on it too.
:::

## Java naming conventions

```java
public class IntakeSubsystem {          // PascalCase for classes
    private final SparkMax motorSparkMax;    // camelCase for member fields
    public static final int CAN_ID = 5; // SCREAMING_SNAKE_CASE for constants
    private double targetSpeed;       // camelCase for variables

    public void setSpeed(double speed) {  // camelCase for methods
        targetSpeed = speed;
    }
}
```

::: incremental
- Use full words
- Names should say **what**, not **how**: `IntakeSubsystem`, not `Motor5`
:::

::: incremental
WPILib conventions
- `m_` = "this belongs to the object" — you can tell at a glance
- `k` = "this never changes" — from the WPILib convention
:::

## When two people edit the same line

A **merge conflict** is git saying "I don't know which one you want."

```java
<<<<<<< HEAD
    motor.set(0.5);
=======
    motor.set(0.8);
>>>>>>> alex/612-faster-intake
```

::: incremental
1. Don't panic. Nothing is lost.
2. Delete the `<<<<`, `====`, `>>>>` lines.
3. Decide what the code should actually be - maybe neither version.
4. **Go ask the other person.** They know why they picked 0.8.
5. Commit the resolution.
:::

## Hands-on: your first PR

::: incremental
1. Branch: `yourname/add-me-to-readme`
2. Add your name to the roster at the bottom of this README
3. Commit, push, open a PR
4. **Review someone else's PR** — leave one real comment
5. Get yours reviewed, then merge it
:::

**Everyone leaves today with a merged PR.**

Add yourself in your first PR:

- Brad Sneade — mentor
- *your name here*

---

## Building this deck

```bash
./gradlew slides           # build/slides/slides.html
./gradlew portableSlides   # self-contained, works offline
```

If pandoc isn't on your PATH:

```bash
./gradlew -Ppandoc=/path/to/pandoc slides
```

Press **`S`** in the deck for speaker notes, **`Esc`** for the slide overview.
