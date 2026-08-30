---
title: "Robot Go"
subtitle: "How to Make a Robot Go - FRC BIOCORE"
author: "FRC 8592"
date: "Preseason 2026 → 2027 Season"
theme: night
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
| - | **Buffer meeting** - catch up, ask anything      |

## Two symbols you'll see everywhere

**🟦 roboRIO** - the control system we're using *right now*, WPILib 2026.
Everything you type and deploy this preseason is this.

**🟩 SystemCore** - the new control system for the **2027 season**.
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

Then clone this repo:sudo sed -i 's|signed-by=/etc/apt/trusted.gpg.d/google-chrome.gpg|signed-by=/usr/share/keyrings/google-chrome.gpg|' /etc/apt/sources.list.d/google-chrome-official.list && sudo apt update

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

## 🟩 SystemCore - new for 2027

The biggest control system change since the cRIO.

::: incremental
- **Multiple CAN buses** - no more one-bus bottleneck
- **Smart IO** - smarter, simpler sensor ports
- **Onboard IMU** - a gyro is built in, no separate board
- **Expansion Hub** - shared hardware with FTC
- Cheaper than the roboRIO
:::

## 🟩 What SystemCore removes

| Gone | Use instead |
|---|---|
| Relays | A motor controller, or Smart IO |
| Analog output | - (rarely used) |
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
- **A new Driver Station** - shared between FRC and FTC, no longer LabVIEW
- **Java 25 / C++23** - much newer language features available
- `robotInit()` is deleted - the `Robot()` constructor, which 2026 already uses
- `MotorController.set()` → `setThrottle()`, `stopMotor()` → `disable()`
- All the gamepad classes collapse into one `Gamepad`
- **Commands v3** - a new command framework built on coroutines
:::

::: notes
Commands v3 is genuinely interesting - it uses Java 21+ virtual threads so a
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
public class IntakeSubsystem {                    // PascalCase for classes
    public static final int CAN_ID = 5;           // SCREAMING_SNAKE_CASE for constants
    private final SparkMax motor;                 // camelCase for member fields
    private double targetSpeed;                   // camelCase for member state too

    public void setSpeed(double speed) {          // camelCase for methods
        targetSpeed = speed;            
          // camelCase for locals and parameters
    }
}
```

::: incremental
- Use full words
- Names should say **what**, not **how**: `IntakeSubsystem`, not `Motor5`
:::

::: incremental
You will see two other prefixes in WPILib's own code and in older examples
- `m_` on a member field - `m_motor` instead of `motor`
- `k` on a constant - `kCanId` instead of `CAN_ID`
- **Recognize them; don't write them.** We follow standard Java naming.
- Vendor names like `MotorType.kBrushless` keep their `k` - that's their API,
  not ours to rename.
- PID gains show up as `kP`/`kI`/`kD` in every WPILib doc and tutorial -
  we write `P_GAIN`/`I_GAIN`/`D_GAIN`. Same idea, our spelling.
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
4. **Review someone else's PR** - leave one real comment
5. Get yours reviewed, then merge it
:::

**Everyone leaves today with a merged PR.**

Add yourself in your first PR:
- Brad Sneade - mentor
- *Your name here*

---

# Robot Go 2
Motors & Logging

## From code to spinning shaft

```
Your code  ──>  roboRIO  ──CAN──>  Motor Controller  ──>  Motor
                                          ↑
                                    PDP breaker
                                    (the actual power)
```

::: incremental
- Your code sends a **number between -1.0 and 1.0**
- The controller turns that into voltage
- Every controller has a unique **CAN ID** - set it once, write it down
:::

## Brushed vs brushless

|  | Brushed (CIM, 775) | Brushless (NEO, Kraken, Falcon) |
|---|---|---|
| Power | Less | More |
| Efficiency | Lower | Higher |
| Built-in encoder | No | **Yes** |
| Cost | Cheaper | Pricier |
| Wears out | Brushes wear | Basically doesn't |

**Brushless motors know where they are.** That matters enormously in Session 4.

## Brake vs coast, and current limits

```java
SparkMax motor = new SparkMax(Constants.INTAKE_CAN_ID, MotorType.kBrushless);

SparkMaxConfig config = new SparkMaxConfig();
config.smartCurrentLimit(40)      // amps - protects motor AND battery
      .idleMode(IdleMode.kBrake); // stop dead vs. spin down freely

motor.configure(config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
```

::: incremental
- **Brake** - holds position when you stop. Arms, elevators.
- **Coast** - spins down freely. Drivetrains (usually), flywheels.
- **Current limit** - the difference between a stalled motor and a *burnt* motor.
:::

## Making it move

```java
public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax motor =
        new SparkMax(Constants.INTAKE_CAN_ID, MotorType.kBrushless);

    public void setSpeed(double speed) {
        motor.set(speed);   // -1.0 .. 1.0
    }

    public void stop() {
        motor.set(0.0);
    }
}
```

Bind it to a button in `RobotContainer`:

```java
private final CommandXboxController driver = new CommandXboxController(0);

driver.a().whileTrue(
    Commands.startEnd(() -> intake.setSpeed(0.6),
                      () -> intake.stop(),
                      intake));
```

## 🟩 What this looks like in 2027

:::: {.columns}
::: {.column width="50%"}
**🟦 2026**

```java
motor.set(0.6);
motor.stopMotor();
```
:::
::: {.column width="50%"}
**🟩 2027**

```java
motor.setThrottle(0.6);
motor.disable();
```
:::
::::

`set()` was ambiguous - set *what*? Throttle? Position? Velocity?
`setThrottle()` says what it means.


## Logging: how you find out what happened

**You cannot debug a robot by watching it.** It moves too fast and it
doesn't tell you why.

```java
public class Robot extends TimedRobot {
    public Robot() {
        DataLogManager.start();                          // log to a file
        DriverStation.startDataLog(DataLogManager.getLog());
    }
}
```

::: incremental
- **NetworkTables (NT4)** - live values, robot → dashboard
- **DataLogManager** - writes a file you can scrub through *after* the match
- Log it now; you cannot go back and log a match that already happened.
:::

## Publishing a value

```java
public class IntakeSubsystem extends SubsystemBase {
    private final DoublePublisher velocityPublisher =
        NetworkTableInstance.getDefault()
            .getTable("Intake")
            .getDoubleTopic("Velocity")
            .publish();

    @Override
    public void periodic() {
        velocityPublisher.set(motor.getEncoder().getVelocity());
    }
}
```

Publish once, set it every loop. Now it's on the dashboard *and* in the log file.



## The two tools you'll actually use

**Elastic** - the *driver's* dashboard.
Big readable widgets, match-time information, autonomous chooser.

**AdvantageScope** - the *programmer's* tool.
Scrub through a log file, graph any value against any other, replay the match.

. . .

**SmartDashboard and Shuffleboard are removed in 2027.**

You'll see them in old code and tutorials. Recognize them - don't build on them.


## Reading the Driver Station log

When something breaks, the answer is usually already written down.

::: incremental
- **Orange** = warning. Often "loop time overrun" - your code is too slow.
- **Red** = exception. Read the **first** line and the **first** `frc.robot`
  line in the stack trace. That's your bug.
- **"Watchdog not fed"** = a loop took longer than 20ms.
- Brownout warnings = electrical, not code. Usually.
:::

## Hands-on: motor + telemetry

::: incremental
1. Create `IntakeSubsystem` with one motor
2. Set a **current limit** and an idle mode
3. Bind it to the A button with `whileTrue`
4. Publish motor velocity and applied output to NetworkTables
5. Watch it in AdvantageScope while you run it
6. **PR it.** Include what you saw on the graph.
:::

# Workshop
Motor Helpers & Cleanup {#cleanup-workshop}

## Why we keep doing this

Three times this preseason we stop adding features and clean up instead.

::: incremental
- Real teams spend more time reading code than writing it
- The code you write in October is the code you debug in March
- Refactoring is a *skill*, and it needs reps
:::

## Code smell #1: magic numbers

```java
// Before - what is 0.6? Why 40? What's 5?
SparkMax motor = new SparkMax(5, MotorType.kBrushless);
config.smartCurrentLimit(40);
motor.set(0.6);
```

```java
// After - Constants.java
public static final class IntakeConstants {
    public static final int    CAN_ID        = 5;
    public static final int    CURRENT_LIMIT = 40;
    public static final double INTAKE_SPEED  = 0.6;
}
```

**A number in the middle of your code is a number nobody can change safely.**

::: notes
The tuning argument sells this better than the readability argument: when all
the numbers live in one file, you can tune a mechanism without hunting through
five classes, and the diff of what you tuned is one readable file.
:::

## Code smell #2: copy-pasted configuration

Four motors, four identical twelve-line config blocks.

. . .

Now change the current limit on all four. Miss one. Debug it for an hour.

```java
public final class MotorHelper {
    public static SparkMax createSparkMax(int canId, int currentLimit,
                                          IdleMode idleMode, boolean inverted) {
        SparkMax motor = new SparkMax(canId, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(currentLimit)
              .idleMode(idleMode)
              .inverted(inverted);
        motor.configure(config, ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);
        return motor;
    }
}
```

## Now it's one line

```java

private final SparkMax motor = MotorHelper.createSparkMax(
    IntakeConstants.CAN_ID,
    IntakeConstants.CURRENT_LIMIT,
    IdleMode.kBrake,
    false);
```

::: incremental
- Every motor configured the same way, guaranteed
- Change the pattern once, it changes everywhere
- The subsystem now reads like *what it does*, not *how it's wired*
:::

## The cleanup checklist

::: incremental
- [ ] Any number that isn't 0 or 1 lives in `Constants.java`
- [ ] No block of code appears twice
- [ ] Every class and method name says what it does
- [ ] Dead code and commented-out code is **deleted** (git remembers it)
- [ ] Every public method has a one-line comment saying why it exists
- [ ] It still builds and still runs on the robot
- [ ] **PR it** - cleanup PRs get reviewed like any other
:::

# Robot Go 3
Commands `[CMD]`

## The mental model

::: incremental
- A **Subsystem** is a *thing* - the intake, the arm, the drivetrain.
  It owns hardware.
- A **Command** is a *request* - "run the intake", "raise the arm to 40°".
- The **Scheduler** decides who gets what, and stops two commands from
  fighting over the same motor.
:::

. . .

**One subsystem, one owner at a time.** That rule is the whole framework.

## A command's life

```java
public class IntakeCommand extends Command {
    private final IntakeSubsystem intake;

    public IntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);   // "I need this subsystem"
    }

    @Override public void initialize() { intake.setSpeed(0.6); }  // once, at start
    @Override public void execute()    { }                        // every 20ms
    @Override public boolean isFinished() { return intake.hasGamePiece(); }
    @Override public void end(boolean interrupted) { intake.stop(); }
}
```

## Requirements are the safety net

```java
addRequirements(intake);
```

::: incremental
- Two commands both need the intake?
- The **new one wins**; the old one gets `end(interrupted = true)`.
- Your motor never gets two conflicting commands in the same loop.
- **This is why you declare requirements.** Skip it and you get chaos.
:::

## Default commands

What should a subsystem do when nobody's asking for anything?

```java
drivetrain.setDefaultCommand(
    Commands.run(() -> drivetrain.arcadeDrive(
                          -driver.getLeftY(),
                          -driver.getRightX()),
                 drivetrain));
```

The drivetrain drives from the sticks unless something else takes over.

## Composing commands

```java
// One after another
intakeCommand.andThen(indexCommand);

// At the same time
shooterSpinUp.alongWith(armRaise);

// Whichever finishes first wins
intakeCommand.withTimeout(3.0);

// A whole sequence
Commands.sequence(
    arm.goToAngle(45),
    intake.runUntilPiece(),
    arm.goToAngle(0));
```

**Small commands compose into big behavior.** That's the payoff.


## Binding to the controller

```java
private final CommandXboxController driver = new CommandXboxController(0);

private void configureBindings() {
    driver.a().onTrue(new IntakeCommand(intake));    // on press
    driver.b().whileTrue(shooter.spinUpCommand());   // while held
    driver.x().toggleOnTrue(arm.raiseCommand());     // press on/press off
}
```

| Binding | Fires |
|---|---|
| `onTrue` | Once, when pressed |
| `whileTrue` | Runs while held, cancels on release |
| `toggleOnTrue` | Press to start, press again to stop |

::: notes
Choosing the wrong binding type is a very common bug and it looks like a code
bug rather than a binding bug. If a mechanism "won't stop", check whether they
used onTrue where they wanted whileTrue.
:::

## 🟩 Commands in 2027

Setup has already moved from `robotInit()` into the constructor - **our repo
is on the right side of this today.** 2027 deletes `robotInit()` entirely.

:::: {.columns}
::: {.column width="50%"}
**Old code you'll still find**

```java
public class Robot extends TimedRobot {
  @Override
  public void robotInit() {
    container =
      new RobotContainer();
  }
}
```
:::
::: {.column width="50%"}
**🟦 2026 and 🟩 2027**

```java
public class Robot extends TimedRobot {
  public Robot() {
    container =
      new RobotContainer();
  }
}
```
:::
::::

**Commands v3** is also coming: commands written as straight-line code that
pauses and resumes, instead of split across `initialize`/`execute`/`isFinished`.

*Concept only - the API is still alpha.*

::: notes
Commands v3 leans on Java 21+ virtual threads, which the roboRIO's JVM couldn't
do - that's why it's arriving with SystemCore rather than earlier. Everything
in this session still applies; v3 changes how you write the body, not what a
command or a requirement means.
:::

## Hands-on: write a real command

::: incremental
1. Write `IntakeCommand` - runs the intake until a sensor sees a game piece
2. `addRequirements()` - and understand why
3. Give it a **timeout** so a broken sensor can't hold the subsystem forever
4. Bind it to a button
5. Add a default command to your subsystem
6. **PR it**
:::

# Robot Go 4 
PID

## Open loop vs closed loop

**Open loop** - "spin at 60% power."

```java
motor.set(0.6);
```

The motor doesn't know where it is. Battery sags, load changes, results change.

. . .

**Closed loop** - "*be* at 40 degrees."

The motor measures, compares, and corrects. Every 20 milliseconds. Forever.

## Error is the whole idea

```
error = where I want to be  −  where I actually am
```

::: incremental
- Error is big? Push hard.
- Error is small? Ease off.
- Error is zero? Stop.
:::

. . .

Everything else in PID is refinement of that one sentence.

## P - Proportional

```java
double output = P_GAIN * error;
```

::: incremental
- The further away you are, the harder you push. That's it.
- **Too small**: never gets there - "steady-state error"
- **Too big**: overshoots, comes back, overshoots - oscillation
- **P alone almost never quite arrives.** Gravity, friction, and stiction win
  near the setpoint, where the error is small.
:::

## I - Integral

```java
double output = P_GAIN * error + I_GAIN * accumulatedError;
```

::: incremental
- Adds up error **over time** - "we've been a little short for a while now"
- Kills the steady-state error P leaves behind
- **Too big**: winds up, then overshoots badly and oscillates slowly
- Most FRC mechanisms use **`I_GAIN = 0`**. Try feedforward first.
:::

## D - Derivative

```java
double output = P_GAIN * error + I_GAIN * accumulatedError + D_GAIN * errorRate;
```

::: incremental
- Responds to how **fast** error is changing
- Acts like a brake - damps the overshoot P causes
- Lets you run a higher `P_GAIN` without oscillating
- **Too big**: jittery and noisy, because it amplifies sensor noise
:::

## Feedforward 

PID reacts to error. **Feedforward predicts what you need before there's error.**

```java
SimpleMotorFeedforward feedforward =
    new SimpleMotorFeedforward(S_GAIN, V_GAIN, A_GAIN);
PIDController pidController = new PIDController(P_GAIN, I_GAIN, D_GAIN);

double volts = feedforward.calculate(targetVelocity)
             + pidController.calculate(currentVelocity, targetVelocity);
motor.setVoltage(volts);
```

::: incremental
- **S_GAIN** - voltage just to overcome friction and start moving
- **V_GAIN** - voltage per unit of velocity
- **A_GAIN** - voltage per unit of acceleration
- **Feedforward does the work. PID cleans up the difference.**
:::

## A tuning workflow that won't break the robot

::: incremental
1. **Everything to zero.** P_GAIN, I_GAIN, D_GAIN, all of it.
2. Feedforward first - find `S_GAIN` (barely moves), then `V_GAIN`.
3. Raise `P_GAIN` until it oscillates. Then **halve it**.
4. Add `D_GAIN` to damp what's left.
5. Only add `I_GAIN` if you still have persistent error. Usually you won't.
6. **Graph it in AdvantageScope.** Setpoint and measurement on one plot.
:::

**Have a hand on the e-stop. Start with small setpoints.**

## Units will get you

```java
// Encoder says 2048 ticks. Two thousand what?
double position = encoder.getPosition();
```

::: incremental
- Ticks → rotations → **gear ratio** → inches or degrees
- Do the conversion **once**, in the subsystem, and never again
- Everything outside the subsystem speaks real-world units
- "It moved 40 times too far" is a gear ratio. Every single time.
:::

```java
private static final double GEAR_RATIO                 = 9.0;
private static final double WHEEL_CIRCUMFERENCE_INCHES = 4.0 * Math.PI;

public double getPositionInches() {
    return encoder.getPosition() / GEAR_RATIO * WHEEL_CIRCUMFERENCE_INCHES;
}
```

## 🟩 PID in 2027

`PIDCommand`, `ProfiledPIDCommand`, and `TrapezoidProfileCommand` are removed.

Use the controller classes directly inside your own command - which is what
this session already teaches.

::: incremental
- `PIDController` - stays
- `SimpleMotorFeedforward`, `ArmFeedforward`, `ElevatorFeedforward` - stay
- `ProfiledPIDController` - stays
- Only the *command wrappers* go away
:::

## Hands-on: hold a setpoint

::: incremental
1. Pick a mechanism with an encoder
2. Feedforward first - find `S_GAIN` and `V_GAIN`
3. Add `P_GAIN`, tune until it oscillates, halve it
4. Add `D_GAIN`
5. Graph setpoint vs. measurement in AdvantageScope
6. **PR it** - include the graph and your final gains
:::

# ↻ Workshop - Cleanup #2

## This week's target: the PID code

Same [cleanup checklist](#cleanup-workshop) - new target.

::: incremental
- Are your gains in `Constants.java`, or scattered in the subsystem?
- Did you copy a PID block between two subsystems?
- Are the unit conversions in **one** place?
- Does `getPositionInches()` actually return inches?
- Delete the tuning experiments you left commented out.
:::

# Building this deck

```bash
./gradlew slides           # build/slides/slides.html
./gradlew portableSlides   # self-contained, works offline
```

If pandoc isn't on your PATH:

```bash
./gradlew -Ppandoc=/path/to/pandoc slides
```

Press **`S`** in the deck for speaker notes, **`Esc`** for the slide overview.