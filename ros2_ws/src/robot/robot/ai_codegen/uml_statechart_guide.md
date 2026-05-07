# UML Statechart Guide

How to draw a statechart for your robot mission so it can be read unambiguously
— by teammates, instructors, or an AI agent generating code from it.

---

## Why a Statechart?

A statechart lets you design the robot's behavior before writing a single line
of code. It separates *what the robot does* from *when it does it*. Once the
diagram is clear, the code almost writes itself.

---

## Elements at a Glance

| Element | Drawn as | Meaning |
|---------|----------|---------|
| State | Rounded rectangle | A mode the robot stays in until something happens |
| Transition | Arrow between states | The robot switches mode |
| Event | Text before `/` on an arrow | What triggers the transition |
| Guard | Text in `[...]` on an arrow | Extra condition that must also be true |
| Action | Text after `/` on an arrow | What to do *during* the transition |
| Entry action | `entry /` inside a state box | Runs once when entering the state |
| Do action | `do /` inside a state box | Runs every loop tick while in the state |
| Exit action | `exit /` inside a state box | Runs once when leaving the state |
| Initial pseudostate | Filled circle `●` | Where execution begins |
| Final pseudostate | Bullseye `◉` | Where execution ends (optional) |

---

## States

A state is a mode your robot is currently in. Draw it as a rounded rectangle.
The name goes at the top in UPPERCASE.

Internal behaviors go below the name, each on its own line with a keyword:

```
╭─────────────────────╮
│        IDLE         │
│ entry / orange LED  │
│ do / check buttons  │
╰─────────────────────╯
```

### The three keywords

**`entry /`** — runs exactly once, the moment the robot enters the state.
Use this for setup: turn on an LED, start a motor, reset a timer.

**`do /`** — runs on every FSM loop tick while the robot remains in the state.
Use this for continuous checks or ongoing output.

**`exit /`** — runs exactly once, just before leaving the state.
Use this for cleanup: stop a motor, turn off an LED.

> If you only have one behavior and it's clearly setup, writing it without a
> keyword is acceptable — but adding `entry /` makes the intent unambiguous.

### What goes inside vs. on the arrow

| Put it **inside the state** | Put it **on the arrow** |
|-----------------------------|------------------------|
| An LED that stays on while in this state | An LED that flashes once as acknowledgment |
| A motor that runs throughout this state | A motor command that starts the move |
| A continuous sensor read | A one-time check that decides whether to leave |

---

## Transitions

A transition is an arrow from one state to another. It fires when its event
occurs and any guard condition is true.

### Syntax

```
event [guard] / action
```

All three parts are optional, but at least one should be present.

```
╭──────────╮                     ╭──────────╮
│  STATE A │ ──────────────────► │  STATE B │
╰──────────╯  button_pressed(1)  ╰──────────╯
              [velocity > 0]
              / stop()
```

Read it as: *"When button 1 is pressed, and the robot is moving, stop the
robot and go to STATE B."*

### Event — what triggers the transition

An event is the thing that *happens*. For this robot, common events are:

| Event | Meaning |
|-------|---------|
| `button_pressed(N)` | Button N was pressed (one-shot edge) |
| `button_held(N)` | Button N is currently held |
| `motion_done` | The active MotionHandle finished |
| `limit_triggered(N)` | Limit switch N fired |
| `timeout` | A timer expired |
| *(no event)* | Transition fires immediately every tick |

### Guard — an extra condition in `[...]`

A guard is a Boolean condition that must be true *in addition to* the event.
Use it when the same event can lead to different states depending on context:

```
             [disk present]
             / close gripper
    IDLE ──────────────────────► GRIPPING

             [no disk]
             / sound buzzer
    IDLE ──────────────────────► IDLE
```

### Action — what to do during `/`

An action is a short, instantaneous operation that happens *as part of the
transition*, before the robot settles into the new state.

Good uses: acknowledge a button (blink LED), log a message, start a timer.

Not good uses: wait for a motor to finish, run a long sequence. Those belong
in the destination state's `entry /` or as a background task.

### Multiple actions — arrow vs. state

When a transition or state needs more than one action, ask:
*"Is this action about what just happened, or about where I'm going?"*

| It's about… | Put it |
|---|---|
| The event itself (acknowledgment, log) | On the arrow: `/ blink GREEN; log press` |
| Setting up the new state | `entry /` in the destination state |
| Cleaning up regardless of which transition fires | `exit /` in the source state |

**Keep arrows short.** If you need more than two actions on an arrow, they
almost always belong in `entry /` instead. An arrow with five items is a
signal that the destination state has implicit setup that should be made
explicit.

For complex multi-step sequences (extend arm, open gripper, lower lift),
give the sequence a name and reference it as a single call:

```
╭───────────╮
│  PICKING  │
│ entry /   │
│  pick_    │
│  sequence │
╰───────────╯
```

Define `pick_sequence()` as a helper in your code. The diagram stays readable
and the detail lives where it belongs — in the implementation.

**Multiple arrow actions** are written with a semicolon. Two is the practical
maximum before it becomes unreadable:

```
button_pressed(1) / blink GREEN; stop buzzer
```

If you reach three, move them to `entry /` of the destination state.

---

## Pseudostates

### Initial pseudostate `●`

Every statechart has exactly one. Draw a filled circle with an arrow pointing
to the first real state. The transition from `●` can carry an action label
(like `/ init robot`) but has no event — it fires immediately at startup.

```
●  ──── init robot  ────►  ╭──────╮
                           │ IDLE │
                           ╰──────╯
```

### Final pseudostate `◉`

Optional. Use it when the mission has a clear end point. The robot does
nothing once it reaches `◉` (or you can loop it back to IDLE).

```
╭──────╮
│ DONE │ ──── / stop() ────► ◉
╰──────╯
```

---

## Complete Example

The diagram below describes a simple LED cycle: pressing BTN_1 advances
through colors, with a green blink confirming each press.

```
                    ●
                    │
                    │ init robot
                    ▼
            ╭──────────────╮ ◄───────────────────────────────────────────╮
            │     IDLE     │                                             │
            │ entry / LEDs │                                             │
            │         off  │                                             │
            ╰──────────────╯                                             │
                    │                                                    │
                    │ button_pressed(1) / blink GREEN                    │
                    ▼                                                    │
            ╭──────────────╮                                             │
            │    ORANGE    │                                             │
            │ entry / light│                                             │
            │  orange LED  │                                             │
            ╰──────────────╯                                             │
                    │                                                    │
                    │ button_pressed(1) / blink GREEN                    │
                    ▼                                                    │
            ╭──────────────╮                                             │
            │    PURPLE    │  ── button_pressed(1) / blink GREEN ────────╯
            │ entry / light│
            │  purple LED  │
            ╰──────────────╯
```

**Reading the diagram:**
- The robot starts in IDLE with all LEDs off
- Each BTN_1 press triggers a transition; the green blink is a transition action
  (instantaneous acknowledgment — firmware handles the timing)
- Arriving in ORANGE or PURPLE lights the corresponding LED (entry action)
- From PURPLE, BTN_1 loops back to IDLE

---

## Conventions for This Platform

### Naming states

Use short UPPERCASE names that describe the robot's *situation*, not
implementation details:

| Good | Avoid |
|------|-------|
| `IDLE`, `MOVING`, `GRIPPING` | `STATE_1`, `WAITING_FOR_FLAG` |
| `APPROACH`, `ALIGNED`, `DONE` | `set_motor_running`, `loop` |

### One active motion per state

A state that calls a motion API (`move_forward`, `purepursuit_follow_path`,
etc.) should be the only state managing that motion. Don't start a new motion
without cancelling the previous one.

### INIT is always the first state

Every diagram for this robot starts with an INIT state (or the `●` pseudostate
transitioning with `/ init robot`). The INIT sequence is fixed:

1. Clear ESTOP/ERROR if needed
2. Set firmware to RUNNING
3. Reset odometry *(if the mission uses position)*
4. Wait for first pose update *(if the mission uses position)*

You do not need to show these four steps in the diagram — they are implied by
`init robot` on the initial transition. The code generator fills them in
automatically.

### Transition actions should be instantaneous

If an action takes time (motor move, servo sweep, sensor read), it belongs in
the *entry action* of the destination state — started as a background task —
not on the transition arrow. The arrow fires in one loop tick.

---

## Checklist Before Handing Off

Before asking an AI agent to generate code from your diagram, verify:

- [ ] Every arrow has at least an event or a guard
- [ ] Trigger and action are separated by `/` on every arrow
- [ ] Every state with `entry /` behavior has the keyword written explicitly
- [ ] No transition action takes significant time (motor moves belong in entry actions)
- [ ] The diagram has exactly one initial pseudostate `●` (or an INIT box)
- [ ] Every state has at least one outgoing transition (no deadlocks)
- [ ] All motion states have a cancel path (e.g., BTN_2 → IDLE)
