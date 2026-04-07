# AI Code Generation from UML Statechart

This folder contains everything needed to generate a `main.py` FSM from a UML
statechart design using any AI coding agent (Claude Code, Codex, Gemini CLI,
Cursor, etc.).

## What's in this folder

| File | Purpose |
|------|---------|
| `uml_statechart_guide.md` | How to draw a valid UML statechart for this robot |
| `ai_agent_codegen.md` | Instructions the agent reads to understand how to generate code |
| `test_UML.png` | Example UML diagram |
| `test_UML_generated.py` | Code generated from `test_UML.png` — reference output |

---

## How to generate code from a diagram

### Step 1 — Prepare your statechart

Follow the conventions in `uml_statechart_guide.md` and run through the
checklist at the bottom before proceeding.

**Your design can be in any format the agent can read:**

| Format | Examples |
|--------|---------|
| Image | `.png`, `.jpg`, `.svg`, `.pdf` |
| Text / markup | Markdown description, PlantUML, Mermaid, plain text |
| Multiple files | Split across diagrams, one file per superstate, a PDF + a text annotation |

For complex designs with superstates and substates, splitting across multiple
files is encouraged — see the section below.

### Step 2 — Prompt the agent

Give the agent the following prompt. Replace `<path>` with the absolute path
to `ros2_ws/src/robot` and list your design files under item 5:

---

```
You are a code generation agent for a ROS2 robot platform.
Your job is to read a UML statechart design and produce a complete main.py file.

Read these files in order before generating:

1. <path>/examples/ai_codegen/ai_agent_codegen.md  — generation rules
2. <path>/robot/README.md                          — setup flow and API overview
3. <path>/robot/API_REFERENCE.md                   — full method signatures
4. <path>/robot/hardware_map.py                    — enums and constants
5. <your design file(s)>                           — the statechart to implement
   e.g. my_mission.png
        my_mission_superstates.pdf
        my_mission_notes.md

Write the generated code to:
<path>/robot/main.py

Follow every rule in ai_agent_codegen.md exactly.
Do not add features not shown in the design.
Do not modify any file other than main.py.
```

---

### Step 3 — Review and run

Before running, verify:
- States in code match states in the diagram
- Entry actions are in the *previous* state's branch (not inside the state they belong to)
- Transition actions appear before `state = "..."`, after any cleanup
- INIT includes the ESTOP/ERROR check
- Tick-rate control block is present and unmodified

Copy to `main.py` if the agent wrote to a different file, then:

```bash
ros2 run robot robot
```

---

## Working with superstates and substates

For complex designs, split the description across multiple files and list them
all under item 5 of the prompt. The agent will read them together.

**Recommended structure:**

```
my_mission/
├── overview.png         ← top-level superstate diagram
├── superstate_ARM.png   ← detail of the ARM superstate and its substates
├── superstate_DRIVE.png ← detail of the DRIVE superstate
└── notes.md             ← any constraints, timings, or edge cases in plain text
```

**How superstates translate to code:**

A superstate groups substates that share common transitions (e.g., any substate
in `ARMED` goes to `SAFE` on `estop_pressed`). The agent will flatten this into
prefixed state names and repeat the shared guard in each substate:

```
Superstate ARMED
├── substate IDLE     → code state "ARMED_IDLE"
├── substate MOVING   → code state "ARMED_MOVING"
└── shared transition: estop_pressed / → SAFE
```

```python
elif state == "ARMED_IDLE":
    if robot.get_button(Button.BTN_ESTOP):   # shared superstate transition
        robot.stop()
        state = "SAFE"
    elif robot.was_button_pressed(Button.BTN_1):
        state = "ARMED_MOVING"

elif state == "ARMED_MOVING":
    if robot.get_button(Button.BTN_ESTOP):   # shared superstate transition
        robot.stop()
        state = "SAFE"
    elif drive_handle.is_finished():
        state = "ARMED_IDLE"
```

If the shared transition logic is non-trivial, tell the agent to factor it into
a helper function (e.g., `check_estop(robot, state)`) and call it at the top
of each substate branch.

---

## Agent-specific notes

### Claude Code / claude-cli
Reads images, PDFs, and text files natively. List all design files by path —
no special handling needed.

### Cursor / Copilot (IDE agents)
Attach files as context using the IDE's attachment feature. For multi-file
designs, open all files in the editor before prompting. Also keep
`ai_agent_codegen.md` open so it is included in the context window.

### Gemini CLI / Codex CLI
Pass files via the CLI's `--file` or `--context` flags. If the agent cannot
read images, export the diagram as PlantUML or Mermaid text, or write a plain
text description using the notation from `uml_statechart_guide.md`.

### Agents without filesystem access
Paste the contents of `ai_agent_codegen.md` directly into the prompt, then
describe your statechart as text using the notation in `uml_statechart_guide.md`.
For multi-file designs, concatenate all descriptions into one prompt block with
clear section headings.

---

## Updating the instructions

If the Robot API changes, update `ai_agent_codegen.md` (API quick reference
section) and `../README.md` / `../API_REFERENCE.md` as described in the
maintenance checklist in `API_REFERENCE.md`. The prompt itself does not need
to change.
