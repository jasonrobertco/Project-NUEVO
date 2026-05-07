# Lab Manuals

This directory contains source material for the MAE162E lab manuals.

Most older labs are still self-contained LaTeX projects.
Newer operator-facing notes may live directly as Markdown.

## Structure

```
docs/lab_manual/
├── Lab1_ROS2/
│   ├── main.tex               # Lab 1 source
│   ├── figures/               # Images included by main.tex
│   ├── ol-softwaremanual.cls  # (unused template; kept for reference)
│   └── ...
├── Lab_system_setup/
│   ├── README.md              # current markdown operations note
│   └── figures/               # optional images used by the note
└── README.md                  # this file
```

## LaTeX Labs

Each LaTeX lab is a self-contained project inside its own subdirectory.
Compile from inside that directory so relative paths (`figures/`, `code/`)
resolve correctly.

### Option 1 — Overleaf (recommended for sharing)

1. Zip the lab directory (e.g. `Lab1_ROS2/`)
2. On [overleaf.com](https://www.overleaf.com), click **New Project → Upload Project** and upload the zip
3. Set the main document to `main.tex` (Project Settings → Main document)
4. Click **Recompile**

### Option 2 — Local with `latexmk` (recommended for local builds)

Requires a TeX distribution: [TeX Live](https://tug.org/texlive/) (Linux/macOS) or [MiKTeX](https://miktex.org/) (Windows).

**Step 1 — Install Pygments** (required by the `minted` code-listing package):

```bash
pip3 install Pygments
```

**Step 2 — Make sure `pygmentize` is on your PATH.**

After installing, verify with:

```bash
pygmentize -V
```

If the command is not found, add the Python user bin directory to your shell profile (`~/.zshrc` or `~/.bashrc`):

```bash
export PATH="$(python3 -m site --user-base)/bin:$PATH"
```

Then reload your shell (`source ~/.zshrc`) and verify again.

**Step 3 — Compile:**

```bash
cd docs/lab_manual/Lab1_ROS2
latexmk -pdf -shell-escape main.tex
```

`latexmk` automatically runs as many passes as needed and handles `minted`'s auxiliary files correctly.

To clean auxiliary files after building:

```bash
latexmk -c        # remove aux files, keep PDF
latexmk -C        # remove aux files and PDF
```

### Option 4 — VS Code with LaTeX Workshop

1. Install the [LaTeX Workshop](https://marketplace.visualstudio.com/items?itemName=James-Yu.latex-workshop) extension
2. Open `main.tex` in VS Code
3. Save the file — the extension auto-compiles and opens a PDF preview pane
4. Use **Ctrl+Alt+B** (or **Cmd+Alt+B** on macOS) to manually trigger a build

## Required Packages

The lab manuals use standard packages available in any full TeX Live or MiKTeX installation:

`amsmath`, `amssymb`, `fancyhdr`, `graphicx`, `float`, `subcaption`, `caption`,
`tikz`, `circuitikz`, `fullpage`, `hyperref`, `cleveref`, `enumitem`, `multicol`,
`xcolor`, `soul`, `cancel`, `listings`, `booktabs`

If a package is missing, install it via your TeX distribution's package manager:

```bash
# TeX Live
tlmgr install <package-name>

# MiKTeX — packages install automatically on first use, or via the MiKTeX Console
```

## Adding a New Lab

If the new lab is LaTeX-based:

1. Copy an existing lab directory as a starting point:
   ```bash
   cp -r docs/lab_manual/Lab1_ROS2 docs/lab_manual/Lab2_MotionControl
   ```
2. Edit `main.tex` — update the `\title{}` and replace section content
3. Add figures to the `figures/` subdirectory and reference them with `\includegraphics{figures/<name>}`
4. Add this lab to the table below

If the new lab is Markdown-based:

1. Create a new lab directory
2. Add a `README.md` as the primary document
3. Keep any images in a local `figures/` directory
4. Add this lab to the table below

## Lab Index

| Lab | Title | Directory |
|-----|-------|-----------|
| 1 | ROS2 Introduction and System Integration | `Lab1_ROS2/` |
| System Setup | Raspberry Pi setup and runtime operations | `Lab_system_setup/` |
