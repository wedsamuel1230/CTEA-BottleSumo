# UV Environment Setup Guide

## What's Been Configured

This project now uses **uv** for Python environment management with **Python 3.13**.

### Project Structure
```
realtime_monitor/
├── .venv/              # Virtual environment (Python 3.13.7)
├── pyproject.toml      # Project configuration
├── viewer.py           # Main application
├── run_viewer.sh       # Convenience script to run viewer
└── SETUP.md           # This file
```

## How to Use

### Option 1: Using the convenience script (Recommended)
```bash
cd realtime_monitor
./run_viewer.sh
```

### Option 2: Manual activation
```bash
cd realtime_monitor
source .venv/bin/activate
python viewer.py
```

### Option 3: Using uv directly
```bash
cd realtime_monitor
uv run viewer.py
```

## VS Code Integration

The workspace is configured to automatically use the Python 3.13 virtual environment:
- Just open the project in VS Code
- The Python extension will use `.venv/bin/python` automatically
- Press F5 or click the Run button to debug

## Environment Details

- **Python Version**: 3.13.7
- **Environment Manager**: uv 0.8.22
- **Tkinter**: Built-in (version 8.6)
- **TK_SILENCE_DEPRECATION**: Enabled (no more warnings!)

## Adding Dependencies

If you need to add Python packages:
```bash
# Activate the environment first
source .venv/bin/activate

# Install with uv
uv pip install package-name

# Or update pyproject.toml and sync
uv sync
```

## Troubleshooting

### If Python 3.13 isn't found:
```bash
source $HOME/.local/bin/env
uv python install 3.13
```

### If the environment gets corrupted:
```bash
rm -rf .venv
uv venv --python 3.13
source .venv/bin/activate
```

### If uv isn't in PATH:
```bash
source $HOME/.local/bin/env
# Or add this to your ~/.zshrc:
export PATH="$HOME/.local/bin:$PATH"
```

## Why This Setup?

- ✅ **Modern Python**: Python 3.13 with latest features and performance improvements
- ✅ **No Tk Warning**: TK_SILENCE_DEPRECATION is set in the environment
- ✅ **Fast**: uv is significantly faster than pip/conda
- ✅ **Isolated**: Virtual environment keeps dependencies separate
- ✅ **Cross-platform**: Works on macOS, Linux, and Windows
