#!/bin/bash

VENV_PATH="$(pyenv prefix dronogeddon_env)"

export PATH="$VENV_PATH/bin:$PATH"
export PYTHONPATH="$VENV_PATH/lib/python3.10/site-packages:$PYTHONPATH"

echo "Switched to PyEnv virtualenv: $VENV_PATH"