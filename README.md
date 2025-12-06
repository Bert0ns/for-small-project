# Foundations of Operation research - Small Project

## How to create a virtual enviroment

1. `python3 -m venv .venv`
2. `source .venv/bin/activate`
3. `pip install -r requirements.txt`
    4. If cffi installation fails you need to install build tools: `sudo apt update && sudo apt install -y build-essential python3-dev libffi-dev`

## Examples

```bash
python ./main.py data/Building1.csv
```

```bash
python ./main.py data/Building2.csv
```
