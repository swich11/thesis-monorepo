from pathlib import Path

print(__file__)
myPath = Path(__file__)/'datasets'

print(myPath.resolve())