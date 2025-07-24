this repo is kinda confusing but to start

- make a new virtual environment (python -m venv .venv)
- activate it (.venv/Scripts/activate) - this may change if you are not on windows
- install packages (pip install -r requirements.txt)
- to run the robot controller, run (python robot_controller.py --serial-port *serial port*)
    - the serial port is in the COM* format in windows, an ESP must be connected 