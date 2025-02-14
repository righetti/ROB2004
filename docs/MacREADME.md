## A quick tutorial for editing the interpreter for use on mac

On Mac, there is one extra step, after you have finished setting up your environment and installing all the packages, to run the simulator in an IDE. The problem is with the mujoco.viewer.launch_passive(self.model, self.data) in the sim.py, which is invoked when you run a finger simulator (robot_sim = NYUFingerSimulator()). On Mac, if you run this using a python interpreter, which is the standard environment, you will get an error that says you must run it using mjpython:

"`launch_passive` requires that the Python script be run under `mjpython` on macOS"

This is an issue since the only normal way you can run the Jupyter Notebook is to create the mjpython shell in terminal, inside your virtual environment, and then copy and paste the cells to run the notebook.

However, there is a solution in that you make a new interpreter based on your virtual environment and run it based on that interpreter.

You need to find the path of the interpreter first, which can be done using the command "which mjpython" inside your virtual environment. This will print the path to the terminal.

You may get something like (/opt/miniconda3/envs/ROBenv/bin/mjpython), and the next step is to configure the interpreter. I would recommend VS Code for this step, but you can search how to change interpreters in other IDEs.
In VS code, you want to invoke the command pallete selector (Should be command + shift + p on Mac), and get the option for "Python: Select Interpreter". After pressing that, you want to paste the path to the interpreter in the "Enter Interpreter Path:" section.

After, you want to go to the notebook you want to run, and then select the kernel. If it is already the currently selected kernel, you are good to go. If not, you want to press "Select another Kernel," and "Python Environments...", and then select your interpreter. You should find an interpreter with the path name from earlier and select it. After these steps, you should have switched into the correct interpreter and now be able to run the simulator notebooks properly.
