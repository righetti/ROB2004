## Extra Steps For MaC PCs

On Mac, there is one extra step before the simulator can be executed. First, make sure you have made your virtual environment and installed all the packages as explained before. When running on Mac, the 'mujoco.viewer.launch_passive(self.model, self.data)' in the 'sim.py' causes problem when invoked by the default python interpreter. You will get an error that says you must run it using `mjpython`:
```bash
`launch_passive` requires that the Python script be run under `mjpython` on macOS
```
First, you need to find the path to the `mjpython`, which can be done using the command "which mjpython" inside your virtual environment. This will print the path to the terminal. You may get something like (`/opt/miniconda3/envs/ROBenv/bin/mjpython`). Then you need to configure the VSCode to use this interpreter instead. While you have the VSCode open, press command + shift + p and search for and select "Python: Select Interpreter" and paste the path to the interpreter in the "Enter Interpreter Path:" section.

After, you want to go to the notebook you want to run, and then select the kernel. If it is already the currently selected kernel, you are good to go. If not, you want to press "Select another Kernel," and "Python Environments...", and then select your interpreter. You should find an interpreter with the path name from earlier and select it. After these steps, you should have switched into the correct interpreter and now be able to run the simulator notebooks properly.
