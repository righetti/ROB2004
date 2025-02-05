# Installation Instructions 
## Install Anaconda

A virtual environment in Python is a isolated environment, separate from the main Python installation, that allows you to install packages and dependencies for a specific project without affecting the global Python installation. This helps to maintain a clean and organized environment, particularly when working with multiple projects that may have conflicting dependencies.

Anaconda is a distribution of the Python for scientific computing and data science. It includes pre-built packages for data analysis and visualization.

Anaconda includes tools, such as "conda", to create and manage virtual environments, allowing you to switch between environments and manage the packages installed in each environment.

1. Download the Anaconda installer for Windows from [here](https://www.anaconda.com/products/individual#windows).


2. Run the installer and follow the instructions. Make sure to select the option to add Anaconda to the PATH environment variable. Guide on how to add variables to the PATH environment variable can be found [here](https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/).

3. Open the Anaconda Navigator or open the Anaconda prompt by searching for "Anaconda Prompt" in the Start menu. There should be a (base) appearing to the left in your terminal. Close and restart your terminal if this did not work or try running ``` conda init ``` if that helps. 

### Setup the virtual environment

1. To create the virtual environment for the labs, change to your desired directory and clone Professor Righetti's repository found [here](https://github.com/righetti/ROB2004) using

``` git clone https://github.com/righetti/ROB2004```

Note if you are unfamiliar with git, you can download the repository as a zip file by clicking on the green "Code" button on the top right of the repository page and selecting "Download ZIP" and unzip it.

2. Change directory to the laboratory folder:

``` cd ROB2004 ```

3. Create the virtual environment using conda:

```bash 
conda create --name rob2004 python==3.8
```

This should create a virtual environment for the class.

4. Activate the virtual environment:

```conda activate rob2004```

ensure you have the environment activated by checking the name of the environment in the terminal. It should be (rob2004) instead of (base).

5. Install the Python library for the class. While you're in the classes repository, run the following:
```bash
pip install .
```

5. Open the project in VSCode and open the notebook you're interested in.

6. To run the notebook, select the kernel to be the environment you just created. You should now be able to run the notebook on your labptop (only for simulation).



