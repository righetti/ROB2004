# Installation Instructions 

Based on the operating system that you have and your preferences, setup a virtual envionment for the labs and activate it. Let's assume you use `miniconda` and have created an environment called `rob2004`. In a terminal, go to your desired directory on the computer and clone Professor Righetti's repository found [here](https://github.com/righetti/ROB2004) by running:

``` 
git clone https://github.com/righetti/ROB2004
```

Note if you are unfamiliar with git, you can download the repository as a zip file by clicking on the green "Code" button on the top right of the repository page and selecting "Download ZIP" and unzip it.

Then activate your environment and simply install the packages required for the lab by running:

```bash
conda activate rob2004
pip install -e .
```
Open the project in VSCode and open the notebook you're interested in. To run the notebook, select the kernel to be the environment you just created. You should now be able to run the notebook on your labptop (only for simulation).

# Extra Notes For Mac
For those of you who have a Mac computer, refer to [here](MacREADME.md) for extra steps required to run on Mac. 



