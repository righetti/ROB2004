## A Quick Tutorial on using a linux terminal

In this Tutorial we are going to cover few main commands that will be used extensively in class. 
To begin open a new terminal. To open a terminal, find the launcher and then type terminal. Click on the terminal icon to open the terminal. 

The aim of this tutorial is to equip you with the required commands so that you can navigate to the nyu_finger directory stored in your PC. We will then download the lab exercises for this course in this location and work on the robots ! You will use this commands every class to run your code ! So feel free to comeback to it whenever you forget something. 


The six commands that we are going to learn are : 

## Navigating using linux terminal  

1. ```ls``` : ls is short for list. This command shows all the folders in the current location. To see what folders are in your current location type ls in your terminal and press ENTER. 

2. ```cd``` : cd is short for change directory. We use this command to navigate into a desired folder. For example to go into the devel folder in your pc, type cd ./devel and press ENTER. This will take you from the root directory into the devel directory. Now continue to use ls and cd commands to navigate to the nyu_finger directory. The nyu_finger directory is located in the ./devel/workspace/src/nyu_finger.

3. ```mkdir``` : this command is short for make a directory. Use this command to create a directory with the names of your team members. That is type mkdir yourname_yourteamatename and press ENTER. 

Now that you have created your own directory, navigate into this directory so that we can then download the course exercise materials. To download the materials we will use an online version control system called git. It helps store both small and large projects (software) and helps keep track of all the changed made to it over time. 

## cloning a repository from github

4. ```git clone git@github.com:righetti/ROB2004.git``` : The clone argument tells git to download all the code that is stored in the url that follows it. In this case the code is stored in this url - https://github.com/righetti/ROB2004 . To clone the directory copy the above command on your terminal and press ENTER. Note : you need to use ```ctrl + shift + V``` to paste from the clipboard into the terminal. 

After cloning the repository navigate into your first lab exercise using the first 2 commands that you just learned. The first lab can be found in ```./ROB2004/laboratory/Lab1 ``` . 

## Opening Jupyter Notebook

5. Your last step is to start a jupyter notebook so that you can see the exercises that you need to complete. To do this type ```jupyter notebook``` in your terminal and press ENTER. This will then open an internet browser (firefox) showing the list of exercise files. You need to only study the jupyter notebook files which have the ```.ipynb``` extension. 


## Updating your github repository (Please skip this for you first lab)

6. After your first lab you will need to update the ROB2004 repository that you cloned previously to update the local code stored on your machine to the latest version available online. To do this, you will first need to store your changes of the previous lab to git by using the following command - ```git add --all```. This command tells gits to find all the files that have been modified along with their changes. Then use this command ```git commit --all -m "your message"```. The commit command tells git to store all these changes. After which you can update your code by using the command ```git pull```. 
