# Optimum-Mindstorm
Lap Simulation package for LHRe FSAE

# Documentation
TODO

# Installation and Setup

    1) Git bash setup

        Install git bash https://git-scm.com/download/win
        This is for version control :) you may need to do some research in order to learn how this works...

    2) Install Python

        Install latest version of python: https://www.python.org/downloads/
        NOTE: when running installer, on the launching page of the installer, check the "Add Python X.XX to PATH" button to YES

    3) Clone REPO somewhere on your computer

        Go to the folder you want to install the REPO, right click, and click "Git BASH Here"

        Run the following command:

        git clone https://github.com/LonghornRacingElectric/JoVogelOverhaul.git

        You will need to signin to your github account for this to work, and have your github account be added to the LHRe repo

    4) Install Python Packages

        To run the solver/engine/other scripts, installing the necessary python packages is required.

        (The following won't work if you didn't add Python to PATH. If you find the following isn't working, check the following to
        ensure python and pip are added to path: https://datatofish.com/add-python-to-windows-path/)
        
        To install packages, go to the REPO folder in file explorer, right click on the REPO folder, and click 'git bash here'.
        Once on the command line, run the following command:
        (The following line must be run from command line inside the MMM repo)
        
        pip install -r requirements.txt

        If this doesn't work, and you tried adding python to path, you can try the following commands:

        python -m pip install -r requirements.txt

        (If you use anaconda, try this:)

        conda install --file requirements.txt -p ./lib

    5) Install Visual Studio Code

        Install here: https://code.visualstudio.com/download

        I recommend adding jupyter notebooks plugin for easy data analysis 
        (explanation on how to do this here: https://towardsdatascience.com/installing-jupyter-notebook-support-in-visual-studio-code-91887d644c5d)


    6) Running the lapsim Engine

        In your IDE or command line, run the main.py. 
        This will generate the data output from the sweep in this folder: results/{name}.csv
        
        You can do data analysis on this .csv however you want -
        But I heavily recommend using the jupyter notebooks already setup already
        One such notebook is results/lapsim_analysis.ipynb

    ^^^ For any issues with this setup & execution, please ask Kieran Cosgrove ^^^

# Credits
Adapted from Jonathan Vogel Optimum-Mindstorm Simulation package from CUFSAE

Developed by Kieran Cosgrove - email kierancoz.w@gmail.com with any questions
