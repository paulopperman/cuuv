# Threat UUV modeling

Repository for threat UUV models and simulation.  Detailed documentation is developed in the [wiki](https://gitlab.nps.edu/team1-cuuv/threat-uuv-modeling/wikis/home).

# Technology
*Describe software, languages, and packages required to run the code.*

* NetLogo 6.0.4
* Python 3.6
  * [Jupyter](http://jupyter.org/) - browser based tool for working with jupyter notebooks (`*.ipynb` files).
  * [numpy](http://www.numpy.org/) - python mathematics library
  * [matplotlib](https://matplotlib.org/) - python plotting library
  * [pandas](https://pandas.pydata.org/) - data processing and analysis library
* R 3.5.1 - statistical analysis language
  * [IRkernel](https://github.com/IRkernel/IRkernel) - Jupyer notebook kernel for the R language

# Setup / Installation Instructions
*Describe steps for how to execute the code.*

#### NetLogo

Download and run the installation file for the desired version at https://ccl.northwestern.edu/netlogo/oldversions.shtml

#### Python

The recommended python setup uses anaconda.

1. Download the anaconda installer for python 3.x from [here](https://www.anaconda.com/download/).
2. Install anaconda, following these [installation instructions](https://docs.anaconda.com/anaconda/install/).
3. Use [Anaconda Navigator](https://docs.anaconda.com/anaconda/navigator/getting-started/) or the [conda command line interface](https://conda.io/docs/user-guide/getting-started.html) to create a Python 3 environment.  The conda command to create the environment and install packages is `conda create -n nps-env python=3.6 pandas jupyter numpy matplotlib`

# Code Structure

The main simulation loop runs in `potential-field-model.nlogo`.  Detailed
procedures for simulating specific components and setting up the environment are
included as separate `*.nls` code files.

The `Mission Profile.ipynb` file is used to develop mission profile patch data to be loaded into the netlogo simulation.  The output location must be consistent with the file locations in the netlogo files.

# References
*Cite applicable references for software or models*

Barisic, Matko, Nikola Miskovic, and Zoran Vukic. 2009. Heuristic Parameter Tuning Procedures for a Virtual Potential Based AUV Trajectory Planner. IFAC Proceedings Volumes (IFAC-PapersOnline). Vol. 42. IFAC. https://doi.org/10.3182/20090916-3-BR-3001.0043.

Healey, A. 2006. “Guidance Laws, Obstacle Avoidance and Artificial Potential Functions.” In Advances in Unmanned Marine Vehicles, edited by G. N. Roberts and R. Sutton, 43–66. London: Institution of Engineering and Technology. https://doi.org/10.1049/PBCE069E_ch3.

Rimon, Elon, and Daniel E Koditschek. 1992. “Exact Robot Navigation Using Artificial Potential Functions.” IEEE Transactions on Robotics and Automation 8 (5): 501–18. http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=163777.

Wilenski, Uri. 1999. “NetLogo.” Evanston, IL: Center for Connected Learning and Computer-Based Modeling, Northwestern University. http://ccl.northwestern.edu/netlogo/.
