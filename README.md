# Vision2023-Competition
Vision code for the 2023 season 

How to Setup Vision development environment:

Install Visual Code studio
  Install Viscual Code Studio extensions
  - Use Python (Intellisense from microsoft)
  - WPI extension
  - OpenCV - intellisense

  Optional:
   - Java (for robot code)
   - C/C++ for microsoft

Install python (python.org) 
 - install with path setting enabled 
 - make sure pip is installed, and other default options


Open command line (cmd in windows) - This will also verify install
 - py -m pip install --upgrade pip

To install number python (more math operatios)
 - pip install numpy

To install open CV
 - pip install opencv-python

To get networktables (To talk to our robot)
 - pip3 install --upgrade robotpy
 - pip install pynetworktables
 
To get apriltags (to detect Apriltags)
 - pip install robotpy[robotpy_apriltag]

Clone Vision repository
 - git clone https://github.com/FRC2706/Vision2022-Competition.git

Open folder to Vision repository in VS Code
- MergeViewer lets you run the vision code from your labtop
- MergeFRCPipeline.py is the mainline use for the python code
