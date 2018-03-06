
Baxter Basic Demo: e.g., Art and Teaching



1. Basic Concept


This is code for a basic demo of the Baxter robot.
You can get the robot to paint by recording motions, or have the robot introduce itself in a classroom, etc.
Extended versions of this code were used for the Baxter robot to paint some paintings for the international robot art competition (robotart.org), and to act as a teaching assistant for the Design of Embedded and Intelligent Systems (DEIS) robotics course at Halmstad University, Sweden.


2. Content

This repository should include:

robot_faces: some example faces for Baxter's display

scripts: the python backend

src: the C++ frontend  

speech_recognition: example files for recognizing some names of electronics parts (e.g. for a quiz)

baxter_manual

this readme file


3. What to do?

-Prepare the code on your system.
Make a catkin package and compile the C++ code with catkin_make

Put the images and sounds somewhere nice, and the speech recognition files where pocketsphinx can find them.
(Make sure that pocketsphinx is installed if you want to use speech recognition.)

Change the paths in the python file based on where you put the images and sounds.

-Follow the instructions in the baxter_manual.


5.Licenses

This code is based on Rethink Robotics' Baxter SDK (www.rethinkrobotics.com). 
Please respect their license.


For the code from this author, the MIT license applies:

Copyright 2018 Martin Cooney

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated dataset and documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
