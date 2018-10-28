# Team 1072 - Summer Robot Code 2018
We set out to master the ins and outs of FRC robot code over a single summer based around the POWER UP! game concept. We aimed to learn the essentials of both teleoperated and autonomous functions to best prepare us to assist in robot code next build season. Feel free to follow along with our progress at [1072 GitHub Landing - Projects](harkerrobo.github.io/projects.html).
## Cloning this repository
To clone this repository to your computer, first navigate to the required directory.
```
cd #enter your required directory here
```
Then, clone the project into this directory, which will create a folder for you containing the project contents.
```
git clone https://github.com/HarkerRobo/SummerRoboCode2018
```
Finally wrap up the setup with the following commands to ensure the repository was correctly cloned:
```
cd SummerRoboCode2018
git fetch origin 
git checkout master
```
## Project Structure
### Robot
#### team1072.robot
Contains the commands and subsystems packages as well as the Robot class (fundamental base of code), all constants used in the code, and a class to handle input/output.
### Commands
#### team1072.robot.commands.auton
Contains all commands (or singular actions to be performed by the robot, per WPILib's command-based structure) related to the structure of the autonomous period.
#### team1072.robot.commands.drivetrain
Contains all commands to be primarily applied to the robot's drivetrain, including for both manual and robot-controlled functionality.
#### team1072.robot.commands.elevator
Contains all commands to be primarily applied to the robot's elevator, including for both manual and robot-controlled functionality.
#### team1072.robot.commands.intake
Contains all commands to be primarily applied to the robot's intake.
#### team1072.robot.commands.util
Contains all utility commands, most of which can be reapplied to future FRC code bases.
### Subsystems
#### team1072.robot.subsystems
Contains all subsystems (or singular parts of the robot on which only one command can be performed at once, per the WPILib's command-based structure) on the physical robot.

## Authors
* **Finn Frankis** - *Wrote bulk of code (with significant assistance from others below)* - [FinnitoProductions](https://github.com/FinnitoProductions)
* **Rahul Goyal** - *Provided continual advice and assistance, created majority of autonomous paths* - [bfte17](https://github.com/bfte17)
* **Anand Rajamani** - *Served as guide and mentor throughout process* - [asid61](https://github.com/asid61)
