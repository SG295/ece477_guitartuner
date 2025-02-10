# Automatic Guitar Tuner 🎸
☆ **Automatic Guitar Tuner Project for ECE477 (Senior Design) @ Purdue University** ☆

**Team Page: https://engineering.purdue.edu/477grp11/Team/team.html**

**Project Description:** Our automatic guitar tuner is a tool anyone can use by selecting the desired note and holding the tool against the tuning peg. A microphone will listen as the string is plucked by the user and make the proper adjustments thorugh a motor. An LCD screen will communicate when the guitar adjustments are finished being made. The design will also have other modes such as a free spin mode to allow the motor to assist in the restringing of a guitar.
Design parameters are:
  * A 3D printed case that holds all the components (PCB, battery, motor, etc.)
  * An STM32 microcontroller that controls all the components of the tuner (battery, motor, LCD screen, microphone)
  * A stepper motor connected to the microcontroller that rotates based on feedback from the microphone
  * A digital microphone that listens when prompted by the microcontroller. First, I2S is used to read the digital signal and then DSP is performed so that the microcontroller knows what frequency is being played.
  * A rechargable battery connected to a battery management system and the microcontroller. Powers all the components in the tuner.
  * A graphical LCD screen that is controlled by buttons on the tuner. It includes "Power", "^", ">", "<", and "v" buttons for the user to press to control the contents of the screen.

***

**For team members:**

Please make a branch for your respective part and title it according (i.e. if you're working on the motor, you can just title it "motor").

The command to create a branch (do this locally after you've pulled):
  * git checkout -b NEW_BRANCH_NAME

The commands (in order) to push your NEW branch: 
  * git add .
  * git commit -m "put message here"
  * git push origin -u NEW_BRANCH_NAME
   
**Once you have created and pushed the branch, you can push normally if you're on the branch. You can check this with "git branch".**

I have setup a platoform IO project here with basic settings with the STM32F407VG Discovery Boards we have at our table (looks like the image below). Make sure you're actually within the project folder (titled "PlatformIO-GuitarTuner") and not this repo when trying to upload code to the board. I'll list some documentation that will be useful below:
  * Reference Manual: https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
  * STM32F407 Data Sheet: https://www.st.com/resource/en/datasheet/dm00037051.pdf

**DISCOVERY BOARD:**
![71fB+DA9BdL _AC_UF894,1000_QL80_](https://github.com/user-attachments/assets/ef7bb40d-5acd-4cdc-8fb9-4a604e6ddd3f)
