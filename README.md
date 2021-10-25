# CS-350-EmergingSysArch

## Selected Artifactes
1. Task Scheduler -> gpiointerrupt.c
2. Byte Reader -> uartecho.c

## Summarize the project and what problem it was solving.
The main porject on display is the Task Scheduler. We have been directed to create a prototype of a thermostat for our company. This thermostat should read room temperature and output via LED whether heat should be ON or OFF. Data should be sent out via UART to simulate data being sent to a server, with next phase being the design an implementation of data transmission over WiFi.

## What did you do particularly well?
The design of the State Machines(SM) are an area I felt accomplished in. Used a minimum number of states in each SM to provide the most effienct work. The Set Point SM has an SP_Hold state which is used as a waiting state between button presses. The Check Temperature SM could be better renamed as a Heat SM with H_ON and H_OFF denoting the thermostat as in a heating state or not. Then The Check Seconds SM is used to remove our counting outside of the main loop and make data transfer it's own SM.
```c
// State Machine States
enum SP_States { SP_Start, SP_UP, SP_DOWN, SP_Hold} SP_States; //Set Point States
char Set_Point(char state);
enum CT_States { CT_Start, CT_ON, CT_OFF } CT_States; //Check Temperature States
char Check_Temp(char state);
enum CS_States { CS_Start, CS_ADD } CS_States; // Check Seconds States
char Check_Seconds(char state);
```

## Where could you improve?
I did not develop a Timing Diagram for this project. I think a diagram showing the true times of when states are being switched would show an indication that the CT_State does not always update at the expected time based on UART output with a delay between reporting output from Heat ON and OFF.

I believe an addition improvement if the embedded device is capable is to pass our task details in through a text file. Current task variables are hardcoded. By passing as a text file and then using an iterative loop for setting each task in out task struct array, this could allow for future personalize setting.

## What tools and/or resources are you adding to your support network?
A new understanding of working with embedded devices. Preconceived idea was to just create code to tell the device what to do. Now there is an understanding that multiple factors exist. Factors like interrupts and byte conversion for input handling that were not previously taken into account.

## What skills from this project will be particularly transferable to other projects and/or course work?
Within the relam of coding and not just embedded devices, working with State Machines and Switch statements seem to be a particularly transferable skill and adds a new view point of view for problem solving and working with timed task.

## How did you make this project maintainable, readable, and adaptable?
The project was made maintainable, readable, and adaptable through the use of multiple state machine functions and a task scheduler. Using these versus' a  cluster of code to perform all task in one sitting has allowed for new task creation and updating and changes to current task reachable.
