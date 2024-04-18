We have implemented a design with states to take and process the input.
All the instructions have to be preloaded into the "instr_mem.txt".
The Program will start with PC = 0, i.e from the starting instruction in the "instr_mem.txt"

The design is implemented as 
1) Data Unit module named as "data_path"
2) Control Unit module named as "control_path"
3) A top level module names as "top_module" in which both the above modules are instantiated.

All the requrired data storage and lines reside in Data unit.
The control Unit has read access to the Instruction Register in DU.
The control will provide with several control signals for the data flow in DU.

Each instruction has 2 states. 
    0) The Instruction Fetch and decode.
        The instruction is fetched from memory and appropriate control signals are generated for the operation.
    1) Operations and write results.
    	The control signals dictate the flow of data by the selector lines in MUX. 
    	All the results are written to appropriate places once the calculation is complete.
    	
    	
    	
A sample run is demonstrated using iverilog simulation.
Given Instructions :
    0: ST R2,0(R4)   // opcode = 101010
    1: CALL #16	     // opcode = 100000. 16 is the starting address of Instr 4. Each instr is 4 bytes
    2: NOP 	     // opcode = 110011. Do Nothing 
    3: HALT	     // opcode = 110010
    4: NOP
    5: NOP
    6: RET	     // opcode = 110001. Return to the next instr of the last_called place.
    

initially PC = 0, SP = 1024
*) instr 0 is executed and PC becomes PC = 4 and SP remains Same
*) instr 1 is executed ans PC becomes PC = 16 while storing the next_instr in stack. SP becomes 1020.
*) instr 4 is executed and PC becomes PC = 20. SP remains same
*) instr 5 is executed and PC becomes PC = 24. SP remains same
*) instr 6 is executed and PC returns to the value in stack PC = 8. and Stack becomes 1024.
*) instr 2 is executed and PC becomes PC = 12 and SP remains same.
*) instr 3 is seen and Program is halted immediately.

Sample run screenshot is attached with annotation.
