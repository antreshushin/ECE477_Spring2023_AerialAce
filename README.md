# ECE477_Spring2023_AerialAce
ECE 477 Purdue Senior Design. 

Instructions:
In main func:
  Set 4 float variables, each representing a value for each of the 4 commands.
  Set 1 int var representing the state such that:-
    //1 = initialising
		//2 = cal_unflexed
		//3 = cal_flexed
		//4 = running
		//5 = idle
   
   Call main_LCD(currDisp, 4 floats, state_int);


For print_progress function:
	
	In main call print_progress x number of times where x is number of seconds.
	print_progress(currDisp, seconds, currTim);
	Where currTim keeps track of which patricular second is calling for a progress bar section
