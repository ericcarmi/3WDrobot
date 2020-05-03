

struct Calibration{

    int mode = 0;
    bool countbool = false;
    int counter = 0;

    def reset()
    {
	mode = 0;
	countbool = false;
	counter = 0;
    }

    def linearPWMsweep(int T)
    {
	int speed = 1;
	while(speed < 256)
	{
	    // Wait for some amount of time before increasing speed
	    // Should use binary dirac delta
	    // Set period with integer, counter increases every loop
	    //
	    counter+=1;
	    countbool = (counter%T) and 1;
	    if(!countbool)
	    {
		counter = 0;
		speed += 1;
	    }
	}
    }

};
