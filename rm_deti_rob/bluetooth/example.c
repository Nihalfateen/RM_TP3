#include "rmi-mr32.h"
#include "bluetooth_comm.h"


int main(void)
{
	int i = 0;
	initPIC32();
    configBTUart(3, 115200); // Configure Bluetooth UART
    bt_on();     // enable bluetooth channel; printf
                // is now redirected to the bluetooth UART

	while(1)
	{
		printf("Press Start to continue");
		while(!startButton());
		do
		{
			waitTick40ms();
			printf("Cnt=%03d\n", i++);
		} while(!stopButton());
	}
	return 0;
}	
