#include <iostream>
#include <wiringPi.h>
#include <stdio.h>
#include <string.h>

using namespace std;

typedef unsigned char 		byte;

/*
 +-----+-----+---------+------+---+---Pi 3---+---+------+---------+-----+-----+
 | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
 +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
 |   2 |   8 |   SDA.1 | ALT0 | 1 |  3 || 4  |   |      | 5v      |     |     |
 |   3 |   9 |   SCL.1 | ALT0 | 1 |  5 || 6  |   |      | 0v      |     |     |
 |   4 |   7 | GPIO. 7 |   IN | 0 |  7 || 8  | 1 | ALT5 | TxD     | 15  | 14  |
 |     |     |      0v |      |   |  9 || 10 | 1 | ALT5 | RxD     | 16  | 15  |
 |  17 |   0 | GPIO. 0 |   IN | 0 | 11 || 12 | 0 | IN   | GPIO. 1 | 1   | 18  |
 |  27 |   2 | GPIO. 2 |   IN | 0 | 13 || 14 |   |      | 0v      |     |     |
 |  22 |   3 | GPIO. 3 |   IN | 0 | 15 || 16 | 0 | IN   | GPIO. 4 | 4   | 23  |
 |     |     |    3.3v |      |   | 17 || 18 | 0 | IN   | GPIO. 5 | 5   | 24  |
 |  10 |  12 |    MOSI | ALT0 | 0 | 19 || 20 |   |      | 0v      |     |     |
 |   9 |  13 |    MISO | ALT0 | 0 | 21 || 22 | 0 | IN   | GPIO. 6 | 6   | 25  |
 |  11 |  14 |    SCLK | ALT0 | 0 | 23 || 24 | 1 | OUT  | CE0     | 10  | 8   |
 |     |     |      0v |      |   | 25 || 26 | 1 | OUT  | CE1     | 11  | 7   |
 |   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
 |   5 |  21 | GPIO.21 |   IN | 0 | 29 || 30 |   |      | 0v      |     |     |
 |   6 |  22 | GPIO.22 |   IN | 1 | 31 || 32 | 0 | IN   | GPIO.26 | 26  | 12  |
 |  13 |  23 | GPIO.23 |   IN | 0 | 33 || 34 |   |      | 0v      |     |     |
 |  19 |  24 | GPIO.24 |   IN | 0 | 35 || 36 | 0 | IN   | GPIO.27 | 27  | 16  |
 |  26 |  25 | GPIO.25 |   IN | 0 | 37 || 38 | 0 | IN   | GPIO.28 | 28  | 20  |
 |     |     |      0v |      |   | 39 || 40 | 0 | IN   | GPIO.29 | 29  | 21  |
 +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
 +-----+-----+---------+------+---+---Pi 3---+---+------+---------+-----+-----+
 */

#define PI_SPI_MOSI 10	// pin 19
#define PI_SPI_MISO 9	// pin 21
#define PI_SPI_SCLK 11	// pin 23
#define PI_SPI_CE0	8	// pin 24

void RFM69_IO_CS0(int pState)
{
	digitalWrite(PI_SPI_CE0, pState);	
}

void RFM69_IO_SCK(int pState)
{
	digitalWrite(PI_SPI_SCLK, pState);	
}

void RFM69_IO_MOSI(int pState)
{
	digitalWrite(PI_SPI_MOSI, pState);		
}

int RFM69_IO_MISO(void)
{
	return digitalRead(PI_SPI_MISO);
}

void RFM_SPI_Dly(void)
{
	volatile int dummy = 100;
	
	while (dummy)
	{
		
		dummy--;
	}
}

void SPI_Enable_Bus_Drivers()
{
	RFM69_IO_CS0(1);
	RFM69_IO_SCK(1);
	RFM69_IO_MOSI(1);
	
	pinMode(PI_SPI_MISO, INPUT);
	pinMode(PI_SPI_MOSI, OUTPUT);
	pinMode(PI_SPI_CE0, OUTPUT);
	pinMode(PI_SPI_SCLK, OUTPUT);
}

void SPI_Disable_Bus_Drivers()
{	
	pinMode(PI_SPI_MISO, INPUT);
	pinMode(PI_SPI_MOSI, INPUT);
	pinMode(PI_SPI_CE0, INPUT);
	pinMode(PI_SPI_SCLK, INPUT);
}

void SPI_Open(void)
{
	RFM69_IO_CS0(0);
	RFM_SPI_Dly();
	RFM69_IO_SCK(0);
	RFM_SPI_Dly();
}

void SPI_Close(void)
{
	RFM_SPI_Dly();
	
	RFM69_IO_SCK(0);
	
	RFM_SPI_Dly();
	
	RFM69_IO_MOSI(1);
	
	RFM_SPI_Dly();
	
	RFM69_IO_CS0(1); 	
}


void SPI_Raw_Write_Byte(byte WrPara)
{
	byte bitcnt;    
    
	for (bitcnt = 8; bitcnt != 0; bitcnt--)
	{
		RFM69_IO_SCK(0);
		RFM_SPI_Dly();
		
		if (WrPara & 0x80)
			RFM69_IO_MOSI(1);
		else
			RFM69_IO_MOSI(0);
		
		RFM_SPI_Dly();
		RFM69_IO_SCK(1);
		
		RFM_SPI_Dly();
		
		WrPara <<= 1;
	}        
}

byte SPI_Raw_Read_Byte(void)
{
	byte RdPara = 0;
	byte bitcnt;
	volatile byte highs = 0;
	volatile byte lows = 0;

	for (bitcnt = 8; bitcnt != 0; bitcnt--)
	{
		RFM69_IO_SCK(0);
		RdPara <<= 1;
		RFM_SPI_Dly();        
		RFM69_IO_SCK(1);
		RFM_SPI_Dly();

		if (RFM69_IO_MISO() == 0)
		{
			lows++;
		}
		else
		{			
			highs++;
			RdPara |= 0x01;
		}
	}
    
	return (RdPara);
}

void SPI_Command_Send(const byte pCommand,const uint32_t pAdr, byte *Rd_Data, const uint32_t rd_size)
{
	SPI_Open();
	SPI_Raw_Write_Byte(pCommand);   
	SPI_Raw_Write_Byte(pAdr>>16);
	SPI_Raw_Write_Byte(pAdr>>8);
	SPI_Raw_Write_Byte(pAdr>>0);
	
	for (uint32_t x = 0; x < rd_size; x++)
	{
		Rd_Data[x] = SPI_Raw_Read_Byte();
	}
	
	SPI_Close();
    
	return;
}


int main(int argc, char *argv[])
{
	char sz[] = "Hello, World!";	//Hover mouse over "sz" while debugging to see its contents
	
	int setup_res = wiringPiSetupGpio();
	if (setup_res == 0)
	{
		cout << "Setup_IO() Passed" << endl;
		SPI_Enable_Bus_Drivers();
	}
	else
	{
		cout << "Setup_IO() Failed" << endl;
		SPI_Disable_Bus_Drivers();
		return -1;
	}
	
	static byte test1[0x100000];	
	SPI_Command_Send(0x03, 0x000000,&test1[0], sizeof(test1));
	
	cout << "first read done" << endl;
	
	static byte test2[0x100000];	
	SPI_Command_Send(0x03, 0x000000,&test2[0], sizeof(test2));
	
	cout << "second read done" << endl;

	static byte test3[0x100000];	
	SPI_Command_Send(0x03, 0x000000,&test3[0], sizeof(test3));

	cout << "third read done" << endl;

	int res1 = memcmp(test1, test2, sizeof(test1));
	int res2 = memcmp(test1, test3, sizeof(test1));
	
	if (res1 == 0 && res2 == 0)
	{
		FILE *ptr = fopen("/home/pi/Desktop/test1.bin","wb");  // r for read, b for binary
		fwrite(test1,sizeof(test1),1,ptr); // read 10 bytes to our buffer
		fclose(ptr);
		cout << "File written" << endl;
	}
	
//	static byte settings[256];	
//	SPI_Command_Send(0x03, 0x0FF100,&settings[0], 256);
	
	SPI_Disable_Bus_Drivers();
	
	cout << "Disabled Drivers" << endl;
	return 0;
}