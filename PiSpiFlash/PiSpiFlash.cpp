#include <iostream>
#include <wiringPi.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>


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
#define PI_SPI_CE1	7	// pin 26

#define JEDEC_SPI_READ_DATA_BYTES		0x03
#define JEDEC_SPI_READ_IDENTIFICATION	0x9F
#define JEDEC_SPI_SECTOR_ERASE			0xD8

#define JEDEC_SPI_WRITE_PAGES			0x02

#define JEDEC_SPI_READ_STATUS_REGISTER	0x05

#define JEDEC_SPI_WRITE_ENABLE			0x06
#define JEDEC_SPI_WRITE_DISABLE			0x04

volatile uint32_t SPI_Soft_Delay_Reload_Slow = 200;
volatile uint32_t SPI_Soft_Delay_Reload_Fast = 100;

volatile uint32_t SPI_Soft_Delay_Reload = SPI_Soft_Delay_Reload_Fast;

typedef enum
{
	NONE,CE0,CE1
}spi_port_t;

uint32_t GetTickCount()
{
	struct timespec ts;
	unsigned theTick = 0U;
	clock_gettime(CLOCK_REALTIME, &ts);
	theTick  = ts.tv_nsec / 1000000;
	theTick += ts.tv_sec * 1000;
	return theTick;
}

uint64_t Get_Checksum(const byte* pBuffer,uint32_t pSize)
{
	uint64_t check_temp = 0xFFFFFFFFFFFFFFFF;
		
	for (uint32_t x = 0; x < pSize; x++)
	{
		check_temp -= (uint64_t)pBuffer[x];
	}
		
	return check_temp;
}

void SPI_IO_Clock_Fast()
{
	SPI_Soft_Delay_Reload = SPI_Soft_Delay_Reload_Fast;
}

void SPI_IO_Clock_Slow()
{
	SPI_Soft_Delay_Reload = SPI_Soft_Delay_Reload_Slow;
}

void SPI_IO_CS(const spi_port_t pPort,const int pState)
{
	if (pPort == spi_port_t::CE0)
	{
		digitalWrite(PI_SPI_CE0, pState);
	}
	else if (pPort == spi_port_t::CE1)
	{
		digitalWrite(PI_SPI_CE1, pState);
	}
}

void SPI_IO_SCK(const int pState)
{
	digitalWrite(PI_SPI_SCLK, pState);	
}

void SPI_IO_MOSI(const int pState)
{
	digitalWrite(PI_SPI_MOSI, pState);		
}

int SPI_IO_MISO()
{
	return digitalRead(PI_SPI_MISO);
}

void SPI_IO_Soft_Delay()
{
	volatile static uint32_t dummy =  SPI_Soft_Delay_Reload;
	
	while (dummy)
	{
		
		dummy--;
	}
}

void SPI_IO_Enable_Bus_Drivers()
{
	SPI_IO_CS(spi_port_t::CE0,1);
	SPI_IO_CS(spi_port_t::CE1,1);
	
	SPI_IO_SCK(1);
	SPI_IO_MOSI(1);
	
	pinMode(PI_SPI_MISO, INPUT);
	pinMode(PI_SPI_MOSI, OUTPUT);
	pinMode(PI_SPI_CE0, OUTPUT);
	pinMode(PI_SPI_CE1, OUTPUT);
	pinMode(PI_SPI_SCLK, OUTPUT);
}

void SPI_IO_Disable_Bus_Drivers()
{	
	pinMode(PI_SPI_MISO, INPUT);
	pinMode(PI_SPI_MOSI, INPUT);
	pinMode(PI_SPI_CE0, INPUT);
	pinMode(PI_SPI_CE1, INPUT);
	pinMode(PI_SPI_SCLK, INPUT);
}

void SPI_IO_Open(const spi_port_t pPort)
{
	SPI_IO_CS(pPort,0);	
	SPI_IO_Soft_Delay();
}

void SPI_IO_Close(const spi_port_t pPort)
{	
	SPI_IO_CS(pPort,1);
	SPI_IO_Soft_Delay();
	SPI_IO_MOSI(1);	
}


void SPI_IO_Raw_Write_Byte(const byte WrPara)
{
	byte bitcnt;
	volatile byte temp = WrPara;
    
	for (bitcnt = 8; bitcnt != 0; bitcnt--)
	{
		SPI_IO_SCK(0);
		SPI_IO_Soft_Delay();
		
		if (temp & 0x80)
			SPI_IO_MOSI(1);
		else
			SPI_IO_MOSI(0);
		
		SPI_IO_Soft_Delay();
		SPI_IO_SCK(1);
		
		SPI_IO_Soft_Delay();
		
		temp <<= 1;
	}        
}

byte SPI_IO_Raw_Read_Byte()
{
	byte RdPara = 0;
	byte bitcnt;
	volatile byte highs = 0;
	volatile byte lows = 0;

	for (bitcnt = 8; bitcnt != 0; bitcnt--)
	{
		SPI_IO_SCK(0);
		RdPara <<= 1;
		SPI_IO_Soft_Delay();        
		SPI_IO_SCK(1);
		SPI_IO_Soft_Delay();

		if (SPI_IO_MISO() == 0)
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

void SPI_IO_Command_with_Address_Send(const spi_port_t pPort,const byte pCommand,const uint32_t pAdr, byte *Rd_Data, const uint32_t rd_size)
{
	SPI_IO_Open(pPort);
	
	SPI_IO_Raw_Write_Byte(pCommand);   
	SPI_IO_Raw_Write_Byte(pAdr>>16);
	SPI_IO_Raw_Write_Byte(pAdr>>8);
	SPI_IO_Raw_Write_Byte(pAdr>>0);
	
	for (uint32_t x = 0; x < rd_size; x++)
	{
		Rd_Data[x] = SPI_IO_Raw_Read_Byte();
	}
	
	SPI_IO_Close(pPort);
    
	return;
}

void SPI_READ_JEDEC(const spi_port_t pPort, byte* pBuffer)
{
	SPI_IO_Open(pPort);
	
	SPI_IO_Raw_Write_Byte(JEDEC_SPI_READ_IDENTIFICATION);   
	
	for (uint32_t x = 0; x < 3; x++)
	{
		pBuffer[x] = SPI_IO_Raw_Read_Byte();
	}
	
	SPI_IO_Close(pPort);
}

void SPI_IO_READ_STATUS_REGISTER(const spi_port_t pPort, byte* pBuffer)
{
	SPI_IO_Open(pPort);
	
	SPI_IO_Raw_Write_Byte(JEDEC_SPI_READ_STATUS_REGISTER);
	
	pBuffer[0] = SPI_IO_Raw_Read_Byte();
	
	SPI_IO_Close(pPort);
}

void SPI_IO_SIMPLE_COMMAND(const spi_port_t pPort,const byte pCommand)
{
	SPI_IO_Open(pPort);
	SPI_IO_Raw_Write_Byte(pCommand);
	SPI_IO_Close(pPort);
}

void SPI_IO_SECTOR_ERASE(const spi_port_t pPort, const uint32_t pAdr)
{
	SPI_IO_Clock_Slow(); // SPI_IO_SECTOR_ERASE()
	
	SPI_IO_Open(pPort);
	
	SPI_IO_Raw_Write_Byte(JEDEC_SPI_SECTOR_ERASE);   
	
	SPI_IO_Raw_Write_Byte(pAdr>>16);
	SPI_IO_Raw_Write_Byte(pAdr>>8);
	SPI_IO_Raw_Write_Byte(pAdr>>0);	
	
	SPI_IO_Close(pPort);
	
	SPI_IO_Clock_Fast(); // SPI_IO_SECTOR_ERASE()
}

void SPI_IO_PAGE_WRITE(const spi_port_t pPort,const byte pCommand, const uint32_t pAdr,const byte* pBuffer,const uint32_t pSize=256)
{
	SPI_IO_Clock_Slow(); // SPI_IO_PAGE_WRITE()

	SPI_IO_Open(pPort);
	
	SPI_IO_Raw_Write_Byte(pCommand);
	
	SPI_IO_Raw_Write_Byte(pAdr>>16);
	SPI_IO_Raw_Write_Byte(pAdr>>8);
	SPI_IO_Raw_Write_Byte(pAdr>>0);	
	
	for (uint32_t x = 0; x < pSize; x++)
	{
		SPI_IO_Raw_Write_Byte(pBuffer[x]);
	}
	
	SPI_IO_Close(pPort);
	
	SPI_IO_Clock_Fast(); // SPI_IO_PAGE_WRITE()
}

void SPI_IO_WRITE_ENABLE(spi_port_t pPort)
{
	SPI_IO_Clock_Slow(); // SPI_IO_WRITE_ENABLE()
	
	SPI_IO_SIMPLE_COMMAND(pPort,JEDEC_SPI_WRITE_ENABLE);

	SPI_IO_Clock_Fast(); // SPI_IO_WRITE_ENABLE()
}

void SPI_IO_WRITE_DISABLE(spi_port_t pPort)
{
	SPI_IO_Clock_Slow(); // SPI_IO_WRITE_DISABLE()
	
	SPI_IO_SIMPLE_COMMAND(pPort,JEDEC_SPI_WRITE_DISABLE);

	SPI_IO_Clock_Fast(); // SPI_IO_WRITE_DISABLE()
}

void SPI_IO_CHIP_ERASE_ALL_SECTORS(const char* pTestName,const char* pTestSeq,const spi_port_t pPort)
{
	byte status_temp = 0;
	volatile uint32_t sector_address;
		
	for (uint32_t sector = 0; sector < 16; sector++)
	{
		sector_address = sector * 0x10000;
		
		SPI_IO_WRITE_ENABLE(pPort);
		
		SPI_IO_SECTOR_ERASE(pPort,sector_address);
		
		uint32_t sector_start = GetTickCount();
		uint32_t sector_stop = 0;
		while (sector_stop==0)
		{			
			status_temp = 0xFF;
			SPI_IO_READ_STATUS_REGISTER(pPort,&status_temp);
			if ((status_temp&0x01) == 0)
			{
				sector_stop = GetTickCount();
			}			
		}
		
		char temp_message[200];
		sprintf(temp_message,"sector_erase %02X %06X in %dms",sector,sector_address,sector_stop-sector_start);
		cout << pTestName << "," << pTestSeq << "," << temp_message << endl;
	}
		
	SPI_IO_WRITE_DISABLE(pPort);
}

void SPI_IO_CHIP_WRITE_ALL_PAGES(const spi_port_t pPort,const char* pTestName,const char* pTestSeq,const byte pCommand,const byte* pBuffer,const uint32_t pSize,uint64_t* pChecksum=0)
{
	SPI_IO_WRITE_ENABLE(pPort);
	
	byte status_temp = 0;
	volatile uint32_t page_address;
	volatile uint32_t page_max = pSize/256;
	
	cout << pTestName << "," << pTestSeq << "," << "Write Start" << endl;	
	
	if (pChecksum)
	{
		*pChecksum = Get_Checksum(pBuffer, pSize);
	}	
	
	int mod_count = 0;
		
	for (uint32_t page = 0; page < page_max; page++)
	{
		page_address = page * 0x100;

		while (1)
		{
			status_temp = 0x00;
			SPI_IO_WRITE_ENABLE(pPort);
			SPI_IO_READ_STATUS_REGISTER(pPort,&status_temp);
			if ((status_temp&0x02) == 0x02)
			{
				break;
			}
			usleep(100);
		}
		
		SPI_IO_PAGE_WRITE(pPort,pCommand,page_address,&pBuffer[page_address],256);
		
		cout << ".";		
		mod_count++;
		
		if (mod_count >= 128)
		{
			cout << endl;			
			mod_count = 0;
		}
				
		uint32_t page_wr_start = GetTickCount();
		uint32_t page_wr_stop = 0;
		
		while (page_wr_stop==0)
		{			
			status_temp = 0xFF;
			SPI_IO_READ_STATUS_REGISTER(pPort,&status_temp);
			if ((status_temp&0x01) == 0)
			{
				page_wr_stop = GetTickCount();
			}
			usleep(100);
		}
		
		//char temp_message[200];
		//sprintf(temp_message,"page_write @ %06X in %dms",page_address,page_wr_stop-page_wr_start);
		//cout << temp_message << endl;
	}

	cout << pTestName << "," << pTestSeq << "," << "Write Done" << endl;
	
	SPI_IO_WRITE_DISABLE(pPort);
}
		
void SPI_IO_CHIP_Fast_Copy_To_File(const spi_port_t pPort,const char* pTestName,const char* pTestSeq,const char *filename,const uint32_t pStartAddress,const uint32_t pSize,uint64_t* pChecksum=0)
{
	byte *Temp_Buffer = (byte*) malloc(pSize);
	SPI_IO_Command_with_Address_Send(pPort,JEDEC_SPI_READ_DATA_BYTES, pStartAddress,&Temp_Buffer[0], pSize);
		
	if (pChecksum)
	{
		*pChecksum = Get_Checksum(Temp_Buffer, pSize);
	}		
	
	{
		FILE *ptr = fopen(filename,"wb");  // r for read, b for binary
		fwrite(Temp_Buffer,pSize,1,ptr); // read 10 bytes to our buffer
		fclose(ptr);
		cout << pTestName << "," << pTestSeq << "," << "File written : " << filename << endl;
	}
	
	free(Temp_Buffer);
}

void SPI_IO_CHIP_Copy_Buffer_To_File_Tripple_Check(const spi_port_t pPort,const char* pTestName,const char* pTestSeq,const char *filename,const uint32_t pStartAddress,const uint32_t pSize,uint64_t* pChecksum=0)
{
	byte *test1 = (byte*) malloc(pSize);
	byte *test2 = (byte*) malloc(pSize);
	byte *test3 = (byte*) malloc(pSize);
	
	char message[200];
	
	SPI_IO_Command_with_Address_Send(pPort,JEDEC_SPI_READ_DATA_BYTES, pStartAddress,&test1[0], pSize);
	uint64_t test1_checksum = Get_Checksum(test1, pSize);
	sprintf(message, ",test1_checksum:%016LX", test1_checksum);	
	cout << pTestName << "," << pTestSeq << ","<<  "chip tripple reading,first read done" << message << endl;
	
	SPI_IO_Command_with_Address_Send(pPort,JEDEC_SPI_READ_DATA_BYTES, pStartAddress,&test2[0], pSize);
	uint64_t test2_checksum = Get_Checksum(test2, pSize);
	sprintf(message, ",test2_checksum:%016LX", test2_checksum);	
	cout << pTestName << "," << pTestSeq << ","<< "chip tripple reading,second read done" << message << endl;

	SPI_IO_Command_with_Address_Send(pPort,JEDEC_SPI_READ_DATA_BYTES, pStartAddress,&test3[0], pSize);
	uint64_t test3_checksum = Get_Checksum(test3, pSize);
	sprintf(message, ",test3_checksum:%016LX", test3_checksum);
	cout << pTestName << "," << pTestSeq << ","<< "chip tripple reading,third read done" << message << endl;

	int res1 = memcmp(test1, test2, pSize);
	int res2 = memcmp(test1, test3, pSize);
	
	if (res1 == 0 && res2 == 0)
	{
		FILE *ptr = fopen(filename, "wb");  // r for read, b for binary
		fwrite(test1, pSize, 1, ptr); // read 10 bytes to our buffer
		fclose(ptr);
		cout << pTestName << "," << pTestSeq << "," << "chip tripple reading,File written" << filename << endl;
		
		if (pChecksum)
		{

			*pChecksum = Get_Checksum(test1, pSize);
		}		
	}
	else
	{
		cout << pTestName << "," << pTestSeq << "," << "chip tripple reading,Verify Error!" << endl;		
	}
	
	free(test1);
	free(test2);
	free(test3);
}

void Program_Chip_Cycle(const spi_port_t pPort, const char* pTestName, const char* pTestSeq, const char* program_data_filename, const char* file_root, const uint32_t pChipSizeBytes = 0x100000)
{
	SPI_IO_CHIP_ERASE_ALL_SECTORS(pTestName,pTestSeq,pPort);
	
	byte* write_buffer = (byte*) malloc(pChipSizeBytes);
	char filename_temp[300];
	sprintf(filename_temp, "%s/%s", file_root, program_data_filename);
			
	FILE *ptr = fopen(filename_temp, "rb");  // r for read, b for binary
	fread(write_buffer, pChipSizeBytes, 1, ptr); // read 10 bytes to our buffer
	fclose(ptr);
			
	char temp_message[200];
	uint64_t checksum_prog_file = Get_Checksum(write_buffer, pChipSizeBytes);
	sprintf(temp_message, "checksum_prog_file:0x%016LX", checksum_prog_file);
			
	cout << pTestName << "," << pTestSeq << "," << "Programming with :" << filename_temp << "," << temp_message << endl;					

	SPI_IO_CHIP_WRITE_ALL_PAGES(pPort, pTestName, pTestSeq, JEDEC_SPI_WRITE_PAGES, write_buffer, pChipSizeBytes,nullptr);
	
	uint64_t checksum_prog_dump = 0;
	SPI_IO_CHIP_Copy_Buffer_To_File_Tripple_Check(pPort,pTestName,pTestSeq, filename_temp, 0, pChipSizeBytes,&checksum_prog_dump);		
}

void chip_full_test(const spi_port_t pPort,const char* pTestName,const char* pTestSeq,const char* program_data_filename,const char* file_root,const uint32_t pChipSizeBytes=0x100000)
{
	uint64_t checksum_inital = 0; /// 0xfffffffff00fffff = erased
	uint64_t checksum_blank = 0; 
	uint64_t checksum_post = 0;
	uint64_t checksum_prog_file = 0;
	uint64_t checksum_prog_dump = 0;

	
	cout << endl << pTestName << "," << pTestSeq << ",chip_full_test(),start" << endl;
	
	if(1)
	{
		char filename_temp[300];
		sprintf(filename_temp,"%s/%s_%s_inital_read.bin", file_root, pTestName,pTestSeq);
		SPI_IO_CHIP_Fast_Copy_To_File(pPort,pTestName,pTestSeq, filename_temp, 0, pChipSizeBytes,&checksum_inital);

		char temp_message[200];
		sprintf(temp_message,"checksum_inital:0x%016LX",checksum_inital);
		cout << pTestName << "," << pTestSeq << "," << temp_message << endl;
	}
	
	if (1)
	{
		SPI_IO_CHIP_ERASE_ALL_SECTORS(pTestName,pTestSeq,pPort);
	}
	
	if(1)
	{
		char filename_temp[300];
		sprintf(filename_temp,"%s/%s_%s_blank_check.bin", file_root, pTestName,pTestSeq);

		cout << pTestName << "," << pTestSeq << "," << "Fast Copy to :" << filename_temp << endl;
		
		SPI_IO_CHIP_Fast_Copy_To_File(pPort,pTestName,pTestSeq, filename_temp, 0, pChipSizeBytes,&checksum_blank);

		char temp_message[200];
		sprintf(temp_message,"checksum_blank:0x%016LX",checksum_blank);
		cout << pTestName << "," << pTestSeq << "," << temp_message << endl;
	}	
	
	{
		byte* write_buffer = (byte*) malloc(pChipSizeBytes);
		{
			char filename_temp[300];
			sprintf(filename_temp,"%s/%s", file_root, program_data_filename);
			
			FILE *ptr = fopen(filename_temp,"rb");  // r for read, b for binary
			fread(write_buffer,pChipSizeBytes,1,ptr); // read 10 bytes to our buffer
			fclose(ptr);
			
			char temp_message[200];
			checksum_prog_file = Get_Checksum(write_buffer, pChipSizeBytes);
			sprintf(temp_message,"checksum_prog_file:0x%016LX",checksum_prog_file);
			
			cout << pTestName << "," << pTestSeq << "," << "Programming with :" << filename_temp << "," << temp_message << endl;					
		}

		SPI_IO_CHIP_WRITE_ALL_PAGES(pPort,pTestName,pTestSeq,JEDEC_SPI_WRITE_PAGES, write_buffer, pChipSizeBytes,&checksum_prog_file);		
		
		free(write_buffer);
	}

	if(1)
	{
		char filename_temp[300];
		sprintf(filename_temp,"%s/%s_%s_prog_dump.bin", file_root, pTestName,pTestSeq);
		
		SPI_IO_CHIP_Copy_Buffer_To_File_Tripple_Check(pPort,pTestName,pTestSeq, filename_temp, 0, pChipSizeBytes,&checksum_prog_dump);

		char temp_message[200];
		sprintf(temp_message,"checksum_prog_dump:0x%016LX",checksum_prog_dump);
		cout << pTestName << "," << pTestSeq << "," << temp_message << endl;		
	}
	
	cout << pTestName << "," << pTestSeq << "chip_full_test(),stop" << endl << endl;
}

int main(int argc, char *argv[])
{
	int setup_res = wiringPiSetupGpio();
	if (setup_res == 0)
	{
		cout << "Setup_IO() Passed" << endl;
		SPI_IO_Enable_Bus_Drivers();
	}
	else
	{
		cout << "Setup_IO() Failed" << endl;
		SPI_IO_Disable_Bus_Drivers();
		return -1;
	}
	
	if(0)
	{
		byte write_buffer[0x100000];
		memset(write_buffer, 0xFF, sizeof(write_buffer));
		FILE *ptr = fopen("/home/pi/Desktop/blank.bin","wb");  // r for read, b for binary
		fwrite(write_buffer,sizeof(write_buffer),1,ptr); // read 10 bytes to our buffer
		fclose(ptr);			
	}
	
	if(0)
	{
		byte write_buffer[0x100000];
		for (int x = 0; x < sizeof(write_buffer); x++)
		{
			write_buffer[x] = rand();
		}
		FILE *ptr = fopen("/home/pi/Desktop/rand.bin","wb");  // r for read, b for binary
		fwrite(write_buffer,sizeof(write_buffer),1,ptr); // read 10 bytes to our buffer
		fclose(ptr);			
	}	
	
	{
		byte id_jedec_ce0[3] = { };
		byte id_jedec_ce1[3] = { };
		char temp_message[200];
	
		SPI_READ_JEDEC(spi_port_t::CE0, id_jedec_ce0);
		SPI_READ_JEDEC(spi_port_t::CE1, id_jedec_ce1);
			
		sprintf(temp_message,"id_jedec_ce0:%02X%02X%02X", id_jedec_ce0[0], id_jedec_ce0[1], id_jedec_ce0[2]);
		cout << temp_message << endl;	
		sprintf(temp_message,"id_jedec_ce1:%02X%02X%02X", id_jedec_ce1[0], id_jedec_ce1[1], id_jedec_ce1[2]);
		cout << temp_message << endl;		
	}
	
	char file_root[] = "/home/pi/Desktop/bin_files";
	char blank_filename[] = { "blank.bin" };
	char test1_filename[] = { "test1.bin" };
	char rand_filename[] = { "rand.bin" };
	
	chip_full_test(spi_port_t::CE0, "M45PE80", "Rand", rand_filename, file_root);
	chip_full_test(spi_port_t::CE0, "M45PE80", "Blank", blank_filename, file_root);
	chip_full_test(spi_port_t::CE0, "M45PE80", "test1", test1_filename, file_root);

	chip_full_test(spi_port_t::CE1, "AT25SF081B", "Rand", rand_filename, file_root);
	chip_full_test(spi_port_t::CE1, "AT25SF081B", "Blank", blank_filename, file_root);
	chip_full_test(spi_port_t::CE1, "AT25SF081B", "test1", test1_filename, file_root);
	
	//Program_Chip_Cycle(spi_port_t::CE0, "M45PE80", "prog", test1_filename, file_root);
	//Program_Chip_Cycle(spi_port_t::CE1, "AT25SF081B", "prog", test1_filename, file_root);

	SPI_IO_Disable_Bus_Drivers();
	
	cout << "Disabled Drivers" << endl;
	return 0;
}