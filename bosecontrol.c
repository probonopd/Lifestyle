/*
    Control BOSE using a 30 cm wire as antenna on GPIO_4 pin 7

    To compile:
    gcc -Wall -O4 -o bosecontrol bosecontrol.c -std=gnu99 -lm

    This file contains parts of code from Pifm.c by Oliver Mattos and Oskar Weigl

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <math.h>
#include <fcntl.h>
#include <assert.h>
#include <malloc.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>
#include <stdint.h>
#include <time.h>
#include <getopt.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int  mem_fd;
char *gpio_mem, *gpio_map;
char *spi0_mem, *spi0_map;
double ppm_correction;
double pllo_frequency;

//#define PLL0_FREQUENCY 250000000.0
#define PLL0_FREQUENCY 500000000.0

#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

// I/O access
volatile unsigned *gpio;
volatile unsigned *allof7e;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
#define GPIO_GET *(gpio+13)  // sets   bits which are 1 ignores bits which are 0

#define ACCESS(base) *(volatile int*)((int)allof7e+base-0x7e000000)
#define SETBIT(base, bit) ACCESS(base) |= 1<<bit
#define CLRBIT(base, bit) ACCESS(base) &= ~(1<<bit)

#define CM_GP0CTL (0x7e101070)
#define GPFSEL0 (0x7E200000)
#define CM_GP0DIV (0x7e101074)
#define CLKBASE (0x7E101000)
#define DMABASE (0x7E007000)

#define START_IN	8
#define TRIGGER_OUT	17

struct GPCTL
{
char SRC         : 4;
char ENAB        : 1;
char KILL        : 1;
char             : 1;
char BUSY        : 1;
char FLIP        : 1;
char MASH        : 2;
unsigned int     : 13;
char PASSWD      : 8;
};

int verbose;

// Set up a memory regions to access GPIO
void setup_io()
{
/* open /dev/mem */
if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0)
	{
	printf("can't open /dev/mem, need to run as root\n");
	exit(-1);
   }

/* mmap GPIO */
gpio_map = mmap(
     NULL,             // Any adddress in our space will do
     BLOCK_SIZE,       // Map length
     PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
     MAP_SHARED,       // Shared with other processes
     mem_fd,           // File to map
     GPIO_BASE         // Offset to GPIO peripheral
   );

close(mem_fd); // No need to keep mem_fd open after mmap

if(gpio_map == MAP_FAILED)
	{
	printf("mmap error %d\n", (int)gpio_map); // errno also set!
	exit(-1);
	}

// Always use volatile pointer!
gpio = (volatile unsigned *)gpio_map;

} /* end function setup_io */

void set_gpio_directions()
{
// TRIGGER_OUT to output
INP_GPIO(TRIGGER_OUT);
OUT_GPIO(TRIGGER_OUT);
} /* end function set_gpio_directions */

void getRealMemPage(void** vAddr, void** pAddr)
{
void* a = valloc(4096);
((int*)a)[0] = 1;  // use page to force allocation
mlock(a, 4096);  // lock into ram
*vAddr = a;  // yay - we know the virtual address
unsigned long long frameinfo;
int fp = open("/proc/self/pagemap", 'r');
lseek(fp, ((int)a)/4096*8, SEEK_SET);
read(fp, &frameinfo, sizeof(frameinfo));
    
*pAddr = (void*)((int)(frameinfo*4096));
}

void freeRealMemPage(void* vAddr)
{
munlock(vAddr, 4096);  // unlock ram.
free(vAddr);
}

void start_rf_output(int source)
{
/* open /dev/mem */
if( (mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0)
	{
	fprintf(stderr, "freq_pi: can't open /dev/mem, aborting.\n");
	exit (1);
	}
    
allof7e = (unsigned *)mmap( NULL, 0x01000000,  /*len */ PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, 0x20000000  /* base */ );

if( (int)allof7e == -1) exit(1);

SETBIT(GPFSEL0 , 14);
CLRBIT(GPFSEL0 , 13);
CLRBIT(GPFSEL0 , 12);
 
struct GPCTL setupword = {source, 1, 0, 0, 0, 1,0x5a};

ACCESS(CM_GP0CTL) = *((int*)&setupword);
}


void modulate(int m)
{
ACCESS(CM_GP0DIV) = (0x5a << 24) + 0x4d72 + m;
}

struct CB
{
volatile unsigned int TI;
volatile unsigned int SOURCE_AD;
volatile unsigned int DEST_AD;
volatile unsigned int TXFR_LEN;
volatile unsigned int STRIDE;
volatile unsigned int NEXTCONBK;
volatile unsigned int RES1;
volatile unsigned int RES2;
};

struct DMAregs
{
volatile unsigned int CS;
volatile unsigned int CONBLK_AD;
volatile unsigned int TI;
volatile unsigned int SOURCE_AD;
volatile unsigned int DEST_AD;
volatile unsigned int TXFR_LEN;
volatile unsigned int STRIDE;
volatile unsigned int NEXTCONBK;
volatile unsigned int DEBUG;
};

struct PageInfo
{
void* p;  // physical address
void* v;   // virtual address
};


struct PageInfo constPage;   
struct PageInfo instrPage;
struct PageInfo instrs[1024];

int set_frequency(uint32_t frequency)
{
if(verbose)
	{
	fprintf(stderr, "set_frequency(): arg frequency=%d\n", frequency);
	}

uint32_t ua;
uint16_t divi; // integer part divider [23:12]		12 bits wide, max 4095
uint16_t divf; // fractional part divider [11:0]	12 bits wide, max 4095

/* set frequeny */

/* calculate divider */
double da; 

da = pllo_frequency;
da += pllo_frequency * (ppm_correction / 1000000.0);
da /= (double) frequency;

divi = (int) da;
divf = 4096.0 * (da - (double)divi);

if(verbose)
	{
	fprintf(stderr, "ppm_correction=%f frequency=%d da=%f divi=%d divf=%d\n", ppm_correction, frequency, da, divi, divf);
	}

ua = (0x5a << 24) + (divi << 12) + divf;

ACCESS(CM_GP0DIV) = ua;

if(verbose)
	{
	fprintf(stderr, "set_frequency: frequency set to %d\n", frequency);
	}

return 1;
} /* end function set_frequency */


void print_usage()
{

fprintf(stderr,\
"\nPanteltje freq_pi-%s\n\
Usage:\nbosecontrol [-f frequency] [-h] [-r] [-v] [-y ppm_correction]\n\
-f int        frequency to output on GPIO_4 pin 7, on my revision 2 board from 130 kHz to 250 MHz,\n\
               phase noise is caused by divf (fractional part of divider) not being zero, use -v to show divf.\n\
-h            help (this help).\n\
-v            verbose.\n\
-y float      frequency correction in parts per million (ppm), positive or negative, for calibration, default 0.\n\
\n");


} /* end function print_usage */


int main(int argc, char **argv)
{
int a;
uint32_t frequency = 0; // -Wall

/* defaults */
verbose = 0;
pllo_frequency = PLL0_FREQUENCY;
ppm_correction = 0.0;


/* proces any command line arguments */
while(1)
	{
	a = getopt(argc, argv, "f:h:vy:");
	if(a == -1) break;

	switch(a)
		{
		case 'f':	// frequency
			a = atoi(optarg);
			frequency = a;
			break;
		case 'h': // help
			print_usage();
			exit(1);
			break;
			break;
		case 'v': // verbose
			verbose = 1 - verbose;
			break;
		case 'y': // ppm correction
			ppm_correction = atof(optarg);
			break;
        case -1:
        	break;
		case '?':
			if (isprint(optopt) )
 				{
 				fprintf(stderr, "send_iq: unknown option `-%c'.\n", optopt);
 				}
			else
				{
				fprintf(stderr, "freq_pi: unknown option character `\\x%x'.\n", optopt);
				}
			print_usage();

			exit(1);
			break;			
		default:
			print_usage();

			exit(1);
			break;
		}/* end switch a */
	}/* end while getopt() */


setup_io();

set_gpio_directions(); // start pin only 

int clock_source;

/* init hardware */


clock_source = 6; /* this GPIO_4 pin 7 allows only the PLLD clock as source, the other clock GPIO lines are not on a pin in revision 2 board, so we have to work with PLLD, and that seems to be 500 MHz */
start_rf_output(clock_source);

////////////////////////////////////
// BOSE specific code starts here //
////////////////////////////////////

// 27.145 MHz
frequency = 27145000;

// Since we can't switch off the sender fast enough, we set it to another frequency instead, simulating "off"
// 49.830 MHz
int other_frequency = 49830000;

const int raw_NEC2_186_85_1[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-564,564,-564,564,-564,564,-564,564,-564,564,-564,564,-1692,564,-1692,564,-1692,564,-1692,564,-1692,564,-1692,564,-1692,564,-38628 };

const int raw_NEC2_186_85_2[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-1692,564,-564,564,-564,564,-564,564,-564,564,-564,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-1692,564,-1692,564,-1692,564,-38628 };

const int raw_NEC2_186_85_3[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-564,564,-564,564,-564,564,-564,564,-564,564,-564,564,-564,564,-1692,564,-1692,564,-1692,564,-1692,564,-1692,564,-1692,564,-38628 };

const int raw_NEC2_186_85_6[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-1692,564,-1692,564,-564,564,-564,564,-564,564,-564,564,-564,564,-1692,564,-564,564,-564,564,-1692,564,-1692,564,-1692,564,-1692,564,-1692,564,-38628 };

const int raw_NEC2_186_85_10[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-564,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-1692,564,-38628 };

const int raw_NEC2_186_85_11[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-564,564,-564,564,-564,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-1692,564,-38628 };

const int raw_NEC2_186_85_13[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-564,564,-564,564,-564,564,-564,564,-1692,564,-564,564,-564,564,-1692,564,-1692,564,-1692,564,-1692,564,-38628 };

const int raw_NEC2_186_85_14[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-564,564,-564,564,-564,564,-1692,564,-564,564,-564,564,-564,564,-1692,564,-1692,564,-1692,564,-1692,564,-38628 };

const int raw_NEC2_186_85_15[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-1692,564,-564,564,-564,564,-564,564,-564,564,-564,564,-564,564,-564,564,-564,564,-1692,564,-1692,564,-1692,564,-1692,564,-38628 };

const int raw_NEC2_186_85_24[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-564,564,-564,564,-1692,564,-1692,564,-564,564,-564,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-564,564,-1692,564,-1692,564,-1692,564,-38628 };

const int raw_NEC2_186_85_25[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-1692,564,-1692,564,-564,564,-564,564,-564,564,-564,564,-1692,564,-1692,564,-564,564,-564,564,-1692,564,-1692,564,-1692,564,-38628 };

const int raw_NEC2_186_85_26[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-564,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-1692,564,-1692,564,-1692,564,-38628 };

const int raw_NEC2_186_85_75[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-1692,564,-564,564,-564,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-38628 };

const int raw_NEC2_186_85_76[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-564,564,-1692,564,-1692,564,-564,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-38628 };

const int raw_NEC2_186_85_223[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-564,564,-564,564,-564,564,-564,564,-1692,564,-564,564,-564,564,-38628 };

const int raw_NEC2_186_85_78[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-38628 };

const int raw_NEC2_186_85_79[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-1692,564,-564,564,-564,564,-1692,564,-564,564,-564,564,-564,564,-564,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-38628 };

const int raw_NEC2_186_85_82[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-1692,564,-564,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-38628 };

const int raw_NEC2_186_85_83[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-38628 };

const int raw_NEC2_186_85_85[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-38628 };

const int raw_NEC2_186_85_86[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-38628 };

const int raw_NEC2_186_85_92[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-564,564,-564,564,-1692,564,-564,564,-1692,564,-38628 };

const int raw_NEC2_186_85_93[] = { 9024,-4512,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-564,564,-1692,564,-1692,564,-1692,564,-564,564,-1692,564,-564,564,-564,564,-1692,564,-564,564,-564,564,-564,564,-1692,564,-564,564,-1692,564,-38628 };


 int i, *p;
    
for(i=0; i<(&raw_NEC2_186_85_1)[1]-raw_NEC2_186_85_1; i++){
	if(raw_NEC2_186_85_1[i] > 0) {
		set_frequency(frequency);
	} else {
		set_frequency(other_frequency);
	}
	usleep(abs(raw_NEC2_186_85_1[i])-100); 
	// The "-100" was found out by experimentation, because the original remote sounded "higher" on gqrx
	// TODO: Measure timings
}
    


/* RF off */
clock_source = 0; // ground
start_rf_output(clock_source);

exit(0);
} /* end function  main */
