////////////////////////////////////////////////////////////////////////////////////////////
// Trying to control BOSE using a 30 cm wire as antenna on the TX pin on a ESP8266 module
// Adopted from https://github.com/jokrug/espfuchs/blob/master/user/oscillator.c
// Can we get this to compile on esp8266/Arduino? *** NOT WORKING YET, help welcome ***
////////////////////////////////////////////////////////////////////////////////////////////

#ifdef ESP8266
extern "C" {
#include "user_interface.h"
}
#endif

int setBitPattern();
void printBitField();
void setBit(int bitNr, bool val);
int  getBit(int bitNr);
void setFrequency(enum Frequencies freq);
void frequencyEnumToString(enum Frequencies freq, char* buf);

#include "slc_register.h"
#include <c_types.h>
#include "pin_mux_register.h"
#include "dmastuff.h"
#include "io.h"

#define I2SDMABUFLEN (100)    //Length of one buffer, in 32-bit words.

//Bit clock @ 40MHz = 25ns
//WS_I2S_DIV - if 1 will actually be 2.  Can't be less than 2.
//    CLK_I2S = 80MHz / I2S_CLKM_DIV_NUM
//    BCLK = CLK_I2S / I2S_BCK_DIV_NUM
//    WS = BCLK/ 2 / (16 + I2S_BITS_MOD)
//    Note that I2S_CLKM_DIV_NUM must be >5 for I2S data
//    I2S_CLKM_DIV_NUM - 5-127
//    I2S_BCK_DIV_NUM - 2-127

// https://github.com/jokrug/espfuchs/issues/1
// Among a lot of harmonics, you will get 5.714+3.333=9.047MHz
// The third harmonic is (almost) the desired 27.141MHz.
// The amplitude of this frequency however, will be much lower than the 9.047 and the 5.714. Maybe it is still strong enough. It's worth a test.
ws_i2s_bck = 1; // defines I2S-clock of 80MHz
ws_i2s_div = 2; // defines I2S-clock of 80MHz
freq1Bits = 14; // 80 / 14 = 5.714
freq2Bits = 24; // 80 / 24 = 3.333

static struct sdio_queue i2sBufDesc; //I2S DMA buffer descriptor

static uint32_t i2sBD[I2SDMABUFLEN];
static int bitFieldSize = 0;
static int dataLen = 0;
static int kgv = 0;

int ICACHE_FLASH_ATTR ggt(int m, int n)
{
  if (n == 0)
    return m;
  else
    return ggt(n, m % n);
}

int ICACHE_FLASH_ATTR calcKgv(int m, int n)
{
  int o = ggt(m, n);
  int p = (m * n) / o;
  return p;
}

int ICACHE_FLASH_ATTR setBitPattern()
{
  os_memset( i2sBD, 0x00, sizeof(i2sBD));
  int bitCount = 0;

  kgv = calcKgv( freq1Bits, 32 );

  if (freq2Bits > 0)
    kgv = calcKgv( freq2Bits, kgv );

  for (int i1 = 0; i1 < kgv; i1++ )
  {
    bool bit1 = (bitCount % freq1Bits) > freq1Bits / 2;
    bool bit2 = true;
    if (freq2Bits > 0)
      bit2 = (bitCount % freq2Bits) > freq2Bits / 2;

    setBit(bitCount, (bit1 && bit2) );
    bitCount++;
  }
  return ( bitCount );
}

void ICACHE_FLASH_ATTR setBit( int bitNr, bool val)
{
  if ( val )
    i2sBD[bitNr >> 5] |= 0x80000000 >> (bitNr % 32);
  else
    i2sBD[bitNr >> 5] &= ~(0x80000000 >> (bitNr % 32));
}

int ICACHE_FLASH_ATTR getBit( int bitNr)
{
  return ( (i2sBD[bitNr >> 5] >> (bitNr % 32)) & 1);
}

//Initialize I2S subsystem for DMA circular buffer use
void ICACHE_FLASH_ATTR initI2S()
{
  bitFieldSize = setBitPattern();
  dataLen = (bitFieldSize >> 3); // dataLen (bytes) = bitFieldSize / 8

  //Initialize DMA buffer descriptor.
  i2sBufDesc.owner = 1;
  i2sBufDesc.eof = 1;
  i2sBufDesc.sub_sof = 0;
  i2sBufDesc.datalen = dataLen;
  i2sBufDesc.blocksize = dataLen;
  i2sBufDesc.buf_ptr = (uint32_t)&i2sBD;
  i2sBufDesc.unused = 0;
  i2sBufDesc.next_link_ptr = (int)(&i2sBufDesc);

  //Reset DMA
  SET_PERI_REG_MASK(SLC_CONF0, SLC_RXLINK_RST | SLC_TXLINK_RST);
  CLEAR_PERI_REG_MASK(SLC_CONF0, SLC_RXLINK_RST | SLC_TXLINK_RST);

  //Clear DMA int flags
  SET_PERI_REG_MASK(SLC_INT_CLR,  0xffffffff);
  CLEAR_PERI_REG_MASK(SLC_INT_CLR,  0xffffffff);

  //Enable and configure DMA
  CLEAR_PERI_REG_MASK(SLC_CONF0, (SLC_MODE << SLC_MODE_S));
  SET_PERI_REG_MASK(SLC_CONF0, (1 << SLC_MODE_S));
  SET_PERI_REG_MASK(SLC_RX_DSCR_CONF, SLC_INFOR_NO_REPLACE | SLC_TOKEN_NO_REPLACE);
  CLEAR_PERI_REG_MASK(SLC_RX_DSCR_CONF, SLC_RX_FILL_EN | SLC_RX_EOF_MODE | SLC_RX_FILL_MODE);

  //Feed dma the 1st buffer desc addr
  //To send data to the I2S subsystem, counter-intuitively we use the RXLINK part, not the TXLINK as you might
  //expect. The TXLINK part still needs a valid DMA descriptor, even if it's unused: the DMA engine will throw
  //an error at us otherwise. Just feed it any random descriptor.
  CLEAR_PERI_REG_MASK(SLC_TX_LINK, SLC_TXLINK_DESCADDR_MASK);
  SET_PERI_REG_MASK(SLC_TX_LINK, ((uint32)&i2sBufDesc) & SLC_TXLINK_DESCADDR_MASK); //any random desc is OK, we don't use TX but it needs something valid
  CLEAR_PERI_REG_MASK(SLC_RX_LINK, SLC_RXLINK_DESCADDR_MASK);
  SET_PERI_REG_MASK(SLC_RX_LINK, ((uint32)&i2sBufDesc) & SLC_RXLINK_DESCADDR_MASK);

  //clear any interrupt flags that are set
  WRITE_PERI_REG(SLC_INT_CLR, 0xffffffff);

  //Start transmission
  SET_PERI_REG_MASK(SLC_TX_LINK, SLC_TXLINK_START);
  SET_PERI_REG_MASK(SLC_RX_LINK, SLC_RXLINK_START);

  //----

  //Init pins to i2s functions
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_I2SO_DATA);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_I2SO_WS);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_I2SO_BCK);

  //Enable clock to i2s subsystem
  i2c_writeReg_Mask_def(i2c_bbpll, i2c_bbpll_en_audio_clock_out, 1);

  //Reset I2S subsystem
  CLEAR_PERI_REG_MASK(I2SCONF, I2S_I2S_RESET_MASK);
  SET_PERI_REG_MASK(I2SCONF, I2S_I2S_RESET_MASK);
  CLEAR_PERI_REG_MASK(I2SCONF, I2S_I2S_RESET_MASK);

  //Select 16bits per channel (FIFO_MOD=0), no DMA access (FIFO only)
  CLEAR_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN | (I2S_I2S_RX_FIFO_MOD << I2S_I2S_RX_FIFO_MOD_S) | (I2S_I2S_TX_FIFO_MOD << I2S_I2S_TX_FIFO_MOD_S));
  //Enable DMA in i2s subsystem
  SET_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN);

  //tx/rx binaureal
  CLEAR_PERI_REG_MASK(I2SCONF_CHAN, (I2S_TX_CHAN_MOD << I2S_TX_CHAN_MOD_S) | (I2S_RX_CHAN_MOD << I2S_RX_CHAN_MOD_S));

  //Clear int
  SET_PERI_REG_MASK(I2SINT_CLR,   I2S_I2S_TX_REMPTY_INT_CLR | I2S_I2S_TX_WFULL_INT_CLR |
                    I2S_I2S_RX_WFULL_INT_CLR | I2S_I2S_PUT_DATA_INT_CLR | I2S_I2S_TAKE_DATA_INT_CLR);
  CLEAR_PERI_REG_MASK(I2SINT_CLR, I2S_I2S_TX_REMPTY_INT_CLR | I2S_I2S_TX_WFULL_INT_CLR |
                      I2S_I2S_RX_WFULL_INT_CLR | I2S_I2S_PUT_DATA_INT_CLR | I2S_I2S_TAKE_DATA_INT_CLR);

//trans master&rece slave,MSB shift,right_first,msb right
    CLEAR_PERI_REG_MASK(I2SCONF, I2S_TRANS_SLAVE_MOD|
                                            (I2S_BITS_MOD<<I2S_BITS_MOD_S)|
                                            (I2S_BCK_DIV_NUM <<I2S_BCK_DIV_NUM_S)|
                                            (I2S_CLKM_DIV_NUM<<I2S_CLKM_DIV_NUM_S));

    SET_PERI_REG_MASK(I2SCONF, I2S_RIGHT_FIRST|I2S_MSB_RIGHT|I2S_RECE_SLAVE_MOD|
                                            I2S_RECE_MSB_SHIFT|I2S_TRANS_MSB_SHIFT|
                                            ((ws_i2s_bck&I2S_BCK_DIV_NUM )<<I2S_BCK_DIV_NUM_S)|
                                            ((ws_i2s_div&I2S_CLKM_DIV_NUM)<<I2S_CLKM_DIV_NUM_S));

    //No idea if ints are needed...
    //clear int
    SET_PERI_REG_MASK(I2SINT_CLR,   I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|
                    I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);
    CLEAR_PERI_REG_MASK(I2SINT_CLR,   I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|
                    I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);
    //enable int
    //SET_PERI_REG_MASK(I2SINT_ENA,   I2S_I2S_TX_REMPTY_INT_ENA|I2S_I2S_TX_WFULL_INT_ENA|
    //I2S_I2S_RX_REMPTY_INT_ENA|I2S_I2S_TX_PUT_DATA_INT_ENA|I2S_I2S_RX_TAKE_DATA_INT_ENA);

    //Start transmission
    SET_PERI_REG_MASK(I2SCONF,I2S_I2S_TX_START);
}


  void setup() {
    // put your setup code here, to run once:

  }

  void loop() {
    // put your main code here, to run repeatedly:

  }