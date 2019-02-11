#include <math.h>

/* Chip Select & Reset pins for EVE2 controller */
#define PIN_NUM_CS 5
#define PIN_NUM_RST 2

/* FT81x graphics engine specific macros useful for static display list generation. PLEASE CHECK USED FUNCTIONS FOR CORRECT DATA STRUCTURE*/
#define ALPHA_FUNC(func,ref) ((9UL<<24)|(((func)&7UL)<<8)|(((ref)&255UL)<<0))
#define BEGIN(prim) ((31UL<<24)|(((prim)&15UL)<<0))
#define BITMAP_HANDLE(handle) ((5UL<<24)|(((handle)&31UL)<<0))
#define BITMAP_LAYOUT(format,linestride,height) ((7UL<<24)|(((format)&31UL)<<19)|(((linestride)&1023UL)<<9)|(((height)&511UL)<<0))
#define BITMAP_SIZE(filter,wrapx,wrapy,width,height) ((8UL<<24)|(((filter)&1UL)<<20)|(((wrapx)&1UL)<<19)|(((wrapy)&1UL)<<18)|(((width)&511UL)<<9)|(((height)&511UL)<<0))
#define BITMAP_TRANSFORM_A(a) ((21UL<<24)|(((a)&131071UL)<<0))
#define BITMAP_TRANSFORM_B(b) ((22UL<<24)|(((b)&131071UL)<<0))
#define BITMAP_TRANSFORM_C(c) ((23UL<<24)|(((c)&16777215UL)<<0))
#define BITMAP_TRANSFORM_D(d) ((24UL<<24)|(((d)&131071UL)<<0))
#define BITMAP_TRANSFORM_E(e) ((25UL<<24)|(((e)&131071UL)<<0))
#define BITMAP_TRANSFORM_F(f) ((26UL<<24)|(((f)&16777215UL)<<0))
#define BLEND_FUNC(src,dst) ((11UL<<24)|(((src)&7UL)<<3)|(((dst)&7UL)<<0))
#define CALL(dest) ((29UL<<24)|(((dest)&65535UL)<<0))
#define CELL(cell) ((6UL<<24)|(((cell)&127UL)<<0))
#define CLEAR(c,s,t) ((0x26<<24)|(c<<2)|(s<<1)| t)
#define CLEAR_COLOR_A(alpha) ((15UL<<24)|(((alpha)&255UL)<<0))
#define CLEAR_COLOR_RGB(red,green,blue) ((0x02<<24) | (red<<16) | (blue<<8) || green)
#define CLEAR_STENCIL(s) ((17UL<<24)|(((s)&255UL)<<0))
#define CLEAR_TAG(s) ((18UL<<24)|(((s)&255UL)<<0))
#define COLOR_A(alpha) ((16UL<<24)|(((alpha)&255UL)<<0))
#define COLOR_MASK(r,g,b,a) ((32UL<<24)|(((r)&1UL)<<3)|(((g)&1UL)<<2)|(((b)&1UL)<<1)|(((a)&1UL)<<0))
#define COLOR_RGB(red,green,blue) ((4UL<<24)|(((red)&255UL)<<16)|(((green)&255UL)<<8)|(((blue)&255UL)<<0))
#define D_DISPLAY() 0x00
#define END() ((33UL<<24))
#define JUMP(dest) ((30UL<<24)|(((dest)&65535UL)<<0))
#define LINE_WIDTH(width) ((14UL<<24)|(((width)&4095UL)<<0))
#define MACRO(m) ((37UL<<24)|(((m)&1UL)<<0))
#define POINT_SIZE(size) ((13UL<<24)|(((size)&8191UL)<<0))
#define RESTORE_CONTEXT() ((35UL<<24))
#define RETURN() ((36UL<<24))
#define SAVE_CONTEXT() ((34UL<<24))
#define STENCIL_FUNC(func,ref,mask) ((10UL<<24)|(((func)&7UL)<<16)|(((ref)&255UL)<<8)|(((mask)&255UL)<<0))
#define STENCIL_MASK(mask) ((19UL<<24)|(((mask)&255UL)<<0))
#define STENCIL_OP(sfail,spass) ((12UL<<24)|(((sfail)&7UL)<<3)|(((spass)&7UL)<<0))
#define TAG(s) ((3UL<<24)|(((s)&255UL)<<0))
#define TAG_MASK(mask) ((20UL<<24)|(((mask)&1UL)<<0))
#define VERTEX2F(x,y) ((1UL<<30)|(((x)&32767UL)<<15)|(((y)&32767UL)<<0))
#define VERTEX2II(x,y,handle,cell) ((2UL<<30)|(((x)&511UL)<<21)|(((y)&511UL)<<12)|(((handle)&31UL)<<7)|(((cell)&127UL)<<0))

/* EVE2 Constants */
#define EVE2_WRITE_MASK 0b10000000
#define EVE2_BL_MIN_FREQ 250
#define EVE2_BL_MAX_FREQ 10000
#define TIMEOUT_EVE2_DETECT 300 //in miliseconds

/* Display Specific Timing Parameters. Parameters set a for QVGA resolution with a 3.5" display */
#define DISP_HCYCLE 408
#define DISP_HOFFSET 68
#define DISP_HSYNC0 0
#define DISP_HSYNC1 10
#define DISP_VCYCLE 262
#define DISP_VOFFSET 18
#define DISP_VSYNC0 0
#define DISP_VSYNC1 2
#define DISP_SWIZZLE 0
#define DISP_PCLK_POL 0
#define DISP_CSPREAD 1
#define DISP_HSIZE 320
#define DISP_VSIZE 280

/* CMD Definitions */
#define ACTIVE 0x00
#define DLSWAP_FRAME 0x02

/* General Registers */
#define REG_ID 0x302000
#define REG_GPIO 0x302094
#define REG_GPIO_DIR 0x302090
#define REG_PWM_DUTY 0x3020D4
#define REG_PWM_HZ 0x3020D0
#define REG_CMD_DL 0x302100
#define REG_VOL_PB 0x302080
#define REG_VOL_SOUND 0x302084
#define REG_SOUND 0x302088


/* Timing Registers */
#define REG_HCYCLE 0x30202C
#define REG_HOFFSET 0x302030
#define REG_HSYNC0 0x302038
#define REG_HSYNC1 0x30203C
#define REG_HSIZE 0x302034
#define REG_VCYCLE 0x302040
#define REG_VOFFSET 0x302044
#define REG_VSYNC0 0x30204C
#define REG_VSYNC1 0x302050
#define REG_VSIZE 0x302048
#define REG_SWIZZLE 0x302064
#define REG_PCLK_POL 0x30206C
#define REG_CSPREAD 0x302068
#define REG_DLSWAP 0x302054
#define REG_FRAMES 0x302004
#define REG_DITHER 0x302060
#define REG_PCLK   0x302070

/* Graphics primitives */
#define BITMAPS 1
#define POINTS 2
#define LINES 3
#define LINE_STRIP 4
#define EDGE_STRIP_R 5
#define EDGE_STRIP_L 6
#define EDGE_STRIP_A 7
#define EDGE_STRIP_B 8
#define RECTS 9

  
/* Memory Mapping */
#define RAM_DL 0x300000

/* Register's Reset Value */
#define REG_ID_RST 0x7C

//------------------------------ Function Declarations ------------------------------//

/* Drawwing example */
uint8_t eve2_display_ftdi_logo_example(void);

/* SPI Init */
void spi_init_slow_mode(void);
void spi_init_fast_mode(void);

/* EVE2 Commands */
uint8_t eve2_init(void);
uint8_t eve2_config(void);
void eve2_set_backlight_frequency(uint16_t blFreq);
void eve2_set_backlight_intensity(uint8_t dutyCycle);
void eve2_set_driving(uint8_t reg_drive);

/* EVE2 Transactions. Abstraction Layer */
void eve2_send_cmd(uint8_t cmd, uint8_t parameter);

void eve2_wr8(uint32_t addr, uint8_t dataByte);
void eve2_wr16(uint32_t addr, uint16_t dataBytes);
void eve2_wr32(uint32_t addr, uint32_t dataBytes);

uint8_t eve2_rd8(uint32_t addr);
uint16_t eve2_rd16(uint32_t addr);
uint32_t eve2_rd32(uint32_t addr);

/* Hardware Specific Transactions */
static uint8_t* eve2_read(uint8_t* addr, uint8_t addrLength, uint8_t dataLenght);
void eve2_write(uint8_t* addr, uint8_t* dataBytes, uint8_t addrLength, uint8_t dataLenght);

//----------------------------------------------------------------------------------//
