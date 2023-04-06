#define F_CPU 30000000L

#define LCD_RS_0 (GPIO_B9 = 0)
#define LCD_RS_1 (GPIO_B9 = 1)
#define LCD_E_0  (GPIO_B8 = 0)
#define LCD_E_1  (GPIO_B8 = 1)
#define LCD_D4_0 (GPIO_B10 = 0)
#define LCD_D4_1 (GPIO_B10 = 1)
#define LCD_D5_0 (GPIO_B11 = 0)
#define LCD_D5_1 (GPIO_B11 = 1)
#define LCD_D6_0 (GPIO_B2 = 0)
#define LCD_D6_1 (GPIO_B2 = 1)
#define LCD_D7_0 (GPIO_B3 = 0)
#define LCD_D7_1 (GPIO_B3 = 1)
#define CHARS_PER_LINE 16

void Delay_us(unsigned char us);
void waitms (unsigned int ms);
void LCD_pulse (void);
void LCD_byte (unsigned char x);
void WriteData (unsigned char x);
void WriteCommand (unsigned char x);
void LCD_4BIT (void);
void LCDprint(char * string, unsigned char line, unsigned char clear);
