#include <LPC17xx.h>
#include <RTL.h>
#include <stdio.h>
#include <stdint.h>

OS_MUT mutex;
/*================== LED ================================*/
#define u_green (1UL<<28)
#define u_red (1UL<<25)
#define g_green (1UL<<26)
#define g_red (1UL<<17)
#define t_green (1UL<<18)
#define t_red (1UL<<15)
#define buzzer (1UL<<24)

/* ================= LCD PINS (LOCKED – REFERENCE) ================= */
#define LCD_RS_PORT   LPC_GPIO3
#define LCD_RS_PIN    25
#define LCD_RW_PORT   LPC_GPIO3
#define LCD_RW_PIN    26
#define LCD_E_PORT    LPC_GPIO4
#define LCD_E_PIN     28
#define LCD_D4_PORT   LPC_GPIO1
#define LCD_D4_PIN    20
#define LCD_D5_PORT   LPC_GPIO1
#define LCD_D5_PIN    21
#define LCD_D6_PORT   LPC_GPIO1
#define LCD_D6_PIN    22
#define LCD_D7_PORT   LPC_GPIO1
#define LCD_D7_PIN    23

#define BIT(x) (1UL<<(x))
#define SET(p,n) ((p)->FIOSET = BIT(n))
#define CLR(p,n) ((p)->FIOCLR = BIT(n))

/* ================= ULTRASONIC ================= */
#define TRIG (1UL<<0)
#define ECHO (1UL<<1)

/* ================= GAS SENSOR (ADC) ================= */
#define ADC_DONE    (1U<<31)
#define ADC_CLK_EN  (1<<12)
#define SEL_AD0_0   (1<<0)
#define CLKDIV      1
#define PWRUP       (1<<21)
#define START_CNV   (1<<24)

/* ================= DHT11 ================= */
#define DHT_PORT LPC_GPIO0
#define DHT_PIN  16
#define DHT_MASK (1<<DHT_PIN)

/* ================= KEYPAD ================= */
#define R1 (1UL<<4)
#define R2 (1UL<<5)
#define R3 (1UL<<6)
#define R4 (1UL<<7)
#define C1 (1UL<<8)
#define C2 (1UL<<9)
#define C3 (1UL<<10)
#define C4 (1UL<<11)

const char keymap[4][4]={
 {'1','2','3','A'},
 {'4','5','6','B'},
 {'7','8','9','C'},
 {'*','0','#','D'}
};

/* ================= GLOBALS ================= */
volatile unsigned int threshold_cm=0;
volatile unsigned int gas_threshold_adc=0;
volatile unsigned int temp_threshold=0;
volatile unsigned int dist;

/* ================= TIMER1 ================= */
void Timer1_Init(void){
    LPC_SC->PCONP |= (1<<2);
    LPC_TIM1->PR = 24;
}

void delay_us(unsigned int us){
    LPC_TIM1->TC=0;
    LPC_TIM1->TCR=1;
    while(LPC_TIM1->TC<us);
    LPC_TIM1->TCR=0;
}

void delay_ms(unsigned int ms){
    while(ms--) delay_us(1000);
}

/* ================= LCD ================= */
void lcd_gpio_init(void){
    LPC_PINCON->PINSEL7 &= ~((3<<18)|(3<<20));
    LPC_PINCON->PINSEL3 &= ~((3<<8)|(3<<10)|(3<<12)|(3<<14));
    LPC_PINCON->PINSEL9 &= ~(3<<24);

    LCD_RS_PORT->FIODIR |= BIT(LCD_RS_PIN);
    LCD_RW_PORT->FIODIR |= BIT(LCD_RW_PIN);
    LCD_E_PORT->FIODIR  |= BIT(LCD_E_PIN);
    LCD_D4_PORT->FIODIR |= BIT(LCD_D4_PIN);
    LCD_D5_PORT->FIODIR |= BIT(LCD_D5_PIN);
    LCD_D6_PORT->FIODIR |= BIT(LCD_D6_PIN);
    LCD_D7_PORT->FIODIR |= BIT(LCD_D7_PIN);

    CLR(LCD_RW_PORT,LCD_RW_PIN);
}

void lcd_pulse(void){
    SET(LCD_E_PORT,LCD_E_PIN);
    delay_us(1);
    CLR(LCD_E_PORT,LCD_E_PIN);
}

void lcd_write4(unsigned char d){
    CLR(LCD_D4_PORT,LCD_D4_PIN);
    CLR(LCD_D5_PORT,LCD_D5_PIN);
    CLR(LCD_D6_PORT,LCD_D6_PIN);
    CLR(LCD_D7_PORT,LCD_D7_PIN);
    if(d&1) SET(LCD_D4_PORT,LCD_D4_PIN);
    if(d&2) SET(LCD_D5_PORT,LCD_D5_PIN);
    if(d&4) SET(LCD_D6_PORT,LCD_D6_PIN);
    if(d&8) SET(LCD_D7_PORT,LCD_D7_PIN);
    lcd_pulse();
}

void lcd_cmd(unsigned char c){
    CLR(LCD_RS_PORT,LCD_RS_PIN);
    lcd_write4(c>>4);
    lcd_write4(c&0x0F);
    delay_ms(2);
}

void lcd_data(unsigned char d){
    SET(LCD_RS_PORT,LCD_RS_PIN);
    lcd_write4(d>>4);
    lcd_write4(d&0x0F);
    delay_ms(2);
}

void lcd_init(void){
    lcd_gpio_init();
    delay_ms(20);
    lcd_write4(0x03); delay_ms(5);
    lcd_write4(0x03); delay_ms(5);
    lcd_write4(0x03); delay_ms(1);
    lcd_write4(0x02);
    lcd_cmd(0x28);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_cmd(0x01);
}

void lcd_gotoxy(unsigned char c,unsigned char r){
    lcd_cmd(0x80 | (r?0x40:0x00) | c);
}

void lcd_puts(const char *s){
	os_mut_wait(&mutex, 0xFFFF);	
    while(*s) lcd_data(*s++);
	os_mut_release(&mutex);
}

/* ================= KEYPAD ================= */
void keypad_init(void){
    LPC_GPIO0->FIODIR |= (R1|R2|R3|R4);
    LPC_GPIO0->FIODIR &= ~(C1|C2|C3|C4);
}

char keypad_getkey(void){
    unsigned int r; char k;
    while(1){
        for(r=0;r<4;r++){
            LPC_GPIO0->FIOSET=(R1|R2|R3|R4);
            if(r==0) LPC_GPIO0->FIOCLR=R1;
            if(r==1) LPC_GPIO0->FIOCLR=R2;
            if(r==2) LPC_GPIO0->FIOCLR=R3;
            if(r==3) LPC_GPIO0->FIOCLR=R4;
            delay_ms(5);
            if(!(LPC_GPIO0->FIOPIN&C1)) k=keymap[r][0];
            else if(!(LPC_GPIO0->FIOPIN&C2)) k=keymap[r][1];
            else if(!(LPC_GPIO0->FIOPIN&C3)) k=keymap[r][2];
            else if(!(LPC_GPIO0->FIOPIN&C4)) k=keymap[r][3];
            else continue;
            while(!(LPC_GPIO0->FIOPIN&C1)||
                  !(LPC_GPIO0->FIOPIN&C2)||
                  !(LPC_GPIO0->FIOPIN&C3)||
                  !(LPC_GPIO0->FIOPIN&C4));
            delay_ms(20);
            return k;
        }
    }
}

unsigned int keypad_get_number(const char *msg){
    char k; unsigned int n=0; char buf[16];
    lcd_cmd(0x01);
    lcd_puts(msg);
    while(1){
        k=keypad_getkey();
        if(k>='0'&&k<='9'){
            n=n*10+(k-'0');
            lcd_gotoxy(0,1);
            snprintf(buf,16,"%u     ",n);
            lcd_puts(buf);
        } else if(k=='#') return n;
    }
}

/* ================= ULTRASONIC ================= */
unsigned int ultrasonic_read(void){
    unsigned int t=0,to=0;
    LPC_GPIO0->FIOCLR=TRIG;
    delay_us(2);
    LPC_GPIO0->FIOSET=TRIG;
    delay_us(10);
    LPC_GPIO0->FIOCLR=TRIG;
    while(!(LPC_GPIO0->FIOPIN&ECHO)){
        if(++to>30000) return 0;
        delay_us(1);
    }
    LPC_TIM1->TC=0; LPC_TIM1->TCR=1;
    while(LPC_GPIO0->FIOPIN&ECHO){
        if(LPC_TIM1->TC>30000) break;
    }
    t=LPC_TIM1->TC;
    LPC_TIM1->TCR=0;
    return (t*34)/2000;
}

/* ================= DHT11 ================= */
static void delay_us_dht(uint32_t us){ us*=25; while(us--) __NOP(); }
static void delay_ms_dht(uint32_t ms){ while(ms--) delay_us_dht(1000); }
static void dht_out(void){ DHT_PORT->FIODIR|=DHT_MASK; }
static void dht_in(void){ DHT_PORT->FIODIR&=~DHT_MASK; }
static int dht_wait(int l,uint32_t t){
    while(t--){ if(((DHT_PORT->FIOPIN&DHT_MASK)!=0)==l) return 1; delay_us_dht(1); }
    return 0;
}
uint8_t DHT11_Read(uint8_t *h,uint8_t *t){
    uint8_t d[5]={0}; uint32_t i,j;
    dht_out(); DHT_PORT->FIOCLR=DHT_MASK; delay_ms_dht(20);
    DHT_PORT->FIOSET=DHT_MASK; delay_us_dht(30); dht_in();
    if(!dht_wait(0,100)||!dht_wait(1,100)||!dht_wait(0,100)) return 0;
    __disable_irq();
    for(i=0;i<5;i++)for(j=0;j<8;j++){
        d[i]<<=1;
        if(!dht_wait(1,100)) goto ex;
        delay_us_dht(30);
        if(DHT_PORT->FIOPIN&DHT_MASK) d[i]|=1;
        if(!dht_wait(0,100)) goto ex;
    }
ex: __enable_irq();
    if((d[0]+d[1]+d[2]+d[3])!=d[4]) return 0;
    *h=d[0]; *t=d[2]; return 1;
}

/* ================= SENSOR TASKS ================= */
__task void ultrasonic_task(void){
    char b[16];
	LPC_GPIO0->FIODIR |= (u_green|u_red|buzzer);
    while(1){
        dist=ultrasonic_read();
        lcd_gotoxy(0,0);
        snprintf(b,16,"TH:%3ucm  ",threshold_cm); lcd_puts(b);
        lcd_gotoxy(0,1);
        if(dist&&dist<=threshold_cm){
					LPC_GPIO0->FIOSET = u_red|buzzer;
					LPC_GPIO0->FIOCLR = u_green;
					snprintf(b,16,"ALERT D:%3u cm",dist);
				
				}
        else{
					LPC_GPIO0->FIOSET = u_green;
					LPC_GPIO0->FIOCLR = u_red|buzzer;
					snprintf(b,16,"SAFE  D:%3u cm",dist);
        }
					lcd_puts(b);
        os_dly_wait(50);
    }
}

__task void gas_task(void){
    int adc; char b[16];
	LPC_GPIO0->FIODIR |= (g_green|g_red|buzzer);
    while(1){
        LPC_ADC->ADCR|=START_CNV;
        while(!(LPC_ADC->ADDR0&ADC_DONE));
        adc=(LPC_ADC->ADDR0>>4)&0xFFF;
        lcd_gotoxy(0,0);
        snprintf(b,16,"TH:%4u",gas_threshold_adc); lcd_puts(b);
        lcd_gotoxy(0,1);
        if(adc>=gas_threshold_adc){
						LPC_GPIO0->FIOSET = g_red|buzzer;
					LPC_GPIO0->FIOCLR = g_green;
					snprintf(b,16,"ALERT ADC:%u ",adc);
				}
        else{
						LPC_GPIO0->FIOSET = g_green;
					LPC_GPIO0->FIOCLR = g_red|buzzer;
					snprintf(b,16,"SAFE  ADC:%u ",adc);
				}
        lcd_puts(b);
        os_dly_wait(50);
    }
}

__task void DHT_Task(void){
    uint8_t h,t; char b[16];
	LPC_GPIO0->FIODIR |= (t_green|t_red|buzzer);
    while(1){
        lcd_gotoxy(0,0);
        snprintf(b,16,"TH:%3uC",temp_threshold); lcd_puts(b);
        lcd_gotoxy(0,1);
        if(DHT11_Read(&h,&t)){
            if(t>=temp_threshold){
							LPC_GPIO0->FIOSET = t_red|buzzer;
					LPC_GPIO0->FIOCLR = t_green;
							snprintf(b,16,"ALERT T:%3uC",t);
						}
						else{
							LPC_GPIO0->FIOSET = t_green;
					LPC_GPIO0->FIOCLR = t_red|buzzer;
							snprintf(b,16,"SAFE  T:%3uC",t);
						}
        } else snprintf(b,16,"DHT11 ERROR");
        lcd_puts(b);
        os_dly_wait(50);
    }
}

/* ================= MENU TASK ================= */

__task void menu_task(void){
    char ch;
		os_mut_init(&mutex);
	while(1){
    lcd_cmd(0x01);
    lcd_puts("A-Ultra B-Gas");
    lcd_gotoxy(0,1);
    lcd_puts("C-Temp");

    ch = keypad_getkey();
    lcd_cmd(0x01);

    if(ch=='A'){
        threshold_cm = keypad_get_number("Dist TH:");
        os_tsk_create(ultrasonic_task, 1);
			break;
    }
    else if(ch=='B'){
        gas_threshold_adc = keypad_get_number("Gas TH:");
        LPC_SC->PCONP |= ADC_CLK_EN;
        LPC_PINCON->PINSEL1 |= (1<<14);
        LPC_ADC->ADCR = (CLKDIV<<8)|PWRUP|SEL_AD0_0;
        os_tsk_create(gas_task, 1);
			break;
    }
    else if(ch=='C'){
        temp_threshold = keypad_get_number("Temp TH:");
        os_tsk_create(DHT_Task, 1);
			break;
    }
		else{
			lcd_cmd(0x01);
    lcd_puts("Invalid Input!! ");
    lcd_gotoxy(0,1);
    lcd_puts("               ");
		delay_ms(1000);
		}
	}
    os_tsk_delete_self();
}

/*================initialization task===========*/

__task void init_task(void){
    os_mut_init(&mutex);

    Timer1_Init();
    lcd_init();
    keypad_init();

    LPC_GPIO0->FIODIR |= TRIG;
    LPC_GPIO0->FIODIR &= ~ECHO;

    os_tsk_create(menu_task, 1);
    os_tsk_delete_self();
}

/* ================= MAIN ================= */
int main(void){
    os_sys_init(init_task);
}