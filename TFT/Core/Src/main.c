#include "tftlcdlib.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#define MIN(a,b) (((a)<(b))?(a):(b))
#define red		0xf800
#define green		0x07e0
#define blue		0x001f
#define black		0x0000
#define white		0xffff
#define gray		0x8c51
#define yellow		0xFFE0
#define cyan		0x07FF
#define purple		0xF81F

void SystemCoreClockSetHSI(void) {

  RCC->CR |= ((uint32_t)RCC_CR_HSION);                     // Enable HSI
  while ((RCC->CR & RCC_CR_HSIRDY) == 0);                  // Wait for HSI Ready

  RCC->CFGR = RCC_CFGR_SW_HSI;                             // HSI is system clock
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  // Wait for HSI used as system clock

  FLASH->ACR  = FLASH_ACR_PRFTEN;                          // Enable Prefetch Buffer
  FLASH->ACR |= FLASH_ACR_ICEN;                            // Instruction cache enable
  FLASH->ACR |= FLASH_ACR_DCEN;                            // Data cache enable
  FLASH->ACR |= FLASH_ACR_LATENCY_5WS;                     // Flash 5 wait state

  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                         // HCLK = SYSCLK/2
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;                        // APB1 = HCLK
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;                        // APB2 = HCLK

  RCC->CR &= ~RCC_CR_PLLON;                                // Disable PLL

  // PLL configuration:  VCO = HSI/M * N,  Sysclk = VCO/P
  RCC->PLLCFGR = ( 16ul                   |                // PLL_M =  16
                 (192ul <<  6)            |                // PLL_N = 192
                 (  1ul << 16)            |                // PLL_P =   8
                 (RCC_PLLCFGR_PLLSRC_HSI) |                // PLL_SRC = HSI
                 (  3ul << 24)             );              // PLL_Q =   8

  RCC->CR |= RCC_CR_PLLON;                                 // Enable PLL
  while((RCC->CR & RCC_CR_PLLRDY) == 0) __NOP();           // Wait till PLL is ready

  RCC->CFGR &= ~RCC_CFGR_SW;                               // Select PLL as system clock source
  RCC->CFGR |=  RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait till PLL is system clock src
}

void TIM_Init(){

RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	TIM2->PSC = 200;
	TIM2->ARR = 3000;


	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->CR1 |=TIM_CR1_CEN;
	TIM2->SR &= ~TIM_SR_UIF;


}
uint16_t g=0;
void TIM2_IRQHandler() {
		TIM2->SR &= ~TIM_SR_UIF;
		g++;

}

void LCD_Write_Number_dec(uint16_t x, uint16_t y, uint16_t value,uint16_t numberOfChars){

	uint16_t tmp;

	for(int i=0; i<numberOfChars;i++){

		tmp= value%10;
		value = value/10;
		LCD_WriteChar(x+18-i*6,y,tmp+48,YELLOW,BLACK);

	}
}

void LCD_Write_Number_hex(uint16_t x, uint16_t y, uint16_t value){
	uint32_t remainder;
	uint16_t tmp = value;
	uint16_t i=0;
	    while (tmp != 0)
			{
        remainder = tmp % 16;

        if (remainder < 10)
            LCD_WriteChar(x+18-i*6,y,48 + remainder,YELLOW,BLACK);
				else
            LCD_WriteChar(x+18-i*6,y,55 + remainder,YELLOW,BLACK);
				i++;
        tmp = tmp / 16;

    }

}

uint16_t len=0;
void LCD_Write_String(uint16_t x, uint16_t y, unsigned char *str){

	for(int i=0;i<len;i++)
	LCD_WriteChar(x+i*6,y,str[i],BLUE,BLACK);

}

void delay(int number_of_seconds){
	// Converting time into milli_seconds
	int milli_seconds = 1000 * number_of_seconds;
	// Storing start time
	clock_t start_time =clock();
	// looping till required time is not achieved
	while (clock() < start_time + milli_seconds);
}

double get_hour_degree()
{
    time_t rawtime;
    struct tm * timeinfo;

    time ( &rawtime );
    timeinfo = localtime ( &rawtime );
    double h = 8;
    double m = 0;
//    double h = timeinfo->tm_hour;
//    double m = timeinfo->tm_min;
    if (h == 12) h = 0;
    if (m == 60)     {
      m = 0;
      h += 1;
       if(h > 12) h = h - 12;
    }
    double hour_angle = 0.5 * (h * 60 + m);

    return hour_angle;
}

double get_min_degree()
{
    time_t rawtime;
    struct tm * timeinfo;

    time ( &rawtime );
    timeinfo = localtime ( &rawtime );
    double m = 0;
//    double m = timeinfo->tm_min;
    if (m == 60) m = 0;
    double minute_angle = 6 * m;

    return minute_angle;

}

typedef struct {
	volatile uint16_t day;
	uint16_t month;
	volatile uint16_t year;
}Day;

Day* get_day(const char* buff){
    Day* day = (Day*)malloc(sizeof(Day));
    sscanf(buff,"%d/%d/%d",&day->month,&day->day,&day->year);
    return day;
}

uint16_t get_min(const uint16_t* values){
	uint16_t min = values[0];
	for(int i = 0;i < 7;i++){
		if(min > values[i]) min = values[i];
	}
	return min;
}

uint16_t get_max(const uint16_t* values){
	uint16_t max = values[0];
	for(int i = 0;i < 7;i++){
		if(max < values[i]) max = values[i];
	}
	return max;
}

uint16_t* get_mean_values(const uint16_t* values){
	uint16_t* new_val = malloc(sizeof(uint16_t) * 7);
	int step = round((values[0] + values[1] + values[2] + values[3]+ values[4] + values[5] + values[6]) / 7);
	int min = get_min(values);
	int max = get_max(values);
	new_val[0] = min;
	new_val[6] = max;
	for(int i = 1;i < 6;i++){
			new_val[i] = new_val[i-1] + step;
	}
	return new_val;
}

uint16_t get_graph_value(const uint16_t* values,int length,uint16_t value){
	int max = get_max(values);
	uint16_t value2 = (value * length) / max;
	return value2;
}

void draw_covid_cases(){
	//the y axis being drawn
	LCD_HorizontalLine(30,30,210,green);
	LCD_HorizontalLine(31,30,210,green);

	// the x axis being drawn
	LCD_VerticalLine(30,240,210,green);
	LCD_VerticalLine(30,241,210,green);

	//drawing the arrows for the graph
	LCD_Line(240,240,10,310,green);
	LCD_Line(240,240,10,230,green);

	uint16_t clr = 0xF800;
	//draw on x axis
	const char* buff[7] = {"12/27/21","12/28/21","12/29/21","12/30/21","12/31/21","1/1/22","1/2/22"};
	const uint16_t* values[7] = {2503374 ,  2447758 , 2454645  , 2463780  , 2469951  ,2475729 , 2480736};
	const uint16_t* new_values = get_mean_values(values);
	int step = 30;
	int step2 = 210;
	uint16_t before_x = 0,before_y = 0;
	for(int i = 0;i < 5;i++)
	{

		Day* day = get_day(buff[i]);
		// draw on the x axis
		LCD_Write_Number_dec(step,250,day->day,2);
		LCD_WriteChar(step+23,250,'/',YELLOW,BLACK);
		LCD_Write_Number_dec(step+18,250,day->year,2);

		//draw on the y axis
		LCD_Write_Number_dec(5,step2,new_values[i],2);

		//draw points with data
		uint16_t step_y = get_graph_value(new_values,210,values[i]);
		step_y = abs(210-step_y);
		LCD_FilledCircle(step+20,step_y,2,blue);
		if(before_x != 0 && before_y != 0){
			//compute angle between lines;
			uint16_t distance = abs(step+20 - before_x);
			double angle = atan2((210-step_y) - (210 - before_y),step+20 - before_x);
			angle = angle *  57.2958;
			LCD_Line(step+20,step_y,distance+5,(int16_t)angle - 90,blue);

		}
		before_x = step + 20;
		before_y = step_y;
		step2 = step2 - 13 - 23;
		step = step + 13 + 23;

		free(day);
	}

}


int main(){
	SystemCoreClockSetHSI();
	SystemCoreClockUpdate();
	TIM_Init();

	GPIO_Init();
	ADC_Init();

	LCD_Init(BLACK);
	draw_covid_cases();

}


/*int main(){

	SystemCoreClockSetHSI();
	SystemCoreClockUpdate();
	TIM_Init();

	GPIO_Init();
	ADC_Init();

	LCD_Init(BLACK);

	uint16_t clr = 0xF800;

	//time parameters using time.h library
	time_t s, val = 1;
	struct tm* current_time;

	LCD_FilledCircle(120,160,100,TEAL);
	LCD_FilledCircle(120,160,90,BLACK);
	LCD_FilledCircle(120,160,2,BLUE);
	LCD_Circle(120,160,100,BLUE);
	LCD_Circle(120,160,90,BLUE);

	LCD_WriteChar(115,62,'1',clr,TEAL);
	LCD_WriteChar(121,62,'2',clr,TEAL);

	LCD_WriteChar(166,75,'1',clr,TEAL);
	LCD_WriteChar(198,105,'2',clr,TEAL);
	LCD_WriteChar(213,157,'3',clr,TEAL);

	LCD_WriteChar(198,207,'4',clr,TEAL);
	LCD_WriteChar(166,240,'5',clr,TEAL);
	LCD_WriteChar(118,252,'6',clr,TEAL);

	LCD_WriteChar(73,240,'7',clr,TEAL);
	LCD_WriteChar(38,207,'8',clr,TEAL);
	LCD_WriteChar(23,157,'9',clr,TEAL);

	LCD_WriteChar(35,105,'1',clr,TEAL);
	LCD_WriteChar(41,105,'0',clr,TEAL);
	LCD_WriteChar(70,75,'1',clr,TEAL);
	LCD_WriteChar(76,75,'1',clr,TEAL);

	volatile uint16_t angle=180;
	double angleMin=get_min_degree();
	double angleHour=get_hour_degree();
	volatile uint16_t x=1,y=0,z=0;
	while(1){

		if(g>2){

			//sec
			angle=(angle-6);
			//minutes
			angleMin=(angleMin-0.1);
			//hours
			angleHour=(angleHour-0.0083);

			for(int i=0;i<10;i++){
				LCD_Line(120,160,87,angle+6,BLACK);
				LCD_Line(120,160,87,angle,RED);

				//minutes
				LCD_Line(120,160,87,angleMin+0.1,BLACK);
				LCD_Line(120,160,87,angleMin,GREEN);

				//hours
				LCD_Line(120,160,87,angleHour+0.0083,BLACK);
				LCD_Line(120,160,87,angleHour,BLUE);
				//delay(1000);
			}

			if(x==59)
			{
				if(y==59)
				{
					if(z==23)
					{
						z=0;
						y=0;
						x=0;}
					else
					{
						z++;
						y=0;
						x=0;
					}
				}
				else
				{
					y++;
					x=0;}
				}
			else
				x++;


			LCD_FilledCircle(120,160,2,BLUE);

			//display REAL current time (of the computer for example) - time.h library
			s = time(NULL);
			// to get current time
			current_time = localtime(&s);
			//print on display of current time
			LCD_Write_Number_dec(325,300,z,2);//real hour
			LCD_WriteChar(352,300,':',YELLOW,BLACK);
			LCD_Write_Number_dec(350,300,y,2);//real minute
			LCD_WriteChar(378,300,':',YELLOW,BLACK);
			LCD_Write_Number_dec(375,300,x,2);//real second

			g=0;
		}

	}
}

*/
