//Курсовой проект. Ганеев Р.М. 6403
//Подключение библиотек
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include <string.h>
#include <stdio.h> 

//Для работы 1-Wire подключаем библиотеку OneWire.h
#include "OneWire.h"

//Определяем USART
#define OW_USART USART1
//Максимальное количество устройств на шине
#define MAXDEVICES_ON_THE_BUS 6
//Задаём функцию включения-выключения светодиода
#define LED_ONOFF();  GPIOC->ODR ^= GPIO_Pin_13; 

uint8_t devices;
OneWire ow;	
uint32_t pDelay = 300, i;
uint8_t sensor;

//Подключение вывода через SWO - НАЧАЛО
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA           0x01000000

struct __FILE { int handle;};
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
   if (DEMCR & TRCENA) {
		while (ITM_Port32(0) == 0){};
				ITM_Port8(0) = ch;
  }
  return(ch);
}
//SWO - КОНЕЦ 

char *crcOK;

//Настройка вывода UART
void usart_gpio_setup(void) {
	//Объявляем структуру порта
		GPIO_InitTypeDef GPIO_InitStructure;
	//Включаем тактирование UART
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	//Заполняем структуру порта
	//Ножка GPIO_Pin_9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	//Режим работы OpenDrain - GPIO_Mode_AF_OD
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}	
//Настройка светодиода
void board_led_ini(void){
	//Объявляем структуру порта
	GPIO_InitTypeDef  GPIO_InitStructure;
	//Включаем тактирование
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	//Заполняем структуру порта
	//Ножка GPIO_Pin_13
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//Инициализируем порт
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_SetBits(GPIOC, GPIO_Pin_13); 
}

//Включаем обработчик прерываний USART
void USART1_IRQHandler(void){
	owReadHandler();
}

//Получение id датчиков
void get_ROMid (void){
	//Есть ли датчики на шине
   if (owResetCmd() != ONEWIRE_NOBODY) {
				//Получаем переменную devices id всех устройств на шине или возващаем код ошибки
         devices = owSearchCmd(&ow);
		 //Если устройств не найдено, то мигаем светодиодом
		 if (devices <= 0) {
        printf("\n\rError has happened!");
				while (1){
				    LED_ONOFF();
					  pDelay = 10000;
            for (i = 0; i < pDelay * 1; i++) {}   
				}

      }
      printf("\n\rfound %d devices on 1-wire bus", devices);
			
			 i = 0;
			//Выводим в консоль все найденные ROM
		      	for (; i < devices; i++) {
            RomCode *r = &ow.ids[i];
            uint8_t crc = owCRC8(r);
            crcOK = (crc == r->crc)?"CRC OK":"CRC ERROR!";
            printf("\n\rdevice %d (SN: %02X/%02X%02X%02X%02X%02X%02X/%02X) ", i, r->family, r->code[5], r->code[4], r->code[3],
                   r->code[2], r->code[1], r->code[0], r->crc);
            printf("crcOK");
            if (crc != r->crc) {
                printf("\n\r can't read cause CNC error");
           }
		   } 

    }
        pDelay = 1000000;
        for (i = 0; i < pDelay * 1; i++)    
           __asm__("nop");
}
//Сортировка массива по возрастанию
void bubble_sort(double *data, int size) {
   int i, j;
   for (i = 0; i < size; ++i) {
      for (j = size - 1; j > i; --j) {
         if (data[j] < data[j-1]) {
            double t = data[j - 1];
            data[j - 1] = data[j];
            data[j] = t;
         }
      }
   }
}

//Проверка на выход за пределы пороговых значений
void check_mass(double ds[], int nupControl, int ndControl) {
			if(ds[4]>=nupControl){
			printf("\n\rLed_2 is enabled");	
		 }else if(ds[4]<=(ndControl*0.8)){
			printf("\n\rLed_1 is enabled");	
		 }else printf("\n\rLed_3 is enabled");
}

//Main
int main(void) {
int iter = 0;
//Допустимое, нижнее значение
int dControl= 25;
//Допустимое, верхнее значение
int upControl = 31;
//Пороговое, нижнее значение
int ndControl= 30;
//Пороговое, верхнее значение
int nupControl = 31;
//Осреднение
double average = 0;
//Массив для допускового контроля и для ошибок
double b[2] = {0};
//Массив для показаний с первого датчика
double ds1[9] = {0};
//Массив для показаний со второго датчика
double ds2[9] = {0};
//SystemClock_Config();
Temperature tchek;
usart_gpio_setup();
board_led_ini();
get_ROMid();
printf("\n\rControl_Test");
//Производим 5 измерений
	while(iter<5){
		LED_ONOFF();
		printf("\n\r Dimension number - %d",iter);
		for (i = 0; i < devices; i++){
			tchek = readTemperature(&ow, &ow.ids[i], 1);
			b[i]=tchek.inCelsus+(tchek.frac/10.0);
			if(b[i]>upControl){
				printf("\n\r Error: upControl DS18B20 N_%d", i);
				b[i]=404;
			}else
			if(b[i]<dControl){
				printf("\n\r Error: dControl DS18B20 N_%d", i);
				b[i]=404;
			}else
			{
				printf("\n\rDS18B20 N_%d , Temp: %3d.%dC",i,tchek.inCelsus, tchek.frac);				
			}
		}
		if((b[0] == 404)||(b[1] == 404)){
			break;
		}
		pDelay = 1000000; //Скорость измерения
		for (i = 0; i < pDelay * 1; i++){}
		iter++;
}
	
//Если ошибки на двух датчиках
if((b[0] == 404) && (b[1] == 404)){
	printf("\n\rFatal_Error - both sensors are in error");
} else {
		//Если ошибка на первом датчике
		if(b[0] == 404)
		{
			average=b[1];
			printf("\n\rAveraging_1 = %f",average);
			printf("\n\r<--------------------------------------------->");			
			printf("\n\rWrite in Mass");
			iter=0;
			//Производим 9 измерений и результаты записываем в массив
			while(iter<9){		
					LED_ONOFF();
					printf("\n\r Dimension number - %d",iter);
					tchek = readTemperature(&ow, &ow.ids[1], 1);
					ds2[iter]=tchek.inCelsus+(tchek.frac/10.0);
					printf("\n\rDS18B20 N_1 , Temp: %3d.%dC",tchek.inCelsus, tchek.frac);	
					iter++;
					pDelay = 1000000; //Скорость измерения
					for (i = 0; i < pDelay * 1; i++){}  
				}
			//Сортировка и вывод массива 2
			printf("\n\rMassiv");	
			bubble_sort(ds2, sizeof(ds2)/sizeof(ds2[0]));
			for (i = 0; i < sizeof(ds2)/sizeof(ds2[0]); ++i) {
				printf("\n\r%f ", ds2[i]);
		 }
			//Проверка массива 2
			check_mass(ds2,nupControl,ndControl);

		}else {
		
		//Если ошибка на втором датчике	
		if(b[1] == 404)
		{
			average=b[0];
			printf("\n\rAveraging_0 = %f",average);
			printf("\n\r<--------------------------------------------->");			
			printf("\n\rWrite in Mass");
			iter=0;	
			//Производим 9 измерений и результаты записываем в массив			
			while(iter<9){		
					LED_ONOFF();
					printf("\n\r Dimension number - %d",iter);
					tchek = readTemperature(&ow, &ow.ids[0], 1);
					ds1[iter]=tchek.inCelsus+(tchek.frac/10.0);
					printf("\n\rDS18B20 N_0 , Temp: %3d.%dC",tchek.inCelsus, tchek.frac);	
					iter++;
					pDelay = 1000000; //Скорость измерения
					for (i = 0; i < pDelay * 1; i++){}   
				}
			//Сортировка и вывод массива 1
			printf("\n\rMassiv");	
			bubble_sort(ds1, sizeof(ds1)/sizeof(ds1[0]));
			for (i = 0; i < sizeof(ds1)/sizeof(ds1[0]); ++i) {
				printf("\n\r%f ", ds1[i]);
		 }
			//Проверка массива 1
			check_mass(ds1,nupControl,ndControl);
		}else{
			
			//Если ошибок нет
			average = (b[0]+b[1])/2;
			printf("\n\rAveraging = %f",average);	
			printf("\n\r<--------------------------------------------->");			
			printf("\n\rWrite in Mass");
			iter=0;		
			//Производим 9 измерений и результаты записываем в массив
			while(iter<9){		
					LED_ONOFF();
					printf("\n\r Dimension number - %d",iter);
					tchek = readTemperature(&ow, &ow.ids[0], 1);
					ds1[iter]=tchek.inCelsus+(tchek.frac/10.0);
					printf("\n\rDS18B20 N_0 , Temp: %3d.%dC",tchek.inCelsus, tchek.frac);	
					tchek = readTemperature(&ow, &ow.ids[1], 1);
					ds2[iter]=tchek.inCelsus+(tchek.frac/10.0);
					printf("\n\rDS18B20 N_1 , Temp: %3d.%dC",tchek.inCelsus, tchek.frac);	
					iter++;
					pDelay = 1000000; //Скорость измерения
					for (i = 0; i < pDelay * 1; i++){}  
				}
	  	//Сортировка и вывод массива 1
			printf("\n\rMassiv_1");	
			bubble_sort(ds1, sizeof(ds1)/sizeof(ds1[0]));
			for (i = 0; i < sizeof(ds1)/sizeof(ds1[0]); ++i) {
				printf("\n\r%f ", ds1[i]);
		 }	
			//Проверка массива 1
			check_mass(ds1,nupControl,ndControl);
		 
		 	//Сортировка и вывод массива 2
			printf("\n\rMassiv_2");	
			bubble_sort(ds2, sizeof(ds2)/sizeof(ds2[0]));
			for (i = 0; i < sizeof(ds2)/sizeof(ds2[0]); ++i) {
				printf("\n\r%f ", ds2[i]);
		 }
			//Проверка массива 2
			check_mass(ds2,nupControl,ndControl);
		 
		}
	}
}
	printf("\n\rEnd programm");
} 
