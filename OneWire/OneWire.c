#include "OneWire.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
volatile uint8_t recvFlag;
volatile uint16_t rc_buffer[5];
/**
            Пример взят на GitHub для библиотеки libopencm3 автор  Stanislav Lakhtin
            Реализация протокола 1wire на базе библиотеки stdperiph для микроконтроллера STM32F103
            Общая идея заключается в использовании аппаратного USART uK для иммитации работы 1wire.
            Подключение устройств осуществляется на выбранный USART к TX пину, который должен быть подтянут к линии питания сопротивлением 4.7К.
            Реализация библиотеки осуществляет замыкание RX на TX внутри uK, оставляя ножку RX доступной для использования в других задачах.
 */

/// Метод реализует переключение работы USART в half-duplex режим. Метод не работает для 1wire реализации
//void usart_enable_halfduplex() {//передача и прием по одному проводу
//	OW_USART->CR2 &= ~USART_CR2_LINEN;//~включение режима LIN
//	OW_USART->CR2 &= ~USART_CR2_CLKEN;//~включение синхросигнала
//	OW_USART->CR3 &= ~USART_CR3_SCEN;//~включение режима работы со смарт картами
//	OW_USART->CR3 &= ~USART_CR3_IREN;//~ включение режима IRDA
//  OW_USART->CR3 |= USART_CR3_HDSEL;//переключение работы USART в half-duplex
//}
void usart_enable_halfduplex(){//переключение работы USART в half-duplex режим
    USART_HalfDuplexCmd(OW_USART,ENABLE);
    }
/** Метод реализует переключение выбранного USART в нужный режим
 * @param[in] usart Выбранный аппаратный usart -- (USART1, USART2, etc...)
 * @param[in] baud Скорость в бодах (9600, 115200, etc...)
 *
 */

uint8_t getUsartIndex(void);

void usart_setup_(uint32_t baud) {
NVIC_InitTypeDef NVIC_InitStructure;			
USART_InitTypeDef USART_InitStructure;									 
uint8_t irqs[] = {USART1_IRQn,USART2_IRQn,USART3_IRQn};
uint8_t irqNumber = irqs[getUsartIndex()];//irqs[usart_n];  
    
    /* NVIC Configuration */
    /* Enable the USARTx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = irqNumber;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /* Configure the USART1 */
 
    /* USART1 configuration ------------------------------------------------------*/
  
    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength =  8;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
 
    USART_Init(OW_USART, &USART_InitStructure);
 
    /* Enable USART1 */
    USART_Cmd(OW_USART, ENABLE);
		usart_enable_halfduplex();
 
    /* Enable the USART1 Receive interrupt: this interrupt is generated when the
        USART1 receive data register is not empty */
    USART_ITConfig(OW_USART, USART_IT_RXNE, ENABLE);
}

void owInit(OneWire *ow) { //Затираем все ROM и lastROM в структуре типа OneWire
  int i=0, k = 0;
  for (; i < MAXDEVICES_ON_THE_BUS; i++) {
   uint8_t *r = (uint8_t *)&ow->ids[i];      
    k=0;
    for (; k < 8; k++)
    r[k] = 0;
  }
  k=0;
  for (; k < 8; k++)
    ow->lastROM[k] = 0x00;
  ow->lastDiscrepancy = 64;
}

//Обработчик прерыания USART
void owReadHandler() {
  uint8_t index = getUsartIndex();
  /* Проверяем, что мы вызвали прерывание из-за RXNE. */
  if (((OW_USART->CR1 & USART_CR1_RXNEIE) != 0) &&
      ((OW_USART->SR & USART_FLAG_RXNE) != (u16)RESET)) {

    /* Получаем данные из периферии и сбрасываем флаг*/
		while ((OW_USART->SR & USART_FLAG_RXNE) == (u16)RESET){;}		 
    rc_buffer[index] = USART_ReceiveData(OW_USART);              
    recvFlag &= ~(1 << index);//сбрасываем флаг ответ получен после 
  }
}

/** Реализация RESET сброса на шине 1wire
 *
 * @return Возвращает 1 если на шине кто-то есть и 0 в противном случае
 */

uint16_t owResetCmd() {
	uint16_t owPresence;
	
	usart_setup_(9600); //передаем только номер USART и скорость

  owSend(0xF0); // Send RESET отправляем импуль сброса
  owPresence = owEchoRead(); // Ждём PRESENCE импульс присутствия на шине и возвращаем, что есть

	usart_setup_(115200);// перенастраиваем скорость UART
  return owPresence;
}

uint8_t getUsartIndex() {// смотрит по номеру UART c каким будет идти работа
	uint8_t result;
	if(OW_USART==USART1)result = 0;
	else if (OW_USART==USART2)result = 1;
	else if (OW_USART==USART3)result = 2;
	return result;
}

//Функция отправки данных по UART
void owSend(uint16_t data) {
  recvFlag |= (1 << getUsartIndex());//устанавливаем флаг если попадем в обработчик прерывания там он сбросится
  USART_SendData(OW_USART, data);//отправляем данные
	while(USART_GetFlagStatus(OW_USART, USART_FLAG_TC) == RESET);//ждем пока передача закончится
}

uint8_t owReadSlot(uint16_t data) {//читаем у нас пришла единица или ноль в ответ
  return (data == OW_READ) ? 1 : 0; //если пришло 0xFF, то бит = 1, что то другое бит = 0
}

uint16_t owEchoRead() {
  uint8_t i = getUsartIndex();//получаем номер USART
  uint16_t pause = 1000;
  while (recvFlag & (1 << i) && pause--);// ждем пока кто-то не ответит но не больше паузы
  return rc_buffer[i];//в зависимости от используемого номера UART 
}

uint8_t *byteToBits(uint8_t ow_byte, uint8_t *bits) {//разлагаем 1 байт на 8 бит ,кодируем так скасказать в посылку для 1wire
  uint8_t i;
  for (i = 0; i < 8; i++) {
    if (ow_byte & 0x01) {//если текущий бит в байте ==1 то
      *bits = WIRE_1; //заменяем на число которое при передаче по USART для 1wire будет единцией t.e 0xFF
    } else {
      *bits = WIRE_0;// тоже самое только для 0
    }
    bits++;
    ow_byte = ow_byte >> 1; //сдвигаем обработанный бит
  }
  return bits; //возвращае массив для передачи 
}

/**
 * Метод пересылает последовательно 8 байт по одному на каждый бит в data
 * @param usart -- выбранный для эмуляции 1wire UART
 * @param d -- данные
 */
void owSendByte(uint8_t d) {
  uint8_t data[8];
	int i;
  byteToBits(d, data);//преобразовываем байт в биты "массив байт для  передачи UART и эмуляции 1WIRE"
  for (i = 0; i < 8; ++i) {
    owSend(data[i]);
  }
}


uint8_t bitsToByte(uint8_t *bits) {//принимает "кодированый" массив бит полученный по UART и делает из него байт
  uint8_t target_byte, i;
  target_byte = 0;
  for (i = 0; i < 8; i++) {
    target_byte = target_byte >> 1;
    if (*bits == WIRE_1) {//если пришло  по USART 0xFF то это у нас пришла 1ца
      target_byte |= 0x80;//устанавливаем в 1 старший бит
    }
    bits++;//передвигаемся к следующему биту который является либо 0=0x00 или 1=0xFF
  }
  return target_byte; //возвращаем полученный байт
}

/* Подсчет CRC8 массива mas длиной Len */
uint8_t owCRC(uint8_t *mas, uint8_t Len) {
  uint8_t i, dat, crc, fb, st_byt;
  st_byt = 0;
  crc = 0;
  do {
    dat = mas[st_byt];
    for (i = 0; i < 8; i++) {  //Счетчик битов в байте
      fb = crc ^ dat;
      fb &= 1;
      crc >>= 1;
      dat >>= 1;
      if (fb == 1) crc ^= 0x8c; //Полином
    }
    st_byt++;
  } while (st_byt < Len); //Счетчик байтов в массиве
  return crc;
}

uint8_t owCRC8(RomCode *rom){
  return owCRC((uint8_t*)rom, 7);                        
}

/*
 * return 1 if has got one more address
 * return 0 if hasn't
 * return -1 if error reading happened
 *
 * переделать на функции обратного вызова для реакции на ошибки
 */
int hasNextRom(OneWire *ow, uint8_t *ROM) {//
	uint8_t ui32BitNumber = 0;
  int zeroFork = -1;
	uint8_t i = 0;
  if (owResetCmd() == ONEWIRE_NOBODY) { //есть ли кто на шине
    return 0;
  }
  owSendByte(ONEWIRE_SEARCH);//
  do {
		uint8_t answerBit =0;
    int byteNum = ui32BitNumber / 8;
    uint8_t *current = (ROM) + byteNum;
    uint8_t cB, cmp_cB, searchDirection = 0;
    owSend(OW_READ); // чтение прямого бита
    cB = owReadSlot(owEchoRead());//ответ от датчика
    owSend(OW_READ); // чтение инверсного бита
    cmp_cB = owReadSlot(owEchoRead());//ответ от датчика
    if (cB == cmp_cB && cB == 1)//сравниваем два ответа
      return -1;//ошибка никто не ответил 
    if (cB != cmp_cB) { //нормальная ситуация пришло либо 10 либо 01
      searchDirection = cB;//выбираем в каком направлении будем двигатся дальше
			} else {//колизия пришло 00 т.е текущий бит у ROM-ов разный
				if (ui32BitNumber == ow->lastDiscrepancy)//если текущая позиция колизии равна прошлой
        searchDirection = 1;//выбираем в каком направлении будем двигатся дальше
      else {
        if (ui32BitNumber > ow->lastDiscrepancy) {//если мы зашили дальше
          searchDirection = 0;//выбираем в каком направлении будем двигатся дальше 
        } else {
          searchDirection = (uint8_t) ((ow->lastROM[byteNum] >> ui32BitNumber % 8) & 0x01);
        }
        if (searchDirection == 0)
          zeroFork = ui32BitNumber;//запоминаем развилку
      }
    }
    // сохраняем бит
    if (searchDirection)
      *(current) |= 1 << ui32BitNumber % 8;//выставляем бит в текущем байте байте
    answerBit = (uint8_t) ((searchDirection == 0) ? WIRE_0 : WIRE_1);// решаем кого отключить
    owSend(answerBit);//вырубаем "мешающие" устройсва
    ui32BitNumber++;//ищем следующий бит
		} while (ui32BitNumber < 64);//пока не найден весь ROM все биты
  ow->lastDiscrepancy = zeroFork;//запоминаем развилку
  for (; i < 7; i++)
    ow->lastROM[i] = ROM[i];//запоминаем последний ROM
  return ow->lastDiscrepancy > 0;
}

// Возвращает количество устройств на шине или код ошибки, если значение меньше 0
int owSearchCmd(OneWire *ow) {
  int device = 0, nextROM;
  owInit(ow);
  do {
    nextROM = hasNextRom(ow, (uint8_t*)(&ow->ids[device])); //передаем указатель на структуру куда положить след.ROM
    if (nextROM<0)
      return -1;
    device++;
		} while (nextROM && device < MAXDEVICES_ON_THE_BUS);//ищем пока кто-то есть и этих кто-то не больше дефайна
		return device;//возвращаем порядковый номер датчика (устройства) на шине
}

void owSkipRomCmd(OneWire *ow) {//отправляет команду пропуска ROM после этого следующая команда будет
  owResetCmd();                 //для всех устройств на шине
  owSendByte(ONEWIRE_SKIP_ROM);
}

void owMatchRomCmd(RomCode *rom) {//позволяет мастеру обращаться к конкретному  ведомому устройству
	int i = 0;
  owResetCmd(); //Возвращает 1 если на шине кто-то есть и 0 в противном случае
  owSendByte(ONEWIRE_MATCH_ROM);//обращаемся к конкретному устройсву
  for (; i < 8; i++)
	owSendByte(*(((uint8_t *) rom) + i));//"перебираемся по структуре как по массиву" первой звездочкой получаем i тый байт из структуры
}

void owConvertTemperatureCmd(OneWire *ow, RomCode *rom) {
  owMatchRomCmd(rom);//позволяет мастеру обращаться к конкретному  ведомому устройству
  owSendByte(ONEWIRE_CONVERT_TEMPERATURE);//говорим датчику пора бы преобразовать температуру
}

/**
 * Method for reading scratchad DS18B20 OR DS18S20
 * If sensor DS18B20 then data MUST be at least 9 byte
 * If sensor DS18S20 then data MUST be at least 2 byte
 * @param ow -- OneWire pointer
 * @param rom -- selected device on the bus
 * @param data -- buffer for data
 * @return data
 */
//читаем оперативную память датчика "9 байт" SRAM
uint8_t *owReadScratchpadCmd(OneWire *ow, RomCode *rom, uint8_t *data) {
  uint16_t b = 0, p;
  switch (rom->family) {
    case DS18B20:
			p = 72;
		  break;
    case DS18S20:
      p = 72;  //9*8 =72 == равняется 9 байт данных
      break;
    default:
      return data;

  }
	//обращаемся к конкретному устройству
  owMatchRomCmd(rom);
  owSendByte(ONEWIRE_READ_SCRATCHPAD);//отправляем команду на чтение памяти
  while (b < p) {// пока мы не обработали 9 байт (побитно) 
    uint8_t pos = (uint8_t) ((p - 8) / 8 - (b / 8)); //позиция обрабатываемого бита
    uint8_t bt; 
		owSend(OW_READ); //Отправляем импулсь
    bt = owReadSlot(owEchoRead());//Читаем данные 
    if (bt == 1)
      data[pos] |= 1 << b % 8;//высчитываем и выставляем бит в нужной позиции
    else
      data[pos] &= ~(1 << b % 8);//сбрасываем бит в нужной позиции
    b++;//следующий бит
  }
  return data;
}

void owWriteDS18B20Scratchpad(OneWire *ow, RomCode *rom, uint8_t th, uint8_t tl, uint8_t conf) {
  if (rom->family != DS18B20)
    return;
  owMatchRomCmd(rom);//обращаемся к конкретному устройству
  owSendByte(ONEWIRE_WRITE_SCRATCHPAD);//будем записывать в память
  owSendByte(th);//пороги для температур
  owSendByte(tl);
  owSendByte(conf);
}

/**
 * Get last mesaured temperature from DS18B20 or DS18S20. These temperature MUST be measured in previous
 * opearions. If you want to measure new value you can set reSense in true. In this case next invocation
 * that method will return value calculated in that step.
 * @param ow -- OneWire bus pointer
 * @param rom -- selected device
 * @param reSense -- do you want resense temp for next time?
 * @return struct with data
 */
Temperature readTemperature(OneWire *ow, RomCode *rom, uint8_t reSense) {
	Scratchpad_DS18B20 *sp;
	Scratchpad_DS18S20 *spP;
  Temperature t;
	uint8_t pad[9];
  t.inCelsus = 0x00;
  t.frac = 0x00;
  sp = (Scratchpad_DS18B20 *) &pad; 
  spP = (Scratchpad_DS18S20 *) &pad;
  switch (rom->family) {
    case DS18B20:
      owReadScratchpadCmd(ow, rom, pad);//читаем память  для DS18B20
      t.inCelsus = (int8_t) (sp->temp_msb << 4) | (sp->temp_lsb >> 4);//целая часть
      t.frac = (uint8_t) ((((sp->temp_lsb & 0x0F)) * 10) >> 4);//дробная
      break;
    case DS18S20:
      owReadScratchpadCmd(ow, rom, pad);//читаем память  для DS18S20
      t.inCelsus = spP->temp_lsb >> 1;
      t.frac = (uint8_t) 5 * (spP->temp_lsb & 0x01);
      break;
    default:
      return t;
  }
  if (reSense) {
    owConvertTemperatureCmd(ow, rom);//можно сразу после как забрали данные отдаем датчику команду на преобразования температуры
  }
  return t;
}

void owCopyScratchpadCmd(OneWire *ow, RomCode *rom) {
  owMatchRomCmd(rom);
  owSendByte(ONEWIRE_COPY_SCRATCHPAD);
}

void owRecallE2Cmd(OneWire *ow, RomCode *rom) {
  owMatchRomCmd(rom);
  owSendByte(ONEWIRE_RECALL_E2);
}
