#include "misc.h"
#include "string.h"
#include <math.h>
#include <stdlib.h>

//#include "main.h" 

#define		 CS1_LOW 	GPIOA->BSRR = GPIO_BSRR_BR4;
#define 	 CS1_HIGH 	GPIOA->BSRR = GPIO_BSRR_BS4;
#define		 CS2_LOW 	GPIOA->BSRR = GPIO_BSRR_BR3;
#define 	 CS2_HIGH 	GPIOA->BSRR = GPIO_BSRR_BS3;
#define		 CS3_LOW 	GPIOB->BSRR = GPIO_BSRR_BR12;
#define 	 CS3_HIGH 	GPIOB->BSRR = GPIO_BSRR_BS12;
#define		 Speed_Flag_SET 	 GPIOC->BSRR = GPIO_BSRR_BS13; //для оценки времени задержки фильтра
#define 	 Speed_Flag_RESET  GPIOC->BSRR = GPIO_BSRR_BR13; 

#define RX_BUF_SIZE 80
volatile char RX_FLAG_END_LINE = 0;
volatile char RXi;
volatile char RXc;
volatile char RX_BUF[RX_BUF_SIZE] = {'\0'};
volatile char buffer[150] = {'\0'}; 
volatile char NumRX[20] = {'\0'}; 

volatile int Iref, enableStart=1, counter_Err_NonconnectCS1, counter_Err_NonconnectCS2, enable, TX_Enable=1, channelspi=0, adc_out1, adc_out2, ADCmean_In=2042, Calibration=0, InitCalibration=1;
volatile int8_t I_ch1, I_ch2;
volatile int16_t Set_point_Current_Channel=0;
volatile float ki=0.01, Ireal_load=0, e_curr=0, Delta=0;
volatile int *D;

GPIO_InitTypeDef myGPIO;
USART_InitTypeDef myUSART;
NVIC_InitTypeDef myNVIC;
SPI_InitTypeDef mySPI;

void rcc_ini(void); //Вкл тактированния ножки порта LED
void port_ini(void); // инициализация LED
void SysClock_ini(void);//настройка тактовой чатоты
void usart_ini(void);//инициализация UART
void spi_ini(void);//инициализация spi
void USART1_IRQHandler(void);//обработка прерывания по появлкнию данных в data buffer
void USARTSend(const unsigned char *pucBuffer, unsigned long ulCount); //отправка строки в терминал
void clear_RXBuffer(void); // отчистка буффера принятых с ПК данных
int  Spi1_Read_Send_Data(int data, int channel); //прием данных по spi1
int  Spi2_Read_Send_Data(int data, int channel); //прием данных по spi2
void timer_ini(void);//инициализация таймера 3 для работы регулятора по прерываниям //TIM3
void PWM_ini(void); //инициализация таймера для работы в режиме PWM                 //TIM2
	
int main(void)
{
	  int counterUART=0, count_speed=0, i = 0, ch=1, value=-1;
	  	
    rcc_ini();
	  port_ini();
	  SysClock_ini();
    usart_ini(); // иницилизация UART
	  spi_ini(); // иницилизация UART
	  PWM_ini();
	  timer_ini();
	
		counter_Err_NonconnectCS1 = 0;
	  counter_Err_NonconnectCS2 = 0;
    Iref = 2; //ЭТАЛОННОЕ ЗНАЧЕНИЕ ТОКА
	  enable = 1;
    USARTSend(&("\r\nHello, i am main!\r\n"), sizeof("\r\nHello, i am main!\r\n"));
	  TIM3->CR1 |= TIM_CR1_CEN; // Начать отсчёт!
    while(1)
    { 
				/*Speed_Flag_RESET
				if(count_speed==1){
						Speed_Flag_SET
						count_speed=0;
				}		
			  count_speed=1;*/
						 if(TX_Enable==1) counterUART++;
					
						 if(counterUART==100000 && TX_Enable==1){
                 if(enableStart==1){ //при пуске
										 USARTSend(buffer, sizeof(buffer));
										 counterUART=0;								 
										 sprintf(buffer, "\r\nIJBT ON\r\nIref=%d ki=%0.2f I_load =%0.1f I_ch1=%d I_ch2=%d e=%0.1f S_P=%d\r\n", Iref, ki, Ireal_load, I_ch1, I_ch2, e_curr, Set_point_Current_Channel);//(*D/655)
								 }
								 if(enableStart==0){ //при стопе
										 USARTSend(buffer, sizeof(buffer));
										 counterUART=0;								 
										 sprintf(buffer, "\r\nIJBT are disabled\r\nIref=%d ki=%0.2f I_load=%0.1f I_ch1=%d I_ch2=%d e=%0.1f\r\n", Iref, ki, Ireal_load, I_ch1, I_ch2, e_curr);//(*D/655)
								 }
						}
						 //ОБРАБОТКА ПРИНЯТЫХ С ПК ДАННЫХ
						 if (RX_FLAG_END_LINE == 1) {
								//Reset END_LINE Flag
								 RX_FLAG_END_LINE = 0;								
										
								 if(strncmp(RX_BUF, "ki=\r", 3)==0){
										 USARTSend("\r\nTHIS IS A COMMAND \ki=!!!\r\n",sizeof("\r\nTHIS IS A COMMAND \ki=!!!\r\n"));
										 i=3;
										 while(RX_BUF[i]!=13){
												 NumRX[i-3]=RX_BUF[i];
												 i++;
										 }
										 if(atof(NumRX)!=0) ki = atof(NumRX);
										 Delta=round((Set_point_Current_Channel)/ki);
										 sprintf(buffer, "\n\r ki=%0.3f \r\n", ki);
										 USARTSend(buffer, sizeof(buffer));
								 }	
								 if(strncmp(RX_BUF, "Iref=\r", 5)==0){
										 USARTSend("\r\nTHIS IS A COMMAND \Iref=!!!\r\n",sizeof("\r\nTHIS IS A COMMAND \Iref=!!!\r\n"));
										 i=5;
										 while(RX_BUF[i]!=13){
												 NumRX[i-5]=RX_BUF[i];
												 i++;
										 }
										 Iref=atoi(NumRX);		
										 sprintf(buffer, "\n\r Iref=%d \r\n", Iref);
										 USARTSend(buffer, sizeof(buffer));
								 }
								 if(strncmp(RX_BUF, "csc\r", 3)==0){
										USARTSend("\r\nSensors Calibration!!!\r\n",sizeof("\r\nSensors Calibration!!!\r\n"));
										Calibration=1;
								 }
								if(strncmp(RX_BUF, "d\r", 1)==0){
										USARTSend("\r\nSTOP!!!\r\n",sizeof("\r\nSTOP!!!\r\n"));
									enableStart=0;				 
								 }		 
								if(strncmp(RX_BUF, "a\r", 1)==0){
										USARTSend("\r\nSTART!!!\r\n",sizeof("\r\nSTART!!!\r\n"));
										Set_point_Current_Channel=0;
										Delta=0;
										enableStart=1;							 
								 }
								 clear_RXBuffer();
						 }
    }
    
}

void TIM3_IRQHandler(void)
{
	  int data1, data2, data_last, count_last, ch1=1, General=0, i, j;
	
		//Калибровка датчиков тока-----------------------------------------
	  if(InitCalibration==1){//Вызов калибровки датчиков при перезагрузке системы
				Calibration=1;
				InitCalibration=0;
			  Spi1_Read_Send_Data('0', ch1); // первый пуск в холостую, т.к приходит не правильное число
		}
		if(Calibration==1){
				Spi2_Read_Send_Data(-1, ch1);//команда на калиброку модулей каналов тока
				ADCmean_In=0;
				for(j=0; j<100; j++){
					  for(i=0;i<1000;i++)	{//задержка считывания, для того, чтоб данные отличались
						}
						ADCmean_In=ADCmean_In+Spi1_Read_Send_Data('0', ch1);
				}	
				ADCmean_In=ADCmean_In/100;
				Calibration=0; 
				USARTSend("\r\nSensors Calibration Complite!!!\r\n",sizeof("\r\nSensors Calibration Complite!!!\r\n"));
		}
		//-----------------------------------------------------------------		
		if(enable){	
		// 1  канал	 ---------------------------------------------------------------------------
		data1=Spi1_Read_Send_Data('0', ch1); //считываем значение тока нагрузки	
			
		//контроль подключения датчика тока нагрузки--------------------------
 	  if( (data1==ADCmean_In)||(data1==(ADCmean_In-1))||(data1==(ADCmean_In+1)) ){
				counter_Err_NonconnectCS1++;
				if(counter_Err_NonconnectCS1==1000000){	// T(сек) = (Переменная условия * 6.5*(10^-6) //154 это 1 милисекунда //1000000 это 6,5 секунд
						USARTSend(&(" May be non connect Current Sensor\r\n"), sizeof(" May be non connect Current Sensor\r\n"));
						counter_Err_NonconnectCS1=0;
				}
	  }
	  //-------------------------------------------------------------------
		
	  adc_out1=data1;
	  if( (adc_out1==ADCmean_In+1)||(adc_out1==ADCmean_In-1)){ /////приводим ток к Амперам 
				Ireal_load=0;
	  }
	  else{
				Ireal_load=((2000*(adc_out1-ADCmean_In)/30583.33333));/////приводим ток к Амперам 
	  }
		if(enableStart==1){ //при пуске
				e_curr = Iref-Ireal_load; //ошибка по току
				Delta=Delta+e_curr;
				if(ki*Delta>30000) Delta=(30000/ki)-1; //верхний предел Set_point_Current_Channel
				if(ki*Delta<0) Delta=0; //нижний предел Set_point_Current_Channel
			Set_point_Current_Channel=round(ki*Delta);// Уставка для каналов тока
		}
		if(enableStart==0){ //при стопе
				Set_point_Current_Channel=-30000;// D коэффициент заполнения //Dmax=65535 т.е ключ всегда включен Dmin=0  т.е ключ всегда выключен Минимальная скважность при пропускании тока 328
		}
		data2=Spi2_Read_Send_Data(Set_point_Current_Channel, ch1); //отправляем уставку, считываем токи модуля двух каналов 
		//Получаем из 16bit посылки 8bit токи
		I_ch1=0;
		I_ch2=0;
		for(i=0;i<=15;i++) {
				if(i<=7) {
						if((data2&(1<<i))!=0){
								I_ch2|=(1<<i);
						}
				}
				if(i>7) {
						if((data2&(1<<i))!=0){
								I_ch1|=(1<<(i-8));
						}
				}
		}
		// ---------------------------------------------------------------------------------------
		
		//контроль отрицательного тока 1 канал--------------------------
	/*if(Ireal_ch1<-1){
				TIM2->CCMR1 |= (1<<4)|(1<<6); //1.1Включаем на первом канале высокий уровень
				TIM2->CCMR1 &=~(1<<5);        //1.2Включаем на первом канале высокий уровень
				enable=0;
			sprintf(buffer, "\r\nОШИБКА!!! Отрицательный ток датчика 1 канал!Ключи отключены! Ток=%.1f\r\n ", Ireal_ch1);
			USARTSend(buffer, sizeof(buffer));
	  }
		//--------------------------------------------------------------
	*/	
		//контроль отрицательного тока 1 канал--------------------------
	  /*if(Ireal_ch2<-1){
				TIM2->CCMR1 |= (1<<4)|(1<<6); //1.1Включаем на первом канале высокий уровень
				TIM2->CCMR1 &=~(1<<5);        //1.2Включаем на первом канале высокий уровень
				enable=0;
			sprintf(buffer, "\r\nОШИБКА!!! Отрицательный ток датчика 2 канал!Ключи отключены! Ток=%.1f\r\n ", Ireal_ch1);
			USARTSend(buffer, sizeof(buffer));
	  }*/
		//--------------------------------------------------------------
		
	  
/*
	  //контроль работы системы управления по показаниям датчика тока и процентов открытия ключа--------------------------
	  D_one=*D		
		if(count_last==0) I_last = round(Ireal); 
		
	  count_I_last++;
	  if(count_last==2) count_I_last=0;

	  if( (Ireal==I_last)||(data==(data_last+1))||(data==(data_last-1)) ){
				counter_Err_NonconnectCS++;
				if(counter_Err_NonconnectCS==154){ // T(сек) = (Переменная условия * 6.5*(10^-6) //154 это 1 милисекунда //1000000 это 6,5 секунд
						USARTSend(&(" Non connect Current Sensor\r\n"), sizeof(" Non connect Current Sensor\r\n"));
						counter_Err_NonconnectCS=0;
				}
	  }
	 //контроль работы датчика тока при работе системы управления--------------------------
	 */
	  }
		 TIM3->SR &= ~TIM_SR_UIF;//Сбрасываем флаг UIF
}

int Spi1_Read_Send_Data(int data, int channel)
 { 
  //ждём пока опустошится Tx буфер
	while(!(SPI1->SR & SPI_SR_TXE));
	 
	//активируем Chip Select
	if(channel==1){
		 CS1_LOW  
	}
	if(channel==2){
		 CS2_LOW  
	}
	
	//отправляем данные     
	SPI1->DR = data; 
	
  //ждём пока передатчик освободится
	while((SPI1->SR & SPI_SR_BSY))
  {
	}		
		
	//ждём пока придёт ответ
	while(!(SPI1->SR & SPI_SR_RXNE)) // status register: RXNE: Receive buffer not empty
  {
	}
	
	//считываем полученные данные
	data = SPI1->DR;  
  //деактивируем Chip Select
	if(channel==1){
		 CS1_HIGH  
	}
	if(channel==2){
		 CS2_HIGH  
	}
  //возвращаем то, что прочитали
	//ждём пока передатчик освободится
	while((SPI1->SR & SPI_SR_BSY))
	{
	}
		
  return data;  
 }
 
 int Spi2_Read_Send_Data(int data, int channel)
 { 
  //ждём пока опустошится Tx буфер
	while(!(SPI2->SR & SPI_SR_TXE));
	 
	//активируем Chip Select
	if(channel==1){
		 CS3_LOW  
	}
/*	if(channel==2){
		 CS2_LOW  
	}*/
	if(channel==0){
		 CS3_LOW
//		 CS2_LOW  
	}
	
	//отправляем данные     
	SPI2->DR = data; 
	
  //ждём пока передатчик освободится
	while((SPI2->SR & SPI_SR_BSY))
  {
	}		
		
	//ждём пока придёт ответ
	while(!(SPI2->SR & SPI_SR_RXNE)) // status register: RXNE: Receive buffer not empty
  {
	}
	//считываем полученные данные
	data = SPI2->DR;  
  //деактивируем Chip Select
	if(channel==1){
		 CS3_HIGH  
	}
/*	if(channel==2){
		 CS2_HIGH  
	}*/
	if(channel==0){
		 CS3_HIGH
//		 CS2_HIGH  
	}
  //возвращаем то, что прочитали
	//ждём пока передатчик освободится
	while((SPI2->SR & SPI_SR_BSY))
	{
	}
		
  return data;  
 }
 
void rcc_ini(void)
{
	RCC->APB2ENR |= (1<<4); //Активация тактирования GPIOC в enable register
}

void port_ini(void)
{
	  //LED
	  myGPIO.GPIO_Pin = GPIO_Pin_13; // инициализция параметров ножки порта прводится в регистре CRH  configuration register high
    myGPIO.GPIO_Mode = GPIO_Mode_Out_PP;
    myGPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &myGPIO);
}

void SysClock_ini(void)
{
	  ErrorStatus HSEStartUpStatus;
 
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
    /* Системный RESET RCC (делать не обязательно, но полезно на этапе отладки) */
    RCC_DeInit();
 
    /* Включаем HSE (внешний кварц) */
    RCC_HSEConfig( RCC_HSE_ON);
 
    /* Ждем пока HSE будет готов */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
 
    /* Если с HSE все в порядке */
    if (HSEStartUpStatus == SUCCESS)
    {
    /* Следующие две команды касаются исключительно работы с FLASH.
    Если вы не собираетесь использовать в своей программе функций работы с Flash,
    FLASH_PrefetchBufferCmd( ) та FLASH_SetLatency( ) можно закомментировать */
			
        /* Включаем Prefetch Buffer */
//        FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);
 
        /* FLASH Latency.
    Рекомендовано устанавливать:
        FLASH_Latency_0 - 0 < SYSCLK≤ 24 MHz
        FLASH_Latency_1 - 24 MHz < SYSCLK ≤ 48 MHz
        FLASH_Latency_2 - 48 MHz < SYSCLK ≤ 72 MHz */
//        FLASH_SetLatency( FLASH_Latency_2);
        
        /* HCLK = SYSCLK */ /* Смотри на схеме AHB Prescaler. Частота не делится (RCC_SYSCLK_Div1) */                   //Предделители находятся в Clock configuration register (RCC_CFGR)
        RCC_HCLKConfig( RCC_SYSCLK_Div1);
 
        /* PCLK2 = HCLK */ /* Смотри на схеме APB2 Prescaler. Частота не делится (RCC_HCLK_Div1)  */
        RCC_PCLK2Config( RCC_HCLK_Div1);
 
        /* PCLK1 = HCLK/2 */ /* Смотри на схеме APB1 Prescaler. Частота делится на 2 (RCC_HCLK_Div2)
        потому что на выходе APB1 должно быть не более 36МГц (смотри схему) */
        RCC_PCLK1Config( RCC_HCLK_Div2);
 
        /* PLLCLK = 8MHz * 9 = 72 MHz */
        /* Указываем PLL от куда брать частоту (RCC_PLLSource_HSE_Div1) и на сколько ее умножать (RCC_PLLMul_9) */
        /* PLL может брать частоту с кварца как есть (RCC_PLLSource_HSE_Div1) или поделенную на 2 (RCC_PLLSource_HSE_Div2). Смотри схему */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
 
        /* Включаем PLL */
        RCC_PLLCmd( ENABLE);
 
        /* Ждем пока PLL будет готов */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }
 
        /* Переключаем системное тактирование на PLL */
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);
 
        /* Ждем пока переключиться */
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    { /* Проблемы с HSE. Тут можно написать свой код, если надо что-то делать когда микроконтроллер не смог перейти на работу с внешним кварцом */
        /* Пока тут заглушка - вечный цикл*/
        while (1)
        {
        }
    }
}
	void usart_ini(void)
{
		/* Set NVIC Priority Group */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //выбор группы приоритетов 

    /* Enable the USARTx Interrupt */
    myNVIC.NVIC_IRQChannel = USART1_IRQn; 
    myNVIC.NVIC_IRQChannelPreemptionPriority = 0; //Приоритет
    myNVIC.NVIC_IRQChannelSubPriority = 2; //Субприоритет
    myNVIC.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&myNVIC);
	
    RCC->APB2ENR |= (1<<14); //Активация тактирования UART C в enable register
		RCC->APB2ENR |= (1<<2); //Активация тактирования GPIOA в enable register 
	  RCC->APB2ENR |= (1); //Активация тактирования Альтернативных функций в enable register 
    myGPIO.GPIO_Pin = GPIO_Pin_9; //TX
    myGPIO.GPIO_Mode = GPIO_Mode_AF_PP;
    myGPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&myGPIO);
    myGPIO.GPIO_Pin = GPIO_Pin_10; //RX
    myGPIO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    myGPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&myGPIO);
    
    //инициализация USART 
    myUSART.USART_BaudRate = 115200;                                       //проходит в Control register 1 (USART_CR1/2/3)
    myUSART.USART_HardwareFlowControl = USART_HardwareFlowControl_None ;
    myUSART.USART_Mode = USART_Mode_Rx |USART_Mode_Tx;
    myUSART.USART_Parity = USART_Parity_No; //бит четности
    myUSART.USART_StopBits = USART_StopBits_1; //количество стоп битов
    myUSART.USART_WordLength = USART_WordLength_8b; //количество бит в посылке
    USART_Init(USART1,&myUSART);
    
    /* Enable USART1 */
		USART1->CR1|=(1<<13);// USART_Cmd(USART1, ENABLE);//   Control register

 
    /* Enable the USART1 Receive interrupt: this interrupt is generated when the
        USART1 receive data register is not empty */
		USART1->CR1|=(1<<5);// USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  // проходит в Control register 1
}

void USART1_IRQHandler(void) //обработчик прерываний
{
    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET) //убеждаемся, что прилетел байт
    {
			  TX_Enable=0;
        RXc = USART_ReceiveData(USART1);
        RX_BUF[RXi] = RXc;
        RXi++;
 
        if (RXc != 13) { //символ с кодом 13 означает конец строки (НАЖАТИЕ КНОПКИ ENTER)
           if (RXi > RX_BUF_SIZE-1) {
               clear_RXBuffer();
           }
        }
        else {
           RX_FLAG_END_LINE = 1;
					 TX_Enable=1;
        }
            //Echkko
            USART_SendData(USART1, RXc);
	  					
    }
}
	void USARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
    while (ulCount--)
    {
        USART_SendData(USART1, *pucBuffer++);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) //проверяет Transmission Complete flag
        {
        }
    }
}

void clear_RXBuffer(void) {
    for (RXi=0; RXi<RX_BUF_SIZE; RXi++) RX_BUF[RXi] = '\0';
    RXi = 0;
		for (RXi=0; RXi<sizeof(NumRX); RXi++) NumRX[RXi] = '\0';
    RXi = 0;
}

void spi_ini(void)
{
		
	//SPI1----------------------------------------------------------------------------------
	//Вкл. Тактирования	
		RCC->APB2ENR |= (1<<12); //Активация тактирования SPI  в enable register
		RCC->APB2ENR |= (1<<2); //Активация тактирования GPIOA в enable register 
	  RCC->APB2ENR |= (1); //Активация тактирования Альтернативных функций в enable register 
	
	//настройк ножек  SPI в Control Registr H
    myGPIO.GPIO_Pin = GPIO_Pin_5; //SCLK
    myGPIO.GPIO_Mode = GPIO_Mode_AF_PP;
    myGPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&myGPIO);
	
		myGPIO.GPIO_Pin = GPIO_Pin_7; //MOSI
    myGPIO.GPIO_Mode = GPIO_Mode_AF_PP;
    myGPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&myGPIO);
	
	  myGPIO.GPIO_Pin = GPIO_Pin_4; //SS1
    myGPIO.GPIO_Mode = GPIO_Mode_Out_PP;
    myGPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&myGPIO);
    
		myGPIO.GPIO_Pin = GPIO_Pin_3; //SS2
    myGPIO.GPIO_Mode = GPIO_Mode_Out_PP;
    myGPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&myGPIO);
		
    myGPIO.GPIO_Pin = GPIO_Pin_6; //MISO
    myGPIO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    myGPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&myGPIO);
	
		//Заполняем структуру с параметрами SPI модуля в регистре ControlReg1
		 mySPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //SPI_Direction_2Lines_FullDuplex полный дуплекс 
		 mySPI.SPI_DataSize = SPI_DataSize_16b; // передаем по 16 бит
		 mySPI.SPI_CPOL = SPI_CPOL_High ; // Полярность и
	 	 mySPI.SPI_CPHA = SPI_CPHA_1Edge; // фаза тактового сигнала
		 mySPI.SPI_NSS = SPI_NSS_Soft; // Управлять состоянием сигнала NSS программно
	   mySPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // Предделитель SCK
		 mySPI.SPI_FirstBit = SPI_FirstBit_MSB; // Первым отправляется старший бит
		 mySPI.SPI_Mode = SPI_Mode_Master; // Режим - мастер
		 SPI_Init(SPI1, &mySPI); //Настраиваем SPI1
	 	 SPI_Cmd(SPI1, ENABLE); // Включаем модуль SPI1....//   Control register
		// Поскольку сигнал NSS контролируется программно, установим его в единицу
		// Если сбросить его в ноль, то наш SPI модуль подумает, что
		// у нас мультимастерная топология и его лишили полномочий мастера.
		 SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);//установим его в 1
  //--------------------------------------------------------------------------------------
	//SPI2----------------------------------------------------------------------------------
	//Вкл. Тактирования	
		RCC->APB1ENR |= (1<<14); //Активация тактирования SPI  в enable register
		RCC->APB2ENR |= (1<<3); //Активация тактирования GPIOB в enable register 
	  RCC->APB2ENR |= (1); //Активация тактирования Альтернативных функций в enable register 
	
	//настройк ножек  SPI в Control Registr H
    myGPIO.GPIO_Pin = GPIO_Pin_13; //SCLK
    myGPIO.GPIO_Mode = GPIO_Mode_AF_PP;
    myGPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&myGPIO);
	
		myGPIO.GPIO_Pin = GPIO_Pin_15; //MOSI
    myGPIO.GPIO_Mode = GPIO_Mode_AF_PP;
    myGPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&myGPIO);
	
	  myGPIO.GPIO_Pin = GPIO_Pin_12; //SS1
    myGPIO.GPIO_Mode = GPIO_Mode_Out_PP;
    myGPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&myGPIO);
    
	/*myGPIO.GPIO_Pin = GPIO_Pin_3; //SS2
    myGPIO.GPIO_Mode = GPIO_Mode_Out_PP;
    myGPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&myGPIO);
	*/	
    myGPIO.GPIO_Pin = GPIO_Pin_14; //MISO
    myGPIO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    myGPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&myGPIO);
	
		//Заполняем структуру с параметрами SPI модуля в регистре ControlReg1
		 mySPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //SPI_Direction_2Lines_FullDuplex полный дуплекс 
		 mySPI.SPI_DataSize = SPI_DataSize_16b; // передаем по 16 бит
		 mySPI.SPI_CPOL = SPI_CPOL_High ; // Полярность и
	 	 mySPI.SPI_CPHA = SPI_CPHA_1Edge; // фаза тактового сигнала
		 mySPI.SPI_NSS = SPI_NSS_Soft; // Управлять состоянием сигнала NSS программно
	   mySPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // Предделитель SCK
		 mySPI.SPI_FirstBit = SPI_FirstBit_MSB; // Первым отправляется старший бит
		 mySPI.SPI_Mode = SPI_Mode_Master; // Режим - мастер
		 SPI_Init(SPI2, &mySPI); //Настраиваем SPI1
	 	 SPI_Cmd(SPI2, ENABLE); // Включаем модуль SPI1....//   Control register
		// Поскольку сигнал NSS контролируется программно, установим его в единицу
		// Если сбросить его в ноль, то наш SPI модуль подумает, что
		// у нас мультимастерная топология и его лишили полномочий мастера.
		 SPI_NSSInternalSoftwareConfig(SPI2, SPI_NSSInternalSoft_Set);//установим его в 1
  //--------------------------------------------------------------------------------------
}

void timer_ini(void)
{ 
	
	//Вкл. Тактирования	
	  RCC->APB1ENR|=(1<<1); //Активация тактирования TIM3 в enable register
	
	  TIM3->PSC = 72 - 1; //1MHz     // (частота тактирования У нас 72000000)/(72) = Частота счёта таймера 
		TIM3->ARR = 100 ; ////10kHz     //Period предел счета таймера досчитав до 100 будет вызываться прерывание в нашем случае 
		TIM3->DIER |= TIM_DIER_UIE; //разрешаем прерывание от таймера
	  //TIM3->CR1 |= TIM_CR1_CEN; // Начать отсчёт!
 	    /* NVIC Configuration */
    /* Enable the TIM3_IRQn Interrupt */
    myNVIC.NVIC_IRQChannel = TIM3_IRQn;
    myNVIC.NVIC_IRQChannelPreemptionPriority = 0;
    myNVIC.NVIC_IRQChannelSubPriority = 0;
    myNVIC.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&myNVIC);	
	  
	//Вкл. Тактирования	
	  RCC->APB1ENR|=(1<<2); //Активация тактирования TIM4 в enable register
	
		TIM4->PSC = 7200 - 1; //10kHz     // 70000000/(72) = Частота счёта таймера
		TIM4->ARR = 10000 ; ////1Hz     //Period предел счета таймера досчитав до 10000 будет вызываться прерывание в нашем случае 
		TIM4->DIER |= TIM_DIER_UIE; //разрешаем прерывание от таймера
	  //TIM3->CR1 |= TIM_CR1_CEN; // Начать отсчёт!
 	    /* NVIC Configuration */
    /* Enable the TIM3_IRQn Interrupt */
    myNVIC.NVIC_IRQChannel = TIM4_IRQn;
    myNVIC.NVIC_IRQChannelPreemptionPriority = 0;
    myNVIC.NVIC_IRQChannelSubPriority = 0;
    myNVIC.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&myNVIC);	
}

void PWM_ini(void)
{
	//Вкл. Тактирования	
	RCC->APB2ENR|=(1<<2);//Активация тактирования GPIOA в enable register 
	RCC->APB1ENR|=1; //Активация тактирования TIM2 в enable register
	
	myGPIO.GPIO_Pin   = GPIO_Pin_0; //первый канал таймера
	myGPIO.GPIO_Mode  = GPIO_Mode_AF_PP;
	myGPIO.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,&myGPIO);
  TIM2->CCER  |=1; //Разрешаем таймеру использовать первый канал( ножка PA0) в регистре TIMx_CCER capture/compare enable register
	TIM2->CCMR1 |=(1<<4)|(1<<5)|(1<<6); //Включаем на первом канале обртный ШИМ    УПРАВЛЕНИЕ ДРЙВЕРАМИ НУЛЕМ
	//TIM2->CCMR1 &=~(1<<3); 
	TIM2->CR1|=(1); //Активация таймера
}
