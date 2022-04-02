#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#include <util/delay.h>
#include <avr/eeprom.h>
#include "SSD1806/SSD1306.h"
#include "SSD1806/Font5x8.h"

#define ENDERECO_HODOMETRO 0
#define ENDERECO_DIAMETRO_PNEU 4
#define ENDERECO_TEMPERATURA_MAX 6

//Aluno: Felipe Augusto Silva Nascimento
//Matricula: 118111140
//Turma: 02
//Sprint 10 - Novas Funcionalidades: Oscilador Controlado por Tensão (VCO) e Airbag


//DECLARAÇÃO DE VARIÁVEIS GLOBAIS
uint8_t flag_500ms = 0;
uint16_t tempo_ms = 0;
const float PI = 3.14161;
uint32_t vel_carro = 0, TEMPO_100us = 0, TEMPO_100us_anterior = 0, RPM = 0, delta_t = 0;
uint16_t diametro_pneu = 65;
uint32_t dist_km = 0;
uint32_t dist_anterior = 0;
int N_voltas_pneu = 0;
uint32_t tempo_borda_subida;
uint16_t distancia_sonar_cm;
uint16_t bateria = 0, temperatura = 0, temperatura_max = 0;
uint16_t forca = 0, forca_max = 0;
uint16_t acelerador = 0;
uint16_t flag_150ms = 0;
int disp_7seg = 0, cont = 0;

//Protótipo das funções
ISR(TIMER1_CAPT_vect);
ISR(TIMER0_COMPA_vect);
ISR(TIMER2_COMPA_vect);
ISR(INT0_vect);
ISR(PCINT2_vect);
ISR(USART_RX_vect);
void leitura_sensores_ADC(uint8_t *flag_disparo, uint16_t *temperatura_pont, uint16_t *bateria_pont, uint16_t *temp_max_pont, uint16_t *forca_pont);
void USART_Init(unsigned int ubrr);
void USART_Transmit(unsigned char data);
unsigned char USART_Receive(void);
void airbag (uint16_t *distancia_sonar_pont, uint32_t *vel_carro_pont, uint16_t *forca_pont);

int main(void)
{
	USART_Init(MYUBRR);

	// Configuração das portas
	//DDRC = 0b10000000;
	//PORTC = 0b10000000;
	DDRB = 0b11111110;	// Portas B como saidas para o display 7seg
	DDRD &= 0b11000011;	// Declarando PD4, PD5 e PD2 como entradas e PD3 como saída
	PORTD = 0b00111100;	// Desabilita o resistor de Pull-up do Pino PD3, PD4, PD5 e PD6


	EICRA = 0b00000010; // Borda de descida INT0
	EIMSK = 0b00000001; // Habilita a interrupcao externa INT0
	PCICR = 0b00000100; // Habilita a interrupção por mudança nos pinos do PORTD
	PCMSK2 = 0b00110000; // Interrupção habilitada no PCINT21(PD5) e PCINT20(PD4)

	EICRA |= 0b00000010;	//Dispara o INT0 na borda de descida

	//Configuração do TIMER T0
	TCCR0A = 0b00000010;	// Habilita modo CTC do TC0, TOP = OCR0A;
	TCCR0B = 0b00000010;	// Habilita o TC0 com prescaler = 8
	OCR0A = 199;			// Ajusta o comparador para TC0 contar ate 199.
	TIMSK0 = 0b00000010;	// Ativa a interrupção do TC0 na igualdade de comparação com o registrador OCR0A a cada 100us = (199 + 1)*8/16MHz

	//Configuração do ADC
	//ADMUX = 0b01000000; //VCC como referência e habilita a entrada analógica ADC5(PC5)
	ADCSRA = 0b11100111;//Habilita o AD, modo de conversão contínua, prescaler = 128
	ADCSRB = 0b00000000; //Modo de conversão contínua

	//Configuração do TIMER T2 ------> //FAST PWM, TOP = 0xFF e OC2B habilidados
	TCCR2A = 0b00100011; //PWM Rápido não invertido nos pinos OC2B
	TCCR2B = 0b00000100; //Liga TC2, prescaler = 64, fpwm = f0sc/(256*prescaler) = 16MHz/(256*64) = 976Hz
	OCR2B = 255; //controle do ciclo ativo do PWM 0C2B (PD3)
	
	//Configuração do TIMER T1
	TCCR1A = 0;
	TCCR1B = (1 << ICES1) | (1 << CS11) | (1 << CS10); //Captura na borda de subida, TC1 com prescaler = 64
	TIMSK1 = 1 << ICIE1; //Habilita a interrupção por captura

	sei();

	dist_km = eeprom_read_dword(ENDERECO_HODOMETRO); //Lê o valor do hodometro armazenado no EEPROM (posições de 0 à 3)
	diametro_pneu = eeprom_read_byte(ENDERECO_DIAMETRO_PNEU); //Lê o valor do diametro do pneu armazenado no EEPROM (posição 4)
	temperatura_max = eeprom_read_byte(ENDERECO_TEMPERATURA_MAX); //Lê o valor da temperatura máxima da bateria armazenada na EEPROM (posição 6)
	
	dist_anterior = dist_km * 100000;
	
	//Inicia o display SSD1306
	GLCD_Setup();
	GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
	GLCD_InvertScreen();


	while (1)
	{
		
		leitura_sensores_ADC(&flag_150ms, &temperatura, &bateria, &temperatura_max, &forca); //Chama a função que lê os valores ADC
		airbag (&distancia_sonar_cm, &vel_carro, &forca);
		
		GLCD_Clear();
		GLCD_GotoXY(0, 0);
		GLCD_PrintString("LASD Car");
		
		//Imprime o valor das rotações por minuto
		GLCD_GotoXY(0, 15);
		GLCD_PrintInteger(RPM);
		GLCD_GotoXY(30, 15);
		GLCD_PrintString("rpm");
		
		//Imprime o valor da distância sonar
		GLCD_GotoXY(0, 25);
		GLCD_PrintString("Sonar (cm): ");
		GLCD_GotoXY(65, 25);
		GLCD_PrintInteger(distancia_sonar_cm);
		
		//Imprime diametro do pneu
		GLCD_GotoXY(0, 35);
		GLCD_PrintString("D. Pneu (cm): ");
		GLCD_GotoXY(80, 35);
		GLCD_PrintInteger(diametro_pneu);
		
		//Imprime a distancia percorrida
		GLCD_GotoXY(20, 50);
		GLCD_PrintInteger(dist_km);
		GLCD_GotoXY(50, 50);
		GLCD_PrintString("km");
		
		//Imprime a distancia percorrida
		GLCD_GotoXY(80, 50);
		GLCD_PrintInteger(forca);
		GLCD_GotoXY(100, 50);
		GLCD_PrintString("kN");
		
		//Imprime o percentual da bateria
		GLCD_GotoXY(100, 5);
		GLCD_PrintInteger(bateria);
		GLCD_GotoXY(120, 5);
		GLCD_PrintString("%");
		
		//Imprime a temperatura do motor
		GLCD_GotoXY(100, 20);
		GLCD_PrintInteger(temperatura);
		GLCD_GotoXY(120, 20);
		GLCD_PrintString("C");

		GLCD_Render();

		if(!(PIND & 0b10000000))
		{
			GLCD_GotoXY(120, 45);
			GLCD_PrintString("P"); //Se a chave P está aberta, escreve "P" no display

		}
		//Se a chave P está fechada
		if((PIND & 0b10000000))
		{

			if(!(PIND & 0b01000000)) //e a chave D/R está aberta, escreve "D" no display
			{
				GLCD_GotoXY(120, 45);
				GLCD_PrintString("D");

			}
			if((PIND & 0b01000000)) //e a chave D/R está fechada, escreve "R" no display
			{
				GLCD_GotoXY(120, 45);
				GLCD_PrintString("R");
			}

		}
		GLCD_Render();
		//Condição para mostrar a velocidade no display de 7 segmentos
		if((delta_t % 2) / 1000 == 0)
		{
			switch(cont)
			{
				case 0:
				PORTB &= 0b00000001; // resetando PB1 - PB6
				PORTB |= 0b11000000; // resetando PB4, pino que habilita o display das unidades
				PORTB |= ((((vel_carro / 1) % 10) & 0b00001111) << 1);
				break;

				case 1:
				PORTB &= 0b00000001; // resetando PB1 - PB6
				PORTB |= 0b10100000; // resetando PB5 e PB7, pino que habilita o display das dezenas
				PORTB |= ((((vel_carro / 10) % 10) & 0b00001111) << 1);
				break;

				case 2:
				PORTB &= 0b00000001; // resetando PB1 - PB6
				PORTB |= 0b01100000; // resetando PB5 e PB7, pino que habilita o display das centenas
				PORTB |= ((((vel_carro / 100) % 10) & 0b00001111) << 1);
				cont = -1;
				break;
				
			}
		}
		cont++;
	}
}

//-----------------FUNÇÕES UTILIZADAS-----------------

ISR(TIMER1_CAPT_vect) //Captura do timer 1
{
	if (TCCR1B & (1 << ICES1))
	tempo_borda_subida = ICR1;
	else
	distancia_sonar_cm = ((uint32_t)(ICR1 - tempo_borda_subida) * 4) / 58; //Cálculo da distância sonar

	TCCR1B ^= (1 << ICES1);
}

ISR(TIMER0_COMPA_vect)
{
	tempo_ms++;
	TEMPO_100us++;
	
	if ((tempo_ms % 150) == 0)  //True a cada 150ms
	{
		flag_150ms = 1;
	}

	if ((tempo_ms % 500) == 0)  //True a cada 500ms
	{
		flag_500ms = 1;
	}
	

}

ISR(INT0_vect) //Função que realiza o cálculo do RPM e da distância percorrida
{
	if((PIND & 0b10000000)) //Verifica se a chave P está fechada
	{
		N_voltas_pneu++;

		delta_t = TEMPO_100us - TEMPO_100us_anterior;

		RPM = 600000 / delta_t;

		TEMPO_100us_anterior = TEMPO_100us;

		vel_carro = (diametro_pneu * PI * RPM) * 6 / 10000;

		dist_anterior += (diametro_pneu * PI);

		dist_km = dist_anterior / 100000;
		
		eeprom_write_dword(ENDERECO_HODOMETRO, (uint32_t)dist_km); //Escreve na EEPROM a distância percorrida


	}
}

ISR(PCINT2_vect)//Função para incrementar ou decrementar o diâmetro do pneu
{
	if(!(PIND & 0b00010000)) //Incrementa o diametro do pneu
	{
		diametro_pneu++;
		eeprom_write_byte(ENDERECO_DIAMETRO_PNEU, diametro_pneu);//Escreve o diametro do pneu na posição 4 da EEPROM

	}

	if(!(PIND & 0b00100000)) //Decrementa o diametro do pneu
	{
		diametro_pneu--;
		eeprom_write_byte(ENDERECO_DIAMETRO_PNEU, diametro_pneu); //Escreve o diametro do pneu na posição 4 da EEPROM

	}
}
//Função que realiza a leitura dos sensores ADC: acelerador, bateria e temperatura
void leitura_sensores_ADC(uint8_t *flag_disparo, uint16_t *temperatura_pont, uint16_t *bateria_pont, uint16_t *temp_max_pont, uint16_t *forca_pont)
{

	static uint8_t cont_canal = 0; //Contador do número de canal
	static uint8_t cont_temp = 0; //Contadaor da temperatura
	static uint8_t cont_forca = 0; //Contador da força

	if (*flag_disparo) //Verifica se a flag_150ms é verdadeira
	{
		switch(cont_canal)
		{
			case 0: //Leitura do canal 0
			ADMUX = 0b01000000; //Muda para o canal 0
			acelerador = (float)(ADC / 4); //acelerador = ADC/4
			OCR2B = acelerador;
			break;

			case 1: //Leitura do canal 1
			ADMUX = 0b01000001; //Muda para o canal 1
			*bateria_pont = (((float)100 / 1023) * ADC); //Bateria = (100/1023)*ADC
			break;

			case 2: //Leitura do canal 2
			ADMUX = 0b01000010; //Muda para o canal 2
			//Equações utilizadas para o cálculo da temperatura: Vt=(5/1023)*ADC, Rt=((1000*Vt)/(5-Vt)) e T=2,6*Rt-260
			*temperatura_pont = (((float)2.6 * (((float)1000 * (((float)5 / 1023) * ADC)) / (5 - (((float)5 / 1023) * ADC)))) - 260);
			
			//Verifica se o contador de temperatura é zero
			if(cont_temp == 0){
				*temp_max_pont = *temperatura_pont; //Temperatura inicial é igual a temperatura máxima
				cont_temp++;
			}
			else if (*temperatura_pont > *temp_max_pont){ //Se a temperatura for maior que a temperatura máxima, então
				*temp_max_pont = *temperatura_pont; //a váriável temperatura máxima assume os valores da variável temperatura
			}
			eeprom_write_byte(ENDERECO_TEMPERATURA_MAX, *temp_max_pont); //Escreve o valor da temperatura máxima na posição 6 da EEPROM
			break;
			
			case 3: //Leitura do canal 3
			ADMUX = 0b01000011; //Muda para o canal 3
			// Vf = ((5/1023)*ADC) e forca = 1000*(Vf-2.66/0.86)
			*forca_pont = ((float)1000*(((((float)5/1023)*ADC)-(float)2.66)/0.86))/10;
			cont_canal = -1;
			break;
		}
		cont_canal++;
		*flag_disparo = 0;
	}
}
void USART_Init(unsigned int ubrr)
{
	UBRR0H = (unsigned char)(ubrr>>8); //Ajusta a taxa de transmissão
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //Habilita o transmissor e o receptor
	UCSR0C = (1<<USBS0)|(3<<UCSZ00); //Ajusta o formato do frame: 8 bits de dados e 2 de parada
	
}
void USART_Transmit(unsigned char data)
{
	while(!( UCSR0A & (1<<UDRE0)));//Espera a limpeza do registrador de transmissão
	UDR0 = data; //Coloca o dado no registrador e envia o dado
}

// Função para recepção de um frame de 5 a 8bits
unsigned char USART_Receive(void)
{
	while(!(UCSR0A & (1<<RXC0))); //Espera o dado ser recebido
	return UDR0; //Lê o dado recebido e retorna
}
ISR(USART_RX_vect)
{
	char char_recebido;
	char_recebido = UDR0; //Atribui char_recebido a UDR0
	
	if(char_recebido =='d'){ //Verifica se o usuário enviou o caractere "d" no monitor serial
		USART_Transmit(temperatura_max); //Transmite a temperatura máxima armazenada na EEPROM
	}
	if(char_recebido == 'l'){ //Verifica se o usuário enviou o caractere "l" no monitor serial
		temperatura_max = 0; //Zera o valor da temperatura máxima que está armazenada na EEPROM
	}
}

void airbag (uint16_t *distancia_sonar_pont, uint32_t *vel_carro_pont, uint16_t *forca_pont) {
	
	if ((*distancia_sonar_pont < 300) && (*vel_carro_pont > 20)) //Verifica se a distância sonar é menor que 300cm e a velocidade do carro é maior que 20
	{
		OCR2B = 25; //Se a condição for verdadeira diminuir o duty cicle baixa pra 25
		
		if(*forca_pont > 30){ //Verifica se a força detectada pelo sensor de impacto é maior que 30kN
			PORTC = 0b11111111; //PC6 com nível lógico alto, acionando o airbag
		}
		else {
			PORTC = 0b00000000; //PC6 com nível lógico baixo
		}
		
	}
}