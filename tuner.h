// tuner.h
// HI-TECH C Compiler
// 21-Dec-2014
//

#include "i2c.h"


#define FM1_MIN 1000 //500*2 - 50МГц
#define FM2_MIN 1520 //760*2 - 76МГц


// W_reg  буфер регистров для записи в тюнер
uint8_t reg02H, reg02L;
uint8_t reg03H, reg03L;
uint8_t reg05H, reg05L;

// R_reg  буфер регистров прочитаных с тюнера
uint8_t reg0AH, reg0AL;
uint8_t reg0BH, reg0BL;

//в этом буфере хранится последняя установленная частота соответствующего диапазона
uint16_t freqFM1 = 6730; //5000 - мин. 50МГц  //стартовая частота при инициализации
uint16_t freqFM2 = 10390; //7600 - мин. 76МГц


//функции для работы с тюнером
void tuner_init(void); //инициализация, сброс настроек
void tuner_read(void); //читаем регистры в буфер
void tuner_write(uint8_t reg, uint8_t h_byte, uint8_t l_byte); //запись регистра
void tuner_sleep(void); //выключить тюнер(перевести чип в спящий режим)
void tuner_wakeup(void); //включить тюнер(в рабочий режим)
//команды настройки частоты
uint16_t __min_freq(void); //получить минимальную частоту для диапазона
void tuner_init_band(uint8_t fm1); //выбрать диапазон и установить начальную частоту
void tuner_set_freq(uint16_t freq); //установить частоту
uint16_t tuner_get_freq(void); //получить текущую частоту приема (R_reg)
void tuner_change_band(void); //сменить диапазон на другой
uint8_t tuner_get_band(void); // получить тукущую настройку диапазона 0-76..108, 1-50..76
//поск станции
void tuner_seek_start(uint8_t up); //запустить поиск станции
uint8_t tuner_is_seek(void); //если идет поиск возвращает 1
void tuner_seek_stop_update(void); //проверяет результат и останавливает поиск
uint8_t __seek_complete(void); //флаг завершения поиска (R_reg)
uint8_t tuner_get_rssi(void); //уровень принимаемого сигнала (R_reg)
uint8_t tuner_get_stereo_indicator(void); //индикатор стереоприема (R_reg)
//команды настройки звука
void tuner_volume(uint8_t val); //установить громкость звука 0..15
uint8_t get_tuner_volume(void); //получить текущее значение настройки
void tuner_change_bassb(void); //вкл/выкл усиление низких частот
uint8_t tuner_get_bassb(void);
void tuner_mute(uint8_t enable); //1-отключить звук



///////////////////////////////////////////////////////////////////////////////////////////////////
void tuner_init(void) //инициализация
{
reg02H = 0x00;
reg02L = 0x02; //bit1-soft reset enable
tuner_write(0x02, reg02H, reg02L);

reg02H = 0xc0; //bit15-audio output high-z disable, bit14-mute disable
reg02L = 0x01; //bit0-power up enable
tuner_write(0x02, reg02H, reg02L);

__delay_ms(100);
__delay_ms(100);
__delay_ms(100);
__delay_ms(100);
__delay_ms(100);
__delay_ms(100);

reg03H = 0x00;
reg03L = 0x0a; //диапазон 76-108, шаг сетки 50kHz
tuner_write(0x03, reg03H, reg03L);

__delay_ms(50);

reg05H = 0x88;
reg05L = 0x81; //bit3..0 sound volume
tuner_write(0x05, reg05H, reg05L);

tuner_write(0x07, 0xc0, 0x02); //bit9-расширеный диапазон 50-76МГц

tuner_init_band(0);
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void tuner_read(void) //читаем регистры тюнера в буфер R_reg
{
//_i2c_init();
_i2c_start();
_i2c_sendbyte(0x21); //RDA5807HP address 0x10

reg0AH = _i2c_readbyte(ACK); //читаем пару двухбайтовых регистров (0AH,0BH по даташиту)
reg0AL = _i2c_readbyte(ACK);
reg0BH = _i2c_readbyte(ACK);
reg0BL = _i2c_readbyte(NACK);

_i2c_stop();
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void tuner_write(uint8_t reg, uint8_t h_byte, uint8_t l_byte) //отправляем данные в регистр тюнера
{
//_i2c_init();
_i2c_start();
_i2c_sendbyte(0x22); //RDA5807HP address 0x11

_i2c_sendbyte(reg); //register address (произвольный доступ к регистрам)

_i2c_sendbyte(h_byte); // 15..8
_i2c_sendbyte(l_byte); // 7..0

_i2c_stop();
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void tuner_sleep(void) //отправляем спать
{
CLEAR_BIT(reg02L,0); //power up disable

tuner_write(0x02, reg02H, reg02L);
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void tuner_wakeup(void)
{
SET_BIT(reg02L,0); //bit0 - power up enable

tuner_write(0x02, reg02H, reg02L);

tuner_init_band(tuner_get_band()); //настройка
}


///////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t __min_freq(void) //минимальная частота диапазона
{
if(tuner_get_band()) return FM1_MIN;
else return FM2_MIN;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void tuner_init_band(uint8_t fm1) //1-FM1, 0-FM2 //выбор диапазона и установка начальной частоты
{
if(fm1)  { SET_BIT(reg03L,3); SET_BIT(reg03L,2); tuner_set_freq(freqFM1); }  //11 - 50..76 MHz (FM1)
else  { SET_BIT(reg03L,3); CLEAR_BIT(reg03L,2); tuner_set_freq(freqFM2); }  //10 - 76..108 MHz (FM2)
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void tuner_set_freq(uint16_t freq) //установка частоты приема
{
//пример для 103,95МГц: 10395*20/100-1520 = 559 будет записано в регистр
uint16_t temp = ((uint32_t)freq*20/100)-__min_freq(); //преобразуем

reg03H = temp>>2; //записываем в буфер новое значение частоты (биты 6..15)
reg03L = (reg03L & 0x3f) | ((temp & 0x03)<<6);

SET_BIT(reg03L,4); //разрешаем настройку частоты

tuner_write(0x03, reg03H, reg03L); //отправляем новые настройки в тюнер

__delay_ms(50); //рекомендовано в доках

tuner_read(); //читаем регистры тюнера
}


///////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t tuner_get_freq(void) //получить текущую частоту приема
{
uint16_t temp = (256*reg0AH + reg0AL) & 0x03ff; //0b00000011 0b11111111;

uint16_t freq = (((uint32_t)temp + (uint32_t)__min_freq())*100)/20;

if(tuner_get_band()) freqFM1 = freq; //если диапазон 50-76 обновим буфер freqFM1
else  freqFM2 = freq; //если диапазон 76-108 обновим буфер freqFM2

return freq;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void tuner_change_band(void) //сменить диапазон
{
if(tuner_get_band()) tuner_init_band(0);
else  tuner_init_band(1);
}


///////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t tuner_get_band(void) //получить настройку диапазона
{
if(BIT_IS_SET(reg03L,3) && BIT_IS_SET(reg03L,2)) return 1; //11 значит диапазон 50–76 MHz
else return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void tuner_seek_start(uint8_t up) //сканировать 1-up, 0-down
{
SET_BIT(reg02H,0); //seek start

if(up) SET_BIT(reg02H,1); //seek up //вверх по диапазону
else CLEAR_BIT(reg02H,1);

SET_BIT(reg02L,7); //seek mode  //останавливается на краях диапазона и поднимает флаг seek fail

tuner_write(0x02, reg02H, reg02L);
}



///////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t tuner_is_seek(void) //проверим запущен сейчас поиск или нет
{
if(BIT_IS_SET(reg02H,0)) return 1; //если бит seek start установлен, возвращаем 1
else  return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void tuner_seek_stop_update(void) //
{
if(tuner_is_seek()) //если запущен поиск будет обновлен буфер R_reg
	{
	tuner_read(); //обновим(прчитаем) буфер
	if(__seek_complete()) //проверим флаг зафершения поиска
		{
		CLEAR_BIT(reg02H,0); //и остановим поиск
		tuner_write(0x02, reg02H, reg02L);
		}
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t __seek_complete(void) //проверить флаг seek/tune complete, 1-поиск завершен
{
if BIT_IS_SET(reg0AH,6) return 1;
else  return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t tuner_get_rssi(void) //получить значение RSSI
{
return (reg0BH & 0b11111110);
}


///////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t tuner_get_stereo_indicator(void) //проверить идикатор стерео, 1-стеро
{
if BIT_IS_SET(reg0AH,2) return 1;
else  return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void tuner_volume(uint8_t val) //установка громкости
{
reg05L = (reg05L & 0xf0) | (val & 0x0f);

tuner_write(0x05, reg05H, reg05L); //отправляем новые настройки в тюнер
}


///////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t get_tuner_volume(void) //получить настройку уровня громкости
{
return (reg05L & 0x0f);
}


///////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t tuner_get_bassb(void) //получить состояние настройки bass boost (1 включено)
{
if(BIT_IS_SET(reg02H,4)) return 1;
else return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void tuner_change_bassb(void) //bass boost
{
if(tuner_get_bassb()) CLEAR_BIT(reg02H,4);
else SET_BIT(reg02H,4);  //bit12-Bass Boost enable

tuner_write(0x02, reg02H, reg02L);
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void tuner_mute(uint8_t enable) //1-mute enable
{
if(enable) CLEAR_BIT(reg02H,6);  //bit14-Mute Disable
else SET_BIT(reg02H,6);

tuner_write(0x02, reg02H, reg02L);
}
