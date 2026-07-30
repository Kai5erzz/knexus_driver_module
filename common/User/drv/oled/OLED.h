#ifndef KNEXUS_OLED_H
#define KNEXUS_OLED_H

#include <stdbool.h>
#include <stdint.h>

/* SSD1306-compatible 128x64 OLED, 8x16 ASCII font, four text rows. */
bool OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t line, uint8_t column, char ch);
void OLED_ShowString(uint8_t line, uint8_t column, const char *text);
void OLED_ShowNum(uint8_t line, uint8_t column,
                  uint32_t number, uint8_t length);

bool OLED_IsReady(void);
uint32_t OLED_GetErrorCount(void);

#endif /* KNEXUS_OLED_H */
