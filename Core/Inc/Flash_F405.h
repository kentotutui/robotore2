/*
 * Flash_F405.h
 *
 *  Created on: 2023/02/09
 *      Author: ryuku
 */

#ifndef INC_FLASH_F405_H_
#define INC_FLASH_F405_H_

#include "main.h"
#include <string.h>

extern const uint32_t start_adress_sector6; //sector6 start address
extern const uint32_t end_adress_sector6;
extern const uint32_t start_adress_sector7; //sector7 start address
extern const uint32_t end_adress_sector7;
extern const uint32_t start_adress_sector8; //sector8 start address
extern const uint32_t end_adress_sector8;
extern const uint32_t start_adress_sector9; //sector9 start address
extern const uint32_t end_adress_sector9;
extern const uint32_t start_adress_sector10; //sector10 start address
extern const uint32_t end_adress_sector10;
extern const uint32_t start_adress_sector11; //sector11 start address
//extern const uint32_t middle_adress_sector11; //sector11 middle address
extern const uint32_t end_adress_sector11;

void FLASH_WaitBusy(void);
void FLASH_Erease7(void);
void FLASH_Erease9(void);
void FLASH_Erease10(void);
void FLASH_Erease11(void);
void FLASH_EreaseSector(uint16_t);
//void FLASH_Write_Byte(uint32_t address, uint8_t data);
void FLASH_Write_HalfWord(uint32_t, uint16_t);
void FLASH_Write_Word(uint32_t, uint32_t);
void FLASH_Write_Word_F(uint32_t, float);
void FLASH_Write_Word_S(uint32_t, int32_t);
void FLASH_Write_DoubleWord(uint32_t, int64_t);
void FLASH_ReadData(uint32_t, uint32_t*, uint32_t);
//void FLASH_WriteData(uint32_t address, uint8_t* data, uint32_t size);

#endif /* INC_FLASH_F405_H_ */
