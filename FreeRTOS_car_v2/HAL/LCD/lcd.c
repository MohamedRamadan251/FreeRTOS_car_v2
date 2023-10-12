/*******************************************************************************
 *                                                                             *
 * [FILE NAME]:   lcd.c                                                        *
 *                                                                             *
 * [AUTHOR]:      Ahmed Saeed, Amr Gafar, Kareem Abd-elrasheed,                *
 *                Mohamed Abosreea, Ahmed Maher, and Mohamed Ramadan           *
 *                                                                             *
 * [Version]:     1.0.0                                                        *
 *                                                                             *
 * [DATE]:        09/09/2023                                                   *
 *                                                                             *
 * [DESCRIPTION]: source file for LCD driver                                   *
 *                                                                             *
 *******************************************************************************/

/*******************************************************************************
 *                                 Includes                                    *
 *******************************************************************************/
#include <stdio.h>
#include <stdlib.h>

#include "LIB/bitwise_operation.h"

#include "../../MCAL/GPIO/gpio.h"
#include "../../MCAL/general_functions.h"

#include "lcd.h"
/*******************************************************************************
 *                      Functions Definitions                                  *
 *******************************************************************************/

/*
 * Description :
 * Initialize the LCD:
 * 1. Setup the LCD pins directions by use the DIO driver.
 * 2. Setup the LCD Data Mode 4-bits or 8-bits.
 * Inputs: None.
 * Returns: None.
 */
void lcd_init(void)
{
    /* Configure the direction for RS and E pins as output pins */
    gpio_init(LCD_RS_PORT_ID,LCD_RS_PIN_ID,PIN_OUTPUT_PULL_UP);
    gpio_init(LCD_E_PORT_ID,LCD_E_PIN_ID,PIN_OUTPUT_PULL_UP);

    _delay_ms(20);      /* LCD Power ON delay always > 15ms */

#if(LCD_DATA_BITS_MODE == 4)
    /* Configure 4 pins in the data port as output pins */
    gpio_init(LCD_DATA_PORT_ID,LCD_DB4_PIN_ID,PIN_OUTPUT_PULL_UP);
    gpio_init(LCD_DATA_PORT_ID,LCD_DB5_PIN_ID,PIN_OUTPUT_PULL_UP);
    gpio_init(LCD_DATA_PORT_ID,LCD_DB6_PIN_ID,PIN_OUTPUT_PULL_UP);
    gpio_init(LCD_DATA_PORT_ID,LCD_DB7_PIN_ID,PIN_OUTPUT_PULL_UP);

    /* Send for 4 bit initialization of LCD  */
    lcd_send_command(LCD_TWO_LINES_FOUR_BITS_MODE_INIT1);
    lcd_send_command(LCD_TWO_LINES_FOUR_BITS_MODE_INIT2);

    /* use 2-lines LCD + 4-bits Data Mode + 5*7 dot display Mode */
    lcd_send_command(LCD_TWO_LINES_FOUR_BITS_MODE);

#elif(LCD_DATA_BITS_MODE == 8)
    /* Configure the data port as output port */
    gpio_init(LCD_DATA_PORT_ID,PIN0_ID,PIN_OUTPUT_PULL_UP);
    gpio_init(LCD_DATA_PORT_ID,PIN1_ID,PIN_OUTPUT_PULL_UP);
    gpio_init(LCD_DATA_PORT_ID,PIN2_ID,PIN_OUTPUT_PULL_UP);
    gpio_init(LCD_DATA_PORT_ID,PIN3_ID,PIN_OUTPUT_PULL_UP);
    gpio_init(LCD_DATA_PORT_ID,PIN4_ID,PIN_OUTPUT_PULL_UP);
    gpio_init(LCD_DATA_PORT_ID,PIN5_ID,PIN_OUTPUT_PULL_UP);
    gpio_init(LCD_DATA_PORT_ID,PIN6_ID,PIN_OUTPUT_PULL_UP);
    gpio_init(LCD_DATA_PORT_ID,PIN7_ID,PIN_OUTPUT_PULL_UP);

    /* use 2-lines LCD + 8-bits Data Mode + 5*7 dot display Mode */
    lcd_send_command(LCD_TWO_LINES_EIGHT_BITS_MODE);

#endif

    lcd_send_command(LCD_CURSOR_OFF); /* cursor off */
    lcd_send_command(LCD_CLEAR_COMMAND); /* clear LCD at the beginning */
}

/*
 * Description :
 * Send the required command to the screen.
 * Parameters:
 * - command -> The command to be sent to the LCD.
 * Returns: None.
 */
void lcd_send_command(uint8_t command)
{
    gpio_write_pin(LCD_RS_PORT_ID,LCD_RS_PIN_ID,LOGIC_LOW); /* Instruction Mode RS=0 */
    _delay_ms(1); /* delay for processing Tas = 50ns */
    gpio_write_pin(LCD_E_PORT_ID,LCD_E_PIN_ID,LOGIC_HIGH); /* Enable LCD E=1 */
    _delay_ms(1); /* delay for processing Tpw - Tdws = 190ns */

#if(LCD_DATA_BITS_MODE == 4)
    gpio_write_pin(LCD_DATA_PORT_ID,LCD_DB4_PIN_ID,GET_BIT(command,4));
    gpio_write_pin(LCD_DATA_PORT_ID,LCD_DB5_PIN_ID,GET_BIT(command,5));
    gpio_write_pin(LCD_DATA_PORT_ID,LCD_DB6_PIN_ID,GET_BIT(command,6));
    gpio_write_pin(LCD_DATA_PORT_ID,LCD_DB7_PIN_ID,GET_BIT(command,7));

    _delay_ms(1); /* delay for processing Tdsw = 100ns */
    gpio_write_pin(LCD_E_PORT_ID,LCD_E_PIN_ID,LOGIC_LOW); /* Disable LCD E=0 */
    _delay_ms(1); /* delay for processing Th = 13ns */
    gpio_write_pin(LCD_E_PORT_ID,LCD_E_PIN_ID,LOGIC_HIGH); /* Enable LCD E=1 */
    _delay_ms(1); /* delay for processing Tpw - Tdws = 190ns */

    gpio_write_pin(LCD_DATA_PORT_ID,LCD_DB4_PIN_ID,GET_BIT(command,0));
    gpio_write_pin(LCD_DATA_PORT_ID,LCD_DB5_PIN_ID,GET_BIT(command,1));
    gpio_write_pin(LCD_DATA_PORT_ID,LCD_DB6_PIN_ID,GET_BIT(command,2));
    gpio_write_pin(LCD_DATA_PORT_ID,LCD_DB7_PIN_ID,GET_BIT(command,3));

    _delay_ms(1); /* delay for processing Tdsw = 100ns */
    gpio_write_pin(LCD_E_PORT_ID,LCD_E_PIN_ID,LOGIC_LOW); /* Disable LCD E=0 */
    _delay_ms(1); /* delay for processing Th = 13ns */

#elif(LCD_DATA_BITS_MODE == 8)
    gpio_write_pin(LCD_DATA_PORT_ID,command); /* out the required command to the data bus D0 --> D7 */
    _delay_ms(1); /* delay for processing Tdsw = 100ns */
    gpio_write_pin(LCD_E_PORT_ID,LCD_E_PIN_ID,LOGIC_LOW); /* Disable LCD E=0 */
    _delay_ms(1); /* delay for processing Th = 13ns */
#endif
}

/*
 * Description :
 * Display the required character on the screen
 * Parameters:
 * - data -> The character data to be displayed on the LCD.
 * Returns: None.
 */
void lcd_display_character(uint8_t data)
{
    gpio_write_pin(LCD_RS_PORT_ID,LCD_RS_PIN_ID,LOGIC_HIGH); /* Data Mode RS=1 */
    _delay_ms(1); /* delay for processing Tas = 50ns */
    gpio_write_pin(LCD_E_PORT_ID,LCD_E_PIN_ID,LOGIC_HIGH); /* Enable LCD E=1 */
    _delay_ms(1); /* delay for processing Tpw - Tdws = 190ns */

#if(LCD_DATA_BITS_MODE == 4)
    gpio_write_pin(LCD_DATA_PORT_ID,LCD_DB4_PIN_ID,GET_BIT(data,4));
    gpio_write_pin(LCD_DATA_PORT_ID,LCD_DB5_PIN_ID,GET_BIT(data,5));
    gpio_write_pin(LCD_DATA_PORT_ID,LCD_DB6_PIN_ID,GET_BIT(data,6));
    gpio_write_pin(LCD_DATA_PORT_ID,LCD_DB7_PIN_ID,GET_BIT(data,7));

    _delay_ms(1); /* delay for processing Tdsw = 100ns */
    gpio_write_pin(LCD_E_PORT_ID,LCD_E_PIN_ID,LOGIC_LOW); /* Disable LCD E=0 */
    _delay_ms(1); /* delay for processing Th = 13ns */
    gpio_write_pin(LCD_E_PORT_ID,LCD_E_PIN_ID,LOGIC_HIGH); /* Enable LCD E=1 */
    _delay_ms(1); /* delay for processing Tpw - Tdws = 190ns */

    gpio_write_pin(LCD_DATA_PORT_ID,LCD_DB4_PIN_ID,GET_BIT(data,0));
    gpio_write_pin(LCD_DATA_PORT_ID,LCD_DB5_PIN_ID,GET_BIT(data,1));
    gpio_write_pin(LCD_DATA_PORT_ID,LCD_DB6_PIN_ID,GET_BIT(data,2));
    gpio_write_pin(LCD_DATA_PORT_ID,LCD_DB7_PIN_ID,GET_BIT(data,3));

    _delay_ms(1); /* delay for processing Tdsw = 100ns */
    gpio_write_pin(LCD_E_PORT_ID,LCD_E_PIN_ID,LOGIC_LOW); /* Disable LCD E=0 */
    _delay_ms(1); /* delay for processing Th = 13ns */
    gpio_write_pin(LCD_E_PORT_ID,LCD_E_PIN_ID,LOGIC_HIGH); /* Enable LCD E=1 */
    _delay_ms(1); /* delay for processing Tpw - Tdws = 190ns */

#elif(LCD_DATA_BITS_MODE == 8)
    gpio_write_port(LCD_DATA_PORT_ID,data); /* out the required command to the data bus D0 --> D7 */
    _delay_ms(1); /* delay for processing Tdsw = 100ns */
    gpio_write_pin(LCD_E_PORT_ID,LCD_E_PIN_ID,LOGIC_LOW); /* Disable LCD E=0 */
    _delay_ms(1); /* delay for processing Th = 13ns */
#endif
}

/*
 * Description :
 * Display the required string on the screen.
 * Parameters:
 * - Str -> The null-terminated string to be displayed on the LCD.
 * Returns: None.
 */
void lcd_display_string(const char *Str)
{
    uint8_t i = 0;
    while(Str[i] != '\0')
    {
        lcd_display_character(Str[i]);
        i++;
    }
}

/*
 * Description :
 * Move the cursor to a specified row and column index on the screen.
 * Parameters:
 * - row: The row (0 to 3) where you want to move the cursor.
 * - col: The column (0 to 15) where you want to move the cursor.
 * Returns: None.
 */
void lcd_move_cursor(uint8_t row,uint8_t col)
{
    uint8_t lcd_memory_address;

    /* Calculate the required address in the LCD DDRAM */
    switch(row)
    {
        case 0:
            lcd_memory_address=col;
                break;
        case 1:
            lcd_memory_address=col+0x40;
                break;
        case 2:
            lcd_memory_address=col+0x10;
                break;
        case 3:
            lcd_memory_address=col+0x50;
                break;
    }
    /* Move the LCD cursor to this specific address */
    lcd_send_command(lcd_memory_address | LCD_SET_CURSOR_LOCATION);
}

/*
 * Description :
 * Display the required string in a specified row and column index on the screen.
 * Parameters:
 * - row: The row (0 to 3) where you want to display the string.
 * - col: The column (0 to 15) where you want to display the string.
 * - Str: The null-terminated string to be displayed.
 * Returns: None.
 */
void lcd_display_string_row_column(uint8_t row,uint8_t col,const char *Str)
{
    lcd_move_cursor(row,col); /* go to to the required LCD position */
    lcd_display_string(Str); /* display the string */
}

/*
 * Description :
 * Display the required decimal value on the screen.
 * Parameters:
 * - data: The integer value to be converted and displayed.
 * Returns: None.
 */
void lcd_intger_to_string(sint32_t data)
{
   char buff[100]; /* String to hold the ascii result */
   ltoa(data,buff,10); /* Use itoa C function to convert the data to its corresponding ASCII value, 10 for decimal */
   //sprintf(buff,"%d",data);
   lcd_display_string(buff); /* Display the string */
}

/*
 * Description :
 * Send the clear screen command.
 * Parameters: None.
 * Returns: None.
 */
void lcd_clear_screen(void)
{
    lcd_send_command(LCD_CLEAR_COMMAND); /* Send clear display command */
}


