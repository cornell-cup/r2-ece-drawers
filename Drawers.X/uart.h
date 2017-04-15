
#ifndef UART_H    /* Guard against multiple inclusion */
#define UART_H

#include <stdint.h>

#define SERIAL_ECHO

#define BAUDRATE    9600

#define MSG_BEGIN       'a'
#define MSG_END         '\r'

void initUART(void);
uint8_t serial_get_byte(void);
uint16_t serial_get_angle(void);

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
