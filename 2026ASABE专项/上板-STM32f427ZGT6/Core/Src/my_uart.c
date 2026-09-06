#include "my_uart.h"
#include <ctype.h>
#include <stdlib.h>
#include "usart.h"

extern float Kp, Ki, Kd;
int is_num_char(char c)
{
    return (c == '+' || c == '-' || c == '.' || isdigit((unsigned char)c));
}


void Parse_CommandLine(char *s)
{
    char *p = s;

    while (*p != '\0')
    {

        while (*p == ' ' || *p == '\r' || *p == '\t') p++;

        char key = *p;                 
        if (key != 'p' && key != 'i' && key != 'd')
        {
            // 不认识的字符，跳过一个继续找
            if (*p == '\0') break;
            p++;
            continue;
        }




        // 指向数字开始位�?
        p++; 
        if (!is_num_char(*p))    // 如果后面不是数字（比�?"p i60"），就跳�?
            continue;
        // strtof 会把�?p 开始的数字转成 float，并�?end 指向数字结束位置
        char *end = NULL;
        float val = strtof(p, &end);

        // end == p 说明没解析出数字
        if (end == p)
            continue;

        // 写入对应参数
        if (key == 'p') Kp = val;
        else if (key == 'i') Ki = val;
        else if (key == 'd') Kd = val;
        //HAL_UART_Transmit_IT(&huart4, (const uint8_t*)&Kp, sizeof(Kp)); 
        //HAL_UART_Transmit_IT(&huart4, (const uint8_t*)&Ki, sizeof(Ki));
        //HAL_UART_Transmit_IT(&huart4, (const uint8_t*)&Kd, sizeof(Kd));

        // 继续从数字末尾往后解析下一�?
        p = end;
    }
}
