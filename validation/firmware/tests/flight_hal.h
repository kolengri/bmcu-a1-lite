#pragma once
#include <deque>
static std::deque<uint8_t> log_rx;
static std::vector<uint8_t> log_tx;
enum { RESET=0,GPIO_Mode_IN_FLOATING=5,USART_WordLength_8b=8,USART_Parity_No=0,USART_IT_RXNE=0x20,USART_FLAG_TXE=0x80 };
inline int USART_GetFlagStatus(MockUSART*,int flag){return flag==USART_FLAG_TXE?1:(!log_rx.empty()?1:0);}
inline uint16_t USART_ReceiveData(MockUSART*){auto value=log_rx.front();log_rx.pop_front();return value;}
inline void USART_SendData(MockUSART*,uint16_t value){log_tx.push_back(uint8_t(value));}
