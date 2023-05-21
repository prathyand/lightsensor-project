#include "main.h"
#include "queue.h"
#include "command.h"
#include "interrupt.h"
#include <stdio.h>

extern queue_t qbuff;
extern uint8_t maxcommbuffsize;


uint8_t fetchcomm(uint8_t* combuff){
	uint8_t ch;
	static uint8_t ptr=0, md=0;

	ch = dequeue(&qbuff); //check if the queue has a  new char

	while(ch){

		if(ch=='\n' || ch =='\r'){
			md = 1;
			combuff[ptr]='\0';
			break;
		}

//		save the char
		combuff[ptr++] = ch;
		putchar(ch);

		if(ptr==maxcommbuffsize){
			ptr=0;
			md=0;
			return 2;
		}
//		fetch the next char
		uint32_t mask;
	    mask = disable();
	    ch = dequeue(&qbuff);
	    restore(mask);
	}

	if(md){
		md=0;
		ptr=0;
		return 1;
	}
	return 0;
}



void commandprompt(RTC_TimeTypeDef *sysTime,RTC_DateTypeDef *sysDate){
	if(sysTime==NULL || sysDate==NULL){
		printf("IULS> ");
	}else{
		printf("%02d/%02d/20%02d %02d:%02d:%02d IULS> ",sysDate->Month,sysDate->Date,sysDate->Year,
				sysTime->Hours,sysTime->Minutes,sysTime->Seconds);
	}
	return;
}
