#ifndef TSL237_H
#define TSL237_H

#endif


#define MAX_SAMPLE_TIME 480000

extern uint32_t tsl237_done;

void tsl237_command(char *);
uint32_t tsl237_readsensor(void);
