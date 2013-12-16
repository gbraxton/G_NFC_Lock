/*
 * G_NFC_Lock_v001.c
 *
 * Created: 12/6/2013 8:07:17 AM
 *  Author: gbraxton
 */ 


#include <stdio.h>
#include <stdlib.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include "lcd.h"
#include "mfrc522.h"
#include "queue.h"
#include "uid_db.h"



//*************Scheduler Code - copied from UCR CS 122A include file scheduler.h**********************************
unsigned long tasksPeriodGCD = 50; // Start count from here, down to 0. Default 1ms
unsigned long tasksPeriodCntDown = 0; // Current internal count of 1ms ticks
unsigned char tasksNum = 5; // Number of tasks in the scheduler. Default 0 tasks
typedef struct task {
    signed 	 char state; 		//Task's current state
    unsigned long period; 		//Task period
    unsigned long elapsedTime; 	//Time elapsed since last task tick
    int (*TickFct)(int); 		//Task tick function
} task;
static task task_monitor_lock, task_lock_control, task_main, task_sound, task_lcd;
task* tasks[] = {&task_lcd, &task_monitor_lock, &task_lock_control, &task_main, &task_sound};
//task* tasks[] = {&task_main};
void TimerISR() {
    static unsigned char i;
    for (i = 0; i < tasksNum; i++) {
        if ( tasks[i]->elapsedTime >= tasks[i]->period ) { // Ready
            tasks[i]->state = tasks[i]->TickFct(tasks[i]->state);
            tasks[i]->elapsedTime = 0;
        }
        tasks[i]->elapsedTime += tasksPeriodGCD;
    }
}
ISR(TIMER1_COMPA_vect) {
    // CPU automatically calls when TCNT0 == OCR0 (every 1 ms per TimerOn settings)
    tasksPeriodCntDown--; 			// Count down to 0 rather than up to TOP
    if (tasksPeriodCntDown == 0) { 	// results in a more efficient compare
        TimerISR(); 				// Call the ISR that the user uses
        tasksPeriodCntDown = tasksPeriodGCD;
    }
}
void TimerSet(unsigned long m) {
    tasksPeriodGCD = m;
    tasksPeriodCntDown = tasksPeriodGCD;
}
void TimerOn() {
    // AVR timer/counter controller register TCCR1
    TCCR1B 	= (1<<WGM12)|(1<<CS11)|(1<<CS10);
    // WGM12 (bit3) = 1: CTC mode (clear timer on compare)
    // CS12,CS11,CS10 (bit2bit1bit0) = 011: prescaler /64
    // Thus TCCR1B = 00001011 or 0x0B
    // So, 8 MHz clock or 8,000,000 /64 = 125,000 ticks/s
    // Thus, TCNT1 register will count at 125,000 ticks/s

    // AVR output compare register OCR1A.
    OCR1A 	= 125;	// Timer interrupt will be generated when TCNT1==OCR1A
    // We want a 1 ms tick. 0.001 s * 125,000 ticks/s = 125
    // So when TCNT1 register equals 125,
    // 1 ms has passed. Thus, we compare to 125.
    // AVR timer interrupt mask register

    #if defined (__AVR_ATmega1284__)
    TIMSK1 	= (1<<OCIE1A); // OCIE1A (bit1): enables compare match interrupt - ATMega1284
    #else
    TIMSK 	= (1<<OCIE1A); // OCIE1A (bit1): enables compare match interrupt - ATMega32
    #endif

    // Initialize avr counter
    TCNT1 = 0;

    // TimerISR will be called every tasksPeriodCntDown milliseconds
    tasksPeriodCntDown = tasksPeriodGCD;

    // Enable global interrupts
    SREG |= 0x80;	// 0x80: 1000000
}
//*********************end scheduler code***********************************************************

// ADC constants
const short adc_locked_upper_bound = 48;
const short adc_locked_lower_bound = 43;
const short adc_unlocked_upper_bound = 36;
const short adc_unlocked_lower_bound = 31;

//*********Commands****************
// commands to actuate lock controller
enum Lock_Commands{ lc_idle, 
                    lc_lock, 
                    lc_unlock
                  };
struct _Queue* lock_command_queue;

// commands to play sound; set by lock controller, read by sound controller
enum Sound_Commands{    snd_idle, 
                        snd_locked, 
                        snd_unlocked, 
                        snd_malfunction
                    };
struct _Queue* sound_command_queue;

// commands to lcd display
enum LCD_Commands{  lcd_write_ready, 
                    lcd_write_locked, 
                    lcd_write_unlocked, 
                    lcd_write_master, 
                    lcd_write_add,
                    lcd_write_added,
                    lcd_write_add_exist,
                    lcd_write_add_full, 
                    lcd_write_remove, 
                    lcd_write_removed,
                    lcd_write_remove_empty,
                    lcd_write_invalid,
                    lcd_write_malfunction};
struct _Queue* lcd_command_queue;

//lock positions
enum Lock_Positions{locked, unlocked, malfunction};
unsigned char current_lock_position = locked;

// ****
unsigned char current_remove_option = 0;


void A2D_init() {
    ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADATE);
    ADMUX = 4;//set pin 4 to adc input
    // ADEN: Enables analog-to-digital conversion
    // ADSC: Starts analog-to-digital conversion
    // ADATE: Enables auto-triggering, allowing for constant
    //	    analog to digital conversions.
}

void init_PWM(){
    TCCR2A = (1 << COM2A0) | (1 << WGM21);
    TCCR2B = (1 << CS22);
}

void set_PWM(double frequency) {
    if (frequency < 1){
        OCR2A = 0;
    }else{
        OCR2A = (int)(8000000 / (128 * frequency)) - 1;
    }
}

enum LCD_States{    st_lcd_start, 
                    st_lcd_init, 
                    st_lcd_wait,
                    st_lcd_write,
                    st_lcd_hold};
                    
int lcd_tick(int state){
    static unsigned short lcd_hold_time = 0;
    switch(state){// lcd actions
        case st_lcd_start:
            state = st_lcd_init;
            break;
        case st_lcd_init:
            if(!QueueIsEmpty(lcd_command_queue)){
                state = st_lcd_write;
            } else {
                state = st_lcd_wait;
            }
            break;
        case st_lcd_wait:
            if(!QueueIsEmpty(lcd_command_queue)){
                state = st_lcd_write;
            } else {
                state = st_lcd_wait;
            }
            break;
        case st_lcd_write:
            if( lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_locked || 
                lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_unlocked || 
                lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_invalid ||
                lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_added ||
                lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_add_exist ||
                lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_add_full ||
                lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_removed ||
                lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_remove_empty){
                    
                state = st_lcd_hold;
                lcd_hold_time = 0;
            } else {
                state = st_lcd_wait;
            }
            QueueDequeue(lcd_command_queue);                
            break;
        case st_lcd_hold:
            if(lcd_hold_time < 20){
                lcd_hold_time++;
                state = st_lcd_hold;
            } else {
                state = st_lcd_write;
                QueueEnqueue(lcd_command_queue, lcd_write_ready);
            }                
            break;
        default:
            state = st_lcd_start;
            break;
    }//end lcd transitions
    
    switch(state){
        case st_lcd_init:
            LCD_init();
            break;
        case st_lcd_write:
            if(lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_ready){
                LCD_ClearScreen();
                if(isTagDBEmpty()){
                    LCD_DisplayString(1, (const unsigned char*)"No Valid Keys");
                    LCD_DisplayString(17, (const unsigned char*)"Add w/ Master");
                } else {
                    LCD_DisplayString(1, (const unsigned char*)"Scan Key Tag");
                    if(current_lock_position == locked){
                        LCD_DisplayString(17, (const unsigned char*)"to Unlock");
                    } else if(current_lock_position == unlocked){
                        LCD_DisplayString(17, (const unsigned char*)"to Lock");
                    } else {
                        LCD_DisplayString(17, (const unsigned char*)"malfunction");
                    }
                }                
            } else if(lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_locked){
                LCD_ClearScreen();
                LCD_DisplayString(1, (const unsigned char*)"Locked by");
                LCD_DisplayString(17, (const unsigned char*)"Key# ");
                LCD_DisplayString(22, (const unsigned char*)tag_id);
            } else if(lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_unlocked){
                LCD_ClearScreen();
                LCD_DisplayString(1, (const unsigned char*)"Unlocked by");
                LCD_DisplayString(17, (const unsigned char*)"Key# ");
                LCD_DisplayString(22, (const unsigned char*)tag_id);
            } else if(lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_invalid){
                LCD_ClearScreen();
                LCD_DisplayString(1, (const unsigned char*)"Key Tag");
                LCD_DisplayString(17, (const unsigned char*)"Not Authorized");
            } else if(lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_master){
                LCD_ClearScreen();
                LCD_DisplayString(1, (const unsigned char*)"Master Key");
                LCD_DisplayString(17, (const unsigned char*)"Add or Remove");
            } else if(lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_add){
                LCD_ClearScreen();
                LCD_DisplayString(1, (const unsigned char*)"Scan the Tag");
                LCD_DisplayString(17, (const unsigned char*)"to be Added");
            } else if(lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_remove){
                LCD_ClearScreen();
                LCD_DisplayString(1, (const unsigned char*)valid_tag_db[current_remove_option]);
                LCD_DisplayString(17, (const unsigned char*)"Remove?");
            } else if(lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_malfunction){
                LCD_ClearScreen();
                LCD_DisplayString(1, (const unsigned char*)"Malfunction");
                LCD_DisplayString(17, (const unsigned char*)"Check Lock");
            } else if(lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_add_exist){
                LCD_ClearScreen();
                LCD_DisplayString(1, (const unsigned char*)"Key Tag Already");
                LCD_DisplayString(17, (const unsigned char*)"Added");
            } else if(lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_added){
                LCD_ClearScreen();
                LCD_DisplayString(1, (const unsigned char*)"Key# ");
                LCD_DisplayString(6, (const unsigned char*)tag_id);
                LCD_DisplayString(17, (const unsigned char*)"Added");
            }  else if(lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_add_full){
                LCD_ClearScreen();
                LCD_DisplayString(1, (const unsigned char*)"Over Max # Keys");
                LCD_DisplayString(17, (const unsigned char*)"Cannot Add");
            } else if(lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_removed){
                LCD_ClearScreen();
                LCD_DisplayString(1, (const unsigned char*)"Key Tag Removed");
            } else if(lcd_command_queue->buffer[lcd_command_queue->front] == lcd_write_remove_empty){
                LCD_ClearScreen();
                LCD_DisplayString(1, (const unsigned char*)"No Valid Keys");
                LCD_DisplayString(17, (const unsigned char*)"to Remove");
            }
            break;
        default:
            break;
    }//end lcd actions
    return state;
}    
    
    
// each unit 125ms each
enum Sound_States{  st_sound_start, 
                    st_sound_init, 
                    st_sound_idle,
                    st_sound_lock1,
                    st_sound_lock2,
                    st_sound_lock3,
                    st_sound_lock4,
                    st_sound_unlock1,
                    st_sound_unlock2,
                    st_sound_malfunction1,
                    st_sound_malfunction2,
                    st_sound_malfunction3,
                    st_sound_malfunction4};

int sound_tick(int state){
    switch(state){
        case st_sound_start:
            state = st_sound_init;
            break;
        case st_sound_init:
            state = st_sound_idle;
            break;
        case st_sound_idle:
            if(!QueueIsEmpty(sound_command_queue)){
                if(sound_command_queue->buffer[sound_command_queue->front] == snd_locked){
                    state = st_sound_lock1;
                } else if(sound_command_queue->buffer[sound_command_queue->front] == snd_unlocked){
                    state = st_sound_unlock1;
                } else if(sound_command_queue->buffer[sound_command_queue->front] == snd_malfunction){
                    state = st_sound_malfunction1;
                }
                QueueDequeue(sound_command_queue);
            } else {
                state = st_sound_idle;
            }
            break;
        case st_sound_lock1:
            state = st_sound_lock2;
            break;
        case st_sound_lock2:
            state = st_sound_lock3;
            break;
        case st_sound_lock3:
            state = st_sound_lock4;
            break;
        case st_sound_lock4:
            state = st_sound_idle;
            break;
        case st_sound_unlock1:
            state = st_sound_unlock2;
            break;
        case st_sound_unlock2:
            state = st_sound_idle;
            break;
        case st_sound_malfunction1:
            state = st_sound_malfunction2;            
            break;
        case st_sound_malfunction2:
            state = st_sound_malfunction3;
            break;
        case st_sound_malfunction3:
            state = st_sound_malfunction4;
            break;
        case st_sound_malfunction4:
            if(!QueueIsEmpty(sound_command_queue)){
                state = st_sound_idle;
            } else {
                state = st_sound_malfunction1;
            }
            break;    
        default:
            state = st_sound_start;
            break;
    }
    switch(state){
        case st_sound_init:
            init_PWM();
            break;
        case st_sound_lock1:
            set_PWM(3135.96);
            break;
        case st_sound_lock2:
            set_PWM(0);
            break;
        case st_sound_lock3:
            set_PWM(3135.96);
            break;
        case st_sound_lock4:
            set_PWM(0);
            break;
        case st_sound_unlock1:
            set_PWM(3135.96);
            break;
        case st_sound_unlock2:
            set_PWM(0);
            break;
        case st_sound_malfunction1:
            set_PWM(130.81);
            PORTD |= 0x02;
            break;
        case st_sound_malfunction3:
            set_PWM(0);
            PORTD &= 0xFD;
            break;
        default:
            break;
    }
    return state;
}

enum Lock_Monitor_States{st_lock_monitor_start, st_lock_monitor_run};
int lock_monitor_tick(int state){
    unsigned short adc_reading = ADC;
    switch(state){//lock monitor transitions
        case st_lock_monitor_start:
            state = st_lock_monitor_run;
            break;
        case st_lock_monitor_run:
            state = st_lock_monitor_run;
            break;
        default:
            break;
    }//end lock monitor transitions
    
    switch(state){//lock_monitor actions
        case st_lock_monitor_run:
            if(adc_reading <= adc_unlocked_upper_bound){
                current_lock_position = unlocked;
                PORTD = (PORTD & 0x80) | 0x74; 
            } else if(adc_reading >= adc_locked_lower_bound){
                current_lock_position = locked;                    
                PORTD = (PORTD & 0x80) | 0x71;
            } else {
                current_lock_position = malfunction;
                PORTD = (PORTD & 0x80) | 0x72;
            }
            break;
        default:
            break;
    }//end lock_monitor actions
    return state;
    };

enum Lock_Controller_States{    st_lock_start, 
                                st_lock_init, 
                                st_lock_idle, 
                                st_lock_try1,
                                st_lock_verify1,
                                st_lock_try2,
                                st_lock_verify2 
                            };
                    
int lock_controller_tick(int state){
    switch(state){ // lock transitions
        case st_lock_start:
            state = st_lock_init;
            break;
        case st_lock_init:
            state = st_lock_idle;
            break;
        case st_lock_idle:
            if(!QueueIsEmpty(lock_command_queue)){
                state = st_lock_try1;
            } else {
                state = st_lock_idle;
            }
            break;
        case st_lock_try1:
            state = st_lock_verify1;
            break;
        case st_lock_verify1:
            if( (lock_command_queue->buffer[lock_command_queue->front] == lc_lock) && (current_lock_position == locked) ){
                state = st_lock_idle;
                QueueDequeue(lock_command_queue);
                QueueEnqueue(sound_command_queue, snd_locked);
                QueueEnqueue(lcd_command_queue, lcd_write_locked);
            } else if( (lock_command_queue->buffer[lock_command_queue->front] == lc_unlock) && (current_lock_position == unlocked) ){
                state = st_lock_idle;
                QueueDequeue(lock_command_queue);
                QueueEnqueue(sound_command_queue, snd_unlocked);
                QueueEnqueue(lcd_command_queue, lcd_write_unlocked);
            } else {
                state = st_lock_try2;
            }
            break;
        case st_lock_try2:
            state = st_lock_verify2;
            break;
        case st_lock_verify2:
            if( (lock_command_queue->buffer[lock_command_queue->front] == lc_lock) && (current_lock_position == locked) ){
                state = st_lock_idle;
                QueueDequeue(lock_command_queue);
                QueueEnqueue(sound_command_queue, snd_locked);
                QueueEnqueue(lcd_command_queue, lcd_write_locked);
            } else if( (lock_command_queue->buffer[lock_command_queue->front] == lc_unlock) && (current_lock_position == unlocked) ){
                state = st_lock_idle;
                QueueDequeue(lock_command_queue);
                QueueEnqueue(sound_command_queue, snd_unlocked);
                QueueEnqueue(lcd_command_queue, lcd_write_unlocked);
            } else {
                state = st_lock_idle;
                QueueDequeue(lock_command_queue);
                QueueEnqueue(sound_command_queue, snd_malfunction);
                QueueEnqueue(lcd_command_queue, lcd_write_malfunction);
            }
            break;
        default:
            state = st_lock_start;
            break;
    }// end lock transitions
    
    switch(state){ // lock actions
        case st_lock_try1:
            if(lock_command_queue->buffer[lock_command_queue->front] == lc_lock){
                PORTA |= 0x08;
            } else if(lock_command_queue->buffer[lock_command_queue->front] == lc_unlock){
                PORTA |= 0x02;
            }                
            break;
        case st_lock_verify1:
            if(lock_command_queue->buffer[lock_command_queue->front] == lc_lock){
                PORTA &= ~(0x08);
            } else if(lock_command_queue->buffer[lock_command_queue->front] == lc_unlock){
                PORTA &= ~(0x02);
            }            
            break;
        case st_lock_try2:
            if(lock_command_queue->buffer[lock_command_queue->front] == lc_lock){
                PORTA |= 0x08;
            } else if(lock_command_queue->buffer[lock_command_queue->front] == lc_unlock){
                PORTA |= 0x02;
            }
            break;
        case st_lock_verify2:
            if(lock_command_queue->buffer[lock_command_queue->front] == lc_lock){
                PORTA &= ~(0x08);
            } else if(lock_command_queue->buffer[lock_command_queue->front] == lc_unlock){
                PORTA &= ~(0x02);
            }
            break;
        default:
            break;
    }// end lock actions
    return state;
}

enum main_controller_states{    st_main_start,
                                st_main_init,
                                st_main_wait_new,
                                st_main_checkID,
                                st_main_master,
                                st_main_add_pressed,
                                st_main_add_tag,
                                st_main_rm_pressed,
                                st_main_rm_tag,
                                st_main_lr_pressed,
                                st_main_hold
                            };

int main_controller_tick(int state){
    static unsigned short main_hold_time = 0;
    switch(state){//main_task transitions/actions
        case st_main_start:
            state = st_main_init;
            break;
        case st_main_init:
            SPI_Master_Init();
            PCD_Init();
            state = st_main_wait_new;
            QueueEnqueue(lcd_command_queue, lcd_write_ready);
            break;
        case st_main_wait_new:
            if(PICC_IsNewCardPresent()){
                storeTagID();
                PICC_HaltA();
                state = st_main_checkID;
            } else {
                state = st_main_wait_new;
            }
            break;
        case st_main_checkID:
            if(isMaster(tag_id)){
                state = st_main_master;
                QueueEnqueue(lcd_command_queue, lcd_write_master);
            } else if(search_valid_tag(tag_id) != -1){
                state = st_main_hold;
                main_hold_time = 0;
                if(current_lock_position == locked || current_lock_position == malfunction){
                    QueueEnqueue(lock_command_queue, lc_unlock);
                } else if(current_lock_position == unlocked){
                    QueueEnqueue(lock_command_queue, lc_lock);
                }
            } else {
                state = st_main_hold;
                main_hold_time = 0;
                QueueEnqueue(lcd_command_queue, lcd_write_invalid);
            }
            break;
        case st_main_master:
            if(!(PIND & 0x10)){
                if(isTagDBFull()){
                    state = st_main_hold;
                    main_hold_time = 0;
                    QueueEnqueue(lcd_command_queue, lcd_write_add_full);
                } else {
                    state = st_main_add_pressed;
                    QueueEnqueue(lcd_command_queue, lcd_write_add);
                }                    
            } else if(!(PIND & 0x40)){
                if(isTagDBEmpty()){
                    state = st_main_hold;
                    main_hold_time = 0;
                    QueueEnqueue(lcd_command_queue, lcd_write_remove_empty);
                } else {
                    state = st_main_rm_pressed;
                    current_remove_option = 0;
                    QueueEnqueue(lcd_command_queue, lcd_write_remove);
                }                    
            } else {
                state = st_main_master;
            }
            break;
        case st_main_add_pressed:
            if(PIND & 0x10){
                state = st_main_add_tag;
            } else {
                state = st_main_add_pressed;
            }
            break;
        case st_main_add_tag:
            if(PICC_IsNewCardPresent()){
                storeTagID();
                PICC_HaltA();
                if(isMaster(tag_id)){
                    state = st_main_add_tag;
                } else if(search_valid_tag(tag_id) == -1){
                    state = st_main_hold;
                    main_hold_time = 0;
                    add_valid_tag(tag_id);
                    QueueEnqueue(lcd_command_queue, lcd_write_added);
                } else {
                    state = st_main_hold;
                    main_hold_time = 0;
                    QueueEnqueue(lcd_command_queue, lcd_write_add_exist);
                }
            } else {
                state = st_main_add_tag;
            }
            break;
        case st_main_rm_pressed:
            if(PIND & 0x40){
                state = st_main_rm_tag;
            } else {
                state = st_main_rm_pressed;
            }
            break;
        case st_main_rm_tag:
            if(!(PIND & 0x10)){
                current_remove_option = (current_remove_option + (numValidKeys - 1)) % numValidKeys;
                QueueEnqueue(lcd_command_queue, lcd_write_remove);
                state = st_main_lr_pressed;
            } else if(!(PIND & 0x40)){
                current_remove_option = (current_remove_option + 1) % numValidKeys;
                QueueEnqueue(lcd_command_queue, lcd_write_remove);
                state = st_main_lr_pressed;
            } else if(!(PIND & 0x20)){
                remove_valid_tag(current_remove_option);
                QueueEnqueue(lcd_command_queue, lcd_write_removed);
                state = st_main_hold;
                main_hold_time = 0;
            } else {
                state = st_main_rm_tag;
            }
            break;
        case st_main_lr_pressed:
            if( (PIND & 0x10) && (PIND & 0x40) ){
                state = st_main_rm_tag;
            } else {
                state = st_main_lr_pressed;
            }
            break;
        case st_main_hold:
            if(main_hold_time < 20){
                main_hold_time++;
                state = st_main_hold;
            } else {
                state = st_main_wait_new;
            }
            break;
        default:
            state = st_main_start;
            break;
    }//end main_task transitions/actions
    return state;
};

int main(void)
{
    DDRA = 0xFF;
    DDRB = 0xBF; PORTB = 0x40;
    DDRC = 0xFF;
    DDRD = 0x8F; PORTD = 0x70;
    
    lcd_command_queue = QueueInit(10);
    sound_command_queue = QueueInit(10);
    lock_command_queue = QueueInit(10);
    
    task_monitor_lock.elapsedTime = 200;
    task_monitor_lock.period = 200;
    task_monitor_lock.state = st_lock_monitor_start;
    task_monitor_lock.TickFct = &lock_monitor_tick;
    
    task_lock_control.elapsedTime = 250;
    task_lock_control.period = 250;
    task_lock_control.state = st_lock_start;
    task_lock_control.TickFct = &lock_controller_tick;
    
    task_main.elapsedTime = 100;
    task_main.period = 100;
    task_main.state = st_main_start;
    task_main.TickFct = &main_controller_tick;
    
    task_sound.elapsedTime = 100;
    task_sound.period = 100;
    task_sound.state = st_sound_start;
    task_sound.TickFct = &sound_tick;
    
    task_lcd.elapsedTime = 100;
    task_lcd.period = 100;
    task_lcd.state = st_lcd_start;
    task_lcd.TickFct = &lcd_tick;
    
    A2D_init();
    TimerSet(tasksPeriodGCD);
    TimerOn();
    
    while(1)
    {
        ;
    }
}