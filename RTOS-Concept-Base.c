#include "rims.h"
#define QSize 8

typedef struct task {
    int state;
    unsigned long period;
    unsigned long elapsedTime;
    int (*Function) (int);
} task;

typedef struct _QueueUC {
    unsigned char buf[QSize];
    unsigned char *head;
    unsigned char *tail;
    unsigned char cnt;
} QueueUC;

void QueueInit (QueueUC *Q) {
    (*Q).cnt = 0;
    (*Q).head = (*Q).buf;
    (*Q).tail = (*Q).buf + QSize;
}

unsigned char QueueFull(QueueUC Q) {
    return (Q.cnt == QSize);
}

unsigned char QueueEmpty(QueueUC Q) {
    return (Q.cnt == 0);
}

unsigned char QueueHeadLeft(QueueUC Q) {
    return (Q.head < Q.tail);
}

void QueuePrint(QueueUC Q) {
    unsigned char i;
    unsigned char j;
    
    printf("Queue Contents: \n");
    
    if (QueueHeadLeft(Q)) {
        for (i = 0; i < QSize; i++) {
            printf("Item %d: %d\n", i, Q.head[i]);
        }
    } else {
        for (i = 0; i < QSize - (Q.head - Q.buf); i++) {
            printf("Item %d: %d\n", i, Q.head[i]);
        }
        for (j = 0; j < Q.tail - Q.buf; j++) {
            printf("Item %d: %d\n", i + j + 1, Q.buf[j]);
        }
    }
}

void QueuePush(QueueUC *Q, unsigned char item) {
    if (!QueueFull(*Q)) {
        DisableInterrupts();
        if ((*Q).tail - (*Q).buf == QSize) {
            (*Q).tail = (*Q).buf;
        } else {
            (*Q).tail++;
        }
        (*(*Q).tail) = item;
        (*Q).cnt++;
        EnableInterrupts();
    }
}

unsigned char QueuePop(QueueUC *Q) {
    int i;
    unsigned char item = 0;
    
    if (!QueueEmpty(*Q)) {
        DisableInterrupts();
        item = (*(*Q).head);
        (*Q).cnt--;
        if (((*Q).head + 1) < &(*Q).buf[QSize]) {
            (*Q).head++;
        } else {
            (*Q).head = (*Q).buf;
        }
        EnableInterrupts();
    }
    
    return(item);
}

const unsigned int numTasks = 2;
const unsigned long period = 100;
const unsigned long periodBlinkLED = 1500;
const unsigned long periodThreeLED = 500;

task tasks[2];

enum BL_States {BL0, BL1};
int BlinkLED (int state);

enum TL_States {TL0, TL1, TL2};
int ThreeLED (int state);

void TimerISR() {
    unsigned char i;
    
    for (i = 0; i < numTasks; i++) {
        if (tasks[i].elapsedTime >= tasks[i].period) {
            tasks[i].state = tasks[i].Function(tasks[i].state);
            tasks[i].elapsedTime = 0;
        }
        tasks[i].elapsedTime += period;
    }
}

QueueUC TestQ;

int main() {
    tasks[0].state = BL0;
    tasks[0].period = periodBlinkLED;
    tasks[0].elapsedTime = tasks[0].period;
    tasks[0].Function = &BlinkLED;
    tasks[1].state = TL0;
    tasks[1].period = periodThreeLED;
    tasks[1].elapsedTime = tasks[1].period;
    tasks[1].Function = &ThreeLED;
    
    QueueInit(&TestQ);
    QueuePush(&TestQ, 1);
    QueuePush(&TestQ, 2);
    QueuePrint(TestQ);
    QueuePop(&TestQ);
    QueuePush(&TestQ, 1);
    QueuePush(&TestQ, 2);
    QueuePush(&TestQ, 3);
    QueuePrint(TestQ);
    QueuePop(&TestQ);
    QueuePrint(TestQ);
    QueuePush(&TestQ, 4);
    QueuePrint(TestQ);
    printf("%d, %d, %d, %d", QueuePop(&TestQ), QueuePop(&TestQ), QueuePop(&TestQ), QueuePop(&TestQ));
    
    TimerSet(period);
    TimerOn();
    
    while(1);
    
    return 0;
}

//  State Machine Tick Function for Blink LED 0 SM
int BlinkLED(int state) {
    //  Transitions
    switch (state) {
        case BL0:   state = BL1;    break;
        case BL1:   state = BL0;  break;
        default:    state = BL0;  break;
    }
    //  Actions
    switch (state) {
        case BL0: B0 = 0;                 break;
        case BL1: B0 = 1;                 break;
    }
    
    return state;
}

//  State Machine Tick Function for Cycling Blink Thru 5-7
int ThreeLED(int state) {
    //  Transitions
    switch (state) {
        case TL0:    state = TL1;     break;
        case TL1:    state = TL2;     break;
        case TL2:    state = TL0;     break;
        default:     state = TL0;     break;
    }
    //  Actions
    switch (state) {
        case TL0:    B5 = 1; B6 = 0; B7 = 0; break;
        case TL1:    B5 = 0; B6 = 1; B7 = 0; break;
        case TL2:    B5 = 0; B6 = 0; B7 = 1; break;
    }
    
    return state;
}