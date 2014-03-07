#include "MyQueue.h"
#include <stdlib.h>


struct rData *head;
struct rData *current;

void InitQueue() {
    head = NULL;
    current = NULL;
}

void push(char value) {
    struct rData * thisGuy = (struct rData*)malloc(sizeof(struct rData));
    thisGuy->value = value;
    thisGuy->next = NULL;
    if(head == NULL || current == NULL){
        head = thisGuy;
        if(current != NULL) {
            free(current);
        }
        current = thisGuy;
    }
    else    current->next = thisGuy;
}

char pop() {
    if(head == NULL){
        return 0;
    }
    else {
        struct rData *temp;
        char value;
        temp = head;
        head = head->next;
        value = temp->value;
        free(temp);
        return value;
    }
}

//int isEmpty() {
//    if(head == NULL || current == NULL)
//        return FALSE;
//    return TRUE;
//}