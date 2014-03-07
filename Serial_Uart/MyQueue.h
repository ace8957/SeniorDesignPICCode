/* 
 * File:   myQueue.h
 * Author: JBB
 *
 * Created on March 6, 2014, 4:57 PM
 */

#ifndef MYQUEUE_H
#define	MYQUEUE_H

#ifdef	__cplusplus
extern "C" {
#endif
//#define TRUE 1
//#define FALSE 0

    struct rData {
        char value;
        struct rData *next;
    };

    

    void InitQueue();
    void push(char value);
    char pop();
    //int isEmpty();



#ifdef	__cplusplus
}
#endif

#endif	/* MYQUEUE_H */

