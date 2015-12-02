/* 
 * File:   sensorThread_public.h
 * Author: Khai Ngo
 *
 * Created on September 27, 2015, 11:34 PM
 */

#ifndef SENSORTHREAD_PUBLIC_H
#define	SENSORTHREAD_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    void sendDistancetoQueue(uint8_t distance);
    
#ifdef	__cplusplus
}
#endif

#endif	/* SENSORTHREAD_PUBLIC_H */
