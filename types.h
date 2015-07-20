/* ************************************************************************** */
/* ************************************************************************** */
#ifndef __TYPES_H__
#define __TYPES_H__

/* ************************************************************************** */

typedef unsigned char   byte;

typedef enum {
	TERMINAL_PRINT=0,
	TERMINAL_READ,
	TERMINAL_SHOW,
	TERMINAL_ENTER,
	TERMINAL_WAIT,
} t_terminal_state;

typedef enum {
	E_OK=0,
	E_FAIL,
	E_BADPTR
} t_error;

typedef enum {
	SCI_TX_READY=0,
	SCI_TX_SENDING,
	SCI_TX_FINISH,
	SCI_NONE
} t_sci_stat;

typedef struct {
	struct {
		t_sci_stat tx :4;
	} sci;
	struct {
		t_error error :2;
	} sys;
} t_status;




/* ************************************************************************** */

#endif //__TYPES_H__
/* ************************************************************************** */
