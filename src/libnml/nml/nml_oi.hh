/********************************************************************
* Description: nml_oi.cc
*   Defines Generic NML Message structures used to log errors and
*   interact with an Operator Interface from within an NML_MODULE.
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: LGPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change: 
********************************************************************/

#ifndef NML_OI_HH
#define NML_OI_HH

#include "cms.hh"		// class CMS
#include "nml.hh"		// class NML
#include "nmlmsg.hh"		// class NMLmsg

// NML operator interface stuff for errors, text, and graphics display

#define NML_ERROR_TYPE    ((NMLTYPE) 1)
#define NML_TEXT_TYPE     ((NMLTYPE) 2)
#define NML_DISPLAY_TYPE  ((NMLTYPE) 3)

// Sizes for strings for the above messages

#define NML_ERROR_LEN 256
#define NML_TEXT_LEN 256
#define NML_DISPLAY_LEN 256

class NML_ERROR:public NMLmsg {
  public:
    NML_ERROR()
      : NMLmsg(NML_ERROR_TYPE, sizeof(NML_ERROR)),
        error{}
    {};
    ~NML_ERROR() {
    };

    void update(CMS * cms);
    char error[NML_ERROR_LEN];
};

class NML_TEXT:public NMLmsg {
  public:
    NML_TEXT()
      : NMLmsg(NML_TEXT_TYPE, sizeof(NML_TEXT)),
        text{}
    {};
    ~NML_TEXT() {
    };

    void update(CMS * cms);
    char text[NML_TEXT_LEN];
};

class NML_DISPLAY:public NMLmsg {
  public:
    NML_DISPLAY()
      : NMLmsg(NML_DISPLAY_TYPE, sizeof(NML_DISPLAY)),
        display{}
    {};
    ~NML_DISPLAY() {
    };

    void update(CMS * cms);
    char display[NML_DISPLAY_LEN];
};

// NML format function
extern int nmlErrorFormat(NMLTYPE type, void *buffer, CMS * cms);

#endif
