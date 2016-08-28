/* 
 * File:   C5Decoder.h
 * Author: pargles
 *
 * Created on August 28, 2016, 10:43 AM
 */
#pragma once
#include "c5decoderdefns.h"

class C5Decoder
{
    public:
    const char a_delim='\n';
      
    
    void runC5Decoder(char *Argv[]){
        main(*Argv, *Argv);
    }

};

