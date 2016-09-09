#pragma once
//#include "defns.h"
#include "libc5.h"

/*
 * @author: Pargles Dall'Oglio
 * class made to interface C library with C++ HEVC functions
 */
class C5Decoder
{
    GlobalValues allAllocatedVariables;
    
private:
    
    void getGlobals(){
        allAllocatedVariables = getAllocatedValues();
    }
    
public:
  
  void loadC5Tree(char *Argv[]){
      loadTreeToMemory(3,Argv);
      getGlobals();
      
  }
  
  int classifyCurrentLine(char attributes[]){
      return splitCU(attributes,&allAllocatedVariables);
  }

};
//! \}
