#pragma once
#include "defns.h"

//! \ingroup libMD5
//! \{
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
