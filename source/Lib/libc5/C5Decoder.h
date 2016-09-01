#pragma once
#include "defns.h"

//! \ingroup libMD5
//! \{
class C5Decoder
{
public:
  
  void loadC5Tree(char *Argv[]){
      loadTreeToMemory(3,Argv);
  }
  
  int classifyCurrentLine(char attributes[]){
      return splitCU(attributes);
  }

};
//! \}
