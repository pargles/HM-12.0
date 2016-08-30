#pragma once
#include "defns.h"

//! \ingroup libMD5
//! \{
class C5Decoder
{
public:
const char a_delim='\n';
  
  void decodeC5(char *Argv[]){
      mainFunctionDecoder(3,  Argv);
  }

};
//! \}
