#pragma once
#include "libc5.h"

//! \ingroup libMD5
//! \{
class C5
{
public:
  /**
   * compute digest over buf of length len.
   * multiple calls may extend the digest over more data.
   */
  void setHeader( char *header){
     //c5header = str_split(header, a_delim);
  }
  
  void setUpC5(char *Argv[]){
      //mainFunction(3,  Argv);
  }
  
  void setData(char* data){
      //c5data = str_split(data, a_delim);;
  }
  /*
  void setTestData(char* testData){
      c5test= str_split(testData, a_delim);
  }*/
  
  void setParameters(char *listOfParemeters[]){
      
  }
  
  void runC5(char *Argv[]){
      mainFunction(3,  Argv);
  }

};
//! \}
