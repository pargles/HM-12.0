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
