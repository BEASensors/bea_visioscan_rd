#ifndef COMMON_LIBS
#define COMMON_LIBS
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <ctype.h>

#define DEBUG_POS std::to_string(__FILE__)+ "-"+std::to_string(__FUNCTION__) + "-"+std::to_string(__LINE__)+"--"
#define DEBUG_POS_FL std::to_string(__FUNCTION__) + "-"+std::to_string(__LINE__)+"--"

namespace std
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
    template <typename Type>
    Type to_number(const string &str)
    {
        istringstream iss(str);
        Type num;
        iss >> num;
        return num;
    }

}

void  ShowCharArray(char *icharArray,int len);
std::vector<std::string> YSplitString(const std::string &input,const char delimiter);
int Hex2Ascii(char* hex, char* ascii);
int Hex2char(uint8_t c);
unsigned int GetCStrLen(const char *str);

#endif // COMMON_LIBS

