#include <common_libs.h>
//@
void  ShowCharArray(char *icharArray,int len)
{
    unsigned char *charArray=(unsigned char*)icharArray;
    for(int i=0;i<len;i++)
    {
        printf("%02x  ",charArray[i]);
    }
    std::cout<<std::endl;
}
//@
std::vector<std::string> YSplitString(const std::string &input,const char delimiter)
{
    std::istringstream stream(input);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(stream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

//@
unsigned int GetCStrLen(const char *str)
{
    const char *cp = str;
    while(*cp++);
    return (cp - str -1);
}
//@
int Hex2char(uint8_t c)
{
    return ((c >= '0') && (c <= '9')) ? int(c - '0') :
           ((c >= 'A') && (c <= 'F')) ? int(c - 'A' + 10) :
           ((c >= 'a') && (c <= 'f')) ? int(c - 'a' + 10) :
           -1;
}
//@
int Hex2Ascii(char* hex, char* ascii)
{
    int hexLen = GetCStrLen(hex);
    int asciiLen = 0;
    for (int i = 0, cnt = 0; i<hexLen; i++)
    {
        char c = Hex2char(hex[i]);
        if (-1 == c)
            continue;
        if(cnt) {
            cnt = 0;
            ascii[asciiLen++] += c;
        } else {
            cnt = 1;
            ascii[asciiLen] = c << 4;
        }
    }
    ascii[asciiLen++] = 0;
    return asciiLen;
}
