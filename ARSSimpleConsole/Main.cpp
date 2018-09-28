#include <windows.h>
#include <stdio.h>
#include <string>

#define BUFSIZE 4096 
 
int main(int argc, char* argv[]) 
{ 
   CHAR   chBuf[BUFSIZE]; 
   DWORD  dwRead, dwWritten; 
   HANDLE hStdin, hStdout; 
   BOOL   bSuccess; 
 
   // Configure cor usando system,
   if(argc > 1)
   {
		std::string color = "COLOR ";
		color.append(argv[1]);
		system(color.c_str());
   }

   hStdout = GetStdHandle(STD_OUTPUT_HANDLE); 
   hStdin  = GetStdHandle(STD_INPUT_HANDLE); 
   if ( 
       (hStdout == INVALID_HANDLE_VALUE) || 
       (hStdin  == INVALID_HANDLE_VALUE) 
      ){
      ExitProcess(1); 
   }

   // Pega dado do input padrão e manda para o output padrão
   while(1)
   { 
      bSuccess = ReadFile(hStdin, chBuf, BUFSIZE, &dwRead, NULL); 
      bSuccess = WriteFile(hStdout, chBuf, dwRead, &dwWritten, NULL); 
   } 

   return 0;
}

