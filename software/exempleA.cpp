// basic file operations
#include <iostream>
#include <fstream>
#include <chrono>
#include <cstdlib>
#include <unistd.h>
using namespace std;

int main () {
  ofstream myfile;
  ifstream myfile2;
  myfile.open ("/dev/ttyUSB0");
  // myfile2.open ("/dev/ttyUSB0");
    // myfile.flush();

  while(true)
  {
    usleep(100000);
    myfile << "0/HELLO_BASE\r\n";
    myfile.flush();
  }
  myfile.close();
  return 0;
}

// #include <fstream>
// #include <iostream>
// #include <sstream>
// #include <string>
// using namespace std;

// int main()
// {
//     ifstream infile("/dev/ttyUSB0", ios::in);
//     while (true)
//     {
//         for (std::string line; std::getline(infile, line); ) 
//         {
//             std::cout << line << std::endl;
//         }
//     }
// }

// #include <iostream>
// #include <fstream>

// int main () {
//     std::string str;
//     std::fstream f;
//     f.open("/dev/ttyUSB0");
//     while(true)
//     {
//     while (f >> str)
//     {
//         std::cout << str;
//     }
//     }
// }

// #include <cstdlib>
// #include <iostream>
// #include <fstream>
// #include <sstream>

// using namespace std;

// // int main()
// // {
// // char ch;
// // ifstream f;
// // f.open("/dev/ttyUSB0");
// // while (f.get(ch))
// // {
// //     // f.get(ch);
// // cout << ch << " ";
// // }
// // return 0;
// // }

// int main()
// {   
//     char c;
//     ifstream usbRead;
//     usbRead.open("/dev/ttyUSB0");
//     char reponse[20];
//     int i = 0;
//     while(usbRead.get(c))
//     {
//         reponse[i] = c;
//         i++;
//         if(c == '\n') break;
//     }
//     std::cout << reponse;
//     return 0;
// }