#include<iostream>
#include "process.h"
#include "show_result.h"
using std::cout, std::endl;

int main(int argc, char const *argv[])
{
    cout << "Planning starts." << endl;
    Process my_process;
    my_process.planProcess();
    cout << "Planning ends." << endl;

    cout << "Showing result starts." << endl;
    ShowResult my_show_result;
    my_show_result.drawResult();
    cout << "Showing result ends." << endl;
    

    return 0;
}
