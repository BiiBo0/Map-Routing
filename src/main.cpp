#include <iostream>          
#include "core/tests_run.h"  

using namespace std;

int main()
{
    Profiler timer("mainFunction");

    cout << "\nEnter your choice: [1] Trial Cases [2] Sample Test Cases [3] Complete Test Cases   [4]Bouns Test  ... [any key for exit] ";
    int level;
    cin >> level;
    
    do
    {
        ExcuteProblem(level);

        cout << "\nEnter your choice: [1] Trial Cases [2] Sample Test Cases [3] Complete Test Cases   [4]Bouns Test  ... [any key for exit] ";
        cin >> level;

    } while (level == 1 || level == 2 || level == 3|| level==4);  
    
    return 0;
}
