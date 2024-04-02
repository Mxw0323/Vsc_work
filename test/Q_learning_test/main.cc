#include <iostream>
#include <fstream>
#include <string>
#include "Q_learning.h"
int main()
{
    std::shared_ptr<QLearning> a_star = std::make_shared<QLearning>();
    a_star->Excute();
    return 1;
}
