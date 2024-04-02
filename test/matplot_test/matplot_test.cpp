#include "matplotlibcpp.h"
#include <cmath>

namespace plt = matplotlibcpp;

void test1()
{
    plt::plot({1,3,2,4});
    plt::show();
}

void test2()
{
        // 数据处理
    int n = 5000;
    std::vector<double> x(n), y(n), z(n), w(n,2);
    for(int i=0; i<n; ++i) {
        x.at(i) = i*i;
        y.at(i) = sin(2*M_PI*i/360.0);
        z.at(i) = log(i);
    }

    // 设置分辨率
    plt::figure_size(1200, 780);
    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(x, y);
    // Plot a red dashed line from given x and y data.
    plt::plot(x, w,"r--");
    // Plot a line whose name will show up as "log(x)" in the legend.
    plt::named_plot("log(x)", x, z);
    // 设置x轴
    plt::xlim(0, 1000*1000);
    // 图表标题
    plt::title("Sample figure");
    // 添加图例
    plt::legend();
    // 保存为照片
    plt::save("./basic.png");
}
int main()
{
    test1();
    // test2();
    return 0;
}
