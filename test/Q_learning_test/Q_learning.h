#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>
#include <unordered_map>
#include <matplotlibcpp.h>
#include <stdlib.h>
#include <algorithm>
#include <ctime>

class QLearning
{
private:
    // 定义行列坐标
    typedef struct
    {
        int row = -1;
        int col = -1;
    } SubPos;

    typedef struct
    {
        int node_ind = -1;
        double reward = -1;
    } NodeInfo;

    int rows_ = 20;
    int cols_ = 20;
    int start_node_ind_ = 3;
    int goal_node_ind_ = rows_ * cols_ - 3;
    SubPos start_node_sub_;
    SubPos goal_node_sub_;

    // Q矩阵、迭代次数、奖励矩阵等参数
    std::vector<std::vector<double>> q_mat_;
    double gamma_ = 0.6;//回报折扣系数
    double alpha_ = 0.4;//学习率
    int iter_max = 200;  
    std::vector<double> lens_iter_;
    std::unordered_map<int, std::vector<QLearning::NodeInfo>> reward_matrix_;
    std::vector<std::vector<int>> obs_matrix_;
    std::unordered_map<int, std::vector<int>> path_list_;

    // 排序函数模板
    template <typename T>
    std::vector<size_t> sort_indexes_e(std::vector<T> &v)
    {
        std::vector<size_t> idx(v.size());
        std::iota(idx.begin(), idx.end(), 0);
        std::sort(idx.begin(), idx.end(),
                  [&v](size_t i1, size_t i2)
                  { return v[i1] < v[i2]; });
        return idx;
    }

public:
    QLearning(/* args */){};
    ~QLearning(){};

    // 主函数
    bool Excute();

    // 定义障碍物区域
    void DefObsZone();

    // 构造奖励矩阵
    void DefReward();

    // 线性坐标转为行列坐标
    bool Ind2Sub(int ind, SubPos *sub);

    // 行列坐标转为线性坐标
    bool Sub2Ind(int *ind, SubPos sub);

    // 路径搜索
    void SearchPath();

    // 更新路径
    void UpdatePath(int i);

    // 打印和画图
    void PrintAndPLot();

    // 获得某节点周边可行子节点
    std::vector<int> GetChildNodes(int parent_node);
};
