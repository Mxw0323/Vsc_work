#include "Q_learning.h"

// 主程序
bool QLearning::Excute()
{
    // 初始化
    bool flag = QLearning::Ind2Sub(start_node_ind_, &start_node_sub_);
    flag = QLearning::Ind2Sub(goal_node_ind_, &goal_node_sub_);

    //定义障碍物栅格
    QLearning::DefObsZone();

    // 构造奖励矩阵
    QLearning::DefReward();

    // 开始搜索
    QLearning::SearchPath();

    // 打印路径及画图
    QLearning::PrintAndPLot();

    return true;
}

/* 搜索路径 */
void QLearning::SearchPath()
{
    for (int i = 0; i < iter_max; i++)
    {
        std::srand(time(0));  // 用当前时间来设定rand函数所用的随机数产生演算法的种子值
        std::cout << "----------------- iter: " << i << std::endl;

        // 初始化当前节点状态
        int node_now_ind = start_node_ind_;
        double q_max = 0;
        
        // 重复运行，直至到达goalPos状态
        while (true)
        {
            // 1. 生成当前节点状态的所有可能行动
            std::vector<QLearning::NodeInfo> actions_now_info = reward_matrix_[node_now_ind];

            // 2. 在这些索引中，随机选择一个动作，并把它作为下一状态
            int lens = actions_now_info.size();
            int rand_idx = std::rand() % lens;
            QLearning::NodeInfo action_now_info = actions_now_info[rand_idx];

            // 3. 找到下一个状态所有可能的动作
            std::vector<QLearning::NodeInfo> actions_next_info = reward_matrix_[action_now_info.node_ind];

            // 4. 在下一个状态的这些所有可能动作中，找到最大的Q值
            double q_max = 0;
            for (int i = 0; i < actions_next_info.size(); i++)
            {
                q_max = std::max(q_max, q_mat_[action_now_info.node_ind][actions_next_info[i].node_ind]);
            }

            // 5. 更新Q值表
            q_mat_[node_now_ind][action_now_info.node_ind] =
                (1 - alpha_) * q_mat_[node_now_ind][action_now_info.node_ind] +
                alpha_ * (reward_matrix_[node_now_ind][rand_idx].reward + gamma_ * q_max);

            // 6. 检查是否到达终点
            if (node_now_ind == goal_node_ind_)
            {
                break;
            }
            else
            {
                node_now_ind = action_now_info.node_ind;  // 把下一状态设置为当前状态
            }
        }
        // 更新路径
        QLearning::UpdatePath(i);
    }
}

/* 更新路径 */
void QLearning::UpdatePath(int iter)
{
    int start = start_node_ind_;
    std::vector<int> path{start};
    int move = 0;
    std::vector<double> q_mat_i;
    // 循环迭代直至找s到终点Goal
    while (move != goal_node_ind_)
    {
        // 调用函数，生成Q矩阵排序索引
        q_mat_i = q_mat_[start];
        std::vector<size_t> sort_idx = sort_indexes_e(q_mat_i);
        move = sort_idx[sort_idx.size() - 1];

        // 判断move是否已经存在于path里面
        int num = std::count(path.begin(), path.end(), move);
        int step = 2;
        while (num >= 1)
        {
            move = sort_idx[sort_idx.size() - step]; // 降序索引
            num = std::count(path.begin(), path.end(), move);
            step++;
        }

        // 把下一个动作加到路径中
        path.push_back(move);
        start = move;
    }
    path_list_.insert(std::make_pair(iter, path));

    // 计算长度
    double len = 0;
    for (int i = 0; i < path.size() - 1; i++)
    {
        SubPos node_sub1, node_sub2;
        bool flag = QLearning::Ind2Sub(path[i], &node_sub1);
        flag = QLearning::Ind2Sub(path[i + 1], &node_sub2);
        double delt_len = std::sqrt(std::pow((node_sub1.row - node_sub2.row), 2) +
                                    std::pow((node_sub1.col - node_sub2.col), 2));
        len = len + delt_len;
    }
    lens_iter_.push_back(len);
}

/* 线性索引坐标转为行列坐标, base-0 */
bool QLearning::Ind2Sub(int ind, SubPos *sub)
{
    if (ind < 0 || ind >= rows_ * cols_)
    {
        return false;
    }
    else
    {
        sub->row = ind % rows_; // 行, base-0
        sub->col = ind / rows_; // 列，base-0
        return true;
    }
}

// /* 行列坐标转为线性索引坐标, base-0 */
bool QLearning::Sub2Ind(int *ind, SubPos sub)
{
    if (sub.row < 0 || sub.col < 0 || sub.row > rows_ - 1 || sub.col > cols_ - 1)
    {
        return false;
    }
    else
    {
        *ind = sub.col * rows_ + sub.row;
        return true;
    }
}

/* 定义障碍物区域 */
void QLearning::DefObsZone()
{
    // 全部初始化为1
    std::vector<int> tmp = std::vector<int>(cols_, 1);
    for (int i = 0; i < rows_; i++)
    {
        obs_matrix_.push_back(tmp);
    }

    for (int i = 1; i < 5; i++)
    {
        obs_matrix_[1][i] = 2;
    }
    // field(5,3:5) = 2;
    for (int i = 2; i < 5; i++)
    {
        obs_matrix_[4][i] = 2;
    }

    // field(4,11:15) = 2;
    for (int i = 10; i < 15; i++)
    {
        obs_matrix_[3][i] = 2;
    }

    // field(2,13:17) = 2;
    for (int i = 12; i < 17; i++)
    {
        obs_matrix_[1][i] = 2;
    }
    // field(7,14:18) = 2;
    for (int i = 13; i < 18; i++)
    {
        obs_matrix_[6][i] = 2;
    }
    // field(3:10,19) = 2;
    for (int i = 2; i < 10; i++)
    {
        obs_matrix_[i][18] = 2;
    }
    // field(15:18,19) = 2;
    for (int i = 14; i < 18; i++)
    {
        obs_matrix_[i][18] = 2;
    }
    // field(3:10,19) = 2;
    for (int i = 2; i < 10; i++)
    {
        obs_matrix_[i][18] = 2;
    }
    // field(3:10,7) = 2;
    for (int i = 2; i < 10; i++)
    {
        obs_matrix_[i][6] = 2;
    }
    // field(9:19,2) = 2;
    for (int i = 8; i < 19; i++)
    {
        obs_matrix_[i][1] = 2;
    }
    // field(15:17,7) = 2;
    for (int i = 14; i < 17; i++)
    {
        obs_matrix_[i][6] = 2;
    }
    // field(10,3:7) = 2;
    for (int i = 2; i < 7; i++)
    {
        obs_matrix_[9][i] = 2;
    }
    // field(13,5:8) = 2;
    for (int i = 4; i < 8; i++)
    {
        obs_matrix_[12][i] = 2;
    }
    // field(6:8,4) = 2;
    for (int i = 5; i < 8; i++)
    {
        obs_matrix_[i][3] = 2;
    }
    // field(13:18,4) = 2;
    for (int i = 12; i < 18; i++)
    {
        obs_matrix_[i][3] = 2;
    }
    // field(6:16,10) = 2;
    for (int i = 5; i < 16; i++)
    {
        obs_matrix_[i][9] = 2;
    }
    // field(19:20,10) = 2;
    for (int i = 18; i < 20; i++)
    {
        obs_matrix_[i][9] = 2;
    }
    // field(17,13:17) = 2;
    for (int i = 12; i < 17; i++)
    {
        obs_matrix_[16][i] = 2;
    }
    // field(18,6:11) = 2;
    for (int i = 5; i < 11; i++)
    {
        obs_matrix_[17][i] = 2;
    }
    // field(10:17,13) = 2;
    for (int i = 9; i < 17; i++)
    {
        obs_matrix_[i][12] = 2;
    }
    // field(10,13:17) = 2;
    for (int i = 12; i < 17; i++)
    {
        obs_matrix_[9][i] = 2;
    }
    // field(14,15:19) = 2;
    for (int i = 14; i < 19; i++)
    {
        obs_matrix_[13][i] = 2;
    }
    // field(7,12) = 2;
    obs_matrix_[6][11] = 2;
}

/* 构造奖励矩阵 */
void QLearning::DefReward()
{
    // 根据邻接点信息构造奖励矩阵
    std::vector<QLearning::NodeInfo> childs_node_info;
    QLearning::NodeInfo node_info;
    for (int i = 0; i < rows_ * cols_; i++)
    {
        int parent_node_ind = i;
        std::vector<int> child_nodes_ind = QLearning::GetChildNodes(parent_node_ind);
        childs_node_info.clear();

        for (int k = 0; k < child_nodes_ind.size(); k++)
        {
            if (std::abs(child_nodes_ind[k] - parent_node_ind) == 1 ||
                std::abs(child_nodes_ind[k] - parent_node_ind) == rows_)
            {
                // 横纵运动
                node_info.node_ind = child_nodes_ind[k];
                node_info.reward = 1;
                childs_node_info.push_back(node_info);
            }
            else if (std::abs(child_nodes_ind[k] - parent_node_ind) == rows_ + 1 ||
                     std::abs(child_nodes_ind[k] - parent_node_ind) == rows_ - 1)
            {
                // 斜向运动
                node_info.node_ind = child_nodes_ind[k];
                node_info.reward = 1 / 1.414;
                childs_node_info.push_back(node_info);
            }
        }
        reward_matrix_.insert(std::make_pair(parent_node_ind, childs_node_info));
    }

    // 初始化Q
    std::vector<double> tmp = std::vector<double>(rows_ * cols_, 0.5);
    for (int i = 0; i < rows_ * cols_; i++)
    {
        q_mat_.push_back(tmp);
    }
}

// /* 打印及画图 */
void QLearning::PrintAndPLot()
{
    // 1. 找到迭代过程中的最优解
    matplotlibcpp::figure(1);
    std::cout << "lens_iter_.size(): " << lens_iter_.size() << std::endl;
    int iter_opt = 0;
    double lens_opt = 1e6;
    for (int i = 0; i < lens_iter_.size(); i++)
    {
        if (lens_iter_[i] < lens_opt)
        {
            lens_opt = lens_iter_[i];
            iter_opt = i;
        }
        std::cout << lens_iter_[i] << std::endl;
    }
    matplotlibcpp::plot(lens_iter_);

    // 2. 生成x/y坐标
        matplotlibcpp::figure(2);
    std::vector<int> final_path_row, final_path_col;
    std::vector<double> final_path_x, final_path_y;
    std::vector<int> path_iter = path_list_[iter_opt];
    for (int i = 0; i < path_iter.size(); i++)
    {
        SubPos final_sub_pos;
        bool flag = QLearning::Ind2Sub(path_iter[i], &final_sub_pos);
        final_path_row.push_back(final_sub_pos.row);
        final_path_col.push_back(final_sub_pos.col);

        // 行列坐标转为xy坐标
        // 注意，sub是base-0
        final_path_y.push_back((rows_ - final_sub_pos.row) - 0.5);
        final_path_x.push_back(final_sub_pos.col + 0.5);
    }

    // 3.画粗线网格
    std::vector<double> x_list(2, 0);
    std::vector<double> y_list(2, 0);
    for (int i = 0; i <= rows_; i++)
    {
        x_list.clear();
        y_list.clear();
        x_list.push_back(0);
        x_list.push_back(cols_);
        y_list.push_back(i);
        y_list.push_back(i);
        //
        matplotlibcpp::plot(x_list, y_list, "k");
    }
    for (int i = 0; i <= cols_; i++)
    {
        x_list.clear();
        y_list.clear();
        y_list.push_back(0);
        y_list.push_back(rows_);
        x_list.push_back(i);
        x_list.push_back(i);
        //
        matplotlibcpp::plot(x_list, y_list, "k");
    }

    // 4. 填充障碍物区间
    std::map<std::string, std::string> keywords = {{"color", "k"}};
    for (int i = 0; i < rows_; i++)
    {
        for (int j = 0; j < cols_; j++)
        {
            if (obs_matrix_[i][j] == 2)
            {
                // 转成xy
                x_list.clear();
                y_list.clear();
                // 左下点
                x_list.push_back(j);
                y_list.push_back((rows_ - i) - 1);
                // 左上点
                x_list.push_back(j);
                y_list.push_back((rows_ - i));
                // 右上点
                x_list.push_back(j + 1);
                y_list.push_back((rows_ - i));
                // 右下点
                x_list.push_back(j + 1);
                y_list.push_back((rows_ - i - 1));

                matplotlibcpp::fill(x_list, y_list, keywords);
            }
        }
    }

    // 5. 填充起点和终点
    keywords = {{"color", "g"}};
    x_list.clear();
    y_list.clear();
    // 左下点
    x_list.push_back(start_node_sub_.col);
    y_list.push_back((rows_ - start_node_sub_.row) - 1);
    // 左上点
    x_list.push_back(start_node_sub_.col);
    y_list.push_back((rows_ - start_node_sub_.row));
    // 右上点
    x_list.push_back(start_node_sub_.col + 1);
    y_list.push_back((rows_ - start_node_sub_.row));
    // 右下点
    x_list.push_back(start_node_sub_.col + 1);
    y_list.push_back((rows_ - start_node_sub_.row - 1));
    matplotlibcpp::fill(x_list, y_list, keywords);

    keywords = {{"color", "r"}};
    // 转成xy
    x_list.clear();
    y_list.clear();
    // 左下点
    x_list.push_back(goal_node_sub_.col);
    y_list.push_back((rows_ - goal_node_sub_.row) - 1);
    // 左上点
    x_list.push_back(goal_node_sub_.col);
    y_list.push_back((rows_ - goal_node_sub_.row));
    // 右上点
    x_list.push_back(goal_node_sub_.col + 1);
    y_list.push_back((rows_ - goal_node_sub_.row));
    // 右下点
    x_list.push_back(goal_node_sub_.col + 1);
    y_list.push_back((rows_ - goal_node_sub_.row - 1));
    matplotlibcpp::fill(x_list, y_list, keywords);

    // 6. 画最终路径
    keywords = {{"color", "b"}, {"linewidth", "2"}};
    matplotlibcpp::plot(final_path_x, final_path_y, keywords);

    // matplotlibcpp::plot(lens);

    // 限制横纵坐标范围
    matplotlibcpp::xlim(0, cols_);
    matplotlibcpp::ylim(0, rows_);

    matplotlibcpp::show();
}

/* 获取可能的子节点 */
std::vector<int> QLearning::GetChildNodes(int parent_node_ind)
{
    //初始化输出
    std::vector<int> child_nodes_ind;

    //  将父节点的线性索引转为行列坐标
    SubPos parent_node_sub;
    QLearning::Ind2Sub(parent_node_ind, &parent_node_sub);

    // 定义8个周边子节点的坐标偏置量
    int offset[8][2] = {{0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}};

    // 选取父节点周边8个节点作为备选子节点，线性化坐标,排除超过边界之外的、位于障碍区的
    for (int i = 0; i < 8; i++)
    {
        // 周边子节点行列坐标
        SubPos child_node_sub;
        child_node_sub.row = parent_node_sub.row + offset[i][0];
        child_node_sub.col = parent_node_sub.col + offset[i][1];

        // 周边子节点线性坐标
        int child_node_ind = 0; // 子节点线性坐标
        bool flag = QLearning::Sub2Ind(&child_node_ind, child_node_sub);
        if (flag == false)
        {
            continue;
        }

        if (!(child_node_ind < 0 || child_node_ind >= rows_ * cols_) &&
            (obs_matrix_[child_node_sub.row][child_node_sub.col] != 2))
        {
            // 未越界，非障碍物区间
            child_nodes_ind.push_back(child_node_ind);
        }
    }
    return child_nodes_ind;
}
