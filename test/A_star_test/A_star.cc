#include "A_star.h"

/* ������ */
bool AStar::Excute()
{
    // ��ʼ��
    bool flag = AStar::Ind2Sub(start_node_ind_, &start_node_sub_);
    flag = AStar::Ind2Sub(goal_node_ind_, &goal_node_sub_);

    //�����ϰ���դ��
    AStar::DefObsZone();

    // Ԥ����
    AStar::PreProcess();

    // ��ʼ����
    AStar::SearchPath();

    // ��ӡ·������ͼ
    AStar::PrintAndPLot();

    return true;
}


/* Ԥ���� */
void AStar::PreProcess()
{
    // ʼ��open_list_��close_list_
    open_list_ = std::make_shared<std::unordered_map<int, AStar::NodesCost>>();
    close_list_ = std::make_shared<std::unordered_map<int, AStar::NodesCost>>();
    path_list_ = std::make_shared<std::unordered_map<int, std::vector<int>>>();
    std::vector<int> child_nodes;
    child_nodes = AStar::GetChildNodes(start_node_ind_);
    std::cout << "child_nodes start: " << std::endl;

    for (int i = 0; i < child_nodes.size(); i++)
    {
        std::cout << child_nodes[i] << std::endl;
    }

    std::cout << "child_nodes end: " << std::endl;

    for (int i = 0; i < child_nodes.size(); i++)
    {
        // ��open_list_��ֵ
        NodesCost ini_nodes_info;
        SubPos node_sub;
        bool flag = AStar::Ind2Sub(child_nodes[i], &node_sub);
        ini_nodes_info.g = std::sqrt(std::pow((start_node_sub_.row - node_sub.row), 2) +
                                     std::pow((start_node_sub_.col - node_sub.col), 2));
        ini_nodes_info.h = std::abs((goal_node_sub_.row - node_sub.row)) +
                           std::abs((goal_node_sub_.col - node_sub.col));
        ini_nodes_info.f = ini_nodes_info.g + ini_nodes_info.h;
        open_list_->insert(std::make_pair(child_nodes[i], ini_nodes_info));
    }

    for (int i = 0; i < rows_ * cols_; i++)
    {
        // ��path_list_��ֵ
        std::vector<int> path_list_tmp(1, i);
        path_list_->insert(std::make_pair(i, path_list_tmp));
    }
}


/* ����·�� */
void AStar::SearchPath()
{
    // ��openList��ʼ�����ƶ�������С�Ľڵ�
    int f_min_node_ind = AStar::FindFminNode();

    while (true)
    {
        // �ҳ����ڵ��8���ӽڵ㣬�ϰ���ڵ���inf��
        std::vector<int> child_nodes;
        child_nodes = AStar::GetChildNodes(f_min_node_ind);

        // �ж���Щ�ӽڵ��Ƿ���openList�У����ڣ���Ƚϸ��£�û����׷�ӵ�openList��
        for (int i = 0; i < child_nodes.size(); i++)
        {
            //������ۺ���
            NodesCost child_node_cost;
            SubPos node_sub_f_min, child_node_sub;
            bool flag = AStar::Ind2Sub(f_min_node_ind, &node_sub_f_min);
            flag = AStar::Ind2Sub(child_nodes[i], &child_node_sub);

            child_node_cost.g = std::sqrt(std::pow((node_sub_f_min.row - child_node_sub.row), 2) +
                                          std::pow((node_sub_f_min.col - child_node_sub.col), 2)) +
                                (*open_list_)[f_min_node_ind].g;
            child_node_cost.h = std::abs((goal_node_sub_.row - child_node_sub.row)) +
                                std::abs((goal_node_sub_.col - child_node_sub.col));
            child_node_cost.f = child_node_cost.g + child_node_cost.h;

            // �жϸýڵ��Ƿ����open_list��
            int in_flag = open_list_->count(child_nodes[i]);
            if (in_flag > 0)
            {
                if (child_node_cost.f < (*open_list_)[child_nodes[i]].f)
                {
                    // ����open_list
                    (*open_list_)[child_nodes[i]] = child_node_cost;

                    // ����path
                    (*path_list_)[child_nodes[i]] = (*path_list_)[f_min_node_ind];
                    (*path_list_)[child_nodes[i]].push_back(child_nodes[i]);
                }
            }
            else
            {
                // ����open_list��path
                open_list_->insert(std::make_pair(child_nodes[i], child_node_cost)); //׷�ӵ�open_list
                (*path_list_)[child_nodes[i]] = (*path_list_)[f_min_node_ind];
                (*path_list_)[child_nodes[i]].push_back(child_nodes[i]);
            }
        }
        // ��openList�Ƴ��ƶ�������С�Ľڵ㵽closeList
        close_list_->insert(std::make_pair(f_min_node_ind, (*open_list_)[f_min_node_ind]));
        open_list_->erase(f_min_node_ind);

        // ������������openList�����ƶ�������С�Ľڵ�
        f_min_node_ind = AStar::FindFminNode();

        // �ж��Ƿ��������յ�
        if (f_min_node_ind == goal_node_ind_)
        {
            close_list_->insert(std::make_pair(f_min_node_ind, (*open_list_)[f_min_node_ind]));
            break;
        }
    }
}

/* ��openlist������С��f��Ӧ�Ľڵ� */
int AStar::FindFminNode()
{
    int f_min_node = -1;
    double f_min = 10000.0;
    std::unordered_map<int, AStar::NodesCost>::iterator it; // ���������
    for (it = open_list_->begin(); it != open_list_->end(); it++)
    {
        if ((*it).second.f < f_min)
        {
            f_min = (*it).second.f;
            f_min_node = (*it).first;
        }
    }
    return f_min_node;
}



/* ��ȡ���ܵ��ӽڵ� */
std::vector<int> AStar::GetChildNodes(int parent_node_ind)
{
    //��ʼ�����
    std::vector<int> child_nodes_ind;

    //  �����ڵ����������תΪ��������
    SubPos parent_node_sub;
    AStar::Ind2Sub(parent_node_ind, &parent_node_sub);

    // ����8���ܱ��ӽڵ������ƫ����
    int offset[8][2] = {{0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}};

    // ѡȡ���ڵ��ܱ�8���ڵ���Ϊ��ѡ�ӽڵ㣬���Ի�����,�ų������߽�֮��ġ�λ���ϰ����ġ�λ��closeList�е�
    for (int i = 0; i < 8; i++)
    {
        // �ܱ��ӽڵ���������
        SubPos child_node_sub;
        child_node_sub.row = parent_node_sub.row + offset[i][0];
        child_node_sub.col = parent_node_sub.col + offset[i][1];

        // �ܱ��ӽڵ���������
        int child_node_ind = 0; // �ӽڵ���������
        bool flag = AStar::Sub2Ind(&child_node_ind, child_node_sub);
        if (flag == false)
        {
            continue;
        }

        if (!(child_node_ind < 0 || child_node_ind >= rows_ * cols_) &&
            ((*obs_matrix_)[child_node_sub.row][child_node_sub.col] != 2) &&
            close_list_->count(child_node_ind) == 0)
        {
            // δԽ�磬���ϰ������䣬��λ��close_list��
            child_nodes_ind.push_back(child_node_ind);
        }
    }
    return child_nodes_ind;
}

/* ������������תΪ��������, base-0 */
bool AStar::Ind2Sub(int ind, SubPos *sub)
{
    if (ind < 0 || ind >= rows_ * cols_)
    {
        std::cout << "ERROR! ������������Խ�磡" << std::endl;
        return false;
    }
    else
    {
        sub->row = (ind + 1) % rows_ - 1; // ��, base-0
        sub->col = (ind + 1) / rows_;     // �У�base-0
        return true;
    }
}

/* ��������תΪ������������, base-0 */
bool AStar::Sub2Ind(int *ind, SubPos sub)
{
    if (sub.row < 0 || sub.col < 0 || sub.row > rows_ - 1 || sub.col > cols_ - 1)
    {
        std::cout << "ERROR! ������������Խ�磡" << std::endl;
        return false;
    }
    else
    {
        *ind = sub.col * rows_ + sub.row;
        return true;
    }
}

/* �����ϰ������� */
void AStar::DefObsZone()
{
    obs_matrix_ = std::make_shared<std::vector<std::vector<int>>>();

    // ȫ����ʼ��Ϊ1
    std::vector<int> tmp = std::vector<int>(cols_, 1);
    for (int i = 0; i < rows_; i++)
    {
        obs_matrix_->push_back(tmp);
    }

    std::cout << "size: " << obs_matrix_->size() << std::endl;

    for (int i = 1; i < 5; i++)
    {
        (*obs_matrix_)[1][i] = 2;
    }
    // field(5,3:5) = 2;
    for (int i = 2; i < 5; i++)
    {
        (*obs_matrix_)[4][i] = 2;
    }

    // field(4,11:15) = 2;
    for (int i = 10; i < 15; i++)
    {
        (*obs_matrix_)[3][i] = 2;
    }

    // field(2,13:17) = 2;
    for (int i = 12; i < 17; i++)
    {
        (*obs_matrix_)[1][i] = 2;
    }
    // field(7,14:18) = 2;
    for (int i = 13; i < 18; i++)
    {
        (*obs_matrix_)[6][i] = 2;
    }
    // field(3:10,19) = 2;
    for (int i = 2; i < 10; i++)
    {
        (*obs_matrix_)[i][18] = 2;
    }
    // field(15:18,19) = 2;
    for (int i = 14; i < 18; i++)
    {
        (*obs_matrix_)[i][18] = 2;
    }
    // field(3:10,19) = 2;
    for (int i = 2; i < 10; i++)
    {
        (*obs_matrix_)[i][18] = 2;
    }
    // field(3:10,7) = 2;
    for (int i = 2; i < 10; i++)
    {
        (*obs_matrix_)[i][6] = 2;
    }
    // field(9:19,2) = 2;
    for (int i = 8; i < 19; i++)
    {
        (*obs_matrix_)[i][1] = 2;
    }
    // field(15:17,7) = 2;
    for (int i = 14; i < 17; i++)
    {
        (*obs_matrix_)[i][6] = 2;
    }
    // field(10,3:7) = 2;
    for (int i = 2; i < 7; i++)
    {
        (*obs_matrix_)[9][i] = 2;
    }
    // field(13,5:8) = 2;
    for (int i = 4; i < 8; i++)
    {
        (*obs_matrix_)[12][i] = 2;
    }
    // field(6:8,4) = 2;
    for (int i = 5; i < 8; i++)
    {
        (*obs_matrix_)[i][3] = 2;
    }
    // field(13:18,4) = 2;
    for (int i = 12; i < 18; i++)
    {
        (*obs_matrix_)[i][3] = 2;
    }
    // field(6:16,10) = 2;
    for (int i = 5; i < 16; i++)
    {
        (*obs_matrix_)[i][9] = 2;
    }
    // field(19:20,10) = 2;
    for (int i = 18; i < 20; i++)
    {
        (*obs_matrix_)[i][9] = 2;
    }
    // field(17,13:17) = 2;
    for (int i = 12; i < 17; i++)
    {
        (*obs_matrix_)[16][i] = 2;
    }
    // field(18,6:11) = 2;
    for (int i = 5; i < 11; i++)
    {
        (*obs_matrix_)[17][i] = 2;
    }
    // field(10:17,13) = 2;
    for (int i = 9; i < 17; i++)
    {
        (*obs_matrix_)[i][12] = 2;
    }
    // field(10,13:17) = 2;
    for (int i = 12; i < 17; i++)
    {
        (*obs_matrix_)[9][i] = 2;
    }
    // field(14,15:19) = 2;
    for (int i = 14; i < 19; i++)
    {
        (*obs_matrix_)[13][i] = 2;
    }
    // field(7,12) = 2;
    (*obs_matrix_)[6][11] = 2;
}

/* ��ӡ����ͼ */
void AStar::PrintAndPLot()
{
    // 1. ��ӡ��������
    std::vector<int> final_path = (*path_list_)[goal_node_ind_];
    std::cout << "A* �㷨����·���ɹ���·�����£�" << std::endl;
    for (int i = 0; i < final_path.size(); i++)
    {
        std::cout << final_path[i] << " �� ";
    }

    std::cout << std::endl;

    // 2. ��ӡ��������
    std::vector<int> final_path_row, final_path_col;
    std::vector<double> final_path_x, final_path_y;
    for (int i = 0; i < final_path.size(); i++)
    {
        SubPos final_sub_pos;
        bool flag = AStar::Ind2Sub(final_path[i], &final_sub_pos);
        final_path_row.push_back(final_sub_pos.row);
        final_path_col.push_back(final_sub_pos.col);

        // ��������תΪxy����
        // ע�⣬sub��base-0
        final_path_y.push_back((rows_ - final_sub_pos.row) - 0.5);
        final_path_x.push_back(final_sub_pos.col + 0.5);

        std::cout << "[" << final_sub_pos.row << "," << final_sub_pos.col << "]"
                  << " �� ";
    }
    std::cout << std::endl;


    // 3.����������
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

    // 4. ����ϰ�������
    std::map<std::string, std::string> keywords = {{"color", "k"}};
    for (int i = 0; i < rows_; i++)
    {
        for (int j = 0; j < cols_; j++)
        {
            if ((*obs_matrix_)[i][j] == 2)
            {
                // ת��xy
                x_list.clear();
                y_list.clear();
                // ���µ�
                x_list.push_back(j);
                y_list.push_back((rows_ - i) - 1);
                // ���ϵ�
                x_list.push_back(j);
                y_list.push_back((rows_ - i));
                // ���ϵ�
                x_list.push_back(j + 1);
                y_list.push_back((rows_ - i));
                // ���µ�
                x_list.push_back(j + 1);
                y_list.push_back((rows_ - i - 1));

                matplotlibcpp::fill(x_list, y_list, keywords);
            }
        }
    }

    // 5. ��������յ�
    keywords = {{"color", "g"}};
    x_list.clear();
    y_list.clear();
    // ���µ�
    x_list.push_back(start_node_sub_.col);
    y_list.push_back((rows_ - start_node_sub_.row) - 1);
    // ���ϵ�
    x_list.push_back(start_node_sub_.col);
    y_list.push_back((rows_ - start_node_sub_.row));
    // ���ϵ�
    x_list.push_back(start_node_sub_.col + 1);
    y_list.push_back((rows_ - start_node_sub_.row));
    // ���µ�
    x_list.push_back(start_node_sub_.col + 1);
    y_list.push_back((rows_ - start_node_sub_.row - 1));
    matplotlibcpp::fill(x_list, y_list, keywords);

    keywords = {{"color", "r"}};
    // ת��xy
    x_list.clear();
    y_list.clear();
    // ���µ�
    x_list.push_back(goal_node_sub_.col);
    y_list.push_back((rows_ - goal_node_sub_.row) - 1);
    // ���ϵ�
    x_list.push_back(goal_node_sub_.col);
    y_list.push_back((rows_ - goal_node_sub_.row));
    // ���ϵ�
    x_list.push_back(goal_node_sub_.col + 1);
    y_list.push_back((rows_ - goal_node_sub_.row));
    // ���µ�
    x_list.push_back(goal_node_sub_.col + 1);
    y_list.push_back((rows_ - goal_node_sub_.row - 1));
    matplotlibcpp::fill(x_list, y_list, keywords);



    // 6. ������·��
    keywords = {{"color", "b"}, {"linewidth", "2"}};
    matplotlibcpp::plot(final_path_x, final_path_y, keywords);

    // ���ƺ������귶Χ
    matplotlibcpp::xlim(0, cols_);
    matplotlibcpp::ylim(0, rows_);

    matplotlibcpp::show();
}
