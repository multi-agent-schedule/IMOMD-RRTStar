/* Copyright (C) 2013-2025, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/)
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 */
/*******************************************************************************
 * File:        imomd_rrt_star.cpp
 *
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     02/20/2022
 *
 * Description: Informable Multi-Objective and Multi-Directiobal RRT*
 *******************************************************************************/
#include <iostream>

#include "imomd_rrt_star/imomd_rrt_star.h"

// std::string locationToGeoJSON(const location_t &loc)
// {
//     std::ostringstream oss;
//     std::string geometry = " \"geometry\": { \"type\": \"Point\", "
//                            "\"coordinates\": [" +
//                            std::to_string(loc.longitude) + ", " + std::to_string(loc.latitude) + "] }";
//     std::string properties;
//     if (loc.id >= 0)
//     {
//         properties = ", \"properties\": { \"id\": " + std::to_string(loc.id) + " }";
//     }

//     oss << "{"
//         << "\"type\": \"Feature\", " << geometry << properties << "}";
//     return oss.str();
// }

// std::string locationsToGeoJSON(const std::vector<location_t> &locs)
// {
//     std::ostringstream oss;
//     oss << "{"
//         << "\"type\": \"FeatureCollection\","
//         << "\"features\": [";

//     bool first = true;
//     for (const auto &loc : locs)
//     {
//         if (!first)
//         {
//             oss << ",";
//         }
//         first = false;
//         oss << locationToGeoJSON(loc);
//     }

//     oss << "]}";
//     std::ofstream fout("locations.geojson");
//     fout << oss.str();
//     fout.close();

//     return oss.str();
// }

// std::string lineStringGeometry(const location_t &from, const location_t &to)
// {
//     std::ostringstream oss;
//     oss << "{\"type\": \"LineString\", \"coordinates\": ["
//         << "[" << from.longitude << ", " << from.latitude << "], "
//         << "[" << to.longitude << ", " << to.latitude << "]]}";
//     return oss.str();
// }

// std::string lineStringFeature(const location_t &from, const location_t &to, double cost = 0)
// {
//     std::ostringstream oss;
//     oss << "{\"type\": \"Feature\", "
//         << "\"geometry\": " << lineStringGeometry(from, to) << ", "
//         << "\"properties\": {\"cost\": " << cost << "}}";
//     return oss.str();
// }

// std::string edgesToGeoJSON(
//     const std::vector<std::unordered_map<size_t, double>> &edges,
//     const std::vector<location_t> &nodes)
// {
//     if (edges.empty())
//         return "{}";

//     std::ostringstream collection;
//     collection << "{\"type\": \"FeatureCollection\", "
//                << "\"features\": [";
//     bool first = true;
//     for (const auto &loc : nodes)
//     {
//         if (!first)
//         {
//             collection << ",";
//         }
//         first = false;
//         collection << locationToGeoJSON(loc);
//     }

//     // for (auto& edge_map : edges) {
//     //     for (auto& edge : edge_map) {
//     //         size_t from = edge.first;
//     //         size_t to = edge.second;
//     //         if (from >= nodes.size() || to >= nodes.size()) continue;
//     //         const location_t& from_loc = nodes.at(from);
//     //         const location_t& to_loc = nodes.at(to);
//     //         std::string line_feature = lineStringFeature(from_loc, to_loc, edge.second);
//     //         collection << "," << line_feature;
//     //     }
//     // }
//     collection << "]}";

//     std::ofstream fout("edges.geojson");
//     if (!fout.is_open())
//     {
//         return "";
//     }
//     fout << collection.str();
//     fout.close();

//     return collection.str();
// }

// 带参数的构造函数

/**
graph TD
   A[构造IMOMD-RRT*] --> B{源点和目标点<br>是否相同?}
   B --> |是| C[报错并退出]
   B --> |否| D[初始化destinations_向量]
   D --> E[初始化tree_layers_向量]
   E --> F[初始化disjoint_set_parent_/children_]
   F --> G[初始化probability_matrix_/bearing_matrix]
   G --> H[初始化expandables_min_heuristic_matrix_ 距离矩阵]
   H --> I[初始化connection_node_matrix_ 连接节点矩阵]
   I --> J[初始化connection_nodes_set_ 连接节点集合]
   J --> K[设置source_tree_id_和target_tree_id_]
   K --> L[构建probability_matrix_概率矩阵]
   L --> M[构建eci_gen_solver_ RTSP求解器]
   M --> N[设置random_gen_随机数生成器]
   N --> O[记录start_time_ 开始时间]
   O --> P{是否记录数据}
   P --> |是| Q[打开command_csv_命令记录CSV文件]
   P --> |否| R[结束构造]

*/
ImomdRRT::ImomdRRT(size_t source, size_t target, std::vector<size_t> objectives,
                   const std::shared_ptr<std::vector<location_t>> map_ptr,
                   const std::shared_ptr<std::vector<std::unordered_map<size_t, double>>> connection_ptr,
                   const imomd_setting_t &setting)
    : setting_(setting), map_ptr_(map_ptr), connection_ptr_(connection_ptr),
      is_connected_graph_(false), is_distance_matrix_updated_(false),
      is_merge_done_(false), is_path_updated_(false), is_computation_finished_(false),
      cond_get_path_(PTHREAD_COND_INITIALIZER), lock_(PTHREAD_MUTEX_INITIALIZER)
{
    bipedlab::debugger::debugColorTextOutput("[IMOMD] Constructing IMOMD-RRT*", 10, BC);

    // 初始化 destinations_ : 目标列表
    destinations_ = {source};
    for (const size_t objective : objectives)
    {
        destinations_.push_back(objective);
    }
    destinations_.push_back(target);

    // 初始化  tree_layers_ : 树层向量
    int dest_count = destinations_.size();
    tree_layers_.resize(dest_count);
    // disjoint_set_parent_/children_ : 不相交集数据结构
    // disjoint_set_parent_ 表示不相交集中每个集合的父节点信息,反映了集合之间的关系,是不相交集算法的核心数据结构。
    // 每个元素表示一个集合的父节点是哪个集合
    disjoint_set_parent_.resize(dest_count);
    // 初始化 probability_matrix_ : 目标选择概率矩阵
    probability_matrix_.resize(dest_count, std::vector<double>(dest_count, 0.0));
    // 初始化 bearing_matrix : 目标方位矩阵
    std::vector<std::vector<std::pair<double, int>>> bearing_matrix(dest_count);
    std::vector<std::vector<double>> inversed_haversine_matrix(
        dest_count, std::vector<double>(dest_count, 0.0));
    std::vector<double> sum_haversine(dest_count, 0.0);

    // 初始化 expandables_min_heuristic_matrix_ : 可扩展节点最小启发矩阵
    expandables_min_heuristic_matrix_.resize(dest_count,
                                             std::vector<std::pair<size_t, double>>(dest_count, {-1, INFINITY}));
    // 初始化 distance_matrix_ : 距离矩阵
    distance_matrix_.resize(dest_count, std::vector<double>(dest_count, 0.0));
    // 初始化 connection_node_matrix_ : 连接节点矩阵
    connection_node_matrix_.resize(dest_count, std::vector<size_t>(dest_count, -1));

    for (int i = 0; i < dest_count; ++i)
    {
        // Initialize Trees rooted at each destinations
        // 获取目标root,这个root就是目标点的节点 id
        size_t root = destinations_[i];
        // tree_layers_ : 树层向量;使用root创建一棵新树,存储在tree_layers_向量的第i个位置。
        tree_layers_[i] = tree_t(i, root);
        // 将新树的根节点root加入可扩展节点列表,开始树的扩展
        updateExpandables(tree_layers_[i], root, true);

        /**
         *
         * 不相交集的使用逻辑流程图
         * IMOMD-RRT*算法中使用的不相交集数据结构与并查集非常相似。事实上,它可以视为一种增强的并查集。
         * 并查集是一种树型数据结构,用于处理一组不相交的集合的数据结构。它支持两种基本操作:
         * 1. Find:查找元素所属的集合。
         * 2. Union:合并两个集合。
         * IMOMD-RRT*算法中的不相交集数据结构也具有这两种基本操作,但还添加了额外的功能以支持算法需求:
         * 1. 追踪每个集合的子集合列表,用于合并集合时也同时合并子集合。
         * 2. 当集合合并时,不仅记录父集合,也记录集合之间的关系(子父关系),这提供了额外的结构信息。
         * 3. 可以使用集合之间的这些关系来判断所有集合是否最终归为同一集合,实现联通性检测。
         * 所以,IMOMD-RRT*算法的不相交集数据结构继承并扩展了并查集,提供了额外的功能以支持算法对集合关系的更丰富追踪和判断。这使其与传统的并查集有一定区别,但最基本的Find和Union操作与并查集相同。
         * 可以说,IMOMD-RRT*算法中的不相交集数据结构是一种带增强功能的并查集。所以,将其视为与并查集相似是正确的。
         graph TD
            A[初始化不相交集] --> B[开始多目标树扩展]
            B --> C{新节点x_new是否连接两树}
            C --> |否| B
            C --> |是| D[查找x_new在两树中的父节点]
            D --> E{两树的父节点是否相同}
            E --> |是| B
            E --> |否| F[查找两树在不相交集中的ID]
            F --> G{集合ID1 < 集合ID2}
            G --> |否| H[交换集合ID1和集合ID2]
            H --> I[将集合ID2设为集合ID1的子集合]
            I --> J[将集合ID2的所有子集合also设为集合ID1的子集合]
            J --> K[clear集合ID2的子集合列表]
            G --> I
            I --> L[检查所有树是否归为同一集合]
            L --> |否| B
            L --> |是| M[多目标树扩展完成]
            M --> N[结束]
        */
        // Initialize Disjoint-set Data Structure
        // 为每个目标初始化一个单元素不相交集。
        // disjoint_set_parent_[i]表示第i个集合的父集合,初始时每个集合的父集合就是它自己。
        disjoint_set_parent_[i] = i;
        // disjoint_set_children_[i]表示第i个集合的子集合列表,初始时每个集合没有子集合,所以列表为空
        disjoint_set_children_[i] = {};

        /**
         各个变量在算法流程中的使用逻辑：
        graph LR
            A[构造IMOMD-RRT*] --> B{初始化:<br>destinations_/tree_layers_<br>source_tree_id_/target_tree_id_<br>max_iter_/max_time_ }
            B --> C[初始化bearing_matrix:存储目标点对方位角]
            C --> D[初始化inversed_haversine_matrix:存储目标点对距离倒数]
            D --> E[计算sum_haversine:获得目标总距离倒数,用于概率归一化]
            E --> F[构建probability_matrix_:概率与距离倒数成正比,更近目标概率更大]
            F --> G[开始多目标树扩展]
            G --> H{新节点x_new是否连接两树}
            H --> |否| Q[随机选择扩展点:根据probability_matrix_选择目标]
            H --> |是| I[更新distance_matrix_/connection_node_matrix_:存储点对最短路径/最新连接节点]
            I --> J[连接两树:更新disjoint_set_:判断是否形成联通图]
            J --> K{形成联通图?<br>disjoint_set_ == single_set}
            K --> |否| Q
            K --> |是|L[10次求解RTSP,更新路径]
            Q --> R[迭代次数%100==0?<br>求解RTSP]
            R --> |是| L
            R --> |否| P{探索完成?<br>或超时/超过max_iter_}
            P --> |否| N[选择最近可扩展节点<br>x_new]
            P --> |是|O[结束]
            N --> O[连接x_new到代价最小父节点<br>更新expandables/回溯树<br>更新选择概率probability_matrix_:x_new靠近目标概率增大]
            O --> G
        */
        // Initialize Distance Matrix & Connection Nodes of destinations
        for (int j = i + 1; j < dest_count; ++j)
        {
            // Calculate Bearing and Haversine Distance to dertmine the
            // probability of selecting random points

            // bearing_matrix用于存储每个目标点对之间的方位角。用于计算目标选择概率矩阵probability_matrix_。
            // 算法会优先选择方位角较少的目标,以避免树重叠

            // 方位角表示目标点对之间的相对方向,它与heading角的区别在于:
            // 方位角:0°表示正北,顺时针增加,范围为0°至360°. bearing_matrix用于多目标采样
            // heading角:0°表示当前行进方向,顺时针增加,范围为0°至360°.heading角用于机器人自身的导航
            bearing_matrix[i].push_back(
                {computeBearing((*map_ptr)[destinations_[i]], (*map_ptr)[destinations_[j]]), j});

            bearing_matrix[j].push_back(
                {computeBearing((*map_ptr)[destinations_[j]], (*map_ptr)[destinations_[i]]), i});

            // inversed_haversine_matrix矩阵存储每个目标点对之间的哈佛派快距离的倒数。
            // Haversine距离越小,两个目标点越近,矩阵中的值越大
            // 矩阵同样用于构建probability_matrix_,
            // 概率与haversine距离的倒数成正比,所以更近的目标选择概率更大
            // connection_ptr矩阵存储节点之间的连接关系和代价,但它包含地图上所有节点的连接,而不是仅目标节点之间的连接
            // inversed_haversine_matrix仅包含目标节点之间的距离信息,
            // 它作为辅助矩阵用于构建probability_matrix_,而不是替代connection_ptrMatrix
            inversed_haversine_matrix[i][j] = 1.0 /
                                              computeHaversineDistance((*map_ptr)[root], (*map_ptr)[destinations_[j]]);
            inversed_haversine_matrix[j][i] = inversed_haversine_matrix[i][j];

            // sum_haversine存储每个目标的总哈佛派快距离倒数,用于后续归一化概率。
            // 将每个目标点对的距离倒数相加,可以获得每个目标的总距离倒数,
            // 然后将概率归一化为(目标点对距离倒数)/ (目标总距离倒数)
            sum_haversine[i] += inversed_haversine_matrix[i][j];
            sum_haversine[j] += inversed_haversine_matrix[j][i];

            // Initialize Distance Matrix
            // distance_matrix_存储每个目标点对之间的距离,初始化为INFINITY。
            // 之后,当两棵树通过新节点x_new相连时,会更新此矩阵中对应目标点对的值为树间最短路径长度。
            // 这个矩阵最终会提供算法中需要的目标点对最短路径信息
            distance_matrix_[i][j] = INFINITY;
            distance_matrix_[j][i] = INFINITY;

            // Initialize Connection Nodes of destinations
            // connection_node_matrix_存储每个目标点对之间的最优连接节点,初始化为-1。
            // 当两棵树通过新节点x_new相连,并找到更短路径时,会更新此矩阵中对应目标点对的值为最新连接的节点x_new。
            // 这个矩阵记录每个目标点对最新的联通节点
            connection_node_matrix_[i][j] = -1;
            connection_node_matrix_[j][i] = -1;

            // Initialize the set of connection nodes for two tree
            // Totally, (n Choose 2) number of set
            // connection_nodes_set_存储两棵树之间所有连接过的节点,用于树合并。
            // 此数据结构以目标点对唯一标识,用于存储对应点对的所有连接节点。
            // 循环中创建的数目为C(n,2),代表目标总数n可选的点对数目。
            connection_nodes_set_.emplace_back(new std::unordered_set<size_t>);
            /**
             * 此处代码等价于：
             * 所以后面在计算索引的时候需要注意！
             * connection_nodes_set_.resize(num_trees * (num_trees - 1) / 2);
                for(int i=0; i<num_trees; ++i) {
                    for(int j=i+1; j<num_trees; ++j) {
                        connection_nodes_set_[i*(i-1)/2 + j] = new unordered_set<int>();
                    }
                }
            */
        }
    }

    // Set source and target tree_id as first and last idx of tree_layers, respectively
    // 设置 源点和目标点的树id为树层的第一个和最后一个索引
    source_tree_id_ = 0;
    target_tree_id_ = dest_count - 1;

    // 以下代码是根据方位角构建目标选择概率矩阵
    // 1. 方位角:概率与方位角差值成正比。方位角差值越大,该方向上可选目标越少,概率越大
    // Construct Matrix of Probability of Sampling Destinations
    // Calculate Probability with Bearing
    // The probability of Selection become larger when ther's not much destinations
    // in that direction.
    for (int i = 0; i < dest_count && dest_count > 3; ++i)
    {
        // 将目标的方位角按照顺时针排序，也就是从小到大排序，因为方位角的0°表示正北,顺时针增加
        // bearing_matrix[i][0]表示目标i与方位角最大相邻目标之间的方位角
        // bearing_matrix[i][1]表示目标i与方位角次大相邻目标之间的方位角
        // 方位角差值越小,该方向上可选目标越多,概率应相应减小,这有助于避免树在此方向上重叠
        // Sort destinations counter-clockwisely
        std::sort(bearing_matrix[i].begin(), bearing_matrix[i].end());

        double sum_bearing = 0.0;
        // 计算当前源点i 与所有的别的目标方位角，并且得到sum_bearing
        for (int j = 0; j < dest_count - 1; ++j)
        {
            // 选择三个点,分别计算它们之间的夹角
            double curr_angle = bearing_matrix[i][j].first;
            double prev_angle = bearing_matrix[i][(j - 2 + dest_count) % (dest_count - 1)].first;
            double next_angle = bearing_matrix[i][(j + 1) % (dest_count - 1)].first;

            // 计算夹角差值,并将其限制在0~2π之间
            double angle_diff_prev = curr_angle - prev_angle;
            angle_diff_prev += angle_diff_prev < 0 ? 2 * M_PI : 0;
            // 计算夹角差值,并将其限制在0~2π之间
            double angle_diff_next = next_angle - curr_angle;
            angle_diff_next += angle_diff_next < 0 ? 2 * M_PI : 0;

            // Probability of selection is propotional to sum of the angles
            // between two destinations on each side
            // 选择概率与两个方向上的夹角之和成正比。因为这里暂时就设置为了方位角差值之和
            // 这里的i 因为 sort，所以导致和 二级索引和 计算方位角的索引不一致，所以需要使用second获取计算时的索引
            int idx = bearing_matrix[i][j].second;
            probability_matrix_[i][idx] = angle_diff_prev + angle_diff_next;
            sum_bearing += probability_matrix_[i][idx];
        }
        // 概率归一化，也就是将i=到 的方位角差值，都需要除以sum_bearing
        for (int j = 0; j < dest_count; ++j)
        {
            probability_matrix_[i][j] /= sum_bearing;
        }
    }

    // Calculate Probability with Haversine Distance
    // Probability of selection is inversely propotional to the haversine distace
    // 这里是 inversed_haversine，也就是说概率与haversine距离成反比
    // 所以距离越近，概率越大
    for (int i = 0; i < dest_count; ++i)
    {
        for (int j = 0; j < dest_count; ++j)
        {
            probability_matrix_[i][j] += inversed_haversine_matrix[i][j] / sum_haversine[i];
        }
    }
    /**
     随机选择扩展点时:
     1. 优先选择方位角差值较大的目标,这可以避免树在同一方向上重叠,产生更佳的采样图。
     2. 优先选择较近的目标,这可以缩短树与树之间形成连接所需的时间,提高探索效率

     Q&A：
     问题：优先选择方位角较小的目标,这可以避免树在同一方向上重叠,产生更佳的采样图。但是这样不会导致多个点都往一个点少的方向扩展吗
     答案：概率设置还考虑了距离因素。虽然方位角较小的目标概率更大,但距离更近的目标概率也更大。
     这使得每个树在扩展过程中可以达到多个目标,而不会完全朝向某一个方向。
     算法不会始终选择方位角最小的目标。它只是增大了该目标的概率,但其他目标仍有一定概率被选择。
     这引入了一定随机性,避免完全朝向某一方向扩展。算法还考虑了树与树之间的连接。
     一旦两棵树通过新节点x_new连接,它们就变得相互可达,概率设置会相应调整。这也会影响两棵树之后的扩展方向。
     算法中RTSP求解器的引入也为扩展方向提供了定向。经由RTSP求解,可以获得一条较短的采样路径,这会影响树的扩展顺序和方向。

    结论：
    1. 同时考虑距离和方位角因素的概率设置。
    2. 维持一定的随机性,而非完全依赖方位角最小的目标。
    3. 考虑树与树之间的连接及其对概率的影响。
    4. RTSP求解器的引入。
    */

    /**
     * 平衡probability_matrix_矩阵的值,避免由于构建矩阵的顺序导致某些目标概率过大的问题。
     * 通过加倍对角元素并再归一化,可以获得一个目标选择概率更为均衡的矩阵。
     *
     */
    // Balance two Probability of selecting destinations
    // Example:
    // d1(exploring tree) -> d2
    // d2(exploring tree) -> d1
    std::vector<double> sum_probability(dest_count, 0.0);
    for (int i = 0; i < dest_count; ++i)
    {
        for (int j = i + 1; j < dest_count; ++j)
        {
            /**
             * 概率 /= 总概率,实现归一化。
             * 分别累加目标i和目标j在probability_matrix_矩阵中对应行和列的所有概率值。
             * 最终获得每个目标的总概率,存储在sum_probability向量中。
             * 这些总概率在后续的归一化中使用,实现概率分布的重新规范化。
             */
            probability_matrix_[i][j] += probability_matrix_[j][i];
            probability_matrix_[j][i] = probability_matrix_[i][j];

            sum_probability[i] += probability_matrix_[i][j];
            sum_probability[j] += probability_matrix_[j][i];
        }

        for (int j = 0; j < dest_count; ++j)
        {
            probability_matrix_[i][j] /= sum_probability[i];
        }
    }

    // Construct RTSP solver
    // 初始化RTSP求解器
    eci_gen_solver_ = EciGenSolver(setting.rtsp_setting);
    shortest_path_ = {};
    shortest_path_cost_ = INFINITY;

    // Initialize Random genrator
    // 初始化随机生成器
    if (setting_.random_seed)
    {
        std::random_device rd;
        random_gen_ = std::mt19937{rd()};
    }
    else
    {
        random_gen_ = std::mt19937{0};
    }
    // 日志设置
    // Logging data
    start_time_ = bipedlab::timing::getCurrentTime();
    if (setting_.log_data)
    {
        try
        {
            std::string date = bipedlab::utils::getTimeNDate();
            std::string path = bipedlab::utils::getCurrentDirectory();

            command_csv_.open(std::string(path) + "/experiments/IMOMD_" +
                              date + "_command_history.csv");

            // Header
            command_csv_ << "CPU_time"
                         << "path_cost"
                         << "tree_size" << endrow;
        }
        catch (const std::exception &ex)
        {
            std::cout << "Exception was thrown: " << ex.what() << std::endl;
        }
    }
}

/**
 graph TD
    A[构造IMOMD-RRT*对象] --> B[设置is_done标志为false]
    B --> C{迭代次数达到上限或<br>运行时间达到上限?}
    C -->|是| S[结束搜索]
    C -->|否| D[扩展树层]
    D --> E{迭代次数%100==0?}
    E -->|否| I[迭代次数+1]
    E -->|是| F[求解RTSP]
    F --> G[检查每棵树是否全部探索完成]
    G --> H{所有树探索完成?}
    H -->|否| I
    H -->|是| L[结束树的扩展]
    I --> J{迭代次数达到上限或<br>运行时间达到上限?}
    J -->|否| D
    J -->|是| O
    L --> O[10次求解RTSP]
    O --> P[设置结束标志]
    P --> Q[结束线程]
    S --> Q
Q：什么情况树会扩展完？
A：
    1. 该树的可扩展节点集合(expandables)为空，即树中所有节点都被探索了
    2. probability_matrix 矩阵中，该树与所有其他树对应的选择概率均被设置为0.表示这个树无需与其他树进行连接。
    3. probability_matrix会在以下情况被设置为0，a:两个树成功连接，他们的最短距离已经确定，不需要进行进一步探索
    4. 所有的可扩展节点的启发距离都大于两个树的当前最短距离，即两个树之间无法再找到更短的路径。

*/
void *ImomdRRT::findShortestPath(void *arg)
{
    ImomdRRT *this_thread = static_cast<ImomdRRT *>(arg);

    bool is_done_iter = false;
    bool is_done_time = false;
    bool is_done_expansion = false;
    while (!is_done_iter && !is_done_time && !is_done_expansion)
    {
        // 扩展树层
        this_thread->expandTreeLayers_();

        // 每100次迭代，求解一次TSP
        // Solve RTSP every 100 iteration of tree expansion
        if (this_thread->iteration_count_ % 100 == 0)
        {
            this_thread->solveRTSP_();

            // 如果每个树都探索完毕，那么就设置整个搜索完毕
            is_done_expansion = true;
            for (const tree_t &tree : this_thread->tree_layers_)
            {
                if (!tree.is_done)
                {
                    is_done_expansion = false;
                }
            }
        }

        bipedlab::debugger::debugColorOutput("[IMOMD] count : ",
                                             this_thread->iteration_count_, 3);

        if (this_thread->iteration_count_++ > this_thread->setting_.max_iter)
        {
            is_done_iter = true;
        }
        else if (bipedlab::timing::spendElapsedTime(this_thread->start_time_) >
                 this_thread->setting_.max_time)
        {
            is_done_time = true;
        }
    }

    if (is_done_iter)
    {
        bipedlab::debugger::debugTitleTextOutput("[IMOMD]", "Reached Max Iteration", 10, BM);
    }
    else if (is_done_time)
    {
        bipedlab::debugger::debugTitleTextOutput("[IMOMD]", "Reached Max Time", 10, BM);
    }

    if (is_done_expansion)
    {
        bipedlab::debugger::debugTitleTextOutput("[IMOMD]", "Tree Exploration Done", 10, BM);
    }

    // 求解 10次TSP
    int count = 0;
    while (count++ < 10)
    {
        this_thread->solveRTSP_();
    }
    // 结束pathfind

    pthread_mutex_lock(&this_thread->lock_);
    this_thread->is_computation_finished_ = true;
    pthread_mutex_unlock(&this_thread->lock_);
    pthread_cond_signal(&this_thread->cond_get_path_);

    pthread_exit(NULL);
}

void *ImomdRRT::printPath(void *arg)
{
    ImomdRRT *this_thread = static_cast<ImomdRRT *>(arg);

    int count = 0;
    while (!this_thread->is_computation_finished_)
    {
        pthread_mutex_lock(&this_thread->lock_);

        // Wait for path update or termination of getShortestPath
        while (!this_thread->is_path_updated_ && !this_thread->is_computation_finished_)
        {
            pthread_cond_wait(&this_thread->cond_get_path_, &this_thread->lock_);
        }
        // this_thread->map_ptr_
        // Export the shortest path
        bipedlab::debugger::debugTitleTextOutput("[main]",
                                                 "print path: " + std::to_string(count++), 10, BG);
        for (auto node : this_thread->shortest_path_)
        {

            std::cout << node << "|" << (*(this_thread->map_ptr_))[node].id << " -> ";
        }
        std::cout << "#" << std::endl;

        std::cout << std::endl;

        // 输出GeoJSON Feature
        std::cout << "{\"type\": \"Feature\", "
                     "\"geometry\": {";

        // 输出原有的LineString
        std::cout << "\"type\": \"LineString\", "
                  << "\"coordinates\": [";

        // 遍历shortest_path
        for (size_t i = 0; i < this_thread->shortest_path_.size(); ++i)
        {

            // 根据id在raw_map中查找点的经纬度坐标
            size_t node_id = this_thread->shortest_path_[i];
            location_t node = (*(this_thread->map_ptr_))[node_id];

            // 输出经纬度坐标
            std::cout << "[" << std::to_string(node.longitude) << ", "
                      << std::to_string(node.latitude) << "]";

            // 添加逗号分隔符
            if (i != this_thread->shortest_path_.size() - 1)
            {
                std::cout << ", ";
            }
        }

        std::cout << "]}, "
                     "\"properties\":"
                     "{\"id\": 0,"
                     "\"stroke\": \"#da3445\","
                     "\"stroke-width\": 3,"
                     "\"stroke-opacity\": 1}"
                  << "}" << std::endl;

        this_thread->is_path_updated_ = false;
        pthread_mutex_unlock(&this_thread->lock_);
    }

    pthread_exit(NULL);
}

std::shared_ptr<std::vector<std::vector<double>>> ImomdRRT::getDistanceMatrix()
{
    return std::make_shared<std::vector<std::vector<double>>>(distance_matrix_);
};

/**
 * 就是针对每一个树进行扩展
 */
void ImomdRRT::expandTreeLayers_()
{
    for (auto &tree : tree_layers_)
    {
        expandTree_(tree);
    }
}

/**
 * 扩展树的流程
 * 算法流程:
 * 1. 如果树为空或探索完成,则返回
 * 2. 选择随机点
 * 3. 选择最近的可扩展节点x_new
 * 4. 将新节点x_new连接到代价最小的父节点
 * 5. 更新可扩展节点
 * 6. 回溯树并尝试找到更短路径
 * 7. 更新选择概率
 * 8. 更新树之间的连接
```mermaid
graph TD
   A[树为空或探索完成] -->|是| END
   A -->|否| B[选择随机点]
   B --> C{选择最近的可扩展节点<br>x_new}
   C -->|x_new| D[将新节点x_new连接到<br>代价最小的父节点]
   D --> E[更新可扩展节点]
   E --> F[回溯树并尝试找到更短路径]
   F --> G[更新选择概率]
   G --> H[更新树之间的连接]
   H --> A
```
*/
void ImomdRRT::expandTree_(tree_t &tree)
{
    if (tree.expandables.empty() || tree.is_done)
    {
        return;
    }

    bipedlab::debugger::debugColorOutput("[IMOMD] Expand Tree : ", tree.root, 3, BG);
    bipedlab::debugger::debugColorOutput("[IMOMD] Total Nodes : ", tree.parent.size(), 1, BK);
    bipedlab::debugger::debugColorOutput("[IMOMD] Expandable Nodes : ", tree.expandables.size(), 1, BK);

    // 选择一个随机点
    location_t random_point = selectRandomVertex_(tree);
    // 选择距离随机点最近的可扩展节点x_new，这里面还会看能不能JPS优化
    // 然后如果经过了JPS优化，那么我们还需要更新树地结构以及树之间地连接关系
    size_t x_new = steerNewNode_(tree, random_point);
    // 为新节点x_new寻找并连接到当前树中代价最小的父节点上
    // 第一是从当前x_new的相邻节点中找到一个使得cost最小的父节点
    // 然后是更新树的结构，设置父子关系
    connectNewNode_(tree, x_new);
    // 更新可扩展节点列表
    // 从可扩展节点列表中删除x_new，然后更新树之间的最小启发距离矩阵
    // 并且还会将x_new的邻居添加到可扩展列表
    updateExpandables(tree, x_new, true);
    // 回溯树，并尝试为节点找到更短路径
    // 如果树的结构发生变化，我们也需要修改设置树之间的连接关系
    rewireTree_(tree, x_new);
    // 更新选择概率
    // 主要功能是当随机点是目标点时，更新概率分布函数
    // 主要是提高别的目标的探索概率
    // 还会检查树是不是探索结束
    // **目标探索概率矩阵地作用！**
    // 探索性 - 保持一定概率探索未连接的树,争取找到关键连接
    // 目标性 - 连接后提高关键树之间的概率,快速形成联通图
    // 平衡 - 归一化使各树有合理概率,避免个别树概率为零无法探索
    // 优化 - 探索完成的树降低概率,不再浪费资源
    updateSelectionProbability(tree, random_point);
    // 更新树之间的连接
    // 当前树的结构发生变化时，我们需要更新当前树到其他树的连接关系
    // x_new 就是导致结构变化的节点
    // 如果x_new首次让两个树连接，那么就会设置两个树的连接关系
    updateConnectionTree_(tree, x_new);
}

/**
 * 功能就是根据地图生成一个随机的节点/也可能是直接使用地图中的随机点（根据goal_bias决定）
graph TD
    A[生成0-1之间随机数rand] --> B{rand < goal_bias?\n基于goal_bias实现了探索性和目标性的控制}
    B -- 是 --> C[根据probability_matrix_选择目标点\n这里是使用轮盘赌采样\n使得采样概率和目标选择概率矩阵中的值成正比]
    C --> D{目标点已被该树探索?}
    D -- 否 --> E[随机点为目标点]
    D -- 是 --> F[计算插值比例ratio]
    F --> G[计算插值坐标\n这里是使用的平滑加权线性插值\n可以使用向量计算的方式优化代码使逻辑更加清晰]
    B -- 否 --> H[从地图中随机选择点]
    E --> I[基于均匀分布返回随机点]
    G --> I
    H --> I
*/
location_t ImomdRRT::selectRandomVertex_(tree_t &tree)
{
    static std::uniform_real_distribution<> dist(0, 1);
    static std::uniform_real_distribution<> ratio_dist(0.0, 0.5);
    // random_gen_ = std::mt19937{0};
    // 以goal_bias的概率控制随机点选择是朝向目标点还是完全随机
    // 如果goal_bias设置为0.05:
    // 有5%的概率会选择目标点作为随机点(exploitation)
    // 有95%的概率会选择地图中的随机点作为随机点(exploration)
    // goal_bias越大，选择目标的概率越大，算法具有目标性
    // 越小时，随机的概率越大，越具有探索性
    location_t random_point;
    if (dist(random_gen_) < setting_.goal_bias)
    {
        // 当小于时，会执行基于probability_matrix_ 的目标点选择。
        // 类似于轮盘赌采样，每个目标点的概率为probability_matrix_中的值，也就是采样概率和probability_matrix_[tree.id][i++]的大小成正比

        // 使用轮盘赌采样的方式，在目标点之间随机选择一个点
        // 每个目标点的选择概率由probability_matrix_中的值决定
        // probability_matrix_[tree.id][i++] 表示第tree.id个目标点的第i个概率
        // 通过不断减去概率值，直到随机数小于零时停止循环，确定最终选择的目标点

        // Select Random Points between destinations based on probability
        double rand = dist(random_gen_);
        int i = 0;
        while (rand >= 0)
        {
            rand -= probability_matrix_[tree.id][i++];
        }
        size_t random_dest = tree_layers_[i - 1].root;

        // 如果当前选择的目标点没有被探索，那么直接将其作为随机点 random_point
        random_point = (*map_ptr_)[random_dest];

        // 如果这个当前这个目标点被探索过了，那么需要通过当前目标点和树根节点之间的距离来计算插值比例ratio
        // 然后根据插值比例ratio来计算插值坐标
        // Select random location between root of tree and random_point
        if (tree.checkVisitedNode(random_dest))
        {
            // 线性平滑加权
            // 我重新写了一个向量的版本
            /**
                 // 计算树根节点和目标点之间的向量
                location_t root_to_dest_vec;
                root_to_dest_vec.latitude = (*map_ptr_)[random_dest].latitude - (*map_ptr_)[tree.root].latitude;
                root_to_dest_vec.longitude = (*map_ptr_)[random_dest].longitude - (*map_ptr_)[tree.root].longitude;

                // 按比例缩放向量实现平滑插值
                root_to_dest_vec.latitude *= ratio;
                root_to_dest_vec.longitude *= ratio;

                // 获得平滑插值后的随机点坐标
                random_point.latitude = (*map_ptr_)[tree.root].latitude + root_to_dest_vec.latitude;
                random_point.longitude = (*map_ptr_)[tree.root].longitude + root_to_dest_vec.longitude;
            */
            double ratio = ratio_dist(random_gen_);

            random_point.id = random_dest;
            random_point.latitude *= ratio;
            random_point.longitude *= ratio;

            // Ratio Randomly selected between 0.0 ~ 0.5
            // Try to avoid tree overlaped, which rarely provide useful connection nodes
            random_point.latitude += (1.0 - ratio) * (*map_ptr_)[tree.root].latitude;
            random_point.longitude += (1.0 - ratio) * (*map_ptr_)[tree.root].longitude;
        }
    }
    else
    {
        // 基于均匀分布的随机生成器进行随机采样
        static std::uniform_int_distribution<size_t> uni_map(0, (*map_ptr_).size() - 1);
        // 当 rand>goal_bias时，会直接选择随机点
        random_point = (*map_ptr_)[uni_map(random_gen_)];
    }

    bipedlab::debugger::debugColorOutput("[IMOMD] Random Point(Graph) : ", random_point.id, 1, BW);
    return random_point;
}

/**
 * 根据随机点获取x_new
 * 从当前树的可扩展列表中选择距离随机点最近的可扩展节点,设置为x_new；然后看能否通过JPS，优化搜索扩展节点
 * 主要是第一个在进行JPS之前需要先更新可扩展列表
 * JPS地过程时，还需要更新树地结构，并且更新树地联通关系
```mermaid
graph TD
A[输入:tree,random_point] --> B[找到距离random_point最近的可扩展节点,先初始化min_dist为无穷大]
B --> C[遍历tree的可扩展节点]
C --> D[计算节点dist与random_point距离]
D --> E{dist < min_dist?}
E -- 是 --> F[更新min_dist]
F --> G[更新最近节点x_new] --> C
G --> G1[结束对可扩展节点的遍历,找到了最近的可扩展结点]
G1 --> I{判断x_new连接数==2?}
I -- 是 --> J1[扩展节点]
J1 --> L
I -- 否 --> L[开始进行JPS]
L-->L1[设置jps_done为false]
L1 -->  K{x_new的连接数 == 2 并且jps搜索未结束?}

K -- 否 --> End[结束LPS] --> END
K -- 是 -->  K1[获取当前节点的两个连接]

K1 -->  P{判断两个连接节点已在tree中?}
P -- 两个都被访问过 --> Q[JPS被设置为结束]  --> End
P -- 只有一个被访问过 --> P1[根据访问顺序确定父子关系]
P1 --> R[设置x_new被访问]
R --> R1[更新tree结构,包括parent和children]
R1 --> R2[更新x_new 的cost]
R2 --> R3[通过x_new更新两个树的连接关系]
R3 --> R4[根据搜索方向更新x_new,继续下一轮JPS] --> K
```
*/
size_t ImomdRRT::steerNewNode_(tree_t &tree, location_t &random_point)
{
    bipedlab::debugger::debugColorTextOutput("[IMOMD] Steer New Node", 2, BC);

    size_t x_new;
    // 初始化min_dist为无穷大
    double min_distance = INFINITY;
    // 遍历 tree的可扩展节点，最后找到一个距离random_point最近的可扩展节点，将x_new设置为该节点
    // Find x_new that minimizes the Dist(x, x_rand)
    for (const auto &expandable : tree.expandables)
    {
        // 对于每一个扩展结点，计算其与随机点的距离
        double distance = computeHaversineDistance((*map_ptr_)[expandable], random_point);

        if (distance < min_distance)
        {
            min_distance = distance;
            x_new = expandable;
        }
    }

    // Jumping Point Search
    // Jump until it meet intersection or already visited nodes
    if ((*connection_ptr_)[x_new].size() == 2)
    {
        // **更新树的可扩展列表**
        // 这里扮演了原始JPS，就是再跳点之前，会将当前节点添加到已访问列表中
        updateExpandables(tree, x_new, false);
    }

    bool is_jps_done = false;
    while ((*connection_ptr_)[x_new].size() == 2 && !is_jps_done)
    {
        // 获取一个迭代器，是x_new的所有出度
        auto it = (*connection_ptr_)[x_new].begin();
        // 获取当前连接关系和第二个连接关系，这里是利用了(*connection_ptr_)[x_new].size() == 2这个判断
        size_t x_1 = (*it).first;
        size_t x_2 = (*(++it)).first;

        // 判断x_1是否被访问过，如果被访问过，在内部继续判断x_2是否被访问过
        if (tree.parent.find(x_1) != tree.parent.end())
        {
            // 这里就是判断 x_2 是否被访问过
            if (tree.parent.find(x_2) != tree.parent.end())
            {
                // 如果x_1和x_2都被访问过，那么就结束JPS
                // x_1 (visited) --> x_new --> x_2 (visited)
                // Q:为什么我们直接退出了JPS，为啥不设置x_new被访问过
                // A:x_1和x_2都已访问,说明x_new所在区域已被当前树完全覆盖
                // 此时继续跳点没有意义,不会联通新的区域
                // 将x_new标记为已访问不会对图的联通性产生实质影响
                // 但是标记为已访问会增加树的规模和数据更新负担
                is_jps_done = true;
            }
            else
            {
                // 这里是更新树的结构
                // x_1 (visited) --> x_new --> x_2 (not visited)
                // 因为x1先被访问，所以x_new的父节点是x_1
                // 这里我们通过设置parent和children来更新树的结构关系
                tree.parent[x_new] = x_1;
                tree.children[x_1] = {x_new};
                // 这里是更新了x_cost的值
                tree.cost[x_new] = tree.cost[x_1] + (*connection_ptr_)[x_1][x_new];

                // **这里是这样的x_new是当前树中和随机点最近的可扩展节点，然后经过了JPS，更新了当前树里面的结构关系**
                // **所以我们需要更新树和树之间的连接关系**
                // 这里是当有新节点x_new 加入树时，需要更新树和树的连接关系
                // 1. 如果x_new连接了两个树，需要更新距离矩阵以及最近连接节点等信息
                // 2. 如果两个树第一次连接，那么需要更新不相交集，进一步处理联通性
                updateConnectionTree_(tree, x_new);

                // 最后是更新x_new，向搜索方向扩展，进入下一轮JPS
                x_new = x_2;
            }
        }
        // x_2 (visited) --> x_new --> x_1 (not visited)
        else
        {
            // 如果x_1没有被访问过，x_2被访问过，那么x_new的父节点是x_2
            // 同理设置x_2
            tree.parent[x_new] = x_2;
            tree.children[x_2] = {x_new};
            tree.cost[x_new] = tree.cost[x_2] + (*connection_ptr_)[x_2][x_new];
            updateConnectionTree_(tree, x_new);
            x_new = x_1;
        }
    }

    bipedlab::debugger::debugColorOutput("[IMOMD] x_new : ", x_new, 1, BW);
    return x_new;
}

/**
 * 为新节点x_new寻找并连接到当前树中代价最小的父节点上
 * 第一是从当前x_new的相邻节点中找到一个使得cost最小的父节点
 * 然后是更新树的结构，设置父子关系
graph TD
A[开始] --> B[初始化最小代价为无穷大]
B --> C[遍历x_new的相邻节点]
C --> D{相邻节点在树中?}
D -- 是 --> E[计算连接代价]
E -- 最小 --> F[更新最小代价]
F --> G[记录相邻节点为父节点]
G --> C
D -- 否 --> C
C -- 遍历结束 --> H{是否找到可连接父节点?}
H -- 是 --> I[设置父子关系]
subgraph 更新树的结构
I --> J[设置x_new代价]
end
H -- 否 --> K[结束]
J --> K

*/
void ImomdRRT::connectNewNode_(tree_t &tree, size_t x_new)
{
    bipedlab::debugger::debugColorTextOutput("[IMOMD] Connect New Node", 2, BC);

    // Select x_parent of x_new that make x_new have the lowest cost-to-come
    // 保存找到的父节点
    size_t x_parent;
    // 初始化x_new的代价为无穷大
    double min_cost_x_new = INFINITY;
    bool is_connectable = false;
    // 循环节点地所有相邻节点x_near
    for (const auto &x_near : (*connection_ptr_)[x_new])
    {
        // 如果这个相邻节点在当前树中
        if (tree.checkVisitedNode(x_near.first))
        {
            // 计算cost x_new  = x_near的代价 + x_near到x_new的代价
            double cost_x_new = tree.cost[x_near.first] + x_near.second;
            // 如果cost x_new 小于 min_cost_x_new
            if (cost_x_new < min_cost_x_new)
            {
                // 更新最小的cost x_new和x_parent
                // 并且设置为已经连接
                x_parent = x_near.first;
                min_cost_x_new = cost_x_new;
                is_connectable = true;
            }
        }
    }

    // Update Tree structure by adding node and edge
    // 这里是更新树的结构
    if (is_connectable)
    {
        tree.parent[x_new] = x_parent;
        tree.children[x_parent].insert(x_new);
        tree.cost[x_new] = min_cost_x_new;
    }

    bipedlab::debugger::debugColorOutput("[IMOMD] x_parent : ", x_parent, 1, BW);
}

/**
 * 从可扩展节点列表中删除x_new，然后更新树之间的最小启发距离矩阵
 * 并且还会将x_new的邻居添加到可扩展列表中
```mermaid
graph TD
BEGIN-->Del[删除x_new<br/>从expandables] -->A
subgraph 更新最小启发距离
A[开始循环遍历所有别的tree,准备更新所有的最小启发距离]-->B{tree是否<br/>为当前tree?}
B -- 否 --> C{判断当前x_new就是和别的树的最小启发距离的节点?}
C -- 是 --> D[准备循环找到并更新最小启发距离]--> E[最大化设置当前树和别的树的最小启发距离]
E --> E1[内层循环<br/>遍历当前树的expandables]
E1 --> F[计算启发距离:<br>cost_expandable_2_tree_root+cost_expandable_2_other_tree_root]
F--优于最小值-->G[更新最小启发距离,以及expandable]
G-->E1
E1-->A
end
A--启发距离循环更新结束-->SN{search_neighbor<br/>为真?}
SN -- 否 --> END
SN -- 是 --> I
subgraph 搜索邻居节点
I[遍历搜索x_new的邻居x_near]-->N[未访问邻居节点x_near]
N-->O[将邻居节点<br/>加入expandables]
O-->P[内层循环<br/>遍历所有其他tree]
P-->Q{tree是否<br/>为当前tree?}
Q--否-->R[计算启发距离]
R--优于最小值-->S[更新最小启发距离]
S-->P
R-->T{内层循环结束}
end

```
*/
void ImomdRRT::updateExpandables(tree_t &tree, size_t x_new, bool search_neighbor)
{
    // 从可扩展集合中删除x_new
    tree.expandables.erase(x_new);

    // 如果x_new是最小启发距离的可扩展节点，需要更新矩阵
    // expandables_min_heuristic_matrix_ 这个变量是在imomdrrt的构造函数中初始化的，每个元素都被初始化为{-1, INFINITY}
    // Update expandables_min_heuristic_matrix_ if x_new is the min expandable

    // 这里是外层循环，遍历所有的别的树
    for (tree_t &other_tree : tree_layers_)
    {
        if (other_tree.id == tree.id)
        {
            continue;
        }
        // 所以这里不会出现key error
        // 但是当我们将x_new从可扩展集合中删除后，如果x_new是最小启发距离的可扩展节点，那么需要更新矩阵
        if (x_new == expandables_min_heuristic_matrix_[tree.id][other_tree.id].first)
        {
            // 更新启发距离矩阵，设置一个最大值
            expandables_min_heuristic_matrix_[tree.id][other_tree.id] = {-1, INFINITY};
            // 这里是遍历当前树的所有可扩展节点，计算他们到别的树的启发距离，最后更新最小值上去
            for (const auto &expandable : tree.expandables)
            {
                // 计算启发距离,因为是启发距离，所以是直接计算的当前扩展点到树根的cost
                double heuristic_distance =
                    computeHaversineDistance((*map_ptr_)[expandable], (*map_ptr_)[tree.root]) +
                    computeHaversineDistance((*map_ptr_)[expandable], (*map_ptr_)[other_tree.root]);

                if (heuristic_distance < expandables_min_heuristic_matrix_[tree.id][other_tree.id].second)
                {
                    expandables_min_heuristic_matrix_[tree.id][other_tree.id] = {expandable, heuristic_distance};
                }
            }
        }
    }

    if (!search_neighbor)
    {
        return;
    }

    // 搜索邻居节点，这里是遍历当前节点的所有出度
    // Search neighborhood of x_new and update expandables and selection probability
    for (const auto &x_near : (*connection_ptr_)[x_new])
    {
        // 如果x_near是未访问的节点，那么将其加入到可扩展节点列表中
        // unexplored neighbor nodes of x_new are added to expandable nodes
        if (!tree.checkVisitedNode(x_near.first))
        {
            // 将未访问的邻居节点加入到可扩展节点列表
            tree.expandables.insert(x_near.first);

            // 更新启发距离矩阵
            // Update expandables_min_heuristic_matrix_
            for (tree_t &other_tree : tree_layers_)
            {
                if (other_tree == tree)
                {
                    continue;
                }

                // 这里是通过更新邻居节点更新当前树到别的树的启发距离
                double heuristic_distace =
                    computeHaversineDistance((*map_ptr_)[x_near.first], (*map_ptr_)[tree.root]) +
                    computeHaversineDistance((*map_ptr_)[x_near.first], (*map_ptr_)[other_tree.root]);

                if (heuristic_distace < expandables_min_heuristic_matrix_[tree.id][other_tree.id].second)
                {
                    expandables_min_heuristic_matrix_[tree.id][other_tree.id] = {x_near.first, heuristic_distace};
                }
            }
        }
    }
}

/**
 * 回溯树，找到最优路径
 * 在 connectNewNode_ 中,我们是基于最小cost连接x_new到其父节点上。
 * 但是此时x_new的引入也会影响到树中其他节点的最优连接方式。
 * rewireTree_ 的目的是考察x_new的引入是否可以优化其他节点的连接,主要有两个作用:
 * x_new可能成为某些节点的更佳父节点,使它们以更少的代价联通到树上。
 * 之前连接的节点由于x_new的引入,也可能存在更优父节点,需要调整。
 * 所以 rewireTree_ 是为了进一步优化树的拓扑结构,在贪心地连接x_new之后,再递归考察x_new对其他节点的影响。
 *
graph TD
A[开始] --> B[遍历x_new的相邻节点]
B -- 相邻节点 --> C{相邻节点在树中?}
C -- 否 --> B
C -- 是 --> D{相邻节点是x_new的父节点?}
D -- 是 --> B
D -- 否 --> E[计算新的连接代价]
E -- 更低代价 --> F[更新父子关系]
subgraph 更新树结构
F --> G[更新关联节点代价]
end
G --> H[更新树之间的连接]
H -- 对于所有cost改变的关联集结点 --> I[递归rewire关联节点]
I -- 递归 --> J[结束]
B -- 遍历结束 --> J
*/
void ImomdRRT::rewireTree_(tree_t &tree, size_t x_new)
{
    bipedlab::debugger::debugColorTextOutput("[IMOMD] Rewiring", 2, BC);

    // 遍历x_new的所有邻居节点x_near
    for (const auto &x_near : (*connection_ptr_)[x_new])
    {
        // Search visited neighbors except the parent of x_new
        // Check whether x_new could create shorter path for neighbors
        // 评估所有在当前树中，并且不是x_new的父节点的节点
        if (tree.checkVisitedNode(x_near.first) && x_near.first != tree.parent[x_new])
        {
            // 在connectNewNode_中有一个类似的代码段，但是是站在x_new的角度，计算以x_near为父节点时x_new的连接代价
            // 从x_near的角度,计算以x_new为父节点时x_near的新的连接代价
            // 这里是判断x_new能否成为x_near的父节点，如果能成为父节点，那么就更新x_near的父节点
            double new_cost_x_near = tree.cost[x_new] + x_near.second;
            // 判断新的cost更低，那么就需要更新树的结构
            if (tree.cost[x_near.first] > new_cost_x_near)
            {
                bipedlab::debugger::debugColorOutput("[IMOMD] x_near(rewired) : ",
                                                     x_near.first, 3, BW);

                // Disconnect with x_near and its parent
                // Connect with x_near and x_new
                tree.children[tree.parent[x_near.first]].erase(x_near.first);
                tree.parent[x_near.first] = x_new;
                tree.children[x_new].insert(x_near.first);

                // Update cost of x_near and its nested children
                // 直接修改cost无法记录哪些节点的cost被修改过
                // 所以这里需要记录哪些节点的cost的被修改过，用于后续处理
                std::vector<size_t> updated_nodes = tree.updateCost(x_near.first, new_cost_x_near);

                // 一般是当前树的结构发生变化时，我们需要更新当前树到其他树的连接关系
                // 导致结构变化的节点是x_near.first
                // 如果x_near.first首次让两个树连接，那么就会设置两个树的连接关系
                // 还会更新树之间的连接矩阵
                updateConnectionTree_(tree, x_near.first);

                // Rewiring recursively
                for (auto rit = updated_nodes.rbegin(); rit != updated_nodes.rend(); ++rit)
                {
                    if (x_near.first == *rit)
                    {
                        continue;
                    }
                    rewireTree_(tree, *rit);
                }
            }
        }
    }
}

/**
 * 主要功能是当随机点是目标点时，更新概率分布函数
 * 主要是提高别的目标的探索概率
 * 还会检查是不是探索结束
 *
 *
graph TD
A[开始] --> B{random_point是目标点?}
B -- 是 --> C[获取other_tree_id]
C --> D{启发距离 ><br/>实际最短距离?}
D -- 是 --> E[两树间探索概率设为0]
E --> F[分别根据目标选择概率矩阵判断设置两树is_done为true]
F --> G[累加计算两树所有概率]
G --> H[分别归一化两树所有概率,这样提高别的目标的概率变大]
H --> I[结束]

B -- 否 --> I
D -- 否 --> I
*/
void ImomdRRT::updateSelectionProbability(tree_t &tree, location_t &random_point)
{
    // Update probabilty of selecting destinations
    // check whether random_point is generated from destinations
    // 判断随机点是否是目标点
    auto it = std::find(destinations_.begin(), destinations_.end(), random_point.id);
    bool is_destination = it != destinations_.end();
    // 获取目标点所在的树的id
    int other_tree_id = std::distance(destinations_.begin(), it);

    // Update Explore Flag Matrix
    // If all expandable nodes have larger heuristic distance than current distance,
    // It stop explore between two destinations.
    // 如果是目标点，获取目标点所在的树
    // 然后判断当前树到目标点所在树的最小启发式距离是否大于当前两个树的实际距离
    if (is_destination &&
        expandables_min_heuristic_matrix_[tree.id][other_tree_id].second >
            distance_matrix_[tree.id][other_tree_id])
    {
        // 如果实际距离小于了启发距离，那么就不要再进行探索了
        // 这里先通过id获取树
        tree_t &other_tree = tree_layers_[other_tree_id];
        // 设置两数的探索概率矩阵，相关值为0
        //  这样就不会进行探索了
        probability_matrix_[tree.id][other_tree.id] = 0.0;
        probability_matrix_[other_tree.id][tree.id] = 0.0;

        // 判断树是不是全部探索完成，也就是概率值都否都是0，都是0的情况下
        // 关闭探索标志
        // 取出某个树的概率分布，判断是不是每个元素都是0
        // std::all_of 所有的lambda函数都返回true，那么返回true
        tree.is_done = std::all_of(probability_matrix_[tree.id].begin(),
                                   probability_matrix_[tree.id].end(), [](double d)
                                   { return d == 0; });

        other_tree.is_done = std::all_of(probability_matrix_[other_tree.id].begin(),
                                         probability_matrix_[other_tree.id].end(), [](double d)
                                         { return d == 0; });
        // 分别计算两个树的选择概率矩阵中所有概率的累加值
        double sum_probability_tree =
            std::accumulate(probability_matrix_[tree.id].begin(),
                            probability_matrix_[tree.id].end(), 0.0);

        double sum_probability_other_tree =
            std::accumulate(probability_matrix_[other_tree.id].begin(),
                            probability_matrix_[other_tree.id].end(), 0.0);
        // 分别归一化两个树的选择概率矩阵
        // 因为之前设置了两树间的概率为0，导致总的探索概率降低了
        // 这里通过归一化，重新使所有概率的和为1
        // 这样可以提高别的目标的选择概率，继续探索
        // Update Probability of selecting destinations as random point
        for (int i = 0; i < (int)destinations_.size(); ++i)
        {
            probability_matrix_[tree.id][i] /= sum_probability_tree;
            probability_matrix_[other_tree.id][i] /= sum_probability_other_tree;
        }
    }
}

/**
 * 一般是当前树的结构发生变化时，我们需要更新当前树到其他树的连接关系
 * x_new 就是导致结构变化的节点
 * 如果x_new首次让两个树连接，那么就会设置两个树的连接关系
 * 还会更新树之间的连接矩阵数据
 *
```
graph TD
A[外层循环遍历所有tree] --> B{tree是否为当前tree?}
B -- 否 --> A
B -- 否 --> C{判断x_new是否在别的tree?}
C -- 是 --> D[计算连接节点集合connection_set_idx的索引]
D --> E[通过连接节点集合计算两树是否是第一次连接?]
E -- 是 --> I[添加x_new到连接节点集合中<br>并且连接两树]
I --> F
E -- 否--> F[计算新路径长度<br>即求和x_new在当前树和其他树的cost]
F --优于当前distance_matrix中的最短距离--> G[更新距离矩阵]
G --> H[更新最近连接节点矩阵的值]
H --> O[设置更新标志位]
O --> P{pseudo_mode是否开启?}
F -- 不优于 --> P
P -- 否-->END
P -- 是--> subgraph_pseudo_mode
```
*/
void ImomdRRT::updateConnectionTree_(tree_t &tree, size_t x_new)
{
    bipedlab::debugger::debugColorTextOutput("[IMOMD] Update Connection of tree", 2, BC);

    // 检查x_new 能否连接另一个树
    // Check whether x_new could connect different two trees
    for (auto &other_tree : tree_layers_)
    {
        // 跳过当前树
        if (other_tree.id == tree.id)
        {
            continue;
        }

        // 检查当前x_new是否在别的树中
        //  Check x_new connects tree and other_tree
        if (other_tree.checkVisitedNode(x_new))
        {
            // 这里是计算索引，因为connection_nodes_set_的初始化代码是用的这种索引逻辑
            // connection_nodes_set_[connection_set_idx] 存储了两个树之间所有的连接节点
            // idx : i*(i-1)/2 + j when (i > j)
            int connection_set_idx = (other_tree.id > tree.id) ? other_tree.id * (other_tree.id - 1) / 2 + tree.id : tree.id * (tree.id - 1) / 2 + other_tree.id;

            // Update Connection between two trees
            // When x_new is the first connection node
            static int connection_number = 0;
            // 如果这个集合是空的，说明这是两个树的第一个连接节点
            // 那么需要把这个节点放到连接节点集合中
            if ((*(connection_nodes_set_[connection_set_idx])).empty())
            {
                // 插入当前的x_new到集合中
                (*(connection_nodes_set_[connection_set_idx])).insert(x_new);

                // 连接两个树，主要是处理联通关系
                //  Connect Two tree
                connectTwoTree_(tree, other_tree);
                // 计数增加！
                connection_number++;
            }

            // Update Distance Matrix & Optimal Connection Node
            // 这里是计算通过x_new 连接两数的distance，也就是cost的值
            double distance = tree.cost.at(x_new) + other_tree.cost.at(x_new);

            // 如果distance比两个数原来的距离小，那么就更新
            // distance_matrix_[tree.id][other_tree.id] 存的是两个树之间最小的距离矩阵
            // connection_node_matrix_[tree.id][other_tree.id] 存的是两个树之间最优的连接节点
            if (distance < distance_matrix_[tree.id][other_tree.id] - 0.1)
            {
                // 对称图
                distance_matrix_[tree.id][other_tree.id] = distance;
                distance_matrix_[other_tree.id][tree.id] = distance;

                // 更新最优连接节点
                connection_node_matrix_[tree.id][other_tree.id] = x_new;
                connection_node_matrix_[other_tree.id][tree.id] = x_new;
                // 这个是算法全局的一个flag
                // 这个flag是用在后面or求解TSP的时候，距离矩阵更新了才有重新优化求解的必要！
                is_distance_matrix_updated_ = true;
            }

            // Save the list of connection node for merge tree later
            // **pseudo_mode 模式下的工作流程是:**
            /**
            只为起点和终点建立真正的 RRT 树,作为父树。
            为其他目标建立 pseudo 树,受父树支配。
            pseudo 树扩展受限,只能在父树未覆盖的区域增长。
            记录所有 pseudo 树的连接节点。
            多棵 pseudo 树连接成联通图后,使用记录的连接节点将它们合并到父树中。
            最后只剩下起点树和终点树,在这两棵树间解决 RTSP。
            */
            if (setting_.pseudo_mode)
            {
                // Check edge(x_new, x_near) are already extended from other trees
                size_t x_parent = tree.parent.at(x_new);
                if (!other_tree.checkVisitedNode(x_parent) ||
                    (other_tree.parent.at(x_parent) != x_new &&
                     other_tree.parent.at(x_new) != x_parent))
                {
                    // update connection nodes set only it is not overlapped
                    (*connection_nodes_set_[connection_set_idx]).insert(x_new);
                }
            }
        }
    }
}

/**
 * 这个代码是判断所有的树是不是联通的，等价于：
 * std::equal(disjoint_set_parent_.begin() + 1,
        disjoint_set_parent_.end(), disjoint_set_parent_.begin());
*/
bool ImomdRRT::checkAllTreesConnected()
{
    int root = disjoint_set_parent_[0]; // 取第一棵树的根节点id
    for (int i = 1; i < tree_layers_.size(); ++i)
    {
        if (disjoint_set_parent_[i] != root)
        {
            // 存在树的根节点id与第一棵树不一致
            return false;
        }
    }
    // 所有树的根节点id都与第一棵树一致
    return true;
}

/**
 * 连接两个树，完成一个类似与并查集地操作，将大树和小树地联通关系进行合并
 * 最后根据这个联通关系，判断是不是所有地树都联通，设置一个flag：is_connected_graph_
 * 会用于优化TSP求解！
graph TD
A[开始] --> B{所有的树都已形成联通图?}
B -- 是 --> C[返回]
B -- 否 --> D[获取两树id]
D --> E{id1==id2?}
E -- 是 --> C
E -- 否 --> F{max_id < min_id?}
F -- 是 --> G[交换两个id,后面将大的合并到小的上]
F -- 否 --> H[设置max_id为min_id的子集]
H --> I[将max_id的所有儿子也设为min_id的子集]
I --> J[清空max_id的儿子列表]
G --> H
J --> K[检查是否所有地树都形成了联通图]
K -- 形成 --> L[设置标志位is_connected_graph_<br>避免不必要的TSP求解]
L --> C
K -- 未形成 --> C
*/
void ImomdRRT::connectTwoTree_(const tree_t &tree1, const tree_t &tree2)
{
    if (is_connected_graph_)
    {
        return;
    }

    // 这里是判断当前tree1和tree2的parent是不是一样的，如果是一样的，那么就是联通的，可以直接返回
    int min_tree_id = disjoint_set_parent_[tree1.id];
    int max_tree_id = disjoint_set_parent_[tree2.id];

    if (max_tree_id == min_tree_id)
    {
        return;
    }
    // 如果不一样，那么就要把两个树合并到一起，这里需要判断树的id的大小，因为我们需要把大的合并到小的上面
    else if (max_tree_id < min_tree_id)
    {
        std::swap(min_tree_id, max_tree_id);
    }

    // unifiy the disjoint set
    // value of disjoint set represents the smallest index of connected trees.
    // 这里是把大的树合并到小的树上面,所以把disjoint_set_parent_上max_tree_id设置为min_tree_id
    disjoint_set_parent_[max_tree_id] = min_tree_id;
    // 然后将大树添加到小树的children中
    disjoint_set_children_[min_tree_id].push_back(max_tree_id);

    // 循环大树地所有子节点
    for (int child : disjoint_set_children_[max_tree_id])
    {
        // 将子节点的parent设置为小树的id，也就是把小树设置为大树，以及大树地所有子节点地父亲
        disjoint_set_parent_[child] = min_tree_id;
        disjoint_set_children_[min_tree_id].push_back(child);
    }
    // 清空大树地儿子列表，因为他的儿子现在是小树地儿子了
    disjoint_set_children_[max_tree_id].clear();

    // 这里是判断所有的树其parent是不是一样的，也就是判断是不是所有的树都联通了，这时我们才设置is_connected_graph_
    // solveRTSP_函数中,只有当is_connected_graph_为true时,才解决RTSP，所以可以避免不必要的TSP求解！提高效率
    // Check whether the value of disjoint set is all same, which means connected graph
    is_connected_graph_ = std::equal(disjoint_set_parent_.begin() + 1,
                                     disjoint_set_parent_.end(), disjoint_set_parent_.begin());
}

void ImomdRRT::mergePseudoTrees_()
{
    bipedlab::debugger::debugColorTextOutput("[IMOMD] Merge Pseudo Trees", 5, BY);
    std::unordered_set<int> unmerged_pseudo_trees;
    for (int i = 1; i < (int)destinations_.size() - 1; ++i)
    {
        unmerged_pseudo_trees.insert(i);
    }

    // Merge pseudo trees into source_tree and target_tree alternatively
    int pseudo_tree_merge;
    int connection_set_idx;
    while (!unmerged_pseudo_trees.empty())
    {
        // Find pseudo tree that has the largest number of connection nodes with source_tree
        int max_connection_number = 0;
        for (int pseudo_tree_id : unmerged_pseudo_trees)
        {
            connection_set_idx = pseudo_tree_id * (pseudo_tree_id - 1) / 2;
            int connection_number = (*connection_nodes_set_[connection_set_idx]).size();
            if (connection_number > max_connection_number)
            {
                max_connection_number = connection_number;
                pseudo_tree_merge = pseudo_tree_id;
            }
        }
        // Merge pseudo tree with source_tree
        if (max_connection_number > 0)
        {
            mergeTwoTree_(tree_layers_[source_tree_id_], tree_layers_[pseudo_tree_merge]);
            unmerged_pseudo_trees.erase(pseudo_tree_merge);
        }

        // Find pseudo tree that has the largest number of connection nodes with target_tree
        max_connection_number = 0;
        for (int pseudo_tree_id : unmerged_pseudo_trees)
        {
            connection_set_idx = target_tree_id_ * (target_tree_id_ - 1) / 2 + pseudo_tree_id;
            int connection_number = (*connection_nodes_set_[connection_set_idx]).size();
            if (connection_number > max_connection_number)
            {
                max_connection_number = connection_number;
                pseudo_tree_merge = pseudo_tree_id;
            }
        }
        // Merge pseudo tree with target_tree
        if (max_connection_number > 0)
        {
            mergeTwoTree_(tree_layers_[target_tree_id_], tree_layers_[pseudo_tree_merge]);
            unmerged_pseudo_trees.erase(pseudo_tree_merge);
        }
    }

    // Update Distance Matrix & Optimal Connection Node
    double min_distance = distance_matrix_[source_tree_id_][target_tree_id_];
    connection_set_idx = target_tree_id_ * (target_tree_id_ - 1) / 2;
    for (auto connection_node : (*connection_nodes_set_[connection_set_idx]))
    {
        double distance = tree_layers_[source_tree_id_].cost.at(connection_node) +
                          tree_layers_[target_tree_id_].cost.at(connection_node);

        if (distance < min_distance)
        {
            distance_matrix_[source_tree_id_][target_tree_id_] = distance;
            distance_matrix_[target_tree_id_][source_tree_id_] = distance;

            connection_node_matrix_[source_tree_id_][target_tree_id_] = connection_node;
            connection_node_matrix_[target_tree_id_][source_tree_id_] = connection_node;

            is_distance_matrix_updated_ = true;
        }
    }
    is_merge_done_ = true;
}

void ImomdRRT::mergeTwoTree_(tree_t &parent_tree, tree_t &child_tree)
{
    bipedlab::debugger::debugColorTextOutput("[IMOMD] Merge Tree ", 4, BG);

    int connection_set_idx = (parent_tree.id > child_tree.id) ? parent_tree.id * (parent_tree.id - 1) / 2 + child_tree.id : child_tree.id * (child_tree.id - 1) / 2 + parent_tree.id;

    std::unordered_set<int> merged_nodes;
    // Trecing back the child_tree from connection node to root of child_tree
    // Merging Main Brach
    // connection_node ----> child_tree(root)
    for (size_t connection_node : (*connection_nodes_set_[connection_set_idx]))
    {
        size_t current_node = connection_node;
        bool is_optimal = true;

        while (current_node != child_tree.root && is_optimal)
        {
            size_t x_new = child_tree.parent[current_node];
            if (parent_tree.checkVisitedNode(x_new))
            {
                if (merged_nodes.find(x_new) != merged_nodes.end())
                {
                    is_optimal = false;
                }
            }
            else
            {
                connectNewNode_(parent_tree, x_new);
                updateExpandables(parent_tree, x_new, true);
                rewireTree_(parent_tree, x_new);
                updateConnectionTree_(parent_tree, x_new);

                merged_nodes.insert(x_new);
            }
            current_node = x_new;
        }
    }

    // Merging Sub branches of child_tree to parent_tree
    for (const auto &node : child_tree.parent)
    {
        if (parent_tree.checkVisitedNode(node.first))
        {
            continue;
        }

        size_t current_node = node.first;
        // Find branch need to merge by backtracking from fringe
        std::vector<size_t> branch = {current_node};
        while (!parent_tree.expandables.count(current_node))
        {
            current_node = child_tree.parent[current_node];
            branch.push_back(current_node);
        }

        // Merge branch
        for (auto it = branch.rbegin(); it != branch.rend(); ++it)
        {
            size_t x_new = *it;

            if (!parent_tree.checkVisitedNode(x_new))
            {
                connectNewNode_(parent_tree, x_new);
                updateExpandables(parent_tree, x_new, true);
                rewireTree_(parent_tree, x_new);
                updateConnectionTree_(parent_tree, x_new);
            }
        }
    }

    // Merge connection nodes of child_tree into parent_tree
    for (const auto &tree : tree_layers_)
    {
        if (tree.id != child_tree.id && tree.id != parent_tree.id)
        {
            // define connection_set idx
            int child_connection_set_idx = (child_tree.id > tree.id) ? child_tree.id * (child_tree.id - 1) / 2 + tree.id : tree.id * (tree.id - 1) / 2 + child_tree.id;

            int parent_connection_set_idx = (parent_tree.id > tree.id) ? parent_tree.id * (parent_tree.id - 1) / 2 + tree.id : tree.id * (tree.id - 1) / 2 + parent_tree.id;

            for (size_t connection_node : (*connection_nodes_set_[child_connection_set_idx]))
            {
                if (!parent_tree.checkVisitedNode(connection_node))
                {
                    (*connection_nodes_set_[parent_connection_set_idx]).insert(connection_node);
                }
            }
        }
    }

    child_tree.is_done = true;
}

void ImomdRRT::updatePath_()
{
    pthread_mutex_lock(&lock_);

    shortest_path_.clear();

    for (auto it = (*sequence_of_tree_id_rtsp_).begin();
         it != std::prev((*sequence_of_tree_id_rtsp_).end()); ++it)
    {
        int current_tree_id = *it;
        int parent_tree_id = *std::next(it, 1);

        // current_tree ----- connection_node ------ parent_tree
        size_t connection_node = connection_node_matrix_[current_tree_id][parent_tree_id];

        std::vector<size_t> tmp_path;
        // connection_node(exclude) ---> current_tree_root(include)
        size_t node = connection_node;
        while (node != tree_layers_[current_tree_id].root)
        {
            node = tree_layers_[current_tree_id].parent[node];
            tmp_path.push_back((*map_ptr_)[node].id);
        }

        // current_tree_root(include) ---> connection_node(exclude)
        for (size_t i = tmp_path.size() - 1; i < tmp_path.size(); --i)
        {
            shortest_path_.push_back(tmp_path[i]);
        }

        // connection_node(include) ---> parent_tree_root(exclude)
        node = connection_node;
        while (node != tree_layers_[parent_tree_id].root)
        {
            shortest_path_.push_back((*map_ptr_)[node].id);
            node = tree_layers_[parent_tree_id].parent[node];
        }
    }
    shortest_path_.push_back(destinations_.back());

    // Thread Lock and Siangl Condition
    is_path_updated_ = true;
    is_distance_matrix_updated_ = false;

    pthread_mutex_unlock(&lock_);
    pthread_cond_signal(&cond_get_path_);
}

void ImomdRRT::solveRTSP_()
{
    double path_cost = 0;
    std::shared_ptr<std::vector<int>> tmp_sequence;

    if (is_connected_graph_ && is_distance_matrix_updated_)
    {
        if (!setting_.pseudo_mode)
        {
            std::tie(path_cost, tmp_sequence) = eci_gen_solver_.solveRTSP(
                getDistanceMatrix(), source_tree_id_, target_tree_id_);

            if (path_cost < shortest_path_cost_)
            {
                sequence_of_tree_id_rtsp_ = tmp_sequence;
                shortest_path_cost_ = path_cost;
                updatePath_();
                logData_();
            }
            else
            {
                bipedlab::debugger::debugColorOutput("Not found the shorter path | ", path_cost, 8, BY);
            }
        }
        else
        {
            if (!is_merge_done_)
            {
                mergePseudoTrees_();
                sequence_of_tree_id_rtsp_ = std::make_shared<std::vector<int>>(
                    std::vector<int>{source_tree_id_, target_tree_id_});
            }
            shortest_path_cost_ = distance_matrix_[source_tree_id_][target_tree_id_];
            updatePath_();
            logData_();
        }
    }
}

void ImomdRRT::logData_()
{
    double cpu_time = bipedlab::timing::spendElapsedTime(start_time_);
    int tree_size = 0;
    for (const auto &tree : tree_layers_)
    {
        tree_size += tree.parent.size();
    }

    std::cout << "Elapsed time[s]: " << std::setw(10) << cpu_time << " | "
              << "Path Cost[m]: " << std::setw(10) << shortest_path_cost_ << " | "
              << "Tree Size: " << std::setw(10) << tree_size
              << std::endl;

    size_t precision = 4;

    if (setting_.log_data)
    {
        command_csv_
            << bipedlab::utils::toStringWithPrecision(cpu_time, precision)
            << bipedlab::utils::toStringWithPrecision(shortest_path_cost_, precision)
            << bipedlab::utils::toStringWithPrecision(tree_size, precision)
            << endrow;
    }
}
