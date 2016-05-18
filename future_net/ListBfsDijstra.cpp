#include "IceMinPQ.h"
#include "ListBfsDijstra.h"
#include <algorithm>
#include <random>
#include <iostream>
using std::cout; using std::endl; using std::flush;
using std::swap; using std::sort;

ListBfsDijstra::ListBfsDijstra(char *topo[MAX_EDGE_NUM], int edge_num, char *demand[MAX_DEMAND_NUM], int demand_num)
:CommonStruct(topo, edge_num, demand, demand_num){}

//--------------- bfs spread + dij + cut ----------------------------------------
bool ListBfsDijstra::lbd_run(vector<st_lbd_path> &paths, int path_id, size_t L, bool inv_dir)
{
    if (inv_dir)  inverse_direct();

    is_cost_one = false;
    bool f = lbd_run(paths, path_id, L);
    
    if (inv_dir){
        inverse_paths_direct(paths);
        inverse_direct();
    }
    return f;
}
void ListBfsDijstra::inverse_paths_direct(vector<st_lbd_path> &ps)
{
    for (size_t i = 0; i < ps.size(); i++)
        ps[i].node.reverse();
}
bool ListBfsDijstra::lbd_run(vector<st_lbd_path> &paths, int path_id, size_t L)
{
    _PATH_ID = path_id;
    vector<bool> valid(MAX_NODES_NUM, true);
    
    if (demand_nodes_num[_PATH_ID] == 0){
        st_lbd_path path;
        lbd_dij_two_nodes(valid, source_node, destin_node, path);
        paths.push_back(path);
        return path.node.size() != 1;
    }
    
    
    vector<st_lbd_fork> forks;
    lbd_start_src(valid, paths); // s -> d1
    #ifdef DEBUG_LBD
    cout << "[after start]" << endl; lbd_dij_show_paths(paths, _PATH_ID);
    #endif

    List_Adaptive LA(L, *this); 
    for (int phase = 1; !paths.empty() && phase < demand_nodes_num[_PATH_ID]; phase++)
    {
        #ifdef DEBUG_LBD
        cout << "\n[[-----------------------" << phase << "-th----------------------]]" << endl;
        #endif
        LA.setN(phase);
        LA.setP0(paths.size());

        lbd_update_forks(paths, forks);
        LA.setF(forks.size());
        #ifdef DEBUG_LBD
        cout << "[after update_forks]" << endl; lbd_show_forks(forks, _PATH_ID);
        #endif

        lbd_select_forks(paths, forks, L);
        #ifdef DEBUG_LBD
        cout << "[after select_forks]" << endl; lbd_show_forks(forks, _PATH_ID);
        #endif

        lbd_extend_paths(paths, forks);
        LA.setT();
        LA.setP1(paths.size());
        #ifdef DEBUG_LBD
        cout << "[after extend_paths]" << endl; lbd_dij_show_paths(paths, _PATH_ID);
        #endif
        #ifdef DEBUG_ADJUST_SHOW
        LA.show();
        #endif
        #ifdef DEBUG_ADJUST_L
        LA.adjust(L);
        #endif
    }// d_n-1 -> d_n
    lbd_end_dst(paths); //d_n -> e
    lbd_select_solu(paths, L);
    #ifdef DEBUG_LBD
    cout << "[after end_dst]" << endl; lbd_dij_show_paths(paths, _PATH_ID);
    #endif
    return true;
}
bool ListBfsDijstra::lbd_select_solu(vector<st_lbd_path> &ps, size_t L)
{
    size_t split = 0;
    for (size_t i = 0; i < ps.size(); i++){
        if (ps[i].back() == destin_node){ // is a solution 
            swap(ps[split++], ps[i]);
        }
    }ps.erase(ps.begin() + split, ps.end());
    if (ps.size() > L){
        nth_element(ps.begin(), ps.begin() + L, ps.end());
        ps.erase(ps.begin() + L, ps.end());
    }
    
    return 0 != split;  // have solution
}
void ListBfsDijstra::lbd_end_dst(vector<st_lbd_path> &ps)
{
    vector<bool> valid(MAX_NODES_NUM, true);
    size_t origin_len = ps.size();
    for (size_t i = 0; i < origin_len; i++){
        st_lbd_path last;
        ps[i].set_valid(valid);
        if(lbd_dij_two_nodes(valid, ps[i].back(), destin_node, last, ps[i].cost, is_cost_one)){
            if (!is_density_graph){
                #ifdef ESCAPE_TRAP_DST
                vector<st_lbd_path> fork_trap;
                vector<st_lbd_path> tmp_fork(1, last);
                lbd_assitant_forks(valid, tmp_fork, fork_trap, ps[i].cost, is_cost_one);
                if (!fork_trap.empty()){
                    ps.push_back(ps[i]);
                    ps.back().append(fork_trap[0]);
                }
                #endif
            }
            ps[i].append(last);
        }
    }
}
void ListBfsDijstra::lbd_extend_paths(vector<st_lbd_path> &ps, vector<st_lbd_fork> &fs)
{
    if (ps.size() < fs.size())
        ps.resize(fs.size()); // after selet, fs.size() <= L

    vector<size_t> alive, dead, counts(ps.size(), 0);
    lbd_ep_dead_alive_path(ps, fs, alive, dead, counts);
    for (size_t i = 0; i < fs.size(); i++){
        size_t pi = fs[i].path_index;
        if (counts[pi] == 1){
            ps[pi].append(fs[i].path);
            counts[pi]--;
        }else if(counts[pi] > 1){
            size_t ep = dead.back(); dead.pop_back();
            ps[ep] = ps[pi];
            counts[pi]--;
            ps[ep].append(fs[i].path);
        }
    }
    if (ps.size() > fs.size()) // in ending phases, less paths can be extended because fewer node kept
        lbd_ep_shrink_paths(ps, dead);
}
void ListBfsDijstra::lbd_ep_shrink_paths(vector<st_lbd_path> &ps, vector<size_t> &dead)
{
    vector<bool> path_alive(ps.size(), true);
    for (size_t i = 0; i < dead.size(); i++)
        path_alive[dead[i]] = false;
    size_t split = 0;
    for (size_t i = 0; i < ps.size(); i++){
        if (path_alive[i])
            ps[split++] = ps[i];
    }ps.erase(ps.begin()+split, ps.end());
}
void ListBfsDijstra::lbd_ep_dead_alive_path(vector<st_lbd_path> &ps, vector<st_lbd_fork> &fs,
                                          vector<size_t> &alive, vector<size_t> &dead, vector<size_t> &counts)
{
    for (size_t i = 0; i < fs.size(); i++)
        counts[fs[i].path_index]++;
    for (size_t i = 0; i < ps.size(); i++)
        counts[i] == 0 ? dead.push_back(i) : alive.push_back(i);
}
void ListBfsDijstra::lbd_select_forks(vector<st_lbd_path> &ps, vector<st_lbd_fork> &fs, size_t L)
{
    if (L >= fs.size()) return;
    lbd_select_ordered_fitness(fs, L);
    //lbd_select_roulette_wheel(fs, L);
}
void ListBfsDijstra::lbd_select_ordered_fitness(vector<st_lbd_fork> &fs, size_t L)
{
	nth_element(fs.begin(), fs.begin() + L, fs.end());
    fs.erase(fs.begin() + L, fs.end());
}
void ListBfsDijstra::lbd_select_roulette_wheel(vector<st_lbd_fork> &fs, size_t L)
{
    sort(fs.begin(), fs.end());

    double total_fitness = 0;
    for (auto &e: fs){
        total_fitness += e.fitness;
    }
    vector<double> select_prob(fs.size(), 0);
    for (size_t i = 0; i < fs.size(); i++){
        select_prob[i] = fs[i].fitness / total_fitness; 
    }
    vector<double> sum_prob(fs.size(), 0);
    sum_prob[0] = select_prob[0];
    for (size_t i = 1; i < fs.size(); i++){
        sum_prob[i] = select_prob[i] + sum_prob[i-1]; 
    }
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 1.0);
    vector<size_t> indices(L, 0);
    for (size_t i = 0; i < L; i++){
        indices[i] = lbd_select_rw_pos(sum_prob, dis(gen));
    }unique(indices.begin(), indices.end());
    vector<st_lbd_fork> fs_tmp;
    for (auto e: indices)
        fs_tmp.push_back(fs[e]);
    fs = fs_tmp;
}
size_t ListBfsDijstra::lbd_select_rw_pos(vector<double> sum_prob, double rand)
{
    for (size_t i = 0; i < sum_prob.size(); i++)
        if (sum_prob[i] > rand)
            return i;
    return 0;        
}
void ListBfsDijstra::lbd_update_forks(vector<st_lbd_path> &ps, vector<st_lbd_fork> &fs)
{
    fs.clear();
    vector<bool> valid(MAX_NODES_NUM, true);
    for (size_t i = 0; i < ps.size(); i++){
        ps[i].set_valid(valid);
        
        vector<st_lbd_path> fork;
        lbd_dij_other_demands(valid, ps[i].back(), fork, ps[i].cost, is_cost_one);
        for (size_t j = 0; j < fork.size(); j++){
            fs.push_back(st_lbd_fork(i, fork[j]));
            fs.back().set_node_num(ps[i].node.size());
            fs.back().set_fitness();
        }
        if (!is_density_graph){
            #ifdef ESCAPE_TRAP_INNER
            vector<st_lbd_path> fork_trap;
            lbd_assitant_forks(valid, fork, fork_trap, ps[i].cost, is_cost_one);
            for (size_t j = 0; j < fork_trap.size(); j++){
                fs.push_back(st_lbd_fork(i, fork_trap[j]));
                fs.back().set_node_num(ps[i].node.size());
                fs.back().set_fitness();
            }
            #endif
        }
    }
}
void ListBfsDijstra::lbd_start_src(vector<bool> &valid, vector<st_lbd_path> &ps)
{
    lbd_dij_other_demands(valid, source_node, ps, 0, is_cost_one);
    
    if (!is_density_graph){
        #ifdef ESCAPE_TRAP_SRC
        vector<st_lbd_path> fork_trap;
        lbd_assitant_forks(valid, ps, fork_trap, 0, is_cost_one);
        ps.insert(ps.end(), fork_trap.begin(), fork_trap.end());
        #endif
    }
    /*
    if (ps.size() > 40){
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(ps.begin(), ps.end(), g);
        ps.erase(ps.begin() + 40, ps.end());
    }*/
}
void ListBfsDijstra::lbd_assitant_forks(vector<bool> &valid, vector<st_lbd_path> &fork, vector<st_lbd_path> &fork_trap, int init_cost, bool is_cost_one)
{
    // hide demand nodes
    for (int i = 0; i < demand_nodes_num[_PATH_ID]; i++)
        valid[demand_nodes[_PATH_ID][i]] = false;

    for (auto &e: fork){
        auto cur_node = e.node.begin();
        auto next_node = cur_node;
        std::advance(cur_node, 1);
        std::advance(next_node, 2);
        
        valid[e.node.back()] = true;
        for (; next_node != e.node.end(); cur_node++, next_node++){
            //if (get_degree_out(*cur_node) <= 1) continue;
            fork_trap.push_back(st_lbd_path(0));
            valid[*cur_node] = false;
            bool found = lbd_assitant_fork(valid, e, fork_trap.back(), init_cost, is_cost_one);
            valid[*cur_node] = true;
            if (!found) fork_trap.pop_back();
            else break;
        }
        valid[e.node.back()] = false;
    }
    // recover demand nodes
    for (int i = 0; i < demand_nodes_num[_PATH_ID]; i++)
        valid[demand_nodes[_PATH_ID][i]] = true;
}
bool ListBfsDijstra::lbd_assitant_fork(vector<bool> &valid, st_lbd_path &fork, st_lbd_path &fork_trap, int init_cost, bool is_cost_one)
{
    int s = fork.node.front();
    int d = fork.node.back();
    lbd_dij_two_nodes(valid, s, d, fork_trap, init_cost, is_cost_one); // return [,]
    return fork_trap.node.size() != 1;
}
bool ListBfsDijstra::lbd_dij_two_nodes(vector<bool> &valid, int src, int dst, st_lbd_path &p, int init_cost, bool is_cost_one)
{// return [,]
    vector<int>  node_to(MAX_NODES_NUM, -1);
    vector<int>  dist_to(MAX_NODES_NUM, INT_INF);
    IceMinPQ<int, double> pq;
    // Dijstra
    dist_to[src] = 0;
    pq.insert(src, 0);
    while(!pq.empty()){
        int min = pq.top(); pq.pop();
        lbd_dij_relax(valid, pq, min, node_to, dist_to, is_cost_one);
        if (min == dst && min != src) break;
    }lbd_dij_recover_path(dst, p, node_to, dist_to, init_cost, is_cost_one);
    p.node.push_front(src);
    return p.cost != INT_INF;
}
void ListBfsDijstra::lbd_dij_other_demands(vector<bool> valid, int src, vector<st_lbd_path> &paths, int init_cost, bool is_cost_one)
{// return [,]
    vector<int>  demands; // demand nodes that can be achieved only passing common nodes.
    vector<int>  node_to(MAX_NODES_NUM, -1);
    vector<int>  dist_to(MAX_NODES_NUM, INT_INF);
    IceMinPQ<int, double> pq;
   // Dijstra-like
    dist_to[src] = 0, pq.insert(src, 0);
    while(!pq.empty()){
        int min = pq.top(); pq.pop();
        if (is_demand(_PATH_ID, min) && min!=src){
            demands.push_back(min), valid[min] = false;
        }else{
            lbd_dij_relax(valid, pq, min, node_to, dist_to, is_cost_one);
        }
    }
    // recover all paths
    for (size_t i = 0; i < demands.size(); i++){
        paths.push_back(st_lbd_path()); 
        lbd_dij_recover_path(demands[i], paths.back(), node_to, dist_to, init_cost, is_cost_one);
        paths.back().node.push_front(src);
    }
}
void ListBfsDijstra::lbd_dij_relax(vector<bool> &valid, IceMinPQ<int, double> &pq, int min, vector<int> &node_to, vector<int> &dist_to, bool is_cost_one)
{
    int out_deg = get_degree_out(min);
    for (int i = 0; i < out_deg; i++){ // relax
        int child = get_child(min, i);
        if (!valid[child]) continue;
        int w = (is_cost_one ? 1 : get_weight_out(min, i));
        int c = dist_to[min] + w;
        if (dist_to[child] > c){
            dist_to[child] = c;
            node_to[child] = min;
            pq.contains(child) ?
            pq.change(child, dist_to[child]):
            pq.insert(child, dist_to[child]);
        }
    }
}
void ListBfsDijstra::lbd_dij_recover_path(int dst, st_lbd_path &p, vector<int> &node_to, vector<int> &dist_to, int init_cost, bool is_cost_one)
{// return (,]
    if (dist_to[dst] == INT_INF) return;

    int e = dst;
    int extra_cost = 0;
    while (node_to[e] != -1){
        if (is_cost_one) extra_cost += get_weight(node_to[e], e);
        p.insert(e);
        e = node_to[e];
    }
    if (is_cost_one){
        p.cost = init_cost + extra_cost;
    }else{
        p.cost = init_cost + dist_to[dst];
    }
}

void ListBfsDijstra::lbd_dij_show_paths(vector<st_lbd_path> &paths, int path_id)
{
    for (size_t i = 0; i < paths.size(); i++){
        cout << i << ": ";
        lbd_dij_show_path(paths[i], path_id);
        cout << endl;
    }
}
void ListBfsDijstra::lbd_dij_show_path(st_lbd_path &path, int path_id)
{
    if (path.node.empty()) return;
    cout << path.cost << "|";
    lbd_show_list(path.node, path_id);
}
void ListBfsDijstra::lbd_show_forks(vector<st_lbd_fork> &fs, int path_id)
{
    for (size_t i = 0; i < fs.size(); i++){
        cout << fs[i].path_index << ":"
             << fs[i].path.cost << "/";
        lbd_show_list(fs[i].path.node, path_id);
        cout << endl;
    }
}
void ListBfsDijstra::lbd_show_list(list<int> &nodes, int path_id)
{
    list<int>::iterator 
    iter = nodes.begin(); 

    for (; iter != nodes.end(); iter++){
        if (is_demand(path_id, *iter))
            cout << "[" <<  *iter << "]" << "->";
        else
            cout << *iter << "->";
    }cout << "|";
}
