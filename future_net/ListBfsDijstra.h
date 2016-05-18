#ifndef __LISTBFSDIJSTRA_H__
#define __LISTBFSDIJSTRA_H__

#include "CommonStruct.h"
#include "IceMinPQ.h"
#include <list>
using std::list;
#include <iostream>
using std::cout; using std::endl;

struct st_lbd_path;
struct st_lbd_fork;

class ListBfsDijstra : public CommonStruct
{
public:
    ListBfsDijstra(char *topo[MAX_EDGE_NUM], int edge_num, char *demand[MAX_DEMAND_NUM], int demand_num);
    void lbd_show_list			(list<int> &nodes, int path_id);
    void lbd_show_forks         (vector<st_lbd_fork> &fs, int path_id);
    void lbd_dij_show_paths     (vector<st_lbd_path> &paths, int path_id);
    void lbd_dij_show_path      (st_lbd_path &path, int path_id);
public:
    bool lbd_run(vector<st_lbd_path> &paths, int path_id, size_t L, bool inv_dir);
	bool lbd_run(vector<st_lbd_path> &paths, int path_id, size_t L);

private:
    void inverse_paths_direct   (vector<st_lbd_path> &ps);
    void inverse_paths_weight   (vector<st_lbd_path> &ps);
    bool lbd_select_solu        (vector<st_lbd_path> &ps, size_t L);
    void lbd_end_dst            (vector<st_lbd_path> &ps);
    void lbd_extend_paths       (vector<st_lbd_path> &ps, vector<st_lbd_fork> &fs);
    void lbd_ep_shrink_paths    (vector<st_lbd_path> &ps, vector<size_t> &dead);
    void lbd_ep_dead_alive_path (vector<st_lbd_path> &ps, vector<st_lbd_fork> &fs, vector<size_t> &alive, vector<size_t> &dead, vector<size_t> &counts);
    void lbd_select_forks       (vector<st_lbd_path> &ps, vector<st_lbd_fork> &fs, size_t L);
    void lbd_select_ordered_fitness(vector<st_lbd_fork> &fs, size_t L);
    void lbd_select_roulette_wheel(vector<st_lbd_fork> &fs, size_t L);
    size_t lbd_select_rw_pos    (vector<double> sum_prob, double rand);
    void lbd_update_forks       (vector<st_lbd_path> &ps, vector<st_lbd_fork> &fs);
    void lbd_assitant_forks     (vector<bool> &valid, vector<st_lbd_path> &fork, vector<st_lbd_path> &fork_trap, int init_cost, bool is_cost_one=false);
    bool lbd_assitant_fork      (vector<bool> &valid, st_lbd_path &fork, st_lbd_path &fork_trap, int init_cost, bool is_cost_one=false);
    void lbd_start_src          (vector<bool> &valid, vector<st_lbd_path> &paths);
protected:    
    bool lbd_dij_two_nodes      (vector<bool> &valid, int src, int dst, st_lbd_path &p, int init_cost=0, bool is_cost_one=false);
    void lbd_dij_other_demands  (vector<bool> valid, int src, vector<st_lbd_path> &paths, int init_cost=0, bool is_cost_one=false);
    void lbd_dij_relax          (vector<bool> &valid, IceMinPQ<int, double> &pq, int min, vector<int> &node_to, vector<int> &dist_to, bool is_cost_one=false);
    void lbd_dij_recover_path   (int dst, st_lbd_path &p, vector<int> &node_to, vector<int> &dist_to, int init_cost=0, bool is_cost_one=false);
    bool is_cost_one;
    
public:    
    int _PATH_ID;
};

struct st_lbd_path
{
    list<int>	 node;
    int          cost;
    vector<bool> valid;
    st_lbd_path(int c=INT_INF):cost(c)
    {valid.resize(MAX_NODES_NUM,false);}
    void set_valid(vector<bool> &v)
    {
        for (size_t i = 0; i < v.size(); i++)
            v[i] = !valid[i];
    }
    bool empty()      {return node.empty();}
    int  back()       {return node.back();}
    bool have(int n)  {return valid[n];}
    void append(int n){node.push_back(n); valid[n]=true;}
    void insert(int n){node.push_front(n);valid[n]=true;}
    void append(st_lbd_path &p)
    {
        cost = p.cost; // init_cost!=0
        p.node.pop_front();
        node.splice(node.end(), p.node);
        for (size_t i = 0; i < MAX_NODES_NUM; i++)
            valid[i] = valid[i] | p.valid[i];
    }
    bool operator<(const st_lbd_path &a) const 
    {
        return cost*node.size() < a.cost*a.node.size();
    }
    bool operator==(const st_lbd_path &a) const
    {
        size_t l1 = node.size(), l2 = a.node.size();
        if (l1 == l2 && cost == a.cost){
            auto iter1 = node.begin();
            auto iter2 = a.node.begin();
            for (; iter1 != node.end(); iter1++, iter2++)
                if (*iter1 != *iter2)
                    return false;
            return true;
        }else{
            return false;
        }
    }
};
struct st_lbd_fork
{
    size_t      path_index;
    st_lbd_path path;
    size_t      nodes_num;
    double      fitness;
    
    st_lbd_fork(size_t pi, st_lbd_path p)
    :path_index(pi), path(p){}
    void set_node_num(size_t nn)
    {
        nodes_num = nn + path.node.size();
    }
    void set_fitness()
    {
        fitness = path.cost * nodes_num;
    }
    bool operator<(const st_lbd_fork &a)const
    {
        return fitness < a.fitness;
    }
};
class List_Adaptive
{
public:
    List_Adaptive(size_t l, ListBfsDijstra &lbd){
        n = 0; cur_paths_sum = 0;
        data = new st_data[lbd.demand_nodes_num[lbd._PATH_ID] + 1]; 
        set_time_slice(lbd);
        start_time = clock();
        last_time = start_time;
    }
    ~List_Adaptive(){delete[] data;}
    void adjust(size_t &L)
    {
        cur_time = clock();
        double time_used_ms = 1.0*(cur_time-start_time)/1000;
        double time_rest_ms = total_time_ms - time_used_ms;
        // TODO:
        int rest_demad_num = demand_num - n;
        double L_new = 1.0*cur_paths_sum / time_used_ms * time_rest_ms / rest_demad_num;
        
        if (L_new < 0) L = 0;
        else if (L_new > MAX_LIST_WIDTH) L = MAX_LIST_WIDTH;
        else L = L_new;
    }
    void setN (size_t x){n  = x;}
    void setL (size_t l){data[n].L  = l;}
    void setP0(size_t x){data[n].P0 = x; cur_paths_sum += x;}
    void setF (size_t x){data[n].F  = x;}
    void setP1(size_t x){data[n].P1 = x;}
    void setT()
    {
        cur_time = clock();
        data[n].T = 1.0*(cur_time-last_time)/1000;
        last_time = cur_time;
    }
    void show()
    {
        cout << n << ":\t["
             << data[n].T << "],"
             << data[n].P0 << "/" << data[n].F << "/" << data[n].P1 << "\t"
             << total_time_ms << " ms." << endl;
    }
private:
    void set_time_slice(ListBfsDijstra &lbd)
    {
        demand_num = lbd.demand_nodes_num[lbd._PATH_ID];
        int D0 = lbd.demand_nodes_num[lbd._PATH_ID];
        int D1 = lbd.demand_nodes_num[1-lbd._PATH_ID];
        int total_demad_num = D0 + D1;
        
        if (D0 != 0 && D1 != 0){
            total_time_ms = 
                1.0 * (TOTAOL_SEC*1000 - TIME_FOR_SAVING) *
                demand_num / total_demad_num; 
        }else if (total_demad_num == 0){
            total_time_ms = 1.0 * (TOTAOL_SEC*1000 - TIME_FOR_SAVING) / 2;
        }else if (D0 == 0){
            total_time_ms = 1.0 * (TOTAOL_SEC*1000 - TIME_FOR_SAVING) / 4;
        }else{
            total_time_ms = 1.0 * (TOTAOL_SEC*1000 - TIME_FOR_SAVING) / 4 * 3;
        }
    }
private:
    size_t n;   // the n-th demand node 
    int demand_num;
    int cur_paths_sum;
    double total_time_ms;

    struct st_data{
        size_t L;   // list width
        size_t P0;  // paths num
        size_t F;   // forks num
        size_t P1;  // new paths num
        double T;   // P0 -> F
        bool if_back_path;
        st_data(){L = P0 = F = P1 = T = 0; if_back_path = false;}
    }; //NOTE: data[t].P0 = data[t-1].P1   
    clock_t start_time; 
    clock_t last_time;
    clock_t cur_time;
    st_data* data;
};

#endif

