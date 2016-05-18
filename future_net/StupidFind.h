#ifndef __STUPIDFIND_H__
#define __STUPIDFIND_H__

#include "ListBfsDijstra.h"
#include <vector>
using std::vector;

#define SOLUS_NUM   8
#define _MAIN_PATH  0
#define _BACK_PATH  1

struct st_sf_pair;
struct st_sf_path;
struct st_sf_solu;

class StupidFind: public ListBfsDijstra
{
public:
    StupidFind(char *topo[MAX_EDGE_NUM], int edge_num, char *demand[MAX_DEMAND_NUM], int demand_num);
    bool run                    (st_sf_path &m, st_sf_path &b);
    void check_legal_path       (st_sf_path &path);
private:
    void select_solution_pair   (vector<st_sf_solu> &solus, st_sf_path &m, st_sf_path &b);
    void adjust_repeat_sides    (vector<st_sf_path> &ms, vector<st_sf_path> &bs, vector<st_sf_pair> &bp, vector<st_sf_solu> &solus);
    void ars_replace_repeats    (st_sf_path &path, vector<int> &rs, vector<vector<int>> &cand);
    void ars_rs_positions       (st_sf_path &path, vector<int> &rs, vector<int> &pos);
    void ars_find_candinates    (st_sf_path &p1, st_sf_path &p2, vector<int> &rs, vector<vector<int>> &cand);
    vector<int> 
         ars_find_del_nodes     (vector<bool> &v, int src, int dst);
    void ars_del_sides          (vector<int> &sides);
    void ars_add_sides          (vector<int> &sides);
    void find_priority_pairs    (vector<st_sf_path> &ms, vector<st_sf_path> &bs, vector<st_sf_pair> &bp);
    void make_main_back_pairs   (vector<st_sf_path> &ms, vector<st_sf_path> &bs, vector<st_sf_pair> &bp);
    bool get_main_back_paths    (vector<st_sf_path> &ms, vector<st_sf_path> &bs, size_t L);

    void sf_show_paths          (vector<st_sf_path> &ps);
    void sf_show_pairs          (vector<st_sf_pair> &ps);
    void sf_show_solu           (vector<st_sf_solu> &s);
    void sf_show_solu           (st_sf_solu &s);
    void sf_show_solu           (st_sf_path &m, st_sf_path &b);
};

struct st_sf_pair
{
    size_t first;
    size_t second;
    int rsn;        // repeat sides num
    int cost;
    st_sf_pair(size_t f=0, size_t s=0, int r=0, int c=INT_INF)
    :first(f), second(s), rsn(r), cost(c){}
    bool operator<(const st_sf_pair &p) const
    {
        return rsn < p.rsn ? true : 
               (rsn == p.rsn ? cost < p.cost : false); 
    }
};

struct st_sf_path
{
    int cost;
    vector<int>  sides;
    bool valid[MAX_EDGE_NUM];
    st_sf_path(){}
    st_sf_path(st_lbd_path &lp, StupidFind &sf)
    {
        cost = lp.cost;
        auto iter1 = lp.node.begin();
        auto iter2 = iter1;
        std::advance(iter2, 1);
        for (; iter2 != lp.node.end(); iter1++, iter2++){
            int s = sf.get_edge(*iter1, *iter2);
            sides.push_back(s);
            valid[s] = true;
        }
    }
    size_t size(){return sides.size();}
    void set_valid(vector<bool> &v, StupidFind &sf)
    {
        for (auto e: sides){
            int s = sf.edges[e].src;
            int d = sf.edges[e].dst;
            v[s] = v[d] = false;
        }
    }
    void adjust_cost(StupidFind &sf)
    {
        cost = 0;
        for (auto e: sides) 
            cost += sf.edges[e].weight;
    }
    int repeat_sides_num(st_sf_path &sp)
    {
        int rsn = 0;
        for (auto e: sp.sides){
            if (valid[e]) rsn++;
        }
        return rsn;
    }
    vector<int> repeat_sides(st_sf_path &sp)
    {
        size_t m = sp.sides.size() > sides.size() ? 
                   sp.sides.size(): sides.size();
        vector<int> rs(m, 0);
        size_t rsn = 0;
        for (auto e: sp.sides){
            if (valid[e]) rs[rsn++] = e;            
        }
        rs.erase(rs.begin() + rsn, rs.end());
        
        return rs;
    }
};

struct st_sf_solu
{
    st_sf_path first;
    st_sf_path second;
    int rsn;        // repeat sides num
    int cost;
    //st_sf_solu(){}
    st_sf_solu(st_sf_path &f, st_sf_path &s, int r=0, int c=INT_INF):first(f), second(s), rsn(r), cost(c)
    {
        first = f;
        second = s;
        cost = f.cost + s.cost;
        rsn = f.repeat_sides_num(s);
    }
    bool operator<(const st_sf_solu &p) const
    {
        return rsn < p.rsn ? true : 
               (rsn == p.rsn ? cost < p.cost : false); 
    }
};

#endif