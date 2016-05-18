#include "types.h"
#include "StupidFind.h"
#include <iostream>
using std::nth_element;
using std::min_element;
using std::cout; using std::endl; using std::flush;

StupidFind::StupidFind(char *topo[MAX_EDGE_NUM], int edge_num, char *demand[MAX_DEMAND_NUM], int demand_num)
:ListBfsDijstra(topo, edge_num, demand, demand_num){}

bool StupidFind::run(st_sf_path &m, st_sf_path &b)
{
    vector<st_sf_path> ms, bs;  // main & back paths
    vector<st_sf_pair> bp;      // best pairs
    size_t L = init_list_width;
    bool ret = get_main_back_paths(ms, bs, L);
    #ifdef DEBUG_SF_SIMPLE
    show_time("get paths...");
    cout << "main:" << ms.size() << ", back:" << bs.size() << endl;
    #ifdef DEBUG_SF
    cout << "Mains: " << endl; sf_show_paths(ms); 
    cout << "Backs: " << endl; sf_show_paths(bs);
    #endif
    #endif
    if (ret) return false;

    make_main_back_pairs(ms, bs, bp);
    #ifdef DEBUG_SF_SIMPLE
    show_time("make pairs...");
    //cout << "pairs:" << bp.size() << endl;
    #ifdef DEBUG_SF
    sf_show_pairs(bp);
    #endif
    #endif

    find_priority_pairs(ms, bs, bp);
    #ifdef DEBUG_SF_SIMPLE
    show_time("find pairs...");
    //cout << "reserve:" << bp.size() << endl; sf_show_pairs(bp);
    #endif

    vector<st_sf_solu> solu;
    adjust_repeat_sides(ms, bs, bp, solu);
    #ifdef DEBUG_SF_SIMPLE
    show_time("adjust sides...");
    #ifdef DEBUG_SF
    sf_show_solu(solu);
    #endif
    #endif

    select_solution_pair(solu, m, b);
    #ifdef DEBUG_SF_SIMPLE
    show_time("select solu...");
    for (auto &e: solu) cout << e.rsn << ", " << e.cost << "\t"; cout << endl; 
    check_legal_path(m);
    check_legal_path(b);
    #ifdef DEBUG_SF
    sf_show_solu(solu);
    #endif
    sf_show_solu(m, b);
    #endif

    return true;
}
void StupidFind::select_solution_pair(vector<st_sf_solu> &solus, st_sf_path &m, st_sf_path &b)
{
    nth_element(solus.begin(), solus.begin() + 1, solus.end());
    m = solus[0].first;
    b = solus[0].second;
}
void StupidFind::adjust_repeat_sides(vector<st_sf_path> &ms, vector<st_sf_path> &bs, vector<st_sf_pair> &bp, vector<st_sf_solu> &solus)
{
    //cout << "adjust_repeat_sides" << endl;    
    for (size_t i = 0; i < bp.size(); i++){
    //cout << "i:" << i << "," << endl;        
        st_sf_path p1 = ms[bp[i].first];
    //cout << "z" << endl;        
        st_sf_path p2 = bs[bp[i].second];
    //cout << "a" << endl;
        vector<int> rs = p1.repeat_sides(p2);
    //cout << "b" << endl;        
        if (rs.empty()){ 
            solus.push_back(st_sf_solu(p1, p2)); continue;
        }
        _PATH_ID = _BACK_PATH;
        vector<vector<int>> cand;        
    //cout << "c" << endl;
        ars_find_candinates(p1, p2, rs, cand);
    //cout << "d" << endl;
        ars_replace_repeats(p2, rs, cand);
    //cout << "e" << endl;
        solus.push_back(st_sf_solu(p1, p2));
    //cout << "f" << endl;
    }
}
void StupidFind::ars_replace_repeats(st_sf_path &path, vector<int> &rs, vector<vector<int>> &cand)
{
    vector<int> pos;
    ars_rs_positions(path, rs, pos);
    for (size_t i = 0; i < rs.size(); i++){
        auto &s = path.sides;
        s.insert(s.erase(s.begin() + pos[i]), cand[i].begin(), cand[i].end());
        for (size_t j = 0; j < pos.size(); j++)
            if (pos[j] > pos[i])
                pos[j] += cand[i].size() - 1; // -1 is the deleted side
    }path.adjust_cost(*this);
}
void StupidFind::ars_rs_positions(st_sf_path &path, vector<int> &rs, vector<int> &pos)
{
    for (auto e: rs){
        for (size_t i = 0; i < path.size(); i++)
            if (path.sides[i] == e){
                pos.push_back(i); break;
            }
    }
}
void StupidFind::ars_find_candinates(st_sf_path &p1, st_sf_path &p2, vector<int> &rs, vector<vector<int>> &cand)
{
    //cout << "22" << endl;
    vector<bool> v(MAX_NODES_NUM, true);
    //cout << "44" << endl;    
    //cout << p1.sides.size() << endl;
    //for (auto e: p1.sides) cout << e << endl;
    ars_del_sides(p1.sides);
    //cout << "3" << endl;
    ars_del_sides(rs);
    //cout << "4" << endl;

    p2.set_valid(v, *this);
    cand.resize(rs.size());
    for (size_t i = 0; i < rs.size(); i++){
        int s = edges[rs[i]].src;
        int d = edges[rs[i]].dst;
        v[s] = v[d] = true;
    //cout << "side: " << rs[i] << "->";
        cand[i] = ars_find_del_nodes(v, s, d);    // s-->d,1
        if (cand[i].empty()){
            cand[i] = vector<int>(1, rs[i]);     // can't find
        }
                        //for(auto e:cand[i]) cout << e << " "; cout << endl;
    }
}
vector<int> 
     StupidFind::ars_find_del_nodes(vector<bool> &v, int src, int dst)
{
    st_lbd_path lp;
    //cout << "HERE7:" << src << "," << dst << ".";
    lbd_dij_two_nodes(v, src, dst, lp, 0);
    //cout << "HERE8" << endl;
    st_sf_path sp(lp, *this);
    sp.set_valid(v, *this);

    return sp.sides;
}
void StupidFind::ars_del_sides(vector<int> &sides)
{
    for (auto e: sides) del_edge(e);
}
void StupidFind::ars_add_sides(vector<int> &sides)
{
    for (auto e: sides) add_edge(e);
}
void StupidFind::find_priority_pairs(vector<st_sf_path> &ms, vector<st_sf_path> &bs, vector<st_sf_pair> &bp)
{
    if (bp.empty()) return;

    // delete high cost
    int base_cost = bp[0].cost;
    bp.erase(remove_if(bp.begin(), bp.end(), [base_cost](st_sf_pair &p){return base_cost < p.cost;}), bp.end());
    if (bp.size() > SOLUS_NUM) bp.erase(bp.begin() + SOLUS_NUM, bp.end());
    // delete pairs rsn == 0 except bp[0]
    size_t last = 0;
    for (size_t i = 1; i < bp.size(); i++){
        if (bp[last].rsn != bp[i].rsn){
            last = i; break;
        }
    }bp.erase(bp.begin()+1, bp.begin()+last);
}
void StupidFind::make_main_back_pairs(vector<st_sf_path> &ms, vector<st_sf_path> &bs, vector<st_sf_pair> &bp)
{
    for (size_t i = 0; i < ms.size(); i++){
        for (size_t j = 0; j < bs.size(); j++){
            int cost = ms[i].cost + bs[j].cost;
            int rsn = ms[i].repeat_sides_num(bs[j]);
            bp.push_back(st_sf_pair(i, j, rsn, cost));
        }
    }sort(bp.begin(), bp.end());
}
bool StupidFind::get_main_back_paths(vector<st_sf_path> &ms, vector<st_sf_path> &bs, size_t L)
{
    // find paths of V1 V2
    vector<st_lbd_path> main;
    vector<st_lbd_path> back;
    lbd_run(main, _MAIN_PATH, L, false);  // s->d of V1
    show_time("Path 1 end...");
    lbd_run(back, _BACK_PATH, L, true); // d->s of V2
    show_time("Path 2 end...");
    // transform path format
    for (size_t i = 0; i < main.size(); i++)
        ms.push_back(st_sf_path(main[i], *this));
    for (size_t i = 0; i < back.size(); i++)
        bs.push_back(st_sf_path(back[i], *this));
    
    return ms.size() == 0 || bs.size() == 0;
}

void StupidFind::sf_show_paths(vector<st_sf_path> &ps)
{
    for (size_t i = 0; i < ps.size(); i++){
        cout << i << ": " << ps[i].cost << ", ";
        for (auto e:ps[i].sides)
            cout << e << "|";
        cout << endl;
    }
}
void StupidFind::sf_show_pairs(vector<st_sf_pair> &ps)
{
    for (auto &e: ps){
        cout << "[" << e.first << "," << e.second << "]"
             << "(" << e.rsn << "," << e.cost << ")" << "\t";
    }cout << endl;
}
void StupidFind::sf_show_solu(vector<st_sf_solu> &s)
{
    for (size_t i = 0; i < s.size(); i++){
        cout << "solu[" << i << "]: ";
        sf_show_solu(s[i]);
    }
}
void StupidFind::sf_show_solu(st_sf_solu &s)
{
    sf_show_solu(s.first, s.second);
}
void StupidFind::sf_show_solu(st_sf_path &m, st_sf_path &b)
{
    cout << "RSN: " << m.repeat_sides_num(b) 
         << ", Cost: " << m.cost + b.cost << endl;

    cout << "Main: " << m.cost << ", ";
    for (auto e: m.sides)
        cout << e << "|";
    cout << endl << "Back: " << b.cost << ", ";
    for (auto e: b.sides)
        cout << e << "|";
    cout << endl;
}

void StupidFind::check_legal_path(st_sf_path &path)
{
    cout << "check_legal_path" << endl;
    vector<int> node;
    for (auto e: path.sides){
        int s = edges[e].src;
        //int d = edges[e].dst;
        //cout << s << "->" << d << ",";
        //cout << s << " ";
        node.push_back(s);
        //node.push_back(d);
    }node.push_back(edges[path.sides.back()].dst);
    cout << endl << "Side Num " << path.sides.size() << endl;
    sort(node.begin(), node.end());
    for (auto e:node) cout << e << ","; cout << endl;
    //auto len = node.size();
    //node.erase(unique(node.begin(), node.end()), node.end());
    //cout << len - node.size() << " repeat nodes" << endl;
}
