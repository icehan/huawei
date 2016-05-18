
#ifndef __COMMONSTRUCT_H__
#define __COMMONSTRUCT_H__

#include "types.h"
#include "lib_io.h"	//MAX_EDGE_NUM, MAX_DEMAND_NUM
#include <time.h>
#include <vector>
using std::vector;
#include <string>
using std::string;

#define MAX_WEIGHT      101     //[1,100]
#define MAX_OUT_DEG		20
#define MAX_NODES_NUM	2000
#define MAX_DEMANDS_NUM	100

struct st_edge{
	bool valid;
	int  label;
	int  src;
	int  dst;
	int  weight;
};
struct st_node{
	bool        valid;
	vector<int> edge_index_in;
	vector<int> edge_index_out;
	st_node(bool v=false):valid(v){edge_index_in.clear();edge_index_out.clear();}
};

//------------------------- CommonStruct -------------------------------------
class CommonStruct 
{
public:
	st_edge edges[MAX_EDGE_NUM];     // initial data of edges in:wafo
	st_node nodes[MAX_NODES_NUM];    // nodes info of edges

	int source_node;					// source node
	int destin_node;					// destination node
	int demand_nodes[MAX_DEMAND_NUM][MAX_DEMANDS_NUM];	// demand nodes	

	int demand_path_num;
	int demand_nodes_num[MAX_DEMAND_NUM];				// number of demand nodes
	bool  is_demand_node[MAX_DEMAND_NUM][MAX_NODES_NUM];

    int total_nodes_num;
	int total_edges_num;			// number of total edges
	int max_label_node;				// max label of nodes
	int max_label_edge;				// max label of edges
    bool is_density_graph;
    size_t init_list_width;
protected:
    clock_t start_time;
    clock_t last_time;
    clock_t cur_time;              // for timeover control
private:
    bool norm_direct;  // s->d or d->s
    
public:
	CommonStruct(char *topo[MAX_EDGE_NUM], int edge_num, char *demand[MAX_DEMAND_NUM], int demand_num);
	void show();
    bool will_timeout();
    void show_time(const char *s=NULL);
    unsigned long time_diff_ms(clock_t &s, clock_t &t){return (t-s)/1000;}

    void inverse_direct();
    bool is_any_demand(int node);
	bool is_demand(int path_id, int node);
    
	int get_weight_out    (int node, int index);
	int get_weight_in     (int node, int index);
	int get_edge_out      (int node, int index);
	int get_edge_in       (int node, int index);
	int get_degree_out    (int node);
	int get_degree_in     (int node);
	int get_parent        (int node, int index);
	int get_child         (int node, int index);
    int get_weight        (int node_s, int node_e);
    int get_edge          (int node_s, int node_e);
    void del_edge         (int edge);
    void add_edge         (int edge);
protected:
    bool set_density_graph();
    size_t set_init_list();
private:
    vector<int>& get_index(int edge, bool as_out, int &index);
	bool is_connected_graph();	// true if path from s to any of dst/demand nodes exists.
    void update_para2();
	void make_index();			// make indices of src-dsts and dst-srcs.
    void update_para1();
	bool simplify();			// delete redundancy edges in edges[], only be labelled.
	bool simplify_redundant_nodes();
    bool if_del_demands(bool d[]);
    void simplify_edges_with_dead_nodes(bool d[]);
    int  find_nodes_tobe_deleted(bool d[], int do_[], int di_[]);
    void get_degree_of_each_node(int do_[], int di_[]);
    void simplify_repeat_sides();
    bool is_the_same_valid_side(int ind_a, int ind_b);
	void simplify_src_dst_sides();
	
	//--------------- Transform: str 2 int ------------------------------------------------------
	void get_edges_from_topo_str(char* topo[MAX_EDGE_NUM], int edge_num);
	void get_demands_from_strs(char* demand[MAX_DEMAND_NUM], int demand_num);
	int  get_demands_from_str(string &demand, int dn[MAX_DEMANDS_NUM]);
};

#endif
