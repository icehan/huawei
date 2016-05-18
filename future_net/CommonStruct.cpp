
#include "CommonStruct.h"
#include "lib_record.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <queue>
#include <vector>
#include <sstream>
#include <iostream>
using namespace std;

//--------------- CommonStruct -------------------------------------------------------------
CommonStruct::CommonStruct(char *topo[MAX_EDGE_NUM], int edge_num, char *demand[MAX_DEMAND_NUM],int demand_num)
{
    start_time = clock(); 
    last_time = start_time;

	demand_path_num = demand_num;
	total_edges_num = edge_num;
    norm_direct = true;

	memset(demand_nodes, -1, sizeof(int)*MAX_DEMAND_NUM*MAX_DEMANDS_NUM);
	get_edges_from_topo_str(topo, edge_num);
	get_demands_from_strs(demand, demand_num);
    if (simplify()) {
        cout << "No solution! -- delete demand nodes" << endl; exit(1);
    }
    update_para1();
	make_index();
    update_para2();    
    if (!is_connected_graph()){
        cout << "No solution! -- sub-graph not include all demand nodes" << endl; exit(1);
    }
    is_density_graph = set_density_graph(); //cout << (is_density_graph ? "density" : "sparse") << endl;; 
    init_list_width = set_init_list();      //cout << "Linit" << init_list_width << endl;
}
size_t CommonStruct::set_init_list()
{
    return is_density_graph ? INIT_LIST_WIDTH : 256;
}
bool CommonStruct::set_density_graph()
{
    size_t total_out_degree = 0;
    size_t total_nodes_num = 0;
    for (int i = 0; i <= max_label_node; i++){
        if (!nodes[i].valid) continue;
        total_out_degree += nodes[i].edge_index_out.size();
        total_nodes_num++;
    }
    double average_out_degree = 1.0*total_out_degree/total_nodes_num;
    return average_out_degree > 6.8;
}

void CommonStruct::inverse_direct()
{
    norm_direct = !norm_direct;
    // inverse source & destin
    swap(source_node, destin_node);
    for (size_t i = 0; i < MAX_EDGE_NUM; i++)
        swap(edges[i].src, edges[i].dst);
    // make index
    make_index();
}
bool CommonStruct::is_any_demand(int node)
{
    for (int i = 0; i < demand_path_num; i++)
        if (is_demand(i, node))
            return true;
    return false;
}
bool CommonStruct::is_demand(int path_id, int node)
{
	return is_demand_node[path_id][node];
}
bool CommonStruct::will_timeout()
{
    cur_time = clock();
	return time_diff_ms(start_time, cur_time) >= 
           (TOTAOL_SEC*1000 - TIME_FOR_SAVING);
}
void CommonStruct::show_time(const char *s)
{
    if (s) cout << s;

    cur_time = clock(); unsigned long t;
    t = time_diff_ms(last_time, cur_time);
    cout << " [" << t/1000 << "]s [" << t%1000 << "]ms. ";
    t = time_diff_ms(start_time, cur_time);
    cout << "Total Time [" << t/1000 << "]s [" << t%1000 << "]ms." << endl;  

    last_time = cur_time;
}
bool CommonStruct::is_connected_graph()
{// return true if all src/dst/demand nodes are connected
	// get all nodes arrived from source
    bool visited[MAX_NODES_NUM] = {0};
	vector<int> all_conn_nodes;
	queue<int> qtmp;
	qtmp.push(source_node);
    visited[source_node] = true;
	while(!qtmp.empty()){
		int head = qtmp.front(); qtmp.pop();
		all_conn_nodes.push_back(head);
		int hod = get_degree_out(head);
		for (int i = 0; i < hod; i++){
			int child = get_child(head, i);
            if (!visited[child]){
            	qtmp.push(child);
                visited[child] = true;
            }
        }
    }
	// check if src/dst and all demands included
	int connected_num = 1; // source_node
	for (size_t i = 1; i < all_conn_nodes.size(); i++){ 
		if (destin_node == all_conn_nodes[i])
			connected_num++;
    }

    for (size_t i = 1; i < all_conn_nodes.size(); i++)
        if (is_any_demand(all_conn_nodes[i]))
	        connected_num++;
	return connected_num == demand_nodes_num[0]+demand_nodes_num[1]+2;
}
// --------------------- interface -----------------------
int CommonStruct::get_weight_out(int node, int index)
{
	return edges[nodes[node].edge_index_out[index]].weight;
}
int CommonStruct::get_weight_in(int node, int index)
{
	return edges[nodes[node].edge_index_in[index]].weight;
}
int  CommonStruct::get_edge_out(int node, int index)
{
    return nodes[node].edge_index_out[index];
}
int  CommonStruct::get_edge_in(int node, int index)
{
    return nodes[node].edge_index_in[index];
}
int CommonStruct::get_degree_out(int node)
{
	return nodes[node].edge_index_out.size();
}
int CommonStruct::get_degree_in(int node)
{
	return nodes[node].edge_index_in.size();
}
int CommonStruct::get_child(int node, int index)
{
	return edges[nodes[node].edge_index_out[index]].dst;
}
int CommonStruct::get_parent(int node, int index)
{
	return edges[nodes[node].edge_index_in[index]].src;
}
int CommonStruct::get_edge(int node_s, int node_e)
{
    int deg_out = get_degree_out(node_s);
    for (int i = 0; i < deg_out; i++)
        if (get_child(node_s, i) == node_e)
            return get_edge_out(node_s, i);
    return -1;
}
int CommonStruct::get_weight(int node_s, int node_e)
{
    return edges[get_edge(node_s, node_e)].weight;
}
// --------------------- interface -----------------------
void CommonStruct::del_edge(int edge)
{
    int index = -1;
    auto &vo = get_index(edge, true, index);
    if (-1 != index) vo.erase(vo.begin() + index);

    auto &vi = get_index(edge, false, index);
    if (-1 != index) vi.erase(vi.begin() + index);
}
void CommonStruct::add_edge(int edge)
{
    int index = -1;
    auto &vo = get_index(edge, true, index);
    if (-1 == index) vo.push_back(edge);

    auto &vi = get_index(edge, false, index);
    if (-1 == index) vi.push_back(edge);
}

vector<int>& CommonStruct::get_index(int edge, bool as_out, int &index)
{
    int  n  = as_out ? edges[edge].src : edges[edge].dst;
    auto &v = as_out ? nodes[n].edge_index_out : nodes[n].edge_index_in;
    for (size_t i = 0; i < v.size(); i++){
        if (edge == v[i]) {index = i; break;}
    }return v;
}
// --------------------- make index -----------------------
void CommonStruct::update_para2()
{
    total_nodes_num = 0;
    for (int i = 0; i <= max_label_node; i++)
        if (nodes[i].valid) total_nodes_num++;
}
void CommonStruct::make_index()
{
    // clear
	for (int i = 0; i <= max_label_node; i++){
        nodes[i].edge_index_out.clear();
        nodes[i].edge_index_in.clear();
        nodes[i].valid = false;
    }
    
    // make
	for (int i = 0; i <= max_label_edge; i++)
	{
		if (!edges[i].valid) continue;
		int src = edges[i].src;
		nodes[src].valid = true;
		nodes[src].edge_index_out.push_back(i);
		int dst = edges[i].dst;
		nodes[dst].valid = true;
		nodes[dst].edge_index_in.push_back(i);
	}
}

// --------------------- simplify -------------------------
void  CommonStruct::update_para1()
{
    // 3 para 
	total_edges_num = 0;
	max_label_node = 0;
	max_label_edge = 0;
	for (int i = 0; i < MAX_EDGE_NUM; i++){
	    if (edges[i].valid){
		    total_edges_num++;
			int s = edges[i].src, d = edges[i].dst, l = edges[i].label;
			if (max_label_node < s) max_label_node = s;
			if (max_label_node < d) max_label_node = d;
			if (max_label_edge < l) max_label_edge = l;
		}
	}
}
bool CommonStruct::simplify()
{// return true when deleting demand nodes
    #ifdef DEBUG_DELETE
    cout << "INIT GRAPH" << endl; show();
    #endif

	simplify_src_dst_sides();
    #ifdef DEBUG_DELETE
    cout << "AFTER SIMP SRC&DST" << endl; show();
	#endif

	bool if_dd = 
	simplify_redundant_nodes();
    #ifdef DEBUG_DELETE
    cout << "AFTER DEL NODES, END GRAPH" << endl; show();
	#endif

	return if_dd;
}
bool CommonStruct::simplify_redundant_nodes()
{
    int dn = 0; //delete num
    bool d[MAX_NODES_NUM] = {0};
	while(true){
    	int do_[MAX_NODES_NUM] = {0};
	    int di_[MAX_NODES_NUM] = {0};
		// calculate degree out/in of each node
        get_degree_of_each_node(do_, di_);
		// label whether deleted, keep source/destin node alive
		dn = find_nodes_tobe_deleted(d, do_, di_);
        if (dn == 0) break;
		// delete nodes & its sides
        simplify_edges_with_dead_nodes(d);
	}return if_del_demands(d);
}
bool CommonStruct::if_del_demands(bool d[])
{
    for (size_t i = 0; i < MAX_NODES_NUM; i++)
	{
		bool f = false;
		for (int j = 0; j < demand_path_num; j++)
			f |= is_demand(j, i);
        if (d[i] && f)
            return true;
	}
    return false;
}
void CommonStruct::simplify_edges_with_dead_nodes(bool d[])
{
    for (int i = 0; i <= max_label_edge; i++){
        if (edges[i].valid){
            if (d[edges[i].src] || d[edges[i].dst]){
                edges[i].valid = false; }}}
    #ifdef DEBUG_DELETE
    cout << "after simplify_edges_with_dead_nodes()" << endl;
	for (int i = 0; i <= max_label_edge; i++)
        if (edges[i].valid)  printf("%d\t%d\t%d\t%d\t|%d\n",edges[i].label, edges[i].src, edges[i].dst, edges[i].weight, edges[i].valid);
    #endif
}
int CommonStruct::find_nodes_tobe_deleted(bool d[], int do_[], int di_[])
{
    int deleted_num = 0;
    for (int i = 0; i < MAX_NODES_NUM; i++){
        if (i != source_node &&      // not src
            i != destin_node &&      // not dst
            0 != do_[i] + di_[i]  && // valid node
            (do_[i] == 0 || di_[i] == 0)) // source or hole
        {
            d[i] = true;
            deleted_num++; 
        }
    }
    d[source_node] = d[destin_node] = false;
    #ifdef DEBUG_DELETE
    cout << "after find_nodes_tobe_deleted()" << endl;
    cout << "delete " << deleted_num << "nodes" << endl;
    for (int i = 0; i < 10; i++)
        cout << d[i] << " ";
    cout << endl;
    #endif
    return deleted_num;
}
void CommonStruct::get_degree_of_each_node(int do_[], int di_[])
{
    for (int i = 0; i <= max_label_edge; i++){
        if (edges[i].valid){
            do_[edges[i].src]++;
            di_[edges[i].dst]++;
        }
    }
    #ifdef DEBUG_DELETE
    cout << "after find_nodes_tobe_deleted()" << endl;
    cout << "out deg / in deg" << endl;
    for (int i = 0; i <= max_label_edge; i++)
        cout << do_[i] << "/" << di_[i] << endl;
    #endif
}

void CommonStruct::simplify_repeat_sides()
{
	for (int i = 0; i < MAX_EDGE_NUM; i++){
		for (int j = i+1; j < MAX_EDGE_NUM; j++){
			if (is_the_same_valid_side(i, j)){
				if (edges[i].weight < edges[j].weight){
					edges[j].valid = false;
                }else{
					edges[i].valid = false;
                }
            }
		}
	}
}
bool CommonStruct::is_the_same_valid_side(int ind_a, int ind_b)
{
	return	edges[ind_a].valid &&
			edges[ind_b].valid &&
			edges[ind_a].src == edges[ind_b].src &&
			edges[ind_a].dst == edges[ind_b].dst;
}
void CommonStruct::simplify_src_dst_sides()
{
	for (int i = 0; i <= max_label_edge; i++){
		if (!edges[i].valid) continue;
		if (edges[i].src == source_node && 
			edges[i].dst == destin_node)    // edge from s to d directly
		    edges[i].valid = false;
		if (edges[i].dst == source_node)    // edge to   s
		    edges[i].valid = false;
		if (edges[i].src == destin_node)    // edge from d
		    edges[i].valid = false;
	}
}
//---------------Transform: str 2 int------------------------------------------------------
void CommonStruct::get_edges_from_topo_str(char* topo[MAX_EDGE_NUM], int edge_num)
{
    int l, s, d, w;
	max_label_node = max_label_edge = 0;
	for (int i = 0; i < edge_num; i++){
		sscanf(topo[i], "%d,%d,%d,%d\n", &l, &s, &d, &w);
		if (l < MAX_EDGE_NUM){
			edges[l] = {true, l, s, d, w};
			if (max_label_node < s) max_label_node = s;
			if (max_label_node < d) max_label_node = d;
			if (max_label_edge < l) max_label_edge = l;
		}else{
			printf("edge id [%d] wrong!\n", l);
			exit(1);
		}
	}
}
void CommonStruct::get_demands_from_strs(char* demand[MAX_DEMAND_NUM], int demand_num)
{
	for (int i = 0; i < demand_num; i++){
		string tmp_dem(demand[i]);
		// str-> int
		demand_nodes_num[i] = get_demands_from_str(tmp_dem, demand_nodes[i]);
		// int -> bool
		memset(is_demand_node[i], false, sizeof(bool)*MAX_NODES_NUM);
		for (int j = 0; j < demand_nodes_num[i]; j++){
			is_demand_node[i][demand_nodes[i][j]] = true;
		}
	}
}
int CommonStruct::get_demands_from_str(string &demand, int dn[MAX_DEMANDS_NUM])
{
	stringstream ss(demand);
	int id, dem; char t;
	ss >> id >> t >> source_node >> t >> destin_node;
	int dnn = 0;
	while (ss >> t >> dem){
		dn[dnn++] = dem;
	}
	return dnn;
}

void CommonStruct::show()
{
	printf("-------------------------------\n");
	printf("src: %d, dst: %d\n", source_node, destin_node);
	for (int i = 0; i < demand_path_num; i++){
		printf("demand_nodes[%d], %d: ", i, demand_nodes_num[i]);
    //		for (int j = 0; j < MAX_NODES_NUM; j++)
    //			if (is_demand(i, j))
    //				printf("%d ", j);
		for (int j = 0; j < demand_nodes_num[i]; j++)
			printf("%d\t", demand_nodes[i][j]);
		printf("\n");
	}
	printf("\ntotal edges num: %d\n", total_edges_num);
	printf("max label node: %d\nmax label edge: %d\n", max_label_node, max_label_edge);

	printf("edges:\n");
	for (int i = 0; i <= max_label_edge; i++){
        if (edges[i].valid) continue;
		printf("edges[%d]\t%d\t%d\t%d\t%d\t|%d\n", i, edges[i].label,
        edges[i].src, edges[i].dst, edges[i].weight, edges[i].valid);
    }
	for (int i = 0; i <= max_label_edge; i++){
        if (!edges[i].valid) continue;
		printf("edges[%d]\t%d\t%d\t%d\t%d\t|%d\n", i, edges[i].label,
        edges[i].src, edges[i].dst, edges[i].weight, edges[i].valid);
    }
	printf("\n-------------------------------\nnodes:\n");
    for (int i = 0; i <= max_label_node; i++){
        if (!nodes[i].valid) continue;
        printf("nodes[%d]\nparent: ", i);
        for (auto e: nodes[i].edge_index_in)
            cout << e << "\t";
        printf("\nchild: ");
        for (auto e: nodes[i].edge_index_out)
            cout << e << "\t";
        printf("\n\n");
    }
	printf("-------------------------------\n");
}
